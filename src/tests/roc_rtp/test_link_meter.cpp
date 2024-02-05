/*
 * Copyright (c) 2024 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <CppUTest/TestHarness.h>

#include "roc_audio/channel_defs.h"
#include "roc_audio/sample_spec.h"
#include "roc_core/fast_random.h"
#include "roc_core/heap_arena.h"
#include "roc_packet/packet_factory.h"
#include "roc_packet/queue.h"
#include "roc_rtp/encoding_map.h"
#include "roc_rtp/headers.h"
#include "roc_rtp/link_meter.h"

namespace roc {
namespace rtp {

namespace {

enum { PacketSz = 100 };

core::HeapArena arena;
packet::PacketFactory packet_factory(arena, PacketSz);

EncodingMap encoding_map(arena);

enum { ChMask = 3, PacketSz = 128, SampleRate = 10000, Duration = 100, RunningWinLen = Duration};
audio::SampleSpec sample_spec(SampleRate, audio::Sample_RawFormat,
                              audio::ChanLayout_Surround, audio::ChanOrder_Smpte, ChMask);
const core::nanoseconds_t start_ts = 1691499037871419405;
const core::nanoseconds_t step_ts = Duration * core::Second / SampleRate;

packet::PacketPtr new_packet(packet::seqnum_t sn, const core::nanoseconds_t ts) {
    packet::PacketPtr packet = packet_factory.new_packet();
    CHECK(packet);

    packet->add_flags(packet::Packet::FlagRTP | packet::Packet::FlagUDP);
    packet->rtp()->payload_type = PayloadType_L16_Stereo;
    packet->rtp()->seqnum = sn;
    packet->rtp()->duration = Duration;
    packet->udp()->enqueue_ts = ts;

    return packet;
}

class StatusWriter : public packet::IWriter {
public:
    explicit StatusWriter(status::StatusCode code)
        : code_(code) {
    }

    virtual ROC_ATTR_NODISCARD status::StatusCode write(const packet::PacketPtr&) {
        return code_;
    }

private:
    status::StatusCode code_;
};

} // namespace

TEST_GROUP(link_meter) { };

TEST(link_meter, has_metrics) {
    packet::Queue queue;
    LinkMeter meter(arena, sample_spec, RunningWinLen);
    meter.set_writer(queue);

    CHECK(!meter.has_metrics());

    LONGS_EQUAL(status::StatusOK, meter.write(new_packet(100, start_ts)));
    UNSIGNED_LONGS_EQUAL(1, queue.size());

    CHECK(!meter.has_metrics());
}

TEST(link_meter, last_seqnum) {
    packet::Queue queue;
    LinkMeter meter(arena, sample_spec, RunningWinLen);
    meter.set_writer(queue);
    core::nanoseconds_t ts = start_ts;

    UNSIGNED_LONGS_EQUAL(0, meter.metrics().ext_last_seqnum);

    LONGS_EQUAL(status::StatusOK, meter.write(new_packet(100, ts)));
    UNSIGNED_LONGS_EQUAL(100, meter.metrics().ext_last_seqnum);
    ts += step_ts;

    // seqnum increased, metric updated
    LONGS_EQUAL(status::StatusOK, meter.write(new_packet(102, ts + step_ts)));
    UNSIGNED_LONGS_EQUAL(102, meter.metrics().ext_last_seqnum);

    // seqnum decreased, ignored
    LONGS_EQUAL(status::StatusOK, meter.write(new_packet(101, ts)));
    UNSIGNED_LONGS_EQUAL(102, meter.metrics().ext_last_seqnum);
    ts += step_ts * 2;

    // seqnum increased, metric updated
    CHECK_EQUAL(status::StatusOK, meter.write(new_packet(103, ts)));

    CHECK_EQUAL(0, meter.mean_jitter());
    CHECK_EQUAL(0, meter.var_jitter());

    LinkMetrics metrics = meter.metrics();
    CHECK_EQUAL(0, metrics.jitter);
    CHECK_EQUAL(0, metrics.cum_loss);
    UNSIGNED_LONGS_EQUAL(103, metrics.ext_last_seqnum);
    CHECK_EQUAL(0, metrics.num_packets_covered);


    UNSIGNED_LONGS_EQUAL(4, queue.size());
}

TEST(link_meter, last_seqnum_wrap) {
    packet::Queue queue;
    LinkMeter meter(arena, sample_spec, RunningWinLen);
    meter.set_writer(queue);
    core::nanoseconds_t ts = start_ts;

    UNSIGNED_LONGS_EQUAL(0, meter.metrics().ext_last_seqnum);

    // no overflow
    LONGS_EQUAL(status::StatusOK, meter.write(new_packet(65533, ts)));
    UNSIGNED_LONGS_EQUAL(65533, meter.metrics().ext_last_seqnum);

    // no overflow
    LONGS_EQUAL(status::StatusOK, meter.write(new_packet(65535, ts + step_ts * 2)));
    UNSIGNED_LONGS_EQUAL(65535, meter.metrics().ext_last_seqnum);

    // overflow
    LONGS_EQUAL(status::StatusOK, meter.write(new_packet(2, ts + step_ts * 3)));
    UNSIGNED_LONGS_EQUAL(65537, meter.metrics().ext_last_seqnum);

    // late packet, ignored
    LONGS_EQUAL(status::StatusOK, meter.write(new_packet(65534, ts + step_ts)));
    UNSIGNED_LONGS_EQUAL(65537, meter.metrics().ext_last_seqnum);

    // new packet
    LONGS_EQUAL(status::StatusOK, meter.write(new_packet(5, ts + step_ts * 6)));
    UNSIGNED_LONGS_EQUAL(65540, meter.metrics().ext_last_seqnum);

    UNSIGNED_LONGS_EQUAL(5, queue.size());
}

TEST(link_meter, forward_error) {
    StatusWriter writer(status::StatusNoMem);
    LinkMeter meter(arena, sample_spec, RunningWinLen);
    meter.set_writer(writer);

    CHECK_EQUAL(status::StatusNoMem, meter.write(new_packet(100, start_ts)));
}

TEST(link_meter, jitter_test) {
    packet::Queue queue;
    LinkMeter meter(arena, sample_spec, RunningWinLen);
    meter.set_writer(queue);
    const size_t num_packets = Duration * 100;
    core::nanoseconds_t ts_store[num_packets];

    core::nanoseconds_t ts = start_ts;
    for (size_t i = 0; i < num_packets; i++) {
        packet::seqnum_t seqnum = 65500 + i;
        ts_store[i] = ts;
        CHECK_EQUAL(status::StatusOK, meter.write(new_packet(seqnum, ts)));
        ts += step_ts + (core::nanoseconds_t)(core::fast_random_gaussian() * core::Millisecond);

        if (i > RunningWinLen) {
            // Check meter metrics running max in min jitter in last Duration number
            // of packets in ts_store.
            core::nanoseconds_t min_jitter = core::Second;
            core::nanoseconds_t max_jitter = 0;
            for (size_t j = 0; j < RunningWinLen; j++) {
                core::nanoseconds_t jitter = std::abs(ts_store[i - j] - ts_store[i - j - 1] - step_ts);
                min_jitter = std::min(min_jitter, jitter);
                max_jitter = std::max(max_jitter, jitter);
            }
            CHECK_EQUAL(min_jitter, meter.metrics().min_jitter);
            CHECK_EQUAL(max_jitter, meter.metrics().max_jitter);

            // Reference average  and variance of jitter from ts_store values.
            core::nanoseconds_t sum = 0;
            for (size_t j = 0; j < RunningWinLen; j++) {
                sum += std::abs(ts_store[i - j] - ts_store[ i - j - 1] - step_ts);
            }
            const core::nanoseconds_t mean = sum / RunningWinLen;

            sum = 0;
            for (size_t j = 0; j < RunningWinLen; j++) {
                core::nanoseconds_t jitter = std::abs(ts_store[i - j] - ts_store[i - j - 1] - step_ts);
                sum += (jitter - mean) * (jitter - mean);
            }
            const core::nanoseconds_t var = sum / RunningWinLen;

            // Check the jitter value
            DOUBLES_EQUAL(mean, meter.mean_jitter(), core::Microsecond * 1);
            DOUBLES_EQUAL(sqrt(var), meter.var_jitter(), core::Microsecond * 1);
        }
    }

}

TEST(link_meter, ascending_test) {
    packet::Queue queue;
    LinkMeter meter(arena, sample_spec, RunningWinLen);
    meter.set_writer(queue);
    const size_t num_packets = Duration * 100;
    core::nanoseconds_t ts_store[num_packets];

    core::nanoseconds_t ts = start_ts;
    for (size_t i = 0; i < num_packets; i++) {
        packet::seqnum_t seqnum = 65500 + i;
        ts_store[i] = ts;
        CHECK_EQUAL(status::StatusOK, meter.write(new_packet(seqnum, ts)));
        ts += step_ts + (core::nanoseconds_t)i * core::Microsecond; // Removed the random component to create an increasing sequence

        if (i > RunningWinLen) {
            // Check meter metrics running max in min jitter in last Duration number
            // of packets in ts_store.
            core::nanoseconds_t min_jitter = core::Second;
            core::nanoseconds_t max_jitter = 0;
            for (size_t j = 0; j < RunningWinLen; j++) {
                core::nanoseconds_t jitter = std::abs(ts_store[i - j] - ts_store[i - j - 1] - step_ts);
                min_jitter = std::min(min_jitter, jitter);
                max_jitter = std::max(max_jitter, jitter);
            }
            CHECK_EQUAL(min_jitter, meter.metrics().min_jitter);
            CHECK_EQUAL(max_jitter, meter.metrics().max_jitter);
        }
    }
}

TEST(link_meter, descending_test) {
    packet::Queue queue;
    LinkMeter meter(arena, sample_spec, RunningWinLen);
    meter.set_writer(queue);
    const size_t num_packets = Duration * 100;
    core::nanoseconds_t ts_store[num_packets];

    core::nanoseconds_t ts = start_ts;
    for (size_t i = 0; i < num_packets; i++) {
        packet::seqnum_t seqnum = 65500 + i;
        ts_store[i] = ts;
        CHECK_EQUAL(status::StatusOK, meter.write(new_packet(seqnum, ts)));
        ts += step_ts - (core::nanoseconds_t)i * core::Nanosecond * 10; // Removed the random component to create an increasing sequence

        if (i > RunningWinLen) {
            // Check meter metrics running max in min jitter in last Duration number
            // of packets in ts_store.
            core::nanoseconds_t min_jitter = core::Second;
            core::nanoseconds_t max_jitter = 0;
            for (size_t j = 0; j < RunningWinLen; j++) {
                core::nanoseconds_t jitter = std::abs(ts_store[i - j] - ts_store[i - j - 1] - step_ts);
                min_jitter = std::min(min_jitter, jitter);
                max_jitter = std::max(max_jitter, jitter);
            }
            CHECK_EQUAL(min_jitter, meter.metrics().min_jitter);
            CHECK_EQUAL(max_jitter, meter.metrics().max_jitter);
        }
    }
}

TEST(link_meter, saw_test) {
    packet::Queue queue;
    LinkMeter meter(arena, sample_spec, RunningWinLen);
    meter.set_writer(queue);
    const size_t num_packets = Duration * 100;
    core::nanoseconds_t ts_store[num_packets];
    core::nanoseconds_t step_ts_inc = core::Nanosecond * 10;
    core::nanoseconds_t step_ts_ = step_ts;

    core::nanoseconds_t ts = start_ts;
    for (size_t i = 0; i < num_packets; i++) {
        packet::seqnum_t seqnum = 65500 + i;
        ts_store[i] = ts;
        CHECK_EQUAL(status::StatusOK, meter.write(new_packet(seqnum, ts)));
        ts += step_ts_;
        step_ts_ += step_ts_inc;
        if (i > 0 && i % RunningWinLen == 0) {
            step_ts_inc = -step_ts_inc;
        }

        if (i > RunningWinLen) {
            // Check meter metrics running max in min jitter in last Duration number
            // of packets in ts_store.
            core::nanoseconds_t min_jitter = core::Second;
            core::nanoseconds_t max_jitter = 0;
            for (size_t j = 0; j < RunningWinLen; j++) {
                core::nanoseconds_t jitter = std::abs(ts_store[i - j] - ts_store[i - j - 1] - step_ts);
                min_jitter = std::min(min_jitter, jitter);
                max_jitter = std::max(max_jitter, jitter);
            }
            CHECK_EQUAL(min_jitter, meter.metrics().min_jitter);
            CHECK_EQUAL(max_jitter, meter.metrics().max_jitter);
        }
    }
}

TEST(link_meter, losses_test) {
    packet::Queue queue;
    LinkMeter meter(arena, sample_spec, RunningWinLen);
    meter.set_reader(queue);
    const size_t num_packets = Duration * 100;
    size_t total_losses = 0;
    size_t fract_losses_cntr = 0;
    size_t n_packets_metricated = 0;

    core::nanoseconds_t ts = start_ts;
    for (size_t i = 0; i < num_packets; i++) {
        packet::seqnum_t seqnum = 65500 + i;
        packet::PacketPtr p = new_packet(seqnum, ts);
        ts += step_ts;
        n_packets_metricated++;

        if (i > 0 && core::fast_random_range(0, 100) < 30) {
            total_losses++;
            fract_losses_cntr++;
            continue;
        } else {
            CHECK_EQUAL(status::StatusOK, queue.write(p));
        }

        packet::PacketPtr pr;
        CHECK_EQUAL(status::StatusOK, meter.read(pr));
        CHECK_EQUAL(pr->rtp()->seqnum, p->rtp()->seqnum);

        if (i > 0) {
            LinkMetrics metrics = meter.metrics();
            DOUBLES_EQUAL((float)fract_losses_cntr / (float)n_packets_metricated,
                        metrics.fract_loss, 1e-4);
            CHECK_EQUAL(total_losses, metrics.cum_loss);
            CHECK_EQUAL(n_packets_metricated, metrics.num_packets_covered);
        }

        // randomly reset metrics
        if (i > 0 && core::fast_random_range(0, 100) < 10) {
            meter.reset_metrics();
            fract_losses_cntr = 0;
            n_packets_metricated = 0;
        }
    }
}

} // namespace rtp
} // namespace roc
