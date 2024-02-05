/*
 * Copyright (c) 2024 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "roc_core/panic.h"
#include "roc_packet/units.h"
#include "link_meter.h"

namespace roc {
namespace rtp {

LinkMeter::LinkMeter(core::IArena& arena,
                    const EncodingMap& encoding_map
                     const audio::SampleSpec& sample_spec,
                     size_t run_win_len)
    : encoding_map_(encoding_map)
    , encoding_(NULL)
    , writer_(NULL)
    , reader_(NULL)
    , sample_spec_(sample_spec)
    , first_packet_jitter_(true)
    , first_packet_losses_(true)
    , has_metrics_(false)
    , first_seqnum_(0)
    , last_seqnum_hi_(0)
    , last_seqnum_lo_(0)
    , win_len_(run_win_len)
    , seqnum_prev_loss_(0)
    , lost_(0)
    , fract_lost_counter_(0)
    , jitter_processed_(0)
    , prev_packet_enq_ts_(-1)
    , packet_jitter_stats_(arena, run_win_len) {
}

bool LinkMeter::has_metrics() const {
    return has_metrics_;
}

const packet::LinkMetrics& LinkMeter::metrics() const {
    return metrics_;
}

bool LinkMeter::has_encoding() const {
    return encoding_ != NULL;
}

const Encoding& LinkMeter::encoding() const {
    if (encoding_ == NULL) {
        roc_panic("link meter: encoding not available");
    }

    return *encoding_;
}

void LinkMeter::process_report(const rtcp::SendReport& report) {
    // Currently LinkMeter calculates all link metrics except RTT, and
    // RTT is calculated by RTCP module and passed here.
    metrics_.rtt = report.rtt;

}

status::StatusCode LinkMeter::write(const packet::PacketPtr& packet) {
    if (!writer_) {
        roc_panic("link meter: forgot to call set_writer()");
    }

    if (!packet) {
        roc_panic("link meter: null packet");
    }

    // When we create LinkMeter, we don't know yet if RTP is used (e.g.
    // for repair packets), so we should be ready for non-rtp packets.
    if (packet->rtp()) {
        // Since we don't know packet type in-before, we also determine
        // encoding dynamically.
        if (!encoding_ || encoding_->payload_type != packet->rtp()->payload_type) {
            encoding_ = encoding_map_.find_by_pt(packet->rtp()->payload_type);
        }
        if (encoding_) {
            update_metrics_(*packet);
        }
    }

    return writer_->write(packet);
}

status::StatusCode LinkMeter::read(packet::PacketPtr& packet) {
    if (!reader_) {
        roc_panic("link meter: forgot to call set_reader()");
    }

    const status::StatusCode code = reader_->read(packet);
    if (code != status::StatusOK) {
        return code;
    }

    if (packet->rtp()) {
        update_losses_(*packet);
    } else {
        roc_panic("link meter: non-rtp packet on reader input");
    }

    return status::StatusOK;
}

void LinkMeter::set_writer(packet::IWriter& writer) {
    writer_ = &writer;
}

void LinkMeter::set_reader(packet::IReader& reader) {
    reader_ = &reader;
}

bool LinkMeter::has_metrics() const {
    return has_metrics_;
}

LinkMetrics LinkMeter::metrics() const {
    return metrics_;
}

void LinkMeter::update_jitter_(const packet::Packet& packet) {
    const packet::seqnum_t pkt_seqnum = packet.rtp()->seqnum;

    // If packet seqnum is before first seqnum, and there was no wrap yet,
    // update first seqnum.
    if ((first_packet_jitter_
        || packet::seqnum_diff(pkt_seqnum, first_seqnum_) < 0)
        && last_seqnum_hi_ == 0) {
        first_seqnum_ = pkt_seqnum;
    }

    // If packet seqnum is after last seqnum, update last seqnum, and
    // also counts possible wraps.
    if (first_packet_jitter_ 
        || packet::seqnum_diff(pkt_seqnum, last_seqnum_lo_) > 0) {
        if (pkt_seqnum < last_seqnum_lo_) {
            last_seqnum_hi_ += (uint16_t)-1;
        }

        if (!first_packet_jitter_) {
            const size_t gap =  (size_t)abs(
                packet::seqnum_diff(packet.rtp()->seqnum, last_seqnum_lo_ + 1));
            // Compute jitter only on consequential packets.
            if (gap == 0 && prev_pack_duration_ > 0){
                const core::nanoseconds_t  d_enq_ts = packet.udp()->enqueue_ts -
                                                        prev_packet_enq_ts_;
                const core::nanoseconds_t d_capt_ts = sample_spec_.samples_per_chan_2_ns(prev_pack_duration_);
                packet_jitter_stats_.add(std::abs(d_enq_ts - d_capt_ts));
                metrics_.max_jitter = packet_jitter_stats_.mov_max();
                metrics_.min_jitter = packet_jitter_stats_.mov_min();
                jitter_processed_++;
                metrics_.jitter = sample_spec_.ns_2_samples_per_chan(mean_jitter());
            }
        }

        prev_packet_enq_ts_ = packet.udp()->enqueue_ts;
        prev_pack_duration_ = packet.rtp()->duration;
        last_seqnum_lo_ = pkt_seqnum;
        prev_stream_timestamp = packet.rtp()->stream_timestamp;
    }

    metrics_.ext_first_seqnum = first_seqnum_;
    metrics_.ext_last_seqnum = last_seqnum_hi_ + last_seqnum_lo_;

    // TODO(gh-688):
    //  - fill total_packets
    //  - fill lost_packets
    //  - fill jitter (use encoding_->sample_spec to convert to nanoseconds)

    first_packet_ = false;
    has_metrics_ = true;
}

void LinkMeter::update_losses_(const packet::Packet& packet) {

    if (first_packet_losses_) {
        first_packet_losses_ = false;
        metrics_.num_packets_covered = 1;
        seqnum_prev_loss_ = packet.rtp()->seqnum;
        return;
    }
    const ssize_t gap = packet::seqnum_diff(packet.rtp()->seqnum, seqnum_prev_loss_ + 1);
    roc_panic_if_msg (gap < 0, "RTPStats: negative gap detected %ld", gap);

    if (gap > 0) {
        lost_ += (size_t)gap;
        fract_lost_counter_ += (size_t)gap;
    }
    seqnum_prev_loss_ = packet.rtp()->seqnum;
    metrics_.num_packets_covered += 1 + (size_t)gap;
    metrics_.cum_loss = lost_;
    metrics_.fract_loss = (float)fract_lost_counter_ /
        (float)(metrics_.num_packets_covered);
}

core::nanoseconds_t rtp::LinkMeter::mean_jitter() const {
    return packet_jitter_stats_.mov_avg();
}

core::nanoseconds_t rtp::LinkMeter::var_jitter() const {
    return packet_jitter_stats_.mov_var();
}

void LinkMeter::reset_metrics() {
    fract_lost_counter_ = 0;
    metrics_.num_packets_covered = 0;
}
} // namespace rtp
} // namespace roc
