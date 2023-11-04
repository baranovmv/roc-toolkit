/*
 * Copyright (c) 2023 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "stream_stats_monitor.h"
#include "roc_status/status_code.h"

namespace roc {
namespace rtp {

StreamStatsMonitor::StreamStatsMonitor(packet::IReader& reader,
                                       core::IArena& arena,
                                       const audio::SampleSpec& sample_spec,
                                       const StreamStatsConfig &config)
    : reader_(reader)
    , arena_(arena)
    , config_(config)
    , sample_spec_(sample_spec)
    , rtp_(arena, config)
{}

status::StatusCode StreamStatsMonitor::read(packet::PacketPtr& packet)
{
    status::StatusCode result = reader_.read(packet);
    if (result == status::StatusOK){
        if (packet->rtp()) {
            rtp_.process(packet);
        }
    }

    return result;
}
StreamStats StreamStatsMonitor::stats() const {
    StreamStats result;
    result
    return StreamStats();
}

StreamStatsMonitor::RTPStats::RTPStats(core::IArena& arena, const StreamStatsConfig &config)
    : lost_(0)
    , jitter_processed_(0)
    , prev_packet_enq_ts_(-1)
    , prev_seqnum_(0)
    , packet_jitter_stats_(arena, config.window_npackets)
{}

bool StreamStatsMonitor::RTPStats::process(const packet::PacketPtr& packet) {
    if (prev_packet_enq_ts_ == -1) {
        prev_packet_enq_ts_ = packet->udp()->enqueue_ts;

    } else {
        const size_t gap = gap_(packet);
        // Compute jitter only on consequential packets.
        if (gap == 0){
            const core::nanoseconds_t  d_enq_ts = packet->udp()->enqueue_ts - prev_packet_enq_ts_;
            const core::nanoseconds_t d_capt_ts = packet->rtp()->capture_timestamp - prev_capt_ts;
            packet_jitter_stats_.add(d_enq_ts - d_capt_ts);
            jitter_processed_++;
        } else {
            lost_ += gap;
        }
    }

    prev_packet_enq_ts_ = packet->udp()->enqueue_ts;
    prev_seqnum_ = packet->rtp()->seqnum;
    prev_stream_timestamp = packet->rtp()->stream_timestamp;
    prev_capt_ts = packet->rtp()->capture_timestamp;
    return false;
}

size_t StreamStatsMonitor::RTPStats::gap_(const packet::PacketPtr& packet) const {
    if (prev_packet_enq_ts_ == -1) {
        roc_panic("RTPStats: attempt to detect gap on the first received packet");
    }

    return (size_t)abs(packet::seqnum_diff( packet->rtp()->seqnum, prev_seqnum_ + 1));
}

core::nanoseconds_t StreamStatsMonitor::RTPStats::mean_jitter() const {
    return packet_jitter_stats_.mov_avg();
}

core::nanoseconds_t StreamStatsMonitor::RTPStats::var_jitter() const {
    return packet_jitter_stats_.mov_var();
}
} // namespace packet
} // namespace roc