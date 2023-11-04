/*
 * Copyright (c) 2023 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

//! @file roc_rtp/stream_stats_monitor.h
//! @brief Calculates basic network stream statistics.


#ifndef ROC_PACKET_STREAMSTATSMONITOR_H_
#define ROC_PACKET_STREAMSTATSMONITOR_H_

#include "roc_core/noncopyable.h"
#include "roc_packet/ireader.h"
#include "roc_core/mov_stats.h"
#include "roc_core/time.h"
#include "roc_audio/sample_spec.h"
#include "roc_packet/units.h"
#include "roc_status/status_code.h"

namespace roc {
namespace rtp {

struct StreamStatsConfig {
    size_t window_npackets;
};

struct StreamStats {
    core::nanoseconds_t windowed_max_jitter;

    size_t windowed_npackets;
    size_t windowed_lost_packets;
    size_t windowed_recovered_packets;

    float packet_loss_rate;
};

class StreamStatsMonitor : public packet::IReader, public core::NonCopyable<> {
public:
    StreamStatsMonitor(packet::IReader& reader, core::IArena& arena,
                       const audio::SampleSpec& sample_spec,
                       const StreamStatsConfig &config);

    virtual ROC_ATTR_NODISCARD status::StatusCode read(packet::PacketPtr& packet);

    ROC_ATTR_NODISCARD StreamStats stats() const;
private:
    packet::IReader&    reader_;
    core::IArena& arena_;
    const StreamStatsConfig config_;
    const audio::SampleSpec sample_spec_;

    class RTPStats {
    public:
        RTPStats(core::IArena& arena, const StreamStatsConfig &config);
        bool process(const packet::PacketPtr& packet);

        core::nanoseconds_t mean_jitter() const;
        core::nanoseconds_t var_jitter() const;
    private:
        size_t lost_;
        size_t jitter_processed_;
        core::nanoseconds_t prev_packet_enq_ts_;
        packet::seqnum_t prev_seqnum_;
        packet::stream_timestamp_t prev_stream_timestamp;
        core::nanoseconds_t prev_capt_ts;
        core::MovStats<core::nanoseconds_t> packet_jitter_stats_;

        size_t gap_(const packet::PacketPtr &packet) const;

    } rtp_;
};

} // namespace packet
} // namespace roc

#endif // ROC_PACKET_STREAMSTATSMONITOR_H_
