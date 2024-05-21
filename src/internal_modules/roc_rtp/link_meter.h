/*
 * Copyright (c) 2024 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

//! @file roc_rtp/link_meter.h
//! @brief RTP link meter.

#ifndef ROC_RTP_LINK_METER_H_
#define ROC_RTP_LINK_METER_H_

#include "roc_audio/latency_monitor.h"
#include "roc_audio/sample_spec.h"
#include "roc_core/csv_dumper.h"
#include "roc_core/iarena.h"
#include "roc_core/mov_stats.h"
#include "roc_core/noncopyable.h"
#include "roc_core/time.h"
#include "roc_packet/ilink_meter.h"
#include "roc_packet/ireader.h"
#include "roc_packet/iwriter.h"
#include "roc_rtcp/reports.h"
#include "roc_rtp/encoding.h"
#include "roc_rtp/encoding_map.h"

namespace roc {
namespace rtp {

//! Link meter.
//!
//! Computes various link metrics based on sequence of RTP packets.
//! Inserted into pipeline in two points:
//!
//!  - As a writer, right after receiving packet, before storing
//!    packet in incoming queue. Here LinkMeter computes metrics
//!    that should be updated as early as possible.
//!
//!  - As a reader, right before decoding packet. Here LinkMeter
//!   1 computes metrics that can be updated only when packets
//!    are going to be played.
//!
//! In both cases, LinkMeter passes through packets to/from nested
//! writer/reader, and updates metrics.
class LinkMeter : public packet::ILinkMeter,
                  public packet::IWriter,
                  public core::NonCopyable<> {
public:
    //! Initialize.
    explicit LinkMeter(core::IArena& arena,
                       const EncodingMap& encoding_map,
                       const audio::SampleSpec& sample_spec,
                       audio::LatencyConfig latency_config,
                       core::CsvDumper* dumper);

    //! Check if metrics are already gathered and can be reported.
    virtual bool has_metrics() const;

    //! Get metrics.
    virtual const packet::LinkMetrics& metrics() const;

    //! Check if packet encoding already detected.
    bool has_encoding() const;

    //! Get detected encoding.
    //! @remarks
    //!  Panics if no encoding detected.
    const Encoding& encoding() const;

    //! Process RTCP report from sender.
    //! @remarks
    //!  Obtains additional information that can't be measured directly.
    void process_report(const rtcp::SendReport& report);

    //! Write packet and update metrics.
    //! @remarks
    //!  Invoked early in pipeline right after the packet is received.
    virtual ROC_ATTR_NODISCARD status::StatusCode write(const packet::PacketPtr& packet);

    //! Set nested packet writer.
    //! @remarks
    //!  Should be called before first write() call.
    void set_writer(packet::IWriter& writer);

    //! Get metrics.
    core::nanoseconds_t mean_jitter() const;

    size_t running_window_len() const;

private:
    void update_jitter_(const packet::Packet& packet);

    const EncodingMap& encoding_map_;
    const Encoding* encoding_;

    packet::IWriter* writer_;

    const audio::SampleSpec sample_spec_;

    bool first_packet_jitter_;
    const size_t win_len_;
    bool has_metrics_;

    packet::LinkMetrics metrics_;

    uint16_t first_seqnum_;
    uint32_t last_seqnum_hi_;
    uint16_t last_seqnum_lo_;

    ssize_t processed_packets_;
    core::nanoseconds_t prev_packet_enq_ts_;
    packet::stream_timestamp_t prev_stream_timestamp_;
    core::MovStats<core::nanoseconds_t> packet_jitter_stats_;

    core::CsvDumper* dumper_;
};

} // namespace rtp
} // namespace roc

#endif // ROC_RTP_LINK_METER_H_
