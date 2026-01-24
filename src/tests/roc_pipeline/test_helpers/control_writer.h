/*
 * Copyright (c) 2023 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef ROC_PIPELINE_TEST_HELPERS_CONTROL_WRITER_H_
#define ROC_PIPELINE_TEST_HELPERS_CONTROL_WRITER_H_

#include <CppUTest/TestHarness.h>

#include "test_helpers/utils.h"

#include "roc_audio/latency_config.h"
#include "roc_core/noncopyable.h"
#include "roc_core/time.h"
#include "roc_packet/ilink_meter.h"
#include "roc_packet/iwriter.h"
#include "roc_packet/ntp.h"
#include "roc_packet/packet_factory.h"
#include "roc_packet/units.h"
#include "roc_rtcp/builder.h"
#include "roc_rtcp/headers.h"
#include "roc_rtcp/print_packet.h"

namespace roc {
namespace pipeline {
namespace test {

// Generates control packets and pass them to destination writer
class ControlWriter : public core::NonCopyable<> {
public:
    ControlWriter(packet::IWriter& writer,
                  packet::PacketFactory& packet_factory,
                  const address::SocketAddr& src_addr,
                  const address::SocketAddr& dst_addr)
        : writer_(writer)
        , packet_factory_(packet_factory)
        , src_addr_(src_addr)
        , dst_addr_(dst_addr)
        , local_source_(0)
        , remote_source_(0)
        , cname_("test_cname") {
    }

    void write_sender_report(packet::ntp_timestamp_t ntp_ts,
                             packet::stream_timestamp_t rtp_ts) {
        core::Slice<uint8_t> buff = packet_factory_.new_packet_buffer();
        CHECK(buff);

        buff.reslice(0, 0);

        rtcp::Config cfg;
        rtcp::Builder bld(cfg, buff);

        rtcp::header::SenderReportPacket sr;
        sr.set_ssrc(local_source_);
        sr.set_ntp_timestamp(ntp_ts);
        sr.set_rtp_timestamp(rtp_ts);

        rtcp::SdesChunk chunk;
        chunk.ssrc = local_source_;
        rtcp::SdesItem item;
        item.type = rtcp::header::SDES_CNAME;
        item.text = cname_;

        bld.begin_sr(sr);
        bld.end_sr();

        bld.begin_sdes();
        bld.begin_sdes_chunk(chunk);
        bld.add_sdes_item(item);
        bld.end_sdes_chunk();
        bld.end_sdes();

        CHECK(bld.is_ok());

        LONGS_EQUAL(status::StatusOK, writer_.write(new_packet_(buff)));
    }

    // Write a sender report with XR DLRR block.
    // This is used to simulate a sender responding to a receiver's RRTR
    // so that the receiver can compute RTT and clock offset.
    void write_sender_report_with_dlrr(packet::ntp_timestamp_t sr_ntp_ts,
                                       packet::stream_timestamp_t rtp_ts,
                                       packet::stream_source_t receiver_ssrc,
                                       packet::ntp_timestamp_t last_rr_ntp,
                                       packet::ntp_timestamp_t delay_last_rr) {
        core::Slice<uint8_t> buff = packet_factory_.new_packet_buffer();
        CHECK(buff);

        buff.reslice(0, 0);

        rtcp::Config cfg;
        rtcp::Builder bld(cfg, buff);

        // Build SR (Sender Report)
        rtcp::header::SenderReportPacket sr;
        sr.set_ssrc(local_source_);
        sr.set_ntp_timestamp(sr_ntp_ts);
        sr.set_rtp_timestamp(rtp_ts);

        bld.begin_sr(sr);
        bld.end_sr();

        // Build XR with DLRR block
        rtcp::header::XrPacket xr;
        xr.set_ssrc(local_source_);

        rtcp::header::XrDlrrBlock dlrr;

        rtcp::header::XrDlrrSubblock dlrr_sub;
        dlrr_sub.set_ssrc(receiver_ssrc);
        dlrr_sub.set_last_rr(last_rr_ntp);
        dlrr_sub.set_delay_last_rr(delay_last_rr);

        bld.begin_xr(xr);
        bld.begin_xr_dlrr(dlrr);
        bld.add_xr_dlrr_report(dlrr_sub);
        bld.end_xr_dlrr();
        bld.end_xr();

        // Build SDES
        rtcp::SdesChunk chunk;
        chunk.ssrc = local_source_;
        rtcp::SdesItem item;
        item.type = rtcp::header::SDES_CNAME;
        item.text = cname_;

        bld.begin_sdes();
        bld.begin_sdes_chunk(chunk);
        bld.add_sdes_item(item);
        bld.end_sdes_chunk();
        bld.end_sdes();

        CHECK(bld.is_ok());

        LONGS_EQUAL(status::StatusOK, writer_.write(new_packet_(buff)));
    }

    void write_receiver_report(packet::ntp_timestamp_t ntp_ts,
                               const audio::SampleSpec& sample_spec) {
        write_receiver_report_with_sr_ref(ntp_ts, ntp_ts, 0, sample_spec);
    }

    // Write a receiver report with separate RRTR and last_sr values.
    // This is used to simulate proper RTCP exchange for clock_offset computation.
    // - rrtr_ntp: RRTR timestamp (when receiver sent RR, in receiver's clock)
    // - last_sr_ntp: Middle 32 bits of sender's SR NTP timestamp
    // - delay_last_sr: Delay between receiving SR and sending RR (in 1/65536 sec units)
    void write_receiver_report_with_sr_ref(packet::ntp_timestamp_t rrtr_ntp,
                                           packet::ntp_timestamp_t last_sr_ntp,
                                           packet::ntp_timestamp_t delay_last_sr,
                                           const audio::SampleSpec& sample_spec) {
        core::Slice<uint8_t> buff = packet_factory_.new_packet_buffer();
        CHECK(buff);

        buff.reslice(0, 0);

        rtcp::Config cfg;
        rtcp::Builder bld(cfg, buff);

        rtcp::header::ReceiverReportPacket rr;
        rr.set_ssrc(local_source_);

        rtcp::header::ReceptionReportBlock rr_blk;
        rr_blk.set_ssrc(remote_source_);
        rr_blk.set_cum_loss(link_metrics_.lost_packets);
        rr_blk.set_last_seqnum(link_metrics_.ext_last_seqnum);
        rr_blk.set_jitter(sample_spec.ns_2_stream_timestamp(link_metrics_.peak_jitter));
        rr_blk.set_last_sr(last_sr_ntp);
        rr_blk.set_delay_last_sr(delay_last_sr);

        rtcp::header::XrPacket xr;
        xr.set_ssrc(local_source_);

        rtcp::header::XrRrtrBlock rrtr;
        rrtr.set_ntp_timestamp(rrtr_ntp);

        rtcp::header::XrMeasurementInfoBlock ms_info;
        ms_info.set_ssrc(remote_source_);
        ms_info.set_first_seq((packet::seqnum_t)link_metrics_.ext_first_seqnum);

        rtcp::header::XrDelayMetricsBlock delay_metrics;
        delay_metrics.set_ssrc(remote_source_);
        delay_metrics.set_e2e_latency(
            packet::nanoseconds_2_ntp(latency_metrics_.e2e_latency));

        rtcp::header::XrQueueMetricsBlock queue_metrics;
        queue_metrics.set_ssrc(remote_source_);
        queue_metrics.set_niq_latency(
            packet::nanoseconds_2_ntp(latency_metrics_.niq_latency));
        queue_metrics.set_niq_stalling(
            packet::nanoseconds_2_ntp(latency_metrics_.niq_stalling));

        rtcp::SdesChunk chunk;
        chunk.ssrc = local_source_;
        rtcp::SdesItem item;
        item.type = rtcp::header::SDES_CNAME;
        item.text = cname_;

        bld.begin_rr(rr);
        bld.add_rr_report(rr_blk);
        bld.end_rr();

        bld.begin_xr(xr);
        bld.add_xr_rrtr(rrtr);
        bld.add_xr_measurement_info(ms_info);
        bld.add_xr_delay_metrics(delay_metrics);
        bld.add_xr_queue_metrics(queue_metrics);
        bld.end_xr();

        bld.begin_sdes();
        bld.begin_sdes_chunk(chunk);
        bld.add_sdes_item(item);
        bld.end_sdes_chunk();
        bld.end_sdes();

        CHECK(bld.is_ok());

        LONGS_EQUAL(status::StatusOK, writer_.write(new_packet_(buff)));
    }

    void set_cname(const char* cname) {
        cname_ = cname;
    }

    void set_local_source(packet::stream_source_t source) {
        local_source_ = source;
    }

    void set_remote_source(packet::stream_source_t source) {
        remote_source_ = source;
    }

    void set_link_metrics(const packet::LinkMetrics& link_metrics) {
        link_metrics_ = link_metrics;
    }

    void set_latency_metrics(const audio::LatencyMetrics& latency_metrics) {
        latency_metrics_ = latency_metrics;
    }

private:
    packet::PacketPtr new_packet_(core::Slice<uint8_t> buffer) {
        packet::PacketPtr pp = packet_factory_.new_packet();
        CHECK(pp);

        pp->add_flags(packet::Packet::FlagUDP);

        pp->udp()->src_addr = src_addr_;
        pp->udp()->dst_addr = dst_addr_;

        pp->set_buffer(buffer);

        if (core::Logger::instance().get_level() >= LogTrace) {
            rtcp::print_packet(buffer);
        }

        return pp;
    }

    packet::IWriter& writer_;

    packet::PacketFactory& packet_factory_;

    address::SocketAddr src_addr_;
    address::SocketAddr dst_addr_;

    packet::stream_source_t local_source_;
    packet::stream_source_t remote_source_;

    packet::LinkMetrics link_metrics_;
    audio::LatencyMetrics latency_metrics_;

    const char* cname_;
};

} // namespace test
} // namespace pipeline
} // namespace roc

#endif // ROC_PIPELINE_TEST_HELPERS_CONTROL_WRITER_H_
