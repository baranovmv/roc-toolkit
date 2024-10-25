/*
 * Copyright (c) 2015 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "roc_audio/packetizer.h"
#include "roc_audio/sample_spec_to_str.h"
#include "roc_core/log.h"
#include "roc_core/panic.h"
#include "roc_status/code_to_str.h"

namespace roc {
namespace audio {

Packetizer::Packetizer(core::IArena& arena,
                       packet::IWriter& writer,
                       packet::IComposer& composer,
                       packet::ISequencer& sequencer,
                       IFrameEncoder& payload_encoder,
                       packet::PacketFactory& packet_factory,
                       core::nanoseconds_t packet_length,
                       const SampleSpec& sample_spec)
    : writer_(writer)
    , composer_(composer)
    , sequencer_(sequencer)
    , payload_encoder_(payload_encoder)
    , do_padding_(payload_encoder.variable_frame_bytecount())
    , packet_factory_(packet_factory)
    , sample_spec_(sample_spec)
    , samples_per_packet_(0)
    , payload_size_(0)
    , packet_pos_(0)
    , packet_cts_(0)
    , capture_ts_(0)
    , pkt_sz_stats_(arena, 1000)
    , init_status_(status::NoStatus) {
    roc_panic_if_msg(!sample_spec_.is_valid() || !sample_spec_.is_raw(),
                     "packetizer: required valid sample spec with raw format: %s",
                     sample_spec_to_str(sample_spec_).c_str());

    if (packet_length <= 0 || sample_spec.ns_2_stream_timestamp(packet_length) <= 0) {
        roc_log(LogError,
                "packetizer: invalid config: packet length is invalid:"
                " packet_length=%.3fms samples_per_packet=%lu",
                (double)packet_length / core::Millisecond,
                (unsigned long)samples_per_packet_);
        init_status_ = status::StatusBadConfig;
        return;
    }

    samples_per_packet_ = sample_spec.ns_2_stream_timestamp(packet_length);
    payload_size_ = do_padding_
                    ? sz_ceil_(payload_encoder.encoded_byte_count(samples_per_packet_))
                    : payload_encoder.encoded_byte_count(samples_per_packet_);

    roc_log(
        LogDebug,
        "packetizer: initializing:"
        " packet_length=%.3fms samples_per_packet=%lu payload_size=%lu sample_spec=%s",
        (double)packet_length / core::Millisecond, (unsigned long)samples_per_packet_,
        (unsigned long)payload_size_, sample_spec_to_str(sample_spec_).c_str());

    init_status_ = status::StatusOK;
}

status::StatusCode Packetizer::init_status() const {
    return init_status_;
}

size_t Packetizer::sample_rate() const {
    roc_panic_if(init_status_ != status::StatusOK);

    return sample_spec_.sample_rate();
}

const PacketizerMetrics& Packetizer::metrics() const {
    roc_panic_if(init_status_ != status::StatusOK);

    return metrics_;
}

status::StatusCode Packetizer::write(Frame& in_frame) {
    roc_panic_if(init_status_ != status::StatusOK);

    sample_spec_.validate_frame(in_frame);

    const sample_t* buffer_ptr = in_frame.raw_samples();
    size_t buffer_samples = in_frame.num_raw_samples() / sample_spec_.num_channels();

    capture_ts_ = in_frame.capture_timestamp();

    while (buffer_samples != 0) {
        if (!packet_) {
            const status::StatusCode code = begin_packet_();
            if (code != status::StatusOK) {
                return code;
            }
        }

        const size_t n_requested =
            std::min(buffer_samples, samples_per_packet_ - packet_pos_);

        const size_t n_encoded = payload_encoder_.write_samples(buffer_ptr, n_requested);
        roc_panic_if_not(n_encoded == n_requested);

        buffer_ptr += n_encoded * sample_spec_.num_channels();
        buffer_samples -= n_encoded;

        packet_pos_ += n_encoded;
        if (capture_ts_) {
            capture_ts_ += sample_spec_.samples_per_chan_2_ns(n_encoded);
        }

        if (packet_pos_ == samples_per_packet_) {
            const status::StatusCode code = end_packet_();
            if (code != status::StatusOK) {
                return code;
            }
        }
    }

    return status::StatusOK;
}

status::StatusCode Packetizer::flush() {
    roc_panic_if(init_status_ != status::StatusOK);

    if (packet_) {
        const status::StatusCode code = end_packet_();
        if (code != status::StatusOK) {
            return code;
        }
    }

    return status::StatusOK;
}

status::StatusCode Packetizer::begin_packet_() {
    const status::StatusCode code = create_packet_();
    if (code != status::StatusOK) {
        return code;
    }

    roc_panic_if(!packet_);

    packet_pos_ = 0;
    packet_cts_ = capture_ts_;

    // Begin encoding samples into packet.
    payload_encoder_.begin_frame(packet_->payload().data(), packet_->payload().size());

    return status::StatusOK;
}

status::StatusCode Packetizer::end_packet_() {
    // How much bytes we've written into packet payload.
    const size_t written_payload_size = payload_encoder_.encoded_byte_count(packet_pos_);
    roc_panic_if_not(written_payload_size <= payload_size_);

    // Finish encoding samples into packet.
    payload_encoder_.end_frame();

    // Fill protocol-specific fields.
    sequencer_.next(*packet_, packet_cts_, (packet::stream_timestamp_t)packet_pos_);

    if (do_padding_) {
        const size_t round_payload_size = sz_ceil_(written_payload_size);
        pkt_sz_stats_.add(round_payload_size);
        pad_packet_(written_payload_size, pkt_sz_stats_.mov_max());
    } else {
        // Apply padding if needed.
        if (packet_pos_ < samples_per_packet_) {
            pad_packet_(written_payload_size, payload_size_);
        }
    }

    const status::StatusCode code = writer_.write(packet_);
    if (code != status::StatusOK) {
        return code;
    }

    metrics_.encoded_packets++;
    metrics_.payload_bytes += written_payload_size;

    packet_ = NULL;
    packet_pos_ = 0;
    packet_cts_ = 0;

    return status::StatusOK;
}

status::StatusCode Packetizer::create_packet_() {
    packet::PacketPtr pp = packet_factory_.new_packet();
    if (!pp) {
        roc_log(LogError, "packetizer: can't allocate packet");
        return status::StatusNoMem;
    }

    pp->add_flags(packet::Packet::FlagAudio);

    core::Slice<uint8_t> buffer = packet_factory_.new_packet_buffer();
    if (!buffer) {
        roc_log(LogError, "packetizer: can't allocate buffer");
        return status::StatusNoMem;
    }

    if (!composer_.prepare(*pp, buffer, payload_size_)) {
        roc_log(LogError, "packetizer: can't prepare packet");
        return status::StatusNoMem;
    }
    pp->add_flags(packet::Packet::FlagPrepared);

    pp->set_buffer(buffer);

    packet_ = pp;
    return status::StatusOK;
}

void Packetizer::pad_packet_(size_t written_payload_size, size_t required_sz) {
    if (written_payload_size == required_sz) {
        return;
    }

    if (!composer_.pad(*packet_, required_sz - written_payload_size)) {
        roc_panic("packetizer: can't pad packet: orig_size=%lu actual_size=%lu",
                  (unsigned long)required_sz, (unsigned long)written_payload_size);
    }
}

size_t Packetizer::sz_ceil_(size_t sz) {
    const size_t round_up_to = 0x40;
    const size_t mask = round_up_to - 1;
    const size_t inv_mask = ((size_t)-1) ^ mask;

    if ((sz & mask) > 0) {
        return sz & inv_mask + round_up_to;
    } else {
        return sz & inv_mask;
    }
}

} // namespace audio
} // namespace roc
