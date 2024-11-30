/*
 * Copyright (c) 2015 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "roc_fec/block_writer.h"
#include "roc_core/fast_random.h"
#include "roc_core/log.h"
#include "roc_core/panic.h"
#include "roc_packet/fec_scheme.h"
#include "roc_rtp/headers.h"
#include "roc_status/code_to_str.h"

namespace roc {
namespace fec {

BlockWriter::BlockWriter(const BlockWriterConfig& config,
                         packet::FecScheme fec_scheme,
                         IBlockEncoder& block_encoder,
                         packet::IWriter& writer,
                         packet::IComposer& source_composer,
                         packet::IComposer& repair_composer,
                         packet::PacketFactory& packet_factory,
                         core::IArena& arena)
    : cur_sblen_(0)
    , next_sblen_(0)
    , cur_rblen_(0)
    , next_rblen_(0)
    , repair_payload_size_(0)
    , block_encoder_(block_encoder)
    , pkt_writer_(writer)
    , source_composer_(source_composer)
    , repair_composer_(repair_composer)
    , packet_factory_(packet_factory)
    , source_block_(arena)
    , repair_block_(arena)
    , first_packet_(true)
    , cur_packet_(0)
    , fec_scheme_(fec_scheme)
    , prev_block_timestamp_valid_(false)
    , prev_block_timestamp_(0)
    , block_max_duration_(0)
    , init_status_(status::NoStatus) {
    if ((init_status_ = block_encoder_.init_status()) != status::StatusOK) {
        return;
    }

    cur_sbn_ = (packet::blknum_t)core::fast_random_range(0, packet::blknum_t(-1));
    cur_block_repair_sn_ =
        (packet::seqnum_t)core::fast_random_range(0, packet::seqnum_t(-1));

    if ((init_status_ = resize(config.n_source_packets, config.n_repair_packets))
        != status::StatusOK) {
        return;
    }

    init_status_ = status::StatusOK;
}

status::StatusCode BlockWriter::init_status() const {
    return init_status_;
}

packet::stream_timestamp_t BlockWriter::max_block_duration() const {
    roc_panic_if(init_status_ != status::StatusOK);

    return (packet::stream_timestamp_t)block_max_duration_;
}

status::StatusCode BlockWriter::resize(size_t sblen, size_t rblen) {
    roc_panic_if(init_status_ != status::StatusOK);

    if (next_sblen_ == sblen && next_rblen_ == rblen) {
        // nothing to do
        return status::StatusOK;
    }

    if (sblen == 0) {
        roc_log(LogError, "fec block writer: resize: sblen can't be zero");
        return status::StatusBadConfig;
    }

    const size_t new_blen = sblen + rblen;

    if (new_blen > block_encoder_.max_block_length()) {
        roc_log(LogDebug,
                "fec block writer: can't update block length, maximum value exceeded:"
                " cur_sbl=%lu cur_rbl=%lu new_sbl=%lu new_rbl=%lu max_blen=%lu",
                (unsigned long)cur_sblen_, (unsigned long)cur_rblen_,
                (unsigned long)sblen, (unsigned long)rblen,
                (unsigned long)block_encoder_.max_block_length());
        return status::StatusBadConfig;
    }

    roc_log(LogDebug,
            "fec block writer: update block size:"
            " cur_sbl=%lu cur_rbl=%lu new_sbl=%lu new_rbl=%lu",
            (unsigned long)cur_sblen_, (unsigned long)cur_rblen_, (unsigned long)sblen,
            (unsigned long)rblen);

    next_sblen_ = sblen;
    next_rblen_ = rblen;

    // max_block_duration() reports maximum duration since last resize,
    // so when resize happens, we reset maximum.
    prev_block_timestamp_valid_ = false;

    return status::StatusOK;
}

status::StatusCode BlockWriter::write(const packet::PacketPtr& pp) {
    roc_panic_if(init_status_ != status::StatusOK);
    roc_panic_if(!pp);

    validate_packet_(pp);

    if (first_packet_) {
        first_packet_ = false;
    }

    if (cur_packet_ == 0) {
        const status::StatusCode code = begin_block_(pp);
        if (code != status::StatusOK) {
            return code;
        }
    }

    const status::StatusCode code = write_source_packet_(pp);
    if (code != status::StatusOK) {
        return code;
    }

    if (HEDLEY_UNLIKELY(!pp->rtp())) {
        roc_panic("fec block writer: support RTP source packets only");
    } else {
        // Copy the entire source rtp packet together with header, so as to
        // be able to alter it before fec encoding.
        // We want SSRC of each source header to hold actual payload size of the packet,
        // in order to be able to recover the size on a receiver side.
        source_block_[cur_packet_].reslice(0, pp->rtp()->overall_sz());
        uint8_t* pdest = source_block_[cur_packet_].data();
        memcpy(pdest, pp->rtp()->header.data(), pp->rtp()->header.size());
        rtp::Header& header = *(rtp::Header*)pdest;
        pdest += pp->rtp()->header.size();
        memcpy(pdest, pp->rtp()->payload.data(), pp->rtp()->payload.size());
        pdest += pp->rtp()->payload.size();
        if (!!pp->rtp()->padding.size()) {
            memcpy(pdest, pp->rtp()->padding.data(), pp->rtp()->padding.size());
        }
//        header.set_ssrc(((header.ssrc() ^ 0xFFFF0000) & 0xFFFF0000)
//                        | (pp->rtp()->overall_sz() & 0x0000FFFF));
    }

    cur_packet_++;

    if (cur_packet_ == cur_sblen_) {
        const status::StatusCode code = end_block_();
        if (code != status::StatusOK) {
            return code;
        }

        next_block_();
    }

    return status::StatusOK;
}

status::StatusCode BlockWriter::begin_block_(const packet::PacketPtr& pp) {
    update_block_duration_(pp);

    if (!apply_sizes_(next_sblen_, next_rblen_)) {
        return status::StatusNoMem;
    }

    roc_log(LogTrace,
            "fec block writer: begin block: sbn=%lu sblen=%lu rblen=%lu",
            (unsigned long)cur_sbn_, (unsigned long)cur_sblen_, (unsigned long)cur_rblen_);

    return status::StatusOK;
}

status::StatusCode BlockWriter::end_block_() {
    status::StatusCode code = status::NoStatus;

    repair_payload_size_ = 0;
    for (size_t i = 0; i < cur_sblen_; ++i) {
        repair_payload_size_ = std::max(repair_payload_size_, source_block_[i].size());
    }
    if ((code = block_encoder_.begin_block(cur_sblen_, cur_rblen_,
                                           repair_payload_size_)) != status::StatusOK){
        roc_log(LogError,
                "fec block writer: can't begin encoder block:"
                " sblen=%lu rblen=%lu",
                (unsigned long)cur_sblen_, (unsigned long)cur_rblen_);

        return code;
    }
    for (size_t i = 0; i < cur_sblen_; ++i) {
        block_encoder_.set_buffer(i, source_block_[i]);
    }

    if ((code = make_repair_packets_()) != status::StatusOK) {
        return code;
    }

    if ((code = encode_repair_packets_()) != status::StatusOK) {
        return code;
    }

    if ((code = compose_repair_packets_()) != status::StatusOK) {
        return code;
    }

    if ((code = write_repair_packets_()) != status::StatusOK) {
        return code;
    }

    block_encoder_.end_block();

    return status::StatusOK;
}

void BlockWriter::next_block_() {
    cur_block_repair_sn_ += (packet::seqnum_t)cur_rblen_;
    cur_sbn_++;
    cur_packet_ = 0;
}

bool BlockWriter::apply_sizes_(size_t sblen, size_t rblen) {
    if (source_block_.size() < sblen) {
        const size_t old_sz = source_block_.size();
        if (!source_block_.resize(sblen)) {
            roc_log(LogError,
                    "fec block writer: can't allocate source block memory:"
                    " cur_sbl=%lu new_bl=%lu",
                    (unsigned long)source_block_.size(), (unsigned long)sblen);
            return false;
        }
        for (size_t i = old_sz; i < sblen; ++i) {
            source_block_[i] = packet_factory_.new_packet_buffer();
            if (!source_block_[i]) {
                roc_log(LogError, "fec block writer: can't allocate buffer");
                return status::StatusNoMem;
            }
        }
    }

    if (repair_block_.size() != rblen) {
        if (!repair_block_.resize(rblen)) {
            roc_log(LogError,
                    "fec block writer: can't allocate repair block memory:"
                    " cur_rbl=%lu new_rbl=%lu",
                    (unsigned long)repair_block_.size(), (unsigned long)rblen);
            return false;
        }
    }

    cur_sblen_ = sblen;
    cur_rblen_ = rblen;

    return true;
}

status::StatusCode BlockWriter::write_source_packet_(const packet::PacketPtr& pp) {
    fill_packet_fec_fields_(pp, (packet::seqnum_t)cur_packet_);

    if (!source_composer_.compose(*pp)) {
        // TODO(gh-183): forward status from crepair_block_.size()omposer
        return status::StatusBadBuffer;
    }
    pp->add_flags(packet::Packet::FlagComposed);

    return pkt_writer_.write(pp);
}

status::StatusCode BlockWriter::make_repair_packets_() {
    for (size_t i = 0; i < cur_rblen_; i++) {
        packet::PacketPtr rp;
        const status::StatusCode code = make_repair_packet_((packet::seqnum_t)i, rp);
        if (code != status::StatusOK) {
            return code;
        }

        roc_panic_if(!rp);
        repair_block_[i] = rp;
    }

    return status::StatusOK;
}

status::StatusCode BlockWriter::make_repair_packet_(packet::seqnum_t pack_n,
                                                    packet::PacketPtr& result_packet) {
    packet::PacketPtr packet = packet_factory_.new_packet();
    if (!packet) {
        roc_log(LogError, "fec block writer: can't allocate packet");
        return status::StatusNoMem;
    }

    core::Slice<uint8_t> buffer = packet_factory_.new_packet_buffer();
    if (!buffer) {
        roc_log(LogError, "fec block writer: can't allocate buffer");
        return status::StatusNoMem;
    }

    if (!repair_composer_.align(buffer, 0, block_encoder_.buffer_alignment())) {
        roc_log(LogError, "fec block writer: can't align packet buffer");
        // TODO(gh-183): forward status from composer
        return status::StatusBadBuffer;
    }

    if (!repair_composer_.prepare(*packet, buffer, repair_payload_size_)) {
        roc_log(LogError, "fec block writer: can't prepare packet");
        // TODO(gh-183): forward status from composer
        return status::StatusBadBuffer;
    }
    packet->add_flags(packet::Packet::FlagPrepared);

    packet->set_buffer(buffer);

    validate_packet_(packet);
    fill_packet_fec_fields_(packet, (packet::seqnum_t)cur_sblen_ + pack_n);

    result_packet = packet;
    return status::StatusOK;
}

status::StatusCode BlockWriter::encode_repair_packets_() {
    for (size_t i = 0; i < cur_rblen_; i++) {
        packet::PacketPtr rp = repair_block_[i];
        if (!rp) {
            continue;
        }
        block_encoder_.set_buffer(cur_sblen_ + i, rp->fec()->payload);
    }

    block_encoder_.fill_buffers();

    return status::StatusOK;
}

status::StatusCode BlockWriter::compose_repair_packets_() {
    for (size_t i = 0; i < cur_rblen_; i++) {
        packet::PacketPtr rp = repair_block_[i];
        if (!rp) {
            continue;
        }

        if (!repair_composer_.compose(*rp)) {
            // TODO(gh-183): forward status from composer
            return status::StatusBadBuffer;
        }
        rp->add_flags(packet::Packet::FlagComposed);
    }

    return status::StatusOK;
}

status::StatusCode BlockWriter::write_repair_packets_() {
    for (size_t i = 0; i < cur_rblen_; i++) {
        packet::PacketPtr rp = repair_block_[i];
        if (!rp) {
            continue;
        }

        const status::StatusCode code = pkt_writer_.write(repair_block_[i]);
        if (code != status::StatusOK) {
            return code;
        }

        repair_block_[i] = NULL;
    }

    return status::StatusOK;
}

void BlockWriter::fill_packet_fec_fields_(const packet::PacketPtr& packet,
                                          packet::seqnum_t pack_n) {
    packet::FEC& fec = *packet->fec();

    fec.encoding_symbol_id = pack_n;
    fec.source_block_number = cur_sbn_;
    fec.source_block_length = cur_sblen_;
    fec.block_length = cur_sblen_ + cur_rblen_;
}

void BlockWriter::validate_packet_(const packet::PacketPtr& pp) {
    if (!pp->has_flags(packet::Packet::FlagPrepared)) {
        roc_panic("fec block writer: unexpected packet: must be prepared");
    }

    if (pp->has_flags(packet::Packet::FlagComposed)) {
        roc_panic("fec block writer: unexpected packet: must not be composed");
    }

    if (!pp->has_flags(packet::Packet::FlagFEC)) {
        roc_panic("fec block writer: unexpected non-fec packet");
    }

    if (pp->fec()->fec_scheme != fec_scheme_) {
        roc_panic("fec block writer: unexpected packet fec scheme:"
                  " packet_scheme=%s session_scheme=%s",
                  packet::fec_scheme_to_str(pp->fec()->fec_scheme),
                  packet::fec_scheme_to_str(fec_scheme_));
    }

    const size_t payload_size = pp->fec()->payload.size();

    if (payload_size == 0) {
        roc_panic("fec block writer: unexpected packet with zero payload size");
    }
}

void BlockWriter::update_block_duration_(const packet::PacketPtr& curr_block_pkt) {
    packet::stream_timestamp_diff_t block_dur = 0;
    if (prev_block_timestamp_valid_) {
        block_dur = packet::stream_timestamp_diff(curr_block_pkt->stream_timestamp(),
                                                  prev_block_timestamp_);
    }

    if (block_dur < 0) {
        roc_log(LogTrace, "fec reader: negative block duration: prev_ts=%lu curr_ts=%lu",
                (unsigned long)prev_block_timestamp_,
                (unsigned long)curr_block_pkt->stream_timestamp());
        prev_block_timestamp_valid_ = false;
    } else {
        block_max_duration_ = std::max(block_max_duration_, block_dur);
        prev_block_timestamp_ = curr_block_pkt->stream_timestamp();
        prev_block_timestamp_valid_ = true;
    }
}

} // namespace fec
} // namespace roc
