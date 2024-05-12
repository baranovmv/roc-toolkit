/*
 * Copyright (c) 2015 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "roc_audio/mixer.h"
#include "roc_audio/sample_spec_to_str.h"
#include "roc_core/log.h"
#include "roc_core/panic.h"
#include "roc_core/stddefs.h"

namespace roc {
namespace audio {

Mixer::Mixer(FrameFactory& frame_factory,
             const SampleSpec& sample_spec,
             bool enable_timestamps)
    : sample_spec_(sample_spec)
    , enable_timestamps_(enable_timestamps)
    , init_status_(status::NoStatus) {
    roc_panic_if_msg(!sample_spec_.is_valid() || !sample_spec_.is_raw(),
                     "mixer: required valid sample spec with raw format: %s",
                     sample_spec_to_str(sample_spec_).c_str());

    temp_buf_ = frame_factory.new_raw_buffer();
    if (!temp_buf_) {
        roc_log(LogError, "mixer: can't allocate temporary buffer");
        init_status_ = status::StatusNoMem;
        return;
    }

    temp_buf_.reslice(0, temp_buf_.capacity());

    init_status_ = status::StatusOK;
}

status::StatusCode Mixer::init_status() const {
    return init_status_;
}

void Mixer::add_input(IFrameReader& reader) {
    roc_panic_if(init_status_ != status::StatusOK);

    readers_.push_back(reader);
}

void Mixer::remove_input(IFrameReader& reader) {
    roc_panic_if(init_status_ != status::StatusOK);

    readers_.remove(reader);
}

status::StatusCode Mixer::read(Frame& frame) {
    roc_panic_if(init_status_ != status::StatusOK);

    const size_t max_read = temp_buf_.size();

    sample_t* samples = frame.raw_samples();
    size_t n_samples = frame.num_raw_samples();

    unsigned flags = 0;
    core::nanoseconds_t capture_ts = 0;

    while (n_samples != 0) {
        // Read size is limited with the size of temporary buffer which
        // we retrieved from pool. Usually it's big enough, but we still
        // handle situation when requested read is larger.
        size_t n_read = n_samples;
        if (n_read > max_read) {
            n_read = max_read;
        }

        const status::StatusCode code = read_(samples, n_read, flags, capture_ts);
        if (code != status::StatusOK) {
            return code;
        }

        samples += n_read;
        n_samples -= n_read;
    }

    frame.set_flags(flags);
    frame.set_duration(frame.num_raw_samples() / sample_spec_.num_channels());
    frame.set_capture_timestamp(capture_ts);

    return status::StatusOK;
}

status::StatusCode Mixer::read_(sample_t* out_data,
                                size_t out_size,
                                unsigned& out_flags,
                                core::nanoseconds_t& out_cts) {
    roc_panic_if(!out_data);
    roc_panic_if(out_size == 0);

    const size_t n_readers = readers_.size();

    core::nanoseconds_t cts_base = 0;
    double cts_sum = 0;
    size_t cts_count = 0;

    // Zeroise output frame.
    memset(out_data, 0, out_size * sizeof(sample_t));

    for (IFrameReader* rp = readers_.front(); rp; rp = readers_.nextof(*rp)) {
        sample_t* temp_data = temp_buf_.data();

        Frame temp_frame(temp_data, out_size);

        const status::StatusCode code = rp->read(temp_frame);
        if (code == status::StatusDrain) {
            continue;
        }
        if (code != status::StatusOK) {
            return code;
        }

        for (size_t n = 0; n < out_size; n++) {
            out_data[n] += temp_data[n];

            // Saturate on overflow.
            out_data[n] = std::min(out_data[n], Sample_Max);
            out_data[n] = std::max(out_data[n], Sample_Min);
        }

        // Accumulate flags from all mixed frames.
        out_flags |= temp_frame.flags();

        const core::nanoseconds_t frame_cts = temp_frame.capture_timestamp();

        if (enable_timestamps_ && frame_cts != 0) {
            // Subtract first non-zero timestamp from all other timestamps.
            // Since timestamp calculation is used only when inputs are synchronous
            // and their timestamps are close, this effectively makes all values
            // small, avoiding overflow and rounding errors when adding them.
            if (cts_base == 0) {
                cts_base = frame_cts;
            }
            cts_sum += double(frame_cts - cts_base);
            cts_count++;
        }
    }

    if (cts_count != 0) {
        // Compute average timestamp.
        // Don't forget to compensate everything that we subtracted above.
        out_cts = core::nanoseconds_t(cts_base * ((double)cts_count / n_readers)
                                      + cts_sum / (double)n_readers);
    }

    return status::StatusOK;
}

} // namespace audio
} // namespace roc
