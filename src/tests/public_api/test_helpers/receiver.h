/*
 * Copyright (c) 2020 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef ROC_PUBLIC_API_TEST_HELPERS_RECEIVER_H_
#define ROC_PUBLIC_API_TEST_HELPERS_RECEIVER_H_

#include <CppUTest/TestHarness.h>

#include <stdio.h>

#include "test_helpers/context.h"
#include "test_helpers/utils.h"

#include "roc_core/array.h"
#include "roc_core/panic.h"
#include "roc_core/stddefs.h"

#include "roc/config.h"
#include "roc/receiver.h"

namespace roc {
namespace api {
namespace test {

class Receiver : public core::Thread {
public:
    Receiver(Context& context,
             roc_receiver_config& config,
             float sample_step,
             size_t num_chans,
             size_t frame_size)
        : recv_(NULL)
        , sample_step_(sample_step)
        , num_chans_(num_chans)
        , frame_samples_(frame_size * num_chans) {
        CHECK(roc_receiver_open(context.get(), &config, &recv_) == 0);
        CHECK(recv_);
    }

    virtual ~Receiver() {
        for (size_t slot = 0; slot < source_endp_.size(); slot++) {
            if (source_endp_[slot]) {
                CHECK(roc_endpoint_deallocate(source_endp_[slot]) == 0);
            }
        }

        for (size_t slot = 0; slot < repair_endp_.size(); slot++) {
            if (repair_endp_[slot]) {
                CHECK(roc_endpoint_deallocate(repair_endp_[slot]) == 0);
            }
        }

        CHECK(roc_receiver_close(recv_) == 0);
    }

    void bind(unsigned flags, roc_slot slot = ROC_SLOT_DEFAULT) {
        if (source_endp_.size() < slot + 1) {
            source_endp_.resize(slot + 1);
        }

        if (repair_endp_.size() < slot + 1) {
            repair_endp_.resize(slot + 1);
        }

        if (flags & FlagRS8M) {
            CHECK(roc_endpoint_allocate(&source_endp_[slot]) == 0);
            CHECK(roc_endpoint_set_uri(source_endp_[slot], "rtp+rs8m://127.0.0.1:0")
                  == 0);

            CHECK(roc_endpoint_allocate(&repair_endp_[slot]) == 0);
            CHECK(roc_endpoint_set_uri(repair_endp_[slot], "rs8m://127.0.0.1:0") == 0);

            CHECK(roc_receiver_bind(recv_, slot, ROC_INTERFACE_AUDIO_SOURCE,
                                    source_endp_[slot])
                  == 0);
            CHECK(roc_receiver_bind(recv_, slot, ROC_INTERFACE_AUDIO_REPAIR,
                                    repair_endp_[slot])
                  == 0);
        } else if (flags & FlagLDPC) {
            CHECK(roc_endpoint_allocate(&source_endp_[slot]) == 0);
            CHECK(roc_endpoint_set_uri(source_endp_[slot], "rtp+ldpc://127.0.0.1:0")
                  == 0);

            CHECK(roc_endpoint_allocate(&repair_endp_[slot]) == 0);
            CHECK(roc_endpoint_set_uri(repair_endp_[slot], "ldpc://127.0.0.1:0") == 0);

            CHECK(roc_receiver_bind(recv_, slot, ROC_INTERFACE_AUDIO_SOURCE,
                                    source_endp_[slot])
                  == 0);
            CHECK(roc_receiver_bind(recv_, slot, ROC_INTERFACE_AUDIO_REPAIR,
                                    repair_endp_[slot])
                  == 0);
        } else {
            CHECK(roc_endpoint_allocate(&source_endp_[slot]) == 0);
            CHECK(roc_endpoint_set_uri(source_endp_[slot], "rtp://127.0.0.1:0") == 0);

            CHECK(roc_receiver_bind(recv_, slot, ROC_INTERFACE_AUDIO_SOURCE,
                                    source_endp_[slot])
                  == 0);
        }
    }

    const roc_endpoint* source_endpoint(roc_slot slot = ROC_SLOT_DEFAULT) const {
        return source_endp_[slot];
    }

    const roc_endpoint* repair_endpoint(roc_slot slot = ROC_SLOT_DEFAULT) const {
        return repair_endp_[slot];
    }

    void receive() {
        float rx_buff[MaxBufSize];

        size_t sample_num = 0;
        size_t frame_num = 0;

        bool wait_for_signal = true;
        size_t identical_sample_num = 0;

        size_t nb_success = PacketSamples * SourcePackets * 4;

        float prev_sample = sample_step_;

        while (identical_sample_num < nb_success) {
            frame_num++;

            roc_frame frame;
            memset(&frame, 0, sizeof(frame));
            frame.samples = rx_buff;
            frame.samples_size = frame_samples_ * sizeof(float);

            const int ret = roc_receiver_read(recv_, &frame);
            roc_panic_if_not(ret == 0);

            size_t ns = 0;

            if (wait_for_signal) {
                for (; ns < frame_samples_ && is_zero_(rx_buff[ns]); ns += num_chans_) {
                }

                if (ns < frame_samples_) {
                    wait_for_signal = false;

                    prev_sample = rx_buff[ns];
                    ns += num_chans_;
                }
            }

            if (!wait_for_signal) {
                for (; ns < frame_samples_; ns += num_chans_) {
                    float curr_sample = 0;

                    for (size_t nc = 0; nc < num_chans_; ++nc) {
                        curr_sample = rx_buff[ns + nc];

                        if (is_zero_(increment_sample_value(prev_sample, sample_step_)
                                     - curr_sample)) {
                            identical_sample_num++;
                        } else if (!is_zero_(prev_sample)
                                   && !is_zero_(curr_sample)) { // Allows stream shifts
                            char sbuff[256];
                            snprintf(
                                sbuff, sizeof(sbuff),
                                "failed comparing samples:\n\n"
                                "sample_num: %lu\n"
                                "frame_num: %lu, frame_off=%lu chan=%lu\n"
                                "original: %f, received: %f\n",
                                (unsigned long)identical_sample_num,
                                (unsigned long)frame_num, (unsigned long)ns,
                                (unsigned long)nc,
                                (double)increment_sample_value(prev_sample, sample_step_),
                                (double)curr_sample);
                            roc_panic("%s", sbuff);
                        }
                    }

                    prev_sample = curr_sample;
                    sample_num++;
                }
            }
        }
    }

    void wait_zeros(size_t n_zeros) {
        float rx_buff[MaxBufSize];

        size_t received_zeros = 0;

        while (received_zeros < n_zeros) {
            roc_frame frame;
            memset(&frame, 0, sizeof(frame));
            frame.samples = rx_buff;
            frame.samples_size = frame_samples_ * sizeof(float);

            const int ret = roc_receiver_read(recv_, &frame);
            roc_panic_if_not(ret == 0);

            bool has_non_zero = false;

            for (size_t ns = 0; ns < frame_samples_; ns++) {
                if (!is_zero_(rx_buff[ns])) {
                    has_non_zero = true;
                    break;
                }
            }

            if (has_non_zero) {
                received_zeros = 0;
            } else {
                received_zeros += frame_samples_;
            }
        }
    }

private:
    virtual void run() {
        receive();
    }

    static inline bool is_zero_(float s) {
        return fabs(double(s)) < 1e-9;
    }

    roc_receiver* recv_;

    core::Array<roc_endpoint*, 16> source_endp_;
    core::Array<roc_endpoint*, 16> repair_endp_;

    const float sample_step_;
    const size_t num_chans_;
    const size_t frame_samples_;
};

} // namespace test
} // namespace api
} // namespace roc

#endif // ROC_PUBLIC_API_TEST_HELPERS_RECEIVER_H_
