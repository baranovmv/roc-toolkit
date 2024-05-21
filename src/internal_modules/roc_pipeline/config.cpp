/*
 * Copyright (c) 2024 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "roc_pipeline/config.h"
#include "roc_rtp/headers.h"

namespace roc {
namespace pipeline {

SenderSinkConfig::SenderSinkConfig()
    : input_sample_spec(DefaultSampleSpec)
    , payload_type(rtp::PayloadType_L16_Stereo)
    , packet_length(DefaultPacketLength)
    , enable_timing(false)
    , enable_auto_duration(false)
    , enable_auto_cts(false)
    , enable_profiling(false)
    , enable_interleaving(false) {
}

void SenderSinkConfig::deduce_defaults() {
    latency.deduce_defaults(DefaultLatency, false);
    resampler.deduce_defaults(latency.tuner_backend, latency.tuner_profile);
}

SenderSlotConfig::SenderSlotConfig() {
}

void SenderSlotConfig::deduce_defaults() {
}

ReceiverCommonConfig::ReceiverCommonConfig()
    : output_sample_spec(DefaultSampleSpec)
    , enable_timing(false)
    , enable_auto_reclock(false)
    , enable_profiling(false) {
}

void ReceiverCommonConfig::deduce_defaults() {
}

ReceiverSessionConfig::ReceiverSessionConfig()
    : payload_type(0)
    , enable_beeping(false) {
}

void ReceiverSessionConfig::deduce_defaults() {
    latency.deduce_defaults(DefaultLatency, true);
    watchdog.deduce_defaults(latency.target_latency);
    resampler.deduce_defaults(latency.tuner_backend, latency.tuner_profile);
}

ReceiverSourceConfig::ReceiverSourceConfig()
    : dump_file(NULL) {
}

void ReceiverSourceConfig::deduce_defaults() {
    common.deduce_defaults();
    session_defaults.deduce_defaults();
}

ReceiverSlotConfig::ReceiverSlotConfig()
    : enable_routing(true) {
}

void ReceiverSlotConfig::deduce_defaults() {
}

TranscoderConfig::TranscoderConfig()
    : input_sample_spec(DefaultSampleSpec)
    , output_sample_spec(DefaultSampleSpec)
    , enable_profiling(false) {
}

void TranscoderConfig::deduce_defaults() {
    resampler.deduce_defaults(audio::LatencyTunerBackend_Default,
                              audio::LatencyTunerProfile_Default);
}

} // namespace pipeline
} // namespace roc
