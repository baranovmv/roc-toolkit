/*
 * Copyright (c) 2024 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

//! @file roc_audio/feedback_monitor.h
//! @brief Feedback monitor.

#ifndef ROC_AUDIO_FEEDBACK_MONITOR_H_
#define ROC_AUDIO_FEEDBACK_MONITOR_H_

#include "roc_audio/iframe_writer.h"
#include "roc_audio/latency_tuner.h"
#include "roc_audio/resampler_writer.h"
#include "roc_audio/sample_spec.h"
#include "roc_core/noncopyable.h"
#include "roc_core/rate_limiter.h"
#include "roc_core/time.h"

namespace roc {
namespace audio {

//! Feedback monitor configuration.
struct FeedbackConfig {
    //! Timeout for source feedback.
    //! If there is no new feedback during timeout, feedback monitor resets state.
    core::nanoseconds_t source_timeout;

    //! Cooldown period between source changes.
    //! After source is change, another source change is now allowed during
    //! this period and is ignored.
    core::nanoseconds_t source_cooldown;

    FeedbackConfig()
        : source_timeout(1500 * core::Millisecond)
        , source_cooldown(50 * core::Millisecond) {
    }
};

//! Feedback monitor.
//!
//! @b Features
//!
//!  - handles latency metrics from receiver (obtained via RTCP)
//!  - asks LatencyTuner to calculate scaling factor based on the actual and
//!    target latencies
//!  - passes calculated scaling factor to resampler
//!
//! @b Flow
//!
//!  - when pipeline obtains RTCP report, it calls write_metrics() method
//!  - pipeline periodically calls write() method; it passes latest metrics
//!    to LatencyTuner, and obtains scaling factor for resampler
//!  - feedback monitor has a reference to resampler, and periodically passes
//!    updated scaling factor to it
//!  - pipeline also can query feedback monitor for latency metrics on behalf of
//!    request from user
class FeedbackMonitor : public IFrameWriter, public core::NonCopyable<> {
public:
    //! Constructor.
    FeedbackMonitor(IFrameWriter& writer,
                    ResamplerWriter* resampler,
                    const FeedbackConfig& feedback_config,
                    const LatencyConfig& latency_config,
                    const SampleSpec& sample_spec);

    //! Check if the object was initialized successfully.
    bool is_valid() const;

    //! Check if feedback monitoring is started.
    bool is_started() const;

    //! Enable feedback monitoring.
    void start();

    //! Process feedback from receiver.
    void process_feedback(packet::stream_source_t source_id,
                          const LatencyMetrics& metrics);

    //! Write audio frame.
    //! Passes frame to underlying writer.
    //! If feedback monitoring is started, also performs latency tuning.
    virtual void write(Frame& frame);

    //! Get back latest metrics.
    LatencyMetrics metrics() const;

private:
    bool update_tuner_(packet::stream_timestamp_t duration);

    bool init_scaling_();
    bool update_scaling_();

    LatencyTuner tuner_;

    LatencyMetrics metrics_;
    bool has_metrics_;

    const core::nanoseconds_t metrics_timeout_;
    core::nanoseconds_t metrics_ts_;

    IFrameWriter& writer_;

    ResamplerWriter* resampler_;
    const bool enable_scaling_;

    packet::stream_source_t source_;
    core::RateLimiter source_change_limiter_;

    const SampleSpec sample_spec_;

    bool started_;
    bool valid_;
};

} // namespace audio
} // namespace roc

#endif // ROC_AUDIO_FEEDBACK_MONITOR_H_
