/*
 * Copyright (c) 2017 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "roc_audio/latency_tuner.h"
#include "roc_core/log.h"
#include "roc_core/panic.h"
#include "roc_core/time.h"

namespace roc {
namespace audio {

namespace {

const core::nanoseconds_t LogInterval = 5 * core::Second;

} // namespace

void LatencyConfig::deduce_defaults(core::nanoseconds_t default_target_latency,
                                    bool is_receiver) {
    // Deduce defaults for backend and profile.
    if (tuner_backend == LatencyTunerBackend_Default) {
        tuner_backend = LatencyTunerBackend_Niq;
    }

    if (tuner_profile == LatencyTunerProfile_Default) {
        if (is_receiver) {
            if (tuner_backend == LatencyTunerBackend_Niq) {
                // If latency is low, we assume network jitter is also low. In this
                // case we use responsive profile. Gradual profile could cause
                // oscillations comparable with the latency and break playback.
                //
                // If latency is high, we assume the jitter may be also high. In
                // this case use gradual profile because it can handle high jitter
                // much better.
                tuner_profile =
                    target_latency > 0 && target_latency < 30 * core::Millisecond
                    ? LatencyTunerProfile_Responsive
                    : LatencyTunerProfile_Gradual;
            } else {
                // E2E backend is not affected by network jitter that much, so
                // we can just always use responsive profile.
                tuner_profile = LatencyTunerProfile_Responsive;
            }
        } else {
            // On sender, by default disable latency tuning.
            // Typically latency tuning is done on receiver.
            tuner_profile = LatencyTunerProfile_Intact;
        }
    }

    // Deduce default target latency.
    if (target_latency == 0) {
        if (is_receiver) {
            if (tuner_profile != LatencyTunerProfile_Intact) {
                // Latency tuning is enabled on receiver.
                // Use default if target latency is not specified.
                target_latency = default_target_latency;
            } else {
                // Latency tuning is disabled on receiver.
                // Most likely, it is enabled on sender. To make tuning work on sender,
                // user should configure exactly same target latency on both sides.
                // To make this requirement clear, in this case we don't use any default
                // and require target latency to be specified explicitly on both sides.
                // So if target latency was not set, we assign an invalid value to it
                // to trigger config validation failure.
                target_latency = -1;
            }
        } else {
            if (tuner_profile != LatencyTunerProfile_Intact) {
                // See comment above.
                // If latency tuning is enabled on sender, we don't use any default
                // and require target latency to be specified explicitly.
                target_latency = -1;
            }
        }
    }

    // If latency tuning is enabled.
    if (tuner_profile != LatencyTunerProfile_Intact) {
        // Deduce defaults for min_latency & max_latency if both are zero.
        if (min_latency == 0 && max_latency == 0) {
            if (target_latency > 0) {
                // Out formula doesn't work well on latencies close to zero.
                const core::nanoseconds_t floored_target_latency =
                    std::max(target_latency, core::Millisecond);

                // On sender, apply multiplier to make default tolerance a bit higher than
                // on receiver. This way, if bounding is enabled on both sides, receiver
                // will always trigger first.
                const int multiplier = is_receiver ? 1 : 4;

                // This formula returns target_latency * N, where N starts with larger
                // number and approaches 0.5 as target_latency grows.
                // By default we're very tolerant and allow rather big oscillations.
                // Examples (for multiplier = 1):
                //  target=1ms -> tolerance=8ms (x8)
                //  target=10ms -> tolerance=20ms (x2)
                //  target=200ms -> tolerance=200ms (x1)
                //  target=2000ms -> tolerance=1444ms (x0.722)
                const core::nanoseconds_t latency_tolerance = core::nanoseconds_t(
                    floored_target_latency
                    * (std::log((200 * core::Millisecond) * 2 * multiplier)
                       / std::log(floored_target_latency * 2)));

                min_latency = target_latency - latency_tolerance;
                max_latency = target_latency + latency_tolerance;

                if (upper_threshold_coef == 0.f) {
                    upper_threshold_coef = (float)max_latency / (float)target_latency;
                }
            } else {
                // Can't deduce min_latency & max_latency without target_latency.
                min_latency = -1;
                min_latency = -1;
            }
        }

        // Deduce defaults for scaling_interval & scaling_tolerance.
        if (scaling_interval == 0) {
            scaling_interval = 5 * core::Millisecond;
        }
        if (scaling_tolerance == 0) {
            scaling_tolerance = 0.005f;
        }
    }

    // If latency bounding is enabled.
    if (min_latency != 0 || max_latency != 0) {
        // Deduce default for stale_tolerance.
        if (stale_tolerance == 0) {
            if (target_latency > 0) {
                // Consider queue "stalling" if at least 1/4 of the missing latency
                // is caused by lack of new packets.
                stale_tolerance = (target_latency - min_latency) / 4;
            } else {
                // Can't deduce stale_tolerance without target_latency.
                stale_tolerance = -1;
            }
        }
    }
}

LatencyTuner::LatencyTuner(const LatencyConfig& config,
                           const SampleSpec& sample_spec,
                           core::CsvDumper* dumper)
    : stream_pos_(0)
    , scale_interval_(0)
    , scale_pos_(0)
    , report_interval_(sample_spec.ns_2_stream_timestamp_delta(LogInterval))
    , report_pos_(0)
    , freq_coeff_(0)
    , freq_coeff_max_delta_(config.scaling_tolerance)
    , backend_(config.tuner_backend)
    , profile_(config.tuner_profile)
    , enable_tuning_(config.tuner_profile != audio::LatencyTunerProfile_Intact &&
                     config.target_latency == 0)
    , enable_bounds_(config.tuner_profile != audio::LatencyTunerProfile_Intact
                     || config.min_latency != 0 || config.max_latency != 0)
    , has_niq_latency_(false)
    , niq_latency_(0)
    , niq_stalling_(0)
    , has_e2e_latency_(false)
    , e2e_latency_(0)
    , has_metrics_(false)
    , target_latency_(0)
    , min_latency_(0)
    , max_latency_(0)
    , max_stalling_(0)
    , sample_spec_(sample_spec)
    , valid_(false)
    , target_latency_state_(TL_START)
    , last_target_latency_update_(0)
    , lat_update_upper_thrsh_(config.upper_threshold_coef)
    , lat_update_dec_step_(upper_coef_to_step_lat_update_(config.upper_threshold_coef))
    , lat_update_inc_step_(lower_thrs_to_step_lat_update_(config.upper_threshold_coef))
    , last_lat_limit_log_(0)
    , dumper_(dumper) {
    roc_log(LogDebug,
            "latency tuner: initializing:"
            " target_latency=%ld(%.3fms) start_latency=%ld(%.3fms)"
            " min_latency=%ld(%.3fms) max_latency=%ld(%.3fms)"
            " latency_upper_limit_coef=%f",
            (long)sample_spec_.ns_2_stream_timestamp_delta(config.target_latency),
            (double)config.target_latency / core::Millisecond,
            (long)sample_spec_.ns_2_stream_timestamp_delta(config.start_latency),
            (double)config.start_latency / core::Millisecond,
            (long)sample_spec_.ns_2_stream_timestamp_delta(config.min_latency),
            (double)config.min_latency / core::Millisecond,
            (long)sample_spec_.ns_2_stream_timestamp_delta(config.max_latency),
            (double)config.max_latency / core::Millisecond,
            (double)config.upper_threshold_coef);

    roc_log(LogDebug,
            "latency tuner: initializing:"
            " stale_tolerance=%ld(%.3fms)"
            " scaling_interval=%ld(%.3fms) scaling_tolerance=%f"
            " backend=%s profile=%s tuning=%s",
            (long)sample_spec_.ns_2_stream_timestamp_delta(config.stale_tolerance),
            (double)config.stale_tolerance / core::Millisecond,
            (long)sample_spec_.ns_2_stream_timestamp_delta(config.scaling_interval),
            (double)config.scaling_interval / core::Millisecond,
            (double)config.scaling_tolerance, latency_tuner_backend_to_str(backend_),
            latency_tuner_profile_to_str(profile_),
            enable_tuning_ ? "enabled" : "disabled");

    if (config.target_latency < 0) {
        roc_log(LogError,
                "latency tuner: invalid config:"
                " target_latency should not be negative");
        return;
    }

    if (config.start_latency < 0) {
        roc_log(LogError,
                "latency tuner: invalid config:"
                " start_latency should not be negative");
        return;
    }

    if (config.start_latency > 0 && config.target_latency > 0) {
        roc_log(LogError,
                "latency tuner: invalid config:"
                " start_latency and target_latency must not be positive altogether");
        return;
    }

    if (enable_bounds_ || enable_tuning_) {
        target_latency_ = sample_spec_.ns_2_stream_timestamp_delta(
                            config.target_latency == 0
                                ? config.start_latency
                                : config.target_latency);

        if (target_latency_ <= 0) {
            roc_log(LogError,
                    "latency tuner: invalid config: target latency is invalid:"
                    " start_latency=%ld(%.3fms), target_latency=%ld(%.3fms)",
                    (long)sample_spec_.ns_2_stream_timestamp_delta(config.start_latency),
                    (double)config.start_latency / core::Millisecond,
                    (long)sample_spec_.ns_2_stream_timestamp_delta(config.target_latency),
                    (double)config.start_latency / core::Millisecond);
            return;
        }

        if (enable_bounds_) {
            min_latency_ = sample_spec_.ns_2_stream_timestamp_delta(config.min_latency);
            max_latency_ = sample_spec_.ns_2_stream_timestamp_delta(config.max_latency);
            max_stalling_ =
                sample_spec_.ns_2_stream_timestamp_delta(config.stale_tolerance);

            if (target_latency_ < min_latency_ || target_latency_ > max_latency_) {
                roc_log(
                    LogError,
                    "latency tuner: invalid config: target_latency is out of bounds:"
                    " target_latency=%ld(%.3fms)"
                    " min_latency=%ld(%.3fms) max_latency=%ld(%.3fms)",
                    (long)sample_spec_.ns_2_stream_timestamp_delta(target_latency_),
                    (double)config.target_latency / core::Millisecond,
                    (long)sample_spec_.ns_2_stream_timestamp_delta(config.min_latency),
                    (double)config.min_latency / core::Millisecond,
                    (long)sample_spec_.ns_2_stream_timestamp_delta(config.max_latency),
                    (double)config.max_latency / core::Millisecond);
                return;
            } else if ((float)target_latency_ * lat_update_upper_thrsh_
                           > (float)max_latency_
                       && enable_tuning_) {
                roc_log(
                    LogError,
                    "latency tuner: invalid config: upper threshold coefficient is"
                    " out of bounds: "
                    " target_latency * %f = %ld(%.3fms)"
                    " min_latency=%ld(%.3fms) max_latency=%ld(%.3fms)",
                    (double)lat_update_upper_thrsh_,
                    (long)sample_spec_.ns_2_stream_timestamp_delta(
                        (packet::stream_source_t)(target_latency_
                                                  * lat_update_upper_thrsh_)),
                    (double)(target_latency_ * lat_update_upper_thrsh_)
                        / core::Millisecond,
                    (long)sample_spec_.ns_2_stream_timestamp_delta(config.min_latency),
                    (double)config.min_latency / core::Millisecond,
                    (long)sample_spec_.ns_2_stream_timestamp_delta(config.max_latency),
                    (double)config.max_latency / core::Millisecond);
            }
        }

        if (enable_tuning_) {
            scale_interval_ =
                sample_spec_.ns_2_stream_timestamp_delta(config.scaling_interval);

            if (config.scaling_interval <= 0 || scale_interval_ <= 0) {
                roc_log(
                    LogError,
                    "latency tuner: invalid config: scaling_interval is out of bounds:"
                    " scaling_interval=%ld(%.3fms)",
                    (long)sample_spec_.ns_2_stream_timestamp_delta(
                        config.scaling_interval),
                    (double)config.scaling_interval / core::Millisecond);
                return;
            }

            if (config.scaling_tolerance <= 0) {
                roc_log(
                    LogError,
                    "latency tuner: invalid config: scaling_tolerance is out of bounds:"
                    " scaling_tolerance=%f",
                    (double)config.scaling_tolerance);
                return;
            }

            if (config.upper_threshold_coef < 0) {
                roc_log(LogError,
                        "latency tuner: invalid config: upper threshold coef is negative:"
                        " upper_threshold_coef=%f",
                        (double)config.upper_threshold_coef);
            }

            fe_.reset(new (fe_) FreqEstimator(profile_ == LatencyTunerProfile_Responsive
                                                  ? FreqEstimatorProfile_Responsive
                                                  : FreqEstimatorProfile_Gradual,
                                              (packet::stream_timestamp_t)target_latency_,
                                              dumper_));
            if (!fe_) {
                return;
            }
        }
    }

    valid_ = true;
}

bool LatencyTuner::is_valid() const {
    return valid_;
}

void LatencyTuner::write_metrics(const LatencyMetrics& latency_metrics,
                                 const packet::LinkMetrics& link_metrics) {
    roc_panic_if(!is_valid());

    if (latency_metrics.niq_latency > 0 || latency_metrics.niq_stalling > 0
        || has_niq_latency_) {
        niq_latency_ =
            sample_spec_.ns_2_stream_timestamp_delta(latency_metrics.niq_latency);
        niq_stalling_ =
            sample_spec_.ns_2_stream_timestamp_delta(latency_metrics.niq_stalling);
        has_niq_latency_ = true;
    }

    if (latency_metrics.e2e_latency > 0 || has_e2e_latency_) {
        e2e_latency_ =
            sample_spec_.ns_2_stream_timestamp_delta(latency_metrics.e2e_latency);
        has_e2e_latency_ = true;
    }

    if (enable_tuning_) {
        update_target_latency_(link_metrics.max_jitter, link_metrics.jitter,
                               latency_metrics.fec_block_duration);
    }

    if (dumper_) {
        core::CsvEntry e;
        e.type = 't';
        e.n_fields = 3;
        e.fields[0] = core::timestamp(core::ClockUnix);
        e.fields[1] = niq_latency_;
        e.fields[2] = target_latency_;
        dumper_->write(e);
    }

    latency_metrics_ = latency_metrics;
    link_metrics_ = link_metrics;
    has_metrics_ = true;
}

bool LatencyTuner::update_stream() {
    roc_panic_if(!is_valid());

    packet::stream_timestamp_diff_t latency = 0;

    switch (backend_) {
    case audio::LatencyTunerBackend_Niq:
        if (!has_niq_latency_) {
            return true;
        }
        latency = niq_latency_;
        break;

    case audio::LatencyTunerBackend_E2e:
        if (!has_e2e_latency_) {
            return true;
        }
        latency = e2e_latency_;
        break;

    default:
        break;
    }

    if (enable_bounds_) {
        if (!check_bounds_(latency)) {
            return false;
        }
    }

    if (enable_tuning_) {
        compute_scaling_(latency);
    }

    return true;
}

void LatencyTuner::advance_stream(packet::stream_timestamp_t duration) {
    roc_panic_if(!is_valid());

    if (last_target_latency_update_ == 0) {
        last_target_latency_update_ = core::timestamp(core::ClockMonotonic);
    }

    stream_pos_ += duration;

    report_();
}

float LatencyTuner::get_scaling() const {
    roc_panic_if(!is_valid());

    return freq_coeff_;
}

bool LatencyTuner::check_bounds_(const packet::stream_timestamp_diff_t latency) {
    // Queue is considered "stalling" if there were no new packets for
    // some period of time.
    const bool is_stalling = backend_ == audio::LatencyTunerBackend_Niq
        && niq_stalling_ > max_stalling_ && max_stalling_ > 0;

    if (latency < min_latency_ && is_stalling) {
        // There are two possible reasons why queue latency becomes lower than minimum:
        //  1. either we were not able to compensate clock drift (or compensation is
        //     disabled) and queue slowly exhausted,
        //  2. or there is a burst packet delay or drop, which led to sharp decrease
        //     of the latency.
        //
        // In the first case we normally want to terminate/restart session, but the
        // second case is often not a big deal. If the burst is short, pipeline
        // can easily recover from it, and terminating session would be worse.
        // In this case, we want to keep things as is and leave decision to the
        // watchdog. If the burst was short, watchdog will keep session, otherwise
        // no_playback_timeout will trigger and watchdog will terminate session.
        //
        // To distinguish between the cases, we check network queue stalling metric,
        // which shows delay since last received packet. If there were no packets
        // during notable amount of time, we assume that the second case takes place.
        return true;
    }

    if (latency < min_latency_ || latency > max_latency_) {
        roc_log(
            LogDebug,
            "latency tuner: latency out of bounds:"
            " latency=%ld(%.3fms) target=%ld(%.3fms)"
            " min=%ld(%.3fms) max=%ld(%.3fms) stale=%ld(%.3fms)",
            (long)latency, sample_spec_.stream_timestamp_delta_2_ms(latency),
            (long)target_latency_,
            sample_spec_.stream_timestamp_delta_2_ms(target_latency_), (long)min_latency_,
            sample_spec_.stream_timestamp_delta_2_ms(min_latency_), (long)max_latency_,
            sample_spec_.stream_timestamp_delta_2_ms(max_latency_), (long)niq_stalling_,
            sample_spec_.stream_timestamp_delta_2_ms(niq_stalling_));
        return false;
    }

    return true;
}

void LatencyTuner::compute_scaling_(packet::stream_timestamp_diff_t latency) {
    if (latency < 0) {
        latency = 0;
    }

    if (stream_pos_ < scale_pos_) {
        return;
    }

    while (stream_pos_ >= scale_pos_) {
        fe_->update((packet::stream_timestamp_t)latency);
        scale_pos_ += (packet::stream_timestamp_t)scale_interval_;
    }

    freq_coeff_ = fe_->freq_coeff();
    freq_coeff_ = std::min(freq_coeff_, 1.0f + freq_coeff_max_delta_);
    freq_coeff_ = std::max(freq_coeff_, 1.0f - freq_coeff_max_delta_);
}

void LatencyTuner::report_() {
    if (stream_pos_ < report_pos_) {
        return;
    }

    while (stream_pos_ >= report_pos_) {
        report_pos_ += (packet::stream_timestamp_t)report_interval_;
    }

    roc_log(LogInfo,
            "latency tuner:"
            " e2e_latency=%ld(%.3fms) niq_latency=%ld(%.3fms) target_latency=%ld(%.3fms)"
            " jitter=%.3fms stale=%ld(%.3fms)"
            " fe=%.6f eff_fe=%.6f fe_stable=%s",
            (long)e2e_latency_, sample_spec_.stream_timestamp_delta_2_ms(e2e_latency_),
            (long)niq_latency_, sample_spec_.stream_timestamp_delta_2_ms(niq_latency_),
            (long)target_latency_,
            sample_spec_.stream_timestamp_delta_2_ms(target_latency_),
            (double)link_metrics_.jitter / core::Millisecond, (long)niq_stalling_,
            sample_spec_.stream_timestamp_delta_2_ms(niq_stalling_),
            (double)(fe_ && freq_coeff_ > 0 ? fe_->freq_coeff() : 0), (double)freq_coeff_,
            fe_ && fe_->stable() ? "true" : "false");

    if (has_metrics_) {
        roc_log(LogDebug,
                "Latency monitor Link Metrics:"
                " cum_loss=%ld jitter=%.1fms"
                " running_jitter(Max/Min)=%.1f/%.1fms"
                " total_packets=%ld",
                link_metrics_.lost_packets,
                (double)link_metrics_.jitter / core::Millisecond,
                (double)link_metrics_.max_jitter / core::Millisecond,
                (double)link_metrics_.min_jitter / core::Millisecond,
                link_metrics_.total_packets);
        roc_log(LogDebug, "Latency monitor: fec block duration=%.1fms",
                (double)latency_metrics_.fec_block_duration / core::Millisecond);
    }

    if (sample_spec_.ns_2_stream_timestamp_delta(latency_metrics_.fec_block_duration)
        >= max_latency_) {
        roc_log(LogInfo,
                "Latency tuner: FEC block %.1fms is longer than the max "
                "limit for latency %d(%.1fms)",
                (double)latency_metrics_.fec_block_duration / core::Millisecond,
                max_latency_, (double)max_latency_ / core::Millisecond);
    }
}

// Decides if the latency should be adjusted and orders fe_ to do so if needed.
//
// 1. Decides to decrease latency if current value is greater than upper threshold,
//    The target latency is supposed to change smoothely, so we just cut the current
//    latency value by some percentage.
//
// 2. Decides to increase latency if it is lesser than lower threshold (which
//    could be close or equal to target latency itself).
//    This could/should be done effectively as it could possibly mean that the user
//    is already perceives some losses.
//
//    NB: After the increasement the new latency target value must not be greater than
//        upper threshold in any circumstances.
//
//
void LatencyTuner::update_target_latency_(const core::nanoseconds_t max_jitter_ns,
                                          const core::nanoseconds_t mean_jitter_ns,
                                          const core::nanoseconds_t fec_block_ns) {
    const core::nanoseconds_t estimate = std::max(
        std::max((core::nanoseconds_t)(max_jitter_ns * 1.15), mean_jitter_ns * 3),
        fec_block_ns);
    const core::nanoseconds_t now = core::timestamp(core::ClockMonotonic);
    core::nanoseconds_t cur_tl_ns =
        sample_spec_.stream_timestamp_delta_2_ns(target_latency_);

    if (target_latency_state_ == TL_NONE) {
        // If there is no active timeout, check if evaluated target latency is
        // significantly smaller, than the latency in action so that we could decrease it.
        if (estimate < cur_tl_ns && estimate * lat_update_upper_thrsh_ < cur_tl_ns
            && fe_->stable()) {
            const core::nanoseconds_t new_tl_ns =
                (core::nanoseconds_t)(cur_tl_ns * lat_update_dec_step_);
            const packet::stream_timestamp_diff_t new_tl_ts =
                sample_spec_.ns_2_stream_timestamp_delta(new_tl_ns);
            if (new_tl_ts < min_latency_) {
                if (now > last_lat_limit_log_
                    && (now - last_lat_limit_log_) > 5 * core::Second) {
                    last_lat_limit_log_ = now;
                    roc_log(LogDebug,
                            "Latency tuner:"
                            " not decreasing target latency lower than limit %ld(%.3fms)",
                            (long)min_latency_,
                            sample_spec_.stream_timestamp_delta_2_ms(min_latency_));
                }
                return;
            }

            roc_log(LogInfo,
                    "Latency monitor:"
                    " decreasing target latency %ld(%.3fms) → %ld(%.3fms)",
                    (long)target_latency_, (double)cur_tl_ns / core::Millisecond,
                    (long)new_tl_ts, (double)new_tl_ns / core::Millisecond);
            roc_log(LogDebug,
                    "Latency monitor:"
                    "\testimate %.3fms * %.3f = %.3fms,\tnew tl  %.3fms * %f = %.3fms",
                    (double)estimate / core::Millisecond, (double)lat_update_upper_thrsh_,
                    (double)estimate * (double)lat_update_upper_thrsh_
                        / core::Millisecond,
                    (double)cur_tl_ns / core::Millisecond, (double)lat_update_dec_step_,
                    (double)(new_tl_ns / core::Millisecond));

            cur_tl_ns = new_tl_ns;
            target_latency_ = new_tl_ts;
            last_target_latency_update_ = now;
            target_latency_state_ = TL_DEC_TIMEOUT;
            fe_->update_target_latency((packet::stream_timestamp_t)target_latency_);

            return;
            // If evaluated target latency is greater, than we must increase it.
        } else if (estimate > cur_tl_ns) {
            const core::nanoseconds_t new_tl_ns =
                (core::nanoseconds_t)(estimate * lat_update_inc_step_);
            packet::stream_timestamp_diff_t new_tl_ts =
                sample_spec_.ns_2_stream_timestamp_delta(new_tl_ns);

            if (new_tl_ts > max_latency_) {
                if (now > last_lat_limit_log_
                    && (now - last_lat_limit_log_) > 5 * core::Second) {
                    last_lat_limit_log_ = now;

                    roc_log(LogDebug,
                            "Latency tuner:"
                            " capping target latency %ld(%.3fms)"
                            " as max limit is lower %ld(%.3fms)",
                            (long)new_tl_ts, (double)new_tl_ns / core::Millisecond,
                            (long)max_latency_,
                            sample_spec_.stream_timestamp_delta_2_ms(max_latency_));
                }
                new_tl_ts = max_latency_;
            }

            roc_log(LogInfo,
                    "Latency tuner:"
                    " increasing target latency %ld(%.3fms) → %ld(%.3fms)",
                    (long)target_latency_, (double)cur_tl_ns / core::Millisecond,
                    (long)new_tl_ts, (double)new_tl_ns / core::Millisecond);

            target_latency_ = new_tl_ts;
            last_target_latency_update_ = now;
            target_latency_state_ = TL_INC_TIMEOUT;
            fe_->update_target_latency((packet::stream_timestamp_t)target_latency_);

            return;
        }
        // Waiting the timeout since last decreasement.
    } else if (target_latency_state_ == TL_DEC_TIMEOUT
               && now - last_target_latency_update_ > 15 * core::Second) {
        target_latency_state_ = TL_NONE;
        // Waiting the timeout since the startup.
    } else if (target_latency_state_ == TL_START
               && (last_target_latency_update_ == 0
                   || (now - last_target_latency_update_ > 5 * core::Second))) {
        target_latency_state_ = TL_NONE;

        // Waiting the timeout since the last increasement.
    } else if (target_latency_state_ == TL_INC_TIMEOUT
               && now - last_target_latency_update_ > 5 * core::Second) {
        target_latency_state_ = TL_NONE;
    }
}

// Calculates latency decreasment step value such that
// if current latency equals exactly upper threshold value,
// after the decreasment it will get in the middle between threshold and estimated
// value.
float LatencyTuner::upper_coef_to_step_lat_update_(const float x) {
    return ((x + 1.f) / (x * 2.f));
}

// Calculates latency increasment step value based on
// upper_threshold_coef.
float LatencyTuner::lower_thrs_to_step_lat_update_(const float x) {
    return (x + 1.f) / 2.f;
}

const char* latency_tuner_backend_to_str(LatencyTunerBackend backend) {
    switch (backend) {
    case LatencyTunerBackend_Default:
        return "default";

    case LatencyTunerBackend_Niq:
        return "niq";

    case LatencyTunerBackend_E2e:
        return "e2e";
    }

    return "<invalid>";
}

const char* latency_tuner_profile_to_str(LatencyTunerProfile profile) {
    switch (profile) {
    case LatencyTunerProfile_Default:
        return "default";

    case LatencyTunerProfile_Intact:
        return "intact";

    case LatencyTunerProfile_Responsive:
        return "responsive";

    case LatencyTunerProfile_Gradual:
        return "gradual";
    }

    return "<invalid>";
}

} // namespace audio
} // namespace roc
