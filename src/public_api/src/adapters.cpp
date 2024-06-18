/*
 * Copyright (c) 2017 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "adapters.h"

#include "roc_address/interface.h"
#include "roc_audio/channel_defs.h"
#include "roc_audio/freq_estimator.h"
#include "roc_audio/pcm_format.h"
#include "roc_audio/resampler_config.h"
#include "roc_core/attributes.h"
#include "roc_core/log.h"

namespace roc {
namespace api {

// Note: ROC_ATTR_NO_SANITIZE_UB is used from *_from_user() functions because we don't
// want sanitizers to fail us when enums contain arbitrary values; we correctly handle
// all these cases.

#ifdef __clang__
// On clang, we use switches with original enum value. This allows compiler to warn us
// if we forget to list a value.
template <class T> ROC_ATTR_NO_SANITIZE_UB T enum_from_user(T t) {
    return t;
}
#else
// On other compilers, we use switches with enum value casted to int. This prevents
// some broken compiler versions (e.g. older gcc) to optimize out the code after switch
// which corresponds to unmatched value. Clang does not have this problem, so we don't
// use this hack on it to benefit from the warnings.
template <class T> ROC_ATTR_NO_SANITIZE_UB int enum_from_user(T t) {
    return t;
}
#endif

ROC_ATTR_NO_SANITIZE_UB
bool context_config_from_user(node::ContextConfig& out, const roc_context_config& in) {
    if (in.max_packet_size != 0) {
        out.max_packet_size = in.max_packet_size;
    }

    if (in.max_frame_size != 0) {
        out.max_frame_size = in.max_frame_size;
    }

    return true;
}

ROC_ATTR_NO_SANITIZE_UB
bool sender_config_from_user(node::Context& context,
                             pipeline::SenderSinkConfig& out,
                             const roc_sender_config& in) {
    if (!sample_spec_from_user(out.input_sample_spec, in.frame_encoding, false)) {
        roc_log(LogError, "bad configuration: invalid roc_sender_config.frame_encoding");
        return false;
    }

    if (in.packet_encoding != 0) {
        if (!packet_encoding_from_user(out.payload_type, in.packet_encoding)) {
            roc_log(LogError,
                    "bad configuration: invalid roc_sender_config.packet_encoding:"
                    " should be zero or valid encoding id");
            return false;
        }
        const rtp::Encoding* encoding =
            context.encoding_map().find_by_pt(out.payload_type);
        if (!encoding) {
            roc_log(LogError,
                    "bad configuration: invalid roc_sender_config.packet_encoding:"
                    " no built-in or registered encoding found with id %u",
                    (unsigned)out.payload_type);
            return false;
        }
    } else {
        const rtp::Encoding* encoding =
            context.encoding_map().find_by_spec(out.input_sample_spec);
        if (!encoding) {
            roc_log(LogError,
                    "bad configuration:"
                    " failed to select packet_encoding matching frame_encoding,"
                    " set roc_sender_config.packet_encoding manually");
            return false;
        }
        out.payload_type = encoding->payload_type;
    }

    if (in.packet_length != 0) {
        out.packet_length = (core::nanoseconds_t)in.packet_length;
    }

    if (in.target_latency != 0) {
        out.latency.target_latency = (core::nanoseconds_t)in.target_latency;
    }

    if (in.min_latency != 0) {
        out.latency.min_latency = (core::nanoseconds_t)in.min_latency;
    }

    if (in.max_latency != 0) {
        out.latency.max_latency = (core::nanoseconds_t)in.max_latency;
    }

    out.enable_timing = false;
    out.enable_auto_duration = true;
    out.enable_auto_cts = true;

    out.enable_interleaving = in.packet_interleaving;

    if (!fec_encoding_from_user(out.fec_encoder.scheme, in.fec_encoding)) {
        roc_log(LogError,
                "bad configuration: invalid roc_sender_config.fec_encoding:"
                " should be valid enum value");
        return false;
    }

    if (in.fec_block_source_packets != 0 || in.fec_block_repair_packets != 0) {
        out.fec_writer.n_source_packets = in.fec_block_source_packets;
        out.fec_writer.n_repair_packets = in.fec_block_repair_packets;
    }

    if (!clock_source_from_user(out.enable_timing, in.clock_source)) {
        roc_log(LogError,
                "bad configuration: invalid roc_sender_config.clock_source:"
                " should be valid enum value");
        return false;
    }

    if (!latency_tuner_backend_from_user(out.latency.tuner_backend,
                                         in.latency_tuner_backend)) {
        roc_log(LogError,
                "bad configuration: invalid roc_sender_config.latency_tuner_backend:"
                " should be valid enum value");
        return false;
    }

    if (!latency_tuner_profile_from_user(out.latency.tuner_profile,
                                         in.latency_tuner_profile)) {
        roc_log(LogError,
                "bad configuration: invalid roc_sender_config.latency_tuner_profile:"
                " should be valid enum value");
        return false;
    }

    if (!resampler_backend_from_user(out.resampler.backend, in.resampler_backend)) {
        roc_log(LogError,
                "bad configuration: invalid roc_sender_config.resampler_backend:"
                " should be valid enum value");
        return false;
    }

    if (!resampler_profile_from_user(out.resampler.profile, in.resampler_profile)) {
        roc_log(LogError,
                "bad configuration: invalid roc_sender_config.resampler_profile:"
                " should be valid enum value");
        return false;
    }

    return true;
}

ROC_ATTR_NO_SANITIZE_UB
bool receiver_config_from_user(node::Context&,
                               pipeline::ReceiverSourceConfig& out,
                               const roc_receiver_config& in) {
    if (in.target_latency != 0) {
        out.session_defaults.latency.target_latency =
            (core::nanoseconds_t)in.target_latency;
    }

    if (in.start_latency != 0) {
        if (in.target_latency != 0) {
            roc_log(LogError,
                    "bad configuration:"
                    " start latency must be 0 if latency tuning is disabled"
                    " (target_latency != 0)");
            return false;
        }
        out.session_defaults.latency.start_latency =
            (core::nanoseconds_t)in.start_latency;
    }

    if (in.min_latency != 0) {
        out.session_defaults.latency.min_latency = (core::nanoseconds_t)in.min_latency;
    }

    if (in.max_latency != 0) {
        out.session_defaults.latency.max_latency = (core::nanoseconds_t)in.max_latency;
    }

    if (in.no_playback_timeout != 0) {
        out.session_defaults.watchdog.no_playback_timeout = in.no_playback_timeout;
    }

    if (in.choppy_playback_timeout != 0) {
        out.session_defaults.watchdog.choppy_playback_timeout =
            in.choppy_playback_timeout;
    }

    out.common.enable_timing = false;
    out.common.enable_auto_reclock = true;

    if (!sample_spec_from_user(out.common.output_sample_spec, in.frame_encoding, false)) {
        roc_log(LogError,
                "bad configuration: invalid roc_receiver_config.frame_encoding");
        return false;
    }

    if (!clock_source_from_user(out.common.enable_timing, in.clock_source)) {
        roc_log(LogError,
                "bad configuration: invalid roc_receiver_config.clock_source:"
                " should be valid enum value");
        return false;
    }

    if (!latency_tuner_backend_from_user(out.session_defaults.latency.tuner_backend,
                                         in.latency_tuner_backend)) {
        roc_log(LogError,
                "bad configuration: invalid roc_receiver_config.latency_tuner_backend:"
                " should be valid enum value");
        return false;
    }

    if (!latency_tuner_profile_from_user(out.session_defaults.latency.tuner_profile,
                                         in.latency_tuner_profile)) {
        roc_log(LogError,
                "bad configuration: invalid roc_receiver_config.latency_tuner_profile:"
                " should be valid enum value");
        return false;
    }

    if (!resampler_backend_from_user(out.session_defaults.resampler.backend,
                                     in.resampler_backend)) {
        roc_log(LogError,
                "bad configuration: invalid roc_receiver_config.resampler_backend:"
                " should be valid enum value");
        return false;
    }

    if (!resampler_profile_from_user(out.session_defaults.resampler.profile,
                                     in.resampler_profile)) {
        roc_log(LogError,
                "bad configuration: invalid roc_receiver_config.resampler_profile:"
                " should be valid enum value");
        return false;
    }

    return true;
}

ROC_ATTR_NO_SANITIZE_UB bool interface_config_from_user(netio::UdpConfig& out,
                                                        const roc_interface_config& in) {
    if (in.outgoing_address[0] != '\0') {
        if (!out.bind_address.set_host_port_auto(in.outgoing_address, 0)) {
            roc_log(LogError,
                    "bad configuration: invalid roc_interface_config.outgoing_address:"
                    " should be either empty or valid IPv4/IPv6 address");
            return false;
        }
    }

    if (in.multicast_group[0] != '\0') {
        if (strlen(in.multicast_group) >= sizeof(out.multicast_interface)) {
            roc_log(LogError,
                    "bad configuration: invalid roc_interface_config.multicast_group:"
                    " should be no longer than %d characters",
                    (int)sizeof(out.multicast_interface) - 1);
            return false;
        }

        address::SocketAddr addr;
        if (!addr.set_host_port_auto(in.multicast_group, 0)) {
            roc_log(LogError,
                    "bad configuration: invalid roc_interface_config.multicast_group:"
                    " should be either empty or valid IPv4/IPv6 address");
            return false;
        }

        strcpy(out.multicast_interface, in.multicast_group);
    }

    out.enable_reuseaddr = (in.reuse_address != 0);

    return true;
}

ROC_ATTR_NO_SANITIZE_UB
bool sample_spec_from_user(audio::SampleSpec& out,
                           const roc_media_encoding& in,
                           bool is_network) {
    if (in.rate != 0) {
        out.set_sample_rate(in.rate);
    } else {
        roc_log(LogError,
                "bad configuration: invalid roc_media_encoding.rate:"
                " should be non-zero");
        return false;
    }

    if (!sample_format_from_user(out, in.format, is_network)) {
        roc_log(LogError,
                "bad configuration: invalid roc_media_encoding.format:"
                " should be valid enum value");
        return false;
    }

    if (in.channels != 0) {
        if (in.channels == ROC_CHANNEL_LAYOUT_MULTITRACK) {
            if (in.tracks == 0) {
                roc_log(LogError,
                        "bad configuration: invalid roc_media_encoding:"
                        " if channels is ROC_CHANNEL_LAYOUT_MULTITRACK,"
                        " then tracks should be non-zero");
                return false;
            }
            if (in.tracks > audio::ChannelSet::max_channels()) {
                roc_log(LogError,
                        "bad configuration: invalid roc_media_encoding:"
                        " invalid tracks count: got=%u expected=[1;%u]",
                        (unsigned)in.tracks, (unsigned)audio::ChannelSet::max_channels());
                return false;
            }
        } else {
            if (in.tracks != 0) {
                roc_log(LogError,
                        "bad configuration: invalid roc_media_encoding:"
                        " if channels is not ROC_CHANNEL_LAYOUT_MULTITRACK,"
                        " then tracks should be zero");
                return false;
            }
        }
        if (!channel_set_from_user(out.channel_set(), in.channels, in.tracks)) {
            roc_log(LogError,
                    "bad configuration: invalid roc_media_encoding.channels:"
                    " should be valid enum value");
            return false;
        }
    } else {
        roc_log(LogError,
                "bad configuration: invalid roc_media_encoding.channels:"
                " should be non-zero");
        return false;
    }

    return true;
}

ROC_ATTR_NO_SANITIZE_UB
bool sample_format_from_user(audio::SampleSpec& out, roc_format in, bool is_network) {
    switch (enum_from_user(in)) {
    case ROC_FORMAT_PCM_FLOAT32:
        out.set_sample_format(audio::SampleFormat_Pcm);
        // TODO(gh-608): use PcmFormat_Float32_Be instead of PcmFormat_SInt16_Be
        out.set_pcm_format(is_network ? audio::PcmFormat_SInt16_Be
                                      : audio::PcmFormat_Float32);
        return true;
    }

    return false;
}

ROC_ATTR_NO_SANITIZE_UB
bool channel_set_from_user(audio::ChannelSet& out,
                           roc_channel_layout in,
                           unsigned int in_tracks) {
    out.clear();

    switch (enum_from_user(in)) {
    case ROC_CHANNEL_LAYOUT_MULTITRACK:
        out.set_layout(audio::ChanLayout_Multitrack);
        out.set_order(audio::ChanOrder_None);
        out.set_range(0, in_tracks - 1);
        return true;

    case ROC_CHANNEL_LAYOUT_MONO:
        out.set_layout(audio::ChanLayout_Surround);
        out.set_order(audio::ChanOrder_Smpte);
        out.set_mask(audio::ChanMask_Surround_Mono);
        return true;

    case ROC_CHANNEL_LAYOUT_STEREO:
        out.set_layout(audio::ChanLayout_Surround);
        out.set_order(audio::ChanOrder_Smpte);
        out.set_mask(audio::ChanMask_Surround_Stereo);
        return true;
    }

    return false;
}

ROC_ATTR_NO_SANITIZE_UB
bool clock_source_from_user(bool& out_timing, roc_clock_source in) {
    switch (enum_from_user(in)) {
    case ROC_CLOCK_SOURCE_DEFAULT:
    case ROC_CLOCK_SOURCE_EXTERNAL:
        out_timing = false;
        return true;

    case ROC_CLOCK_SOURCE_INTERNAL:
        out_timing = true;
        return true;
    }

    return false;
}

ROC_ATTR_NO_SANITIZE_UB
bool latency_tuner_backend_from_user(audio::LatencyTunerBackend& out,
                                     roc_latency_tuner_backend in) {
    switch (enum_from_user(in)) {
    case ROC_LATENCY_TUNER_BACKEND_DEFAULT:
        out = audio::LatencyTunerBackend_Default;
        return true;

    case ROC_LATENCY_TUNER_BACKEND_NIQ:
        out = audio::LatencyTunerBackend_Niq;
        return true;
    }

    return false;
}

ROC_ATTR_NO_SANITIZE_UB
bool latency_tuner_profile_from_user(audio::LatencyTunerProfile& out,
                                     roc_latency_tuner_profile in) {
    switch (enum_from_user(in)) {
    case ROC_LATENCY_TUNER_PROFILE_DEFAULT:
        out = audio::LatencyTunerProfile_Default;
        return true;

    case ROC_LATENCY_TUNER_PROFILE_INTACT:
        out = audio::LatencyTunerProfile_Intact;
        return true;

    case ROC_LATENCY_TUNER_PROFILE_RESPONSIVE:
        out = audio::LatencyTunerProfile_Responsive;
        return true;

    case ROC_LATENCY_TUNER_PROFILE_GRADUAL:
        out = audio::LatencyTunerProfile_Gradual;
        return true;
    }

    return false;
}

ROC_ATTR_NO_SANITIZE_UB
bool resampler_backend_from_user(audio::ResamplerBackend& out, roc_resampler_backend in) {
    switch (enum_from_user(in)) {
    case ROC_RESAMPLER_BACKEND_DEFAULT:
        out = audio::ResamplerBackend_Default;
        return true;

    case ROC_RESAMPLER_BACKEND_BUILTIN:
        out = audio::ResamplerBackend_Builtin;
        return true;

    case ROC_RESAMPLER_BACKEND_SPEEX:
        out = audio::ResamplerBackend_Speex;
        return true;

    case ROC_RESAMPLER_BACKEND_SPEEXDEC:
        out = audio::ResamplerBackend_SpeexDec;
        return true;
    }

    return false;
}

ROC_ATTR_NO_SANITIZE_UB
bool resampler_profile_from_user(audio::ResamplerProfile& out, roc_resampler_profile in) {
    switch (enum_from_user(in)) {
    case ROC_RESAMPLER_PROFILE_LOW:
        out = audio::ResamplerProfile_Low;
        return true;

    case ROC_RESAMPLER_PROFILE_DEFAULT:
    case ROC_RESAMPLER_PROFILE_MEDIUM:
        out = audio::ResamplerProfile_Medium;
        return true;

    case ROC_RESAMPLER_PROFILE_HIGH:
        out = audio::ResamplerProfile_High;
        return true;
    }

    return false;
}

ROC_ATTR_NO_SANITIZE_UB
bool packet_encoding_from_user(unsigned& out_pt, roc_packet_encoding in) {
    switch (enum_from_user(in)) {
    case ROC_PACKET_ENCODING_AVP_L16_MONO:
        out_pt = rtp::PayloadType_L16_Mono;
        return true;

    case ROC_PACKET_ENCODING_AVP_L16_STEREO:
        out_pt = rtp::PayloadType_L16_Stereo;
        return true;
    }

    out_pt = in;
    return true;
}

ROC_ATTR_NO_SANITIZE_UB
bool fec_encoding_from_user(packet::FecScheme& out, roc_fec_encoding in) {
    switch (enum_from_user(in)) {
    case ROC_FEC_ENCODING_DISABLE:
        out = packet::FEC_None;
        return true;

    case ROC_FEC_ENCODING_DEFAULT:
    case ROC_FEC_ENCODING_RS8M:
        out = packet::FEC_ReedSolomon_M8;
        return true;

    case ROC_FEC_ENCODING_LDPC_STAIRCASE:
        out = packet::FEC_LDPC_Staircase;
        return true;
    }

    return false;
}

ROC_ATTR_NO_SANITIZE_UB
bool interface_from_user(address::Interface& out, const roc_interface& in) {
    switch (enum_from_user(in)) {
    case ROC_INTERFACE_CONSOLIDATED:
        out = address::Iface_Consolidated;
        return true;

    case ROC_INTERFACE_AUDIO_SOURCE:
        out = address::Iface_AudioSource;
        return true;

    case ROC_INTERFACE_AUDIO_REPAIR:
        out = address::Iface_AudioRepair;
        return true;

    case ROC_INTERFACE_AUDIO_CONTROL:
        out = address::Iface_AudioControl;
        return true;
    }

    return false;
}

ROC_ATTR_NO_SANITIZE_UB
bool proto_from_user(address::Protocol& out, const roc_protocol& in) {
    switch (enum_from_user(in)) {
    case ROC_PROTO_RTSP:
        out = address::Proto_RTSP;
        return true;

    case ROC_PROTO_RTP:
        out = address::Proto_RTP;
        return true;

    case ROC_PROTO_RTP_RS8M_SOURCE:
        out = address::Proto_RTP_RS8M_Source;
        return true;

    case ROC_PROTO_RS8M_REPAIR:
        out = address::Proto_RS8M_Repair;
        return true;

    case ROC_PROTO_RTP_LDPC_SOURCE:
        out = address::Proto_RTP_LDPC_Source;
        return true;

    case ROC_PROTO_LDPC_REPAIR:
        out = address::Proto_LDPC_Repair;
        return true;

    case ROC_PROTO_RTCP:
        out = address::Proto_RTCP;
        return true;
    }

    return false;
}

bool proto_to_user(roc_protocol& out, address::Protocol in) {
    switch (enum_from_user(in)) {
    case address::Proto_RTSP:
        out = ROC_PROTO_RTSP;
        return true;

    case address::Proto_RTP:
        out = ROC_PROTO_RTP;
        return true;

    case address::Proto_RTP_RS8M_Source:
        out = ROC_PROTO_RTP_RS8M_SOURCE;
        return true;

    case address::Proto_RS8M_Repair:
        out = ROC_PROTO_RS8M_REPAIR;
        return true;

    case address::Proto_RTP_LDPC_Source:
        out = ROC_PROTO_RTP_LDPC_SOURCE;
        return true;

    case address::Proto_LDPC_Repair:
        out = ROC_PROTO_LDPC_REPAIR;
        return true;

    case address::Proto_RTCP:
        out = ROC_PROTO_RTCP;
        return true;

    case address::Proto_None:
        break;
    }

    return false;
}

ROC_ATTR_NO_SANITIZE_UB
void receiver_slot_metrics_to_user(const pipeline::ReceiverSlotMetrics& slot_metrics,
                                   void* slot_arg) {
    roc_receiver_metrics& out = *(roc_receiver_metrics*)slot_arg;

    memset(&out, 0, sizeof(out));

    out.connection_count = (unsigned)slot_metrics.num_participants;
}

ROC_ATTR_NO_SANITIZE_UB
void receiver_participant_metrics_to_user(
    const pipeline::ReceiverParticipantMetrics& party_metrics,
    size_t party_index,
    void* party_arg) {
    roc_connection_metrics& out = *((roc_connection_metrics*)party_arg + party_index);

    memset(&out, 0, sizeof(out));

    if (party_metrics.latency.e2e_latency > 0) {
        out.e2e_latency = (unsigned long long)party_metrics.latency.e2e_latency;
    }
}

ROC_ATTR_NO_SANITIZE_UB
void sender_slot_metrics_to_user(const pipeline::SenderSlotMetrics& slot_metrics,
                                 void* slot_arg) {
    roc_sender_metrics& out = *(roc_sender_metrics*)slot_arg;

    memset(&out, 0, sizeof(out));

    out.connection_count = (unsigned)slot_metrics.num_participants;
}

ROC_ATTR_NO_SANITIZE_UB
void sender_participant_metrics_to_user(
    const pipeline::SenderParticipantMetrics& party_metrics,
    size_t party_index,
    void* party_arg) {
    roc_connection_metrics& out = *((roc_connection_metrics*)party_arg + party_index);

    memset(&out, 0, sizeof(out));

    if (party_metrics.latency.e2e_latency > 0) {
        out.e2e_latency = (unsigned long long)party_metrics.latency.e2e_latency;
    }
}

ROC_ATTR_NO_SANITIZE_UB
LogLevel log_level_from_user(roc_log_level in) {
    switch (enum_from_user(in)) {
    case ROC_LOG_NONE:
        return LogNone;

    case ROC_LOG_ERROR:
        return LogError;

    case ROC_LOG_INFO:
        return LogInfo;

    case ROC_LOG_NOTE:
        return LogNote;

    case ROC_LOG_DEBUG:
        return LogDebug;

    case ROC_LOG_TRACE:
        return LogTrace;
    }

    return LogError;
}

roc_log_level log_level_to_user(LogLevel in) {
    switch (in) {
    case LogNone:
        return ROC_LOG_NONE;

    case LogError:
        return ROC_LOG_ERROR;

    case LogInfo:
        return ROC_LOG_INFO;

    case LogNote:
        return ROC_LOG_NOTE;

    case LogDebug:
        return ROC_LOG_DEBUG;

    case LogTrace:
        return ROC_LOG_TRACE;
    }

    return ROC_LOG_ERROR;
}

void log_message_to_user(roc_log_message& out, const core::LogMessage& in) {
    out.level = log_level_to_user(in.level);
    out.module = in.module;
    out.file = in.file;
    out.line = in.line;
    out.time = (unsigned long long)in.time;
    out.pid = (unsigned long long)in.pid;
    out.tid = (unsigned long long)in.tid;
    out.text = in.text;
}

} // namespace api
} // namespace roc
