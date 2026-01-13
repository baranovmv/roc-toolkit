/*
 * Copyright (c) 2023 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "test_harness.h"

#include "roc_core/macro_helpers.h"
#include "roc_core/stddefs.h"
#include "roc_core/time.h"

#include "roc/config.h"
#include "roc/sender_encoder.h"

namespace roc {
namespace api {

TEST_GROUP(sender_encoder) {
    roc_sender_config sender_config;

    roc_context* context;

    void setup() {
        roc_context_config config;
        memset(&config, 0, sizeof(config));

        CHECK(roc_context_open(&config, &context) == 0);
        CHECK(context);

        memset(&sender_config, 0, sizeof(sender_config));
        sender_config.frame_encoding.format = ROC_FORMAT_PCM;
        sender_config.frame_encoding.subformat = ROC_SUBFORMAT_PCM_FLOAT32;
        sender_config.frame_encoding.rate = 44100;
        sender_config.frame_encoding.channels = ROC_CHANNEL_LAYOUT_STEREO;
        sender_config.packet_encoding = ROC_PACKET_ENCODING_AVP_L16_STEREO;
        sender_config.fec_encoding = ROC_FEC_ENCODING_DISABLE;
    }

    void teardown() {
        LONGS_EQUAL(0, roc_context_close(context));
    }
};

TEST(sender_encoder, open_close) {
    roc_sender_encoder* encoder = NULL;
    CHECK(roc_sender_encoder_open(context, &sender_config, &encoder) == 0);
    CHECK(encoder);

    LONGS_EQUAL(0, roc_sender_encoder_close(encoder));
}

TEST(sender_encoder, activate) {
    roc_sender_encoder* encoder = NULL;
    CHECK(roc_sender_encoder_open(context, &sender_config, &encoder) == 0);
    CHECK(encoder);

    CHECK(roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_SOURCE, ROC_PROTO_RTP)
          == 0);

    LONGS_EQUAL(0, roc_sender_encoder_close(encoder));
}

TEST(sender_encoder, activate_error) {
    roc_sender_encoder* encoder = NULL;
    CHECK(roc_sender_encoder_open(context, &sender_config, &encoder) == 0);
    CHECK(encoder);

    CHECK(roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_SOURCE, ROC_PROTO_RTP)
          == 0);

    CHECK(roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_SOURCE, ROC_PROTO_RTP)
          == -1);

    LONGS_EQUAL(0, roc_sender_encoder_close(encoder));
}

TEST(sender_encoder, bad_args) {
    { // open
        roc_sender_encoder* encoder = NULL;

        CHECK(roc_sender_encoder_open(NULL, &sender_config, &encoder) == -1);
        CHECK(roc_sender_encoder_open(context, NULL, &encoder) == -1);
        CHECK(roc_sender_encoder_open(context, &sender_config, NULL) == -1);

        roc_sender_config bad_config;
        memset(&bad_config, 0, sizeof(bad_config));
        CHECK(roc_sender_encoder_open(context, &bad_config, &encoder) == -1);
    }
    { // close
        CHECK(roc_sender_encoder_close(NULL) == -1);
    }
    { // activate
        roc_sender_encoder* encoder = NULL;
        CHECK(roc_sender_encoder_open(context, &sender_config, &encoder) == 0);

        CHECK(roc_sender_encoder_activate(NULL, ROC_INTERFACE_AUDIO_SOURCE, ROC_PROTO_RTP)
              == -1);
        CHECK(roc_sender_encoder_activate(encoder, (roc_interface)-1, ROC_PROTO_RTP)
              == -1);
        CHECK(roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_SOURCE,
                                          (roc_protocol)-1)
              == -1);

        LONGS_EQUAL(0, roc_sender_encoder_close(encoder));
    }
    { // query
        roc_sender_encoder* encoder = NULL;
        CHECK(roc_sender_encoder_open(context, &sender_config, &encoder) == 0);

        roc_sender_metrics send_metrics;
        roc_connection_metrics conn_metrics;

        // bad
        CHECK(roc_sender_encoder_query(NULL, &send_metrics, &conn_metrics) == -1);
        CHECK(roc_sender_encoder_query(encoder, NULL, &conn_metrics) == -1);
        CHECK(roc_sender_encoder_query(encoder, &send_metrics, NULL) == -1);

        // good
        CHECK(roc_sender_encoder_query(encoder, &send_metrics, &conn_metrics) == 0);

        LONGS_EQUAL(0, roc_sender_encoder_close(encoder));
    }
}

TEST(sender_encoder, push_frame_args) {
    roc_sender_encoder* encoder = NULL;
    CHECK(roc_sender_encoder_open(context, &sender_config, &encoder) == 0);

    float samples[16] = {};

    { // all good, not activated
        roc_frame frame;
        frame.samples = samples;
        frame.samples_size = ROC_ARRAY_SIZE(samples);
        CHECK(roc_sender_encoder_push_frame(encoder, &frame) == 0);
    }

    CHECK(roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_SOURCE, ROC_PROTO_RTP)
          == 0);

    { // all good, activated
        roc_frame frame;
        frame.samples = samples;
        frame.samples_size = ROC_ARRAY_SIZE(samples);
        CHECK(roc_sender_encoder_push_frame(encoder, &frame) == 0);
    }

    { // null encoder
        roc_frame frame;
        frame.samples = samples;
        frame.samples_size = ROC_ARRAY_SIZE(samples);
        CHECK(roc_sender_encoder_push_frame(NULL, &frame) == -1);
    }

    { // null frame
        CHECK(roc_sender_encoder_push_frame(encoder, NULL) == -1);
    }

    { // null samples, zero sample count
        roc_frame frame;
        frame.samples = NULL;
        frame.samples_size = 0;
        CHECK(roc_sender_encoder_push_frame(encoder, &frame) == 0);
    }

    { // null samples, non-zero sample count
        roc_frame frame;
        frame.samples = NULL;
        frame.samples_size = ROC_ARRAY_SIZE(samples);
        CHECK(roc_sender_encoder_push_frame(encoder, &frame) == -1);
    }

    { // uneven sample count
        roc_frame frame;
        frame.samples = samples;
        frame.samples_size = 1;
        CHECK(roc_sender_encoder_push_frame(encoder, &frame) == -1);
    }

    LONGS_EQUAL(0, roc_sender_encoder_close(encoder));
}

TEST(sender_encoder, push_feedback_packet_args) {
    roc_sender_encoder* encoder = NULL;
    CHECK(roc_sender_encoder_open(context, &sender_config, &encoder) == 0);

    uint8_t bytes[8192] = {};

    { // not activated
        roc_packet packet;
        packet.bytes = bytes;
        packet.bytes_size = ROC_ARRAY_SIZE(bytes);
        CHECK(roc_sender_encoder_push_feedback_packet(
                  encoder, ROC_INTERFACE_AUDIO_CONTROL, &packet)
              == -1);
    }

    CHECK(
        roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_CONTROL, ROC_PROTO_RTCP)
        == 0);

    { // null encoder
        roc_packet packet;
        packet.bytes = bytes;
        packet.bytes_size = ROC_ARRAY_SIZE(bytes);
        CHECK(roc_sender_encoder_push_feedback_packet(NULL, ROC_INTERFACE_AUDIO_CONTROL,
                                                      &packet)
              == -1);
    }

    { // bad interface
        roc_packet packet;
        packet.bytes = bytes;
        packet.bytes_size = ROC_ARRAY_SIZE(bytes);
        CHECK(roc_sender_encoder_push_feedback_packet(encoder, (roc_interface)-1, &packet)
              == -1);
    }

    CHECK(roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_SOURCE, ROC_PROTO_RTP)
          == 0);

    { // unsupported interface
        roc_packet packet;
        packet.bytes = bytes;
        packet.bytes_size = ROC_ARRAY_SIZE(bytes);
        CHECK(roc_sender_encoder_push_feedback_packet(encoder, ROC_INTERFACE_AUDIO_SOURCE,
                                                      &packet)
              == -1);
    }

    { // null packet
        CHECK(roc_sender_encoder_push_feedback_packet(encoder,
                                                      ROC_INTERFACE_AUDIO_CONTROL, NULL)
              == -1);
    }

    { // null bytes, non-zero byte count
        roc_packet packet;
        packet.bytes = NULL;
        packet.bytes_size = ROC_ARRAY_SIZE(bytes);
        CHECK(roc_sender_encoder_push_feedback_packet(
                  encoder, ROC_INTERFACE_AUDIO_CONTROL, &packet)
              == -1);
    }

    { // zero byte count
        roc_packet packet;
        packet.bytes = bytes;
        packet.bytes_size = 0;
        CHECK(roc_sender_encoder_push_feedback_packet(
                  encoder, ROC_INTERFACE_AUDIO_CONTROL, &packet)
              == -1);
    }

    { // large byte count
        float large_bytes[20000] = {};
        roc_packet packet;
        packet.bytes = large_bytes;
        packet.bytes_size = ROC_ARRAY_SIZE(large_bytes);
        CHECK(roc_sender_encoder_push_feedback_packet(
                  encoder, ROC_INTERFACE_AUDIO_CONTROL, &packet)
              == -1);
    }

    LONGS_EQUAL(0, roc_sender_encoder_close(encoder));
}

TEST(sender_encoder, pop_packet_args) {
    roc_sender_encoder* encoder = NULL;
    CHECK(roc_sender_encoder_open(context, &sender_config, &encoder) == 0);

    CHECK(roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_SOURCE, ROC_PROTO_RTP)
          == 0);

    {
        float samples[8192] = {};
        roc_frame frame;
        frame.samples = samples;
        frame.samples_size = ROC_ARRAY_SIZE(samples);
        CHECK(roc_sender_encoder_push_frame(encoder, &frame) == 0);
    }

    uint8_t bytes[8192] = {};

    { // null encoder
        roc_packet packet;
        packet.bytes = bytes;
        packet.bytes_size = ROC_ARRAY_SIZE(bytes);
        CHECK(roc_sender_encoder_pop_packet(NULL, ROC_INTERFACE_AUDIO_SOURCE, &packet)
              == -1);
    }

    { // bad interface
        roc_packet packet;
        packet.bytes = bytes;
        packet.bytes_size = ROC_ARRAY_SIZE(bytes);
        CHECK(roc_sender_encoder_pop_packet(encoder, (roc_interface)-1, &packet) == -1);
    }

    { // unactivated interface
        roc_packet packet;
        packet.bytes = bytes;
        packet.bytes_size = ROC_ARRAY_SIZE(bytes);
        CHECK(roc_sender_encoder_pop_packet(encoder, ROC_INTERFACE_AUDIO_REPAIR, &packet)
              == -1);
    }

    { // null packet
        CHECK(roc_sender_encoder_pop_packet(encoder, ROC_INTERFACE_AUDIO_SOURCE, NULL)
              == -1);
    }

    { // null bytes, non-zero byte count
        roc_packet packet;
        packet.bytes = NULL;
        packet.bytes_size = ROC_ARRAY_SIZE(bytes);
        CHECK(roc_sender_encoder_pop_packet(encoder, ROC_INTERFACE_AUDIO_SOURCE, &packet)
              == -1);
    }

    { // zero byte count
        roc_packet packet;
        packet.bytes = bytes;
        packet.bytes_size = 0;
        CHECK(roc_sender_encoder_pop_packet(encoder, ROC_INTERFACE_AUDIO_SOURCE, &packet)
              == -1);
    }

    { // small byte count
        roc_packet packet;
        packet.bytes = bytes;
        packet.bytes_size = 10;
        CHECK(roc_sender_encoder_pop_packet(encoder, ROC_INTERFACE_AUDIO_SOURCE, &packet)
              == -1);
    }

    { // all good
        roc_packet packet;
        packet.bytes = bytes;
        packet.bytes_size = ROC_ARRAY_SIZE(bytes);
        CHECK(roc_sender_encoder_pop_packet(encoder, ROC_INTERFACE_AUDIO_SOURCE, &packet)
              == 0);

        CHECK(packet.bytes == bytes);
        CHECK(packet.bytes_size > 0);
        CHECK(packet.bytes_size < ROC_ARRAY_SIZE(bytes));
    }

    LONGS_EQUAL(0, roc_sender_encoder_close(encoder));
}

TEST(sender_encoder, pop_packet_duration) {
    roc_sender_encoder* encoder = NULL;
    CHECK(roc_sender_encoder_open(context, &sender_config, &encoder) == 0);
    CHECK(encoder);

    CHECK(roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_SOURCE, ROC_PROTO_RTP)
          == 0);

    // Push one frame with known size
    const size_t samples_count = 8192;
    float samples[samples_count] = {};
    for (size_t i = 0; i < samples_count; i++) {
        samples[i] = 0.5f;
    }

    roc_frame frame;
    frame.samples = samples;
    frame.samples_size = samples_count;
    CHECK(roc_sender_encoder_push_frame(encoder, &frame) == 0);

    // Pop packet and check duration
    uint8_t bytes[8192] = {};
    roc_packet packet;
    packet.bytes = bytes;
    packet.bytes_size = ROC_ARRAY_SIZE(bytes);
    packet.duration = 0; // Initialize to zero

    CHECK(roc_sender_encoder_pop_packet(encoder, ROC_INTERFACE_AUDIO_SOURCE, &packet)
          == 0);

    // Duration should be non-zero and not UINT64_MAX for audio packets
    CHECK(packet.duration > 0);
    CHECK(packet.duration != UINT64_MAX);

    // Calculate expected duration based on packet encoding
    // RTP header (no extensions, no CSRC) is 12 bytes
    // L16 stereo: 2 bytes per sample, 2 channels
    const size_t rtp_header_size = 12;
    const size_t audio_payload = packet.bytes_size - rtp_header_size;
    const size_t samples_per_channel = audio_payload / 4; // 2 bytes * 2 channels
    const size_t sample_rate = 44100;
    const core::nanoseconds_t expected_duration_ns =
        (core::nanoseconds_t)samples_per_channel * core::Second
        / (core::nanoseconds_t)sample_rate;

    const core::nanoseconds_t tolerance = 10 * core::Nanosecond;
    CHECK((core::nanoseconds_t)packet.duration >= expected_duration_ns - tolerance);
    CHECK((core::nanoseconds_t)packet.duration <= expected_duration_ns + tolerance);

    LONGS_EQUAL(0, roc_sender_encoder_close(encoder));
}

TEST(sender_encoder, pop_packet_duration_control) {
    roc_sender_encoder* encoder = NULL;
    CHECK(roc_sender_encoder_open(context, &sender_config, &encoder) == 0);
    CHECK(encoder);

    // Activate both audio and control interfaces
    CHECK(roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_SOURCE, ROC_PROTO_RTP)
          == 0);
    CHECK(
        roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_CONTROL, ROC_PROTO_RTCP)
        == 0);

    // Push frames to generate some packets
    float samples[8192] = {};
    roc_frame frame;
    frame.samples = samples;
    frame.samples_size = ROC_ARRAY_SIZE(samples);

    // Push several frames to ensure control packets might be generated
    for (int i = 0; i < 10; i++) {
        CHECK(roc_sender_encoder_push_frame(encoder, &frame) == 0);
    }

    // Pop all audio packets
    uint8_t bytes[8192] = {};
    roc_packet packet;
    packet.bytes = bytes;
    packet.bytes_size = ROC_ARRAY_SIZE(bytes);

    while (roc_sender_encoder_pop_packet(encoder, ROC_INTERFACE_AUDIO_SOURCE, &packet)
           == 0) {
        // Audio packets should have valid duration
        CHECK(packet.duration != UINT64_MAX);
        CHECK(packet.duration > 0);
        packet.bytes_size = ROC_ARRAY_SIZE(bytes); // Reset for next iteration
    }

    // Try to pop control packet
    packet.bytes_size = ROC_ARRAY_SIZE(bytes);
    packet.duration = 12345; // Set to non-zero/non-UINT64_MAX value

    int result =
        roc_sender_encoder_pop_packet(encoder, ROC_INTERFACE_AUDIO_CONTROL, &packet);
    LONGS_EQUAL(0, result);
    // Control packets should have UINT64_MAX duration (not applicable)
    UNSIGNED_LONGLONGS_EQUAL(UINT64_MAX, packet.duration);

    LONGS_EQUAL(0, roc_sender_encoder_close(encoder));
}

TEST(sender_encoder, pop_packet_duration_multiple) {
    roc_sender_encoder* encoder = NULL;
    CHECK(roc_sender_encoder_open(context, &sender_config, &encoder) == 0);
    CHECK(encoder);

    CHECK(roc_sender_encoder_activate(encoder, ROC_INTERFACE_AUDIO_SOURCE, ROC_PROTO_RTP)
          == 0);

    // Push multiple frames to ensure multiple packets
    const size_t frame_size = 8192;
    float samples[frame_size] = {};
    roc_frame frame;
    frame.samples = samples;
    frame.samples_size = frame_size;

    // Push several frames
    for (size_t i = 0; i < 5; i++) {
        CHECK(roc_sender_encoder_push_frame(encoder, &frame) == 0);
    }

    uint8_t bytes[1099] = {};
    roc_packet packet;
    packet.bytes = bytes;
    packet.bytes_size = ROC_ARRAY_SIZE(bytes);

    core::nanoseconds_t total_duration = 0;
    size_t packet_count = 0;

    const size_t rtp_header_size = 12;
    const size_t sample_rate = 44100;
    const core::nanoseconds_t tolerance = 10 * core::Nanosecond;

    // Pop all packets and validate their durations
    while (true) {
        packet.duration = 0;

        if (roc_sender_encoder_pop_packet(encoder, ROC_INTERFACE_AUDIO_SOURCE, &packet)
            != 0) {
            break;
        }

        packet_count++;

        CHECK(packet.duration > 0);
        CHECK(packet.duration != UINT64_MAX);

        const size_t audio_payload = packet.bytes_size - rtp_header_size;
        const size_t samples_per_channel = audio_payload / 4; // L16 stereo: 2 bytes * 2
                                                              // channels
        const core::nanoseconds_t expected_duration =
            (core::nanoseconds_t)samples_per_channel * core::Second
            / (core::nanoseconds_t)sample_rate;

        // Verify duration matches expected value within tolerance
        const core::nanoseconds_t actual_duration = (core::nanoseconds_t)packet.duration;
        CHECK(actual_duration >= expected_duration - tolerance);
        CHECK(actual_duration <= expected_duration + tolerance);

        total_duration += actual_duration;
    }

    // Should have multiple packets
    CHECK(packet_count > 1);

    // Total duration should be reasonable
    CHECK(total_duration > 0);

    LONGS_EQUAL(0, roc_sender_encoder_close(encoder));
}

} // namespace api
} // namespace roc
