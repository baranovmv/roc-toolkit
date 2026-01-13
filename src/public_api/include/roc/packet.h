/*
 * Copyright (c) 2023 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * \file roc/packet.h
 * \brief Network packet.
 */

#ifndef ROC_PACKET_H_
#define ROC_PACKET_H_

#include "roc/platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Network packet.
 *
 * Represents opaque encoded binary packet.
 *
 * **Thread safety**
 *
 * Should not be used concurrently.
 */
typedef struct roc_packet {
    /** Packet bytes.
     */
    void* bytes;

    /** Packet bytes count.
     */
    size_t bytes_size;

    /** Duration of packet payload in nanoseconds.
     *
     * For RTP packets with audio, represents the duration of audio samples
     * contained in the packet, calculated from sample count and sample rate.
     *
     * For non-audio packets (e.g., RTCP control), this field is set to
     * UINT64_MAX to indicate "not applicable".
     *
     * Only populated when reading packets via roc_sender_encoder_pop_packet().
     * Ignored when writing packets.
     */
    unsigned long long duration;
} roc_packet;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ROC_PACKET_H_ */
