/*
 * Copyright (c) 2026 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * \file roc/state.h
 * \brief Sender and receiver state.
 */

#ifndef ROC_STATE_H_
#define ROC_STATE_H_

#include "roc/platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Sender or receiver state.
 *
 * Unlike other enums of the API, the values are bit flags, so that a set of
 * states can be passed to roc_receiver_poll() and roc_sender_poll() as a
 * bitmask, e.g. <tt>ROC_STATE_ACTIVE | ROC_STATE_BROKEN</tt>.
 * At any moment, sender or receiver is exactly in one of these states.
 */
typedef enum roc_state {
    /** Sender or receiver is transferring media.
     *
     * On receiver, it means that there is at least one connected sender or at
     * least one packet that was not processed yet. On sender, it means that
     * at least one slot is connected.
     */
    ROC_STATE_ACTIVE = (1u << 0),

    /** Sender or receiver is running, but is not transferring media.
     *
     * Receiver in this state produces silence, and sender in this state does
     * not produce packets. It is safe to pause the associated audio device.
     */
    ROC_STATE_IDLE = (1u << 1),

    /** Sender or receiver is paused.
     *
     * Reserved for the upcoming pause and resume operations. It is never
     * reported by the current implementation.
     */
    ROC_STATE_PAUSED = (1u << 2),

    /** Sender or receiver is broken and can't be used anymore.
     *
     * The only allowed operation is to close it.
     */
    ROC_STATE_BROKEN = (1u << 3),

    /** Sender or receiver is closed and can't be used anymore. */
    ROC_STATE_CLOSED = (1u << 4)
} roc_state;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ROC_STATE_H_ */
