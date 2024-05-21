/*
 * Copyright (c) 2017 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

//! @file roc_packet/target_libuv/roc_packet/udp.h
//! @brief UDP packet.

#ifndef ROC_PACKET_UDP_H_
#define ROC_PACKET_UDP_H_

#include <uv.h>

#include "roc_address/socket_addr.h"
#include "roc_core/slice.h"
#include "roc_core/stddefs.h"
#include "roc_core/time.h"

namespace roc {
namespace packet {

//! UDP packet.
struct UDP {
    //! Source address.
    address::SocketAddr src_addr;

    //! Destination address.
    address::SocketAddr dst_addr;

    //! Packet receive timestamp (RTS), nanoseconds since Unix epoch.
    core::nanoseconds_t receive_timestamp;

    //! Timestamp in ns since unix-epoch. It points to a moment when
    //! the packet was transferred to a sink-thread, that "consumes"
    //! this packet. The reason to have it separate is that this
    //! allows us to account additional jitter introduced by
    //! thread-switch time.
    core::nanoseconds_t queue_ts;

    //! Sender request state.
    uv_udp_send_t request;

    UDP();

        //! It points to a moment when the packet was transferred to a sink-thread, that
    //! "consumes" this packet. The reason to have it separate is that this allows
    //! us to account additional jitter introduced by thread-switch time.
    core::nanoseconds_t enqueue_ts;
};

} // namespace packet
} // namespace roc

#endif // ROC_PACKET_UDP_H_
