/*
 * Copyright (c) 2026 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

//! @file roc_netio/target_libuv/roc_netio/network_config.h
//! @brief Network loop config.

#ifndef ROC_NETIO_NETWORK_CONFIG_H_
#define ROC_NETIO_NETWORK_CONFIG_H_

#include "roc_core/stddefs.h"

namespace roc {
namespace netio {

//! Network loop config.
struct NetworkConfig {
    //! Priority of the network thread.
    //! @remarks
    //!  If non-zero, network thread is switched to real-time scheduling policy
    //!  with this priority. Should be in range [0; 99]. Requires privileges.
    int realtime_prio;

    //! Initialize.
    NetworkConfig()
        : realtime_prio(0) {
    }
};

} // namespace netio
} // namespace roc

#endif // ROC_NETIO_NETWORK_CONFIG_H_
