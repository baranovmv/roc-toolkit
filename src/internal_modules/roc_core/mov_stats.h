/*
 * Copyright (c) 2019 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

//! @file roc_core/mov_stats.h
//! @brief Profiler.

#ifndef ROC_TOOLKIT_MOV_STATS_H
#define ROC_TOOLKIT_MOV_STATS_H

#include "roc_core/array.h"
#include "roc_core/iarena.h"
#include "roc_core/panic.h"

namespace roc {
namespace core {

//! Rolling window moving average and variance.
//!
//! Efficiently implements moving average and variance based on approach
//! described in https://www.dsprelated.com/showthread/comp.dsp/97276-1.php
//!
//! @tparam T defines a sample type.
template <typename T>
class MovStats {
public:
    //! Initialize.
    MovStats(IArena& arena, const size_t win_len)
        : buffer_(arena)
        , buffer2_(arena)
        , win_len_(win_len)
        , buffer_i_(0)
        , movsum_(T(0))
        , movsum2_(T(0))
        , mov_var_(T(0))
        , full_(false)
        , mov_max_cntr_(0)
        , first_(true)
    {
        if(!buffer_.resize(win_len)){
            roc_panic("MovStats: can't allocate storage for the ring buffer");
        }
        if(!buffer2_.resize(win_len)){
            roc_panic("MovStats: can't allocate storage for the ring buffer");
        }
        memset(buffer_.data(), 0, sizeof(T)*buffer_.size());
        memset(buffer2_.data(), 0, sizeof(T)*buffer2_.size());
    }

    //! Shift rolling window by one sample x.
    void add(const T& x)
    {
        const T x2 = x*x;
        const T x_old = buffer_[buffer_i_];
        buffer_[buffer_i_] = x;
        const T x2_old = buffer2_[buffer_i_];
        buffer2_[buffer_i_] = x2;

        movsum_ += x - x_old;
        movsum2_ += x2 - x2_old;

        if (first_) {
            first_ = false;
            mov_max_ = x;
            mov_max_cntr_++;
        } else {
            if (x > mov_max_) {
                mov_max_ = x;
                mov_max_cntr_ = 1;
            } else if (x == mov_max_) {
                mov_max_cntr_++;
            }

            if (mov_max_ == x_old) {
                mov_
            }
        }

        buffer_i_++;
        if (buffer_i_ == win_len_) {
            buffer_i_ = 0;
            full_ = true;
        }

    }

    //! Get moving average value.
    T mov_avg() const
    {
        const T n = full_ ? T(win_len_) : T(buffer_i_ + 1);
        return movsum_ / n;
    }

    //! Get variance.
    T mov_var() const
    {
        const T n = full_ ? T(win_len_) : T(buffer_i_+1);
        if (n == 1) {
            return (T)sqrt(movsum2_ - movsum_ * movsum_);
        } else {
            return (T)sqrt((n*movsum2_ - movsum_ * movsum_) / (n * n));
        }
    }

    //! Extend rolling window length.
    //! @remarks
    //! Potentially could cause a gap in the estimated values as
    //! decreases effective window size by dropping samples to the right from
    //! the cursor in the ring buffers:
    //!          buffer_i_        win_len_ old       win_len_ new
    //!             ↓                    ↓                  ↓
    //!  [■■■■■■■■■■□□□□□□□□□□□□□□□□□□□□□--------------------]
    //!             ↑         ↑          ↑
    //!                Dropped samples.
    void extend_win(const size_t new_win)
    {
        if (new_win <= win_len_) {
            roc_panic("MovStats: the window length can only grow");
        }
        if (!buffer_.resize(new_win)) {
            roc_panic("MovStats: can not increase storage");
        }

        movsum_ = 0;
        movsum2_ = 0;
        for (size_t i = 0; i < buffer_i_; i++) {
            movsum_ += buffer_[i];
            movsum2_ += buffer2_[i];
        }
        full_ = false;
    }

private:
    Array<T> buffer_;
    Array<T> buffer2_;
    const size_t win_len_;
    size_t buffer_i_;
    T movsum_;
    T movsum2_;
    T mov_var_;
    T mov_max_;
    size_t mov_max_cntr_;

    bool full_;
    bool first_;
};

} // namespace core
} // namespace roc

#endif // ROC_TOOLKIT_MOV_STATS_H
