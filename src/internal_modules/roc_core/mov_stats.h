/*
 * Copyright (c) 2019 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

//! @file roc_core/mov_stats.h
//! @brief Profiler.

#ifndef ROC_CORE_MOV_STATS_H_
#define ROC_CORE_MOV_STATS_H_

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
template <typename T> class MovStats {
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
        , mov_max_cntr_(0)
        , full_(false)
        , first_(true)
        , queue_max_(arena, win_len + 1)
        , curr_max_(T(0))
        , queue_min_(arena, win_len + 1)
        , curr_min_(T(0)) {
        if (!buffer_.resize(win_len)) {
            roc_panic("MovStats: can't allocate storage for the ring buffer");
        }
        if (!buffer2_.resize(win_len)) {
            roc_panic("MovStats: can't allocate storage for the ring buffer");
        }
        memset(buffer_.data(), 0, sizeof(T) * buffer_.size());
        memset(buffer2_.data(), 0, sizeof(T) * buffer2_.size());
    }

    //! Shift rolling window by one sample x.
    void add(const T& x) {
        const T x2 = x * x;
        const T x_old = buffer_[buffer_i_];
        buffer_[buffer_i_] = x;
        const T x2_old = buffer2_[buffer_i_];
        buffer2_[buffer_i_] = x2;

        movsum_ += x - x_old;
        movsum2_ += x2 - x2_old;

        buffer_i_++;
        if (buffer_i_ == win_len_) {
            buffer_i_ = 0;
            full_ = true;
        }

        slide_max(x, x_old);
        slide_min(x, x_old);
    }

    // Keeping a sliding max by using a sorted deque.
    // The wedge is always sorted in descending order.
    // The current max is always at the front of the wedge.
    // https://www.geeksforgeeks.org/sliding-window-maximum-maximum-of-all-subarrays-of-size-k/
    void slide_max(const T& x, const T x_old) {
        if (queue_max_.is_empty()) {
            queue_max_.push_back(x);
            curr_max_ = x;
        } else {
            if (queue_max_.front() == x_old) {
                queue_max_.pop_front();
                curr_max_ = queue_max_.is_empty() ? x : queue_max_.front();
            }
            while (!queue_max_.is_empty() && queue_max_.back() < x) {
                queue_max_.pop_back();
            }
            if (queue_max_.is_empty()) {
                curr_max_ = x;
            }
            queue_max_.push_back(x);
        }
    }

    // Keeping a sliding min by using a sorted deque.
    // The wedge is always sorted in ascending order.
    // The current min is always at the front of the wedge.
    // https://www.geeksforgeeks.org/sliding-window-maximum-maximum-of-all-subarrays-of-size-k/
    void slide_min(const T& x, const T x_old) {
        if (queue_min_.is_empty()) {
            queue_min_.push_back(x);
            curr_min_ = x;
        } else {
            if (queue_min_.front() == x_old) {
                queue_min_.pop_front();
                curr_min_ = queue_min_.is_empty() ? x : queue_min_.front();
            }
            while (!queue_min_.is_empty() && queue_min_.back() > x) {
                queue_min_.pop_back();
            }
            if (queue_min_.is_empty()) {
                curr_min_ = x;
            }
            queue_min_.push_back(x);
        }
    }

    //! Get moving average value.
    T mov_avg() const {
        const T n = full_ ? T(win_len_) : T(buffer_i_ + 1);
        return movsum_ / n;
    }

    //! Get variance.
    T mov_var() const {
        const T n = full_ ? T(win_len_) : T(buffer_i_ + 1);
        return (T)sqrt((n * movsum2_ - movsum_ * movsum_) / (n * n));
    }

    T mov_max() const {
        return curr_max_;
    }

    T mov_min() const {
        return curr_min_;
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
    void extend_win(const size_t new_win) {
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

    class Queue {
    public:
        Queue(core::IArena& arena, size_t len)
            : buff_(arena)
            , buff_len_(len)
            , begin_(0)
            , end_(0) {
            if (!buff_.resize(len)) {
                roc_panic("Queue: can't allocate storage for the buffer");
            }
        }

        T& front() {
            if (is_empty()) {
                roc_panic("Queue: front() called on empty buffer");
            }
            return buff_[begin_];
        }

        T& back() {
            if (is_empty()) {
                roc_panic("Queue: back() called on empty buffer");
            }
            return buff_[(end_ - 1 + buff_len_) % buff_len_];
        }

        size_t len() const {
            return (end_ - begin_ + buff_len_) % buff_len_;
        }

        void push_front(const T& x) {
            begin_ = (begin_ - 1 + buff_len_) % buff_len_;
            buff_[begin_] = x;
            roc_panic_if_msg(end_ == begin_, "Queue: buffer overflow");
        }

        void pop_front() {
            if (is_empty()) {
                roc_panic("Queue: pop_front() called on empty buffer");
            }
            begin_ = (begin_ + 1) % buff_len_;
        }

        void push_back(const T& x) {
            buff_[end_] = x;
            end_ = (end_ + 1) % buff_len_;
            roc_panic_if_msg(end_ == begin_, "Queue: buffer overflow");
        }

        void pop_back() {
            if (is_empty()) {
                roc_panic("Queue: pop_back() called on empty buffer");
            }
            end_ = (end_ - 1 + buff_len_) % buff_len_;
        }

        bool is_empty() {
            return begin_ == end_;
        }

    private:
        Array<T> buff_;
        size_t buff_len_;
        size_t begin_;
        size_t end_;
    };

    Queue queue_max_;
    T curr_max_;
    Queue queue_min_;
    T curr_min_;
};

} // namespace core
} // namespace roc

#endif // ROC_CORE_MOV_STATS_H_
