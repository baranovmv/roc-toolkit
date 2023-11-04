/*
 * Copyright (c) 2023 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <CppUTest/TestHarness.h>

#include "roc_core/mov_stats.h"
#include "roc_core/heap_arena.h"

namespace roc {
namespace core {

namespace {

enum { NumObjects = 10, EmbeddedCap = 5 };

struct Object {
    static long n_objects;

    size_t value;

    Object(size_t v = 0)
        : value(v) {
        n_objects++;
    }

    Object(const Object& other)
        : value(other.value) {
        n_objects++;
    }

    ~Object() {
        n_objects--;
    }
};

long Object::n_objects = 0;

} // namespace

TEST_GROUP(movstats) {
    HeapArena arena;
};

TEST(movstats, single_pass) {
    const size_t n = 10;
    MovStats<int64_t> stats(arena, n);
    const int64_t target_avg = (n-1) * n / 2;
    int64_t target_var = 0;
    for (size_t i = 0; i < n; i++) {
        const int64_t x = int64_t(i * n);
        stats.add(x);
        target_var += (x - target_avg) * (x - target_avg);
    }
    target_var = (int64_t) sqrt(target_var / (int64_t)n);

    LONGS_EQUAL(target_avg, stats.mov_avg());
    LONGS_EQUAL(target_var, stats.mov_var());
}

TEST(movstats, one_n_half_pass) {
    const size_t n = 10;
    MovStats<int64_t> stats(arena, n);
    for (size_t i = 0; i < (n * 10 + n/2); i++) {
        const int64_t x = (int64_t) pow(-1., (double)i);
        stats.add(x);
    }

    LONGS_EQUAL(0, stats.mov_avg());
    LONGS_EQUAL(1, stats.mov_var());

    const int64_t target_avg = (n-1) * n / 2;
    int64_t target_var = 0;
    for (size_t i = 0; i < n; i++) {
        const int64_t x = int64_t(i * n);
        stats.add(x);
        target_var += (x - target_avg) * (x - target_avg);
    }
    target_var = (int64_t) sqrt(target_var / (int64_t)n);

    LONGS_EQUAL(target_avg, stats.mov_avg());
    LONGS_EQUAL(target_var, stats.mov_var());
}

} // namespace core
} // namespace roc
