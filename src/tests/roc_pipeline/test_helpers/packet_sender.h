/*
 * Copyright (c) 2015 Roc Streaming authors
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef ROC_PIPELINE_TEST_HELPERS_PACKET_SENDER_H_
#define ROC_PIPELINE_TEST_HELPERS_PACKET_SENDER_H_

#include <CppUTest/TestHarness.h>

#include "roc_core/noncopyable.h"
#include "roc_packet/iwriter.h"
#include "roc_packet/packet_factory.h"
#include "roc_packet/queue.h"

namespace roc {
namespace pipeline {
namespace test {

class PacketSender : public packet::IWriter, core::NonCopyable<> {
public:
    PacketSender(packet::PacketFactory& packet_factory,
                 packet::IWriter* source_writer,
                 packet::IWriter* repair_writer)
        : packet_factory_(packet_factory)
        , source_writer_(source_writer)
        , repair_writer_(repair_writer) {
    }

    virtual void write(const packet::PacketPtr& pp) {
        queue_.write(pp);
    }

    void deliver(size_t n_source_packets) {
        for (size_t np = 0; np < n_source_packets;) {
            packet::PacketPtr pp = queue_.read();
            if (!pp) {
                break;
            }

            if (pp->flags() & packet::Packet::FlagRepair) {
                CHECK(repair_writer_);
                repair_writer_->write(copy_packet_(pp));
            } else {
                CHECK(source_writer_);
                np++;
                source_writer_->write(copy_packet_(pp));
            }
        }
    }

private:
    packet::PacketPtr copy_packet_(const packet::PacketPtr& pa) {
        packet::PacketPtr pb = packet_factory_.new_packet();
        CHECK(pb);

        CHECK(pa->flags() & packet::Packet::FlagUDP);
        pb->add_flags(packet::Packet::FlagUDP);
        *pb->udp() = *pa->udp();

        pb->set_data(pa->data());

        return pb;
    }

    packet::PacketFactory& packet_factory_;
    packet::IWriter* source_writer_;
    packet::IWriter* repair_writer_;
    packet::Queue queue_;
};

} // namespace test
} // namespace pipeline
} // namespace roc

#endif // ROC_PIPELINE_TEST_HELPERS_PACKET_SENDER_H_
