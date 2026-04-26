/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MEM_INTEGRITY_MEM_LINK_HH__
#define __MEM_INTEGRITY_MEM_LINK_HH__

#include <deque>
#include <list>
#include <memory>
#include <vector>

#include "base/addr_range.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/IntegrityMemLink.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/stats.hh"

namespace gem5
{

/**
 * Host-side fake integrity timing model.
 *
 * The object is intended to sit below the guest-visible cache hierarchy and
 * above a memory target. When enabled, each read or write timing request is
 * paired with an internal 8B-per-64B MAC request. The MAC packet is ordinary
 * downstream memory traffic, so it consumes downstream link/controller timing,
 * but it is never exposed to the CPU caches or to the guest physical map.
 */
class IntegrityMemLink : public ClockedObject
{
  protected:
    class DeferredPacket
    {
      public:
        const Tick tick;
        const PacketPtr pkt;
        DeferredPacket(PacketPtr _pkt, Tick _tick) : tick(_tick), pkt(_pkt) {}
    };

    struct ProtectedRequest;

    struct IntegritySenderState : public Packet::SenderState
    {
        ProtectedRequest *request;
        bool mac;

        IntegritySenderState(ProtectedRequest *_request, bool _mac)
            : request(_request), mac(_mac)
        {}
    };

    struct ProtectedRequest
    {
        PacketPtr dataPkt = nullptr;
        PacketPtr macPkt = nullptr;
        bool dataDone = false;
        bool macDone = false;
    };

    class MemoryPort : public RequestPort
    {
      private:
        IntegrityMemLink &link;
        std::deque<DeferredPacket> transmitList;
        EventFunctionWrapper sendEvent;

      public:
        MemoryPort(const std::string &name, IntegrityMemLink &_link);

        bool reqQueueFull(unsigned needed = 1) const;
        void schedTimingReq(PacketPtr pkt, Tick when);
        bool trySatisfyFunctional(PacketPtr pkt);
        void trySendTiming();

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    class CpuPort : public ResponsePort
    {
      private:
        IntegrityMemLink &link;
        const AddrRangeList ranges;
        bool retryReq;
        std::deque<DeferredPacket> transmitList;
        EventFunctionWrapper sendEvent;

      public:
        CpuPort(const std::string &name, IntegrityMemLink &_link,
                const AddrRange &_range);

        bool respQueueFull() const;
        void schedTimingResp(PacketPtr pkt, Tick when);
        void retryStalledReq();
        void trySendTiming();

      protected:
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        Tick recvAtomic(PacketPtr pkt) override;
        Tick recvAtomicBackdoor(PacketPtr pkt,
                                MemBackdoorPtr &backdoor) override;
        void recvFunctional(PacketPtr pkt) override;
        void recvMemBackdoorReq(const MemBackdoorReq &req,
                                MemBackdoorPtr &backdoor) override;
        AddrRangeList getAddrRanges() const override;
    };

    MemoryPort memSidePort;
    CpuPort cpuSidePort;

    const AddrRange visibleRange;
    const bool enabled;
    const uint64_t macLineBytes;
    const uint64_t macBytesPerLine;
    const uint64_t requestQueueSize;
    const uint64_t responseQueueSize;

    std::list<std::unique_ptr<ProtectedRequest>> protectedRequests;

    bool shouldProtect(PacketPtr pkt) const;
    uint64_t macBytesFor(PacketPtr pkt) const;
    PacketPtr makeMacPacket(PacketPtr data_pkt) const;
    std::unique_ptr<ProtectedRequest> makeProtectedRequest(PacketPtr pkt);
    void enqueueDownstream(PacketPtr pkt, Tick when);
    void handleProtectedResp(PacketPtr pkt, IntegritySenderState *state);
    void maybeComplete(ProtectedRequest *request);
    void eraseProtectedRequest(ProtectedRequest *request);
    Tick recvAtomicProtected(PacketPtr pkt);
    void accountMac(PacketPtr pkt);

    statistics::Scalar integrityMacReadReqs;
    statistics::Scalar integrityMacWriteReqs;
    statistics::Scalar integrityMacReadBytes;
    statistics::Scalar integrityMacWriteBytes;
    statistics::Scalar integrityMacPairedReads;
    statistics::Scalar integrityMacPairedWrites;
    statistics::Scalar integrityMacRejectedReqs;

  public:
    PARAMS(IntegrityMemLink);

    IntegrityMemLink(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    void init() override;
};

} // namespace gem5

#endif // __MEM_INTEGRITY_MEM_LINK_HH__
