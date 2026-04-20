/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MEM_CXL_MEM_LINK_HH__
#define __MEM_CXL_MEM_LINK_HH__

#include <deque>

#include "base/types.hh"
#include "mem/port.hh"
#include "params/CxlMemLink.hh"
#include "sim/clocked_object.hh"
#include "sim/stats.hh"

namespace gem5
{

/**
 * A deliberately abstract CXL Type 3 CXL.mem link model.
 *
 * The object is a two-port timing bridge. It does not enumerate a CXL device
 * or model CXL.io; it only adds flit-based FIFO and serialization delay
 * between the host/Ruby side and a backing memory controller. Optional
 * base-link latency parameters are retained for calibration experiments, but
 * project configs default them to zero.
 */
class CxlMemLink : public ClockedObject
{
  protected:
    class DeferredPacket
    {
      public:
        const Tick tick;
        const PacketPtr pkt;
        const uint64_t flits;

        DeferredPacket(PacketPtr _pkt, Tick _tick, uint64_t _flits)
            : tick(_tick), pkt(_pkt), flits(_flits)
        {}
    };

    struct LinkSchedule
    {
        Tick readyTick = 0;
        Tick queueWait = 0;
        Tick serialization = 0;
        Tick baseLatency = 0;
    };

    class CxlRequestPort;

    class CxlResponsePort : public ResponsePort
    {
      private:
        CxlMemLink &link;
        CxlRequestPort &memSidePort;
        const AddrRangeList ranges;

        std::deque<DeferredPacket> transmitList;
        uint64_t queuedFlits;
        uint64_t reservedRespFlits;
        bool retryReq;
        EventFunctionWrapper sendEvent;

        bool respQueueCanFit(uint64_t flits) const;
        void reserveResp(uint64_t flits);
        void consumeRespReservation(uint64_t flits);
        void schedTimingResp(PacketPtr pkt, Tick when, uint64_t flits);
        void retryStalledReq();
        void trySendTiming();

      public:
        CxlResponsePort(const std::string &name, CxlMemLink &_link,
                        CxlRequestPort &_mem_side_port,
                        const std::vector<AddrRange> &_ranges);

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

        friend class CxlRequestPort;
    };

    class CxlRequestPort : public RequestPort
    {
      private:
        CxlMemLink &link;
        CxlResponsePort &cpuSidePort;

        std::deque<DeferredPacket> transmitList;
        uint64_t queuedFlits;
        EventFunctionWrapper sendEvent;

        bool reqQueueCanFit(uint64_t flits) const;
        void schedTimingReq(PacketPtr pkt, Tick when, uint64_t flits);
        bool trySatisfyFunctional(PacketPtr pkt);
        void trySendTiming();

      public:
        CxlRequestPort(const std::string &name, CxlMemLink &_link,
                       CxlResponsePort &_cpu_side_port);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;

        friend class CxlResponsePort;
    };

    CxlRequestPort memSidePort;
    CxlResponsePort cpuSidePort;

    const AddrRangeList ranges;
    const uint64_t flitSizeBytes;
    const double bandwidth;
    const Tick m2sLatency;
    const Tick s2mLatency;
    const uint64_t requestHeaderFlits;
    const uint64_t responseHeaderFlits;
    const uint64_t m2sQueueDepthFlits;
    const uint64_t s2mQueueDepthFlits;

    Tick nextM2SReady;
    Tick nextS2MReady;
    Tick lastM2SQueueUpdate;
    Tick lastS2MQueueUpdate;

    uint64_t dataFlits(PacketPtr pkt) const;
    uint64_t m2sRequestFlits(PacketPtr pkt) const;
    uint64_t s2mResponseFlitsForRequest(PacketPtr pkt) const;
    uint64_t s2mResponseFlits(PacketPtr pkt) const;
    Tick serializationDelay(uint64_t flits) const;
    LinkSchedule scheduleM2S(Tick arrival, uint64_t flits);
    LinkSchedule scheduleS2M(Tick arrival, uint64_t flits);

    void accountM2SQueueOccupancy(uint64_t queued_flits);
    void accountS2MQueueOccupancy(uint64_t queued_flits);
    void recordM2SPacket(uint64_t flits, const LinkSchedule &schedule);
    void recordS2MPacket(uint64_t flits, const LinkSchedule &schedule);
    void recordM2SStall();
    void recordS2MStall();

    statistics::Scalar m2sPackets;
    statistics::Scalar s2mPackets;
    statistics::Scalar m2sFlits;
    statistics::Scalar s2mFlits;
    statistics::Scalar m2sQueueWaitTicks;
    statistics::Scalar s2mQueueWaitTicks;
    statistics::Scalar m2sSerializationTicks;
    statistics::Scalar s2mSerializationTicks;
    statistics::Scalar m2sBaseLatencyTicks;
    statistics::Scalar s2mBaseLatencyTicks;
    statistics::Scalar m2sTotalDelayTicks;
    statistics::Scalar s2mTotalDelayTicks;
    statistics::Scalar m2sQueueOccupancyFlitTicks;
    statistics::Scalar s2mQueueOccupancyFlitTicks;
    statistics::Scalar m2sFullEvents;
    statistics::Scalar s2mFullEvents;

  public:
    PARAMS(CxlMemLink);

    CxlMemLink(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    void init() override;
};

} // namespace gem5

#endif // __MEM_CXL_MEM_LINK_HH__
