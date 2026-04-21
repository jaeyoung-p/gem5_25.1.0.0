/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MEM_CXL_MEM_LINK_HH__
#define __MEM_CXL_MEM_LINK_HH__

#include <deque>
#include <vector>

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
 * The object is a shared timing bottleneck with matched vectors of CPU-side
 * ingress ports and memory-side egress ports. It does not enumerate a CXL
 * device or model CXL.io; it only adds flit-based FIFO and serialization
 * delay between the host/Ruby side and backing memory controllers. Optional
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
        const PortID portId;
        const AddrRangeList ranges;

        std::deque<DeferredPacket> transmitList;
        bool retryReq;
        EventFunctionWrapper sendEvent;

        void schedTimingResp(PacketPtr pkt, Tick when, uint64_t flits);
        void trySendTiming();

      public:
        CxlResponsePort(const std::string &name, CxlMemLink &_link,
                        PortID _port_id, const AddrRange &_range);

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
        friend class CxlMemLink;
    };

    class CxlRequestPort : public RequestPort
    {
      private:
        CxlMemLink &link;
        const PortID portId;

        std::deque<DeferredPacket> transmitList;
        EventFunctionWrapper sendEvent;

        void schedTimingReq(PacketPtr pkt, Tick when, uint64_t flits);
        bool trySatisfyFunctional(PacketPtr pkt);
        void trySendTiming();

      public:
        CxlRequestPort(const std::string &name, CxlMemLink &_link,
                       PortID _port_id);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;

        friend class CxlResponsePort;
        friend class CxlMemLink;
    };

    std::vector<CxlRequestPort> memSidePorts;
    std::vector<CxlResponsePort> cpuSidePorts;

    const std::vector<AddrRange> portRanges;
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
    uint64_t queuedM2SFlits;
    uint64_t queuedS2MFlits;
    uint64_t reservedS2MFlits;

    bool use256BSlotModel() const;
    uint64_t serializationUnitBytes() const;
    uint64_t dataUnits(PacketPtr pkt) const;
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
    bool m2sQueueCanFit(uint64_t flits) const;
    bool s2mQueueCanFit(uint64_t flits) const;
    void reserveS2MResp(uint64_t flits);
    void consumeS2MRespReservation(uint64_t flits);
    void enqueueM2S(uint64_t flits);
    void dequeueM2S(uint64_t flits);
    void enqueueS2M(uint64_t flits);
    void dequeueS2M(uint64_t flits);
    void retryStalledReqs();
    CxlRequestPort &memSidePort(PortID port_id);
    const CxlRequestPort &memSidePort(PortID port_id) const;
    CxlResponsePort &cpuSidePort(PortID port_id);
    const CxlResponsePort &cpuSidePort(PortID port_id) const;

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
