/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MEM_CXL_MEM_LINK_HH__
#define __MEM_CXL_MEM_LINK_HH__

#include <array>
#include <deque>
#include <memory>
#include <vector>

#include "base/types.hh"
#include "mem/port.hh"
#include "params/CxlMemLink.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/stats.hh"

namespace gem5
{

/**
 * A deliberately abstract CXL Type 3 CXL.mem link model.
 *
 * The object is a shared timing bottleneck with matched vectors of CPU-side
 * ingress ports and memory-side egress ports. It does not enumerate a CXL
 * device or model CXL.io; it only adds flit-based FIFO and serialization
 * delay between the host/Ruby side and backing memory controllers.
 *
 * The current implementation intentionally covers only a narrow first-pass
 * 256B flit subset for the memory-only NUMA path:
 * - direct-attached Type 3-style M2S/S2M traffic only
 * - explicit internal message types for Req/RwD/NDR/DRS
 * - one active data-header start per emitted flit
 * - rollover of data-bearing messages across flits
 * - no BISnp/BIRsp, LOpt 256B halves, CRC/FEC, or replay correctness model
 */
class CxlMemLink : public ClockedObject
{
  protected:
    class DeferredPacket
    {
      public:
        const Tick tick;
        const PacketPtr pkt;

        DeferredPacket(PacketPtr _pkt, Tick _tick) : tick(_tick), pkt(_pkt) {}
    };

    struct LinkSchedule
    {
        Tick readyTick = 0;
        Tick queueWait = 0;
        Tick serialization = 0;
        Tick baseLatency = 0;
    };

    enum class MessageClass : uint8_t
    {
        M2SReq,
        M2SRwD,
        S2MNDR,
        S2MDRS,
    };

    enum class LinkDirection : uint8_t
    {
        M2S,
        S2M,
    };

    struct ProtocolMessage
    {
        const MessageClass msgClass;
        const PacketPtr pkt;
        const PortID portId;
        const Tick arrivalTick;
        const uint64_t dataSlotsTotal;
        const uint64_t trailerSlotsTotal;
        const uint64_t reservedFlits;

        bool headerSent = false;
        uint64_t dataSlotsSent = 0;
        uint64_t trailerSlotsSent = 0;
        uint64_t emittedFlits = 0;
        Tick serviceStartTick = 0;
        Tick completionTick = 0;
        Tick lastFlitTick = 0;

        ProtocolMessage(MessageClass _msg_class, PacketPtr _pkt,
                        PortID _port_id, Tick _arrival_tick,
                        uint64_t _data_slots_total,
                        uint64_t _trailer_slots_total,
                        uint64_t _reserved_flits)
            : msgClass(_msg_class),
              pkt(_pkt),
              portId(_port_id),
              arrivalTick(_arrival_tick),
              dataSlotsTotal(_data_slots_total),
              trailerSlotsTotal(_trailer_slots_total),
              reservedFlits(_reserved_flits)
        {}

        bool dataBearing() const;
        uint64_t remainingDataSlots() const;
        uint64_t remainingTrailerSlots() const;
        bool complete() const;
    };

    using ProtocolMessagePtr = std::shared_ptr<ProtocolMessage>;

    struct DirectionState
    {
        LinkDirection direction;
        Tick baseLatency = 0;
        uint64_t queueDepthFlits = 0;
        Tick nextFlitTick = 0;
        Tick lastQueueUpdate = 0;
        uint64_t queuedFlits = 0;
        std::array<uint32_t, 2> prevTailCounts = {0, 0};
        std::deque<ProtocolMessagePtr> pending;
        ProtocolMessagePtr activeDataMsg;
        EventFunctionWrapper emitEvent;

        DirectionState(CxlMemLink &link, LinkDirection _direction,
                       Tick _base_latency, uint64_t _queue_depth,
                       const std::string &event_name);
        DirectionState(DirectionState &&) = default;
        DirectionState &operator=(DirectionState &&) = default;
        DirectionState(const DirectionState &) = delete;
        DirectionState &operator=(const DirectionState &) = delete;
    };

    struct FlitBuildState
    {
        std::array<std::array<uint32_t, 4>, 2> groupCounts = {{
            {0, 0, 0, 0},
            {0, 0, 0, 0},
        }};
        bool startedDataHeader = false;
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

        void schedTimingResp(PacketPtr pkt, Tick when);
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

        void schedTimingReq(PacketPtr pkt, Tick when);
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

    DirectionState m2sState;
    DirectionState s2mState;
    Tick lastM2SQueueUpdate;
    Tick lastS2MQueueUpdate;
    uint64_t reservedS2MFlits;

    bool use256BFlitPacker() const;
    uint64_t serializationUnitBytes() const;
    Tick serializationDelay(uint64_t flits) const;

    MessageClass m2sMessageClass(PacketPtr pkt) const;
    MessageClass s2mMessageClass(PacketPtr pkt) const;
    uint64_t dataSlots(PacketPtr pkt) const;
    uint64_t trailerSlots(MessageClass msg_class, PacketPtr pkt) const;
    uint64_t reservedFlits(MessageClass msg_class, PacketPtr pkt) const;
    uint64_t reservedS2MRespFlits(PacketPtr pkt) const;
    Tick standaloneMessageDelay(MessageClass msg_class, PacketPtr pkt) const;

    DirectionState &directionState(LinkDirection direction);
    const DirectionState &directionState(LinkDirection direction) const;
    void accountQueueOccupancy(DirectionState &state);
    bool queueCanFit(const DirectionState &state, uint64_t flits) const;
    void enqueueMessage(DirectionState &state, const ProtocolMessagePtr &msg);
    void dequeueMessage(DirectionState &state, const ProtocolMessagePtr &msg);
    void maybeScheduleFlit(DirectionState &state, Tick when);
    void processDirectionFlit(DirectionState &state);
    void touchMessageFlit(const ProtocolMessagePtr &msg, Tick flit_tick);
    void markHeaderSent(const ProtocolMessagePtr &msg, Tick flit_tick);
    void consumeContinuationSlot(const ProtocolMessagePtr &msg,
                                 Tick flit_tick);
    void completeMessage(DirectionState &state, const ProtocolMessagePtr &msg,
                         Tick completion_tick);

    int slotGroup(int slot) const;
    int groupCountIndex(MessageClass msg_class) const;
    uint32_t maxGroupMessages(MessageClass msg_class) const;
    bool isDataBearing(MessageClass msg_class) const;
    bool isSlotLegal(MessageClass msg_class, bool header_slot) const;
    bool canStartDataHeader(const ProtocolMessagePtr &msg,
                            bool header_slot) const;
    bool canPackGroupMessage(const DirectionState &state,
                             const FlitBuildState &flit,
                             MessageClass msg_class, int slot,
                             uint32_t count) const;
    void recordGroupMessage(FlitBuildState &flit, MessageClass msg_class,
                            int slot, uint32_t count);
    uint32_t packNdrHeaders(DirectionState &state, FlitBuildState &flit,
                            int slot, Tick flit_start, Tick flit_end);
    bool startDataHeader(DirectionState &state, FlitBuildState &flit, int slot,
                         Tick flit_start);
    bool packReqHeader(DirectionState &state, FlitBuildState &flit, int slot,
                       Tick flit_start, Tick flit_end);
    bool packHeaderSlot(DirectionState &state, FlitBuildState &flit, int slot,
                        Tick flit_start, Tick flit_end);

    void accountM2SQueueOccupancy(uint64_t queued_flits);
    void accountS2MQueueOccupancy(uint64_t queued_flits);
    void recordM2SPacket(uint64_t flits, const LinkSchedule &schedule);
    void recordS2MPacket(uint64_t flits, const LinkSchedule &schedule);
    void recordM2SStall();
    void recordS2MStall();
    void reserveS2MResp(uint64_t flits);
    void consumeS2MRespReservation(uint64_t flits);
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
