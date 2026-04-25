/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mem/cxl_mem_link.hh"

#include <algorithm>
#include <cmath>

#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/CxlMemLink.hh"

namespace gem5
{

bool
CxlMemLink::ProtocolMessage::dataBearing() const
{
    return dataSlotsTotal != 0 || trailerSlotsTotal != 0;
}

uint64_t
CxlMemLink::ProtocolMessage::remainingDataSlots() const
{
    return dataSlotsTotal - dataSlotsSent;
}

uint64_t
CxlMemLink::ProtocolMessage::remainingTrailerSlots() const
{
    return trailerSlotsTotal - trailerSlotsSent;
}

bool
CxlMemLink::ProtocolMessage::complete() const
{
    return headerSent && remainingDataSlots() == 0 &&
           remainingTrailerSlots() == 0;
}

CxlMemLink::DirectionState::DirectionState(CxlMemLink &link,
                                           LinkDirection _direction,
                                           Tick _base_latency,
                                           uint64_t _queue_depth,
                                           const std::string &event_name)
    : direction(_direction),
      baseLatency(_base_latency),
      queueDepthFlits(_queue_depth),
      emitEvent(
          [&link, _direction] {
              link.processDirectionFlit(link.directionState(_direction));
          },
          event_name)
{}

CxlMemLink::CxlRequestPort::CxlRequestPort(const std::string &name,
                                           CxlMemLink &_link, PortID _port_id)
    : RequestPort(name),
      link(_link),
      portId(_port_id),
      sendEvent([this] { trySendTiming(); }, name)
{}

CxlMemLink::CxlResponsePort::CxlResponsePort(const std::string &name,
                                             CxlMemLink &_link,
                                             PortID _port_id,
                                             const AddrRange &_range)
    : ResponsePort(name),
      link(_link),
      portId(_port_id),
      ranges({_range}),
      retryReq(false),
      sendEvent([this] { trySendTiming(); }, name)
{}

CxlMemLink::CxlMemLink(const Params &p)
    : ClockedObject(p),
      portRanges(p.port_ranges.begin(), p.port_ranges.end()),
      flitSizeBytes(p.flit_size_bytes),
      bandwidth(p.bandwidth),
      m2sLatency(p.m2s_latency),
      s2mLatency(p.s2m_latency),
      requestHeaderFlits(p.request_header_flits),
      responseHeaderFlits(p.response_header_flits),
      m2sQueueDepthFlits(p.m2s_queue_depth_flits),
      s2mQueueDepthFlits(p.s2m_queue_depth_flits),
      m2sState(*this, LinkDirection::M2S, m2sLatency, m2sQueueDepthFlits,
               csprintf("%s.m2s_emit", name())),
      s2mState(*this, LinkDirection::S2M, s2mLatency, s2mQueueDepthFlits,
               csprintf("%s.s2m_emit", name())),
      lastM2SQueueUpdate(0),
      lastS2MQueueUpdate(0),
      reservedS2MFlits(0),
      ADD_STAT(m2sPackets, statistics::units::Count::get(),
               "CXL.mem M2S packets accepted by the link"),
      ADD_STAT(s2mPackets, statistics::units::Count::get(),
               "CXL.mem S2M packets accepted by the link"),
      ADD_STAT(m2sFlits, statistics::units::Count::get(),
               "CXL.mem M2S 256B flits emitted by the link"),
      ADD_STAT(s2mFlits, statistics::units::Count::get(),
               "CXL.mem S2M 256B flits emitted by the link"),
      ADD_STAT(m2sQueueWaitTicks, statistics::units::Tick::get(),
               "M2S FIFO wait before the first flit containing the packet"),
      ADD_STAT(s2mQueueWaitTicks, statistics::units::Tick::get(),
               "S2M FIFO wait before the first flit containing the packet"),
      ADD_STAT(m2sSerializationTicks, statistics::units::Tick::get(),
               "M2S serialization time from first to last emitted flit"),
      ADD_STAT(s2mSerializationTicks, statistics::units::Tick::get(),
               "S2M serialization time from first to last emitted flit"),
      ADD_STAT(m2sBaseLatencyTicks, statistics::units::Tick::get(),
               "M2S fixed base link latency"),
      ADD_STAT(s2mBaseLatencyTicks, statistics::units::Tick::get(),
               "S2M fixed base link latency"),
      ADD_STAT(m2sTotalDelayTicks, statistics::units::Tick::get(),
               "Total M2S CXL delay"),
      ADD_STAT(s2mTotalDelayTicks, statistics::units::Tick::get(),
               "Total S2M CXL delay"),
      ADD_STAT(m2sQueueOccupancyFlitTicks, statistics::units::Tick::get(),
               "M2S queued flit occupancy integrated over time"),
      ADD_STAT(s2mQueueOccupancyFlitTicks, statistics::units::Tick::get(),
               "S2M queued flit occupancy integrated over time"),
      ADD_STAT(m2sFullEvents, statistics::units::Count::get(),
               "Requests rejected because the M2S FIFO was full"),
      ADD_STAT(s2mFullEvents, statistics::units::Count::get(),
               "Requests rejected because the S2M FIFO reservation was full")
{
    fatal_if(!use256BFlitPacker(),
             "CxlMemLink real flit packer currently supports only 256B mode");
    fatal_if(requestHeaderFlits != 1 || responseHeaderFlits != 1,
             "CxlMemLink 256B packer expects one request header and one "
             "response header flit parameter");
    fatal_if(m2sQueueDepthFlits == 0 || s2mQueueDepthFlits == 0,
             "CxlMemLink FIFO depths must be non-zero");

    const auto cpu_port_count = p.port_cpu_side_ports_connection_count;
    const auto mem_port_count = p.port_mem_side_ports_connection_count;
    fatal_if(cpu_port_count == 0 || mem_port_count == 0,
             "CxlMemLink requires at least one CPU-side and one mem-side "
             "port connection");
    fatal_if(cpu_port_count != mem_port_count,
             "CxlMemLink requires matching CPU-side and mem-side port "
             "counts, got %d CPU-side and %d mem-side ports",
             cpu_port_count, mem_port_count);
    fatal_if(portRanges.size() != cpu_port_count,
             "CxlMemLink requires one port range per CPU-side port, got "
             "%zu ranges for %d ports",
             portRanges.size(), cpu_port_count);

    memSidePorts.reserve(mem_port_count);
    cpuSidePorts.reserve(cpu_port_count);

    for (PortID i = 0; i < mem_port_count; ++i) {
        memSidePorts.emplace_back(csprintf("%s.mem_side_ports[%d]", name(), i),
                                  *this, i);
    }
    for (PortID i = 0; i < cpu_port_count; ++i) {
        cpuSidePorts.emplace_back(csprintf("%s.cpu_side_ports[%d]", name(), i),
                                  *this, i, portRanges[i]);
    }
}

Port &
CxlMemLink::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_side_ports" && idx < memSidePorts.size()) {
        return memSidePorts[idx];
    }
    if (if_name == "cpu_side_ports" && idx < cpuSidePorts.size()) {
        return cpuSidePorts[idx];
    }
    return ClockedObject::getPort(if_name, idx);
}

void
CxlMemLink::init()
{
    fatal_if(cpuSidePorts.empty() || memSidePorts.empty(),
             "CxlMemLink must have connected ports\n");

    for (PortID i = 0; i < cpuSidePorts.size(); ++i) {
        if (!cpuSidePorts[i].isConnected() || !memSidePorts[i].isConnected()) {
            fatal("Both ports of CxlMemLink channel %d must be connected.\n",
                  i);
        }
        cpuSidePorts[i].sendRangeChange();
    }
}

bool
CxlMemLink::use256BFlitPacker() const
{
    return flitSizeBytes == 256;
}

uint64_t
CxlMemLink::serializationUnitBytes() const
{
    return flitSizeBytes;
}

Tick
CxlMemLink::serializationDelay(uint64_t flits) const
{
    return static_cast<Tick>(std::ceil(
        static_cast<double>(flits * serializationUnitBytes()) * bandwidth));
}

const char *
CxlMemLink::messageClassName(MessageClass msg_class) const
{
    switch (msg_class) {
        case MessageClass::M2SReq:
            return "M2S Req";
        case MessageClass::M2SRwD:
            return "M2S RwD";
        case MessageClass::S2MNDR:
            return "S2M NDR";
        case MessageClass::S2MDRS:
            return "S2M DRS";
    }

    panic("Unreachable CXL.mem message class");
    return "unknown";
}

const char *
CxlMemLink::directionName(LinkDirection direction) const
{
    switch (direction) {
        case LinkDirection::M2S:
            return "M2S";
        case LinkDirection::S2M:
            return "S2M";
    }

    panic("Unreachable CXL.mem link direction");
    return "unknown";
}

Addr
CxlMemLink::debugPacketAddr(PacketPtr pkt) const
{
    return (pkt && pkt->req && pkt->req->hasPaddr()) ? pkt->req->getPaddr()
                                                     : 0;
}

CxlMemLink::MessageClass
CxlMemLink::m2sMessageClass(PacketPtr pkt) const
{
    return pkt->isWrite() ? MessageClass::M2SRwD : MessageClass::M2SReq;
}

CxlMemLink::MessageClass
CxlMemLink::s2mMessageClass(PacketPtr pkt) const
{
    return pkt->hasData() ? MessageClass::S2MDRS : MessageClass::S2MNDR;
}

uint64_t
CxlMemLink::dataSlots(PacketPtr pkt) const
{
    const uint64_t bytes = pkt->getSize();
    return std::max<uint64_t>(1, (bytes + 15) / 16);
}

uint64_t
CxlMemLink::trailerSlots(MessageClass msg_class, PacketPtr pkt) const
{
    if (msg_class == MessageClass::M2SRwD && pkt->isMaskedWrite()) {
        return 1;
    }
    return 0;
}

uint64_t
CxlMemLink::reservedFlits(MessageClass msg_class, PacketPtr pkt) const
{
    if (!isDataBearing(msg_class)) {
        return 1;
    }

    const uint64_t follow_slots =
        dataSlots(pkt) + trailerSlots(msg_class, pkt);
    if (follow_slots <= 14) {
        return 1;
    }

    return 1 + ((follow_slots - 14) + 13) / 14;
}

uint64_t
CxlMemLink::reservedS2MRespFlits(PacketPtr pkt) const
{
    return reservedFlits(
        pkt->hasRespData() ? MessageClass::S2MDRS : MessageClass::S2MNDR, pkt);
}

Tick
CxlMemLink::standaloneMessageDelay(MessageClass msg_class, PacketPtr pkt) const
{
    return serializationDelay(reservedFlits(msg_class, pkt));
}

CxlMemLink::DirectionState &
CxlMemLink::directionState(LinkDirection direction)
{
    return direction == LinkDirection::M2S ? m2sState : s2mState;
}

const CxlMemLink::DirectionState &
CxlMemLink::directionState(LinkDirection direction) const
{
    return direction == LinkDirection::M2S ? m2sState : s2mState;
}

void
CxlMemLink::accountQueueOccupancy(DirectionState &state)
{
    if (state.direction == LinkDirection::M2S) {
        accountM2SQueueOccupancy(state.queuedFlits);
    } else {
        accountS2MQueueOccupancy(state.queuedFlits);
    }
}

bool
CxlMemLink::queueCanFit(const DirectionState &state, uint64_t flits) const
{
    return state.queuedFlits + flits <= state.queueDepthFlits;
}

void
CxlMemLink::enqueueMessage(DirectionState &state, ProtocolMessagePtr msg)
{
    accountQueueOccupancy(state);
    DPRINTF(CxlMemLink,
            "enqueue %s %s addr %#x port %d arrival %llu flits %llu "
            "queued_before %llu active %d pending %llu\n",
            directionName(state.direction), messageClassName(msg->msgClass),
            debugPacketAddr(msg->pkt), msg->portId,
            static_cast<unsigned long long>(msg->arrivalTick),
            static_cast<unsigned long long>(msg->reservedFlits),
            static_cast<unsigned long long>(state.queuedFlits),
            state.activeDataMsg ? 1 : 0,
            static_cast<unsigned long long>(state.pending.size()));
    state.queuedFlits += msg->reservedFlits;
    const Tick arrival_tick = msg->arrivalTick;
    state.pending.push_back(std::move(msg));
    maybeScheduleFlit(state, arrival_tick);
}

void
CxlMemLink::dequeueMessage(DirectionState &state, const ProtocolMessage &msg)
{
    accountQueueOccupancy(state);
    panic_if(state.queuedFlits < msg.reservedFlits,
             "CxlMemLink %s underflow in %s queued flits", name(),
             state.direction == LinkDirection::M2S ? "M2S" : "S2M");
    state.queuedFlits -= msg.reservedFlits;
}

void
CxlMemLink::maybeScheduleFlit(DirectionState &state, Tick when)
{
    if (state.emitEvent.scheduled()) {
        return;
    }

    if (state.pending.empty() && !state.activeDataMsg) {
        return;
    }

    Tick schedule_tick = std::max(when, state.nextFlitTick);
    if (!state.activeDataMsg && !state.pending.empty()) {
        schedule_tick =
            std::max(schedule_tick, state.pending.front()->arrivalTick);
    }
    DPRINTF(
        CxlMemLink,
        "schedule %s flit at %llu next %llu pending %llu active %d "
        "queued %llu\n",
        directionName(state.direction),
        static_cast<unsigned long long>(std::max(schedule_tick, curTick())),
        static_cast<unsigned long long>(state.nextFlitTick),
        static_cast<unsigned long long>(state.pending.size()),
        state.activeDataMsg ? 1 : 0,
        static_cast<unsigned long long>(state.queuedFlits));
    schedule(state.emitEvent, std::max(schedule_tick, curTick()));
}

void
CxlMemLink::touchMessageFlit(ProtocolMessage &msg, Tick flit_tick)
{
    if (msg.emittedFlits == 0 || msg.lastFlitTick != flit_tick) {
        ++msg.emittedFlits;
        msg.lastFlitTick = flit_tick;
    }
}

void
CxlMemLink::markHeaderSent(ProtocolMessage &msg, Tick flit_tick)
{
    touchMessageFlit(msg, flit_tick);
    if (!msg.headerSent) {
        msg.headerSent = true;
        msg.serviceStartTick = flit_tick;
    }
}

void
CxlMemLink::consumeContinuationSlot(ProtocolMessage &msg, Tick flit_tick)
{
    touchMessageFlit(msg, flit_tick);
    if (msg.remainingDataSlots() != 0) {
        ++msg.dataSlotsSent;
    } else {
        panic_if(msg.remainingTrailerSlots() == 0,
                 "CxlMemLink %s consumed continuation slot after completion",
                 name());
        ++msg.trailerSlotsSent;
    }
}

void
CxlMemLink::completeMessage(DirectionState &state, ProtocolMessagePtr msg,
                            Tick completion_tick)
{
    msg->completionTick = completion_tick;
    DPRINTF(CxlMemLink,
            "complete %s %s addr %#x port %d start %llu done %llu "
            "flits %llu data %llu/%llu trailer %llu/%llu queued_before %llu\n",
            directionName(state.direction), messageClassName(msg->msgClass),
            debugPacketAddr(msg->pkt), msg->portId,
            static_cast<unsigned long long>(msg->serviceStartTick),
            static_cast<unsigned long long>(completion_tick),
            static_cast<unsigned long long>(msg->emittedFlits),
            static_cast<unsigned long long>(msg->dataSlotsSent),
            static_cast<unsigned long long>(msg->dataSlotsTotal),
            static_cast<unsigned long long>(msg->trailerSlotsSent),
            static_cast<unsigned long long>(msg->trailerSlotsTotal),
            static_cast<unsigned long long>(state.queuedFlits));
    dequeueMessage(state, *msg);

    LinkSchedule schedule;
    schedule.queueWait = msg->serviceStartTick - msg->arrivalTick;
    schedule.serialization = completion_tick - msg->serviceStartTick;
    schedule.baseLatency = state.baseLatency;
    schedule.readyTick = completion_tick + state.baseLatency;

    if (state.direction == LinkDirection::M2S) {
        recordM2SPacket(msg->emittedFlits, schedule);
        DPRINTF(CxlMemLink, "send %s %s addr %#x to mem_side[%d] at %llu\n",
                directionName(state.direction),
                messageClassName(msg->msgClass), debugPacketAddr(msg->pkt),
                msg->portId,
                static_cast<unsigned long long>(schedule.readyTick));
        memSidePort(msg->portId).schedTimingReq(msg->pkt, schedule.readyTick);
    } else {
        recordS2MPacket(msg->emittedFlits, schedule);
        DPRINTF(CxlMemLink, "send %s %s addr %#x to cpu_side[%d] at %llu\n",
                directionName(state.direction),
                messageClassName(msg->msgClass), debugPacketAddr(msg->pkt),
                msg->portId,
                static_cast<unsigned long long>(schedule.readyTick));
        cpuSidePort(msg->portId).schedTimingResp(msg->pkt, schedule.readyTick);
    }
}

int
CxlMemLink::slotGroup(int slot) const
{
    return std::min(slot / 4, 3);
}

int
CxlMemLink::groupCountIndex(MessageClass msg_class) const
{
    switch (msg_class) {
        case MessageClass::M2SReq:
        case MessageClass::S2MNDR:
            return 0;
        case MessageClass::M2SRwD:
        case MessageClass::S2MDRS:
            return 1;
    }

    panic("Unreachable CXL.mem message class");
    return 0;
}

uint32_t
CxlMemLink::maxGroupMessages(MessageClass msg_class) const
{
    switch (msg_class) {
        case MessageClass::M2SReq:
            return 4;
        case MessageClass::M2SRwD:
            return 2;
        case MessageClass::S2MNDR:
            return 6;
        case MessageClass::S2MDRS:
            return 3;
    }

    panic("Unreachable CXL.mem message class");
    return 0;
}

bool
CxlMemLink::isDataBearing(MessageClass msg_class) const
{
    return msg_class == MessageClass::M2SRwD ||
           msg_class == MessageClass::S2MDRS;
}

bool
CxlMemLink::isSlotLegal(MessageClass msg_class, bool header_slot) const
{
    switch (msg_class) {
        case MessageClass::M2SReq:
            return true;
        case MessageClass::M2SRwD:
        case MessageClass::S2MDRS:
            return true;
        case MessageClass::S2MNDR:
            return true;
    }

    panic("Unreachable CXL.mem message class");
    return false;
}

bool
CxlMemLink::canStartDataHeader(const ProtocolMessage &msg,
                               bool header_slot) const
{
    if (!msg.dataBearing()) {
        return false;
    }

    if (!header_slot) {
        return true;
    }

    const uint64_t remaining =
        msg.remainingDataSlots() + msg.remainingTrailerSlots();
    return remaining <= 16;
}

bool
CxlMemLink::canPackGroupMessage(const DirectionState &state,
                                const FlitBuildState &flit,
                                MessageClass msg_class, int slot,
                                uint32_t count) const
{
    const int index = groupCountIndex(msg_class);
    const int group = slotGroup(slot);
    const uint32_t limit = maxGroupMessages(msg_class);
    const uint32_t current = flit.groupCounts[index][group];

    if (current + count > limit) {
        return false;
    }

    if (group == 0) {
        return state.prevTailCounts[index] + current + count <= limit;
    }

    return flit.groupCounts[index][group - 1] + current + count <= limit;
}

void
CxlMemLink::recordGroupMessage(FlitBuildState &flit, MessageClass msg_class,
                               int slot, uint32_t count)
{
    flit.groupCounts[groupCountIndex(msg_class)][slotGroup(slot)] += count;
}

uint32_t
CxlMemLink::packNdrHeaders(DirectionState &state, FlitBuildState &flit,
                           int slot, Tick flit_start, Tick flit_end)
{
    const bool header_slot = slot == 0;
    const uint32_t slot_capacity = header_slot ? 2 : 3;
    uint32_t packed = 0;

    while (packed < slot_capacity && !state.pending.empty()) {
        const auto &msg = state.pending.front();
        if (msg->arrivalTick > flit_start ||
            msg->msgClass != MessageClass::S2MNDR) {
            break;
        }
        if (!canPackGroupMessage(state, flit, msg->msgClass, slot,
                                 packed + 1)) {
            break;
        }

        auto completed = std::move(state.pending.front());
        state.pending.pop_front();
        markHeaderSent(*completed, flit_start);
        recordGroupMessage(flit, completed->msgClass, slot, 1);
        completeMessage(state, std::move(completed), flit_end);
        ++packed;
    }

    return packed;
}

uint32_t
CxlMemLink::packCompleteDataMessage(DirectionState &state,
                                    FlitBuildState &flit, int slot,
                                    Tick flit_start, Tick flit_end)
{
    if (state.pending.empty()) {
        return 0;
    }

    const auto &msg = state.pending.front();
    if (msg->arrivalTick > flit_start || !msg->dataBearing()) {
        return 0;
    }

    const uint64_t follow_slots =
        msg->remainingDataSlots() + msg->remainingTrailerSlots();
    const uint64_t available_follow_slots = 14 - slot;
    if (follow_slots > available_follow_slots) {
        return 0;
    }
    if (!canPackGroupMessage(state, flit, msg->msgClass, slot, 1)) {
        return 0;
    }

    auto completed = std::move(state.pending.front());
    state.pending.pop_front();
    markHeaderSent(*completed, flit_start);
    recordGroupMessage(flit, completed->msgClass, slot, 1);
    ++flit.dataHeaderStarts;

    for (uint64_t i = 0; i < follow_slots; ++i) {
        consumeContinuationSlot(*completed, flit_start);
    }
    panic_if(!completed->complete(),
             "CxlMemLink %s failed to complete an in-flit data message",
             name());
    completeMessage(state, std::move(completed), flit_end);

    return 1 + follow_slots;
}

bool
CxlMemLink::startDataHeader(DirectionState &state, FlitBuildState &flit,
                            int slot, Tick flit_start)
{
    if (state.pending.empty()) {
        return false;
    }

    const auto &msg = state.pending.front();
    const bool header_slot = slot == 0;
    if (msg->arrivalTick > flit_start || !msg->dataBearing()) {
        return false;
    }
    const uint64_t follow_slots =
        msg->remainingDataSlots() + msg->remainingTrailerSlots();
    if (follow_slots <= 14 &&
        follow_slots > static_cast<uint64_t>(14 - slot)) {
        return false;
    }
    if (!canStartDataHeader(*msg, header_slot)) {
        panic("CxlMemLink %s cannot legally start %s in an H-slot with "
              "%llu remaining follow-on slots",
              name(),
              msg->msgClass == MessageClass::M2SRwD ? "M2S RwD" : "S2M DRS",
              static_cast<unsigned long long>(msg->remainingDataSlots() +
                                              msg->remainingTrailerSlots()));
    }
    if (flit.dataHeaderStarts != 0 ||
        !canPackGroupMessage(state, flit, msg->msgClass, slot, 1)) {
        return false;
    }

    state.activeDataMsg = std::move(state.pending.front());
    state.pending.pop_front();
    markHeaderSent(*state.activeDataMsg, flit_start);
    recordGroupMessage(flit, state.activeDataMsg->msgClass, slot, 1);
    ++flit.dataHeaderStarts;
    return true;
}

bool
CxlMemLink::packReqHeader(DirectionState &state, FlitBuildState &flit,
                          int slot, Tick flit_start, Tick flit_end)
{
    if (state.pending.empty()) {
        return false;
    }

    const auto &msg = state.pending.front();
    if (msg->arrivalTick > flit_start ||
        msg->msgClass != MessageClass::M2SReq ||
        !canPackGroupMessage(state, flit, msg->msgClass, slot, 1)) {
        return false;
    }

    auto completed = std::move(state.pending.front());
    state.pending.pop_front();
    markHeaderSent(*completed, flit_start);
    recordGroupMessage(flit, completed->msgClass, slot, 1);
    completeMessage(state, std::move(completed), flit_end);
    return true;
}

uint32_t
CxlMemLink::packHeaderSlot(DirectionState &state, FlitBuildState &flit,
                           int slot, Tick flit_start, Tick flit_end)
{
    if (state.pending.empty()) {
        return 0;
    }

    const auto &msg = state.pending.front();
    if (msg->arrivalTick > flit_start) {
        return 0;
    }

    switch (msg->msgClass) {
        case MessageClass::M2SReq:
            return packReqHeader(state, flit, slot, flit_start, flit_end) ? 1
                                                                          : 0;
        case MessageClass::S2MNDR:
            return packNdrHeaders(state, flit, slot, flit_start, flit_end) != 0
                       ? 1
                       : 0;
        case MessageClass::M2SRwD:
        case MessageClass::S2MDRS:
            if (const uint32_t consumed = packCompleteDataMessage(
                    state, flit, slot, flit_start, flit_end)) {
                return consumed;
            }
            return startDataHeader(state, flit, slot, flit_start) ? 1 : 0;
    }

    panic("Unreachable CXL.mem message class");
    return 0;
}

void
CxlMemLink::processDirectionFlit(DirectionState &state)
{
    if (state.pending.empty() && !state.activeDataMsg) {
        return;
    }

    const Tick flit_start = std::max(curTick(), state.nextFlitTick);
    const Tick flit_end = flit_start + serializationDelay(1);
    FlitBuildState flit;
    bool emitted_payload = false;
    DPRINTF(CxlMemLink,
            "emit %s flit start %llu end %llu pending %llu active_addr %#x "
            "queued %llu\n",
            directionName(state.direction),
            static_cast<unsigned long long>(flit_start),
            static_cast<unsigned long long>(flit_end),
            static_cast<unsigned long long>(state.pending.size()),
            state.activeDataMsg ? debugPacketAddr(state.activeDataMsg->pkt)
                                : 0,
            static_cast<unsigned long long>(state.queuedFlits));

    for (int slot = 0; slot < 15;) {
        if (slot == 0 && state.activeDataMsg) {
            ++slot;
            continue;
        }

        if (slot > 0 && state.activeDataMsg) {
            consumeContinuationSlot(*state.activeDataMsg, flit_start);
            emitted_payload = true;
            if (state.activeDataMsg->complete()) {
                auto completed = std::move(state.activeDataMsg);
                completeMessage(state, std::move(completed), flit_end);
            }
            ++slot;
            continue;
        }

        if (const uint32_t consumed =
                packHeaderSlot(state, flit, slot, flit_start, flit_end)) {
            emitted_payload = true;
            slot += consumed;
        } else {
            ++slot;
        }
    }

    panic_if(!emitted_payload,
             "CxlMemLink %s emitted an empty protocol flit while work was "
             "pending",
             name());

    if (state.direction == LinkDirection::M2S) {
        ++m2sFlits;
    } else {
        ++s2mFlits;
    }

    state.nextFlitTick = flit_end;
    for (int i = 0; i < 2; ++i) {
        state.prevTailCounts[i] = flit.groupCounts[i][3];
    }
    DPRINTF(CxlMemLink,
            "emit %s flit done pending %llu active_addr %#x queued %llu "
            "tail[%u,%u]\n",
            directionName(state.direction),
            static_cast<unsigned long long>(state.pending.size()),
            state.activeDataMsg ? debugPacketAddr(state.activeDataMsg->pkt)
                                : 0,
            static_cast<unsigned long long>(state.queuedFlits),
            state.prevTailCounts[0], state.prevTailCounts[1]);

    if (!state.pending.empty() || state.activeDataMsg) {
        maybeScheduleFlit(state, flit_end);
    }

    retryStalledReqs();
}

void
CxlMemLink::accountM2SQueueOccupancy(uint64_t queued_flits)
{
    m2sQueueOccupancyFlitTicks +=
        queued_flits * (curTick() - lastM2SQueueUpdate);
    lastM2SQueueUpdate = curTick();
}

void
CxlMemLink::accountS2MQueueOccupancy(uint64_t queued_flits)
{
    s2mQueueOccupancyFlitTicks +=
        queued_flits * (curTick() - lastS2MQueueUpdate);
    lastS2MQueueUpdate = curTick();
}

void
CxlMemLink::recordM2SPacket(uint64_t flits, const LinkSchedule &schedule)
{
    [[maybe_unused]] const uint64_t emitted_flits = flits;
    ++m2sPackets;
    m2sQueueWaitTicks += schedule.queueWait;
    m2sSerializationTicks += schedule.serialization;
    m2sBaseLatencyTicks += schedule.baseLatency;
    m2sTotalDelayTicks +=
        schedule.queueWait + schedule.serialization + schedule.baseLatency;
}

void
CxlMemLink::recordS2MPacket(uint64_t flits, const LinkSchedule &schedule)
{
    [[maybe_unused]] const uint64_t emitted_flits = flits;
    ++s2mPackets;
    s2mQueueWaitTicks += schedule.queueWait;
    s2mSerializationTicks += schedule.serialization;
    s2mBaseLatencyTicks += schedule.baseLatency;
    s2mTotalDelayTicks +=
        schedule.queueWait + schedule.serialization + schedule.baseLatency;
}

void
CxlMemLink::recordM2SStall()
{
    ++m2sFullEvents;
}

void
CxlMemLink::recordS2MStall()
{
    ++s2mFullEvents;
}

void
CxlMemLink::reserveS2MResp(uint64_t flits)
{
    accountS2MQueueOccupancy(s2mState.queuedFlits);
    reservedS2MFlits += flits;
}

void
CxlMemLink::consumeS2MRespReservation(uint64_t flits)
{
    panic_if(reservedS2MFlits < flits,
             "CxlMemLink %s received a %llu-flit response with only "
             "%llu reserved S2M flits\n",
             name(), static_cast<unsigned long long>(flits),
             static_cast<unsigned long long>(reservedS2MFlits));
    reservedS2MFlits -= flits;
}

void
CxlMemLink::retryStalledReqs()
{
    for (auto &port : cpuSidePorts) {
        if (port.retryReq) {
            port.retryReq = false;
            port.sendRetryReq();
        }
    }
}

CxlMemLink::CxlRequestPort &
CxlMemLink::memSidePort(PortID port_id)
{
    assert(port_id < memSidePorts.size());
    return memSidePorts[port_id];
}

const CxlMemLink::CxlRequestPort &
CxlMemLink::memSidePort(PortID port_id) const
{
    assert(port_id < memSidePorts.size());
    return memSidePorts[port_id];
}

CxlMemLink::CxlResponsePort &
CxlMemLink::cpuSidePort(PortID port_id)
{
    assert(port_id < cpuSidePorts.size());
    return cpuSidePorts[port_id];
}

const CxlMemLink::CxlResponsePort &
CxlMemLink::cpuSidePort(PortID port_id) const
{
    assert(port_id < cpuSidePorts.size());
    return cpuSidePorts[port_id];
}

bool
CxlMemLink::CxlResponsePort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(CxlMemLink, "recvTimingReq[%d]: %s addr %#x size %u\n", portId,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    panic_if(pkt->cacheResponding(), "CxlMemLink should not see packets "
                                     "where a cache is already responding");

    if (retryReq) {
        return false;
    }

    const auto msg_class = link.m2sMessageClass(pkt);
    const uint64_t req_flits = link.reservedFlits(msg_class, pkt);
    const bool expects_response = pkt->needsResponse();
    const uint64_t resp_flits =
        expects_response ? link.reservedS2MRespFlits(pkt) : 0;

    panic_if(req_flits > link.m2sQueueDepthFlits,
             "CxlMemLink %s M2S packet requires %llu flits but FIFO depth "
             "is only %llu flits\n",
             link.name(), static_cast<unsigned long long>(req_flits),
             static_cast<unsigned long long>(link.m2sQueueDepthFlits));
    panic_if(resp_flits > link.s2mQueueDepthFlits,
             "CxlMemLink %s S2M response requires %llu flits but FIFO depth "
             "is only %llu flits\n",
             link.name(), static_cast<unsigned long long>(resp_flits),
             static_cast<unsigned long long>(link.s2mQueueDepthFlits));

    if (!link.queueCanFit(link.m2sState, req_flits)) {
        link.recordM2SStall();
        retryReq = true;
        return false;
    }
    if (expects_response &&
        link.s2mState.queuedFlits + link.reservedS2MFlits + resp_flits >
            link.s2mQueueDepthFlits) {
        link.recordS2MStall();
        retryReq = true;
        return false;
    }

    if (expects_response) {
        link.reserveS2MResp(resp_flits);
        DPRINTF(CxlMemLink,
                "reserve S2M response addr %#x req_flits %llu resp_flits "
                "%llu reserved %llu queued %llu\n",
                link.debugPacketAddr(pkt),
                static_cast<unsigned long long>(req_flits),
                static_cast<unsigned long long>(resp_flits),
                static_cast<unsigned long long>(link.reservedS2MFlits),
                static_cast<unsigned long long>(link.s2mState.queuedFlits));
    }

    const Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;
    const Tick arrival_tick = curTick() + receive_delay;
    auto msg = std::make_unique<ProtocolMessage>(
        msg_class, pkt, portId, arrival_tick,
        link.isDataBearing(msg_class) ? link.dataSlots(pkt) : 0,
        link.trailerSlots(msg_class, pkt), req_flits);
    link.enqueueMessage(link.m2sState, std::move(msg));
    return true;
}

void
CxlMemLink::CxlRequestPort::schedTimingReq(PacketPtr pkt, Tick when)
{
    if (transmitList.empty()) {
        link.schedule(sendEvent, when);
    }

    DPRINTF(CxlMemLink,
            "queue mem_side[%d] req addr %#x at %llu txq_before %llu\n",
            portId, link.debugPacketAddr(pkt),
            static_cast<unsigned long long>(when),
            static_cast<unsigned long long>(transmitList.size()));
    transmitList.emplace_back(pkt, when);
}

void
CxlMemLink::CxlResponsePort::schedTimingResp(PacketPtr pkt, Tick when)
{
    if (transmitList.empty()) {
        link.schedule(sendEvent, when);
    }

    DPRINTF(CxlMemLink,
            "queue cpu_side[%d] resp addr %#x at %llu txq_before %llu\n",
            portId, link.debugPacketAddr(pkt),
            static_cast<unsigned long long>(when),
            static_cast<unsigned long long>(transmitList.size()));
    transmitList.emplace_back(pkt, when);
}

void
CxlMemLink::CxlRequestPort::trySendTiming()
{
    assert(!transmitList.empty());

    const DeferredPacket req = transmitList.front();
    if (req.tick > curTick()) {
        DPRINTF(CxlMemLink,
                "defer mem_side[%d] req addr %#x until %llu current %llu\n",
                portId, link.debugPacketAddr(req.pkt),
                static_cast<unsigned long long>(req.tick),
                static_cast<unsigned long long>(curTick()));
        return;
    }

    if (sendTimingReq(req.pkt)) {
        DPRINTF(CxlMemLink, "sent mem_side[%d] req addr %#x txq_before %llu\n",
                portId, link.debugPacketAddr(req.pkt),
                static_cast<unsigned long long>(transmitList.size()));
        transmitList.pop_front();

        if (!transmitList.empty()) {
            const DeferredPacket next_req = transmitList.front();
            link.schedule(sendEvent, std::max(next_req.tick, curTick()));
        }

        link.retryStalledReqs();
    } else {
        DPRINTF(CxlMemLink, "blocked mem_side[%d] req addr %#x txq %llu\n",
                portId, link.debugPacketAddr(req.pkt),
                static_cast<unsigned long long>(transmitList.size()));
    }
}

void
CxlMemLink::CxlResponsePort::trySendTiming()
{
    assert(!transmitList.empty());

    const DeferredPacket resp = transmitList.front();
    if (resp.tick > curTick()) {
        DPRINTF(CxlMemLink,
                "defer cpu_side[%d] resp addr %#x until %llu current %llu\n",
                portId, link.debugPacketAddr(resp.pkt),
                static_cast<unsigned long long>(resp.tick),
                static_cast<unsigned long long>(curTick()));
        return;
    }

    if (sendTimingResp(resp.pkt)) {
        DPRINTF(CxlMemLink,
                "sent cpu_side[%d] resp addr %#x txq_before %llu\n", portId,
                link.debugPacketAddr(resp.pkt),
                static_cast<unsigned long long>(transmitList.size()));
        transmitList.pop_front();

        if (!transmitList.empty()) {
            const DeferredPacket next_resp = transmitList.front();
            link.schedule(sendEvent, std::max(next_resp.tick, curTick()));
        }

        link.retryStalledReqs();
    } else {
        DPRINTF(CxlMemLink, "blocked cpu_side[%d] resp addr %#x txq %llu\n",
                portId, link.debugPacketAddr(resp.pkt),
                static_cast<unsigned long long>(transmitList.size()));
    }
}

bool
CxlMemLink::CxlRequestPort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(CxlMemLink, "recvTimingResp[%d]: %s addr %#x size %u\n", portId,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    const auto msg_class = link.s2mMessageClass(pkt);
    const uint64_t flits = link.reservedFlits(msg_class, pkt);
    panic_if(flits > link.s2mQueueDepthFlits,
             "CxlMemLink %s response requires %llu flits but FIFO depth is "
             "only %llu flits\n",
             link.name(), static_cast<unsigned long long>(flits),
             static_cast<unsigned long long>(link.s2mQueueDepthFlits));

    link.consumeS2MRespReservation(flits);
    DPRINTF(CxlMemLink,
            "consume S2M reservation addr %#x flits %llu reserved_left %llu "
            "queued %llu\n",
            link.debugPacketAddr(pkt), static_cast<unsigned long long>(flits),
            static_cast<unsigned long long>(link.reservedS2MFlits),
            static_cast<unsigned long long>(link.s2mState.queuedFlits));
    panic_if(!link.queueCanFit(link.s2mState, flits),
             "CxlMemLink %s S2M response exceeded actual queue capacity after "
             "reservation",
             link.name());

    const Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;
    const Tick arrival_tick = curTick() + receive_delay;
    auto msg = std::make_unique<ProtocolMessage>(
        msg_class, pkt, portId, arrival_tick,
        link.isDataBearing(msg_class) ? link.dataSlots(pkt) : 0,
        link.trailerSlots(msg_class, pkt), flits);
    link.enqueueMessage(link.s2mState, std::move(msg));
    return true;
}

void
CxlMemLink::CxlRequestPort::recvReqRetry()
{
    trySendTiming();
}

void
CxlMemLink::CxlResponsePort::recvRespRetry()
{
    trySendTiming();
}

Tick
CxlMemLink::CxlResponsePort::recvAtomic(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "CxlMemLink should not see packets "
                                     "where a cache is already responding");

    Tick latency = link.m2sLatency +
                   link.standaloneMessageDelay(link.m2sMessageClass(pkt), pkt);
    const bool expects_response = pkt->needsResponse();
    latency += link.memSidePort(portId).sendAtomic(pkt);
    if (expects_response) {
        latency += link.s2mLatency +
                   link.standaloneMessageDelay(link.s2mMessageClass(pkt), pkt);
    }
    return latency;
}

Tick
CxlMemLink::CxlResponsePort::recvAtomicBackdoor(PacketPtr pkt,
                                                MemBackdoorPtr &backdoor)
{
    Tick latency = link.m2sLatency +
                   link.standaloneMessageDelay(link.m2sMessageClass(pkt), pkt);
    const bool expects_response = pkt->needsResponse();
    latency += link.memSidePort(portId).sendAtomicBackdoor(pkt, backdoor);
    if (expects_response) {
        latency += link.s2mLatency +
                   link.standaloneMessageDelay(link.s2mMessageClass(pkt), pkt);
    }
    return latency;
}

void
CxlMemLink::CxlResponsePort::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(name());

    for (const auto &deferred : transmitList) {
        if (pkt->trySatisfyFunctional(deferred.pkt)) {
            pkt->makeResponse();
            return;
        }
    }

    if (link.memSidePort(portId).trySatisfyFunctional(pkt)) {
        return;
    }

    pkt->popLabel();
    link.memSidePort(portId).sendFunctional(pkt);
}

void
CxlMemLink::CxlResponsePort::recvMemBackdoorReq(const MemBackdoorReq &req,
                                                MemBackdoorPtr &backdoor)
{
    link.memSidePort(portId).sendMemBackdoorReq(req, backdoor);
}

bool
CxlMemLink::CxlRequestPort::trySatisfyFunctional(PacketPtr pkt)
{
    for (const auto &deferred : transmitList) {
        if (pkt->trySatisfyFunctional(deferred.pkt)) {
            pkt->makeResponse();
            return true;
        }
    }

    return false;
}

void
CxlMemLink::CxlRequestPort::recvRangeChange()
{
    link.cpuSidePort(portId).sendRangeChange();
}

AddrRangeList
CxlMemLink::CxlResponsePort::getAddrRanges() const
{
    return ranges;
}

} // namespace gem5
