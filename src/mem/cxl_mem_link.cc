/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mem/cxl_mem_link.hh"

#include <algorithm>
#include <cmath>

#include "base/logging.hh"
#include "debug/CxlMemLink.hh"

namespace gem5
{

CxlMemLink::CxlRequestPort::CxlRequestPort(const std::string &name,
                                           CxlMemLink &_link,
                                           CxlResponsePort &_cpu_side_port)
    : RequestPort(name),
      link(_link),
      cpuSidePort(_cpu_side_port),
      queuedFlits(0),
      sendEvent([this] { trySendTiming(); }, name)
{}

CxlMemLink::CxlResponsePort::CxlResponsePort(
    const std::string &name, CxlMemLink &_link, CxlRequestPort &_mem_side_port,
    const std::vector<AddrRange> &_ranges)
    : ResponsePort(name),
      link(_link),
      memSidePort(_mem_side_port),
      ranges(_ranges.begin(), _ranges.end()),
      queuedFlits(0),
      reservedRespFlits(0),
      retryReq(false),
      sendEvent([this] { trySendTiming(); }, name)
{}

CxlMemLink::CxlMemLink(const Params &p)
    : ClockedObject(p),
      memSidePort(p.name + ".mem_side_port", *this, cpuSidePort),
      cpuSidePort(p.name + ".cpu_side_port", *this, memSidePort, p.ranges),
      ranges(p.ranges.begin(), p.ranges.end()),
      flitSizeBytes(p.flit_size_bytes),
      bandwidth(p.bandwidth),
      m2sLatency(p.m2s_latency),
      s2mLatency(p.s2m_latency),
      requestHeaderFlits(p.request_header_flits),
      responseHeaderFlits(p.response_header_flits),
      m2sQueueDepthFlits(p.m2s_queue_depth_flits),
      s2mQueueDepthFlits(p.s2m_queue_depth_flits),
      nextM2SReady(0),
      nextS2MReady(0),
      lastM2SQueueUpdate(0),
      lastS2MQueueUpdate(0),
      ADD_STAT(m2sPackets, statistics::units::Count::get(),
               "CXL.mem M2S packets accepted by the link"),
      ADD_STAT(s2mPackets, statistics::units::Count::get(),
               "CXL.mem S2M packets accepted by the link"),
      ADD_STAT(m2sFlits, statistics::units::Count::get(),
               "CXL.mem M2S flits serialized by the link"),
      ADD_STAT(s2mFlits, statistics::units::Count::get(),
               "CXL.mem S2M flits serialized by the link"),
      ADD_STAT(m2sQueueWaitTicks, statistics::units::Tick::get(),
               "M2S FIFO wait before serialization starts"),
      ADD_STAT(s2mQueueWaitTicks, statistics::units::Tick::get(),
               "S2M FIFO wait before serialization starts"),
      ADD_STAT(m2sSerializationTicks, statistics::units::Tick::get(),
               "M2S serialization delay"),
      ADD_STAT(s2mSerializationTicks, statistics::units::Tick::get(),
               "S2M serialization delay"),
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
    fatal_if(flitSizeBytes == 0, "CxlMemLink flit size must be non-zero");
    fatal_if(requestHeaderFlits == 0 || responseHeaderFlits == 0,
             "CxlMemLink header flit counts must be non-zero");
    fatal_if(m2sQueueDepthFlits == 0 || s2mQueueDepthFlits == 0,
             "CxlMemLink FIFO depths must be non-zero");
}

Port &
CxlMemLink::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_side_port") {
        return memSidePort;
    }
    if (if_name == "cpu_side_port") {
        return cpuSidePort;
    }
    return ClockedObject::getPort(if_name, idx);
}

void
CxlMemLink::init()
{
    if (!cpuSidePort.isConnected() || !memSidePort.isConnected()) {
        fatal("Both ports of a CxlMemLink must be connected.\n");
    }

    cpuSidePort.sendRangeChange();
}

uint64_t
CxlMemLink::dataFlits(PacketPtr pkt) const
{
    const uint64_t bytes = pkt->getSize();
    return (bytes + flitSizeBytes - 1) / flitSizeBytes;
}

uint64_t
CxlMemLink::m2sRequestFlits(PacketPtr pkt) const
{
    uint64_t flits = requestHeaderFlits;

    if (pkt->isWrite()) {
        flits += dataFlits(pkt);
    }

    return flits;
}

uint64_t
CxlMemLink::s2mResponseFlitsForRequest(PacketPtr pkt) const
{
    uint64_t flits = responseHeaderFlits;

    if (pkt->hasRespData()) {
        flits += dataFlits(pkt);
    }

    return flits;
}

uint64_t
CxlMemLink::s2mResponseFlits(PacketPtr pkt) const
{
    uint64_t flits = responseHeaderFlits;

    if (pkt->hasData()) {
        flits += dataFlits(pkt);
    }

    return flits;
}

Tick
CxlMemLink::serializationDelay(uint64_t flits) const
{
    return static_cast<Tick>(
        std::ceil(static_cast<double>(flits * flitSizeBytes) * bandwidth));
}

CxlMemLink::LinkSchedule
CxlMemLink::scheduleM2S(Tick arrival, uint64_t flits)
{
    LinkSchedule schedule;
    const Tick start = std::max(arrival, nextM2SReady);
    schedule.queueWait = start - arrival;
    schedule.serialization = serializationDelay(flits);
    schedule.baseLatency = m2sLatency;
    nextM2SReady = start + schedule.serialization;
    schedule.readyTick = nextM2SReady + schedule.baseLatency;
    return schedule;
}

CxlMemLink::LinkSchedule
CxlMemLink::scheduleS2M(Tick arrival, uint64_t flits)
{
    LinkSchedule schedule;
    const Tick start = std::max(arrival, nextS2MReady);
    schedule.queueWait = start - arrival;
    schedule.serialization = serializationDelay(flits);
    schedule.baseLatency = s2mLatency;
    nextS2MReady = start + schedule.serialization;
    schedule.readyTick = nextS2MReady + schedule.baseLatency;
    return schedule;
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
    ++m2sPackets;
    m2sFlits += flits;
    m2sQueueWaitTicks += schedule.queueWait;
    m2sSerializationTicks += schedule.serialization;
    m2sBaseLatencyTicks += schedule.baseLatency;
    m2sTotalDelayTicks +=
        schedule.queueWait + schedule.serialization + schedule.baseLatency;
}

void
CxlMemLink::recordS2MPacket(uint64_t flits, const LinkSchedule &schedule)
{
    ++s2mPackets;
    s2mFlits += flits;
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

bool
CxlMemLink::CxlRequestPort::reqQueueCanFit(uint64_t flits) const
{
    return queuedFlits + flits <= link.m2sQueueDepthFlits;
}

bool
CxlMemLink::CxlResponsePort::respQueueCanFit(uint64_t flits) const
{
    return queuedFlits + reservedRespFlits + flits <= link.s2mQueueDepthFlits;
}

void
CxlMemLink::CxlResponsePort::reserveResp(uint64_t flits)
{
    link.accountS2MQueueOccupancy(queuedFlits);
    reservedRespFlits += flits;
}

void
CxlMemLink::CxlResponsePort::consumeRespReservation(uint64_t flits)
{
    panic_if(reservedRespFlits < flits,
             "CxlMemLink %s received a %llu-flit response with only "
             "%llu reserved S2M flits\n",
             link.name(), flits, reservedRespFlits);
    reservedRespFlits -= flits;
}

bool
CxlMemLink::CxlResponsePort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(CxlMemLink, "recvTimingReq: %s addr %#x size %u\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    panic_if(pkt->cacheResponding(), "CxlMemLink should not see packets "
                                     "where a cache is already responding");

    if (retryReq) {
        return false;
    }

    const uint64_t req_flits = link.m2sRequestFlits(pkt);
    const bool expects_response = pkt->needsResponse();
    const uint64_t resp_flits =
        expects_response ? link.s2mResponseFlitsForRequest(pkt) : 0;

    panic_if(req_flits > link.m2sQueueDepthFlits,
             "CxlMemLink %s M2S packet requires %llu flits but FIFO depth "
             "is only %llu flits\n",
             link.name(), req_flits, link.m2sQueueDepthFlits);
    panic_if(resp_flits > link.s2mQueueDepthFlits,
             "CxlMemLink %s S2M response requires %llu flits but FIFO depth "
             "is only %llu flits\n",
             link.name(), resp_flits, link.s2mQueueDepthFlits);

    if (!memSidePort.reqQueueCanFit(req_flits)) {
        link.recordM2SStall();
        retryReq = true;
    } else if (expects_response && !respQueueCanFit(resp_flits)) {
        link.recordS2MStall();
        retryReq = true;
    } else {
        if (expects_response) {
            reserveResp(resp_flits);
        }

        const Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
        pkt->headerDelay = pkt->payloadDelay = 0;
        const auto schedule =
            link.scheduleM2S(curTick() + receive_delay, req_flits);
        link.recordM2SPacket(req_flits, schedule);
        memSidePort.schedTimingReq(pkt, schedule.readyTick, req_flits);
    }

    return !retryReq;
}

void
CxlMemLink::CxlRequestPort::schedTimingReq(PacketPtr pkt, Tick when,
                                           uint64_t flits)
{
    if (transmitList.empty()) {
        link.schedule(sendEvent, when);
    }

    link.accountM2SQueueOccupancy(queuedFlits);
    queuedFlits += flits;
    transmitList.emplace_back(pkt, when, flits);
}

void
CxlMemLink::CxlResponsePort::schedTimingResp(PacketPtr pkt, Tick when,
                                             uint64_t flits)
{
    if (transmitList.empty()) {
        link.schedule(sendEvent, when);
    }

    link.accountS2MQueueOccupancy(queuedFlits);
    queuedFlits += flits;
    transmitList.emplace_back(pkt, when, flits);
}

void
CxlMemLink::CxlRequestPort::trySendTiming()
{
    assert(!transmitList.empty());

    const DeferredPacket req = transmitList.front();
    assert(req.tick <= curTick());

    if (sendTimingReq(req.pkt)) {
        transmitList.pop_front();

        link.accountM2SQueueOccupancy(queuedFlits);
        assert(queuedFlits >= req.flits);
        queuedFlits -= req.flits;

        if (!transmitList.empty()) {
            const DeferredPacket next_req = transmitList.front();
            link.schedule(sendEvent, std::max(next_req.tick, curTick()));
        }

        cpuSidePort.retryStalledReq();
    }
}

void
CxlMemLink::CxlResponsePort::trySendTiming()
{
    assert(!transmitList.empty());

    const DeferredPacket resp = transmitList.front();
    assert(resp.tick <= curTick());

    if (sendTimingResp(resp.pkt)) {
        transmitList.pop_front();

        link.accountS2MQueueOccupancy(queuedFlits);
        assert(queuedFlits >= resp.flits);
        queuedFlits -= resp.flits;

        if (!transmitList.empty()) {
            const DeferredPacket next_resp = transmitList.front();
            link.schedule(sendEvent, std::max(next_resp.tick, curTick()));
        }

        if (retryReq && memSidePort.queuedFlits < link.m2sQueueDepthFlits) {
            retryReq = false;
            sendRetryReq();
        }
    }
}

bool
CxlMemLink::CxlRequestPort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(CxlMemLink, "recvTimingResp: %s addr %#x size %u\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    const uint64_t flits = link.s2mResponseFlits(pkt);
    panic_if(flits > link.s2mQueueDepthFlits,
             "CxlMemLink %s response requires %llu flits but FIFO depth is "
             "only %llu flits\n",
             link.name(), flits, link.s2mQueueDepthFlits);

    cpuSidePort.consumeRespReservation(flits);

    const Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;
    const auto schedule = link.scheduleS2M(curTick() + receive_delay, flits);
    link.recordS2MPacket(flits, schedule);
    cpuSidePort.schedTimingResp(pkt, schedule.readyTick, flits);

    return true;
}

void
CxlMemLink::CxlResponsePort::retryStalledReq()
{
    if (retryReq) {
        retryReq = false;
        sendRetryReq();
    }
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

    Tick latency =
        link.m2sLatency + link.serializationDelay(link.m2sRequestFlits(pkt));
    const bool expects_response = pkt->needsResponse();
    latency += memSidePort.sendAtomic(pkt);
    if (expects_response) {
        latency += link.s2mLatency +
                   link.serializationDelay(link.s2mResponseFlits(pkt));
    }
    return latency;
}

Tick
CxlMemLink::CxlResponsePort::recvAtomicBackdoor(PacketPtr pkt,
                                                MemBackdoorPtr &backdoor)
{
    Tick latency =
        link.m2sLatency + link.serializationDelay(link.m2sRequestFlits(pkt));
    const bool expects_response = pkt->needsResponse();
    latency += memSidePort.sendAtomicBackdoor(pkt, backdoor);
    if (expects_response) {
        latency += link.s2mLatency +
                   link.serializationDelay(link.s2mResponseFlits(pkt));
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

    if (memSidePort.trySatisfyFunctional(pkt)) {
        return;
    }

    pkt->popLabel();
    memSidePort.sendFunctional(pkt);
}

void
CxlMemLink::CxlResponsePort::recvMemBackdoorReq(const MemBackdoorReq &req,
                                                MemBackdoorPtr &backdoor)
{
    memSidePort.sendMemBackdoorReq(req, backdoor);
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
    cpuSidePort.sendRangeChange();
}

AddrRangeList
CxlMemLink::CxlResponsePort::getAddrRanges() const
{
    return ranges;
}

} // namespace gem5
