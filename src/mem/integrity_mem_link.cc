/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mem/integrity_mem_link.hh"

#include <algorithm>
#include <cstring>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/IntegrityMemLink.hh"

namespace gem5
{

IntegrityMemLink::MemoryPort::MemoryPort(const std::string &name,
                                         IntegrityMemLink &_link)
    : RequestPort(name),
      link(_link),
      sendEvent([this] { trySendTiming(); }, name)
{}

IntegrityMemLink::CpuPort::CpuPort(const std::string &name,
                                   IntegrityMemLink &_link,
                                   const AddrRange &_range)
    : ResponsePort(name),
      link(_link),
      ranges({_range}),
      retryReq(false),
      sendEvent([this] { trySendTiming(); }, name)
{}

IntegrityMemLink::IntegrityMemLink(const Params &p)
    : ClockedObject(p),
      memSidePort(p.name + ".mem_side_port", *this),
      cpuSidePort(p.name + ".cpu_side_port", *this, p.visible_range),
      visibleRange(p.visible_range),
      enabled(p.enable),
      macLineBytes(p.mac_line_bytes),
      macBytesPerLine(p.mac_bytes_per_line),
      requestQueueSize(p.request_queue_size),
      responseQueueSize(p.response_queue_size),
      ADD_STAT(integrityMacReadReqs, statistics::units::Count::get(),
               "Hidden integrity MAC read requests issued"),
      ADD_STAT(integrityMacWriteReqs, statistics::units::Count::get(),
               "Hidden integrity MAC write requests issued"),
      ADD_STAT(integrityMacReadBytes, statistics::units::Byte::get(),
               "Hidden integrity MAC read bytes issued"),
      ADD_STAT(integrityMacWriteBytes, statistics::units::Byte::get(),
               "Hidden integrity MAC write bytes issued"),
      ADD_STAT(integrityMacPairedReads, statistics::units::Count::get(),
               "Guest-visible reads paired with hidden MAC reads"),
      ADD_STAT(integrityMacPairedWrites, statistics::units::Count::get(),
               "Guest-visible writes paired with hidden MAC writes"),
      ADD_STAT(integrityMacRejectedReqs, statistics::units::Count::get(),
               "Guest-visible requests rejected by the integrity link")
{
    fatal_if(macLineBytes == 0 || macBytesPerLine == 0,
             "IntegrityMemLink MAC line and MAC sizes must be non-zero");
    fatal_if(requestQueueSize < 2,
             "IntegrityMemLink request_queue_size must be at least 2");
    fatal_if(responseQueueSize == 0,
             "IntegrityMemLink response_queue_size must be non-zero");
}

Port &
IntegrityMemLink::getPort(const std::string &if_name, PortID idx)
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
IntegrityMemLink::init()
{
    fatal_if(!cpuSidePort.isConnected() || !memSidePort.isConnected(),
             "IntegrityMemLink requires both ports to be connected");
    cpuSidePort.sendRangeChange();
}

bool
IntegrityMemLink::shouldProtect(PacketPtr pkt) const
{
    return enabled && pkt->isRequest() && (pkt->isRead() || pkt->isWrite()) &&
           pkt->getSize() != 0 && visibleRange.contains(pkt->getAddr());
}

uint64_t
IntegrityMemLink::macBytesFor(PacketPtr pkt) const
{
    const Addr start = pkt->getAddr();
    const Addr end = start + pkt->getSize() - 1;
    const uint64_t first_line = (start - visibleRange.start()) / macLineBytes;
    const uint64_t last_line = (end - visibleRange.start()) / macLineBytes;
    return (last_line - first_line + 1) * macBytesPerLine;
}

PacketPtr
IntegrityMemLink::makeMacPacket(PacketPtr data_pkt) const
{
    const uint64_t mac_bytes = macBytesFor(data_pkt);
    Request::Flags flags;
    flags.set(Request::NO_ACCESS);
    auto req = std::make_shared<Request>(data_pkt->getAddr(), mac_bytes, flags,
                                         data_pkt->req->requestorId());

    PacketPtr mac_pkt = new Packet(req, data_pkt->isRead() ? MemCmd::ReadReq
                                                           : MemCmd::WriteReq);
    mac_pkt->allocate();
    mac_pkt->qosValue(data_pkt->qosValue());

    if (mac_pkt->isWrite()) {
        std::memset(mac_pkt->getPtr<uint8_t>(), 0, mac_pkt->getSize());
    }

    return mac_pkt;
}

std::unique_ptr<IntegrityMemLink::ProtectedRequest>
IntegrityMemLink::makeProtectedRequest(PacketPtr pkt)
{
    auto request = std::make_unique<ProtectedRequest>();
    request->dataPkt = pkt;
    request->macPkt = makeMacPacket(pkt);
    pkt->pushSenderState(new IntegritySenderState(request.get(), false));
    request->macPkt->pushSenderState(
        new IntegritySenderState(request.get(), true));
    return request;
}

void
IntegrityMemLink::accountMac(PacketPtr pkt)
{
    if (pkt->isRead()) {
        ++integrityMacReadReqs;
        integrityMacReadBytes += pkt->getSize();
    } else {
        ++integrityMacWriteReqs;
        integrityMacWriteBytes += pkt->getSize();
    }
}

void
IntegrityMemLink::enqueueDownstream(PacketPtr pkt, Tick when)
{
    memSidePort.schedTimingReq(pkt, when);
}

bool
IntegrityMemLink::MemoryPort::reqQueueFull(unsigned needed) const
{
    return transmitList.size() + needed > link.requestQueueSize;
}

void
IntegrityMemLink::MemoryPort::schedTimingReq(PacketPtr pkt, Tick when)
{
    if (transmitList.empty()) {
        link.schedule(sendEvent, when);
    }

    transmitList.emplace_back(pkt, when);
}

void
IntegrityMemLink::MemoryPort::trySendTiming()
{
    assert(!transmitList.empty());

    const DeferredPacket req = transmitList.front();
    assert(req.tick <= curTick());

    PacketPtr pkt = req.pkt;
    DPRINTF(IntegrityMemLink, "trySend request %s addr %#x size %u\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    if (sendTimingReq(pkt)) {
        transmitList.pop_front();

        if (!transmitList.empty()) {
            const DeferredPacket next_req = transmitList.front();
            link.schedule(sendEvent, std::max(next_req.tick, curTick()));
        }

        link.cpuSidePort.retryStalledReq();
    }
}

bool
IntegrityMemLink::MemoryPort::recvTimingResp(PacketPtr pkt)
{
    auto *state = dynamic_cast<IntegritySenderState *>(pkt->senderState);
    if (state) {
        pkt->popSenderState();
        link.handleProtectedResp(pkt, state);
        delete state;
        return true;
    }

    link.cpuSidePort.schedTimingResp(pkt, curTick() + pkt->headerDelay +
                                              pkt->payloadDelay);
    pkt->headerDelay = pkt->payloadDelay = 0;
    return true;
}

void
IntegrityMemLink::MemoryPort::recvReqRetry()
{
    trySendTiming();
}

void
IntegrityMemLink::MemoryPort::recvRangeChange()
{
    link.cpuSidePort.sendRangeChange();
}

bool
IntegrityMemLink::MemoryPort::trySatisfyFunctional(PacketPtr pkt)
{
    for (const auto &deferred : transmitList) {
        if (pkt->trySatisfyFunctional(deferred.pkt)) {
            pkt->makeResponse();
            return true;
        }
    }
    return false;
}

bool
IntegrityMemLink::CpuPort::respQueueFull() const
{
    return transmitList.size() >= link.responseQueueSize;
}

void
IntegrityMemLink::CpuPort::schedTimingResp(PacketPtr pkt, Tick when)
{
    if (transmitList.empty()) {
        link.schedule(sendEvent, when);
    }

    transmitList.emplace_back(pkt, when);
}

void
IntegrityMemLink::CpuPort::trySendTiming()
{
    assert(!transmitList.empty());

    const DeferredPacket resp = transmitList.front();
    assert(resp.tick <= curTick());

    PacketPtr pkt = resp.pkt;
    DPRINTF(IntegrityMemLink, "trySend response %s addr %#x size %u\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    if (sendTimingResp(pkt)) {
        transmitList.pop_front();

        if (!transmitList.empty()) {
            const DeferredPacket next_resp = transmitList.front();
            link.schedule(sendEvent, std::max(next_resp.tick, curTick()));
        }

        retryStalledReq();
    }
}

void
IntegrityMemLink::CpuPort::retryStalledReq()
{
    if (retryReq && !respQueueFull() && !link.memSidePort.reqQueueFull(2) &&
        link.protectedRequests.size() < link.responseQueueSize) {
        retryReq = false;
        sendRetryReq();
    }
}

bool
IntegrityMemLink::CpuPort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(IntegrityMemLink, "recvTimingReq %s addr %#x size %u\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    if (retryReq) {
        return false;
    }

    const bool protect = link.shouldProtect(pkt);
    const unsigned needed = protect ? 2 : 1;
    if (link.memSidePort.reqQueueFull(needed) ||
        (protect && link.protectedRequests.size() >= link.responseQueueSize) ||
        respQueueFull()) {
        retryReq = true;
        ++link.integrityMacRejectedReqs;
        return false;
    }

    Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;
    const Tick when = curTick() + receive_delay;

    if (!protect) {
        link.enqueueDownstream(pkt, when);
        return true;
    }

    auto request = link.makeProtectedRequest(pkt);
    PacketPtr mac_pkt = request->macPkt;
    link.accountMac(mac_pkt);

    if (pkt->isRead()) {
        ++link.integrityMacPairedReads;
    } else {
        ++link.integrityMacPairedWrites;
    }

    link.protectedRequests.emplace_back(std::move(request));
    link.enqueueDownstream(pkt, when);
    link.enqueueDownstream(mac_pkt, when);
    return true;
}

void
IntegrityMemLink::CpuPort::recvRespRetry()
{
    trySendTiming();
}

Tick
IntegrityMemLink::recvAtomicProtected(PacketPtr pkt)
{
    PacketPtr mac_pkt = makeMacPacket(pkt);
    accountMac(mac_pkt);
    if (pkt->isRead()) {
        ++integrityMacPairedReads;
    } else {
        ++integrityMacPairedWrites;
    }

    const Tick data_latency = memSidePort.sendAtomic(pkt);
    const Tick mac_latency = memSidePort.sendAtomic(mac_pkt);
    delete mac_pkt;
    return std::max(data_latency, mac_latency);
}

Tick
IntegrityMemLink::CpuPort::recvAtomic(PacketPtr pkt)
{
    if (link.shouldProtect(pkt)) {
        return link.recvAtomicProtected(pkt);
    }
    return link.memSidePort.sendAtomic(pkt);
}

Tick
IntegrityMemLink::CpuPort::recvAtomicBackdoor(PacketPtr pkt,
                                              MemBackdoorPtr &backdoor)
{
    return link.memSidePort.sendAtomicBackdoor(pkt, backdoor);
}

void
IntegrityMemLink::CpuPort::recvFunctional(PacketPtr pkt)
{
    if (link.memSidePort.trySatisfyFunctional(pkt)) {
        return;
    }
    link.memSidePort.sendFunctional(pkt);
}

void
IntegrityMemLink::CpuPort::recvMemBackdoorReq(const MemBackdoorReq &req,
                                              MemBackdoorPtr &backdoor)
{
    link.memSidePort.sendMemBackdoorReq(req, backdoor);
}

AddrRangeList
IntegrityMemLink::CpuPort::getAddrRanges() const
{
    return ranges;
}

void
IntegrityMemLink::handleProtectedResp(PacketPtr pkt,
                                      IntegritySenderState *state)
{
    ProtectedRequest *request = state->request;
    if (state->mac) {
        request->macDone = true;
        delete pkt;
    } else {
        request->dataDone = true;
        request->dataPkt = pkt;
    }
    maybeComplete(request);
}

void
IntegrityMemLink::maybeComplete(ProtectedRequest *request)
{
    if (!request->dataDone || !request->macDone) {
        return;
    }

    PacketPtr pkt = request->dataPkt;
    const Tick when = curTick() + pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;
    cpuSidePort.schedTimingResp(pkt, when);
    eraseProtectedRequest(request);
}

void
IntegrityMemLink::eraseProtectedRequest(ProtectedRequest *request)
{
    for (auto it = protectedRequests.begin(); it != protectedRequests.end();
         ++it) {
        if (it->get() == request) {
            protectedRequests.erase(it);
            return;
        }
    }
    panic("IntegrityMemLink lost a protected request");
}

} // namespace gem5
