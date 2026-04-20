# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause

from m5.objects.ClockedObject import ClockedObject
from m5.params import *


class CxlMemLink(ClockedObject):
    type = "CxlMemLink"
    cxx_header = "mem/cxl_mem_link.hh"
    cxx_class = "gem5::CxlMemLink"

    mem_side_port = RequestPort(
        "This port sends requests toward the memory device and receives responses"
    )
    cpu_side_port = ResponsePort(
        "This port receives host requests and sends responses back"
    )

    ranges = VectorParam.AddrRange(
        [AllMemory], "Host physical address ranges forwarded through this link"
    )

    flit_size_bytes = Param.Unsigned(
        256, "CXL.cachemem flit payload mode to model, usually 68 or 256 bytes"
    )
    bandwidth = Param.MemoryBandwidth(
        "64GiB/s", "Per-direction CXL.mem serialization bandwidth"
    )
    m2s_latency = Param.Latency(
        "0ns", "Optional fixed host-to-device CXL.mem link latency"
    )
    s2m_latency = Param.Latency(
        "0ns", "Optional fixed device-to-host CXL.mem link latency"
    )

    request_header_flits = Param.Unsigned(
        1, "Simplified M2S CXL.mem request/control overhead in flits"
    )
    response_header_flits = Param.Unsigned(
        1, "Simplified S2M CXL.mem response/control overhead in flits"
    )
    m2s_queue_depth_flits = Param.Unsigned(
        256, "Host-to-device FIFO capacity in CXL flits"
    )
    s2m_queue_depth_flits = Param.Unsigned(
        256, "Device-to-host FIFO capacity in CXL flits"
    )
