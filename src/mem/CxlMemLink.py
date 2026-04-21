# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause

from m5.objects.ClockedObject import ClockedObject
from m5.params import *


class CxlMemLink(ClockedObject):
    type = "CxlMemLink"
    cxx_header = "mem/cxl_mem_link.hh"
    cxx_class = "gem5::CxlMemLink"

    mem_side_ports = VectorRequestPort(
        "Ports that send requests toward backing media controllers and "
        "receive responses"
    )
    cpu_side_ports = VectorResponsePort(
        "Ports that receive host requests and send responses back"
    )

    port_ranges = VectorParam.AddrRange(
        [AllMemory],
        "One forwarded host physical address range per CPU-side ingress port",
    )

    flit_size_bytes = Param.Unsigned(
        256,
        "CXL.cachemem link mode to model, usually 68 or 256 bytes; in 256B "
        "mode the implementation serializes payload at 16B slot granularity",
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
        1,
        "Simplified M2S CXL.mem header cost in serialized units; in 256B "
        "mode one unit corresponds to one 16B slot",
    )
    response_header_flits = Param.Unsigned(
        1,
        "Simplified S2M CXL.mem header cost in serialized units; in 256B "
        "mode one unit corresponds to one 16B slot",
    )
    m2s_queue_depth_flits = Param.Unsigned(
        256, "Host-to-device FIFO capacity in CXL flits"
    )
    s2m_queue_depth_flits = Param.Unsigned(
        256, "Device-to-host FIFO capacity in CXL flits"
    )
