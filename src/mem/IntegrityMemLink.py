# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause

from m5.objects.ClockedObject import ClockedObject
from m5.params import *


class IntegrityMemLink(ClockedObject):
    type = "IntegrityMemLink"
    cxx_header = "mem/integrity_mem_link.hh"
    cxx_class = "gem5::IntegrityMemLink"

    mem_side_port = RequestPort(
        "Port that sends data and hidden MAC requests toward memory"
    )
    cpu_side_port = ResponsePort(
        "Port that receives guest-visible requests from the cache hierarchy"
    )

    visible_range = Param.AddrRange(
        "Guest-visible data range advertised by this integrity link"
    )
    enable = Param.Bool(False, "Enable hidden MAC timing requests")
    mac_line_bytes = Param.Unsigned(64, "Protected data line size in bytes")
    mac_bytes_per_line = Param.Unsigned(
        8, "Hidden MAC bytes associated with each protected data line"
    )
    request_queue_size = Param.Unsigned(
        64, "Maximum queued downstream packets"
    )
    response_queue_size = Param.Unsigned(
        64, "Maximum protected requests waiting for paired responses"
    )
