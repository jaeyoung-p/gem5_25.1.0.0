# Copyright (c) 2026
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Fixed two-tier ordinary RAM memory component."""

from typing import (
    Dict,
    List,
    Sequence,
    Tuple,
)

from m5.objects import (
    AbstractMemory,
    CxlMemLink,
    MemCtrl,
)
from m5.params import (
    AddrRange,
    Port,
)
from m5.util.convert import toMemorySize

from ...utils.override import overrides
from ..boards.abstract_board import AbstractBoard
from .abstract_memory_system import AbstractMemorySystem
from .dram_interfaces.ddr5 import DDR5_6400_4x8_32GiB


class TwoTierMemory(AbstractMemorySystem):
    """Local 8-channel DDR5 node plus a CXL-like memory-only node.

    Node 0 is one uniform 8-channel DDR5 memory system split into low and high
    guest physical ranges only because x86 reserves the 3-4 GiB PCI hole. The
    low and high halves use identical interleaving and timing, but separate
    AbstractMemory objects so KVM backs exactly the ranges Linux sees. Node 1 is
    a 64 GiB high-address memory-only node reached through one shared CXL.mem
    bottleneck before its two backing DDR5 media controllers.
    """

    _node0_low_start = 0
    _node0_low_size = "3GiB"
    _node0_high_start = 0x100000000
    _node0_high_size = "61GiB"
    _node1_start = 0x1040000000
    _node1_size = "64GiB"
    _node0_channels = 8
    _node1_channels = 2
    _interleaving_size = 64

    def __init__(
        self,
        cxl_flit_size_bytes: int = 256,
        cxl_link_bandwidth: str = "64GiB/s",
        cxl_base_latency: str = "0ns",
        cxl_queue_depth_flits: int = 256,
    ) -> None:
        super().__init__()

        object.__setattr__(
            self,
            "node0_ranges",
            [
                AddrRange(
                    start=self._node0_low_start, size=self._node0_low_size
                ),
                AddrRange(
                    start=self._node0_high_start, size=self._node0_high_size
                ),
            ],
        )
        object.__setattr__(
            self,
            "node1_ranges",
            [AddrRange(start=self._node1_start, size=self._node1_size)],
        )
        object.__setattr__(self, "node0_channel_ranges", [])
        object.__setattr__(self, "node1_channel_ranges", [])

        self.node0_low_ctrls = self._create_channel_group(
            num_channels=self._node0_channels,
            static_latency="10ns",
        )
        self.node0_high_ctrls = self._create_channel_group(
            num_channels=self._node0_channels,
            static_latency="10ns",
        )
        object.__setattr__(
            self,
            "_node0_ctrls",
            list(self.node0_low_ctrls) + list(self.node0_high_ctrls),
        )
        self.slow_ctrls = self._create_channel_group(
            num_channels=self._node1_channels,
            static_latency="10ns",
        )
        self.slow_cxl_link = CxlMemLink(
            flit_size_bytes=cxl_flit_size_bytes,
            bandwidth=cxl_link_bandwidth,
            m2s_latency=cxl_base_latency,
            s2m_latency=cxl_base_latency,
            m2s_queue_depth_flits=cxl_queue_depth_flits,
            s2m_queue_depth_flits=cxl_queue_depth_flits,
        )
        for channel in range(self._node1_channels):
            self.slow_cxl_link.mem_side_ports = self.slow_ctrls[channel].port

        self.set_memory_range(self.get_default_memory_ranges())

    def _create_channel_group(
        self, num_channels: int, static_latency: str
    ) -> List[MemCtrl]:
        ctrls = [
            MemCtrl(dram=DDR5_6400_4x8_32GiB()) for _ in range(num_channels)
        ]

        for ctrl in ctrls:
            ctrl.static_frontend_latency = static_latency
            ctrl.static_backend_latency = static_latency

        return ctrls

    def _interleaved_range(
        self, memory_range: AddrRange, channel: int, num_channels: int
    ) -> AddrRange:
        intlv_bits = num_channels.bit_length() - 1
        intlv_low_bit = self._interleaving_size.bit_length() - 1

        return AddrRange(
            start=memory_range.start,
            size=memory_range.size(),
            intlvHighBit=intlv_low_bit + intlv_bits - 1,
            xorHighBit=0,
            intlvBits=intlv_bits,
            intlvMatch=channel,
        )

    def _interleave_range(
        self, ctrls: List[MemCtrl], memory_range: AddrRange, num_channels: int
    ) -> List[AddrRange]:
        channel_ranges = []
        for channel, ctrl in enumerate(ctrls):
            channel_range = self._interleaved_range(
                memory_range, channel, num_channels
            )
            ctrl.dram.range = channel_range
            channel_ranges.append(channel_range)
        return channel_ranges

    def _configure_slow_cxl_link(self) -> None:
        slow_channel_ranges = [
            self._interleaved_range(
                self.node1_ranges[0], channel, self._node1_channels
            )
            for channel in range(self._node1_channels)
        ]

        self.slow_cxl_link.port_ranges = slow_channel_ranges

        object.__setattr__(self, "node1_channel_ranges", slow_channel_ranges)

    def _all_controllers(self) -> List[MemCtrl]:
        return self._node0_ctrls + list(self.slow_ctrls)

    def _all_drams(self) -> List[AbstractMemory]:
        return [ctrl.dram for ctrl in self._all_controllers()]

    def get_default_memory_ranges(self) -> List[AddrRange]:
        return self.node0_ranges + self.node1_ranges

    def get_numa_memory_ranges(self) -> Dict[int, List[AddrRange]]:
        return {
            0: self.node0_ranges,
            1: self.node1_ranges,
        }

    def get_node0_controller(self) -> MemCtrl:
        return self.get_node0_controllers()[0]

    def get_slow_controller(self) -> MemCtrl:
        return self.get_slow_controllers()[0]

    def get_node0_controllers(self) -> List[MemCtrl]:
        return self._node0_ctrls

    def get_slow_controllers(self) -> List[MemCtrl]:
        return self.slow_ctrls

    def get_node0_port(self) -> Port:
        return self._node0_ctrls[0].port

    def get_slow_port(self) -> Port:
        return self.slow_cxl_link.cpu_side_ports[0]

    @overrides(AbstractMemorySystem)
    def incorporate_memory(self, board: AbstractBoard) -> None:
        for channels in (self._node0_channels, self._node1_channels):
            if channels <= 0 or (channels & (channels - 1)):
                raise ValueError(
                    "TwoTierMemory channel counts must be powers of 2"
                )
        if self._interleaving_size < int(board.get_cache_line_size()):
            raise ValueError(
                "TwoTierMemory interleaving size can not be smaller than "
                "the board cache line size"
            )

        pass

    @overrides(AbstractMemorySystem)
    def get_mem_ports(self) -> Sequence[Tuple[object, Port]]:
        return [
            (
                self.node0_channel_ranges[channel],
                self._node0_ctrls[channel].port,
            )
            for channel in range(len(self._node0_ctrls))
        ] + [
            (
                self.node1_channel_ranges[channel],
                self.slow_cxl_link.cpu_side_ports[channel],
            )
            for channel in range(self._node1_channels)
        ]

    @overrides(AbstractMemorySystem)
    def get_memory_controllers(self) -> List[MemCtrl]:
        return self._all_controllers()

    @overrides(AbstractMemorySystem)
    def get_mem_interfaces(self) -> List[AbstractMemory]:
        return self._all_drams()

    @overrides(AbstractMemorySystem)
    def get_size(self) -> int:
        return self._size

    @overrides(AbstractMemorySystem)
    def get_uninterleaved_range(self) -> List[AddrRange]:
        return self.get_default_memory_ranges()

    @overrides(AbstractMemorySystem)
    def set_memory_range(self, ranges: List[AddrRange]) -> None:
        expected = [
            (self._node0_low_start, toMemorySize(self._node0_low_size)),
            (self._node0_high_start, toMemorySize(self._node0_high_size)),
            (self._node1_start, toMemorySize(self._node1_size)),
        ]

        if len(ranges) != len(expected) or any(
            int(mem_range.start) != start or mem_range.size() != size
            for mem_range, (start, size) in zip(ranges, expected)
        ):
            raise ValueError(
                "TwoTierMemory requires fixed RAM ranges totaling 64GiB "
                "for node 0 across [0, 3GiB) plus [4GiB, 65GiB), "
                "and 64GiB for node 1 at [65GiB, 129GiB)."
            )

        object.__setattr__(self, "node0_ranges", [ranges[0], ranges[1]])
        object.__setattr__(self, "node1_ranges", [ranges[2]])
        self._size = sum(mem_range.size() for mem_range in ranges)

        node0_low_channel_ranges = self._interleave_range(
            self.node0_low_ctrls, self.node0_ranges[0], self._node0_channels
        )
        node0_high_channel_ranges = self._interleave_range(
            self.node0_high_ctrls, self.node0_ranges[1], self._node0_channels
        )
        object.__setattr__(
            self,
            "node0_channel_ranges",
            node0_low_channel_ranges + node0_high_channel_ranges,
        )
        self._interleave_range(
            self.slow_ctrls, self.node1_ranges[0], self._node1_channels
        )
        self._configure_slow_cxl_link()
