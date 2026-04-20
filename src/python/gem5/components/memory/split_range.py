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

"""Multi-range channeled memory for large x86 full-system maps."""

from math import log
from typing import (
    List,
    Optional,
    Sequence,
    Tuple,
    Type,
    Union,
)

from m5.objects import (
    AbstractMemory,
    DRAMInterface,
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


def _try_convert(val, cls):
    try:
        return cls(val)
    except:
        raise Exception(f"Could not convert {val} to {cls}")


def _is_pow2(num: int) -> bool:
    return num > 0 and (num & (num - 1)) == 0


class SplitRangeChanneledMemory(AbstractMemorySystem):
    """A ChanneledMemory-like component for sparse physical RAM maps.

    This component creates one interleaved controller set for each supplied
    range. For x86 full-system memory above 3 GiB, this allows RAM below and
    above the 3-4 GiB platform hole without using stock ChanneledMemory's
    single contiguous range contract.
    """

    def __init__(
        self,
        dram_interface_class: Type[DRAMInterface],
        num_channels: Union[int, str],
        interleaving_size: Union[int, str],
        size: Optional[str] = None,
        addr_mapping: Optional[str] = None,
    ) -> None:
        num_channels = _try_convert(num_channels, int)
        interleaving_size = _try_convert(interleaving_size, int)

        if size:
            size = _try_convert(size, str)

        if addr_mapping:
            addr_mapping = _try_convert(addr_mapping, str)

        super().__init__()

        if not _is_pow2(num_channels):
            raise ValueError("num_channels must be a power of 2")
        if not _is_pow2(interleaving_size):
            raise ValueError("interleaving_size must be a power of 2")

        self._dram_class = dram_interface_class
        self._num_channels = num_channels
        self._intlv_size = interleaving_size
        self._addr_mapping = (
            addr_mapping
            if addr_mapping
            else self._dram_class.addr_mapping.value
        )
        self._size = (
            toMemorySize(size)
            if size
            else self._get_dram_size(num_channels, self._dram_class)
        )

        object.__setattr__(self, "_ranges", [])
        object.__setattr__(self, "_dram", [])

    def _get_dram_size(self, num_channels: int, dram: DRAMInterface) -> int:
        return num_channels * (
            dram.device_size.value
            * dram.devices_per_rank.value
            * dram.ranks_per_channel.value
        )

    def _interleaved_range(
        self, memory_range: AddrRange, channel: int, dram: DRAMInterface
    ) -> AddrRange:
        if self._addr_mapping == "RoRaBaChCo":
            rowbuffer_size = (
                dram.device_rowbuffer_size.value * dram.devices_per_rank.value
            )
            intlv_low_bit = int(log(rowbuffer_size, 2))
        elif self._addr_mapping in ["RoRaBaCoCh", "RoCoRaBaCh"]:
            intlv_low_bit = int(log(self._intlv_size, 2))
        else:
            raise ValueError(
                "Only these address mappings are supported: "
                "RoRaBaChCo, RoRaBaCoCh, RoCoRaBaCh"
            )

        intlv_bits = int(log(self._num_channels, 2))
        return AddrRange(
            start=memory_range.start,
            size=memory_range.size(),
            intlvHighBit=intlv_low_bit + intlv_bits - 1,
            xorHighBit=0,
            intlvBits=intlv_bits,
            intlvMatch=channel,
        )

    def _validate_ranges(self, ranges: List[AddrRange]) -> None:
        if not ranges:
            raise ValueError("at least one memory range is required")

        total = sum(memory_range.size() for memory_range in ranges)
        if total != self._size:
            raise ValueError(
                f"range total {total} does not match memory size {self._size}"
            )

        sorted_ranges = sorted(
            (int(memory_range.start), memory_range.size())
            for memory_range in ranges
        )
        previous_end = None
        for start, size in sorted_ranges:
            if previous_end is not None and start < previous_end:
                raise ValueError("memory ranges must be non-overlapping")
            previous_end = start + size

    @overrides(AbstractMemorySystem)
    def incorporate_memory(self, board: AbstractBoard) -> None:
        if self._intlv_size < int(board.get_cache_line_size()):
            raise ValueError(
                "Memory interleaving size can not be smaller than"
                " board's cache line size.\nBoard's cache line size: "
                f"{board.get_cache_line_size()}\n, This memory's "
                f"interleaving size: {self._intlv_size}"
            )

    @overrides(AbstractMemorySystem)
    def get_mem_ports(self) -> Sequence[Tuple[AddrRange, Port]]:
        return [(ctrl.dram.range, ctrl.port) for ctrl in self.mem_ctrl]

    @overrides(AbstractMemorySystem)
    def get_memory_controllers(self) -> List[MemCtrl]:
        return list(self.mem_ctrl)

    @overrides(AbstractMemorySystem)
    def get_mem_interfaces(self) -> List[AbstractMemory]:
        return list(self._dram)

    @overrides(AbstractMemorySystem)
    def get_size(self) -> int:
        return self._size

    @overrides(AbstractMemorySystem)
    def set_memory_range(self, ranges: List[AddrRange]) -> None:
        self._validate_ranges(ranges)

        range_list = list(ranges)
        dram_interfaces = []
        mem_ctrls = []

        for memory_range in range_list:
            for channel in range(self._num_channels):
                dram = self._dram_class(addr_mapping=self._addr_mapping)
                dram.range = self._interleaved_range(
                    memory_range, channel, dram
                )
                dram_interfaces.append(dram)
                mem_ctrls.append(MemCtrl(dram=dram))

        object.__setattr__(self, "_ranges", range_list)
        self._dram = dram_interfaces
        self.mem_ctrl = mem_ctrls

    @overrides(AbstractMemorySystem)
    def get_uninterleaved_range(self) -> List[AddrRange]:
        return list(self._ranges)
