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

"""x86 stdlib board variant for large sparse full-system memory maps."""

from typing import (
    List,
    Optional,
    Tuple,
)

from m5.objects import (
    X86ACPISlit,
    X86ACPISrat,
    X86ACPISratMemoryAffinity,
    X86ACPISratProcessorLocalApic,
    X86E820Entry,
)
from m5.params import AddrRange
from m5.util.convert import toMemorySize

from ...utils.override import overrides
from .abstract_system_board import AbstractSystemBoard
from .x86_board import X86Board

LOW_MEM_LIMIT = toMemorySize("3GiB")
HIGH_MEM_BASE = 0x100000000
IO_MARKER_BASE = 0xC0000000
IO_MARKER_SIZE = 0x100000
M5OPS_BASE = 0xFFFF0000
LEGACY_LOW_RAM_SIZE = toMemorySize("639KiB")
LINUX_NUMA_LOW_RAM_BASE = toMemorySize("1MiB")


class LargeMemoryX86Board(X86Board):
    """X86Board derivative that supports RAM above the 3-4 GiB hole.

    Stock X86Board remains limited to the documented 3 GiB ordinary-memory
    map. This board is the project-local place for large x86 full-system maps,
    including optional memory-system hooks:

    * get_default_memory_ranges(): explicit non-contiguous RAM ranges.
    * get_numa_memory_ranges(): node-to-ranges metadata for SRAT/SLIT.
    """

    @overrides(AbstractSystemBoard)
    def _setup_memory_ranges(self) -> None:
        memory = self.get_memory()
        used_project_ranges = hasattr(memory, "get_default_memory_ranges")

        if used_project_ranges:
            data_ranges = memory.get_default_memory_ranges()
        elif memory.get_size() <= LOW_MEM_LIMIT:
            data_ranges = [AddrRange(memory.get_size())]
        else:
            data_ranges = [
                AddrRange(LOW_MEM_LIMIT),
                AddrRange(
                    HIGH_MEM_BASE,
                    size=memory.get_size() - LOW_MEM_LIMIT,
                ),
            ]

        try:
            memory.set_memory_range(data_ranges)
        except Exception as err:
            if not used_project_ranges and memory.get_size() > LOW_MEM_LIMIT:
                raise ValueError(
                    "LargeMemoryX86Board split memory above 3GiB around the "
                    "x86 3-4GiB hole, but the supplied memory object rejected "
                    "multiple ranges. Use SplitRangeChanneledMemory or a "
                    "memory object that provides explicit multi-range support."
                ) from err
            raise

        self.mem_ranges = data_ranges + [
            AddrRange(IO_MARKER_BASE, size=IO_MARKER_SIZE)
        ]

    @overrides(X86Board)
    def _setup_board(self) -> None:
        super()._setup_board()

        if not self.is_fullsystem():
            return

        self._replace_e820_table()
        self._setup_numa_acpi_tables()

    def _data_mem_ranges(self) -> List[AddrRange]:
        return [
            memory_range
            for memory_range in self.mem_ranges
            if not (
                int(memory_range.start) == IO_MARKER_BASE
                and memory_range.size() == IO_MARKER_SIZE
            )
        ]

    def _uses_numa_acpi(self) -> bool:
        return hasattr(self.get_memory(), "get_numa_memory_ranges")

    def _minimum_e820_ram_start(self) -> int:
        if self._uses_numa_acpi():
            return LINUX_NUMA_LOW_RAM_BASE
        return 0

    def _e820_ram_segments(self) -> List[Tuple[int, int]]:
        segments = []
        minimum_start = self._minimum_e820_ram_start()

        for memory_range in self._data_mem_ranges():
            start = int(memory_range.start)
            end = start + memory_range.size()

            if end <= minimum_start:
                continue
            start = max(start, minimum_start)
            size = end - start

            if start == 0:
                if size <= 0x100000:
                    continue
                start = 0x100000
                size -= 0x100000

            segments.append((start, size))

        return segments

    def _replace_e820_table(self) -> None:
        if self._uses_numa_acpi():
            e820_entries = [
                (0, f"{LEGACY_LOW_RAM_SIZE:d}B", 1),
                (
                    LEGACY_LOW_RAM_SIZE,
                    f"{LINUX_NUMA_LOW_RAM_BASE - LEGACY_LOW_RAM_SIZE:d}B",
                    2,
                ),
            ]
        else:
            e820_entries = [
                (0, "639KiB", 1),
                (0x9FC00, "385KiB", 2),
            ]

        for start, size in self._e820_ram_segments():
            e820_entries.append((start, f"{size:d}B", 1))

        e820_entries.append(
            (IO_MARKER_BASE, f"{M5OPS_BASE - IO_MARKER_BASE:d}B", 2)
        )
        e820_entries.append((M5OPS_BASE, "64KiB", 2))

        self.workload.e820_table.entries = [
            X86E820Entry(addr=addr, size=size, range_type=range_type)
            for addr, size, range_type in sorted(e820_entries)
        ]

    def _numa_range_segment(
        self, memory_range: AddrRange
    ) -> Optional[Tuple[int, int]]:
        start = int(memory_range.start)
        end = start + memory_range.size()

        if end <= LINUX_NUMA_LOW_RAM_BASE:
            return None
        start = max(start, LINUX_NUMA_LOW_RAM_BASE)
        return start, end - start

    def _setup_numa_acpi_tables(self) -> None:
        memory = self.get_memory()
        if not hasattr(memory, "get_numa_memory_ranges"):
            return

        node_ranges = memory.get_numa_memory_ranges()
        if not node_ranges:
            return

        node_ids = sorted(int(node_id) for node_id in node_ranges.keys())
        if node_ids != [0, 1]:
            raise ValueError(
                "LargeMemoryX86Board NUMA ACPI support currently expects "
                "exactly nodes 0 and 1."
            )

        srat_records = [
            X86ACPISratProcessorLocalApic(
                proximity_domain=0,
                apic_id=apic_id,
                flags=1,
            )
            for apic_id in range(self.get_processor().get_num_cores())
        ]
        srat_records.append(
            X86ACPISratMemoryAffinity(
                proximity_domain=0,
                base_address=0,
                address_length=LEGACY_LOW_RAM_SIZE,
                flags=1,
            )
        )

        for node_id in node_ids:
            for memory_range in node_ranges[node_id]:
                segment = self._numa_range_segment(memory_range)
                if segment is None:
                    continue
                start, size = segment

                srat_records.append(
                    X86ACPISratMemoryAffinity(
                        proximity_domain=node_id,
                        base_address=start,
                        address_length=size,
                        flags=1,
                    )
                )

        srat = X86ACPISrat(
            records=srat_records,
            oem_id="gem5",
            oem_table_id="NUMA",
        )
        slit = X86ACPISlit(
            locality_count=2,
            distances=[10, 20, 20, 10],
            oem_id="gem5",
            oem_table_id="NUMA",
        )

        rsdt = self.workload.acpi_description_table_pointer.rsdt
        xsdt = self.workload.acpi_description_table_pointer.xsdt
        rsdt.entries.extend([srat, slit])
        xsdt.entries.extend([srat, slit])
