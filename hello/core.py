import sys

from nmigen import *
from nmigen.back.pysim import *

from nmigen_soc import wishbone
from minerva.core import Minerva


class WishboneROM(Elaboratable):
    def __init__(self, rom_init):
        self.bus = wishbone.Interface(addr_width=29, data_width=32, granularity=8)
        self.rom_init = rom_init

    def elaborate(self, platform):
        m = Module()

        rom = Memory(width=32, depth=len(self.rom_init), init=self.rom_init)
        m.submodules.rom_rp = rom_rp = rom.read_port()

        m.d.comb += [
            rom_rp.addr.eq(self.bus.adr),
            self.bus.dat_r.eq(rom_rp.data)
        ]

        m.d.sync += self.bus.ack.eq(0)
        with m.If(self.bus.cyc & self.bus.stb & ~self.bus.ack):
            m.d.sync += self.bus.ack.eq(1)

        return m


class WishboneArbiter(Elaboratable):
    def __init__(self):
        self.ibus   = wishbone.Interface(addr_width=30, data_width=32, granularity=8)
        self.dbus   = wishbone.Interface(addr_width=30, data_width=32, granularity=8)
        self.shared = wishbone.Interface(addr_width=30, data_width=32, granularity=8)

    def elaborate(self, platform):
        m = Module()

        rr = Signal()

        with m.If(rr):
            m.d.comb += self.dbus.connect(self.shared)
            with m.If(~self.dbus.cyc & self.ibus.cyc):
                m.d.sync += rr.eq(0)
        with m.Else():
            m.d.comb += self.ibus.connect(self.shared)
            with m.If(~self.ibus.cyc & self.dbus.cyc):
                m.d.sync += rr.eq(1)

        return m


class Top(Elaboratable):
    def __init__(self, rom_init):
        self.out  = wishbone.Interface(addr_width=1, data_width=8)
        self.halt = wishbone.Interface(addr_width=1, data_width=8)
        self.rom_init = rom_init

    def elaborate(self, platform):
        m = Module()

        m.submodules.rom = rom = WishboneROM(rom_init=self.rom_init)

        decoder = wishbone.Decoder(addr_width=30, data_width=32, granularity=8)
        decoder.add(rom.bus,   addr=0x00000000)
        decoder.add(self.out,  addr=0x80000000, sparse=True)
        decoder.add(self.halt, addr=0x80000004, sparse=True)
        m.submodules.decoder = decoder

        arbiter = m.submodules.arbiter = WishboneArbiter()

        m.submodules.cpu = cpu = Minerva()
        m.d.comb += [
            arbiter.ibus.cyc.eq(cpu.ibus.cyc),
            arbiter.ibus.stb.eq(cpu.ibus.stb),
            arbiter.ibus.adr.eq(cpu.ibus.adr),
            arbiter.ibus.sel.eq(cpu.ibus.sel),
            cpu.ibus.ack.eq(arbiter.ibus.ack),
            cpu.ibus.dat_r.eq(arbiter.ibus.dat_r),

            arbiter.dbus.cyc.eq(cpu.dbus.cyc),
            arbiter.dbus.stb.eq(cpu.dbus.stb),
            arbiter.dbus.adr.eq(cpu.dbus.adr),
            arbiter.dbus.sel.eq(cpu.dbus.sel),
            arbiter.dbus.we.eq(cpu.dbus.we),
            arbiter.dbus.dat_w.eq(cpu.dbus.dat_w),
            cpu.dbus.ack.eq(arbiter.dbus.ack),
            cpu.dbus.dat_r.eq(arbiter.dbus.dat_r),

            arbiter.shared.connect(decoder.bus)
        ]

        return m


if __name__ == "__main__":
    with open("hello.bin", "rb") as f:
        prog = [w for w in iter(lambda: int.from_bytes(f.read(4), byteorder="little"), 0)]

    dut = Top(rom_init=prog)

    with Simulator(dut) as sim:
        def process():
            while not ((yield dut.halt.cyc) and (yield dut.halt.stb)):
                if (yield dut.out.cyc) and (yield dut.out.stb):
                    if (yield dut.out.we):
                        sys.stdout.write(chr((yield dut.out.dat_w[:8])))
                    yield dut.out.ack.eq(1)
                    yield Tick()
                    yield dut.out.ack.eq(0)
                yield Tick()

        sim.add_clock(1e-6)
        sim.add_sync_process(process)
        sim.run()
