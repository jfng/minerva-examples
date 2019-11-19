TRIPLE  = riscv64-unknown-elf
CC      = $(TRIPLE)-gcc
OBJCOPY = $(TRIPLE)-objcopy

ASFLAGS = -march=rv32i -mabi=ilp32
LDFLAGS = -nostdlib -Tlink.ld

all: hello.bin

hello.bin: hello
	$(OBJCOPY) -O binary $< $@

clean:
	rm hello.bin hello
