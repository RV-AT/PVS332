addi x1,x0,0
addi x2,x0,0
addi x5,x0,0
addi x6,x0,0x40    
addi x7,x0,0x00    ;data base address
lui x8,0x20         ;RAM base address
lui x5,0x0ffe0      ;UART base address
lw x1,0(x7)          ;move data from ROM into RAM(in order to test 8 bit rw)
addi x7,x7,0x4
sw x1,0(x8)
addi x8,x8,0x4
bne x7,x6,0xff8
addi x5,x5,0x180    ;GPIO setting
addi x1,x0,0xfff
sw x1,0(x5)          ;Test input & output
sw x1,0x8(x5)
sw x0,0x8(x5)
sw x1,0x8(x5)
lui x8,0x20
lui x6,0x70000      ;ESRAM base address
addi x7,x6,0x40     ;Start test ESRAM
lb x1,0(x8)
sw x1,0(x5)
addi x8,x8,1
sb x1,0(x6)
addi x6,x6,1
bne x6,x7,0xff6



