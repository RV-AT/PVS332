addi x1,x0,0
addi x2,x0,0
addi x5,x0,0
addi x6,x0,0x20    
addi x7,x0,0x00    ;data base address
addi x8,x0,0
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
sw x1,0x1c(x5)       ;enable all AF functions
lui x5,0x0ffe0      ;UART init,FIFO Disable
addi x5,x5,0x80
addi x4,x0,0x86    
sw x4,0(x5) 
addi x5,x5,4
addi x4,x0,0x40
sw x4,0(x5)          ;Set UART divide ratio=32
addi x5,x5,4
addi x8,x0,0        ;UART Transmit setting
addi x4,x0,0x20
addi x6,x0,0
lui x8,0x20         ;RAM base address
addi x3,x0,0
lb x3,0(x8)          ;UART Transmit start
addi x8,x8,0x1
sw x3,0(x5)
addi x6,x6,1
addi x1,x0,0 ; Delay
addi x2,x0,0x18
addi x1,x1,1
bne x1,x2,0xffe
bne x6,x4,0xfec    ;JMP 



