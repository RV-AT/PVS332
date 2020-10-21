addi x1,x0,0
addi x2,x0,0
addi x5,x0,0
addi x6,x0,0x20    
addi x7,x0,0x00    ;data base address
addi x8,x0,0
lui x8,0x20         ;RAM base address
lui x5,0x0ffe0      ;CPERI Base address
addi x5,x5,0x180    ;GPIO offset
addi x1,x0,0xfff
sw x1,0(x5)          ;Test input & output
sw x1,0x8(x5)
sw x0,0x8(x5)
sw x1,0x1c(x5)       ;enable all AF functions
lui x5,0x0ffe0      
addi x5,x5,0x80     ;UART offset
addi x4,x0,0x96    ;UART init,TFIFO Enabled
sw x4,0(x5) 
addi x5,x5,4
addi x4,x0,0x40
sw x4,0(x5)          ;Set UART divide ratio=32
addi x5,x5,4
addi x8,x0,0        ;UART Transmit setting
addi x4,x0,0x10
addi x6,x0,0x0
lui x8,0x20         ;RAM base address
addi x3,x0,0
addi x3,x3,0x3      ;UART Transmit start
addi x8,x8,0x1
sw x3,0(x5)
addi x4,x4,1
bne x6,x4,-0x8   ;JMP 
addi x1,x0,0 ; Delay
addi x2,x0,0x7FF
addi x1,x1,1
bne x1,x2,0xffe
bne x2,x0,-0x12

