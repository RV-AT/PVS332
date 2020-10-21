lui x2,0x0ffc0
addi x1,x0,0
sw x1,0x4(x2)
sw x1,0x0(x2)   ;Clear MTIME
sw x1,0xc(x2)   ;Write MTIMECMP
addi x1,x0,0x300  ;0x00000300 Cycles
sw x1,0x8(x2)
lui x2,0x0ffde
addi x2,x2,0x7ff
addi x2,x2,0x01
addi x1,x0,0xfff
sw x1,0x0(x2)
sw x1,0x4(x2)
sw x1,0x24(x2)
addi x0,x0,0
addi x0,x0,0
beq x0,x0,-0x4
