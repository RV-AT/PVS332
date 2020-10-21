/*
适用于PRV332SV0的指令解码单元
*/
module ins_dec(

input wire [31:0]ins,
input wire [3:0]statu_cpu,
input wire [1:0]msu,					//当前处理器权限

input wire tsr,
input wire tvm,

output wire [2:0]opc_biu,
//译码器结果输出(exu)
output wire addi ,
output wire slti,
output wire sltiu,
output wire andi,
output wire ori,
output wire xori,
output wire slli,
output wire srli,
output wire srai,

output wire lui,
output wire auipc,
output wire addp,
output wire subp,
output wire sltp,
output wire sltup,
output wire andp,
output wire orp,
output wire xorp,
output wire sllp,
output wire srlp,
output wire srap,

output wire jal,
output wire jalr,

output wire beq,
output wire bne,
output wire blt,
output wire bltu,
output wire bge,
output wire bgeu,
//load指令对数据进行符号位拓展信号
output wire lb,
output wire lh,

output wire csrrw,
output wire csrrs,
output wire csrrc,
output wire csrrwi,
output wire csrrsi,
output wire csrrci,
//amo指令添加
output wire lr_w,
output wire sc_w,
output wire amoswap,
output wire amoadd,
output wire amoxor,
output wire amoand,
output wire amoor,
output wire amomin,
output wire amomax,
output wire amominu,
output wire amomaxu,

output wire csr_wr,

output wire gpr_wr,

output wire ebreak,
output wire ecall,
output wire ret,
output wire fence,
output wire [4:0]rs1_index,
output wire [4:0]rs2_index,
output wire [4:0]rd_index,
output wire [11:0]csr_index,


output wire [19:0]imm20,
output wire [11:0]imm12,
output wire [4:0] shamt,

output wire [3:0]ins_flow,				//指令流报告
output wire ill_ins					   //非法指令




);

//处理器状态编码
parameter if0 = 4'b0000;
parameter ex0 = 4'b0001;
parameter mem0= 4'b0010;
parameter mem1=4'b1010;
parameter ex1 = 4'b1001;
parameter wb = 4'b0011;
parameter exc = 4'b1111;
//ins_flow,
parameter if_ex_mem_wb=4'b0001;
parameter if_ex_wb	 =4'b0010;
parameter if_ex_mem_ex_mem_wb=4'b0011;
//opc_biu
parameter w8 = 3'b001;
parameter w16= 3'b010;
parameter w32= 3'b011;
parameter r8 = 3'b101;
parameter r16= 3'b110;
parameter r32= 3'b111;
/*
parameter nop  = 6'b000000;
parameter addi = 6'b000001;
parameter slti = 6'b000010;
parameter sltiu= 6'b000011;
parameter andi = 6'b000100;
parameter ori	= 6'b000101;
parameter xori	= 6'b000110;
parameter slli = 6'b000111;
parameter srli	= 6'b001000;
parameter srai = 6'b001001;
parameter lui  = 6'b001010;
parameter auipc= 6'b001011;
parameter addp	= 6'b001100;
parameter subp = 6'b001101;
parameter sltp	= 6'b001110;
parameter sltup= 6'b001111;
parameter andp	= 6'b010000;
parameter orp	= 6'b010001;
parameter xorp = 6'b010010;
parameter sllp = 6'b010011;
parameter srlp = 6'b010100;
parameter srap = 6'b010101;
*/

assign addi = ((ins[6:0]==7'b0010011)&(ins[14:12]==3'b000))? 1'b1 : 1'b0;
assign slti = ((ins[6:0]==7'b0010011)&(ins[14:12]==3'b010))? 1'b1 : 1'b0;
assign sltiu= ((ins[6:0]==7'b0010011)&(ins[14:12]==3'b011))? 1'b1 : 1'b0;
assign xori = ((ins[6:0]==7'b0010011)&(ins[14:12]==3'b100))? 1'b1 : 1'b0;
assign ori  = ((ins[6:0]==7'b0010011)&(ins[14:12]==3'b110))? 1'b1 : 1'b0;
assign andi = ((ins[6:0]==7'b0010011)&(ins[14:12]==3'b111))? 1'b1 : 1'b0;
assign slli = ((ins[6:0]==7'b0010011)&(ins[14:12]==3'b001))? 1'b1 : 1'b0;
assign srli = ((ins[6:0]==7'b0010011)&(ins[14:12]==3'b101)&(ins[31:25]==7'b0000000))? 1'b1 : 1'b0;
assign srai = ((ins[6:0]==7'b0010011)&(ins[14:12]==3'b101)&(ins[31:25]==7'b0100000))? 1'b1 : 1'b0;

assign lui = ((ins[6:0])==7'b0110111)?1'b1 : 1'b0;
assign auipc = (ins[6:0]==7'b0010111) ? 1'b1 : 1'b0;

assign addp = ((ins[6:0]==7'b0110011)&(ins[14:12]==3'b000)&(ins[31:25]==7'b0000000))? 1'b1 : 1'b0;
assign subp = ((ins[6:0]==7'b0110011)&(ins[14:12]==3'b000)&(ins[31:25]==7'b0100000))? 1'b1 : 1'b0;
assign sllp = ((ins[6:0]==7'b0110011)&(ins[14:12]==3'b001))? 1'b1 : 1'b0;
assign sltp = ((ins[6:0]==7'b0110011)&(ins[14:12]==3'b010))? 1'b1 : 1'b0;
assign sltup= ((ins[6:0]==7'b0110011)&(ins[14:12]==3'b011))? 1'b1 : 1'b0;
assign xorp = ((ins[6:0]==7'b0110011)&(ins[14:12]==3'b100))? 1'b1 : 1'b0;
assign srlp = ((ins[6:0]==7'b0110011)&(ins[14:12]==3'b101)&(ins[31:25]==7'b0000000))? 1'b1 : 1'b0;
assign srap = ((ins[6:0]==7'b0110011)&(ins[14:12]==3'b101)&(ins[31:25]==7'b0100000))? 1'b1 : 1'b0;
assign orp  = ((ins[6:0]==7'b0110011)&(ins[14:12]==3'b110))? 1'b1 : 1'b0;
assign andp = ((ins[6:0]==7'b0110011)&(ins[14:12]==3'b111))? 1'b1 : 1'b0;

assign jal  = (ins[6:0]==7'b1101111)? 1'b1 : 1'b0;
assign jalr = (ins[6:0]==7'b1100111)? 1'b1 : 1'b0;

assign beq  = ((ins[6:0]==7'b1100011)&(ins[14:12]==3'b000))?1'b1:1'b0;
assign bne  = ((ins[6:0]==7'b1100011)&(ins[14:12]==3'b001))?1'b1:1'b0; 
assign blt  = ((ins[6:0]==7'b1100011)&(ins[14:12]==3'b100))?1'b1:1'b0;
assign bge  = ((ins[6:0]==7'b1100011)&(ins[14:12]==3'b101))?1'b1:1'b0;
assign bltu = ((ins[6:0]==7'b1100011)&(ins[14:12]==3'b110))?1'b1:1'b0;
assign bgeu = ((ins[6:0]==7'b1100011)&(ins[14:12]==3'b111))?1'b1:1'b0;

assign csrrw= ((ins[6:0]==7'b1110011)&(ins[14:12]==3'b001))?1'b1:1'b0;
assign csrrs= ((ins[6:0]==7'b1110011)&(ins[14:12]==3'b010))?1'b1:1'b0;
assign csrrc= ((ins[6:0]==7'b1110011)&(ins[14:12]==3'b011))?1'b1:1'b0;
assign csrrwi=((ins[6:0]==7'b1110011)&(ins[14:12]==3'b101))?1'b1:1'b0;
assign csrrsi=((ins[6:0]==7'b1110011)&(ins[14:12]==3'b110))?1'b1:1'b0;
assign csrrci=((ins[6:0]==7'b1110011)&(ins[14:12]==3'b111))?1'b1:1'b0;

assign lr_w		=(ins[6:0] == 7'b0101111)&(ins[31:27]==5'b00010);
assign sc_w		=(ins[6:0] == 7'b0101111)&(ins[31:27]==5'b00011);
assign amoswap	=(ins[6:0] == 7'b0101111)&(ins[31:27]==5'b00001);
assign amoadd	=(ins[6:0] == 7'b0101111)&(ins[31:27]==5'b00000);
assign amoxor	=(ins[6:0] == 7'b0101111)&(ins[31:27]==5'b00100);
assign amoand	=(ins[6:0] == 7'b0101111)&(ins[31:27]==5'b01100);
assign amoor	=(ins[6:0] == 7'b0101111)&(ins[31:27]==5'b01000);
assign amomin	=(ins[6:0] == 7'b0101111)&(ins[31:27]==5'b10000);
assign amomax	=(ins[6:0] == 7'b0101111)&(ins[31:27]==5'b10100);
assign amominu	=(ins[6:0] == 7'b0101111)&(ins[31:27]==5'b11000);
assign amomaxu	=(ins[6:0] == 7'b0101111)&(ins[31:27]==5'b11100);

assign ebreak=((ins[6:0]==7'b1110011)&(ins[14:12]==3'b000)&(ins[31:25]==12'b0000_0000_0001))?1'b1:1'b0;
assign ecall =((ins[6:0]==7'b1110011)&(ins[14:12]==3'b000)&(ins[31:25]==12'b0000_0000_0000))?1'b1:1'b0;
assign ret   =(ins[6:0]==7'b1110011)&(ins[14:12]==3'b000)&((ins[31:25]==12'b001100000010)|(ins[31:25]==12'b000100000010)|(ins[31:25]==12'b000000000010))?1'b1:1'b0;

assign opc_biu=((ins[6:0]==7'b0100011)&(ins[14:12]==3'b000))?w8:((ins[6:0]==7'b0100011)&(ins[14:12]==3'b001))?w16:
((ins[6:0]==7'b0100011)&(ins[14:12]==3'b010))?w32:((ins[6:0]==7'b0000011)&((ins[14:12]==3'b000)|(ins[14:12]==3'b100)))?r8:
((ins[6:0]==7'b0000011)&((ins[14:12]==3'b001)|(ins[14:12]==3'b101)))?r16:((ins[6:0]==7'b0000011)&(ins[14:12]==3'b010))?r32: //到这里是RV32I的内存指令
(amoswap|amoadd|amoxor|amoand|amoor|amomin|amomax|amominu|amomaxu)&mem1?r32:(amoswap|amoadd|amoxor|amoand|amoor|amomin|amomax|amominu|amomaxu)&mem0?w32:3'b000; //AMO添加
//让人头大的指令译码逻辑
//微码+1

assign fence=(ins[6:0]==7'b0001111);

assign rs1_index=ins[19:15];      //这个接口也被用作是zimm
assign rs2_index=ins[24:20];
assign rd_index =ins[11:7];
assign csr_index=ins[31:20];
assign imm20    =ins[31:12];
assign imm12    =((ins[6:0]==7'b1100011) | (ins[6:0]==7'b0100011)) ? {ins[31:25],ins[11:7]} : ins[31:20];
assign shamt    =ins[24:20];


assign ill_ins=  !((ins[6:0] == 7'b0110111) |
                   (ins[6:0] == 7'b0010111) |
                   (ins[6:0] == 7'b1101111) |
                   (ins[6:0] == 7'b1100111) & (ins[14:12] == 3'b000) |
                   (ins[6:0] == 7'b1100011) & (ins[14:12] != 3'b010)  |
                   (ins[6:0] == 7'b0000011) & ((ins[14:12] != 3'b011) | (ins[14:12] != 3'b110) | (ins[14:12] != 3'b111)) |
                   (ins[6:0] == 7'b0100011) & ((ins[14:12] == 3'b000) | (ins[14:12] != 3'b001) | (ins[14:12] != 3'b010)) |
                   (ins[6:0] == 7'b0010011) |
                   (ins[6:0] == 7'b0110011) |
                   (ins[6:0] == 7'b0001111) & (ins[14:12] == 3'b000) |
                   (ins[6:0] == 7'b1110011) |
						 (ins[6:0] == 7'b0101111));					//amo指令添加


assign ins_flow = (lui|auipc|jal|jalr|beq|bne|blt|bge|bltu|bgeu|addi|slti|sltiu|xori|
						ori|andi|slli|srli|srai|addp|subp|sllp|sltp|sltup|xorp|srlp|srap|orp|
						andp|csrrw|csrrs|csrrc|csrrwi|csrrsi|csrrci|fence)? if_ex_wb :					//内部操作时用第一执行序列
						((ins[6:0]==7'b0000011)|(ins[6:0]==7'b0100011)|lr_w|sc_w)? if_ex_mem_wb :  //一次内存访问第二执行序列
						(amoswap|amoadd|amoxor|amoand|amoxor|amomin|amomax|								//原子指令第三执行序列
						amominu|amomaxu)? if_ex_mem_ex_mem_wb : 4'b1111;
						
//对load指令的符号位拓展信号						
assign lb = ((ins[6:0]==7'b0000011)&(ins[14:12]==3'b000));
assign lh = ((ins[6:0]==7'b0000011)&(ins[14:12]==3'b001));
//返回寄存器写信号
assign csr_wr = csrrw | csrrs | csrrc| csrrwi|csrrsi|csrrci;
assign gpr_wr = (statu_cpu==wb)&(!beq)&(!bne)&(!blt)&(!bge)&(!bltu)&(!bgeu)&
				(opc_biu!=w32)&(opc_biu!=w16)&(opc_biu!=w8)&(!ecall)&(!ebreak)&(!fence);



endmodule
