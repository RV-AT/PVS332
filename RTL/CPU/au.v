/*
适用于PRV332EXU的地址 csr计算单元
*/
module au(
input wire clk,
input wire rst,
input wire [3:0]statu_cpu,
input wire [2:0]opc_biu,

input wire rdy_alu,

input wire jalr,
input wire jal,

input wire beq,
input wire bne,
input wire blt,
input wire bltu,
input wire bge,
input wire bgeu,

input wire csrrw,
input wire csrrs,
input wire csrrc,
input wire csrrwi,
input wire csrrsi,
input wire csrrci,

//amo指令添加
input wire lr_w,
input wire sc_w,
input wire amoswap,
input wire amoadd,
input wire amoxor,
input wire amoand,
input wire amoor,
input wire amomin,
input wire amomax,
input wire amominu,
input wire amomaxu,

input wire pc_jmp,

//au需要用到的操作数
input wire [4:0]rs1_index,
input wire [31:0]rs1,
input wire [31:0]csr,
input wire [11:0]imm12,
input wire [19:0]imm20,
input wire [31:0]pc,

output reg [31:0]addr_csr,
output reg [31:0]pc_next

);
//biu操作码
parameter w8 = 3'b001;
parameter w16= 3'b010;
parameter w32= 3'b011;
parameter r8 = 3'b101;
parameter r16= 3'b110;
parameter r32= 3'b111;
//处理器状态编码
parameter if0 = 4'b0000;
parameter ex0 = 4'b0001;
parameter mem0= 4'b0010;
parameter mem1=4'b1010;
parameter ex1 = 4'b1001;
parameter wb = 4'b0011;
parameter exc = 4'b1111;

always@(posedge clk)begin
	if(rst)begin
		pc_next <= 32'b0;
		addr_csr <= 32'b0;
	end
	else if((statu_cpu==ex0)&rdy_alu)begin
		pc_next <= (jal?{{11{imm20[19]}},imm20,1'b0}:jalr?{{19{imm12[11]}},imm12,1'b0}:pc) +  	//下一个pc值选取
		(jal?pc:jalr?rs1:((beq | bne | blt | bltu | bge | bgeu )&pc_jmp)?{{19{imm12[11]}},imm12,1'b0}:32'd4);
		
		addr_csr <= (lr_w|sc_w|amoswap|amoadd|amoxor|amoand|amoor|amomin|amomax|amominu|amomaxu)?rs1:
		((opc_biu==r8)|(opc_biu==r16)|(opc_biu==r32)|(opc_biu==w8)|(opc_biu==w16)|(opc_biu==w32))?(rs1 + {{20{imm12[11]}},imm12}):
		csrrw ?  rs1 : csrrs ? (csr | rs1) : csrrc ? (csr | !rs1):csrrwi? {27'b0,rs1_index} :csrrsi? (csr | {27'b0,rs1_index}):
      csrrci? (csr | !{27'b0,rs1_index}):32'b0;
	end
end

endmodule
