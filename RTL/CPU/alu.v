/*
适用于PRV332SV0的EXU模块的ALU算术逻辑单元
*/
module alu(

input clk,
input rst,
input wire [3:0]statu_cpu,
input wire [2:0]opc_biu,

//exu控制信号
//译码器结果输出(exu)
input wire addi ,
input wire slti,
input wire sltiu,
input wire andi,
input wire ori,
input wire xori,
input wire slli,
input wire srli,
input wire srai,

input wire lui,
input wire auipc,
input wire addp,
input wire subp,
input wire sltp,
input wire sltup,
input wire andp,
input wire orp,
input wire xorp,
input wire sllp,
input wire srlp,
input wire srap,

input wire jal,
input wire jalr,

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
//alu需要用到的操作数
input wire [4:0]rs1_index,
input wire [31:0]csr,
input wire [31:0]rs1,
input wire [31:0]rs2,
input wire [19:0]imm20,
input wire [11:0]imm12,
input wire [4:0] shamt,
input wire [31:0]data_biu,
input wire [31:0]pc,
//alu输出
output reg [31:0]data_rd,
output reg [31:0]data_tobiu,
output wire pc_jmp,
output wire rdy_alu			//ALU准备完毕信号

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

reg [4:0]shift_counter; //移位计数器

wire [31:0]sub_out;
wire [31:0]add_out;
wire [31:0]and_out;
wire [31:0]or_out;
wire [31:0]xor_out;
wire [31:0]cmp_out;
wire [31:0]amo_cmp_out;
//第一数据输入
wire [31:0]ds1;
//第二数据输入
wire [31:0]sub_ds2;
wire [31:0]add_ds2;
wire [31:0]and_ds2;
wire [31:0]or_ds2;
wire [31:0]xor_ds2;
wire [31:0]cmp_ds2;
//第三数据输出
wire [31:0]data_out_rd;

wire [31:0]imm32;				//对imm12进行符号位拓展后的32位数

assign imm32 = {{20{imm12[11]}},imm12};	//对imm12进行符号位拓展

assign ds1 = (amoswap|amoadd|amoxor|amoand|amoor|amomin|amomax|amominu|amomaxu)?data_biu:rs1;

assign sub_ds2 = rs2;
assign add_ds2 = (addp|amoadd)?rs2:addi?imm32:pc;  //当amoadd和add立即数的时候选择rs2作为加法器操作数2 
assign and_ds2 = (andp|amoand)?rs2:imm32;
assign or_ds2  = (orp|amoor)? rs2:imm32;
assign xor_ds2 = (xorp|amoxor)?rs2:imm32;
assign cmp_ds2 = (sltp|sltup)? rs2:imm32;

assign sub_out = ds1 - sub_ds2;
assign add_out = ds1 + add_ds2;
assign and_out = ds1 & and_ds2;
assign or_out  = ds1 | or_ds2;
assign xor_out = ds1 ^ xor_ds2;
assign cmp_out = ((sltiu|sltup)&(ds1<cmp_ds2))?32'd1:(slti|sltp)? 32'd0:(!(ds1[31]^cmp_ds2[31])&(ds1<cmp_ds2)|ds1[31]&!cmp_ds2[31])?32'd1 : 32'd0;
assign amo_cmp_out =((amomax|amomaxu)&(rs2>data_biu))?rs2 : ((amomin|amominu)&(rs2<data_biu))?rs2:data_biu;

assign data_out_rd = (lui?{imm20,12'b0}:32'b0)|((jal|jalr)?(pc+32'd4):32'b0)|
							((addi|addp)?add_out:32'b0) | (subp?sub_out:32'b0)|
							((andi|andp)?and_out:32'b0) | ((ori|orp)?or_out:32'b0)|
							((sltiu|sltp|slti|sltp)?cmp_out : 32'b0)|
							((csrrwi|csrrsi|csrrci|csrrw|csrrs|csrrc)?csr:32'b0)|
							((amoswap|amoadd|amoxor|amoand|amoor|amomin|amomax|amominu|amomaxu)?data_biu:32'b0);

always@(posedge clk)begin

	if(rst)begin
		data_rd <= 32'b0;
		data_tobiu <= 32'b0;
		shift_counter <= 5'b0; 
	end
	
	else if(shift_counter==5'b0)begin
		if(statu_cpu==ex0)begin
			data_rd <= data_out_rd;
			shift_counter <= (slli|srli|srai)?shamt : (sllp|srlp|srap)?rs2[4:0]:5'b0;
			data_tobiu <= ((opc_biu==w8)|(opc_biu==w16)|(opc_biu==w32)|sc_w)?rs2:32'b0;
		end
		else if(statu_cpu==ex1)begin
			data_rd <= data_rd;
			shift_counter <= shift_counter;
			data_tobiu <= (amoswap?rs2:32'b0)|(amoadd? add_out:32'b0)|(amoxor? xor_out:32'b0)|(amoand? add_out:32'b0)|
							  ((amomax|amomaxu)?amo_cmp_out:32'b0)|((amomin|amominu)?amo_cmp_out:32'b0);
		end
	end
	
	else if(shift_counter != 5'b0)begin
		if(slli | sllp)begin
            
			data_rd <= (data_rd<<1);
			shift_counter <= shift_counter - 5'd1;
					
        
		end
		else if(srli|srlp)begin
           
			data_rd <= (data_rd>>1);
			shift_counter <= shift_counter-5'd1;
					
		end
		else if(srai | srap)begin
            
			data_rd <= (data_rd>>1);                                 //2019 5.23此处有一个bug，我不知道有符号数怎么搞
			shift_counter<=shift_counter - 5'd1;
					
		end
	end
end

	
assign pc_jmp = beq & (rs1 == rs2) | bne & (rs1 !=rs2) | blt & ((rs1[31]==1'b1)&(rs2[31]==1'b0) | (rs1[31]==rs2[31])&(rs1 < rs2)) | 
	bltu & (rs1 < rs2) | bge & ((rs1[31]==1'b0&rs2[31]==1'b1 | (rs1[31]==rs2[31])&rs1 < rs2)) | bgeu & (rs1 > rs2) | jal | jalr;	
		
assign rdy_alu = (((slli|srli|srai)&((shamt==5'b0)|(shift_counter==5'd1))) | ((sllp|srlp|srap)&((rs2[4:0]==5'b0)|(shift_counter==5'd1)))|
						!(slli|srli|srai|sllp|srlp|srap))&((statu_cpu==ex0)|(statu_cpu==ex1));
	

		

endmodule
