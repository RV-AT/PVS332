/*
此总线是PRV332SV0处理器的AHB总线接口，适配在BIU模块之中
*/
module ahb(

input clk,
input rst,

//ahb
output [33:0]haddr,
output hwrite,
output [2:0]hsize,
output [2:0]hburst,
output [3:0]hprot,
output [1:0]htrans,
output hmastlock,
output [31:0]hwdata,

input wire hready,
input wire hresp,
input wire hreset_n,
input wire [31:0]hrdata,

//对BIU内部信号

input r32,
input r16,
input r8,
input w32,
input w16,
input w8,
output ahb_acc_fault,
output [31:0]data_out,
output rdy_ahb,
input [33:0]addr_in,
input [31:0]data_in

);
reg statu_ahb;//ahb总线状态机转换
reg [31:0]data_ahb;
always@(posedge clk)begin
	if(rst)begin
		statu_ahb <= 1'b0;
	end
	else if(statu_ahb==1'b0)begin
		statu_ahb <= (r32|r16|r8|w32|w16|w8)?1'b1 : statu_ahb;
	end
	else if(statu_ahb==1'b1)begin
		statu_ahb <= (hready|hresp)?1'b0:statu_ahb;
	end
	
end

assign haddr= addr_in;
assign hwrite= (w32|w16|w8)&(!statu_ahb);
//assign hsize = (w32|r32)?2'b10:((w16|r16)?2'b01:2'b00);//modified
assign hsize=
(
	{2{w32|r32}}&2'b10|
	{2{w16|r16}}&2'b01|
	2'b00
);

assign hburst= 3'b000;
assign hprot = 4'b0011;
assign htrans= ((!statu_ahb)&(w32|w16|w8|r32|r16|r8))?2'b10:2'b00; //modified
assign hmastlock= 1'b0;
assign hwdata = statu_ahb?data_in : 32'b0;

assign ahb_acc_fault = statu_ahb&hresp;
assign rdy_ahb = statu_ahb&hready;

assign data_out = ({32{statu_ahb&hready}}&hrdata)|data_ahb;



always@(posedge clk)begin
	data_ahb <= (rst)?32'b0:(statu_ahb&hready)?hrdata : data_ahb;
	
end

endmodule
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
/*
BIU for PRV332SV0 Module
Engineer:Jack,pan
Company:CQUPT
2019 9.4 v0.0
2019 9.12 V0.1
*/
module biu(

input clk,
input rst,
//对ahb的信号
output [33:0]haddr,
output hwrite,
output [1:0]hsize,
output [2:0]hburst,
output [3:0]hprot,
output [1:0]htrans,
output hmastlock,
output [31:0]hwdata,

input wire hready,
input wire hresp,
input wire hreset_n,
input wire [31:0]hrdata,

//操作码
input wire[2:0]opc,
//机器状态输入
input wire[3:0]statu_cpu,
//pmp单元使用的信号
output wire [33:0]addr_out,
//pmp检查错误信号
input wire pmp_chk_fault,
//satp寄存器输入
input wire [31:0]satp,
//地址输入
input wire [31:0]addr,
//pc输入
input wire [31:0]pc,
//数据输入
input wire [31:0]biu_data_in,
//数据输出
output reg [31:0]biu_data_out,
//当前机器状态输入
input wire [1:0]msu,
output reg [31:0]ins,

//biu准备好信号
output wire rdy_biu,

//mxr,sum输入
input wire mxr,
input wire sum,

//异常报告

output wire ins_addr_mis,
output wire ins_acc_fault,
output wire load_addr_mis,
output wire load_acc_fault,
output wire st_addr_mis,
output wire st_acc_fault,
output wire ins_page_fault,
output wire ld_page_fault,
output wire st_page_fault


);

//biu主状态机状态
parameter stb		=7'b0000000;
parameter rdy		=7'b0000001;
parameter err		=7'b0000010;
parameter ifnp		=7'b0001000;
parameter ifwp0	=7'b0010000;
parameter ifwp1	=7'b0010001;
parameter ifwp2	=7'b0010010;
parameter ifwp3	=7'b0010011;
parameter ifwp4	=7'b0010100;
parameter r32np	=7'b0011000;
parameter r32wp0	=7'b0100000;
parameter r32wp1	=7'b0100001;
parameter r32wp2	=7'b0100010;
parameter r32wp3	=7'b0100011;
parameter r32wp4	=7'b0100100;
parameter r16np	=7'b0101000;
parameter r16wp0	=7'b0110000;
parameter r16wp1	=7'b0110001;
parameter r16wp2	=7'b0110010;
parameter r16wp3	=7'b0110011;
parameter r16wp4	=7'b0110100;
parameter r8np		=7'b0111000;
parameter r8wp0	=7'b1000000;
parameter r8wp1	=7'b1000001;
parameter r8wp2	=7'b1000010;
parameter r8wp3	=7'b1000011;
parameter r8wp4	=7'b1000100;
parameter w32np	=7'b1001000;
parameter w32wp0	=7'b1010000;
parameter w32wp1	=7'b1010001;
parameter w32wp2	=7'b1010010;
parameter w32wp3	=7'b1010011;
parameter w32wp4	=7'b1010100;
parameter w16np	=7'b1011000;
parameter w16wp0	=7'b1100000;
parameter w16wp1	=7'b1100001;
parameter w16wp2	=7'b1100010;
parameter w16wp3	=7'b1100011;
parameter w16wp4	=7'b1100100;
parameter w8np		=7'b1101000;
parameter w8wp0	=7'b1110000;
parameter w8wp1	=7'b1110001;
parameter w8wp2	=7'b1110010;
parameter w8wp3	=7'b1110011;
parameter w8wp4	=7'b1110100;
//opc_biu
parameter opw8 = 3'b001;
parameter opw16= 3'b010;
parameter opw32= 3'b011;
parameter opr8 = 3'b101;
parameter opr16= 3'b110;
parameter opr32= 3'b111;

reg [6:0]statu_biu;

//通AHB单元的地址线
wire [33:0]addr_ahb;
//送AHB单元的数据线
wire [31:0]data_ahb;
//AHB输出数据线
wire [31:0]data_ahb_out;
//AHB准备好
wire rdy_ahb;
//过数据交换机送AHB之前的数据线
wire [31:0]data_dcv;
//ahb输出过数据交换机的数据线
wire [31:0]data_dcv_out;
//送mmu的基础地址总线
wire [33:0]addr_mmu;
//mmu输出地址总线
wire [31:0]addr_mmu_in;
//数据过多路复用器送mmu
wire [31:0]data_mmu;
//mmu输出新页表
wire [31:0]pte;
//ahb控制信号
wire w32;
wire w16;
wire w8;
wire r32;
wire r16;
wire r8;
//mmu报告错误


wire ahb_acc_fault;	//ahb总线出错信号
wire mmu_ld_page_fault;//mmu页面读取错误信号
wire mmu_st_page_fault;//mmu页面写入错误信号
wire page_not_value; //页面不存在信号
wire pmp_chk_err;		  //pmp单元检查错误信号
wire addr_mis;			  //地址不对齐错误信号

//biu主状态机
always@(posedge clk)begin
		if(rst)begin
			statu_biu <= stb;
		end
		else if(statu_biu==stb)begin
			if((statu_cpu==4'b0000)&(!addr_mis))begin
				statu_biu <= satp[31]?ifwp0:ifnp;
			end
			else if((statu_cpu[2:0]==3'b010)&(opc==3'b001)&(!addr_mis))begin
				statu_biu <= satp[31]?w8wp0 : w8np;
			end
			else if((statu_cpu[2:0]==3'b010)&(opc==3'b010)&(!addr_mis))begin
				statu_biu <= satp[31]?w16wp0 : w16np;
			end
			else if((statu_cpu[2:0]==3'b010)&(opc==3'b011)&(!addr_mis))begin
				statu_biu <= satp[31]?w32wp0 : w32np;
			end
			else if((statu_cpu[2:0]==3'b010)&(opc==3'b101)&(!addr_mis))begin
				statu_biu <= satp[31]?r8wp0 : r8np;
			end
			else if((statu_cpu[2:0]==3'b010)&(opc==3'b110)&(!addr_mis))begin
				statu_biu <= satp[31]?r16wp0 : r16np;
			end
			else if((statu_cpu[2:0]==3'b010)&(opc==3'b111)&(!addr_mis))begin
				statu_biu <= satp[31]?r32wp0 : r32np;
			end
			else begin
				statu_biu <= statu_biu;
			end
		
		end
//if状态机转换		
		else if(statu_biu==ifnp)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
		
		else if(statu_biu==ifwp0)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? ifwp1 : statu_biu;
		end
		
		else if(statu_biu==ifwp1)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? ifwp2 : statu_biu;
		end
				
		else if(statu_biu==ifwp2)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? ifwp3 : statu_biu;
		end
	
		else if(statu_biu==ifwp3)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? ifwp4 : statu_biu;
		end
	
		else if(statu_biu==ifwp4)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
		
		else if(statu_biu==rdy)begin
				statu_biu <= stb;
		end
//w32状态机转换		
		else if(statu_biu==w32np)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
		
		else if(statu_biu==w32wp0)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? w32wp1 : statu_biu;
		end
		
		else if(statu_biu==w32wp1)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? w32wp2 : statu_biu;
		end
				
		else if(statu_biu==w32wp2)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? w32wp3 : statu_biu;
		end
	
		else if(statu_biu==w32wp3)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? w32wp4 : statu_biu;
		end
	
		else if(statu_biu==w32wp4)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
	
//w16状态机转换		
		else if(statu_biu==w16np)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
		
		else if(statu_biu==w16wp0)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? w16wp1 : statu_biu;
		end
		
		else if(statu_biu==w16wp1)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? w16wp2 : statu_biu;
		end
				
		else if(statu_biu==w16wp2)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? w16wp3 : statu_biu;
		end
	
		else if(statu_biu==w16wp3)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? w16wp4 : statu_biu;
		end
	
		else if(statu_biu==w16wp4)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
	
//w8状态机转换		
		else if(statu_biu==w8np)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
		
		else if(statu_biu==w8wp0)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? w8wp1 : statu_biu;
		end
		
		else if(statu_biu==w8wp1)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? w8wp2 : statu_biu;
		end
				
		else if(statu_biu==w8wp2)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? w8wp3 : statu_biu;
		end
	
		else if(statu_biu==w8wp3)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? w8wp4 : statu_biu;
		end
	
		else if(statu_biu==w8wp4)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
//r32状态机转换		
		else if(statu_biu==r32np)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
		
		else if(statu_biu==r32wp0)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? r32wp1 : statu_biu;
		end
		
		else if(statu_biu==r32wp1)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? r32wp2 : statu_biu;
		end
				
		else if(statu_biu==r32wp2)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? r32wp3 : statu_biu;
		end
	
		else if(statu_biu==r32wp3)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? r32wp4 : statu_biu;
		end
	
		else if(statu_biu==r32wp4)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
	
//r16状态机转换		
		else if(statu_biu==r16np)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
		
		else if(statu_biu==r16wp0)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? r16wp1 : statu_biu;
		end
		
		else if(statu_biu==r16wp1)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? r16wp2 : statu_biu;
		end
				
		else if(statu_biu==r16wp2)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? r16wp3 : statu_biu;
		end
	
		else if(statu_biu==r16wp3)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? r16wp4 : statu_biu;
		end
	
		else if(statu_biu==r16wp4)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
	
//r8状态机转换		
		else if(statu_biu==r8np)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end
		
		else if(statu_biu==r8wp0)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? r8wp1 : statu_biu;
		end
		
		else if(statu_biu==r8wp1)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? r8wp2 : statu_biu;
		end
				
		else if(statu_biu==r8wp2)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? r8wp3 : statu_biu;
		end
	
		else if(statu_biu==r8wp3)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis|page_not_value) ? stb : (rdy_ahb&(mmu_ld_page_fault|mmu_st_page_fault))?stb : rdy_ahb ? r8wp4 : statu_biu;
		end
	
		else if(statu_biu==r8wp4)begin
				statu_biu <= (ahb_acc_fault|pmp_chk_err|addr_mis) ? stb : rdy_ahb ? rdy : statu_biu;
		end	
	

end

assign pmp_chk_err = pmp_chk_fault;

assign addr_mmu_in = ((statu_biu==ifnp)|(statu_biu==ifwp0)|(statu_biu==ifwp1)|(statu_biu==ifwp1)|(statu_biu==ifwp2)|(statu_biu==ifwp3)|(statu_biu==ifwp4))?pc:addr;
assign addr_ahb = (statu_biu==ifnp)?{2'b0,pc}:((statu_biu==w32np)|(statu_biu==w16np)|(statu_biu==w8np)|(statu_biu==r32np)|(statu_biu==r16np)|(statu_biu==r8np)|(statu_biu==rdy))?{2'b0,addr}:addr_mmu;

//送AHB单元数据交换器					
assign data_ahb = ((statu_biu==ifwp1)|(statu_biu==ifwp3)|(statu_biu==w32wp1)|(statu_biu==w32wp3)|
						(statu_biu==w16wp1)|(statu_biu==w16wp3)|(statu_biu==w8wp1) |(statu_biu==w8wp3)|
						(statu_biu==r32wp1)|(statu_biu==r32wp3)|(statu_biu==r16wp1)|(statu_biu==r16wp3)|
						(statu_biu==r16wp1)|(statu_biu==r8wp3)| //写回页表的时候不进行数据交换
						(statu_biu==w32np)|(statu_biu==w32wp4)|//32位内存访问时不进行数据交换
						(((statu_biu==w16np)|(statu_biu==w16wp4)|(statu_biu==w8np)|(statu_biu==w8wp4))&(addr_ahb[1:0]==2'b00))) ? data_dcv://16,8位访问且地址对齐
						(((statu_biu==w16np)|(statu_biu==w16wp4)|(statu_biu==w8np)|(statu_biu==w8wp4))&(addr_ahb[1:0]==2'b01))?{8'b0,data_dcv[15:0],8'b0}:
						(((statu_biu==w16np)|(statu_biu==w16wp4)|(statu_biu==w8np)|(statu_biu==w8wp4))&(addr_ahb[1:0]==2'b10))?{data_dcv[15:0],16'b0}:
						(((statu_biu==w8np)|(statu_biu==w8wp4))&(addr_ahb[1:0]==2'b11))?{data_dcv[7:0],24'b0}:data_dcv;

assign data_dcv = ((statu_biu==ifwp1)|(statu_biu==ifwp3)|(statu_biu==w32wp1)|(statu_biu==w32wp3)|
						 (statu_biu==w16wp1)|(statu_biu==w16wp3)|(statu_biu==w8wp1)|
						 (statu_biu==w8wp3)|(statu_biu==r32wp1)|(statu_biu==r32wp3)|
						 (statu_biu==r16wp1)|(statu_biu==r16wp3)|(statu_biu==r8wp1)|(statu_biu==r8wp3))?pte:biu_data_in;
						 
assign addr_mis =  ((statu_cpu[2:0]==3'b010)&(((opc[1:0]==2'b11)&(addr[1:0]!=2'b00))|((opc[1:0]==2'b10)&(addr[1:0]==2'b11))))|	//内存访问时，全长访问，半场访问，字节访问造成的地址不对齐异常
						 (statu_cpu==4'b0000)&(pc[1:0]!=2'b00);
						 
assign data_mmu=data_ahb_out;

assign data_dcv_out = (((opc==opr16)|(opc==opr8))&(addr_ahb[1:0]==2'b00)) ? data_ahb_out://16,8位访问且地址对齐
						(((opc==opr16))&(addr_ahb[1:0]==2'b01))?{16'b0,data_ahb_out[23:8]}:
						(((opc==opr16))&(addr_ahb[1:0]==2'b10))?{16'b0,data_ahb_out[31:16]}:
						((opc==opr8)&(addr_ahb[1:0]==2'b01))?{24'b0,data_ahb_out[15:8]}:
						((opc==opr8)&(addr_ahb[1:0]==2'b10))?{24'b0,data_ahb_out[23:16]}:
						(((opc==opr8))&(addr_ahb[1:0]==2'b11))?{24'b0,data_ahb_out[31:24]}:data_ahb_out;

//pmp单元检查错误信号
assign addr_out = addr_ahb;
//ahb总线控制信号
assign w32 = (!pmp_chk_fault&!page_not_value&!addr_mis)&((statu_biu==ifwp1)|(statu_biu==ifwp3)|(statu_biu==w32np)|(statu_biu==w32wp1)|(statu_biu==w32wp3)|(statu_biu==w32wp4)|
				 (statu_biu==w16wp1)|(statu_biu==w16wp3)|(statu_biu==w8wp1)|(statu_biu==w8wp3)|(statu_biu==r32wp1)|
				 (statu_biu==r32wp3)|(statu_biu==r16wp1)|(statu_biu==r16wp3)|(statu_biu==r8wp1)|(statu_biu==r8wp3));

assign w16 = (!pmp_chk_fault&!page_not_value&!addr_mis)&((statu_biu==w16np)|(statu_biu==w16wp4));
assign w8  = (!pmp_chk_fault&!page_not_value&!addr_mis)&((statu_biu==w8np)|(statu_biu==w8wp4));

assign r32 = (!pmp_chk_fault&!page_not_value&!addr_mis)&((statu_biu==ifnp)|(statu_biu==ifwp0)|(statu_biu==ifwp2)|(statu_biu==ifwp4)|(statu_biu==w32wp0)|(statu_biu==w32wp2)|(statu_biu==w16wp0)|(statu_biu==w16wp2)|
	     (statu_biu==w8wp0)|(statu_biu==w8wp2)|(statu_biu==r8wp0)|(statu_biu==r8wp2)|(statu_biu==r16wp0)|(statu_biu==r16wp2)|(statu_biu==r32wp0)|(statu_biu==r32wp2)|
		(statu_biu==r32np)|(statu_biu==r32wp4));
assign r16 = (!pmp_chk_fault&!page_not_value&!addr_mis)&((statu_biu==r16np)|(statu_biu==r16wp4));
assign r8  = (!pmp_chk_fault&!page_not_value&!addr_mis)&((statu_biu==r8np)|(statu_biu==r8wp4));

assign rdy_biu=(statu_biu==rdy);

						
always@(posedge clk)begin
		biu_data_out <= rst?32'b0:((statu_cpu[1:0]==2'b10)&(statu_biu==rdy))?data_dcv_out:biu_data_out;
		ins			 <= rst?32'b0:((statu_cpu==4'b0000)&(statu_biu==rdy))?data_ahb_out:ins;

end

mmu mmu(
.clk (clk),
.rst (rst),
.statu_biu (statu_biu),
.data_in (data_mmu),
.addr (addr_mmu_in),
.satp (satp),
.addr_mmu (addr_mmu),
.pte_new (pte),
.mxr(mxr),
.sum(sum),
.msu(msu),
.ld_page_fault(mmu_ld_page_fault),
.st_page_fault(mmu_st_page_fault),
.page_not_value(page_not_value)

);

ahb ahb(
.clk(clk),
.rst(rst),

.haddr(haddr),
.hwrite(hwrite),
.hsize(hsize),
.hburst(hburst),
.hprot(hprot),
.htrans(htrans),
.hmastlock(hmastlock),
.hwdata(hwdata),

.hready(hready),
.hresp(hresp),
.hreset_n(hreset_n),
.hrdata(hrdata),

.r32(r32),
.r16(r16),
.r8(r8),
.w32(w32),
.w16(w16),
.w8(w8),
.ahb_acc_fault(ahb_acc_fault),
.data_out(data_ahb_out),
.rdy_ahb(rdy_ahb),
.addr_in(addr_ahb),
.data_in(data_ahb)


);

exce_chk exce_chk(

//biu当前状态输入
.statu_biu(statu_biu),
.statu_cpu(statu_cpu),
.opc(opc),
.rdy_ahb(rdy_ahb),

//错误输入
//pmp检查出错
.pmp_chk_fault(pmp_chk_fault),
//ahb总线出错
.ahb_acc_fault(ahb_acc_fault),
.addr_mis(addr_mis),
.page_not_value(page_not_value),
//mmu单元与页表不符合造成的错误
.mmu_ld_page_fault(mmu_ld_page_fault),
.mmu_st_page_fault(mmu_st_page_fault),

//对外报告异常类型
//注，对外报告错误有可能需要延迟一个周期的持续时间
.ins_addr_mis(ins_addr_mis),
.ins_acc_fault(ins_acc_fault),
.load_addr_mis(load_addr_mis),
.load_acc_fault(load_acc_fault),
.st_addr_mis(st_addr_mis),
.st_acc_fault(st_acc_fault),
.ins_page_fault(ins_page_fault),
.ld_page_fault(ld_page_fault),
.st_page_fault(st_page_fault)


);


             					
endmodule
	
	
	
	
	
	
	
	
	
	
	
	
/*

适用于PRV332SV0处理器的CSR单元

*/
module csr(

input wire clk,
input wire rst,  

input wire [3:0]statu_cpu,
output reg [31:0]pc,
output reg [1:0]msu,		//处理器权限输出 
//pmp检查信号
input [33:0]pmp_addr,
output wire pmp_chk_fault,
//执行阶段需要用到的信号
output wire [31:0]csr_out,

output reg mxr,
output reg sum,
//中断发生时需要用到的信号
output wire mie_out,
output wire sie_out,
output wire [31:0]medeleg_out,
output wire [31:0]mideleg_out,
//mie
output wire meie_out,
output wire seie_out,
output wire mtie_out,
output wire stie_out,
output wire msie_out,
output wire ssie_out,
//mip
output wire ssip_out,
output wire msip_out,
output wire stip_out,
output wire mtip_out,
output wire seip_out,
output wire meip_out,
input wire [1:0]priv_d,		//发生异常的时候要更改的目的权限 mcause
input wire [31:0]cause,		//异常发生的原因
input wire [31:0]tval,		//异常值
output wire tsr_out,
output wire tvm_out,
output wire [31:0]satp_out,
//wb阶段需要用到的额信号
input wire [31:0]csr_in,
input wire [11:0]csr_index,//csr索引
input wire [31:0]pc_next,
input wire csr_wr,				//除开mip和sip的csr写请求
//在wb阶段进行mip和sip寄存器的更新
input wire mtip_in,
input wire mtip_wr,
input wire meip_in,
input wire meip_wr,
input wire msip_in,
input wire msip_wr,
input wire stip_in,
input wire stip_wr,
input wire seip_in,
input wire seip_wr,
input wire ssip_in,
input wire ssip_wr,


input wire ret						//返回信号

);
//处理器状态编码
parameter if0 = 4'b0000;
parameter ex0 = 4'b0001;
parameter mem0= 4'b0010;
parameter mem1=4'b1010;
parameter ex1 = 4'b1001;
parameter wb = 4'b0011;
parameter exc = 4'b1111;
//处理器权限编码
parameter m = 2'b11;
parameter h = 2'b10; 
parameter s = 2'b01;
parameter u = 2'b00;
//csr索引编码
parameter mcycle_index   = 12'hb00;		//机器运行周期计数
parameter minstret_index = 12'hb02;		//机器执行指令计数
parameter mstatus_index  = 12'h300;
parameter medeleg_index  = 12'h302;
parameter mideleg_index  = 12'h303;
parameter mie_index      = 12'h304;
parameter mtvec_index    = 12'h305;
parameter mscratch_index = 12'h340;
parameter mepc_index     = 12'h341;
parameter mcause_index   = 12'h342;
parameter mtval_index    = 12'h343;
parameter mip_index 		 = 12'h344;
parameter pmpcfg0_index  = 12'h3a0;
parameter pmpcfg1_index  = 12'h3a1;
parameter pmpcfg2_index  = 12'h3a2;
parameter pmpcfg3_index  = 12'h3a3;
parameter pmpaddr0_index = 12'h3b0;
parameter pmpaddr1_index = 12'h3b1;
parameter pmpaddr2_index = 12'h3b2;
parameter pmpaddr3_index = 12'h3b3;
parameter sstatus_index  = 12'h100;
parameter sie_index 		 = 12'h104;
parameter stvec_index	 = 12'h105;
parameter sscratch_index = 12'h140;
parameter sepc_index  	 = 12'h141;
parameter scause_index	 = 12'h142;
parameter stval_index 	 = 12'h143;
parameter sip_index		 = 12'h144;
parameter satp_index		 = 12'h180;
//mcause编码
parameter usint =32'h8000000; 
parameter ssint =32'h8000001;
parameter msint =32'h8000003; 
parameter utint =32'h8000004; 
parameter stint =32'h8000005; 
parameter mtint =32'h8000007; 
parameter ueint =32'h8000008;
parameter seint =32'h8000009; 
parameter meint =32'h800000b; 
parameter iam	 =32'h0000000;
parameter iaf   =32'h0000001;
parameter ii    =32'h0000002;
parameter bk    =32'h0000003;
parameter lam	 =32'h0000004;
parameter laf	 =32'h0000005;
parameter sam	 =32'h0000006;
parameter saf	 =32'h0000007;
parameter ecu	 =32'h0000008;
parameter ecs	 =32'h0000009;
parameter ecm	 =32'h000000b;
parameter ipf	 =32'h000000c;
parameter lpf	 =32'h000000d;
parameter spf	 =32'h000000f;
//pc复位值
parameter pc_rst=32'h00000000;


//m模式下使用的csr
//mcycle
reg [31:0]mcycle;
//minstret
reg [31:0]minstret;

//mstatus
reg tsr;
reg tvm;

reg mprv;
reg [1:0]mpp;
reg spp;
reg mpie;
reg spie;
reg mie;
reg sie;
//mtvec
reg [31:0]mtvec;
//mepc
reg [31:0]mepc;
//mcause
reg [31:0]mcause;
//mtval
reg [31:0]mtval;
//mideleg
reg [31:0]mideleg;
//medeleg
reg [31:0]medeleg;
//mip
reg meip;	//只读
reg seip;	//m模式下读写，s模式下只读
reg mtip;	//只读
reg stip;	//m模式下读写，s模式下只读
reg msip;	//只读
reg ssip;	//读写
//mie
reg meie;
reg seie;
reg mtie;
reg stie;
reg msie;
reg ssie;
//mscratch
reg [31:0]mscratch;
//pmpcfg
reg [7:0]pmp0cfg;
reg [7:0]pmp1cfg;
reg [7:0]pmp2cfg;
reg [7:0]pmp3cfg;
reg [7:0]pmp4cfg;
reg [7:0]pmp5cfg;
reg [7:0]pmp6cfg;
reg [7:0]pmp7cfg;
//pmpaddr
reg [31:0]pmpaddr0;
reg [31:0]pmpaddr1;
reg [31:0]pmpaddr2;
reg [31:0]pmpaddr3;

//s模式下使用的csr
//sstatus

//stvec
reg [31:0]stvec;
//stval
reg [31:0]stval;
//sie
//sip
//sepc
reg [31:0]sepc;
//scause
reg [31:0]scause;

//sscratch
reg [31:0]sscratch;
//satp
reg [31:0]satp;

assign meie_out=meie;
assign seie_out=seie;
assign mtie_out=mtie;
assign stie_out=stie;
assign msie_out=msie;
assign ssie_out=ssie;

always@(posedge clk)begin
//复位状态
	if(rst)begin
		msu<= m;
		//m模式下使用的csr
		//mstatus
		tsr <= 1'b0;
		tvm <= 1'b0;
		mxr <= 1'b0;
		sum <= 1'b0;
		mprv<= 1'b0;
		mpp <= 2'b0;
		spp <= 1'b0;
		mpie<= 1'b0;
		spie<= 1'b0;
		mie <= 1'b0;
		sie <= 1'b0;
		//mtvec
		mtvec <= 32'b0;
		//mepc
		mepc  <= 32'b0;
		//mcause
		mcause <=32'b0;
		//mtval
		//mideleg
		mideleg<=32'b0;
		//medeleg
		medeleg<=32'b0;

		//mie
		meie <= 1'b0;
		seie <= 1'b0;
		mtie <= 1'b0;
		stie <= 1'b0;
		msie <= 1'b0;
		ssie <= 1'b0;
		//mscratch
		mscratch <= 32'b0;
		//pmpcfg
		pmp0cfg <= 8'b0;
		pmp1cfg <= 8'b0;
		pmp2cfg <= 8'b0;
		pmp3cfg <= 8'b0;
		pmp4cfg <= 8'b0;
		pmp5cfg <= 8'b0;
		pmp6cfg <= 8'b0;
		pmp7cfg <= 8'b0;
		//pmpaddr
		pmpaddr0 <= 32'b0;
		pmpaddr1 <= 32'b0;
		pmpaddr2 <= 32'b0;
		pmpaddr3 <= 32'b0;

		//s模式下使用的csr
		//sstatus

		//stvec
		stvec <= 32'b0;
		//sie
		//sip
		//sscratch
		sscratch <= 32'b0;
		pc <= pc_rst;
		satp <= 32'b0;
	end
	//此段为wb阶段的除去mip和sip寄存器的更新，
	else if((statu_cpu==wb)&csr_wr)begin

		if(csr_index==mstatus_index)begin
			tsr <= csr_in[22];
			tvm <= csr_in[20];
			mxr <= csr_in[19];
			sum <= csr_in[18];
			mprv<= csr_in[17];
			mpp <= csr_in[12:11];
			spp <= csr_in[8];
			mpie<= csr_in[7];
			spie<= csr_in[5];
			mie <= csr_in[3];
			sie <= csr_in[1];
			pc	 <= pc_next;
		end
		else if(csr_index==medeleg_index)begin
			medeleg <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==mideleg_index)begin
			mideleg <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==mie_index)begin
			meie <= csr_in[11];
			seie <= csr_in[9];
			mtie <= csr_in[7];
			stie <= csr_in[5];
			msie <= csr_in[3];
			ssie <= csr_in[1];
			pc	 <= pc_next;
		end
		else if(csr_index==mtvec_index)begin
			mtvec <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==mscratch_index)begin
			mscratch <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==mepc_index)begin
			mepc		<= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==mcause_index)begin
			mcause <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==mtval_index)begin
			mtval <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==pmpcfg0_index)begin
			pmp0cfg <= csr_in[7:0];
			pmp1cfg <= csr_in[15:8];
			pmp2cfg <= csr_in[23:16];
			pmp3cfg <= csr_in[31:24];
			pc	 <= pc_next;
		end
		else if(csr_index==pmpcfg1_index)begin
			pmp4cfg <= csr_in[7:0];
			pmp5cfg <= csr_in[15:8];
			pmp6cfg <= csr_in[23:16];
			pmp7cfg <= csr_in[31:24];
			pc	 <= pc_next;
		end
		else if(csr_index==pmpaddr0_index)begin
			pmpaddr0 <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==pmpaddr1_index)begin
			pmpaddr1 <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==pmpaddr2_index)begin
			pmpaddr2 <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==pmpaddr3_index)begin
			pmpaddr3 <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==sstatus_index)begin
			mxr <= csr_in[19];
			sum <= csr_in[18];
			spp <= csr_in[8];
			
			sie <= csr_in[1]; 
			
			pc	 <= pc_next;
		end
		else if(csr_index==sie_index)begin
			seie <= csr_in[9];
			stie <= csr_in[5];
			ssie <= csr_in[1];
			pc	 <= pc_next;
		end
		else if(csr_index==stvec_index)begin
			stvec <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==sscratch_index)begin
			sscratch <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==sepc_index)begin
			sepc <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==scause_index)begin
			scause <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==stval_index)begin
			stval <= csr_in;
			pc	 <= pc_next;
		end
		else if(csr_index==satp_index)begin
			satp <= csr_in;
			pc	 <= pc_next;
		end
	end
	//返回指令
	else if((statu_cpu==wb)&(ret))begin
		if(msu==m)begin
			msu <= mpp;
			mie <= mpie;
			pc	 <= mepc;
		end
		else if(msu==s)begin
			msu <= spp ? s : u;
			sie <= spie;
			pc  <= sepc;
		end
	end
	else if((statu_cpu==wb)&(!ret)&(!csr_wr))begin
		pc <= pc_next;
	end
	//发生异常
	else if(statu_cpu==exc)begin
		if(priv_d==m)begin
			mepc <= ((cause==ecu)|(cause==ecs)|(cause==ecm)|(cause==bk))? (pc+32'd4) : pc;	//当发生的异常是ecall和ebreak的时候对pc+4
			mtval<= tval;
			mcause <= cause;
			mpie <= mie;
			mie  <= 1'b0;
			mpp  <= msu;
			msu  <= m;
			pc   <= (mtvec[1:0]==2'b00) ? {mtvec[31:2],2'b00} : (!mcause[31])? {mtvec[31:3],2'b00} : 
						{mtvec[31:2],2'b00} + {22'b0,cause[7:0],2'b00};
		end
		else if(priv_d==s)begin
			sepc <= ((cause==ecu)|(cause==ecs)|(cause==ecm)|(cause==bk))? (pc+32'd4) : pc;
			stval<= tval;
			scause <= cause;
			spie <= mie;
			sie  <= 1'b0;
			spp  <= (msu==u)?1'b0:1'b1;
			msu  <= s;
			pc   <= (stvec[1:0]==2'b00) ? {stvec[31:2],2'b00} : (!scause[31])? {stvec[31:3],2'b00} : 
						{stvec[31:2],2'b00} + {22'b0,cause[7:0],2'b00};
		end
	end
	
end
//mip和sip寄存器更新
always@(posedge clk)begin
	if(rst)begin			//mip
		meip <= 1'b0;	//只读
		seip <= 1'b0;	//m模式下读写，s模式下只读
		mtip <= 1'b0;	//只读
		stip <= 1'b0;	//m模式下读写，s模式下只读
		msip <= 1'b0;	//只读
		ssip <= 1'b0;	//读写
	end
	else begin
		meip <= meip_wr?meip_in:meip;
		seip <= seip_wr?seip_in:seip;
		mtip <= mtip_wr?mtip_in:mtip;
		stip <= stip_wr?stip_in:stip;
		msip <= msip_wr?msip_in:msip;
	   ssip <= ssip_wr?ssip_in:ssip;
	end
end

//mcycle寄存器更新
always@(posedge clk)begin
	if(rst)begin
		mcycle <= 32'b0;
		minstret <= 32'b0;
	end
	else if(statu_cpu==wb)begin	//每次当处理器运行到写回阶段的时候进行minstret寄存器更新
		if(csr_index==mcycle)begin
			mcycle <= csr_in;
		end
		else if(csr_index==minstret)begin
			minstret <= csr_in;
		end
		else begin	
			minstret <= minstret+32'd1;
		end
	end
	else begin
		mcycle <= mcycle + 32'd1;	//每次处理器不在写回的时候进行mcycyle寄存器更新
	end	
end
			

assign csr_out = ((csr_index==mcycle_index)?mcycle:32'b0)|
					  ((csr_index==minstret_index)?minstret:32'b0)|
					  ((csr_index==mstatus_index)?{9'b0,tsr,1'b0,tvm,mxr,sum,mprv,4'b0,mpp,2'b0,spp,mpie,1'b0,spie,1'b0,mie,1'b0,sie,1'b0}:32'b0)|
					  ((csr_index==medeleg_index)?medeleg :32'b0)|
					  ((csr_index==medeleg_index)?mideleg :32'b0)|
					  ((csr_index==mie_index)?{20'b0,meie,1'b0,seie,1'b0,mtie,1'b0,stie,1'b0,msie,1'b0,ssie,1'b0}:32'b0)|
					  ((csr_index==mtvec_index)?mtvec:32'b0)|
					  ((csr_index==mscratch_index)?mscratch:32'b0)|
					  ((csr_index==mepc_index)?mepc:32'b0)|
					  ((csr_index==mcause_index)?mcause:32'b0)|
					  ((csr_index==mtval_index)?mtval:32'b0)|
					  ((csr_index==mip_index)?{20'b0,meip,1'b0,seip,1'b0,mtip,1'b0,stip,1'b0,msip,1'b0,ssip,1'b0}:32'b0)|
					  ((csr_index==pmpcfg0_index)?{pmp3cfg,pmp2cfg,pmp1cfg,pmp0cfg}:32'b0)|
					  ((csr_index==pmpcfg1_index)?{pmp7cfg,pmp6cfg,pmp5cfg,pmp4cfg}:32'b0)|
					  ((csr_index==pmpaddr0_index)?pmpaddr0:32'b0)|
					  ((csr_index==pmpaddr1_index)?pmpaddr1:32'b0)|
					  ((csr_index==pmpaddr2_index)?pmpaddr2:32'b0)|
					  ((csr_index==pmpaddr3_index)?pmpaddr3:32'b0)|
					  ((csr_index==sstatus_index)?{12'b0,mxr,sum,1'b0,4'b0,4'b0,spp,2'b0,spie,1'b0,2'b0,sie,1'b0}:32'b0)|
					  ((csr_index==sie_index)?{22'b0,seie,1'b0,2'b0,stie,1'b0,2'b0,ssie,1'b0}:32'b0)|
					  ((csr_index==stvec_index)?stvec:32'b0)|
					  ((csr_index==sscratch_index)?sscratch:32'b0)|
					  ((csr_index==sepc_index)?sepc:32'b0)|
					  ((csr_index==scause_index)?scause:32'b0)|
					  ((csr_index==stval_index)?stval:32'b0)|
					  ((csr_index==sip_index)?{22'b0,seip,1'b0,2'b0,stip,1'b0,2'b0,ssip,1'b0}:32'b0)|
					  ((csr_index==satp_index)?satp:32'h00000000);
assign satp_out = satp;
assign pmp_chk_fault = 1'b0;
assign tsr_out = tsr;
assign tvm_out = tvm;
assign meie_out = meie;
assign seie_out = seie;
assign mtie_out = mtie;
assign stie_out = stie;
assign msie_out = msie;
assign ssie_out = ssie;
assign ssip_out = ssip;
assign msip_out = msip;
assign stip_out = stip;
assign mtip_out = mtip;
assign seip_out = seip;
assign meip_out = meip;

assign mie_out = mie;
assign sie_out = sie;
assign mideleg_out = mideleg;
assign medeleg_out = medeleg;

/*	
parameter mstatus_index  = 12'h300;
parameter medeleg_index  = 12'h302;
parameter mideleg_index  = 12'h303;
parameter mie_index      = 12'h304;
parameter mtvec_index    = 12'h305;
parameter mscratch_index = 12'h340;
parameter mepc_index     = 12'h341;
parameter mcause_index   = 12'h342;
parameter mtval_index    = 12'h343;
parameter mip_index 		 = 12'h344;
parameter pmpcfg0_index  = 12'h3a0;
parameter pmpcfg1_index  = 12'h3a1;
parameter pmpcfg2_index  = 12'h3a2;
parameter pmpcfg3_index  = 12'h3a3;
parameter pmpaddr0_index = 12'h3b0;
parameter pmpaddr1_index = 12'h3b1;
parameter pmpaddr2_index = 12'h3b2;
parameter pmpaddr3_index = 12'h3b3;
parameter sstatus_index  = 12'h100;
parameter sie_index 		 = 12'h104;
parameter stvec_index	 = 12'h105;
parameter sscratch_index = 12'h140;
parameter sepc_index  	 = 12'h141;
parameter scause_index	 = 12'h142;
parameter stval_index 	 = 12'h143;
parameter sip_index		 = 12'h144;
parameter satp_index		 = 12'h180;	
*/	
	
		
endmodule

/*
适用于PRV332SV0处理器的csr_gpr_iu单元 data_gpr
*/
module csr_gpr_iu(
input wire clk,
input wire rst,
//中断输入
input wire timer_int,
input wire soft_int,
input wire ext_int,

//模块准备好信号
//exu准备好信号
input wire rdy_exu,
//biu准备好信号
input wire rdy_biu,
output wire sum,
output wire mxr,
//satp信号
output wire [31:0]satp,
//pmp检查信号
input wire [33:0]pmp_addr,
output wire pmp_chk_fault,
//biu异常报告
input wire ins_addr_mis_in,
input wire ins_acc_fault_in,
input wire load_addr_mis_in,
input wire load_acc_fault_in,
input wire st_addr_mis_in,
input wire st_acc_fault_in,
input wire ins_page_fault_in,
input wire ld_page_fault_in,
input wire st_page_fault_in,

input wire [31:0]ins,
input wire [31:0]addr_biu,
//机器状态输出
output reg [3:0] statu_cpu,
output wire [1:0]msu,
output wire [31:0]pc,
//exu控制信号
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
//exu需要用到的操作数
output wire [4:0]rs1_index,
output wire [31:0]rs1,
output wire [31:0]rs2,
output wire [31:0]csr_out,
output wire [19:0]imm20,
output wire [11:0]imm12,
output wire [4:0] shamt,
//exu反馈的数
input wire [31:0]csr_in,
input wire [31:0]data_gpr,
input wire [31:0]pc_next,

//biu操作信号
output wire [2:0]opc_biu



);
parameter if0 = 4'b0000;
parameter ex0 = 4'b0001;
parameter mem0= 4'b0010;
parameter mem1=4'b1010;
parameter ex1 = 4'b1001;
parameter wb = 4'b0011;
parameter exc = 4'b1111;

//处理器权限编码
parameter m = 2'b11;
parameter h = 2'b10; 
parameter s = 2'b01;
parameter u = 2'b00;
//ins_flow,
parameter if_ex_mem_wb=4'b0001;
parameter if_ex_wb	 =4'b0010;
parameter if_ex_mem_ex_mem_wb=4'b0011;


//指令执行流信号
wire [3:0]ins_flow;

//异常指令信号
wire ill_ins;
wire ecall;
wire ebreak;

//fence信号
wire fence;

//csr写信号
wire csr_wr;
//gpr写信号
wire gpr_wr;
//中断受理信号
wire int_acc;
//返回指令信号
wire ret;

//中断等待位输入
wire stip;
wire ssip;
wire seip;
//中断使能位输入
wire sie;
wire mie;
wire mtie;
wire msie;
wire meie;
wire stie;
wire ssie;
wire seie;

wire [31:0]mideleg;
wire [31:0]medeleg;
wire msip;
wire mtip;
wire meip;

//送csr的信号
wire mtip_tocsr;
wire mtip_wr;
wire meip_tocsr;
wire meip_wr;
wire msip_tocsr;
wire msip_wr;
wire stip_tocsr;
wire stip_wr;
wire seip_tocsr;
wire seip_wr;
wire ssip_tocsr;
wire ssip_wr;

wire [1:0]priv_d; //中断目标权限
wire [31:0]cause;  //造成异常的原因
wire [31:0]tval;	  //中断值

//csr索引
wire [11:0]csr_index;
//gpr索引
wire [4:0]rs2_index;
wire [4:0]rd_index;

wire tvm;
wire tsr;

//biu异常报告(注意！此处需要将biu的异常报告寄存一排，保证exc阶段能捕获异常值
//biu异常报告
wire ins_addr_mis;
wire ins_acc_fault;
wire load_addr_mis;
wire load_acc_fault;
wire st_addr_mis;
wire st_acc_fault;
wire ins_page_fault;
wire ld_page_fault;
wire st_page_fault;


//通用寄存器组
reg [31:0] gpr [30:0];

reg ins_addr_mis_reg;
reg ins_acc_fault_reg;
reg load_addr_mis_reg;
reg load_acc_fault_reg;
reg st_addr_mis_reg;
reg st_acc_fault_reg;
reg ins_page_fault_reg;
reg ld_page_fault_reg;
reg st_page_fault_reg;

always@(posedge clk)begin
	if(rst)begin
		ins_addr_mis_reg 	<= 1'b0;
		ins_acc_fault_reg	<= 1'b0;
		load_addr_mis_reg <= 1'b0;
		load_acc_fault_reg<= 1'b0;
		st_addr_mis_reg	<= 1'b0;
		st_acc_fault_reg	<= 1'b0;
		ins_page_fault_reg<= 1'b0;
		ld_page_fault_reg	<= 1'b0;
		st_page_fault_reg	<= 1'b0;
	end
	else begin
		ins_addr_mis_reg 	<= ins_addr_mis_in;
		ins_acc_fault_reg	<= ins_acc_fault_in;
		load_addr_mis_reg <= load_addr_mis_in;
		load_acc_fault_reg<= load_acc_fault_in;
		st_addr_mis_reg	<= st_addr_mis_in;
		st_acc_fault_reg	<= st_acc_fault_in;
		ins_page_fault_reg<= ins_page_fault_in;
		ld_page_fault_reg	<= ld_page_fault_in;
		st_page_fault_reg	<= st_page_fault_in;	
	end
end
assign ins_addr_mis	=ins_addr_mis_reg  | ins_addr_mis_in;
assign ins_acc_fault =ins_acc_fault_reg | ins_acc_fault_in;
assign load_addr_mis =load_addr_mis_reg | load_addr_mis_in;
assign load_acc_fault=load_acc_fault_reg| load_acc_fault_in;
assign st_addr_mis   =st_addr_mis_reg	 | st_addr_mis_in;
assign st_acc_fault	=st_acc_fault_reg	 | st_acc_fault_in;
assign ins_page_fault=ins_page_fault_reg| ins_page_fault_in;
assign ld_page_fault =ld_page_fault_reg | ld_page_fault_in;
assign st_page_fault =st_page_fault_reg | st_page_fault_in;
//异常信号经过这样处理之后可以持续两个周期
//以上是异常寄存一拍的代码


//处理器状态转换
always@(posedge clk)begin
	if(rst)begin
		statu_cpu <= if0;
	end
	//处理器在if0状态，根据biu异常报告选择跳ex0还是exc
	else if(statu_cpu==if0)begin
		statu_cpu <=  (ins_addr_mis | ins_acc_fault | load_addr_mis | load_acc_fault|st_addr_mis |
		st_acc_fault | ins_page_fault | ld_page_fault | st_page_fault )?exc : rdy_biu?ex0 : statu_cpu;
	end
	//处理器在ex0状态，同时处理器正在译码指令，根据是否有非法指令和指令流指示选择下一个执行阶段
	else if(statu_cpu==ex0)begin
		statu_cpu <= (ill_ins|ecall|ebreak)?exc:(rdy_exu&(ins_flow==if_ex_mem_wb))?mem0:(rdy_exu&(ins_flow==if_ex_wb))?wb:
							(rdy_exu&(ins_flow==if_ex_mem_ex_mem_wb))?mem1:statu_cpu;
	end
	//处理器在mem0状态，处理器根据是否发生异常选择跳转对象
	else if(statu_cpu==mem0)begin
		statu_cpu <= (ins_addr_mis | ins_acc_fault | load_addr_mis | load_acc_fault|st_addr_mis |
		st_acc_fault | ins_page_fault | ld_page_fault | st_page_fault )?exc : rdy_biu?wb : statu_cpu;
	end
	//处理器在mem1状态
	else if(statu_cpu==mem1)begin
		statu_cpu <= (ins_addr_mis | ins_acc_fault | load_addr_mis | load_acc_fault|st_addr_mis |
		st_acc_fault | ins_page_fault | ld_page_fault | st_page_fault )?exc : rdy_biu?ex1 : statu_cpu;
	end
	//处理器在ex1状态
	else if(statu_cpu==ex1)begin
		statu_cpu <= rdy_exu ? mem0 : statu_cpu;
	end
	//处理器在wb状态
	else if(statu_cpu==wb)begin
		statu_cpu <= int_acc? exc : if0;
	end
	else if(statu_cpu==exc)begin
		statu_cpu <= if0;
	end
end

//通用寄存器组写
always@(posedge clk)begin
	if((statu_cpu==wb)&gpr_wr)begin
		if(rd_index==5'b0)begin
		end
		else begin
		gpr[rd_index-5'd1] <= data_gpr;
		end
	end
	else begin
	
	end
end


 (* DONT_TOUCH = "true" *)ins_dec ins_dec(
.ins(ins),
.statu_cpu(statu_cpu),
.msu(msu),					//当前处理器权限
.tsr(tsr),
.tvm(tvm),

.opc_biu(opc_biu),
//译码器结果输出(exu)
.addi(addi) ,
.slti(slti),
.sltiu(sltiu),
.andi(andi),
.ori(ori),
.xori(xori),
.slli(slli),
.srli(srli),
.srai(srai),

.lui(lui),
.auipc(auipc),
.addp(addp),
.subp(subp),
.sltp(sltp),
.sltup(sltup),
.andp(andp),
.orp(orp),
.xorp(xorp),
.sllp(sllp),
.srlp(srlp),
.srap(srap),

.jal(jal),
.jalr(jalr),

.beq(beq),
.bne(bne),
.blt(blt),
.bltu(bltu),
.bge(bge),
.bgeu(bgeu),
//load指令对数据进行符号位拓展信号
.lb(lb),
.lh(lh),

.csrrw(csrrw),
.csrrs(csrrs),
.csrrc(csrrc),
.csrrwi(csrrwi),
.csrrsi(csrrsi),
.csrrci(csrrci),
//amo指令添加
.lr_w(lr_w),
.sc_w(sc_w),
.amoswap(amoswap),
.amoadd(amoadd),
.amoxor(amoxor),
.amoand(amoand),
.amoor(amoor),
.amomin(amomin),
.amomax(amomax),
.amominu(amominu),
.amomaxu(amomaxu),

.csr_wr(csr_wr),

.gpr_wr(gpr_wr),

.ebreak(ebreak),
.ecall(ecall),
.fence(fence),
.ret(ret),
.rs1_index(rs1_index),
.rs2_index(rs2_index),
.rd_index(rd_index),
.csr_index(csr_index),

.imm20(imm20),
.imm12(imm12),
.shamt(shamt),

.ins_flow(ins_flow),				//指令流报告
.ill_ins(ill_ins)					   //非法指令

);

//中断裁决器，控制器

 (* DONT_TOUCH = "true" *)int_ctrl int_ctrl(
.clk(clk),
.rst(rst),
//处理器状态输入
.statu_cpu(statu_cpu),
//处理器权限输入
.msu(msu),
//pc值输入
.pc(pc),
//处理器外部的中断线输入
.timer_int_in(timer_int),
.soft_int_in(soft_int),
.ext_int_in(ext_int),

//中断异常委托输入
.mideleg(mideleg),
.medeleg(medeleg),

//csr要写的数据输入
.csr_in(csr_in),
.csr_index(csr_index),
.csr_wr(csr_wr),

//中断使能位输入
.sie(sie),
.mie(mie),
.mtie(mtie),
.msie(msie),
.meie(meie),
.stie(stie),
.ssie(ssie),
.seie(seie),
//中断等待位输入
.stip_in(stip),
.ssip_in(ssip),
.seip_in(seip),

//译码器异常报告
.ecall(ecall),
.ebreak(ebreak),
.ill_ins(ill_ins),
.ins(ins),

//处理器中断接收信号
.int_acc(int_acc),
//biu异常报告
.ins_addr_mis(ins_addr_mis),
.ins_acc_fault(ins_acc_fault),
.load_addr_mis(load_addr_mis),
.load_acc_fault(load_acc_fault),
.st_addr_mis(st_addr_mis),
.st_acc_fault(st_acc_fault),
.ins_page_fault(ins_page_fault),
.ld_page_fault(ld_page_fault),
.st_page_fault(st_page_fault),

//biu当前访问的va地址
.addr_biu(addr_biu),

//送csr的信号
.mtip(mtip_tocsr),
.mtip_wr(mtip_wr),
.meip(meip_tocsr),
.meip_wr(meip_wr),
.msip(msip_tocsr),
.msip_wr(msip_wr),
.stip(stip_tocsr),
.stip_wr(stip_wr),
.seip(seip_tocsr),
.seip_wr(seip_wr),
.ssip(ssip_tocsr),
.ssip_wr(ssip_wr),

.priv_d(priv_d), //中断目标权限
.cause(cause),  //造成异常的原因
.tval(tval)	  //中断值

);

 (* DONT_TOUCH = "true" *)csr csr(

.clk(clk),
.rst(rst),

.statu_cpu(statu_cpu),
.pc(pc),
.msu(msu),		//处理器权限输出
//pmp检查信号
.pmp_addr(pmp_addr),
.pmp_chk_fault(pmp_chk_fault),
//执行阶段需要用到的信号
.csr_out(csr_out),

.mxr(mxr),
.sum(sum),
//中断发生时需要用到的信号
.mie_out(mie),
.sie_out(sie),
.mideleg_out(mideleg),
.medeleg_out(medeleg),
//mie
.meie_out(meie),
.seie_out(seie),
.mtie_out(mtie),
.stie_out(stie),
.msie_out(msie),
.ssie_out(ssie),
//mip
.ssip_out(ssip),
.msip_out(msip),
.stip_out(stip),
.mtip_out(mtip),
.seip_out(seip),
.meip_out(meip),
.priv_d(priv_d),		//发生异常的时候要更改的目的权限
.cause(cause),		//异常发生的原因
.tval(tval),		//异常值
.tsr_out(tsr),
.tvm_out(tvm),
.satp_out(satp),
//wb阶段需要用到的额信号
.csr_in(csr_in),
.csr_index(csr_index),//csr索引
.pc_next(pc_next),
.csr_wr(csr_wr),				//除开mip和sip的csr写请求
//在wb阶段进行mip和sip寄存器的更新
.mtip_in(mtip_tocsr),
.mtip_wr(mtip_wr),
.meip_in(meip_tocsr),
.meip_wr(meip_wr),
.msip_in(msip_tocsr),
.msip_wr(msip_wr),
.stip_in(stip_tocsr),
.stip_wr(stip_wr),
.seip_in(seip_tocsr),
.seip_wr(seip_wr),
.ssip_in(ssip_tocsr),
.ssip_wr(ssip_wr),

.ret(ret)						//返回信号
);


//通用寄存器组数据输出rs1，rs2赋值
assign rs1 = (rs1_index==5'b0)?32'b0 : gpr[rs1_index-5'd1];
assign rs2 = (rs2_index==5'b0)?32'b0 : gpr[rs2_index-5'd1];


endmodule
//此模块为错误检查单元，将BIU里面所有模块发生的错误汇总生成mcause可用的错误信息

module exce_chk(
//biu当前状态输入
input [6:0]statu_biu,
//cpu状态输入
input [3:0]statu_cpu,
//操作码输入
input [2:0]opc,
//ahb准备好输入
input rdy_ahb,

//错误输入
//pmp检查出错
input pmp_chk_fault,
//ahb总线出错
input ahb_acc_fault,
input addr_mis,
input page_not_value,
//mmu单元与页表不符合造成的错误
input mmu_ld_page_fault,
input mmu_st_page_fault,

//对外报告异常类型
//注，对外报告错误有可能需要延迟一个周期的持续时间
output wire ins_addr_mis,
output wire ins_acc_fault,
output wire load_addr_mis,
output wire load_acc_fault,
output wire st_addr_mis,
output wire st_acc_fault,
output wire ins_page_fault,
output wire ld_page_fault,
output wire st_page_fault

);

//没有什么用的状态参数
parameter stb		=7'b0000000;
parameter rdy		=7'b0000001;
parameter err		=7'b0000010;
parameter ifnp		=7'b0001000;
parameter ifwp0	=7'b0010000;
parameter ifwp1	=7'b0010001;
parameter ifwp2	=7'b0010010;
parameter ifwp3	=7'b0010011;
parameter ifwp4	=7'b0010100;
parameter r32np	=7'b0011000;
parameter r32wp0	=7'b0100000;
parameter r32wp1	=7'b0100001;
parameter r32wp2	=7'b0100010;
parameter r32wp3	=7'b0100011;
parameter r32wp4	=7'b0100100;
parameter r16np	=7'b0101000;
parameter r16wp0	=7'b0110000;
parameter r16wp1	=7'b0110001;
parameter r16wp2	=7'b0110010;
parameter r16wp3	=7'b0110011;
parameter r16wp4	=7'b0110100;
parameter r8np		=7'b0111000;
parameter r8wp0	=7'b1000000;
parameter r8wp1	=7'b1000001;
parameter r8wp2	=7'b1000010;
parameter r8wp3	=7'b1000011;
parameter r8wp4	=7'b1000100;
parameter w32np	=7'b1001000;
parameter w32wp0	=7'b1010000;
parameter w32wp1	=7'b1010001;
parameter w32wp2	=7'b1010010;
parameter w32wp3	=7'b1010011;
parameter w32wp4	=7'b1010100;
parameter w16np	=7'b1011000;
parameter w16wp0	=7'b1100000;
parameter w16wp1	=7'b1100001;
parameter w16wp2	=7'b1100010;
parameter w16wp3	=7'b1100011;
parameter w16wp4	=7'b1100100;
parameter w8np		=7'b1101000;
parameter w8wp0	=7'b1110000;
parameter w8wp1	=7'b1110001;
parameter w8wp2	=7'b1110010;
parameter w8wp3	=7'b1110011;
parameter w8wp4	=7'b1110100;

assign ins_addr_mis = (statu_cpu==4'b0000)&(addr_mis);
assign ins_acc_fault= (statu_cpu==4'b0000)&((ahb_acc_fault)|(pmp_chk_fault));
assign load_addr_mis= (statu_cpu[2:0]==3'b010)&(opc[2])&(addr_mis);

assign load_acc_fault= (statu_cpu[2:0]==3'b010)&((statu_biu==r32np)|(statu_biu==r32wp4)|(statu_biu==r16np)|
							  (statu_biu==r16wp4)|(statu_biu==r8np)|(statu_biu==r8wp4))&(ahb_acc_fault|pmp_chk_fault);
assign st_addr_mis  = (statu_cpu[2:0]==3'b010)&(!opc[2])&(addr_mis);
assign st_acc_fault = (statu_cpu[2:0]==3'b010)&((statu_biu==w32np)|(statu_biu==w32wp4)|(statu_biu==w16np)|
							  (statu_biu==w16wp4)|(statu_biu==w8np)|(statu_biu==w8wp4))&(ahb_acc_fault|pmp_chk_fault);
assign ins_page_fault=rdy_ahb&((statu_biu==ifwp1)|(statu_biu==ifwp3))&(mmu_ld_page_fault)|(((statu_biu==ifwp1)|(statu_biu==ifwp3))&page_not_value);
assign ld_page_fault =rdy_ahb&((statu_biu==r32wp1)|(statu_biu==r32wp3)|(statu_biu==r16wp1)|(statu_biu==r16wp3)|
							 (statu_biu==r8wp1)|(statu_biu==r8wp3))&((mmu_ld_page_fault))|(((statu_biu==r32wp1)|
							 (statu_biu==r32wp3)|(statu_biu==r16wp1)|(statu_biu==r16wp3)|(statu_biu==r8wp1)|(statu_biu==r8wp3))&page_not_value);
assign st_page_fault =rdy_ahb&((statu_biu==w32wp1)|(statu_biu==w32wp3)|(statu_biu==w16wp1)|(statu_biu==w16wp3)|
							 (statu_biu==w8wp1)|(statu_biu==w8wp3))&((mmu_st_page_fault))|(((statu_biu==w32wp1)|
							 (statu_biu==w32wp3)|(statu_biu==w16wp1)|(statu_biu==w16wp3)|(statu_biu==w8wp1)|(statu_biu==w8wp3))&page_not_value);


endmodule
/*

适用于PRV332SV0处理器的EXU执行单元

2019 9.28重构

*/
module exu(
input wire clk,
input wire rst,

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
//exu需要用到的操作数
input wire [4:0]rs1_index,
input wire [31:0]rs1,
input wire [31:0]rs2,
input wire [31:0]csr,
input wire [19:0]imm20,
input wire [11:0]imm12,
input wire [4:0] shamt,
input wire [31:0]data_biu,
input wire [31:0]pc,

output wire [31:0]pc_next,	//下一个地址
output wire [31:0]addr_csr,	//addr,csr合用输出
output wire [31:0]data_tobiu,
output wire [31:0]data_rd,

output wire rdy_exu
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
/*

reg [4:0] shift_counter;	 //移位计数器

wire jmp;						//跳转信号

always@(posedge clk)begin
	if(rst)begin
		pc_next <= 32'b0;
		addr_csr<= 32'b0;
		data_tobiu<=32'b0;
		data_rd <= 32'b0;
		shift_counter <= 5'b0;
	end
	//当处理器在shift_counter为0且处理器在ex0阶段
	else if((statu_cpu==ex0)&(shift_counter==5'b0))begin
		pc_next <= (jal?{{11{imm20[19]}},imm20,1'b0}:jalr?{{19{imm12[11]}},imm12,1'b0}:pc) +  	//下一个pc值选取
					  (jal?pc:jalr?rs1:((beq | bne | blt | bltu | bge | bgeu )&jmp)?{{19{imm12[11]}},imm12,1'b0}:32'd4);
					  
		addr_csr<= ((opc_biu==r8)|(opc_biu==r16)|(opc_biu==r32)|(opc_biu==w8)|(opc_biu==w16)|(opc_biu==w32))?(rs1 + {{20{imm12[11]}},imm12}) :
					  (lr_w|sc_w|amoswap|amoadd|amoxor|amoand|amoor|amomin|amomax|amominu|amomaxu)?rs1:
                    //csr写回生成
                    csrrw ?  rs1 : csrrs ? (csr | rs1) : csrrc ? (csr | !rs1):csrrwi? {27'b0,rs1_index} :csrrsi? (csr | {27'b0,rs1_index}):
                    csrrci? (csr | !{27'b0,rs1_index}):32'b0;
		
		data_tobiu <= ((opc_biu==w8)|(opc_biu==w16)|(opc_biu==w32)|sc_w)?rs2:32'hffffffff;
		data_rd    <=  {32{addi}} & (rs1 + {{20{imm12[11]}},imm12}) |                                
              {32{slti}} & ((rs1[31]==1'b1 & imm12[11]==1'b0)?32'd1 : (rs1[31]==1'b0 & imm12[11]==1'b1) ? 32'd0 :(rs1 < {{20{imm12[11]}},imm12}) ? {31'b0,1'b1}:{32'b0}) |
              {32{sltiu}}& ((rs1 < {{20{imm12[11]}},imm12}) ? {31'b0,1'b1}:{32'b0}) |
              {32{andi}} & (rs1 & {{20{imm12[11]}},imm12}) |
              {32{ori}}  & (rs1 | {{20{imm12[11]}},imm12}) |
              {32{xori}} & (rs1 ^ {{20{imm12[11]}},imm12}) |
              {32{slli}} & (rs1) | {32{srli}} & (rs1) | {32{srai}} & (rs1) |
              {32{lui}}  & {imm20,12'b0}                     |
              {32{auipc}}& (pc + {imm20,12'b0})              |
              {32{addp}} & (rs1 + rs2)                       |
              {32{subp}} & (rs1 - rs2)                       |
              {32{sltp}} & ((rs1[31]==1'b1 & rs2[31]==1'b0)?32'd1 : (rs1[31]==1'b0 & rs2[31]==1'b1) ? 32'd0 :(rs1 < rs2) ? {31'b0,1'b1}:{32'b0}) |
              {32{sltup}}& ((rs1<rs2)?1'b1 : 1'b0)           |
              {32{andp}} & (rs1 & rs2)                       |
              {32{orp}}  & (rs1 | rs2)                       |
              {32{xorp}} & (rs1 ^ rs2)                       |
              {32{sllp}} & (rs1) | {32{srlp}}&(rs1) | {32{srap}}&(rs1)   |
              {32{jal}}  & (pc + 32'd4)                      |
              {32{jalr}} & (pc + 32'd4)                      |
              {{32{csrrw}}}& csr | csrrs & csr | csrrc & csr | csrrwi & csr | csrrsi & csr |csrrci & csr | 32'b0;
				  
		shift_counter <= (slli | srli | srai )?shamt:(sllp|srlp|srap)?rs2[4:0]:5'b0;
	end
		//当处理器的shift_counter不为0时
	else if((statu_cpu==ex0)&(shift_counter!=5'b0))begin
			if(slli | sllp)begin
            
				data_rd <= (data_rd<<1);
				shift_counter <= shift_counter - 5'd1;
					
        
			end
			else if(srli|srlp)begin
           
				data_rd <= (data_rd>>1);
				shift_counter <= shift_counter-5'd1;
					
			end
			else if(srai | srap)begin
            
				data_rd <= (data_rd>>1);                                 //此处有一个bug，我不知道有符号数怎么搞
				shift_counter<=shift_counter - 5'd1;
					
			end
	end
	
	else if((statu_cpu==ex1))begin
		pc_next <= pc_next;
		addr_csr<= addr_csr;
		data_tobiu<=amoswap?rs2:amoadd?(rs2+data_biu):amoand?(rs2&data_biu):amoor?(rs2|data_biu):
						amoxor?(rs2^data_biu):amomax?((rs1>data_biu)?rs1:data_biu):amomin?((rs1<data_biu)?rs1:data_biu):
						amomax?((rs1>data_biu)?rs1:data_biu):amomin?((rs1<data_biu)?rs1:data_biu):32'b0;
		data_rd <= data_rd;
		shift_counter <= shift_counter;
	end
end

assign rdy_exu = !((slli|srli|srai)&((shamt==5'b0)|(shift_counter==5'd1))) | !((sllp|srlp|srap)&((rs2[4:0]==5'b0)|(shift_counter==5'd1)));
assign jmp = beq & (rs1 == rs2) | bne & (rs1 !=rs2) | blt & (rs1[31]==1'b1&rs2[31]==1'b0 | (rs1[31]==rs2[31])&rs1 < rs2) | bltu & (rs1 < rs2) | bge & ((rs1[31]==1'b0&rs2[31]==1'b1 | (rs1[31]==rs2[31])&rs1 < rs2)) | bgeu & (rs1 > rs2) | jal | jalr;	
*/  // 2019 9,28重构
alu alu(
.clk(clk),
.rst(rst),
.statu_cpu(statu_cpu),
.opc_biu(opc_biu),

//exu控制信号
//译码器结果输出(exu)
.addi(addi) ,
.slti(slti),
.sltiu(sltiu),
.andi(andi),
.ori(ori),
.xori(xori),
.slli(slli),
.srli(srli),
.srai(srai),

.lui(lui),
.auipc(auipc),
.addp(addp),
.subp(subp),
.sltp(sltp),
.sltup(sltup),
.andp(andp),
.orp(orp),
.xorp(xorp),
.sllp(sllp),
.srlp(srlp),
.srap(srap),

.jal(jal),
.jalr(jalr),

.beq(beq),
.bne(bne),
.blt(blt),
.bltu(bltu),
.bge(bge),
.bgeu(bgeu),

.csrrw(csrrw),
.csrrs(csrrs),
.csrrc(csrrc),
.csrrwi(csrrwi),
.csrrsi(csrrsi),
.csrrci(csrrci),
//amo指令添加
.lr_w(lr_w),
.sc_w(sc_w),
.amoswap(amoswap),
.amoadd(amoadd),
.amoxor(amoxor),
.amoand(amoand),
.amoor(amoor),
.amomin(amomin),
.amomax(amomax),
.amominu(amominu),
.amomaxu(amomaxu),

//alu需要用到的操作数
.rs1_index(rs1_index),
.rs1(rs1),
.rs2(rs2),

.imm20(imm20),
.imm12(imm12),
.csr(csr),
.shamt(shamt),
.data_biu(data_biu),
.pc(pc),

//alu输出
.data_rd(data_rd),
.data_tobiu(data_tobiu),
.pc_jmp(pc_jmp),
.rdy_alu(rdy_exu)			//ALU准备完毕信号
);

au au(
.clk(clk),
.rst(rst),
.statu_cpu(statu_cpu),
.opc_biu(opc_biu),

.rdy_alu(rdy_exu),

.jalr(jalr),
.jal(jal),

.beq(beq),
.bne(bne),
.blt(blt),
.bltu(bltu),
.bge(bge),
.bgeu(bgeu),

.csrrw(csrrw),
.csrrs(csrrs),
.csrrc(csrrc),
.csrrwi(csrrwi),
.csrrsi(csrrsi),
.csrrci(csrrci),

.lr_w(lr_w),
.sc_w(sc_w),
.amoswap(amoswap),
.amoadd(amoadd),
.amoxor(amoxor),
.amoand(amoand),
.amoor(amoor),
.amomin(amomin),
.amomax(amomax),
.amominu(amominu),
.amomaxu(amomaxu),

.pc_jmp(pc_jmp),

//au需要用到的操作数
.rs1_index(rs1_index),
.rs1(rs1),

.imm20(imm20),
.imm12(imm12),
.csr(csr),
.pc(pc),

.addr_csr(addr_csr),
.pc_next(pc_next)

);

endmodule
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
/*
适用于PRV332SV0的中断异常控制单元
*/
module int_ctrl(
input clk,
input rst,
//处理器状态输入
input wire [3:0]statu_cpu,
//处理器权限输入
input wire [1:0]msu,
//pc值输入
input wire [31:0]pc,
//处理器外部的中断线输入
input wire timer_int_in,
input wire soft_int_in,
input wire ext_int_in,

//中断异常委托输入
input wire [31:0]mideleg,
input wire [31:0]medeleg,

//csr要写的数据输入
input wire [31:0]csr_in,
input wire [11:0]csr_index,
input wire csr_wr,

//中断使能位输入
input wire sie,
input wire mie,
input wire mtie,
input wire msie,
input wire meie,
input wire stie,
input wire ssie,
input wire seie,
//中断等待位输入
input wire stip_in,
input wire ssip_in,
input wire seip_in,

//译码器异常报告
input wire ecall,
input wire ebreak,
input wire ill_ins,
input wire [31:0]ins,

//处理器中断接收信号
output wire int_acc,
//biu异常报告
input wire ins_addr_mis,
input wire ins_acc_fault,
input wire load_addr_mis,
input wire load_acc_fault,
input wire st_addr_mis,
input wire st_acc_fault,
input wire ins_page_fault,
input wire ld_page_fault,
input wire st_page_fault,

//biu当前访问的va地址
input wire [31:0]addr_biu,

//送csr的信号
output wire mtip,
output wire mtip_wr,
output wire meip,
output wire meip_wr,
output wire msip,
output wire msip_wr,
output wire stip,
output wire stip_wr,
output wire seip,
output wire seip_wr,
output wire ssip,
output wire ssip_wr,

output wire [1:0]priv_d, //中断目标权限
output wire [31:0]cause,  //造成异常的原因
output wire [31:0]tval	  //中断值

);
//处理器状态编码
parameter if0 = 4'b0000;
parameter ex0 = 4'b0001;
parameter mem0= 4'b0010;
parameter mem1=4'b1010;
parameter ex1 = 4'b1001;
parameter wb = 4'b0011;
parameter exc = 4'b1111;
//处理器权限编码
parameter m = 2'b11;
parameter h = 2'b10; 
parameter s = 2'b01;
parameter u = 2'b00;
//mcause编码
parameter usint =32'h8000000; 
parameter ssint =32'h8000001;
parameter msint =32'h8000003; 
parameter utint =32'h8000004; 
parameter stint =32'h8000005; 
parameter mtint =32'h8000007; 
parameter ueint =32'h8000008;
parameter seint =32'h8000009; 
parameter meint =32'h800000b; 
parameter iam	 =32'h0000000;
parameter iaf   =32'h0000001;
parameter ii    =32'h0000002;
parameter bk    =32'h0000003;
parameter lam	 =32'h0000004;
parameter laf	 =32'h0000005;
parameter sam	 =32'h0000006;
parameter saf	 =32'h0000007;
parameter ecu	 =32'h0000008;
parameter ecs	 =32'h0000009;
parameter ecm	 =32'h000000b;
parameter ipf	 =32'h000000c;
parameter lpf	 =32'h000000d;
parameter spf	 =32'h000000f;
//csr索引编码
parameter mstatus_index  = 12'h300;
parameter medeleg_index  = 12'h302;
parameter mideleg_index  = 12'h303;
parameter mie_index      = 12'h304;
parameter mtvec_index    = 12'h305;
parameter mscratch_index = 12'h340;
parameter mepc_index     = 12'h341;
parameter mcause_index   = 12'h342;
parameter mtval_index    = 12'h343;
parameter mip_index 		 = 12'h344;
parameter pmpcfg0_index  = 12'h3a0;
parameter pmpcfg1_index  = 12'h3a1;
parameter pmpcfg2_index  = 12'h3a2;
parameter pmpcfg3_index  = 12'h3a3;
parameter pmpaddr0_index = 12'h3b0;
parameter pmpaddr1_index = 12'h3b1;
parameter pmpaddr2_index = 12'h3b2;
parameter pmpaddr3_index = 12'h3b3;
parameter sstatus_index  = 12'h100;
parameter sie_index 		 = 12'h104;
parameter stvec_index	 = 12'h105;
parameter sscratch_index = 12'h140;
parameter sepc_index  	 = 12'h141;
parameter scause_index	 = 12'h142;
parameter stval_index 	 = 12'h143;
parameter sip_index		 = 12'h144;
parameter satp_index		 = 12'h180;


reg timer_int;
reg soft_int;
reg ext_int;
//中断控制器传给编码器的信号
wire mti;
wire msi;
wire mei;
wire sti;
wire ssi;
wire sei;

//检测到M模式下要读写sip寄存器的信号
wire stip_wr_en;
wire ssip_wr_en;
wire seip_wr_en;



assign mti = ((msu==m)&timer_int&mtie&mie)|((msu==s)&timer_int&(!mideleg[5]))|((msu==u)&timer_int&(!mideleg[4]));
assign sti = ((msu==s)&sie&stie&(timer_int|stip)&mideleg[5]) | ((msu==u)&timer_int&mideleg[4]);

assign msi = ((msu==m)&soft_int&msie&mie)|((msu==s)&soft_int&(!mideleg[1]))|((msu==u)&soft_int&(!mideleg[0]));
assign ssi = ((msu==s)&sie&ssie&(soft_int|ssip)&mideleg[1]) | ((msu==u)&soft_int&mideleg[0]);

assign mei = ((msu==m)&ext_int&meie&mie)|((msu==s)&ext_int&(!mideleg[9]))|((msu==u)&ext_int&(!mideleg[8]));
assign sei = ((msu==s)&sie&seie&(ext_int|seip)&mideleg[9]) | ((msu==u)&ext_int&mideleg[8]);

//中断目的权限编码器，为优先编码器
assign priv_d = (((msu==m)&(ins_acc_fault|ins_addr_mis|ill_ins|load_acc_fault|load_addr_mis|st_addr_mis|st_acc_fault//m模式下发生的任何异常都交给m模式处理
									|ins_page_fault|ld_page_fault|st_page_fault|ecall|ebreak))|
					 ((msu==s)&((ins_acc_fault&!medeleg[1])|(ins_addr_mis&!medeleg[0])|(ill_ins&!medeleg[2])|       //s模式下发生一些不被委托的异常交给m处理
									(load_acc_fault&!medeleg[5])|(load_addr_mis&!medeleg[4])|(st_addr_mis&!medeleg[6])|
									(st_acc_fault&!medeleg[7])|(ins_page_fault&!medeleg[12])|(ld_page_fault&!medeleg[13])|
									(st_page_fault&!medeleg[15])|(ecall&!medeleg[9])|(ebreak&!medeleg[3]))) |
					 ((msu==u)&((ins_acc_fault&!medeleg[1])|(ins_addr_mis&!medeleg[0])|(ill_ins&!medeleg[2])|       //u模式下发生一些不被委托的异常交给m处理
									(load_acc_fault&!medeleg[5])|(load_addr_mis&!medeleg[4])|(st_addr_mis&!medeleg[6])|
									(st_acc_fault&!medeleg[7])|(ins_page_fault&!medeleg[12])|(ld_page_fault&!medeleg[13])|
									(st_page_fault&!medeleg[15])|(ecall&!medeleg[8])|(ebreak&!medeleg[3]))))? m : 
					  
					 (((msu==s)&((ins_acc_fault&medeleg[1])|(ins_addr_mis&medeleg[0])|(ill_ins&medeleg[2])|       //s模式下发生一些委托的异常交给s处理
									(load_acc_fault&medeleg[5])|(load_addr_mis&medeleg[4])|(st_addr_mis&medeleg[6])|
									(st_acc_fault&medeleg[7])|(ins_page_fault&medeleg[12])|(ld_page_fault&medeleg[13])|
									(st_page_fault&medeleg[15])|(ecall&medeleg[9])|(ebreak&medeleg[3]))) |
					 ((msu==u)&((ins_acc_fault&medeleg[1])|(ins_addr_mis&medeleg[0])|(ill_ins&medeleg[2])|       //u模式下发生一些委托的异常交给s处理
									(load_acc_fault&medeleg[5])|(load_addr_mis&medeleg[4])|(st_addr_mis&medeleg[6])|
									(st_acc_fault&medeleg[7])|(ins_page_fault&medeleg[12])|(ld_page_fault&medeleg[13])|
									(st_page_fault&medeleg[15])|(ecall&medeleg[8])|(ebreak&medeleg[3]))))? s :
									
					  (mti|msi|mei)?m : (sti|ssi|sei)? s : h;
					  
//cause状态机编码，为优先级编码器					  
assign cause =ins_addr_mis ? iam : ins_acc_fault ? iaf : ill_ins ? ii : ebreak ? bk : load_addr_mis ? lam : load_acc_fault ? laf :
				  st_addr_mis ? sam : st_acc_fault ? saf : (ecall&(msu==m))? ecm : (ecall&(msu==s))? ecs : (ecall&(msu==u))? ecu : 
				  ins_page_fault ? ipf : ld_page_fault ? lpf : st_page_fault ? spf : mti ? mtint : msi ? msint : mei ? meint : sti ?
				  stint : ssi ? ssint : sei ? seint : 32'hffffffff;
//tval值
assign tval = (ins_addr_mis | ins_acc_fault | ins_page_fault)? pc : (load_addr_mis|load_acc_fault|ld_page_fault|st_addr_mis|st_acc_fault|st_page_fault)? addr_biu : ill_ins ? ins : 32'b0;
//送csr的部分信号

assign mtip 	= mti;
assign mtip_wr = (statu_cpu==wb);
assign meip	   = mei;
assign meip_wr	= (statu_cpu==wb);
assign msip		= msi;
assign msip_wr	= (statu_cpu==wb);

assign stip_wr_en = (msu==m)&(csr_index==sip_index)&csr_wr;
assign ssip_wr_en = (msu==m)&(csr_index==sip_index)&csr_wr;
assign seip_wr_en = (msu==m)&(csr_index==sip_index)&csr_wr;

assign stip		=  !((!stip_in&!sti&!stip_wr_en&!csr_in[5]) | (!stip_in&!sti&!stip_wr_en&csr_in[5]) |
						(!stip_in&!sti&stip_wr_en&!csr_in[5]) | (stip_in&!sti&stip_wr_en&!csr_in[5]));

assign stip_wr	= (statu_cpu==wb);
assign seip		=  !((!seip_in&!sei&!seip_wr_en&!csr_in[9]) | (!seip_in&!sei&!seip_wr_en&csr_in[9]) |
						(!seip_in&!sei&seip_wr_en&!csr_in[9]) | (seip_in&!sei&seip_wr_en&!csr_in[9]));


assign seip_wr = (statu_cpu==wb);
assign ssip		= !((!ssip_in&!ssi&!ssip_wr_en&!csr_in[1]) | (!ssip_in&!ssi&!ssip_wr_en&csr_in[1]) |
						(!ssip_in&!ssi&ssip_wr_en&!csr_in[1]) | (ssip_in&!ssi&ssip_wr_en&!csr_in[1]));

assign ssip_wr	= (statu_cpu==wb);
									
assign int_acc = mti | sti | msi | ssi | mei | sei;									

//当处理器即将进入wb阶段的时候对外部中断进行寄存
always@(posedge clk)begin
	if(rst)begin
		timer_int <= 1'b0;
		soft_int  <= 1'b0;
		ext_int   <= 1'b0;
	end
	else if((statu_cpu==mem1)|(statu_cpu==mem0))begin
		timer_int<=timer_int_in;
		soft_int <= soft_int_in;
		ext_int  <= ext_int_in;
	end
	else begin
		timer_int <= timer_int;
		soft_int  <= soft_int;
		ext_int   <= ext_int;
	end
end

		
endmodule

module mmu(
input clk,
input rst,
//biu当前状态输入
input wire [6:0]statu_biu,
//ahb总线过多路复用器输出
input wire [31:0]data_in,
//地址输入
input wire [31:0]addr,
//satp寄存器输入.

input wire [31:0]satp,
//转换后的地址输出
output wire [33:0]addr_mmu,
//更改后的PTE输出
output wire [31:0]pte_new,
//mstatus寄存器关键位输入
//禁用执行位
input wire mxr,
input wire sum,

//当前机器权限模式输入
input wire [1:0]msu,
//异常输出(由于页表不匹配导致的）
output wire ld_page_fault,
output wire st_page_fault,
output wire page_not_value



);
//没有什么用的状态参数
parameter stb		=7'b0000000;
parameter rdy		=7'b0000001;
parameter err		=7'b0000010;
parameter ifnp		=7'b0001000;
parameter ifwp0	=7'b0010000;
parameter ifwp1	=7'b0010001;
parameter ifwp2	=7'b0010010;
parameter ifwp3	=7'b0010011;
parameter ifwp4	=7'b0010100;
parameter r32np	=7'b0011000;
parameter r32wp0	=7'b0100000;
parameter r32wp1	=7'b0100001;
parameter r32wp2	=7'b0100010;
parameter r32wp3	=7'b0100011;
parameter r32wp4	=7'b0100100;
parameter r16np	=7'b0101000;
parameter r16wp0	=7'b0110000;
parameter r16wp1	=7'b0110001;
parameter r16wp2	=7'b0110010;
parameter r16wp3	=7'b0110011;
parameter r16wp4	=7'b0110100;
parameter r8np		=7'b0111000;
parameter r8wp0	=7'b1000000;
parameter r8wp1	=7'b1000001;
parameter r8wp2	=7'b1000010;
parameter r8wp3	=7'b1000011;
parameter r8wp4	=7'b1000100;
parameter w32np	=7'b1001000;
parameter w32wp0	=7'b1010000;
parameter w32wp1	=7'b1010001;
parameter w32wp2	=7'b1010010;
parameter w32wp3	=7'b1010011;
parameter w32wp4	=7'b1010100;
parameter w16np	=7'b1011000;
parameter w16wp0	=7'b1100000;
parameter w16wp1	=7'b1100001;
parameter w16wp2	=7'b1100010;
parameter w16wp3	=7'b1100011;
parameter w16wp4	=7'b1100100;
parameter w8np		=7'b1101000;
parameter w8wp0	=7'b1110000;
parameter w8wp1	=7'b1110001;
parameter w8wp2	=7'b1110010;
parameter w8wp3	=7'b1110011;
parameter w8wp4	=7'b1110100;

wire [33:0]ag0;
wire [33:0]ag1;
wire [33:0]ag2;
reg  [33:0]reg_ag1;
reg  [33:0]reg_ag2;

assign ag0 = {satp[21:0],12'b0}  + {22'b0,addr[31:22],2'b0};
assign ag1 = {data_in[31:10],12'b0} + {22'b0,addr[21:12],2'b0};
assign ag2 = {data_in[31:10],12'b0} + {22'b0,addr[11:0]};

always@(posedge clk)begin
	reg_ag1 <= (rst|(statu_biu[2:0]==3'b000))?32'b0:(statu_biu[2:0]==3'b001)?ag1 : reg_ag1;
	reg_ag2 <= (rst|(statu_biu[2:0]==3'b000))?32'b0:(statu_biu[2:0]==3'b001)?ag2 : reg_ag2;


end
	
assign ld_page_fault = /*(((statu_biu[2:0]==3'b001)&(statu_biu[6:3]!=4'b0000))&data_in[0]==1'b0) | 
							  (((statu_biu[2:0]==3'b011)&(statu_biu[6:0]!=4'b0000))&data_in[0]==1'b0) |
							  (((statu_biu[2:0]==3'b011)&(statu_biu[6:0]!=4'b0000))&(data_in[1]==1'b0)&(((statu_biu[6:3]==4'b0010)&(mxr==1'b0))|(statu_biu[6:3]==4'b0100)|(statu_biu[6:3]==4'b0110)|(statu_biu[6:3]==4'b1000))) |
							  (((statu_biu[2:0]==3'b011)&(statu_biu[6:0]!=4'b0000))&(data_in[3]==1'b0)&(statu_biu[6:3]==4'b0010))|
							  (((statu_biu[2:0]==3'b011)&(statu_biu[6:0]!=4'b0000))&(data_in[4]==1'b1)&(msu!=2'b00))|				 //U模式访问非用户界面造成错误
							  (((statu_biu[2:0]==3'b011)&(statu_biu[6:0]!=4'b0000))&(data_in[4]==1'b1)&(msu==2'b01)&(sum==1'b0))| //sum造成错误
							  (((statu_biu[2:0]==3'b011)&(statu_biu[6:0]!=4'b0000))&(data_in[4]==1'b1)&(data_in[7:6]==2'b00)) ;  //AD为0造成错误
*/
							  (((statu_biu==ifwp1)|(statu_biu==ifwp3)|(statu_biu==r32wp1)|(statu_biu==r16wp1)|(statu_biu==r8wp1)|(statu_biu==r32wp3)|(statu_biu==r16wp3)|(statu_biu==r8wp3))&(data_in[0]==1'b0))|
							  (((statu_biu==r16wp3)|(statu_biu==r8wp3))&(data_in[0]==1'b0))|
							  (((statu_biu==r32wp1)|(statu_biu==r32wp3)|(statu_biu==r16wp1)|(statu_biu==r16wp3)|(statu_biu==r8wp1)|(statu_biu==r8wp3))&((mxr==1'b0)&(data_in[1]==1'b1)))|
							  (((statu_biu==ifwp1)|(statu_biu==ifwp3))&(data_in[3]==1'b0))|
							  (((statu_biu==ifwp1)|(statu_biu==ifwp3)|(statu_biu==r32wp1)|(statu_biu==r32wp3)|(statu_biu==r16wp1)|(statu_biu==r16wp3)|(statu_biu==r8wp1)|(statu_biu==r8wp3))&(msu!=2'b01)&(data_in[4]==1'b1))|	//U模式访问非用户界面造成错误
							  (((statu_biu==ifwp1)|(statu_biu==ifwp3)|(statu_biu==r32wp1)|(statu_biu==r32wp3)|(statu_biu==r16wp1)|(statu_biu==r16wp3)|(statu_biu==r8wp1)|(statu_biu==r8wp3))&(sum==1'b0)&(msu==2'b01)&(data_in[4]==1'b1))| //sum导致错误
							  (((statu_biu==ifwp1)|(statu_biu==ifwp3)|(statu_biu==r32wp1)|(statu_biu==r32wp3)|(statu_biu==r16wp1)|(statu_biu==r16wp3)|(statu_biu==r8wp1)|(statu_biu==r8wp3))&(data_in[7:6]==2'b00));   //AD为0发生错误
							  
							  
							  
assign st_page_fault = (((statu_biu[2:0]==3'b001)&(statu_biu[6:3]!=4'b0000))&data_in[0]==1'b0) | 
							  (((statu_biu[2:0]==3'b011)&(statu_biu[6:0]!=4'b0000))&data_in[0]==1'b0) |
							  (((statu_biu[2:0]==3'b011)&(statu_biu[6:0]!=4'b0000))&(data_in[2]==1'b0)&((statu_biu[6:3]==4'b1010)|(statu_biu[6:3]==4'b1100)|(statu_biu[6:3]==4'b0110)|(statu_biu[6:3]==4'b1110))) | //要写不是W的页面
							  (((statu_biu[2:0]==3'b011)&(statu_biu[6:0]!=4'b0000))&(data_in[4]==1'b1)&(data_in[7:6]==2'b00));	 //AD为0造成错误

assign page_not_value =(((statu_biu==ifwp1)|(statu_biu==ifwp3)|(statu_biu==r32wp1)|(statu_biu==r16wp1)|(statu_biu==r8wp1)|(statu_biu==r32wp3)|(statu_biu==r16wp3)|(statu_biu==r8wp3)	//页面不存在信号，阻止PTE回写
								 |(statu_biu==w32wp1)|(statu_biu==w32wp3)|(statu_biu==w16wp1)|(statu_biu==w16wp3)|(statu_biu==w8wp1)|(statu_biu==w8wp3))&(data_in[0]==1'b0)); 							  

assign addr_mmu =  ((statu_biu[2:1]==2'b00)?ag0 : 34'b0) |	//addr输出数据选择器
						 ((statu_biu[2:1]==2'b01)?reg_ag1 : 34'b0) |
						 ((statu_biu[2:1]==2'b10)?reg_ag2 : 34'b0) ;
//生成新的PTE，当访问，读/写的时候对A制1，当写的时候置D为1
assign pte_new  =  (((statu_biu[6:3]==4'b0001)|(statu_biu[6:3]==4'b0010)|(statu_biu[6:3]==4'b0100)|(statu_biu[6:3]==4'b0110)|(statu_biu[6:3]==4'b1000)|(statu_biu[6:3]==4'b1010)|(statu_biu[6:3]==4'b1100)|(statu_biu[6:3]==4'b1110))?{data_in[31:8],2'b11,data_in[5:0]}:32'b0) | 
					    (((statu_biu[6:3]==4'b1010)|(statu_biu[6:3]==4'b1100)|(statu_biu[6:3]==4'b1110))?{data_in[31:8],2'b11,data_in[5:0]}:32'b0); 
		   


endmodule
/*
prv332sv0 cpu
*support rv32ia,mmu,privilege msu
*fully compatible with the code of prv32f0(px_rv32) 
prv_processor  family 1  stepping 2A

*/
module prv332sv0(
input wire clk,
input wire rst,
//中断信号
input wire timer_int,
input wire soft_int,
input wire ext_int,
//对ahb的信号
output wire [33:0]haddr,
output wire hwrite,
output wire [1:0]hsize,
output wire [2:0]hburst,
output wire [3:0]hprot,
output wire [1:0]htrans,
output wire hmastlock,
output wire [31:0]hwdata,

input wire hready,
input wire hresp,
input wire hreset_n,
input wire [31:0]hrdata

);
//opc_biu
parameter w8 = 3'b001;
parameter w16= 3'b010;
parameter w32= 3'b011;
parameter r8 = 3'b101;
parameter r16= 3'b110;
parameter r32= 3'b111;

//机器状态输入
wire[3:0]statu_cpu;
//pmp单元使用的信号
wire [33:0]addr_out;
//pmp检查错误信号
wire pmp_chk_fault;
//satp寄存器输入
wire [31:0]satp;

//pc输入
wire [31:0]pc;
//数据输入

//数据输出
wire [31:0]biu_data_out;
//当前机器状态输入
wire [1:0]msu;
wire [31:0]ins;

//biu准备好信号
wire rdy_biu;

//mxr,sum输入
wire mxr;
wire sum;

//biu异常报告

wire ins_addr_mis;
wire ins_acc_fault;
wire load_addr_mis;
wire load_acc_fault;
wire st_addr_mis;
wire st_acc_fault;
wire ins_page_fault;
wire ld_page_fault;
wire st_page_fault;

//模块准备好信号
//exu准备好信号
wire rdy_exu;

//exu控制信号
//译码器结果输出(exu)
wire addi;
wire slti;
wire sltiu;
wire andi;
wire ori;
wire xori;
wire slli;
wire srli;
wire srai;

wire lui;
wire auipc;
wire addp;
wire subp;
wire sltp;
wire sltup;
wire andp;
wire orp;
wire xorp;
wire sllp;
wire srlp;
wire srap;

wire jal;
wire jalr;

wire beq;
wire bne;
wire blt;
wire bltu;
wire bge;
wire bgeu;
//load指令对数据进行符号位拓展信号
wire lb;
wire lh;

wire csrrw;
wire csrrs;
wire csrrc;
wire csrrwi;
wire csrrsi;
wire csrrci;
//amo指令添加
wire lr_w;
wire sc_w;
wire amoswap;
wire amoadd;
wire amoxor;
wire amoand;
wire amoor;
wire amomin;
wire amomax;
wire amominu;
wire amomaxu;
//exu需要用到的操作数
wire [4:0]rs1_index;
wire [31:0]rs1;
wire [31:0]rs2;
wire [31:0]csr;
wire [19:0]imm20;
wire [11:0]imm12;
wire [4:0] shamt;

//biu操作信号
wire [2:0]opc_biu;

wire [31:0]pc_next;	//下一个地址
wire [31:0]addr_csr;	//addr,csr合用输出
wire [31:0]data_tobiu;
wire [31:0]data_rd;


wire [31:0]data_togpr;

assign data_togpr = ((opc_biu==r16)&lh)?{{16{biu_data_out[15]}},biu_data_out[15:0]}:
	((opc_biu==r16)&!lh)?{16'b0,biu_data_out[15:0]}:((opc_biu==r8)&lb)?{{24{biu_data_out[7]}},biu_data_out[7:0]}:
	((opc_biu==r16)&!lb)?{24'b0,biu_data_out[7:0]}:(opc_biu==r32)?biu_data_out : data_rd;

 (* DONT_TOUCH = "true" *)biu biu(

.clk(clk),
.rst(rst),
//对ahb的信号
.haddr(haddr),
.hwrite(hwrite),
.hsize(hsize),
.hburst(hburst),
.hprot(hprot),
.htrans(htrans),
.hmastlock(hmastlock),
.hwdata(hwdata),
.hready(hready),
.hresp(hresp),
.hreset_n(hreset_n),
.hrdata(hrdata),

//操作码
.opc(opc_biu),
//机器状态输入
.statu_cpu(statu_cpu),
//pmp单元使用的信号
.addr_out(addr_out),
//pmp检查错误信号
.pmp_chk_fault(pmp_chk_fault),
//satp寄存器输入
.satp(satp),
//地址输入
.addr(addr_csr),
//pc输入
.pc(pc),
//数据输入
.biu_data_in(data_tobiu),
//数据输出
.biu_data_out(biu_data_out),
//当前机器状态输入
.msu(msu),
.ins(ins),
//biu准备好信号
.rdy_biu(rdy_biu),
//mxr,sum输入
.mxr(mxr),
.sum(sum),
//异常报告
.ins_addr_mis(ins_addr_mis),
.ins_acc_fault(ins_acc_fault),
.load_addr_mis(load_addr_mis),
.load_acc_fault(load_acc_fault),
.st_addr_mis(st_addr_mis),
.st_acc_fault(st_acc_fault),
.ins_page_fault(ins_page_fault),
.ld_page_fault(ld_page_fault),
.st_page_fault(st_page_fault)
);


/*
适用于PRV332SV0处理器的csr_gpr_iu单元
*/
 (* DONT_TOUCH = "true" *)csr_gpr_iu csr_gpr_iu(
.clk(clk),
.rst(rst),
//中断输入
.timer_int(timer_int),
.soft_int(soft_int),
.ext_int(ext_int),

//模块准备好信号
//exu准备好信号
.rdy_exu(rdy_exu),
//biu准备好信号
.rdy_biu(rdy_biu),
//satp信号
.satp(satp),
//pmp检查信号
.pmp_addr(addr_out),
.pmp_chk_fault(pmp_chk_fault),
//biu异常报告
.ins_addr_mis_in(ins_addr_mis),
.ins_acc_fault_in(ins_acc_fault),
.load_addr_mis_in(load_addr_mis),
.load_acc_fault_in(load_acc_fault),
.st_addr_mis_in(st_addr_mis),
.st_acc_fault_in(st_acc_fault),
.ins_page_fault_in(ins_page_fault),
.ld_page_fault_in(ld_page_fault),
.st_page_fault_in(st_page_fault),

.ins(ins),
.addr_biu(addr_csr),
//机器状态输出
.statu_cpu(statu_cpu),
.msu(msu),
.pc(pc),
//exu控制信号
//译码器结果输出(exu)
.addi(addi) ,
.slti(slti),
.sltiu(sltiu),
.andi(andi),
.ori(ori),
.xori(xori),
.slli(slli),
.srli(srli),
.srai(srai),

.lui(lui),
.auipc(auipc),
.addp(addp),
.subp(subp),
.sltp(sltp),
.sltup(sltup),
.andp(andp),
.orp(orp),
.xorp(xorp),
.sllp(sllp),
.srlp(srlp),
.srap(srap),

.jal(jal),
.jalr(jalr),

.beq(beq),
.bne(bne),
.blt(blt),
.bltu(bltu),
.bge(bge),
.bgeu(bgeu),
//load指令对数据进行符号位拓展信号
.lb(lb),
.lh(lh),

.csrrw(csrrw),
.csrrs(csrrs),
.csrrc(csrrc),
.csrrwi(csrrwi),
.csrrsi(csrrsi),
.csrrci(csrrci),
//amo指令添加
.lr_w(lr_w),
.sc_w(sc_w),
.amoswap(amoswap),
.amoadd(amoadd),
.amoxor(amoxor),
.amoand(amoand),
.amoor(amoor),
.amomin(amomin),
.amomax(amomax),
.amominu(amominu),
.amomaxu(amomaxu),

.rs1_index(rs1_index),

.rs1(rs1),
.rs2(rs2),
.csr_out(csr),
.imm20(imm20),
.imm12(imm12),
.shamt(shamt),
//exu反馈的数
.csr_in(addr_csr),
.data_gpr(data_togpr),
.pc_next(pc_next),

.mxr(mxr),
.sum(sum),

//biu操作信号
.opc_biu(opc_biu)

);


exu exu(
.clk(clk),
.rst(rst),

.statu_cpu(statu_cpu),
.opc_biu(opc_biu),
//exu控制信号
//译码器结果输出(exu)
.addi(addi) ,
.slti(slti),
.sltiu(sltiu),
.andi(andi),
.ori(ori),
.xori(xori),
.slli(slli),
.srli(srli),
.srai(srai),

.lui(lui),
.auipc(auipc),
.addp(addp),
.subp(subp),
.sltp(sltp),
.sltup(sltup),
.andp(andp),
.orp(orp),
.xorp(xorp),
.sllp(sllp),
.srlp(srlp),
.srap(srap),

.jal(jal),
.jalr(jalr),

.beq(beq),
.bne(bne),
.blt(blt),
.bltu(bltu),
.bge(bge),
.bgeu(bgeu),

.csrrw(csrrw),
.csrrs(csrrs),
.csrrc(csrrc),
.csrrwi(csrrwi),
.csrrsi(csrrsi),
.csrrci(csrrci),
//amo指令添加
.lr_w(lr_w),
.sc_w(sc_w),
.amoswap(amoswap),
.amoadd(amoadd),
.amoxor(amoxor),
.amoand(amoand),
.amoor(amoor),
.amomin(amomin),
.amomax(amomax),
.amominu(amominu),
.amomaxu(amomaxu),

.rs1_index(rs1_index),
.rs1(rs1),
.rs2(rs2),

.imm20(imm20),
.imm12(imm12),
.csr(csr),
.shamt(shamt),
.data_biu(biu_data_out),
.pc(pc),

.pc_next(pc_next),	//下一个地址
.addr_csr(addr_csr),	//addr,csr合用输出
.data_tobiu(data_tobiu),
.data_rd(data_rd),

. rdy_exu(rdy_exu)

);


endmodule
