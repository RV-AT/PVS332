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
	
	
	
	
	
	
	
	
	
	
	
	
