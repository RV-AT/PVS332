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
