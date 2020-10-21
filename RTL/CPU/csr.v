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

