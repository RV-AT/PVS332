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
