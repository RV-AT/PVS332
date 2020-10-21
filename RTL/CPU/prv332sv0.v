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
