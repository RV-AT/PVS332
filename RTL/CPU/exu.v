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
