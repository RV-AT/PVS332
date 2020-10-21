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
