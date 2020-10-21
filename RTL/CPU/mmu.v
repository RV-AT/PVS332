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
