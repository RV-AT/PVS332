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
