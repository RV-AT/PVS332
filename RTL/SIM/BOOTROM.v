`timescale 1 ns/ 100 ps
module BOOTROM(input [14:0]addra,input clka,output reg[31:0]douta,input ena);
reg [31:0]BRAM32[32767:0];

initial
begin 
	$readmemb("./bin.txt",BRAM32);
	#5 douta<=0;
end
always@(posedge clka )
begin
	douta<=(ena)?BRAM32[addra]:32'b0;
end



endmodule