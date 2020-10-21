`define SIMULATION
`timescale 1 ns/ 100 ps
//傻风牌SoC验证平台
module PVS332_SELFTEST();
// constants                                           
// general purpose registers

// test vector input registers
	reg CLK;
    reg RST_N;
    wire [21:0]SRAM_Addr;
	wire [31:0]SRAM_Data;
	wire [3:0]SRAM_BSEL;
	wire [22:0]AFGPIO;
	wire [3:0]QPI1;
	wire QPI_CS;
	wire QCLK;
	initial
	begin
		#0 CLK=0;RST_N=0;
		#100 RST_N=1;
	end
	always #10 CLK=~CLK; //50MHz CLK
	assign AFGPIO[12]=!(~AFGPIO[11]);
	PRV332_SoC
	SoC1(  
    .CLK(CLK),
    .RST_N(RST_N),
    .AFGPIO(AFGPIO), 
    //SPI/QPI
	.SPI_CS(QPI_CS),
	.SPI_MOSI(QPI1[0]),
    .SPI_MISO(QPI1[1]), 
    .SPI_SCLK(QCLK),
    //SRAM
    .SRAM_Addr(SRAM_Addr),
    .SRAM_Data(SRAM_Data),
	.SRAM_BSEL(SRAM_BSEL),
    .SRAM_CS(SRAM_CS),
    .SRAM_WR(SRAM_WR),
    .SRAM_OE(SRAM_OE),
	.SRAM_RDY(1'b1)
    );
    VIS62
	ExtSRAM1(
	SRAM_Data,
	SRAM_Addr,
	SRAM_BSEL,
	SRAM_CS,
    SRAM_WR,
    SRAM_OE
	);
	
                                                
endmodule

