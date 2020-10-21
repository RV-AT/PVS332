// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2019.1 (win64) Build 2552052 Fri May 24 14:49:42 MDT 2019
// Date        : Thu Nov 14 18:22:59 2019
// Host        : PC running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               D:/projects/FPGA/PVS332/Xilinx/PVS332.runs/clk_wiz_0_synth_1/clk_wiz_0_stub.v
// Design      : clk_wiz_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7s15ftgb196-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
module clk_wiz_0(clk_in2, clk_in_sel, clk_out1, resetn, locked, 
  clk_in1)
/* synthesis syn_black_box black_box_pad_pin="clk_in2,clk_in_sel,clk_out1,resetn,locked,clk_in1" */;
  input clk_in2;
  input clk_in_sel;
  output clk_out1;
  input resetn;
  output locked;
  input clk_in1;
endmodule
