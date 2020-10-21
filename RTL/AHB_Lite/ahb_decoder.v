//
//  Module: AHB decoder
//  (defined by ARM Design Kit Technical Reference Manual--AHB component)
//  Author: Lianghao Yuan
//  Email: yuanlianghao@gmail.com
//  Date: 07/13/2015
//  Description:
//  The system decoder decodes the address bus and generates select lines to
//  each of the system bus slaves, indicating that a read or write access to
//  that slave is required. The default configuration is 7 slots. No REMAP
//  signal implemented. 

`ifndef AHB_DECODER_V
`define AHB_DECODER_V

//`include "ahb_defines.v"

module ahb_decoder
(
  // -------------
  // Input pins //
  // -------------
  input [35:0] HADDR,
  // --------------
  // Output pins //
  // --------------
  output HSELx0,
  output HSELx7,
  output HSELx1,
  output HSELx2,
  output HSELx3,
  output HSELx4,
  output HSELx5,
  output HSELx6
);
/*
RV/AT机中，规定地址[15:0]为段内偏移，[16:23]为小段号，[31:24]为段号，[35:32]为大段号
 [ 35    :     32 ]    [31    :    24]  [23   :    16]   [15   :   0]
 Big Segment number_____Segment number__ Small Segment___   Offset
*/
//小段号译码
parameter sseg0  = 8'h00;
parameter sseg1  = 8'h01;
parameter sseg2  = 8'h02;
parameter sseg3  = 8'h03;
parameter sseg4  = 8'h04;
parameter sseg5  = 8'h05;
parameter sseg6  = 8'h06;
parameter sseg7  = 8'h07;
parameter sseg8  = 8'h08;
parameter sseg9  = 8'h09;
parameter sseg10 = 8'h0A;
parameter sseg11 = 8'h0B;
parameter sseg12 = 8'h0C;
parameter sseg13 = 8'h0D;
parameter sseg14 = 8'h0E;
parameter sseg15 = 8'h0F;
parameter sseg16 = 8'h10;
parameter sseg17 = 8'h11;
parameter sseg18 = 8'h12;
parameter sseg19 = 8'h13;
parameter sseg20 = 8'h14;
parameter sseg21 = 8'h15;
parameter sseg22 = 8'h16;
parameter sseg23 = 8'h17;
parameter sseg24 = 8'h18;
parameter sseg25 = 8'h19;
parameter sseg26 = 8'h1A;
parameter sseg27 = 8'h1B;
parameter sseg28 = 8'h1C;
parameter sseg29 = 8'h1D;
parameter sseg30 = 8'h1E;
parameter sseg31 = 8'h1F; 
//段号译码
parameter seg0  = 8'h00;
parameter seg1  = 8'h01;
parameter seg2  = 8'h02;
parameter seg3  = 8'h03;
parameter seg4  = 8'h04;
parameter seg5  = 8'h05;
parameter seg6  = 8'h06;
parameter seg7  = 8'h07;
parameter seg8  = 8'h08;
parameter seg9  = 8'h09;
parameter seg10 = 8'h0A;
parameter seg11 = 8'h0B;
parameter seg12 = 8'h0C;
parameter seg13 = 8'h0D;
parameter seg14 = 8'h0E;
parameter seg15 = 8'h0F;
parameter seg16 = 8'h10;
parameter seg17 = 8'h11;
parameter seg18 = 8'h12;
parameter seg19 = 8'h13;
parameter seg20 = 8'h14;
parameter seg21 = 8'h15;
parameter seg22 = 8'h16;
parameter seg23 = 8'h17;
parameter seg24 = 8'h18;
parameter seg25 = 8'h19;
parameter seg26 = 8'h1A;
parameter seg27 = 8'h1B;
parameter seg28 = 8'h1C;
parameter seg29 = 8'h1D;
parameter seg30 = 8'h1E;
parameter seg31 = 8'h1F;

//大段号译码
parameter bseg0 = 4'h0;
parameter bseg1 = 4'h1;
parameter bseg2 = 4'h2;
parameter bseg3 = 4'h3;
parameter bseg4 = 4'h4;
parameter bseg5 = 4'h5;
parameter bseg6 = 4'h6;
parameter bseg7 = 4'h7;
parameter bseg8 = 4'h8;
parameter bseg9 = 4'h9;
parameter bseg10 = 4'hA;
parameter bseg11 = 4'hB;
parameter bseg12 = 4'hC;
parameter bseg13 = 4'hD;
parameter bseg14 = 4'hE;
parameter bseg15 = 4'hF;

//小段号译码
wire small_segment0;
wire small_segment1;
wire small_segment2;
wire small_segment3;
wire small_segment4;
wire small_segment5;
wire small_segment6;
wire small_segment7;
wire small_segment8;
wire small_segment9;
wire small_segment10;
wire small_segment11;
wire small_segment12;
wire small_segment13;
wire small_segment14;
wire small_segment15;
wire small_segment16;
wire small_segment17;
wire small_segment18;
wire small_segment19;
wire small_segment20;
wire small_segment21;
wire small_segment22;
wire small_segment23;
wire small_segment24;
wire small_segment25;
wire small_segment26;
wire small_segment27;
wire small_segment28;
wire small_segment29;
wire small_segment30;
wire small_segment31;
//段号译码
wire segment0;
wire segment1;
wire segment2;
wire segment3;
wire segment4;
wire segment5;
wire segment6;
wire segment7;
wire segment8;
wire segment9;
wire segment10;
wire segment11;
wire segment12;
wire segment13;
wire segment14;
wire segment15;
wire segment16;
wire segment17;
wire segment18;
wire segment19;
wire segment20;
wire segment21;
wire segment22;
wire segment23;
wire segment24;
wire segment25;
wire segment26;
wire segment27;
wire segment28;
wire segment29;
wire segment30;
wire segment31;
//大段号译码
wire big_segment0;
wire big_segment1;
wire big_segment2;
wire big_segment3;
wire big_segment4;
wire big_segment5;
wire big_segment6;
wire big_segment7;
wire big_segment8;
wire big_segment9;
wire big_segment10;
wire big_segment11;
wire big_segment12;
wire big_segment13;
wire big_segment14;
wire big_segment15;
assign small_segment0 = (HADDR[31:16]==sseg0);
assign small_segment1 = (HADDR[31:16]==sseg1);
assign small_segment2 = (HADDR[31:16]==sseg2);
assign small_segment3 = (HADDR[31:16]==sseg3);
assign small_segment4 = (HADDR[31:16]==sseg4);
assign small_segment5 = (HADDR[31:16]==sseg5);
assign small_segment6 = (HADDR[31:16]==sseg6);
assign small_segment7 = (HADDR[31:16]==sseg7);
assign small_segment8 = (HADDR[31:16]==sseg8);
assign small_segment9 = (HADDR[31:16]==sseg9);
assign small_segment10 = (HADDR[31:16]==sseg10);
assign small_segment11 = (HADDR[31:16]==sseg11);
assign small_segment12 = (HADDR[31:16]==sseg12);
assign small_segment13 = (HADDR[31:16]==sseg13);
assign small_segment14 = (HADDR[31:16]==sseg14);
assign small_segment15 = (HADDR[31:16]==sseg15);
assign small_segment16 = (HADDR[31:16]==sseg16);
assign small_segment17 = (HADDR[31:16]==sseg17);
assign small_segment18 = (HADDR[31:16]==sseg18);
assign small_segment19 = (HADDR[31:16]==sseg19);
assign small_segment20 = (HADDR[31:16]==sseg20);
assign small_segment21 = (HADDR[31:16]==sseg21);
assign small_segment22 = (HADDR[31:16]==sseg22);
assign small_segment23 = (HADDR[31:16]==sseg23);
assign small_segment24 = (HADDR[31:16]==sseg24);
assign small_segment25 = (HADDR[31:16]==sseg25);
assign small_segment26 = (HADDR[31:16]==sseg26);
assign small_segment27 = (HADDR[31:16]==sseg27);
assign small_segment28 = (HADDR[31:16]==sseg28);
assign small_segment29 = (HADDR[31:16]==sseg29);
assign small_segment30 = (HADDR[31:16]==sseg30);
assign small_segment31 = (HADDR[31:16]==sseg31);

assign segment0 = (HADDR[31:16]==seg0);
assign segment1 = (HADDR[31:16]==seg1);
assign segment2 = (HADDR[31:16]==seg2);
assign segment3 = (HADDR[31:16]==seg3);
assign segment4 = (HADDR[31:16]==seg4);
assign segment5 = (HADDR[31:16]==seg5);
assign segment6 = (HADDR[31:16]==seg6);
assign segment7 = (HADDR[31:16]==seg7);
assign segment8 = (HADDR[31:16]==seg8);
assign segment9 = (HADDR[31:16]==seg9);
assign segment10 = (HADDR[31:16]==seg10);
assign segment11 = (HADDR[31:16]==seg11);
assign segment12 = (HADDR[31:16]==seg12);
assign segment13 = (HADDR[31:16]==seg13);
assign segment14 = (HADDR[31:16]==seg14);
assign segment15 = (HADDR[31:16]==seg15);
assign segment16 = (HADDR[31:16]==seg16);
assign segment17 = (HADDR[31:16]==seg17);
assign segment18 = (HADDR[31:16]==seg18);
assign segment19 = (HADDR[31:16]==seg19);
assign segment20 = (HADDR[31:16]==seg20);
assign segment21 = (HADDR[31:16]==seg21);
assign segment22 = (HADDR[31:16]==seg22);
assign segment23 = (HADDR[31:16]==seg23);
assign segment24 = (HADDR[31:16]==seg24);
assign segment25 = (HADDR[31:16]==seg25);
assign segment26 = (HADDR[31:16]==seg26);
assign segment27 = (HADDR[31:16]==seg27);
assign segment28 = (HADDR[31:16]==seg28);
assign segment29 = (HADDR[31:16]==seg29);
assign segment30 = (HADDR[31:16]==seg30);
assign segment31 = (HADDR[31:16]==seg31);

assign big_segment0 = (HADDR[35:32]==bseg0);
assign big_segment1 = (HADDR[35:32]==bseg1);
assign big_segment2 = (HADDR[35:32]==bseg2);
assign big_segment3 = (HADDR[35:32]==bseg3);
assign big_segment4 = (HADDR[35:32]==bseg4);
assign big_segment5 = (HADDR[35:32]==bseg5);
assign big_segment6 = (HADDR[35:32]==bseg6);
assign big_segment7 = (HADDR[35:32]==bseg7);
assign big_segment8 = (HADDR[35:32]==bseg8);
assign big_segment9 = (HADDR[35:32]==bseg9);
assign big_segment10 = (HADDR[35:32]==bseg10);
assign big_segment11 = (HADDR[35:32]==bseg11);
assign big_segment12 = (HADDR[35:32]==bseg12);
assign big_segment13 = (HADDR[35:32]==bseg13);
assign big_segment14 = (HADDR[35:32]==bseg14);
assign big_segment15 = (HADDR[35:32]==bseg15);



assign HSELx0 = big_segment0 & segment0 & small_segment0;	//第0大段第0段第0小段，Page 0x0000_0000-0x0000_ffff 	(64KiB)
assign HSELx1=  big_segment0 & (segment0 | segment1 | segment2 | segment3) & !small_segment0; 
//(HSELx0==0)&(HADDR[35:26]==10'h00); Page 0x0001_0000-0x03ff_ffff 	(64MiB)

assign HSELx2=  big_segment0 & (segment0 | segment1 | segment2 | segment3 | segment4 | segment5 | segment6 | segment7);                 
//(HADDR[35:26]==10'h01);		//Page 0x0400_0000-0x07ff_ffff 	(64MiB)


	assign HSELx3=0;		//
	assign HSELx4=0;		//
	assign HSELx5=0;		//
	assign HSELx6=0;		//
	assign HSELx7={HSELx0,HSELx1,HSELx2,HSELx3,HSELx4,HSELx5,HSELx6}==0;//Reserved
endmodule

`endif // AHB_DECODER_V
