//////////////////////////////////////////////////////////////////////
////                                                              ////
////  Simple Small VGA IP Core                                    ////
////                                                              ////
////  This file is part of the Simple Small VGA project           ////
////                                                              ////
////                                                              ////
////  Description                                                 ////
////  512 entry FIFO for storing line video data. It uses one     ////
////  clock for reading and writing.                              ////
////                                                              ////
////  To Do:                                                      ////
////   Nothing                                                    ////
////                                                              ////
////  Author(s):                                                  ////
////      - Damjan Lampret, lampret@opencores.org                 ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000 Authors and OPENCORES.ORG                 ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
//
// CVS Revision History
//
// $Log: not supported by cvs2svn $
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

module ssvga_fifo(
	clk, crt_clk, rst, dat_i, wr_en, rd_en,
	dat_o, full, empty, ssvga_en
);

//
// I/O ports
//
input			clk;		// Clock
input           crt_clk;    // Clock for monitor
input			rst;		// Reset
input	[31:0]	dat_i;		// Input data
input			wr_en;		// Write enable
input			rd_en;		// Read enable
output	[7:0]	dat_o;		// Output data
output			full;		// Full flag
output			empty;		// Empty flag
input           ssvga_en ;  // vga enable

//
// Internal wires and regs
//
reg	[7:0]		wr_ptr;		    // Write pointer
reg	[7:0]		wr_ptr_plus1;   // Write pointer
reg	[9:0]		rd_ptr;		    // Read pointer
reg	[9:0]		rd_ptr_plus1;	// Read pointer plus1
wire			rd_en_int;	    // FIFO internal read enable

wire [9:0]      gray_rd_ptr;      // gray code of read pointer
wire [9:0]      gray_wr_ptr;      // gray code of write pointer
wire [7:0]      gray_wr_ptr_plus1;// gray code of write + 1 pointer

reg  [9:0]      gray_read_ptr;    // sinchronized gray read pointer on clk clock
wire [9:0]      sync_gray_rd_ptr; // intermediate sinc. of gray read pointer

reg             rd_ssvga_en;    // sinchronized ssvga enable on crt_clk clock
wire            sync_ssvga_en;  // intermediate sinc. of ssvga enable signal

//
// Write pointer + 1
//

always @(posedge clk or posedge rst)
    if (rst)
		wr_ptr_plus1 <= #1 8'b0000_0001 ;
    else if (~ssvga_en)
        wr_ptr_plus1 <= #1 8'b0000_0001 ;
	else if (wr_en)
		wr_ptr_plus1 <= #1 wr_ptr_plus1 + 1;

//
// Write pointer
//
always @(posedge clk or posedge rst)
	if (rst)
		wr_ptr <= #1 8'b0000_0000;
    else if (~ssvga_en)
        wr_ptr <= #1 8'b0000_0000;
	else if (wr_en)
		wr_ptr <= #1 wr_ptr_plus1 ;

//
// Read pointer
//
always @(posedge crt_clk or posedge rst)
	if (rst)
		rd_ptr <= #1 10'b00_0000_0000;
    else if (~rd_ssvga_en)
        rd_ptr <= #1 10'b00_0000_0000;
	else if (rd_en_int)
		rd_ptr <= #1 rd_ptr_plus1 ;

always @(posedge crt_clk or posedge rst)
	if (rst)
		rd_ptr_plus1 <= #1 10'b00_0000_0001;
    else if (~rd_ssvga_en)
        rd_ptr_plus1 <= #1 10'b00_0000_0001;
	else if (rd_en_int)
		rd_ptr_plus1 <= #1 rd_ptr_plus1 + 1 ;

//
// Empty is asserted when both pointers match
//
//assign empty = ( rd_ptr == {wr_ptr, 2'b00} ) ;
assign empty = ( gray_wr_ptr == gray_read_ptr ) ;

//
// Full is asserted when both pointers match
// and wr_ptr did increment in previous clock cycle
//
//assign full = ( wr_ptr_plus1 == rd_ptr[9:2] ) ;
assign full = ( gray_wr_ptr_plus1 == gray_read_ptr[9:2] ) ;

wire valid_pix = 1'b1 ;

//
// Read enable for FIFO
//
assign rd_en_int = rd_en & !empty & valid_pix;

wire [8:0] ram_pix_address = rd_en_int ? {rd_ptr_plus1[9:2], rd_ptr_plus1[0]} : {rd_ptr[9:2], rd_ptr[0]} ;

wire [7:0] dat_o_low ;
wire [7:0] dat_o_high ;

assign dat_o = rd_ptr[1] ? dat_o_high : dat_o_low ;

//#############################################################################
// binary to gray converters for counter of different clock domain comparison
assign gray_rd_ptr          = (rd_ptr >> 1)          ^ rd_ptr ;
assign gray_wr_ptr          = ({1'b0, wr_ptr, 1'b0}) ^ ({wr_ptr, 2'b00}) ;
assign gray_wr_ptr_plus1    = (wr_ptr_plus1 >> 1)    ^ wr_ptr_plus1 ;

//#############################################################################
// interemediate stage to clk synchronization flip - flops - this ones are prone to metastability
synchronizer_flop   #(10) read_ptr_sync
(
    .data_in        (gray_rd_ptr), 
    .clk_out        (clk), 
    .sync_data_out  (sync_gray_rd_ptr), 
    .async_reset    (rst)
) ;
always@(posedge clk or posedge rst)
begin
    if (rst)
        gray_read_ptr <= #1 10'b0 ;
    else
        gray_read_ptr <= #1 sync_gray_rd_ptr ;
end

//##############################################################################
// interemediate stage ssvga_en synchronization flip - flop - this one is prone to metastability
synchronizer_flop   ssvga_enable_sync
(
    .data_in        (ssvga_en), 
    .clk_out        (crt_clk), 
    .sync_data_out  (sync_ssvga_en), 
    .async_reset    (rst)
) ;
// crt side ssvga enable flip flop - gets a value from intermediate stage sync flip flop
always@(posedge crt_clk or posedge rst)
begin
    if (rst)
        rd_ssvga_en <= #1 1'b0 ;
    else
        rd_ssvga_en <= #1 sync_ssvga_en ;
end


RAMB4_S8_S16 ramb4_s8_0(
	.CLKA(crt_clk),
	.RSTA(rst),
	.ADDRA(ram_pix_address),
	.DIA(8'h00),
	.ENA(1'b1),
	.WEA(1'b0),
	.DOA(dat_o_low),

	.CLKB(clk),
	.RSTB(rst),
	.ADDRB(wr_ptr),
	.DIB(dat_i[15:0]),
	.ENB(1'b1),
	.WEB(wr_en),
	.DOB()
);

RAMB4_S8_S16 ramb4_s8_1(
	.CLKA(crt_clk),
	.RSTA(rst),
	.ADDRA(ram_pix_address),
	.DIA(8'h00),
	.ENA(1'b1),
	.WEA(1'b0),
	.DOA(dat_o_high),

	.CLKB(clk),
	.RSTB(rst),
	.ADDRB(wr_ptr),
	.DIB(dat_i[31:16]),
	.ENB(1'b1),
	.WEB(wr_en),
	.DOB()
);

endmodule
