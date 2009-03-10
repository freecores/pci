//////////////////////////////////////////////////////////////////////
////                                                              ////
////  Generic Two-Port Synchronous RAM                            ////
////                                                              ////
////  This file is part of pci bridge project                     ////
////  http://www.opencores.org/cvsweb.shtml/pci/                  ////
////                                                              ////
////  Description                                                 ////
////  This block is a wrapper with common two-port                ////
////  synchronous memory interface for different                  ////
////  types of ASIC and FPGA RAMs. Beside universal memory        ////
////  interface it also provides behavioral model of generic      ////
////  two-port synchronous RAM.                                   ////
////  It should be used in all OPENCORES designs that want to be  ////
////  portable accross different target technologies and          ////
////  independent of target memory.                               ////
////                                                              ////
////  Supported ASIC RAMs are:                                    ////
////  - Artisan Double-Port Sync RAM                              ////
////  - Avant! Two-Port Sync RAM (*)                              ////
////  - Virage 2-port Sync RAM                                    ////
////                                                              ////
////  Supported FPGA RAMs are:                                    ////
////  - Xilinx Virtex RAMB4_S16_S16                               ////
////                                                              ////
////  To Do:                                                      ////
////   - fix Avant!                                               ////
////   - xilinx rams need external tri-state logic                ////
////   - add additional RAMs (Altera, VS etc)                     ////
////                                                              ////
////  Author(s):                                                  ////
////      - Damjan Lampret, lampret@opencores.org                 ////
////      - Miha Dolenc, mihad@opencores.org                      ////
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
`include "pci_constants.v"

module WB_TPRAM
(
	// Generic synchronous two-port RAM interface
	clk_a,
    rst_a,
    ce_a,
    we_a,
    oe_a,
    addr_a,
    di_a,
    do_a,
	clk_b,
    rst_b,
    ce_b,
    we_b,
    oe_b,
    addr_b,
    di_b,
    do_b
);

//
// Default address and data buses width
//
parameter aw = 8;
parameter dw = 40;

//
// Generic synchronous two-port RAM interface
//
input			clk_a;	// Clock
input			rst_a;	// Reset
input			ce_a;	// Chip enable input
input			we_a;	// Write enable input
input			oe_a;	// Output enable input
input 	[aw-1:0]	addr_a;	// address bus inputs
input	[dw-1:0]	di_a;	// input data bus
output	[dw-1:0]	do_a;	// output data bus
input			clk_b;	// Clock
input			rst_b;	// Reset
input			ce_b;	// Chip enable input
input			we_b;	// Write enable input
input			oe_b;	// Output enable input
input 	[aw-1:0]	addr_b;	// address bus inputs
input	[dw-1:0]	di_b;	// input data bus
output	[dw-1:0]	do_b;	// output data bus

//
// Internal wires and registers
//


`ifdef WB_ARTISAN_SDP
    `define RAM_SELECTED
    //
    // Instantiation of ASIC memory:
    //
    // Artisan Synchronous Double-Port RAM (ra2sh)
    //
    art_hsdp_256x40 /*#(dw, 1<<aw, aw) */ artisan_sdp
    (
    	.qa(do_a),
    	.clka(clk_a),
    	.cena(~ce_a),
    	.wena(~we_a),
    	.aa(addr_a),
    	.da(di_a),
    	.oena(~oe_a),
    	.qb(do_b),
    	.clkb(clk_b),
    	.cenb(~ce_b),
    	.wenb(~we_b),
    	.ab(addr_b),
    	.db(di_b),
    	.oenb(~oe_b)
    );

`endif

`ifdef AVANT_ATP
    `define RAM_SELECTED
    //
    // Instantiation of ASIC memory:
    //
    // Avant! Asynchronous Two-Port RAM
    //
    avant_atp avant_atp(
    	.web(~we),
    	.reb(),
    	.oeb(~oe),
    	.rcsb(),
    	.wcsb(),
    	.ra(addr),
    	.wa(addr),
    	.di(di),
    	.do(do)
    );

`endif

`ifdef VIRAGE_STP
    `define RAM_SELECTED
    //
    // Instantiation of ASIC memory:
    //
    // Virage Synchronous 2-port R/W RAM
    //
    virage_stp virage_stp(
    	.QA(do_a),
    	.QB(do_b),

    	.ADRA(addr_a),
    	.DA(di_a),
    	.WEA(we_a),
    	.OEA(oe_a),
    	.MEA(ce_a),
    	.CLKA(clk_a),

    	.ADRB(adr_b),
    	.DB(di_b),
    	.WEB(we_b),
    	.OEB(oe_b),
    	.MEB(ce_b),
    	.CLKB(clk_b)
    );

`endif

`ifdef WB_XILINX_RAMB4
    `define RAM_SELECTED
    //
    // Instantiation of FPGA memory:
    //
    // Virtex/Spartan2
    //

    //
    // Block 0
    //

    RAMB4_S16_S16 ramb4_s16_s16_0(
    	.CLKA(clk_a),
    	.RSTA(rst_a),
    	.ADDRA(addr_a),
    	.DIA(di_a[15:0]),
    	.ENA(ce_a),
    	.WEA(we_a),
    	.DOA(do_a[15:0]),

    	.CLKB(clk_b),
    	.RSTB(rst_b),
    	.ADDRB(addr_b),
    	.DIB(di_b[15:0]),
    	.ENB(ce_b),
    	.WEB(we_b),
    	.DOB(do_b[15:0])
    );

    //
    // Block 1
    //

    RAMB4_S16_S16 ramb4_s16_s16_1(
    	.CLKA(clk_a),
    	.RSTA(rst_a),
    	.ADDRA(addr_a),
    	.DIA(di_a[31:16]),
    	.ENA(ce_a),
    	.WEA(we_a),
    	.DOA(do_a[31:16]),

    	.CLKB(clk_b),
    	.RSTB(rst_b),
    	.ADDRB(addr_b),
    	.DIB(di_b[31:16]),
    	.ENB(ce_b),
    	.WEB(we_b),
    	.DOB(do_b[31:16])
    );

    //
    // Block 2
    //
    // block ram2 wires - non generic width of block rams
    wire [15:0] blk2_di_a = {8'h00, di_a[39:32]} ;
    wire [15:0] blk2_di_b = {8'h00, di_b[39:32]} ;

    wire [15:0] blk2_do_a ;
    wire [15:0] blk2_do_b ;

    assign do_a[39:32] = blk2_do_a[7:0] ;
    assign do_b[39:32] = blk2_do_b[7:0] ;

    RAMB4_S16_S16 ramb4_s16_s16_2(
            .CLKA(clk_a),
            .RSTA(rst_a),
            .ADDRA(addr_a),
            .DIA(blk2_di_a),
            .ENA(ce_a),
            .WEA(we_a),
            .DOA(blk2_do_a),

            .CLKB(clk_b),
            .RSTB(rst_b),
            .ADDRB(addr_b),
            .DIB(blk2_di_b),
            .ENB(ce_b),
            .WEB(we_b),
            .DOB(blk2_do_b)
    );

`endif
`ifdef WB_XILINX_DIST_RAM
    `define RAM_SELECTED
    reg [(aw-1):0] out_address ;
    always@(posedge clk_b or posedge rst_b)
    begin
        if ( rst_b )
            out_address <= #1 0 ;
        else if (ce_b)
            out_address <= #1 addr_b ;
    end

    WB_DIST_RAM #(aw) wb_distributed_ram
    (
        .data_out       (do_b),
        .we             (we_a),
        .data_in        (di_a),
        .read_address   (out_address),
        .write_address  (addr_a),
        .wclk           (clk_a)
    );
`endif

`ifdef RAM_SELECTED
    `undef RAM_SELECTED
`else
    //
    // Generic two-port synchronous RAM model
    //

    //
    // Generic RAM's registers and wires
    //
    reg	[dw-1:0]	mem [(1<<aw)-1:0];	// RAM content
    reg	[dw-1:0]	do_reg_a;		// RAM data output register
    reg	[dw-1:0]	do_reg_b;		// RAM data output register

    //
    // Data output drivers
    //
    assign do_a = (oe_a) ? do_reg_a : {dw{1'bz}};
    assign do_b = (oe_b) ? do_reg_b : {dw{1'bz}};

    //
    // RAM read and write
    //
    always @(posedge clk_a)
    	if (ce_a && !we_a)
    		do_reg_a <= #1 mem[addr_a];
    	else if (ce_a && we_a)
    		mem[addr_a] <= #1 di_a;

    //
    // RAM read and write
    //
    always @(posedge clk_b)
    	if (ce_b && !we_b)
    		do_reg_b <= #1 mem[addr_b];
    	else if (ce_b && we_b)
    		mem[addr_b] <= #1 di_b;
`endif

// synopsys translate_off
initial
begin
    if (dw !== 40)
    begin
        $display("RAM instantiation error! Expected RAM width %d, actual %h!", 40, dw) ;
        $finish ;
    end
    `ifdef XILINX_RAMB4
        if (aw !== 8)
        begin
            $display("RAM instantiation error! Expected RAM address width %d, actual %h!", 40, aw) ;
            $finish ;
        end
    `endif
    // currenlty only artisan ram of depth 256 is supported - they don't provide generic ram models
    `ifdef ARTISAN_SDP
        if (aw !== 8)
        begin
            $display("RAM instantiation error! Expected RAM address width %d, actual %h!", 40, aw) ;
            $finish ;
        end
    `endif
end
// synopsys translate_on

endmodule

`ifdef WB_XILINX_DIST_RAM
module WB_DIST_RAM (data_out, we, data_in, read_address, write_address, wclk);
    parameter addr_width = 4 ;
    output [39:0] data_out;
    input we, wclk;
    input [39:0] data_in;
    input [addr_width - 1:0] write_address, read_address;

    wire [3:0] waddr = write_address ;
    wire [3:0] raddr = read_address ;

    RAM16X1D ram00 (.DPO(data_out[0]),  .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[0]),  .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram01 (.DPO(data_out[1]),  .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[1]),  .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram02 (.DPO(data_out[2]),  .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[2]),  .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram03 (.DPO(data_out[3]),  .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[3]),  .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram04 (.DPO(data_out[4]),  .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[4]),  .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram05 (.DPO(data_out[5]),  .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[5]),  .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram06 (.DPO(data_out[6]),  .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[6]),  .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram07 (.DPO(data_out[7]),  .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[7]),  .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram08 (.DPO(data_out[8]),  .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[8]),  .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram09 (.DPO(data_out[9]),  .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[9]),  .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram10 (.DPO(data_out[10]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[10]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram11 (.DPO(data_out[11]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[11]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram12 (.DPO(data_out[12]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[12]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram13 (.DPO(data_out[13]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[13]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram14 (.DPO(data_out[14]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[14]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram15 (.DPO(data_out[15]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[15]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram16 (.DPO(data_out[16]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[16]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram17 (.DPO(data_out[17]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[17]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram18 (.DPO(data_out[18]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[18]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram19 (.DPO(data_out[19]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[19]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram20 (.DPO(data_out[20]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[20]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram21 (.DPO(data_out[21]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[21]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram22 (.DPO(data_out[22]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[22]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram23 (.DPO(data_out[23]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[23]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram24 (.DPO(data_out[24]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[24]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram25 (.DPO(data_out[25]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[25]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram26 (.DPO(data_out[26]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[26]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram27 (.DPO(data_out[27]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[27]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram28 (.DPO(data_out[28]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[28]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram29 (.DPO(data_out[29]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[29]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram30 (.DPO(data_out[30]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[30]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram31 (.DPO(data_out[31]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[31]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram32 (.DPO(data_out[32]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[32]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram33 (.DPO(data_out[33]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[33]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram34 (.DPO(data_out[34]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[34]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram35 (.DPO(data_out[35]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[35]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram36 (.DPO(data_out[36]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[36]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram37 (.DPO(data_out[37]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[37]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram38 (.DPO(data_out[38]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[38]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
    RAM16X1D ram39 (.DPO(data_out[39]), .SPO(), .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]), .D(data_in[39]), .DPRA0(raddr[0]), .DPRA1(raddr[1]), .DPRA2(raddr[2]), .DPRA3(raddr[3]), .WCLK(wclk), .WE(we));
endmodule
`endif