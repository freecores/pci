//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "pciw_fifo_control.v"                             ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - Miha Dolenc (mihad@opencores.org)                     ////
////                                                              ////
////  All additional information is avaliable in the README       ////
////  file.                                                       ////
////                                                              ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2001 Miha Dolenc, mihad@opencores.org          ////
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
// $Log
//

/* FIFO_CONTROL module provides read/write address and status generation for
   FIFOs implemented with standard dual port SRAM cells in ASIC or FPGA designs */
`include "pci_constants.v"
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

module PCIW_FIFO_CONTROL
(
    rclock_in,
    wclock_in,
    renable_in,
    wenable_in,
    reset_in,
//    flush_in,         // not used
    almost_full_out,
    full_out,
    almost_empty_out,
    empty_out,
    waddr_out,
    raddr_out,
    rallow_out,
    wallow_out,
    two_left_out
);

parameter ADDR_LENGTH = 7 ;

// independent clock inputs - rclock_in = read clock, wclock_in = write clock
input  rclock_in, wclock_in;

// enable inputs - read address changes on rising edge of rclock_in when reads are allowed
//                 write address changes on rising edge of wclock_in when writes are allowed
input  renable_in, wenable_in;

// reset input
input  reset_in;

// flush input
//input flush_in ;      // not used

// almost full and empy status outputs
output almost_full_out, almost_empty_out;

// full and empty status outputs
output full_out, empty_out;

// read and write addresses outputs
output [(ADDR_LENGTH - 1):0] waddr_out, raddr_out;

// read and write allow outputs
output rallow_out, wallow_out ;

// two locations left output indicator
output two_left_out ;

// read address register
reg [(ADDR_LENGTH - 1):0] raddr ;

// write address register
reg [(ADDR_LENGTH - 1):0] waddr;
assign waddr_out = waddr ;

// grey code registers
// grey code pipeline for write address
reg [(ADDR_LENGTH - 1):0] wgrey_minus1 ; // current
reg [(ADDR_LENGTH - 1):0] wgrey_addr ; // current
reg [(ADDR_LENGTH - 1):0] wgrey_next ; // next

// next write gray address calculation - bitwise xor between address and shifted address
wire [(ADDR_LENGTH - 2):0] calc_wgrey_next  = waddr[(ADDR_LENGTH - 1):1] ^ waddr[(ADDR_LENGTH - 2):0] ;

// grey code pipeline for read address
reg [(ADDR_LENGTH - 1):0] rgrey_minus3 ; // three before current
reg [(ADDR_LENGTH - 1):0] rgrey_minus2 ; // two before current
reg [(ADDR_LENGTH - 1):0] rgrey_minus1 ; // one before current
reg [(ADDR_LENGTH - 1):0] rgrey_addr ; // current
reg [(ADDR_LENGTH - 1):0] rgrey_next ; // next

// next read gray address calculation - bitwise xor between address and shifted address
wire [(ADDR_LENGTH - 2):0] calc_rgrey_next  = raddr[(ADDR_LENGTH - 1):1] ^ raddr[(ADDR_LENGTH - 2):0] ;

// FFs for registered empty and full flags
wire empty ;
wire full ;

// registered almost_empty and almost_full flags
wire almost_empty ;
wire almost_full ;

// write allow wire - writes are allowed when fifo is not full
wire wallow = wenable_in && !full ;

// write allow output assignment
assign wallow_out = wallow ;

// read allow wire
wire rallow ;

// full output assignment
assign full_out  = full ;

// almost full output assignment
assign almost_full_out  = almost_full && !full ;

// clear generation for FFs and registers
wire clear = reset_in /*|| flush_in*/ ;     // flush not used for write fifo

reg wclock_nempty_detect ;
always@(posedge reset_in or posedge wclock_in)
begin
    if (reset_in)
        wclock_nempty_detect <= #`FF_DELAY 1'b0 ;
    else
        wclock_nempty_detect <= #`FF_DELAY (rgrey_addr != wgrey_addr) ;
end

wire stretched_empty ;

wire stretched_empty_flop_i = empty && ~wclock_nempty_detect ;

meta_flop #(1) i_meta_flop_stretched_empty
(
    .rst_i      (clear),
    .clk_i      (rclock_in),
    .ld_i       (1'b0),
    .ld_val_i   (1'b0),
    .en_i       (1'b1),
    .d_i        (stretched_empty_flop_i),
    .meta_q_o   (stretched_empty)
) ;

// empty output is actual empty + 1 read clock cycle ( stretched empty )
assign empty_out = empty  || stretched_empty ;

//rallow generation
assign rallow = renable_in && !empty && !stretched_empty ; // reads allowed if read enable is high and FIFO is not empty

// rallow output assignment
assign rallow_out = rallow ;

// almost empty output assignment
assign almost_empty_out = almost_empty && !empty && !stretched_empty ;

// at any clock edge that rallow is high, this register provides next read address, so wait cycles are not necessary
// when FIFO is empty, this register provides actual read address, so first location can be read
reg [(ADDR_LENGTH - 1):0] raddr_plus_one ;


// read address mux - when read is performed, next address is driven, so next data is available immediately after read
// this is convenient for zero wait stait bursts
assign raddr_out = rallow ? raddr_plus_one : raddr ;

always@(posedge rclock_in or posedge clear)
begin
    if (clear)
    begin
        // initial values seem a bit odd - they are this way to allow easier grey pipeline implementation and to allow min fifo size of 8
        raddr_plus_one <= #`FF_DELAY 6 ;
        raddr          <= #`FF_DELAY 5 ;
    end
    else if (rallow)
    begin
        raddr_plus_one <= #`FF_DELAY raddr_plus_one + 1'b1 ;
        raddr          <= #`FF_DELAY raddr_plus_one ;
    end
end

/*-----------------------------------------------------------------------------------------------
Read address control consists of Read address counter and Grey Address pipeline
There are 5 Grey addresses:
    - rgrey_minus3 is Grey Code of address three before current address
    - rgrey_minus2 is Grey Code of address two before current address
    - rgrey_minus1 is Grey Code of address one before current address
    - rgrey_addr is Grey Code of current read address
    - rgrey_next is Grey Code of next read address
--------------------------------------------------------------------------------------------------*/
// grey coded address pipeline for status generation in read clock domain
always@(posedge rclock_in or posedge clear)
begin
    if (clear)
    begin
        rgrey_minus3 <= #`FF_DELAY 0 ;
        rgrey_minus2 <= #`FF_DELAY 1 ;
        rgrey_minus1 <= #`FF_DELAY 3 ;  
        rgrey_addr   <= #`FF_DELAY 2 ;
        rgrey_next   <= #`FF_DELAY 6 ;
    end
    else
    if (rallow)
    begin
        rgrey_minus3 <= #`FF_DELAY rgrey_minus2 ;
        rgrey_minus2 <= #`FF_DELAY rgrey_minus1 ;
        rgrey_minus1 <= #`FF_DELAY rgrey_addr ;
        rgrey_addr   <= #`FF_DELAY rgrey_next ;
        rgrey_next   <= #`FF_DELAY {raddr[ADDR_LENGTH - 1], calc_rgrey_next} ;
    end
end

/*--------------------------------------------------------------------------------------------
Write address control consists of write address counter and three Grey Code Registers:
    - wgrey_minus1 holds grey coded address of one before current write address
    - wgrey_addr represents current Grey Coded write address
    - wgrey_next represents Grey Coded next write address
----------------------------------------------------------------------------------------------*/
// grey coded address pipeline for status generation in write clock domain
always@(posedge wclock_in or posedge clear)
begin
    if (clear)
    begin
        wgrey_minus1 <= #`FF_DELAY 3 ;
        wgrey_addr   <= #`FF_DELAY 2 ;
        wgrey_next   <= #`FF_DELAY 6 ;
    end
    else
    if (wallow)
    begin
        wgrey_minus1 <= #`FF_DELAY wgrey_addr ;
        wgrey_addr   <= #`FF_DELAY wgrey_next ;
        wgrey_next   <= #`FF_DELAY {waddr[(ADDR_LENGTH - 1)], calc_wgrey_next} ;
    end
end

// write address counter - nothing special except initial value
always@(posedge wclock_in or posedge clear)
begin
    if (clear)
        // initial value 5
        waddr <= #`FF_DELAY 5 ;
    else
    if (wallow)
        waddr <= #`FF_DELAY waddr + 1'b1 ;
end

/*------------------------------------------------------------------------------------------------------------------------------
Registered full control:
registered full is set on rising edge of wclock_in, when one location is left in fifo and another is written
It's kept high until something is read from FIFO, which is registered on
next rising write clock edge.

Registered almost full control:
registered almost full is set on rising edge of write clock when two locations are left in fifo and another is written to it.
it's kept high until something is read/written from/to fifo

Registered two left control:
registered two left is set on rising edge of write clock when three locations are left in fifo and another is written to it.
it's kept high until something is read/written from/to fifo.
--------------------------------------------------------------------------------------------------------------------------------*/
wire comb_full          = wgrey_next == rgrey_addr ;
wire comb_almost_full   = wgrey_addr == rgrey_minus2 ;
wire comb_two_left      = wgrey_next == rgrey_minus2 ;
wire comb_three_left    = wgrey_next == rgrey_minus3 ;

//combinatorial input to Registered full FlipFlop
wire reg_full = (wallow && comb_almost_full) || (comb_full) ;

meta_flop #(0) i_meta_flop_full
(
    .rst_i       (clear),
    .clk_i       (wclock_in),
    .ld_i        (1'b0),
    .ld_val_i    (1'b0),
    .en_i        (1'b1), 
    .d_i         (reg_full),
    .meta_q_o    (full)
) ;

// input for almost full flip flop
wire reg_almost_full_in = wallow && comb_two_left || comb_almost_full ;

meta_flop #(0) i_meta_flop_almost_full
(
    .rst_i       (clear),
    .clk_i       (wclock_in),
    .ld_i        (1'b0),
    .ld_val_i    (1'b0),
    .en_i        (1'b1),
    .d_i         (reg_almost_full_in),
    .meta_q_o    (almost_full)
) ;

wire reg_two_left_in = wallow && comb_three_left || comb_two_left ;

meta_flop #(0) i_meta_flop_two_left
(
    .rst_i       (clear),
    .clk_i       (wclock_in),
    .ld_i        (1'b0),
    .ld_val_i    (1'b0),
    .en_i        (1'b1),
    .d_i         (reg_two_left_in),
    .meta_q_o    (two_left_out)
) ;

/*------------------------------------------------------------------------------------------------------------------------------
Registered empty control:
registered empty is set on rising edge of rclock_in,
when only one location is used in and read from fifo. It's kept high until something is written to FIFO, which is registered on
the next read clock.

Registered almost empty control:
almost empty is set on rising clock edge of rclock when two locations are used and one read from FIFO. It's kept high until
something is read/written from/to fifo.
--------------------------------------------------------------------------------------------------------------------------------*/
wire comb_almost_empty  = rgrey_next == wgrey_addr ;
wire comb_empty         = rgrey_addr == wgrey_addr ;
wire comb_two_used      = rgrey_next == wgrey_minus1 ;

// combinatorial input for registered emty FlipFlop
//wire reg_empty = (rallow && comb_almost_empty) || comb_empty ;
wire reg_empty = (rallow && almost_empty) || comb_empty ;

meta_flop #(1) i_meta_flop_empty
(
    .rst_i      (clear),
    .clk_i      (rclock_in),
    .ld_i       (1'b0),
    .ld_val_i   (1'b0),
    .en_i       (1'b1),
    .d_i        (reg_empty),
    .meta_q_o   (empty)
) ;

// input for almost empty flip flop
wire reg_almost_empty = rallow && comb_two_used || comb_almost_empty ;

meta_flop #(0) i_meta_flop_almost_empty
(
    .rst_i      (clear),
    .clk_i      (rclock_in),
    .ld_i       (1'b0),
    .ld_val_i   (1'b0),
    .en_i       (1'b1),
    .d_i        (reg_almost_empty),
    .meta_q_o   (almost_empty)
) ;

endmodule
