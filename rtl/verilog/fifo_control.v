//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "fifo_control.v"                                  ////
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
// $Log: not supported by cvs2svn $
// Revision 1.6  2002/09/30 16:03:04  mihad
// Added meta flop module for easier meta stable FF identification during synthesis
//
// Revision 1.5  2002/09/25 15:53:52  mihad
// Removed all logic from asynchronous reset network
//
// Revision 1.4  2002/03/05 11:53:47  mihad
// Added some testcases, removed un-needed fifo signals
//
// Revision 1.3  2002/02/01 15:25:12  mihad
// Repaired a few bugs, updated specification, added test bench files and design document
//
// Revision 1.2  2001/10/05 08:14:28  mihad
// Updated all files with inclusion of timescale file for simulation purposes.
//
// Revision 1.1.1.1  2001/10/02 15:33:46  mihad
// New project directory structure
//
//

/* FIFO_CONTROL module provides read/write address and status generation for
   FIFOs implemented with standard dual port SRAM cells in ASIC or FPGA designs */

`include "pci_constants.v"

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

module FIFO_CONTROL
(
    rclock_in,
    wclock_in,
    renable_in,
    wenable_in,
    reset_in,
    flush_in,
    full_out,
    almost_empty_out,
    empty_out,
    waddr_out,
    raddr_out,
    rallow_out,
    wallow_out
);

// address length parameter - depends on fifo depth
parameter ADDR_LENGTH = 7 ;

// independent clock inputs - rclock_in = read clock, wclock_in = write clock
input  rclock_in, wclock_in;

// enable inputs - read address changes on rising edge of rclock_in when reads are allowed
//                 write address changes on rising edge of wclock_in when writes are allowed
input  renable_in, wenable_in;

// reset input
input  reset_in;

// flush input
input flush_in ;

// almost empy status output
output almost_empty_out;

// full and empty status outputs
output full_out, empty_out;

// read and write addresses outputs
output [(ADDR_LENGTH - 1):0] waddr_out, raddr_out;

// read and write allow outputs
output rallow_out, wallow_out ;

// read address register
reg [(ADDR_LENGTH - 1):0] raddr ;

// write address register
reg [(ADDR_LENGTH - 1):0] waddr;
assign waddr_out = waddr ;

// grey code registers
// grey code pipeline for write address
reg [(ADDR_LENGTH - 1):0] wgrey_minus1 ; // one before current grey coded write address
reg [(ADDR_LENGTH - 1):0] wgrey_addr ; // current grey coded write address
reg [(ADDR_LENGTH - 1):0] wgrey_next ; // next grey coded write address

// next write gray address calculation - bitwise xor between address and shifted address
wire [(ADDR_LENGTH - 2):0] calc_wgrey_next  = waddr[(ADDR_LENGTH - 1):1] ^ waddr[(ADDR_LENGTH - 2):0] ;

// grey code pipeline for read address
reg [(ADDR_LENGTH - 1):0] rgrey_minus1 ; // one before current
reg [(ADDR_LENGTH - 1):0] rgrey_addr ; // current
reg [(ADDR_LENGTH - 1):0] rgrey_next ; // next

// next read gray address calculation - bitwise xor between address and shifted address
wire [(ADDR_LENGTH - 2):0] calc_rgrey_next  = raddr[(ADDR_LENGTH - 1):1] ^ raddr[(ADDR_LENGTH - 2):0] ;

// FFs for registered empty and full flags
wire empty ;
wire full ;

// almost_empty tag
wire almost_empty ;

// write allow wire - writes are allowed when fifo is not full
wire wallow = wenable_in && !full ;

// write allow output assignment
assign wallow_out = wallow ;

// read allow wire
wire rallow ;

// full output assignment
assign full_out  = full ;

// clear generation for FFs and registers
wire clear = reset_in /*|| flush_in*/ ; // flush changed to synchronous operation

reg wclock_nempty_detect ;
always@(posedge clear or posedge wclock_in)
begin
    if (clear)
        wclock_nempty_detect <= #`FF_DELAY 1'b0 ;
    else
        wclock_nempty_detect <= #`FF_DELAY (rgrey_addr != wgrey_addr) ;
end

wire stretched_empty ;

wire stretched_empty_flop_i = empty && !wclock_nempty_detect ;

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
assign empty_out = empty || stretched_empty ;

//rallow generation
assign rallow = renable_in && !empty && !stretched_empty ; // reads allowed if read enable is high and FIFO is not empty

// rallow output assignment
assign rallow_out = rallow ;

// almost empty output assignment
assign almost_empty_out = almost_empty && !empty && !stretched_empty ;

// at any clock edge that rallow is high, this register provides next read address, so wait cycles are not necessary
// when FIFO is empty, this register provides actual read address, so first location can be read
reg [(ADDR_LENGTH - 1):0] raddr_plus_one ;

// address output mux - when FIFO is not read, current actual address is driven out, when it is read, next address is driven out to provide
// next data immediately
// done for zero wait state burst operation
assign raddr_out = rallow ? raddr_plus_one : raddr ;

always@(posedge rclock_in or posedge clear)
begin
    if (clear)
    begin
        // initial values seem a bit odd - they are this way to allow easier grey pipeline implementation and to allow min fifo size of 8
        raddr_plus_one <= #`FF_DELAY 4 ;
        raddr          <= #`FF_DELAY 3 ;
    end
    else if (flush_in)
    begin
        raddr_plus_one <= #`FF_DELAY waddr + 1'b1 ;
        raddr          <= #`FF_DELAY waddr ;
    end
    else if (rallow)
    begin
        raddr_plus_one <= #`FF_DELAY raddr_plus_one + 1'b1 ;
        raddr          <= #`FF_DELAY raddr_plus_one ;
    end
end

/*-----------------------------------------------------------------------------------------------
Read address control consists of Read address counter and Grey Address pipeline
There are 4 Grey addresses:
    - rgrey_minus1 is Grey Code of address one before current address
    - rgrey_addr is Grey Code of current read address
    - rgrey_next is Grey Code of next read address
--------------------------------------------------------------------------------------------------*/
// grey coded address pipeline for status generation in read clock domain
always@(posedge rclock_in or posedge clear)
begin
    if (clear)
    begin
        rgrey_minus1 <= #`FF_DELAY 0 ;
        rgrey_addr   <= #`FF_DELAY 1 ;
        rgrey_next   <= #`FF_DELAY 3 ; // this grey code is calculated from the current binary address and loaded any time data is read from fifo
    end
    else if (flush_in)
    begin
        // when fifo is flushed, load the register values from the write clock domain.
        // must be no problem, because write pointers are stable for at least 3 clock cycles before flush can occur.
        rgrey_minus1 <= #`FF_DELAY wgrey_minus1 ;
        rgrey_addr   <= #`FF_DELAY wgrey_addr ;
        rgrey_next   <= #`FF_DELAY wgrey_next ;
    end
    else if (rallow)
    begin
        // move the pipeline when data is read from fifo and calculate new value for first stage of pipeline from current binary fifo address
        rgrey_minus1 <= #`FF_DELAY rgrey_addr ;
        rgrey_addr   <= #`FF_DELAY rgrey_next ;
        rgrey_next   <= #`FF_DELAY {raddr[ADDR_LENGTH - 1], calc_rgrey_next} ;
    end
end

/*--------------------------------------------------------------------------------------------
Write address control consists of write address counter and three Grey Code Registers:
    - wgrey_minus1 represents Grey Coded address of location one before current write address
    - wgrey_addr represents current Grey Coded write address
    - wgrey_next represents Grey Coded next write address
----------------------------------------------------------------------------------------------*/
// grey coded address pipeline for status generation in write clock domain
always@(posedge wclock_in or posedge clear)
begin
    if (clear)
    begin
        // initial value is 0
        wgrey_minus1 <= #`FF_DELAY 0 ;
        wgrey_addr   <= #`FF_DELAY 1 ;
        wgrey_next   <= #`FF_DELAY 3 ;
    end
    else
    if (wallow)
    begin
        wgrey_minus1 <= #`FF_DELAY wgrey_addr ;
        wgrey_addr   <= #`FF_DELAY wgrey_next ;
        wgrey_next   <= #`FF_DELAY {waddr[(ADDR_LENGTH - 1)], calc_wgrey_next} ;
    end
end

// write address binary counter - nothing special except initial value
always@(posedge wclock_in or posedge clear)
begin
    if (clear)
        // initial value 3
        waddr <= #`FF_DELAY 3 ;
    else
    if (wallow)
        waddr <= #`FF_DELAY waddr + 1'b1 ;
end

/*------------------------------------------------------------------------------------------------------------------------------
Registered full control:
registered full is set on rising edge of wclock_in, when fifo is almost full and something gets written to it.
It's kept high until something is read from FIFO, which is registered on next rising write clock edge.

Registered almost full control:
Almost full flag is set on rising write clock edge whenever two free locations are left in fifo and another entry is written to it.
It remains set if nothing is read/written from/to fifo. All operations are synchronized on write clock.
--------------------------------------------------------------------------------------------------------------------------------*/
wire comb_full          = wgrey_next == rgrey_addr ;
wire comb_almost_full   = wgrey_next == rgrey_minus1 ;

//combinatorial input to Registered full FlipFlop
wire reg_full = (wallow && comb_almost_full) || (comb_full) ;

meta_flop #(0) i_meta_flop_full
(
    .rst_i      (clear),
    .clk_i      (wclock_in),
    .ld_i       (1'b0),
    .ld_val_i   (1'b0),
    .en_i       (1'b1),
    .d_i        (reg_full),
    .meta_q_o   (full)
) ;

/*------------------------------------------------------------------------------------------------------------------------------
Registered empty control:
registered empty is set on rising edge of rclock_in when one location is occupied and read from it. It remains set until
something is written to fifo which is detected on next read clock edge.

Registered almost empty control:
Almost empty is set on rising clock edge of read clock when two locations are used in fifo and one of them is read from it.
It remains set until something is read/written from/to fifo. All operations are detected on rising edge of read clock.
--------------------------------------------------------------------------------------------------------------------------------*/
wire comb_almost_empty  = rgrey_next == wgrey_addr ;
wire comb_empty         = rgrey_addr == wgrey_addr ;
wire comb_two_used      = rgrey_next == wgrey_minus1 ;

// combinatorial input for registered emty FlipFlop
wire reg_empty = (rallow && comb_almost_empty) || comb_empty ;

// meta flop for empty signal instantiation - reset value 1, load value (flush) 1 etc..
meta_flop #(1) i_meta_flop_empty
(
    .rst_i      (clear),
    .clk_i      (rclock_in),
    .ld_i       (flush_in),
    .ld_val_i   (1'b1),
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
    .ld_i       (flush_in),
    .ld_val_i   (1'b0),
    .en_i       (1'b1),
    .d_i        (reg_almost_empty),
    .meta_q_o   (almost_empty)
) ;

endmodule