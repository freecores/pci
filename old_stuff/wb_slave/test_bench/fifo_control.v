//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "fifo_control.v"                                  ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - mihad@opencores.org                                   ////
////      - Miha Dolenc                                           ////
////                                                              ////
////  All additional information is avaliable in the README.pdf   ////
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
// Revision 1.3  2001/07/30 11:09:37  mihad
// no message
//
//

/* FIFO_CONTROL module provides read/write address and status generation for
   FIFOs implemented with standard dual port SRAM cells in ASIC or FPGA designs */
`include "constants.v"
`ifdef FPGA
    // fifo design in FPGA will be synchronous
    `ifdef SYNCHRONOUS
    `else
        `define SYNCHRONOUS
    `endif
`endif
module FIFO_CONTROL(rclock_in, wclock_in, renable_in, 
                    wenable_in, reset_in, flush_in, 
                    almost_full_out, full_out, 
                    almost_empty_out, empty_out, 
                    waddr_out, raddr_out, rallow_out, wallow_out);

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

// almost full and empy status outputs
output almost_full_out, almost_empty_out;

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
reg [(ADDR_LENGTH - 1):0] wgrey_addr ; // current
reg [(ADDR_LENGTH - 1):0] wgrey_next ; // next

// next write gray address calculation - bitwise xor between address and shifted address
wire [(ADDR_LENGTH - 2):0] calc_wgrey_next  = waddr[(ADDR_LENGTH - 1):1] ^ waddr[(ADDR_LENGTH - 2):0] ;

// grey code pipeline for read address
reg [(ADDR_LENGTH - 1):0] rgrey_minus1 ; // one before current
reg [(ADDR_LENGTH - 1):0] rgrey_addr ; // current
reg [(ADDR_LENGTH - 1):0] rgrey_next ; // next

// next read gray address calculation - bitwise xor between address and shifted address
wire [(ADDR_LENGTH - 2):0] calc_rgrey_next  = raddr[(ADDR_LENGTH - 1):1] ^ raddr[(ADDR_LENGTH - 2):0] ;

// FFs for registered empty and full flags
reg empty ;
reg full ;

// almost_empty and almost_full tag - implemented as latches
reg almost_empty ;
reg almost_full ;

// write allow wire - writes are allowed when fifo is not full
wire wallow = wenable_in && ~full ;

// write allow output assignment
assign wallow_out = wallow ;

// read allow wire
wire rallow ;

// full output assignment
assign full_out  = full ;

// almost full output assignment
assign almost_full_out  = almost_full && ~full ;

// clear generation for FFs and registers
wire clear = reset_in || flush_in ;

`ifdef SYNCHRONOUS
    
    reg wclock_nempty_detect ;
    always@(posedge reset_in or posedge wclock_in)
    begin
        if (reset_in)
            wclock_nempty_detect <= #`FF_DELAY 1'b0 ;
        else
            wclock_nempty_detect <= #`FF_DELAY (rgrey_addr != wgrey_addr) ;
    end

    // special synchronizing mechanism for different implementations - in synchronous imp., empty is prolonged for 1 clock edge if no write clock comes after initial write
    reg stretched_empty ;
    always@(posedge rclock_in or posedge clear)
    begin
        if(clear)
            stretched_empty <= #`FF_DELAY 1'b1 ;
        else
            stretched_empty <= #`FF_DELAY empty && ~wclock_nempty_detect ;
    end
    
    // empty output is actual empty + 1 read clock cycle ( stretched empty )
    assign empty_out = empty  || stretched_empty ;

    //rallow generation    
    assign rallow = renable_in && ~empty && ~stretched_empty ; // reads allowed if read enable is high and FIFO is not empty

    // rallow output assignment
    assign rallow_out = rallow ; 

    // almost empty output assignment
    assign almost_empty_out = almost_empty && ~empty && ~stretched_empty ;

    // at any clock edge that rallow is high, this register provides next read address, so wait cycles are not necessary
    // when FIFO is empty, this register provides actual read address, so first location can be read
    reg [(ADDR_LENGTH - 1):0] raddr_plus_one ;

    // address output mux - when FIFO is empty, current actual address is driven out, when it is non - empty next address is driven out
    // done for zero wait state burst
    assign raddr_out = empty_out ? raddr : raddr_plus_one ;

    // enable for this register
    wire raddr_plus_one_en = rallow ; 
    always@(posedge rclock_in or posedge clear)
    begin
        if (clear)
        begin
            raddr_plus_one[(ADDR_LENGTH - 1):1] <= #`FF_DELAY { (ADDR_LENGTH - 1){1'b0}} ;
            raddr_plus_one[0] <= #`FF_DELAY 1'b1 ;
        end
        else if (raddr_plus_one_en)
            raddr_plus_one <= #`FF_DELAY raddr_plus_one + 1'b1 ;
    end
    
    // raddr is filled with raddr_plus_one on rising read clock edge when rallow is high
    always@(posedge rclock_in or posedge clear)
    begin
	    if (clear)
            // initial value is 000......00
		    raddr <= #`FF_DELAY { ADDR_LENGTH{1'b0}} ;
	    else if (rallow)
	    	raddr <= #`FF_DELAY raddr_plus_one ;
    end

`else            
    // asynchronous RAM storage for FIFOs - somewhat simpler control logic
    //rallow generation    
    assign rallow = renable_in && ~empty ;

    assign rallow_out = rallow;

    assign almost_empty_out = almost_empty && ~empty ;
    
    // read address counter - normal counter, nothing to it
    // for asynchronous implementation, there is no need for pointing to next address.
    // On clock edge that read is performed, read address will change and on the next clock edge
    // asynchronous memory will provide next data
    always@(posedge rclock_in or posedge clear)
    begin
	    if (clear)
            // initial value is 000......00
		    raddr <= #`FF_DELAY { ADDR_LENGTH{1'b0}} ;
	    else if (rallow)
		    raddr <= #`FF_DELAY raddr + 1'b1 ;
    end

    assign empty_out = empty ;
    assign raddr_out = raddr ;
`endif

/*-----------------------------------------------------------------------------------------------
Read address control consists of Read address counter and Grey Address pipeline
There are 3 Grey addresses: 
    - rgrey_minus1 is Grey Code of address one before current address
    - rgrey_addr is Grey Code of current read address
    - rgrey_next is Grey Code of next read address
--------------------------------------------------------------------------------------------------*/

// grey code register for one before read address
always@(posedge rclock_in or posedge clear)
begin
	if (clear)
    begin
        // initial value is 100......011
		rgrey_minus1[(ADDR_LENGTH - 1)] <= #`FF_DELAY 1'b1 ; 
        rgrey_minus1[(ADDR_LENGTH  - 2):2] <= #`FF_DELAY { (ADDR_LENGTH  - 3){1'b0} } ;
        rgrey_minus1[1:0] <= #`FF_DELAY 2'b11 ;
    end
	else
		if (rallow)
			rgrey_minus1 <= #`FF_DELAY rgrey_addr ;
end

// grey code register for read address - represents current Read Address
always@(posedge rclock_in or posedge clear)
begin
	if (clear)
    begin
        // initial value is 100.......01
		rgrey_addr[(ADDR_LENGTH - 1)] <= #`FF_DELAY 1'b1 ;
        rgrey_addr[(ADDR_LENGTH - 2):1] <= #`FF_DELAY { (ADDR_LENGTH - 2){1'b0} } ;
        rgrey_addr[0] <= #`FF_DELAY 1'b1 ;
    end
	else
		if (rallow)
			rgrey_addr <= #`FF_DELAY rgrey_next ;
end

// grey code register for next read address - represents Grey Code of next read address    
always@(posedge rclock_in or posedge clear)
begin
	if (clear)
    begin
        // initial value is 100......00
		rgrey_next[(ADDR_LENGTH - 1)] <= #`FF_DELAY 1'b1 ;
        rgrey_next[(ADDR_LENGTH - 2):0] <= #`FF_DELAY { (ADDR_LENGTH - 1){1'b0} } ;
    end
	else
		if (rallow)
            rgrey_next <= #`FF_DELAY {raddr[ADDR_LENGTH - 1], calc_rgrey_next} ;
end

/*--------------------------------------------------------------------------------------------
Write address control consists of write address counter and two Grey Code Registers:
    - wgrey_addr represents current Grey Coded write address
    - wgrey_next represents Grey Coded next write address
----------------------------------------------------------------------------------------------*/
// grey code register for write address
always@(posedge wclock_in or posedge clear)
begin
	if (clear)
    begin
        // initial value is 100.....001
        wgrey_addr[(ADDR_LENGTH - 1)] <= #`FF_DELAY 1'b1 ;
        wgrey_addr[(ADDR_LENGTH - 2):1] <= #`FF_DELAY { (ADDR_LENGTH - 2){1'b0} } ;
        wgrey_addr[0] <= #`FF_DELAY 1'b1 ;
    end
	else
		if (wallow)
			wgrey_addr <= #`FF_DELAY wgrey_next ;
end

// grey code register for next write address
always@(posedge wclock_in or posedge clear)
begin
	if (clear)
    begin
        // initial value is 100......00
		wgrey_next[(ADDR_LENGTH - 1)] <= #`FF_DELAY 1'b1 ;
        wgrey_next[(ADDR_LENGTH - 2):0] <= #`FF_DELAY { (ADDR_LENGTH - 1){1'b0} } ;
    end
	else
        if (wallow)
            wgrey_next <= #`FF_DELAY {waddr[(ADDR_LENGTH - 1)], calc_wgrey_next} ;
end

// write address counter - nothing special
always@(posedge wclock_in or posedge clear)
begin
	if (clear)
        // initial value 00.........00
		waddr <= #`FF_DELAY { (ADDR_LENGTH){1'b0} } ;
	else
		if (wallow)
			waddr <= #`FF_DELAY waddr + 1'b1 ;
end

/*------------------------------------------------------------------------------------------------------------------------------
Registered full control:
registered full is set on rising edge of wclock_in, when almost full and wallow are high. 
It's kept high until something is read from FIFO, which is registered on
next rising write clock edge.

Latched almost full control:
Almost full flag is latched whenever writes are not allowed ( nothing is beeing written to the FIFO ) or when
write clock is high - that way almost full can still go low if there is half a cycle left between a read and write operation
--------------------------------------------------------------------------------------------------------------------------------*/
//combinatorial input to Registered full FlipFlop
wire reg_full = (wallow && almost_full) || (wgrey_next == rgrey_addr) ;

always@(posedge wclock_in or posedge clear)
begin
	if (clear)
		full <= #`FF_DELAY 1'b0 ;
	else
		full <= #`FF_DELAY reg_full ;
end

// input for almost full latch
wire latch_almost_full_in = (wgrey_next == rgrey_minus1) ;

always@(clear or latch_almost_full_in or wallow or wclock_in)
begin
    if (clear)
        almost_full <= #`FF_DELAY 1'b0 ;
    else
    if ( wclock_in || ~wallow )
        almost_full <= #`FF_DELAY latch_almost_full_in ;
end

/*------------------------------------------------------------------------------------------------------------------------------
Registered empty control:
registered empty is set on rising edge of rclock_in, 
when almost empty is high, and rallow is high. It's kept high until something is written to FIFO, which is registered on
the next read clock.

Latched almost empty control:
Almost empty is latched whenever reads are not allowed or read clock is high.
This way almost empty can still go down if there is at least half of read clock cycle left between write and next read
--------------------------------------------------------------------------------------------------------------------------------*/
// combinatorial input for registered emty FlipFlop
wire reg_empty = (rallow && almost_empty) || (rgrey_addr == wgrey_addr) ;

always@(posedge rclock_in or posedge clear)
begin
    if (clear)
        empty <= #`FF_DELAY 1'b1 ;
	else
        empty <= #`FF_DELAY reg_empty ;
end

// input for almost empty latch
wire latch_almost_empty = (rgrey_next == wgrey_addr) ;
always@(clear or latch_almost_empty or rclock_in or rallow)
begin
    if (clear)
        almost_empty <= #`FF_DELAY 1'b0 ;
    else
        if (rclock_in || ~rallow)
            almost_empty <= #`FF_DELAY latch_almost_empty ;
end

endmodule