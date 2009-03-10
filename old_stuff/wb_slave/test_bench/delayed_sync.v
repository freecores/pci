//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "delayed_sync.v"                                  ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - mihad@opencores.org                                   ////
////      - Miha Dolenc                                           ////
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
// Revision 1.2  2001/07/19 09:52:33  mihad
// Only delayed write storage is provided after merging delayed requests.
// All delayed reads now pass through FIFO.
//
//

// module provides synchronization mechanism between requesting and completing side of the bridge
`include "constants.v"
`include "bus_commands.v"
module DELAYED_SYNC
(
    reset_in,
    req_clk_in,
    comp_clk_in,
    req_in,
    comp_in,
    done_in,
    in_progress_in,
    comp_req_pending_out,
    req_req_pending_out,
    req_comp_pending_out,
    addr_in,
    be_in, 
    addr_out,
    be_out, 
    we_in,
    we_out,
    bc_in,
    bc_out,
    status_in,
    status_out,
    comp_flush_out,
    burst_in,
    burst_out,
    retry_expired_in
);

// system inputs
input reset_in,         // reset input
      req_clk_in,       // requesting clock input
      comp_clk_in ;     // completing clock input

// request, completion, done and in progress indicator inputs
input req_in,           // request qualifier - when 1 it indicates that valid request data is provided on inputs
      comp_in,          // completion qualifier - when 1, completing side indicates that request has completed
      done_in,          // done input - when 1 indicates that requesting side of the bridge has completed a cycle on requesting bus
      in_progress_in ;  // in progress indicator - indicates that current completion is in progress on requesting side of the bridge

// pending indication outputs - completing side only needs to know about requests, while requesting side must know about requests and completions
output  comp_req_pending_out,   // completion side request output - resynchronized from requesting clock to completing clock
        req_req_pending_out,    // request pending output for requesting side
        req_comp_pending_out ;  // completion pending output for requesting side of the bridge - it indicates when completion is ready for completing on requesting bus

// inputs from requesting side - only this side can set address, bus command, byte enables, write enable and burst - outputs are common for both sides
// all signals that identify requests are stored in this module

input [31:0]    addr_in ;   // address bus input
input [3:0]     be_in ;     // byte enable input
input           we_in ;     // write enable input - read/write request indication 1 = write request / 0 = read request
input [3:0]     bc_in ;     // bus command input
input           burst_in ;  // burst indicator    - qualifies operation as burst/single transfer 1 = burst / 0 = single transfer

// common request outputs used both by completing and requesting sides
// this outputs are not resynchronized, since flags determine the request status
output [31:0]   addr_out ;
output [3:0]    be_out ;
output          we_out ;
output [3:0]    bc_out ;
output          burst_out ;

// completion side signals encoded termination status - 0 = normal completion / 1 = error terminated completion
input          status_in ;  
output         status_out ;

// input signals that delayed transaction has been retried for max number of times
// on this signal request is ditched, otherwise it would cause a deadlock
// requestor can issue another request and procedure will be repeated 
input   retry_expired_in ;

// completion flush output - if in 2^^16 clock cycles transaction is not repeated by requesting agent - flush completion data
output  comp_flush_out ;

// flip flop for registering retry_expired
reg     comp_retry_expired ;

// clear of request side request pending flag must be synchronized to request clock
// two FFs are used for this purpose - one is trigered on rising and one on falling edge of request clock
reg     rty_exp_sync_pos, rty_exp_sync_neg ;

// negedge triggered synchronization flip flop
always@(negedge req_clk_in or posedge reset_in)
begin
    if (reset_in)
        rty_exp_sync_neg <= #`FF_DELAY 1'b1 ;
    else
        rty_exp_sync_neg <= #`FF_DELAY comp_retry_expired ;
end

// posedge triggered synchronization flip flop
always@(posedge req_clk_in or posedge reset_in)
begin
    if (reset_in)
        rty_exp_sync_pos <= #`FF_DELAY 1'b1 ;
    else
        rty_exp_sync_pos <= #`FF_DELAY rty_exp_sync_neg ;
end

// wire for clearing retry expired flip flop
wire retry_expired_clear = rty_exp_sync_neg && rty_exp_sync_pos ;

always@(posedge comp_clk_in or posedge retry_expired_clear)
begin
    if (retry_expired_clear)
        comp_retry_expired <= #`FF_DELAY 1'b0 ;
    else
    if (retry_expired_in)
        comp_retry_expired <= #`FF_DELAY 1'b1 ;
end

// output registers for common signals
reg [31:0]   addr_out ;
reg [3:0]    be_out ;
reg          we_out ;
reg [3:0]    bc_out ;
reg          burst_out ;

// registers for storing delayed transaction information
always@(posedge req_clk_in or posedge reset_in)
begin
    if (reset_in)
    begin
        addr_out  <= #`FF_DELAY 32'h0000_0000 ;
        be_out    <= #`FF_DELAY 4'h0 ;
        we_out    <= #`FF_DELAY 1'b0 ;
        bc_out    <= #`FF_DELAY `BC_RESERVED0 ;
        burst_out <= #`FF_DELAY 1'b0 ;
    end
    else
        if (req_in)
        begin
            addr_out  <= #`FF_DELAY addr_in ;
            be_out    <= #`FF_DELAY be_in ;
            we_out    <= #`FF_DELAY we_in ;
            bc_out    <= #`FF_DELAY bc_in ;
            burst_out <= #`FF_DELAY burst_in ;
        end
end

// flip flop for passing completion flag to requesting side
reg req_comp_pending ;

// flip-flop for receiving requests from requesting side - synchronized on requesting clock
reg req_req_pending ;

// wire for clearing request pending FF
// request side request flag flip flop is cleared at reset, when completion is pending or when retry counter expired signal is set
// request flag is set when request input is asserted
wire req_req_clear = reset_in || req_comp_pending || retry_expired_clear;

always@(posedge req_clk_in or posedge req_req_clear)
begin
    if (req_req_clear)
        req_req_pending <= #`FF_DELAY 1'b0 ;
    else
    if (req_in)
        req_req_pending <= #`FF_DELAY 1'b1 ;
end

// output assignement
assign req_req_pending_out = req_req_pending ;

// flip flop for completion receiving - synchronized to completion clock
reg comp_comp_pending ;

// flip-flop for synchronizing received request to completion clock
reg comp_req_pending ;

// wire for clearing completion side request pending flip - flop
// completion side request pending flip flop is cleared at reset or when completion is pending
wire comp_req_clear = reset_in || comp_comp_pending || comp_retry_expired || retry_expired_clear ;

always@(posedge comp_clk_in or posedge comp_req_clear)
begin
    if (comp_req_clear)
        comp_req_pending <= #`FF_DELAY 1'b0 ;
    else
        comp_req_pending <= #`FF_DELAY req_req_pending ;
end

// output assignement
assign comp_req_pending_out = comp_req_pending ;

// FF for completion pending flag clear
// completion flag is set on rising edge of requesting clock when completion side completion pending
// flag is set. Both flags are cleared when done is signaled or cycle counter has expired. Clear is set for one requesting clock cycle.
reg comp_clear ;

always@(posedge comp_clk_in or posedge comp_clear)
begin
    if (comp_clear)
        comp_comp_pending <= #`FF_DELAY 1'b0 ;
    else
    if (comp_in)
        comp_comp_pending <= #`FF_DELAY 1'b1 ;
end

// clocks counter - it counts how many clock edges completion is present without beeing repeated
// if it counts to 2^^16 cycles the completion must be ditched
reg [16:0] comp_cycle_count ;

always@(posedge req_clk_in or posedge reset_in)
begin   
    if (reset_in)
        comp_clear <= #`FF_DELAY 1'b1 ; // at reset this FF is set and is cleared on first requesting clock edge after rese
    else
        comp_clear <= #`FF_DELAY done_in || comp_cycle_count[16] ;
end

always@(posedge req_clk_in or posedge comp_clear)
begin
    if (comp_clear)
        req_comp_pending <= #`FF_DELAY 1'b0 ;
    else
        req_comp_pending <= #`FF_DELAY comp_comp_pending ;
end

// output assignement
assign req_comp_pending_out = req_comp_pending ;

// completion status flip flop - if 0 when completion is signalled it's finished OK otherwise it means error
reg status_out ;
always@(posedge comp_clk_in or posedge reset_in) 
begin
    if (reset_in)
        status_out <= #`FF_DELAY 1'b0 ;
    else
    if (comp_in)
        status_out <= #`FF_DELAY status_in ;
end

// wire for clearing this counter
wire clear_count = in_progress_in || comp_clear ;
always@(posedge req_clk_in or posedge clear_count)
begin
    if (clear_count)
        comp_cycle_count <= #`FF_DELAY 17'h0_0000 ;
    else
    if (req_comp_pending)
        comp_cycle_count <= #`FF_DELAY comp_cycle_count + 1'b1 ;
end

// completion flush output - used for flushing fifos when counter expires
// if counter doesn't expire, fifo flush is up to WISHBONE slave or PCI target state machines
// if they detect that FIFO was emptied by normal reads, flush is not necesarry
// (avoids counter resets etc.)
reg comp_flush_out ;
always@(posedge req_clk_in or posedge reset_in)
begin
    if (reset_in)
        comp_flush_out <= #`FF_DELAY 1'b0 ;
    else
        comp_flush_out <= #`FF_DELAY comp_cycle_count[16] ;
end

endmodule //delayed_sync
