//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "wb_bus_mon.v"                                    ////
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
// Revision 1.2  2001/07/19 09:50:08  mihad
// Only delayed write storage is provided after merging delayed requests.
// All delayed reads now pass through FIFO.
//
//


// WISHBONE bus monitor module - it connects to WISHBONE master signals and 
// monitors for any illegal combinations appearing on the bus.
`define WB_DATA_WIDTH 32 // data bus width
`define WB_ADDR_WIDTH 32 // address bus width
`define WB_TAG_I_WIDTH 1 // tag input bus width
`define WB_TAG_O_WIDTH 1 // tag output bus width
`define WB_SEL_WIDTH 4   // select bus width - depends on data bus width

module WB_BUS_MON(
                    CLK_I, 
                    RST_I,
		            ACK_I, 
                    ADDR_O, 
                    CYC_O, 
                    DAT_I, 
                    DAT_O, 
                    ERR_I, 
                    RTY_I, 
                    SEL_O, 
                    STB_O, 
                    WE_O,
                    TAG_I,
                    TAG_O,
                    CAB_O
                  ) ;

input                           CLK_I  ; 
input                           RST_I  ;
input                           ACK_I  ; 
input   [(`WB_ADDR_WIDTH-1):0]  ADDR_O ; 
input                           CYC_O  ; 
input   [(`WB_DATA_WIDTH-1):0]  DAT_I  ; 
input   [(`WB_DATA_WIDTH-1):0]  DAT_O  ; 
input                           ERR_I  ; 
input                           RTY_I  ; 
input   [(`WB_SEL_WIDTH-1):0]   SEL_O  ; 
input                           STB_O  ; 
input                           WE_O   ;
input   [(`WB_TAG_I_WIDTH-1):0] TAG_I  ;
input   [(`WB_TAG_O_WIDTH-1):0] TAG_O  ;
input                           CAB_O  ;

always@(posedge CLK_I)
begin
    if (RST_I)
    begin
        // when reset is applied, all control signals must be low
        if (CYC_O)
            $display("CYC_O active under reset") ;
        if (STB_O)
            $display("STB_O active under reset") ;
        /*if (ACK_I) 
            $display("ACK_I active under reset") ;*/
        if (ERR_I)
            $display("ERR_I active under reset") ;
        if (RTY_I)
            $display("RTY_I active under reset") ;
        if (CAB_O)
            $display("CAB_O active under reset") ;
    end // reset
    else
    if (~CYC_O)
    begin
        // when cycle indicator is low, all control signals must be low
        if (STB_O)
            $display("STB_O active without CYC_O being active") ;
        /*if (ACK_I) 
            $display("ACK_I active without CYC_O being active") ;*/
        if (ERR_I)
            $display("ERR_I active without CYC_O being active") ;
        if (RTY_I)
            $display("RTY_I active without CYC_O being active") ;
        if (CAB_O)
            $display("CAB_O active without CYC_O being active") ;
    end // ~CYC_O
end

// cycle monitor
always@(posedge CLK_I)
begin
    if (CYC_O && ~RST_I) // cycle in progress
    begin
        if (STB_O)
        begin
            // check for two control signals active at same edge
            if ( ACK_I && RTY_I )
                $display("ACK_I and RTY_I asserted at the same time during cycle") ;
            if ( ACK_I && ERR_I )
                $display("ACK_I and ERR_I asserted at the same time during cycle") ;
            if ( RTY_I && ERR_I )
                $display("RTY_I and ERR_I asserted at the same time during cycle") ;
        end // STB_O
        else
        begin //~STB_O
            // while STB_O is inactive, only ACK_I is allowed to be active
            if ( ERR_I )
                $display("ERR_I asserted during cycle without STB_O") ;
            if ( RTY_I )
                $display("RTY_I asserted during cycle without STB_O") ;
        end   // ~STB_O
    end // cycle in progress
end // cycle monitor

// CAB_O monitor - CAB_O musn't change during one cycle
reg [1:0] first_cab_val ;
always@(posedge CLK_I or CYC_O or RST_I)
begin
    if (~CYC_O || RST_I)
        first_cab_val <= 2'b00 ;
    else
        // cycle in progress - is this first clock edge in a cycle ?
        if (first_cab_val[1] == 1'b0) 
            first_cab_val <= {1'b1, CAB_O} ;
        else if ( first_cab_val[0] != CAB_O )
            $display("CAB_O value changed during cycle") ;
end // CAB_O monitor

// WE_O monitor for consecutive address bursts
reg [1:0] first_we_val ;
always@(posedge CLK_I or CYC_O or RST_I or CAB_O)
begin
    if (~CYC_O || ~CAB_O || RST_I)
        first_we_val <= 2'b00 ;
    else 
    if (STB_O)
    begin 
        // cycle in progress - is this first clock edge in a cycle ?
        if (first_we_val[1] == 1'b0) 
            first_we_val <= {1'b1, WE_O} ;
        else if ( first_we_val[0] != WE_O )
            $display("WE_O value changed during CAB cycle") ;
    end
end // CAB_O monitor

// address monitor for consecutive address bursts
reg [`WB_ADDR_WIDTH:0] address ;
always@(posedge CLK_I or CYC_O or RST_I or CAB_O)
begin
    if (~CYC_O || ~CAB_O || RST_I)
        address <= {(`WB_ADDR_WIDTH + 1){1'b0}} ;
    else
    begin
        if (STB_O && ACK_I)
        begin
            if (address[`WB_ADDR_WIDTH] == 1'b0)
                address <= {1'b1, (ADDR_O + `WB_SEL_WIDTH)} ;
            else
            begin
                if ( address[(`WB_ADDR_WIDTH-1):0] != ADDR_O)
                    $display("Consecutive address burst address incrementing incorrect") ;
                else
                    address <= {1'b1, (ADDR_O + `WB_SEL_WIDTH)} ;
            end
        end
    end
end // address monitor

// data monitor
always@(posedge CLK_I)
begin
    if (CYC_O && STB_O && ~RST_I)
    begin
        if ( (ADDR_O & 32'hFFFFFFFF) !== ADDR_O )
        begin
            $display("Master provided invalid address and qualified it with STB_O") ;
        end
        if ( WE_O )
        begin
            if ( 
                (SEL_O[0] && (( DAT_O[7:0]   & 8'hFF ) !== DAT_O[7:0]   )) ||
                (SEL_O[1] && (( DAT_O[15:8]  & 8'hFF ) !== DAT_O[15:8]  )) ||
                (SEL_O[2] && (( DAT_O[23:16] & 8'hFF ) !== DAT_O[23:16] )) ||
                (SEL_O[3] && (( DAT_O[31:24] & 8'hFF ) !== DAT_O[31:24] ))
               )

                $display("Master provided invalid data during write and qualified it with STB_O") ;
        end
        else
        if (~WE_O && ACK_I)
        begin
            if ( 
                (SEL_O[0] && (( DAT_I[7:0]   & 8'hFF ) !== DAT_I[7:0]   )) ||
                (SEL_O[1] && (( DAT_I[15:8]  & 8'hFF ) !== DAT_I[15:8]  )) ||
                (SEL_O[2] && (( DAT_I[23:16] & 8'hFF ) !== DAT_I[23:16] )) ||
                (SEL_O[3] && (( DAT_I[31:24] & 8'hFF ) !== DAT_I[31:24] ))
               )
                $display("Slave provided invalid data during read and qualified it with ACK_I") ;
        end
    end
end

endmodule // BUS_MON