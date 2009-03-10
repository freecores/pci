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
// Revision 1.1  2001/08/06 18:12:58  mihad
// Pocasi delamo kompletno zadevo
//
//

`include "pci_testbench_defines.v"
// WISHBONE bus monitor module - it connects to WISHBONE master signals and
// monitors for any illegal combinations appearing on the bus.
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
                    CAB_O,
                    log_file_desc
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
input   [(`WB_TAG_WIDTH-1):0] TAG_I  ;
input   [(`WB_TAG_WIDTH-1):0] TAG_O  ;
input                           CAB_O  ;
input [31:0] log_file_desc ;

always@(posedge CLK_I or posedge RST_I)
begin
    if (RST_I)
    begin
        // when reset is applied, all control signals must be low
        if (CYC_O)
        begin
            $display("CYC_O active under reset") ;
            $fdisplay(log_file_desc, "CYC_O active under reset") ;
        end
        if (STB_O)
        begin
            $display("STB_O active under reset") ;
            $fdisplay(log_file_desc, "STB_O active under reset") ;
        end
        /*if (ACK_I)
            $display("ACK_I active under reset") ;*/
        if (ERR_I)
        begin
            $display("ERR_I active under reset") ;
            $fdisplay(log_file_desc, "ERR_I active under reset") ;
        end
        if (RTY_I)
        begin
            $display("RTY_I active under reset") ;
            $fdisplay(log_file_desc, "RTY_I active under reset") ;
        end
        if (CAB_O)
        begin
            $display("CAB_O active under reset") ;
            $fdisplay(log_file_desc, "CAB_O active under reset") ;
        end
    end // reset
    else
    if (~CYC_O)
    begin
        // when cycle indicator is low, all control signals must be low
        if (STB_O)
        begin
            $display("STB_O active without CYC_O being active") ;
            $fdisplay(log_file_desc, "STB_O active without CYC_O being active") ;
        end
        if (ACK_I)
        begin
            $display("ACK_I active without CYC_O being active") ;
            $fdisplay(log_file_desc, "ACK_I active without CYC_O being active") ;
        end
        if (ERR_I)
        begin
            $display("ERR_I active without CYC_O being active") ;
            $fdisplay(log_file_desc, "ERR_I active without CYC_O being active") ;
        end
        if (RTY_I)
        begin
            $display("RTY_I active without CYC_O being active") ;
            $fdisplay(log_file_desc, "RTY_I active without CYC_O being active") ;
        end
        if (CAB_O)
        begin
            $display("CAB_O active without CYC_O being active") ;
            $fdisplay(log_file_desc, "CAB_O active without CYC_O being active") ;
        end
    end // ~CYC_O
end

reg [`WB_DATA_WIDTH-1:0] previous_data ;
reg [`WB_ADDR_WIDTH-1:0] previous_address ;
reg [`WB_SEL_WIDTH-1:0] previous_sel ;
reg can_change ;

// cycle monitor
always@(posedge CLK_I or posedge RST_I)
begin
    if (CYC_O && ~RST_I) // cycle in progress
    begin
        if (STB_O)
        begin
            // check for two control signals active at same edge
            if ( ACK_I && RTY_I )
            begin
                $display("ACK_I and RTY_I asserted at the same time during cycle") ;
                $fdisplay(log_file_desc, "ACK_I and RTY_I asserted at the same time during cycle") ;
            end
            if ( ACK_I && ERR_I )
            begin
                $display("ACK_I and ERR_I asserted at the same time during cycle") ;
                $fdisplay(log_file_desc, "ACK_I and ERR_I asserted at the same time during cycle") ;
            end
            if ( RTY_I && ERR_I )
            begin
                $display("RTY_I and ERR_I asserted at the same time during cycle") ;
                $fdisplay(log_file_desc, "RTY_I and ERR_I asserted at the same time during cycle") ;
            end

            if ( can_change !== 1 )
            begin
                if ( ADDR_O !== previous_address )
                begin
                    $display("WB bus monitor detected address change in the middle of the cycle!") ;
                    $fdisplay(log_file_desc, "WB bus monitor detected address change in the middle of the cycle!") ;
                end

                if ( SEL_O !== previous_sel )
                begin
                    $display("WB bus monitor detected select lines changed in the middle of the cycle!") ;
                    $fdisplay(log_file_desc, "WB bus monitor detected select lines changed in the middle of the cycle!") ;
                end

                if ( (WE_O !== 0) && ( DAT_O !== previous_data ) )
                begin
                    $display("WB bus monitor detected data lines changed in the middle of the cycle!") ;
                    $fdisplay(log_file_desc, "WB bus monitor detected data lines changed in the middle of the cycle!") ;
                end
            end

            if ( ACK_I || RTY_I || ERR_I )
                can_change       = 1 ;
            else
            begin
                previous_data    = DAT_O ;
                previous_address = ADDR_O ;
                previous_sel     = SEL_O ;
                can_change = 0 ;
            end

        end // STB_O
        else
        begin //~STB_O
            // while STB_O is inactive, only ACK_I is allowed to be active
            if ( ERR_I )
            begin
                $display("ERR_I asserted during cycle without STB_O") ;
                $fdisplay(log_file_desc, "ERR_I asserted during cycle without STB_O") ;
            end
            if ( RTY_I )
            begin
                $display("RTY_I asserted during cycle without STB_O") ;
                $fdisplay(log_file_desc, "RTY_I asserted during cycle without STB_O") ;
            end

            can_change = 1 ;
        end   // ~STB_O
    end // cycle in progress
end // cycle monitor

// CAB_O monitor - CAB_O musn't change during one cycle
reg [1:0] first_cab_val ;
always@(posedge CLK_I or RST_I)
begin
    if ((CYC_O === 0) || RST_I)
        first_cab_val <= 2'b00 ;
    else
    begin
        // cycle in progress - is this first clock edge in a cycle ?
        if (first_cab_val[1] === 1'b0)
            first_cab_val <= {1'b1, CAB_O} ;
        else if ( first_cab_val[0] !== CAB_O )
        begin
            $display("CAB_O value changed during cycle") ;
            $fdisplay(log_file_desc, "CAB_O value changed during cycle") ;
        end
    end
end // CAB_O monitor

// WE_O monitor for consecutive address bursts
reg [1:0] first_we_val ;
always@(posedge CLK_I or posedge RST_I)
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
        begin
            $display("WE_O value changed during CAB cycle") ;
            $fdisplay(log_file_desc, "WE_O value changed during CAB cycle") ;
        end
    end
end // CAB_O monitor

// address monitor for consecutive address bursts
reg [`WB_ADDR_WIDTH:0] address ;
always@(posedge CLK_I or posedge RST_I)
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
                begin
                    $display("Consecutive address burst address incrementing incorrect") ;
                    $fdisplay(log_file_desc, "Consecutive address burst address incrementing incorrect") ;
                end
                else
                    address <= {1'b1, (ADDR_O + `WB_SEL_WIDTH)} ;
            end
        end
    end
end // address monitor

// data monitor
always@(posedge CLK_I or posedge RST_I)
begin
    if (CYC_O && STB_O && ~RST_I)
    begin
        if ( ((^ADDR_O) !== 1'b1) && ((^ADDR_O) !== 1'b0) )
        begin
            $display("Master provided invalid address and qualified it with STB_O") ;
            $fdisplay(log_file_desc, "Master provided invalid address and qualified it with STB_O") ;
        end
        if ( WE_O )
        begin
            if (
                (SEL_O[0] && (((^DAT_O[7:0])   !== 1'b0) && ((^DAT_O[7:0])   !== 1'b1))) ||
                (SEL_O[1] && (((^DAT_O[15:8])  !== 1'b0) && ((^DAT_O[15:8])  !== 1'b1))) ||
                (SEL_O[2] && (((^DAT_O[23:16]) !== 1'b0) && ((^DAT_O[23:16]) !== 1'b1))) ||
                (SEL_O[3] && (((^DAT_O[31:24]) !== 1'b0) && ((^DAT_O[31:24]) !== 1'b1)))
               )
            begin
                $display("Master provided invalid data during write and qualified it with STB_O") ;
                $fdisplay(log_file_desc, "Master provided invalid data during write and qualified it with STB_O") ;
                $display("Byte select value: SEL_O = %b, Data bus value: DAT_O =  %h ", SEL_O, DAT_O) ;
                $fdisplay(log_file_desc, "Byte select value: SEL_O = %b, Data bus value: DAT_O =  %h ", SEL_O, DAT_O) ;
            end

        end
        else
        if (~WE_O && ACK_I)
        begin
            if (
                (SEL_O[0] && (((^DAT_I[7:0])   !== 1'b0) && ((^DAT_I[7:0])   !== 1'b1))) ||
                (SEL_O[1] && (((^DAT_I[15:8])  !== 1'b0) && ((^DAT_I[15:8])  !== 1'b1))) ||
                (SEL_O[2] && (((^DAT_I[23:16]) !== 1'b0) && ((^DAT_I[23:16]) !== 1'b1))) ||
                (SEL_O[3] && (((^DAT_I[31:24]) !== 1'b0) && ((^DAT_I[31:24]) !== 1'b1)))
               )
            begin
                $display("Slave provided invalid data during read and qualified it with ACK_I") ;
                $fdisplay(log_file_desc, "Slave provided invalid data during read and qualified it with ACK_I") ;
                $display("Byte select value: SEL_O = %b, Data bus value: DAT_I =  %h ", SEL_O, DAT_I) ;
                $fdisplay(log_file_desc, "Byte select value: SEL_O = %b, Data bus value: DAT_I =  %h ", SEL_O, DAT_I) ;
            end
        end
    end
end

initial
begin
    previous_data = 0 ;
    previous_address = 0 ;
    can_change = 1 ;
end
endmodule // BUS_MON
