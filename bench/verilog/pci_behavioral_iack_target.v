//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "pci_behavioral_iack_target"                      ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - Miha Dolenc (mihad@opencores.org)                     ////
////                                                              ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000 Miha Dolenc, mihad@opencores.org          ////
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
// Revision 1.1  2002/02/01 15:07:51  mihad
// *** empty log message ***
//

`include "pci_constants.v"
`include "timescale.v"
`include "bus_commands.v"

// module is provided just as target for responding to interrupt acknowledge commands, because
// other models don't support this command
module PCI_BEHAVIORAL_IACK_TARGET
(
    CLK,
    AD,
    CBE,
    RST,
    FRAME,
    IRDY,
    DEVSEL,
    TRDY,
    STOP,
    PAR,
    respond,
    interrupt_vector
);

input CLK ;
output [31:0] AD ;
reg    [31:0] AD ;
input  [3:0] CBE ;
input  RST ;
input  FRAME ;
input  IRDY ;
output DEVSEL ;
reg    DEVSEL ;
output TRDY ;
reg    TRDY ;
output STOP ;
reg    STOP ;
output PAR ;
reg    PAR ;
input  respond ;
input [31:0] interrupt_vector ;

reg frame_prev ;
reg [3:0] cbe_prev ;

always@(posedge CLK or negedge RST)
begin
    if ( !RST )
    begin
        frame_prev <= #`FF_DELAY 1'b1 ;
        cbe_prev   <= #`FF_DELAY 4'hF ;
        AD         <= #`FF_DELAY 32'hzzzz_zzzz ;
        DEVSEL     <= #`FF_DELAY 1'bz ;
        TRDY       <= #`FF_DELAY 1'bz ;
        STOP       <= #`FF_DELAY 1'bz ;
        PAR        <= #`FF_DELAY 1'bz ;
    end
    else
    begin
        frame_prev <= #`FF_DELAY FRAME ;
        cbe_prev   <= #`FF_DELAY CBE ;
    end
end

always@(posedge CLK)
begin
    if ( RST )
    begin
        if ( (frame_prev === 1) && (FRAME === 0) && (CBE === `BC_IACK) && (respond === 1) )
            do_reference ;
    end
end

task do_reference ;
begin
    // do medium decode
    @(posedge CLK) ;
    DEVSEL <= #`FF_DELAY 1'b0 ;
    TRDY   <= #`FF_DELAY 1'b0 ;
    STOP   <= #`FF_DELAY 1'b0 ;

    if ( CBE[3] === 0 )
        AD[31:24] <= #`FF_DELAY interrupt_vector[31:24] ;
    else
        AD[31:24] <= #`FF_DELAY 0 ;

    if ( CBE[2] === 0 )
        AD[23:16] <= #`FF_DELAY interrupt_vector[23:16] ;
    else
        AD[23:16] <= #`FF_DELAY 0 ;

    if ( CBE[1] === 0 )
        AD[15:8] <= #`FF_DELAY interrupt_vector[15:8] ;
    else
        AD[15:8] <= #`FF_DELAY 0 ;

    if ( CBE[0] === 0 )
        AD[7:0] <= #`FF_DELAY interrupt_vector[7:0] ;
    else
        AD[7:0] <= #`FF_DELAY 0 ;

    @(posedge CLK) ;
    PAR <= #`FF_DELAY ^( {AD, CBE} ) ;

    if ( FRAME !== 1 )
    begin
        while ( FRAME !== 1 )
        begin
            @(posedge CLK) ;
            PAR <= #`FF_DELAY ^( {AD, CBE} ) ;
        end
    end

    AD         <= #`FF_DELAY 32'hzzzz_zzzz ;
    DEVSEL     <= #`FF_DELAY 1'b1 ;
    TRDY       <= #`FF_DELAY 1'b1 ;
    STOP       <= #`FF_DELAY 1'b1 ;

    @(posedge CLK) ;
    DEVSEL     <= #`FF_DELAY 1'bz ;
    TRDY       <= #`FF_DELAY 1'bz ;
    STOP       <= #`FF_DELAY 1'bz ;
    PAR        <= #`FF_DELAY 1'bz ;
end
endtask // do reference

endmodule
