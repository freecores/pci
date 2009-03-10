//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "dp_async_ram.v"                                  ////
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
// Revision 1.2  2001/07/19 09:50:58  mihad
// Only delayed write storage is provided after merging delayed requests.
// All delayed reads now pass through FIFO.
//
//

// behavioral dual port asynchronous read / synchronous write RAM
module DP_ASYNC_RAM(reset_in, wclock_in, data_in, raddr_in, waddr_in, data_out, wenable_in);

parameter ADDR_LENGTH = 7 ;
parameter SIZE = 128 ;
input reset_in ;
input wclock_in ;
input [39:0] data_in ;
input [(ADDR_LENGTH - 1):0] raddr_in ;
input [(ADDR_LENGTH - 1):0] waddr_in ;
output [39:0] data_out ;
input wenable_in ;

reg [39:0] mem [(SIZE - 1):0] ;
wire [(ADDR_LENGTH - 1):0] raddr = raddr_in ;
wire [(ADDR_LENGTH - 1):0] waddr = waddr_in ;

assign data_out = mem[raddr] ;

always@(posedge wclock_in)
begin
    if (wenable_in)
        mem[waddr] <= #`FF_DELAY data_in ;
end

endmodule