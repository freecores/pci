//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "out_reg.v"                                       ////
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
// Revision 1.1.1.1  2001/10/02 15:33:46  mihad
// New project directory structure
//
//

`include "constants.v"
`include "timescale.v"

// module inferes a single IOB output block as known in FPGA architectures
// It provides data flip flop with clock enable and output enable flip flop with clock enable
// This is tested in Xilinx FPGA - active low output enable
// Check polarity of output enable flip flop for specific architecure.
module OUT_REG
(
    reset_in,
    clk_in,
    dat_en_in,
    en_en_in,
    dat_in,
    en_in,
    en_out,
    dat_out
);

input   reset_in,
        clk_in,
        dat_en_in,
        en_en_in,
        dat_in,
        en_in ;

output dat_out ;
output en_out ;

reg dat_out,
    en_out ;

wire en_n = ~en_in ;

always@(posedge reset_in or posedge clk_in)
begin
    if ( reset_in )
        dat_out <= #`FF_DELAY 1'b0 ;
    else if ( dat_en_in )
        dat_out <= #`FF_DELAY dat_in ;
end

always@(posedge reset_in or posedge clk_in)
begin
    if ( reset_in )
        en_out <= #`FF_DELAY 1'b1 ;
    else if ( en_en_in )
        en_out <= #`FF_DELAY en_n ;
end

endmodule