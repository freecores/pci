//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "bus_commands.v"                                  ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - Miha Dolenc (mihad@opencores.org)                     ////
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
// Revision 1.2  2001/07/19 12:24:35  mihad
// no message
//
//

// definitions of PCI bus commands
`define BC_IACK             4'h0  //used
`define BC_SPECIAL          4'h1  //not used
`define BC_IO_READ          4'h2  //used
`define BC_IO_WRITE         4'h3  //used
`define BC_RESERVED0        4'h4  //not used
`define BC_RESERVED1        4'h5  //not used
`define BC_MEM_READ         4'h6  //used
`define BC_MEM_WRITE        4'h7  //used
`define BC_RESERVED2        4'h8  //not used
`define BC_RESERVED3        4'h9  //not used
`define BC_CONF_READ        4'hA  //used
`define BC_CONF_WRITE       4'hB  //used
`define BC_MEM_READ_MUL     4'hC  //used
`define BC_DUAL_ADDR_CYC    4'hD  //not used
`define BC_MEM_READ_LN      4'hE  //used
`define BC_MEM_WRITE_INVAL  4'hF  //not used