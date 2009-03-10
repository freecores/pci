//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: defines.v                                        ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - Tadej Markovic, tadej@opencores.org                   ////
////                                                              ////
////  All additional information is avaliable in the README.txt   ////
////  file.                                                       ////
////                                                              ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000 Tadej Markovic, tadej@opencores.org       ////
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
// Revision 1.1.1.1  2001/06/06 15:11:35  tadej
// Initial release
//
//

// PCI bridge HOST/GUEST implentation
// - for HOST implementation MUST be written HOST othervise nothing !!!
`define HOST	

// PCI bridge HOST use of IMAGE0 to CONF. SPACE or NORMAL
// - for IMAGE0 to CONF. SPACE MUST be written PCI_IMAGE0_CONF othervise nothing !!!
`define PCI_IMAGE0_CONF

// no. of PCI Target IMAGES
// - used IMAGES MUST be written in a sequence order starting from '1'
//   since PCI_IMAGE 0 is always used by default !!!
`define PCI_IMAGE1
`define PCI_IMAGE2
`define PCI_IMAGE3
//`define PCI_IMAGE4
//`define PCI_IMAGE5

// no. of WISHBONE Slave IMAGES
// - used IMAGES MUST be written in a sequence order starting from '1'
//   since WB_IMAGE 0 is always used by default !!!
`define WB_IMAGE1
`define WB_IMAGE2
`define WB_IMAGE3
//`define WB_IMAGE4
//`define WB_IMAGE5

// the width of the registers
`define REG_WIDTH 32

// timing unit and precision bases
`timescale 1 ns / 100 ps
// timing delays
`define DLY_L1 1
`define DLY_L2 2
`define DLY_L3 3
`define DLY_L4 4
`define DLY_L5 5
