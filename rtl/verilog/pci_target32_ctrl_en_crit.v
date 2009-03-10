//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: pci_target32_ctrl_en_crit.v                      ////
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
// Revision 1.1.1.1  2001/10/02 15:33:47  mihad
// New project directory structure
//
//

// module is used to separate logic which uses criticaly constrained inputs from slower logic.
// It is used to synthesize critical timing logic separately with faster cells or without optimization

`include "constants.v"
`include "timescale.v"

module PCI_TARGET32_CTRL_EN_CRIT
(
    ctrl_en_w,
    ctrl_en_w_irdy,
    pci_irdy_in,
    pci_trdy_en_out,
    pci_stop_en_out,
    pci_devsel_en_out
);

input       ctrl_en_w ;		// ctrl enable signal (composed without critical signals) that don't need critical IRDY input
input       ctrl_en_w_irdy ;// ctrl enable signal (composed without critical signals) that needs AND with critical IRDY input
input       pci_irdy_in ;	// critical constrained input signal

output		pci_trdy_en_out ;	// trdy enable output
output		pci_stop_en_out ;	// stop enable output
output		pci_devsel_en_out ;	// devsel enable output

// control enable signal with preserved hierarchy for minimum delay!
wire	ctrl_en_out = (ctrl_en_w || (ctrl_en_w_irdy && ~pci_irdy_in)) ;

// control enable signal assigned to all three outputs
assign 	pci_trdy_en_out = ctrl_en_out ;
assign 	pci_stop_en_out = ctrl_en_out ;
assign 	pci_devsel_en_out = ctrl_en_out ;


endmodule