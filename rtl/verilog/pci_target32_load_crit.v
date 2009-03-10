//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: pci_target32_load_crit.v                         ////
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

module PCI_TARGET32_LOAD_CRIT
(
    tar_load_out_w,
    tar_load_out_w_irdy,
    load_med_reg_w,
    load_med_reg_w_irdy,
    pci_irdy_in,
    pci_target_load_out,
    load_medium_reg_out
);

input       tar_load_out_w ;		// target load signal (composed without critical signals) that don't need critical inputs
input       tar_load_out_w_irdy ;	// target load signal (composed without critical signals) that needs AND with critical
									// IRDY input
input       load_med_reg_w ;		// load reg signal (composed without critical signals) that don't need critical inputs
input       load_med_reg_w_irdy ;	// load reg signal (composed without critical signals) that needs AND with critical
									// IRDY input
input       pci_irdy_in ;			// critical constrained input signal

output		pci_target_load_out ;	// pci target load output
output		load_medium_reg_out ;	// load medium register output

// pci target load output with preserved hierarchy for minimum delay!
assign 	pci_target_load_out = (tar_load_out_w || (tar_load_out_w_irdy && ~pci_irdy_in)) ;
// load medium register output with preserved hierarchy for minimum delay!
assign 	load_medium_reg_out = (load_med_reg_w || (load_med_reg_w_irdy && ~pci_irdy_in)) ;


endmodule