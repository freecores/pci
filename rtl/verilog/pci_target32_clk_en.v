//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: pci_target32_clk_en.v                            ////
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

module PCI_TARGET32_CLK_EN
(
    addr_phase,
    config_access,
    addr_claim_in,
    disconect_wo_data_in,
    pcir_fifo_data_err_in,
    rw_cbe0,
    pci_frame_in,
    pci_irdy_in,
    state_wait,
    state_transfere,
    state_backoff,
    state_default,
    clk_enable
);

input           addr_phase ;			// indicates registered address phase on PCI bus
input           config_access ;			// indicates configuration access
input           addr_claim_in ;			// indicates claimed input PCI address
input           disconect_wo_data_in ;	// indicates disconnect without data termination from backend
input           pcir_fifo_data_err_in ;	// indicates FIFO data error termination from backend
input           rw_cbe0 ;				// registered (through all cycle) RW (CBE[0]) input signal from PCI bus
input           pci_frame_in ;			// critical constrained input signal
input			pci_irdy_in ;			// critical constrained input signal
input			state_wait ;			// indicates WAIT state of FSM
input 			state_transfere ;		// indicates TRANSFERE state of FSM
input			state_backoff ;			// indicates BACKOFF state of FSM
input			state_default ;			// indicates DEFAULT state of FSM

output			clk_enable ;			// FSM clock enable output


// clock enable signal when FSM is in IDLE state
wire s_idle_clk_en	=	((addr_phase && config_access) || 
						(addr_phase && ~config_access && addr_claim_in)) ;

// clock enable signal when FSM is in WAIT state or in DEFAULT state	
wire s_wait_clk_en	=	(state_wait || state_default) ;

// clock enable signal when FSM is in TRANSFERE state
wire s_tran_clk_en	=	state_transfere && 
						((disconect_wo_data_in && ~pci_irdy_in && ~pci_frame_in) || (pci_frame_in) || 
						(~rw_cbe0 && pcir_fifo_data_err_in && ~pci_irdy_in && ~pci_frame_in)) ;

// clock enable signal when FSM is in BACKOFF state 	
wire s_bcko_clk_en	=	(state_backoff && pci_frame_in) ;

// Clock enable signal for FSM with preserved hierarchy for minimum delay!
assign clk_enable	=	(s_idle_clk_en || s_wait_clk_en || s_tran_clk_en || s_bcko_clk_en) ;


endmodule