//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: pci_target32_interface.v                         ////
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

`include "bus_commands.v"
`include "constants.v"
`include "timescale.v"

module PCI_TARGET32_INTERFACE
(
    // system inputs
    clk_in,
    reset_in,
	
    // PCI Target side of INTERFACE
    address_in,
    addr_claim_out,
    bc_in,
    bc0_in,
    data_in,
    data_out,
    be_in,
    req_in,
    rdy_in,
    addr_phase_in,
    bckp_trdy_in,
    last_reg_in,
    frame_reg_in,
    fetch_pcir_fifo_in,
    load_medium_reg_in,
    sel_fifo_mreg_in,
    sel_conf_fifo_in,
    fetch_conf_in,
    load_to_pciw_fifo_in,
    load_to_conf_in,
    same_read_out,

	norm_access_to_config_out,
	read_completed_out,
	read_processing_out,
	target_abort_out,
	disconect_wo_data_out,
	pciw_fifo_full_out,
	pcir_fifo_data_err_out,
	wbw_fifo_empty_out,
	
	// Delayed synchronizacion module signals
	req_out,       
    done_out,      
    in_progress_out,
	req_req_pending_in, 
    req_comp_pending_in,
	addr_out, 
    be_out,   
    we_out,   
    bc_out, 
    burst_out,
	strd_addr_in,
	strd_bc_in,
    status_in,
    comp_flush_in,

	// FIFO signals
	pcir_fifo_renable_out,  
	pcir_fifo_data_in,   	
	pcir_fifo_be_in, 		
	pcir_fifo_control_in,
	pcir_fifo_flush_out, 	
	pcir_fifo_almost_empty_in, 	    
	pcir_fifo_empty_in,
	pciw_fifo_wenable_out, 	    
	pciw_fifo_addr_data_out,	
	pciw_fifo_cbe_out, 		    
	pciw_fifo_control_out,	
	pciw_fifo_two_left_in,    
	pciw_fifo_almost_full_in,
	pciw_fifo_full_in,
	wbw_fifo_empty_in,
	
	// Configuration space signals
	conf_hit_out,
	conf_addr_out,
	conf_data_out,
	conf_data_in,
	conf_be_out,
	conf_we_out,
	conf_re_out,
	mem_enable_in,
	io_enable_in,
	mem_io_addr_space0_in,
	mem_io_addr_space1_in,
	mem_io_addr_space2_in,
	mem_io_addr_space3_in,
	mem_io_addr_space4_in,
	mem_io_addr_space5_in,
	pre_fetch_en0_in,
	pre_fetch_en1_in,
	pre_fetch_en2_in,
	pre_fetch_en3_in,
	pre_fetch_en4_in,
	pre_fetch_en5_in,
	pci_base_addr0_in,
	pci_base_addr1_in,
	pci_base_addr2_in,
	pci_base_addr3_in,
	pci_base_addr4_in,
	pci_base_addr5_in,
	pci_addr_mask0_in,
	pci_addr_mask1_in,
	pci_addr_mask2_in,
	pci_addr_mask3_in,
	pci_addr_mask4_in,
	pci_addr_mask5_in,
	pci_tran_addr0_in,
	pci_tran_addr1_in,
	pci_tran_addr2_in,
	pci_tran_addr3_in,
	pci_tran_addr4_in,
	pci_tran_addr5_in,
	addr_tran_en0_in,
	addr_tran_en1_in,
	addr_tran_en2_in,
	addr_tran_en3_in,
	addr_tran_en4_in,
	addr_tran_en5_in
) ;

/*==================================================================================================================
System inputs.
==================================================================================================================*/
// PCI side clock and reset
input   clk_in,
        reset_in ;


/*==================================================================================================================
Side of the PCI Target state machine 
==================================================================================================================*/
// Data, byte enables, bus commands and address ports
input	[31:0]	address_in ;		// current request address input - registered
output          addr_claim_out ;	// current request address claim output
input	[3:0]   bc_in ;				// current request bus command input - registered
input			bc0_in ;			// current cycle RW signal
output	[31:0]  data_out ;			// for read operations - current dataphase data output
input   [31:0]  data_in ;			// for write operations - current request data input - registered
input   [3:0]	be_in ;				// current dataphase byte enable inputs - registered
// Port connection control signals from PCI FSM
input         	req_in ;     		// Read is requested to WB master from PCI side
input         	rdy_in ;     		// DATA / ADDRESS selection from PCI side when read or write - registered
input			addr_phase_in ;		// Indicates address phase and also fast-back-to-back address phase - registered
input			bckp_trdy_in ;		// TRDY output (which is registered) equivalent
input		    last_reg_in ;		// Indicates last data phase - registered
input			frame_reg_in ;		// FRAME input signal - registered
input		    fetch_pcir_fifo_in ;// Read enable for PCIR_FIFO when when read is finishen on WB side
input		    load_medium_reg_in ;// Load data from PCIR_FIFO to medium register (first data must be prepared on time)
input		    sel_fifo_mreg_in ;	// Read data selection between PCIR_FIFO and medium register
input		    sel_conf_fifo_in ;	// Read data selection between Configuration registers and "FIFO"
input		    fetch_conf_in ;		// Read enable for configuration space registers
input		    load_to_pciw_fifo_in ;// Write enable to PCIW_FIFO
input		    load_to_conf_in ;	// Write enable to Configuration space registers


/*==================================================================================================================
Status outputs to PCI side (FSM)
==================================================================================================================*/
output			same_read_out ;				// Indicates the same read request (important when read is finished on WB side)
output			norm_access_to_config_out ;	// Indicates the access to Configuration space with MEMORY commands
output			read_completed_out ;		// Indicates that read request is completed on WB side
output			read_processing_out ;		// Indicates that read request is processing on WB side
output			target_abort_out ;			// Indicates target abort termination
output			disconect_wo_data_out ;		// Indicates disconnect with OR without data termination
output			pciw_fifo_full_out ;		// Indicates that write PCIW_FIFO is full
output			pcir_fifo_data_err_out ;	// Indicates data error on current data read from PCIR_FIFO
output			wbw_fifo_empty_out ;		// Indicates that WB SLAVE has no data to be written to PCI bus


/*==================================================================================================================
Read request interface through Delayed sinchronization module to WB Master
==================================================================================================================*/
// request, completion, done and progress indicator outputs for delayed_sync module where they are synchronized
output			req_out,           	// request qualifier - when 1 it indicates that valid data is provided on outputs
      			done_out,          	// done output - when 1 indicates that PCI Target has completed a cycle on its bus
      			in_progress_out ;  	// out progress indicator - indicates that current completion is in progress on 
									//   PCI Target side
// pending indication inputs - PCI Target side must know about requests and completions
input			req_req_pending_in ; 	// request pending input for PCI Target side
input   		req_comp_pending_in ;	// completion pending input for PCI Target side - it indicates when completion
										//   is ready for completing on PCI Target bus
// various data outputs - PCI Target sets address, bus command, byte enables, write enable and burst
output	[31:0]	addr_out ;   // address bus output
output	[3:0]	be_out ;     // byte enable output
output	     	we_out ;     // write enable output - read/write request indication 1 = write request / 0 = read request
output	[3:0]	bc_out ;     // bus command output
output		 	burst_out ;  // pre-fetch enable & burst read - qualifies pre-fetch for access to current image space

// completion side signals encoded termination status - 0 = normal completion / 1 = error terminated completion
input	[31:0]	strd_addr_in ;	// Stored requested read access address
input	[3:0]	strd_bc_in ;	// Stored requested read access bus command
input			status_in ;  	// Error status reported - NOT USED because FIFO control bits determin data error status
input		    comp_flush_in ;	// If completition counter (2^16 clk periods) has expired, PCIR_FIFO must flush data


/*==================================================================================================================
PCIR_PCIW_FIFO signals from pci side
==================================================================================================================*/
// PCIR_FIFO control signals used for fetching data from PCIR_FIFO 
output			pcir_fifo_renable_out ;			// read enable output to PCIR_FIFO
input	[31:0]	pcir_fifo_data_in ;				// data input from PCIR_FIFO
input	[3:0]	pcir_fifo_be_in ;				// byte enable input from PCIR_FIFO
input	[3:0]	pcir_fifo_control_in ;			// control signals input from PCIR_FIFO
output			pcir_fifo_flush_out ;			// flush PCIR_FIFO
input			pcir_fifo_almost_empty_in ;		// almost empty indicator from PCIR_FIFO
input			pcir_fifo_empty_in ;			// empty indicator

// PCIW_FIFO control signals used for sinking data into PCIW_FIFO and status monitoring
output			pciw_fifo_wenable_out ;		// write enable output to PCIW_FIFO
output	[31:0]	pciw_fifo_addr_data_out ;	// address / data output signals to PCIW_FIFO
output	[3:0]	pciw_fifo_cbe_out ;			// command / byte enable signals to PCIW_FIFO
output	[3:0]	pciw_fifo_control_out ;		// control signals to PCIW_FIFO
input			pciw_fifo_two_left_in ;		// two data spaces left in PCIW_FIFO 
input			pciw_fifo_almost_full_in ;	// almost full indicator from PCIW_FIFO
input			pciw_fifo_full_in ;			// full indicator from PCIW_FIFO

// WBW_FIFO empy control signal used when delayed read is complete in PCIR_FIFO
input			wbw_fifo_empty_in ;			// empty indicator from WBW_FIFO


/*==================================================================================================================
Configuration space signals - from and to registers
==================================================================================================================*/
// BUS for reading and writing to configuration space registers
output			conf_hit_out ;	// like "chip select" for configuration space
output	[11:0]	conf_addr_out ;	// address to configuration space when there is access to it
output	[31:0]	conf_data_out ;	// data to configuration space - for writing to registers
input	[31:0]	conf_data_in ;	// data from configuration space - for reading from registers
output	[3:0]	conf_be_out ;	// byte enables used for correct writing to configuration space
output			conf_we_out ;	// write enable control signal - 1 for writing / 0 for nothing
output			conf_re_out ;	// read enable control signal - 1 for reading / 0 for nothing

// Inputs for image control registers
input			mem_enable_in ;	// allowed access to memory mapped image
input			io_enable_in ;	// allowed access to io mapped image

// Inputs needed for determining if image is assigned to memory or io space with pre-fetch and address translation
input			mem_io_addr_space0_in ;	// bit-0 in pci_base_addr0 register 
input			mem_io_addr_space1_in ;	// bit-0 in pci_base_addr1 register 
input			mem_io_addr_space2_in ;	// bit-0 in pci_base_addr2 register 
input			mem_io_addr_space3_in ;	// bit-0 in pci_base_addr3 register 
input			mem_io_addr_space4_in ;	// bit-0 in pci_base_addr4 register 
input			mem_io_addr_space5_in ;	// bit-0 in pci_base_addr5 register
input			pre_fetch_en0_in ;	// bit-1 in pci_image_ctr0 register
input			pre_fetch_en1_in ;	// bit-1 in pci_image_ctr1 register
input			pre_fetch_en2_in ;	// bit-1 in pci_image_ctr2 register
input			pre_fetch_en3_in ;	// bit-1 in pci_image_ctr3 register
input			pre_fetch_en4_in ;	// bit-1 in pci_image_ctr4 register
input			pre_fetch_en5_in ;	// bit-1 in pci_image_ctr5 register

// Input from image registers - register values needed for decoder to work properly
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_base_addr0_in ;	// base address from base address register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_base_addr1_in ; // base address from base address register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_base_addr2_in ; // base address from base address register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_base_addr3_in ; // base address from base address register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_base_addr4_in ; // base address from base address register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_base_addr5_in ; // base address from base address register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_addr_mask0_in ; // masking of base address from address mask register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_addr_mask1_in ; // masking of base address from address mask register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_addr_mask2_in ; // masking of base address from address mask register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_addr_mask3_in ; // masking of base address from address mask register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_addr_mask4_in ; // masking of base address from address mask register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_addr_mask5_in ; // masking of base address from address mask register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_tran_addr0_in ; // translation address from address translation register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_tran_addr1_in ; // translation address from address translation register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_tran_addr2_in ; // translation address from address translation register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_tran_addr3_in ; // translation address from address translation register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_tran_addr4_in ; // translation address from address translation register
input	[31:(32-`PCI_NUM_OF_DEC_ADDR_LINES)]	pci_tran_addr5_in ; // translation address from address translation register

input			addr_tran_en0_in ;	// address translation enable bit
input			addr_tran_en1_in ;	// address translation enable bit
input			addr_tran_en2_in ;	// address translation enable bit
input			addr_tran_en3_in ;	// address translation enable bit
input			addr_tran_en4_in ;	// address translation enable bit
input			addr_tran_en5_in ;	// address translation enable bit

/*==================================================================================================================
END of input / output PORT DEFINITONS !!!
==================================================================================================================*/

// address output from address multiplexer
reg		[31:0]	address ;
// prefetch enable for access to selected image space
reg				pre_fetch_en ;

// Input addresses and image hits from address decoders - addresses are multiplexed to address
wire			hit0_in ;
wire	[31:0]	address0_in ;
wire			hit1_in ;
wire	[31:0]	address1_in ;
`ifdef		PCI_IMAGE2              
wire			hit2_in ;
wire	[31:0]	address2_in ;
`endif
`ifdef		PCI_IMAGE3
wire			hit2_in ;
wire	[31:0]	address2_in ;
wire			hit3_in ;
wire	[31:0]	address3_in ;
`endif
`ifdef		PCI_IMAGE4
wire			hit2_in ;
wire	[31:0]	address2_in ;
wire			hit3_in ;
wire	[31:0]	address3_in ;
wire			hit4_in ;
wire	[31:0]	address4_in ;
`endif
`ifdef		PCI_IMAGE5
wire			hit2_in ;
wire	[31:0]	address2_in ;
wire			hit3_in ;
wire	[31:0]	address3_in ;
wire			hit4_in ;
wire	[31:0]	address4_in ;
wire			hit5_in ;
wire	[31:0]	address5_in ;
`endif
`ifdef		PCI_IMAGE6
wire			hit2_in ;
wire	[31:0]	address2_in ;
wire			hit3_in ;
wire	[31:0]	address3_in ;
wire			hit4_in ;
wire	[31:0]	address4_in ;
wire			hit5_in ;
wire	[31:0]	address5_in ;
`endif

// Include address decoders
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder0 
				   (.hit			(hit0_in), 
					.addr_out		(address0_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr0_in),
					.mask_addr		(pci_addr_mask0_in), 
					.tran_addr		(pci_tran_addr0_in), 
					.at_en			(addr_tran_en0_in),
					.mem_io_space	(mem_io_addr_space0_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder1 
				   (.hit			(hit1_in), 
					.addr_out		(address1_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr1_in),
					.mask_addr		(pci_addr_mask1_in), 
					.tran_addr		(pci_tran_addr1_in), 
					.at_en			(addr_tran_en1_in),  
					.mem_io_space	(mem_io_addr_space1_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
`ifdef		PCI_IMAGE2              
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder2
				   (.hit			(hit2_in), 
					.addr_out		(address2_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr2_in),
					.mask_addr		(pci_addr_mask2_in), 
					.tran_addr		(pci_tran_addr2_in), 
					.at_en			(addr_tran_en2_in),  
					.mem_io_space	(mem_io_addr_space2_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
`endif
`ifdef		PCI_IMAGE3
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder2
				   (.hit			(hit2_in), 
					.addr_out		(address2_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr2_in),
					.mask_addr		(pci_addr_mask2_in), 
					.tran_addr		(pci_tran_addr2_in), 
					.at_en			(addr_tran_en2_in),  
					.mem_io_space	(mem_io_addr_space2_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder3 
				   (.hit			(hit3_in), 
					.addr_out		(address3_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr3_in),
					.mask_addr		(pci_addr_mask3_in), 
					.tran_addr		(pci_tran_addr3_in), 
					.at_en			(addr_tran_en3_in),  
					.mem_io_space	(mem_io_addr_space3_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
`endif
`ifdef		PCI_IMAGE4
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder2
				   (.hit			(hit2_in), 
					.addr_out		(address2_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr2_in),
					.mask_addr		(pci_addr_mask2_in), 
					.tran_addr		(pci_tran_addr2_in), 
					.at_en			(addr_tran_en2_in),  
					.mem_io_space	(mem_io_addr_space2_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder3 
				   (.hit			(hit3_in), 
					.addr_out		(address3_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr3_in),
					.mask_addr		(pci_addr_mask3_in), 
					.tran_addr		(pci_tran_addr3_in), 
					.at_en			(addr_tran_en3_in),  
					.mem_io_space	(mem_io_addr_space3_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder4 
				   (.hit			(hit4_in), 
					.addr_out		(address4_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr4_in),
					.mask_addr		(pci_addr_mask4_in), 
					.tran_addr		(pci_tran_addr4_in), 
					.at_en			(addr_tran_en4_in),  
					.mem_io_space	(mem_io_addr_space4_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
`endif
`ifdef		PCI_IMAGE5
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder2
				   (.hit			(hit2_in), 
					.addr_out		(address2_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr2_in),
					.mask_addr		(pci_addr_mask2_in), 
					.tran_addr		(pci_tran_addr2_in), 
					.at_en			(addr_tran_en2_in),  
					.mem_io_space	(mem_io_addr_space2_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder3 
				   (.hit			(hit3_in), 
					.addr_out		(address3_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr3_in),
					.mask_addr		(pci_addr_mask3_in), 
					.tran_addr		(pci_tran_addr3_in), 
					.at_en			(addr_tran_en3_in),  
					.mem_io_space	(mem_io_addr_space3_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder4 
				   (.hit			(hit4_in), 
					.addr_out		(address4_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr4_in),
					.mask_addr		(pci_addr_mask4_in), 
					.tran_addr		(pci_tran_addr4_in), 
					.at_en			(addr_tran_en4_in),  
					.mem_io_space	(mem_io_addr_space4_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder5 
				   (.hit			(hit5_in), 
					.addr_out		(address5_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr5_in),
					.mask_addr		(pci_addr_mask5_in), 
					.tran_addr		(pci_tran_addr5_in), 
					.at_en			(addr_tran_en5_in),  
					.mem_io_space	(mem_io_addr_space5_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
`endif
`ifdef		PCI_IMAGE6
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder2
				   (.hit			(hit2_in), 
					.addr_out		(address2_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr2_in),
					.mask_addr		(pci_addr_mask2_in), 
					.tran_addr		(pci_tran_addr2_in), 
					.at_en			(addr_tran_en2_in),  
					.mem_io_space	(mem_io_addr_space2_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder3 
				   (.hit			(hit3_in), 
					.addr_out		(address3_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr3_in),
					.mask_addr		(pci_addr_mask3_in), 
					.tran_addr		(pci_tran_addr3_in), 
					.at_en			(addr_tran_en3_in),  
					.mem_io_space	(mem_io_addr_space3_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder4 
				   (.hit			(hit4_in), 
					.addr_out		(address4_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr4_in),
					.mask_addr		(pci_addr_mask4_in), 
					.tran_addr		(pci_tran_addr4_in), 
					.at_en			(addr_tran_en4_in),  
					.mem_io_space	(mem_io_addr_space4_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
	PCI_DECODER   #(`PCI_NUM_OF_DEC_ADDR_LINES) decoder5 
				   (.hit			(hit5_in), 
					.addr_out		(address5_in), 
					.addr_in		(address_in), 
					.base_addr		(pci_base_addr5_in),
					.mask_addr		(pci_addr_mask5_in), 
					.tran_addr		(pci_tran_addr5_in), 
					.at_en			(addr_tran_en5_in),  
					.mem_io_space	(mem_io_addr_space5_in), 
					.mem_en			(mem_enable_in), 
					.io_en			(io_enable_in)
					) ;
`endif

// Internal signals for image hit determination
reg				addr_claim ;// address claim signal is asinchronous set for addr_claim_out signal to PCI Target SM

// Determining if image 0 is assigned to configuration space or as normal pci to wb access!
//   if normal access is allowed to configuration space, then hit0 is hit0_conf
`ifdef		HOST
	`ifdef	PCI_IMAGE6
		parameter	hit0_conf = 1'b0 ;
	`else
		parameter	hit0_conf = 1'b1 ;	// if normal access is allowed to configuration space, then hit0 is hit0_conf
	`endif
`else
		parameter	hit0_conf = 1'b1 ;	// if normal access is allowed to configuration space, then hit0 is hit0_conf
`endif

// Logic with address mux, determining if address is still in the same image space and if it is prefetced or not
`ifdef		PCI_IMAGE6
	always@(hit5_in or     hit4_in or     hit3_in or     hit2_in or     hit1_in or     hit0_in or
			address5_in or address4_in or address3_in or address2_in or address1_in or address0_in or
			pre_fetch_en5_in or
			pre_fetch_en4_in or
			pre_fetch_en3_in or
			pre_fetch_en2_in or
			pre_fetch_en1_in or
			pre_fetch_en0_in 
			)
	begin
		addr_claim <= (hit5_in || hit4_in || hit3_in || hit2_in) || (hit1_in || hit0_in) ;
		case ({hit5_in, hit4_in, hit3_in, hit2_in, hit1_in, hit0_in})
		6'b010000 :
		begin
			address <= address4_in ;
			pre_fetch_en <= pre_fetch_en4_in ;
		end
		6'b001000 :
		begin
			address <= address3_in ;
			pre_fetch_en <= pre_fetch_en3_in ;
		end
		6'b000100 :
		begin
			address <= address2_in ;
			pre_fetch_en <= pre_fetch_en2_in ;
		end
		6'b000010 :
		begin
			address <= address1_in ;
			pre_fetch_en <= pre_fetch_en1_in ;
		end
		6'b000001 :
		begin
			address <= address0_in ;
			pre_fetch_en <= pre_fetch_en0_in ;
		end
		default	: 
		begin
			address <= address5_in ;
			pre_fetch_en <= pre_fetch_en5_in ;
		end
		endcase
	end
`else
	`ifdef		PCI_IMAGE5
		always@(hit5_in or     hit4_in or     hit3_in or     hit2_in or     hit1_in or     hit0_in or
				address5_in or address4_in or address3_in or address2_in or address1_in or address0_in or
				pre_fetch_en5_in or
				pre_fetch_en4_in or
				pre_fetch_en3_in or
				pre_fetch_en2_in or
				pre_fetch_en1_in or
				pre_fetch_en0_in 
				)
		begin
			addr_claim <= (hit5_in || hit4_in || hit3_in || hit2_in) || (hit1_in || hit0_in) ;
			case ({hit5_in, hit4_in, hit3_in, hit2_in, hit1_in, hit0_in})
			6'b010000 :
			begin
				address <= address4_in ;
				pre_fetch_en <= pre_fetch_en4_in ;
			end
			6'b001000 :
			begin
				address <= address3_in ;
				pre_fetch_en <= pre_fetch_en3_in ;
			end
			6'b000100 :
			begin
				address <= address2_in ;
				pre_fetch_en <= pre_fetch_en2_in ;
			end
			6'b000010 :
			begin
				address <= address1_in ;
				pre_fetch_en <= pre_fetch_en1_in ;
			end
			6'b000001 :
			begin
				address <= address0_in ;
				pre_fetch_en <= pre_fetch_en0_in ;
			end
			default	: 
			begin
				address <= address5_in ;
				pre_fetch_en <= pre_fetch_en5_in ;
			end
			endcase
		end
	`else
		`ifdef		PCI_IMAGE4
			always@(hit4_in or     hit3_in or     hit2_in or     hit1_in or     hit0_in or
					address4_in or address3_in or address2_in or address1_in or address0_in or
					pre_fetch_en4_in or
					pre_fetch_en3_in or
					pre_fetch_en2_in or
					pre_fetch_en1_in or
					pre_fetch_en0_in 
					)
			begin
				addr_claim <= hit4_in || (hit3_in || hit2_in || hit1_in || hit0_in) ;
				case ({hit4_in, hit3_in, hit2_in, hit1_in, hit0_in})
				5'b01000 :
				begin
					address <= address3_in ;
					pre_fetch_en <= pre_fetch_en3_in ;
				end
				5'b00100 :
				begin
					address <= address2_in ;
					pre_fetch_en <= pre_fetch_en2_in ;
				end
				5'b00010 :
				begin
					address <= address1_in ;
					pre_fetch_en <= pre_fetch_en1_in ;
				end
				5'b00001 :
				begin
					address <= address0_in ;
					pre_fetch_en <= pre_fetch_en0_in ;
				end
				default	: 
				begin
					address <= address4_in ;
					pre_fetch_en <= pre_fetch_en4_in ;
				end
				endcase
			end
		`else
			`ifdef		PCI_IMAGE3
				always@(hit3_in or     hit2_in or     hit1_in or     hit0_in or
						address3_in or address2_in or address1_in or address0_in or
						pre_fetch_en3_in or
						pre_fetch_en2_in or
						pre_fetch_en1_in or
						pre_fetch_en0_in 
						)
				begin
					addr_claim <= (hit3_in || hit2_in || hit1_in || hit0_in) ;
					case ({hit3_in, hit2_in, hit1_in, hit0_in})
					4'b0100 :
					begin
						address <= address2_in ;
						pre_fetch_en <= pre_fetch_en2_in ;
					end
					4'b0010 :
					begin
						address <= address1_in ;
						pre_fetch_en <= pre_fetch_en1_in ;
					end
					4'b0001 :
					begin
						address <= address0_in ;
						pre_fetch_en <= pre_fetch_en0_in ;
					end
					default	: 
					begin
						address <= address3_in ;
						pre_fetch_en <= pre_fetch_en3_in ;
					end
					endcase
				end
			`else
				`ifdef		PCI_IMAGE2
					always@(hit2_in or     hit1_in or     hit0_in or
							address2_in or address1_in or address0_in or
							pre_fetch_en2_in or
							pre_fetch_en1_in or
							pre_fetch_en0_in 
							)
					begin
						addr_claim <= (hit2_in || hit1_in || hit0_in) ;
						case ({hit2_in, hit1_in, hit0_in})
						3'b010 :
						begin
							address <= address1_in ;
							pre_fetch_en <= pre_fetch_en1_in ;
						end
						3'b001 :
						begin
							address <= address0_in ;
							pre_fetch_en <= pre_fetch_en0_in ;
						end
						default	: 
						begin
							address <= address2_in ;
							pre_fetch_en <= pre_fetch_en2_in ;
						end
						endcase
					end
				`else
					always@(hit1_in or     hit0_in or
							address1_in or address0_in or
							pre_fetch_en1_in or
							pre_fetch_en0_in 
							)
					begin
						addr_claim <= (hit1_in || hit0_in) ;
						case ({hit1_in, hit0_in})
						2'b01 :
						begin
							address <= address0_in ;
							pre_fetch_en <= pre_fetch_en0_in ;
						end
						default	: 
						begin
							address <= address1_in ;
							pre_fetch_en <= pre_fetch_en1_in ;
						end
						endcase
					end
				`endif
			`endif
		`endif
	`endif
`endif

// Address claim output to PCI Target SM
assign	addr_claim_out = addr_claim ;

reg		[11:0]	strd_address ;		// stored INPUT address for accessing Configuration space registers
reg		[31:0]	norm_address ;		// stored normal address (decoded and translated) for access to WB
reg				norm_prf_en ;		// stored pre-fetch enable
reg		[3:0]	norm_bc ;			// stored bus-command
reg				same_read_reg ;		// stored SAME_READ information
reg				bckp_trdy_reg ;		// delayed registered TRDY output equivalent signal
reg				trdy_asserted_reg ;	// delayed bckp_trdy_reg signal

always@(posedge clk_in or posedge reset_in)
begin
    if (reset_in) 
	begin
		strd_address <= 12'h000 ;
		norm_address <= 32'h0000_0000 ;
		norm_prf_en <= 1'b0 ;
		norm_bc <= 4'h0 ;
		same_read_reg <= 1'b0 ;
	end
	else
	begin
		if (addr_phase_in)
		begin
			strd_address <= address_in[11:0] ;
			norm_address <= address ;
			norm_prf_en <= pre_fetch_en ;
			norm_bc <= bc_in ;
			same_read_reg <= same_read_out ;
		end
	end
end
always@(posedge clk_in or posedge reset_in)
begin
    if (reset_in) 
	begin
		bckp_trdy_reg <= 1'b0 ;
		trdy_asserted_reg <= 1'b0 ;
	end
	else
	begin
		bckp_trdy_reg <= bckp_trdy_in ;
		trdy_asserted_reg <= bckp_trdy_reg ;
	end
end
// Signal indicates when target ready is deaserted on PCI bus
wire	trdy_asserted = trdy_asserted_reg && bckp_trdy_reg ;

reg				same_read_request ;

// When delayed read is completed on WB, addres and bc must be compered, if there is the same read request 
always@(address or strd_addr_in or bc_in or strd_bc_in)
begin
	if ((address == strd_addr_in) & (bc_in == strd_bc_in))
		same_read_request <= 1'b1 ;
	else
		same_read_request <= 1'b0 ;
end

assign	same_read_out = (same_read_request) ; // && ~pcir_fifo_empty_in) ;

// Signals for byte enable checking
reg				addr_burst_ok ;
reg				io_be_ok ;
reg				conf_be_ok ;

// Byte enable checking for IO, MEMORY and CONFIGURATION spaces - be_in is active low!
always@(strd_address or be_in)
begin
	case (strd_address[1:0])
	2'b11 :
	begin
		addr_burst_ok <= 1'b0 ;
		io_be_ok <= (be_in[2] && be_in[1] && be_in[0]) ; // only be3 can be active
		conf_be_ok <= 1'b0 ;
	end
	2'b10 :
	begin
		addr_burst_ok <= 1'b0 ;
		io_be_ok <= (~be_in[2] && be_in[1] && be_in[0]) || (be_in[3] && be_in[2] && be_in[1] && be_in[0]) ;
		conf_be_ok <= 1'b0 ;
	end
	2'b01 :
	begin
		addr_burst_ok <= 1'b0 ;
		io_be_ok <= (~be_in[1] && be_in[0]) || (be_in[3] && be_in[2] && be_in[1] && be_in[0]) ;
		conf_be_ok <= 1'b0 ;
	end
	default :	// 2'b00
	begin
		addr_burst_ok <= 1'b1 ;
		io_be_ok <= (~be_in[0]) || (be_in[3] && be_in[2] && be_in[1] && be_in[0]) ;
		conf_be_ok <= 1'b1 ;
	end
	endcase
end

reg				calc_target_abort ;
// Target abort indication regarding the registered bus command and current signals for byte enable checking
always@(bc_in or hit0_in or io_be_ok or conf_be_ok)
begin
	case (bc_in)
	// READ COMMANDS			
	`BC_IO_READ :
	begin
		case ({hit0_in, hit0_conf})
		2'b11 :
		begin
			calc_target_abort <= 1'b1 ;
		end
		default :
		begin
			if (io_be_ok)
			begin
				calc_target_abort <= 1'b0 ;
			end
			else
			begin
				calc_target_abort <= 1'b1 ;
			end
		end
		endcase
	end
	`BC_MEM_READ :
	begin
		case ({hit0_in, hit0_conf})
		2'b11 :
		begin
			calc_target_abort <= 1'b0 ;
		end
		default :
		begin
			calc_target_abort <= 1'b0 ;
		end
		endcase
	end
	`BC_CONF_READ :
	begin
		case (conf_be_ok)
		1'b1 :
		begin
			calc_target_abort <= 1'b0 ;
		end
		default :
		begin
			calc_target_abort <= 1'b1 ;
		end
		endcase
	end
	`BC_MEM_READ_LN,
	`BC_MEM_READ_MUL :
	begin
		case ({hit0_in, hit0_conf})
		2'b11 :
		begin
			calc_target_abort <= 1'b1 ;
		end
		default :
		begin
			calc_target_abort <= 1'b0 ;
		end
		endcase
	end
	// WRITE COMMANDS			
	`BC_IO_WRITE :
	begin
		case ({hit0_in, hit0_conf})
		2'b11 :
		begin
			calc_target_abort <= 1'b1 ;
		end
		default :
		begin
			if (io_be_ok)
			begin
				calc_target_abort <= 1'b0 ;
			end
			else
			begin
				calc_target_abort <= 1'b1 ;
			end
		end
		endcase
	end
	`BC_MEM_WRITE :
	begin
		case ({hit0_in, hit0_conf})
		2'b11 :
		begin
			calc_target_abort <= 1'b0 ;
		end
		default :
		begin
			calc_target_abort <= 1'b0 ;
		end
		endcase
	end
	`BC_CONF_WRITE :
	begin
		case (conf_be_ok)
		1'b1 :
		begin
			calc_target_abort <= 1'b0 ;
		end
		default :
		begin
			calc_target_abort <= 1'b1 ;
		end
		endcase
	end
	`BC_MEM_WRITE_INVAL :
	begin
		case ({hit0_in, hit0_conf})
		2'b11 :
		begin
			calc_target_abort <= 1'b1 ;
		end
		default :
		begin
			calc_target_abort <= 1'b0 ;
		end
		endcase
	end
	default :
	begin
		calc_target_abort <= 1'b0 ;
	end
	endcase
end

// Medium registers for data and control busses from PCIR_FIFO
reg		[31:0]	pcir_fifo_data_reg ;
reg		[3:0]	pcir_fifo_ctrl_reg ;

always@(posedge clk_in or posedge reset_in)
begin
    if (reset_in) 
    begin
    	pcir_fifo_data_reg <= 32'h0000_0000 ;
    	pcir_fifo_ctrl_reg <=  4'h0 ;
    end
    else
    begin
    	if (load_medium_reg_in)
    	begin
    		pcir_fifo_data_reg <= pcir_fifo_data_in ;
    		pcir_fifo_ctrl_reg <= pcir_fifo_control_in ;
    	end
    end
end

// selecting "fifo data" from medium registers or from PCIR_FIFO
wire [31:0]	pcir_fifo_data = sel_fifo_mreg_in ? pcir_fifo_data_in : pcir_fifo_data_reg ;
wire [3:0]	pcir_fifo_ctrl = sel_fifo_mreg_in ? pcir_fifo_control_in : pcir_fifo_ctrl_reg ;

// signal assignments to PCI Target FSM
assign	norm_access_to_config_out = (hit0_in && hit0_conf) ;
assign	read_completed_out = req_comp_pending_in ; // completion pending input for requesting side of the bridge 
assign	read_processing_out = req_req_pending_in ; // request pending input for requesting side
assign	disconect_wo_data_out = (
	((pcir_fifo_ctrl[`LAST_CTRL_BIT] || pcir_fifo_empty_in || ~addr_burst_ok) && ~bc0_in && ~frame_reg_in) ||
	((pciw_fifo_full_in || pciw_fifo_almost_full_in || pciw_fifo_two_left_in || ~addr_burst_ok) && bc0_in && ~frame_reg_in)
								) ;
assign	target_abort_out = (addr_phase_in && calc_target_abort) ;

// control signal assignments to read request sinchronization module
assign	done_out = (~sel_conf_fifo_in && same_read_reg && ~trdy_asserted && last_reg_in) ;
assign	in_progress_out = (~sel_conf_fifo_in && same_read_reg && ~bckp_trdy_in) ;

// signal used for PCIR_FIFO flush (with comp_flush_in signal)
wire	pcir_fifo_flush = (~sel_conf_fifo_in && same_read_reg && ~trdy_asserted && last_reg_in && ~pcir_fifo_empty_in) ;
// flush signal for PCIR_FIFO must be registered, since it asinchronously resets some status registers
reg		pcir_fifo_flush_reg ;
always@(posedge clk_in or posedge reset_in)
begin
    if (reset_in) 
    begin
    	pcir_fifo_flush_reg <=  1'b0 ;
    end
    else
    begin
    	pcir_fifo_flush_reg <= comp_flush_in || pcir_fifo_flush ;
    end
end

// signal assignments from fifo to PCI Target FSM
assign	wbw_fifo_empty_out = wbw_fifo_empty_in ;
assign	pciw_fifo_full_out = (pciw_fifo_full_in || pciw_fifo_almost_full_in || pciw_fifo_two_left_in) ;
assign	pcir_fifo_data_err_out = pcir_fifo_ctrl[`DATA_ERROR_CTRL_BIT] ;
// signal assignments to fifo
assign	pcir_fifo_flush_out							= pcir_fifo_flush_reg ;
assign	pcir_fifo_renable_out						= fetch_pcir_fifo_in ;
assign	pciw_fifo_wenable_out						= load_to_pciw_fifo_in ;
assign	pciw_fifo_control_out[`ADDR_CTRL_BIT]		= 1'b0 ;
assign	pciw_fifo_control_out[`BURST_BIT]			= rdy_in ? 1'b0 : ~frame_reg_in ;
assign	pciw_fifo_control_out[`DATA_ERROR_CTRL_BIT]	= 1'b0 ;
assign	pciw_fifo_control_out[`LAST_CTRL_BIT]		= rdy_in ? (last_reg_in || pciw_fifo_almost_full_in) : 1'b0 ;

// data and address outputs assignments to PCI Target FSM
assign	data_out = sel_conf_fifo_in ? conf_data_in : pcir_fifo_data ;
// data and address outputs assignments to read request sinchronization module
assign	req_out = req_in ;
assign	addr_out = norm_address ;
assign	be_out = be_in ;
assign	we_out = 1'b0 ;
assign	bc_out = norm_bc ;
assign	burst_out = ~frame_reg_in && norm_prf_en ;
// data and address outputs assignments to PCIW_FIFO
assign	pciw_fifo_addr_data_out = rdy_in ? data_in : norm_address ;
assign	pciw_fifo_cbe_out = rdy_in ? be_in : norm_bc ;
// data and address outputs assignments to Configuration space
assign	conf_data_out = data_in ;
assign	conf_addr_out = strd_address[11:0] ;
assign	conf_be_out = be_in ;
assign	conf_we_out = load_to_conf_in ;
assign	conf_re_out = fetch_conf_in ;


endmodule