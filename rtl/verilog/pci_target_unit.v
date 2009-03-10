//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: pci_target_unit.v                                ////
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

// Module instantiates and connects other modules lower in hierarcy
// PCI target unit consists of modules that together form datapath
// between external WISHBONE slaves and external PCI initiators
`include "constants.v"
`include "timescale.v"

module PCI_TARGET_UNIT
(
    reset_in,
    wb_clock_in,
    pci_clock_in,
    ADR_O,
    MDATA_O,
    MDATA_I,
    CYC_O,
    STB_O,
    WE_O,
    SEL_O,
    ACK_I,
    RTY_I,
    ERR_I,
    CAB_O,
	pciu_mem_enable_in,
	pciu_io_enable_in,
    pciu_map_in,
    pciu_pref_en_in,
    pciu_conf_data_in,    
    pciu_wbw_fifo_empty_in,
    pciu_wbu_frame_en_in,
	pciu_bar0_in,
	pciu_bar1_in,
	pciu_bar2_in,
	pciu_bar3_in,
	pciu_bar4_in,
	pciu_bar5_in,
	pciu_am0_in,
	pciu_am1_in,
	pciu_am2_in,
	pciu_am3_in,
	pciu_am4_in,
	pciu_am5_in,
	pciu_ta0_in,
	pciu_ta1_in,
	pciu_ta2_in,
	pciu_ta3_in,
	pciu_ta4_in,
	pciu_ta5_in,
	pciu_at_en_in,
	pciu_cache_line_size_in,
	pciu_pciif_frame_in,
	pciu_pciif_irdy_in,
	pciu_pciif_idsel_in,
	pciu_pciif_frame_reg_in,
	pciu_pciif_irdy_reg_in,
	pciu_pciif_idsel_reg_in,
	pciu_pciif_ad_reg_in,
	pciu_pciif_cbe_reg_in,
	pciu_pciif_bckp_trdy_en_in,
	pciu_pciif_bckp_devsel_in,
	pciu_pciif_bckp_trdy_in,
	pciu_pciif_bckp_stop_in,
	pciu_pciif_trdy_out,
	pciu_pciif_stop_out,
	pciu_pciif_devsel_out,
	pciu_pciif_trdy_en_out,
	pciu_pciif_stop_en_out,
	pciu_pciif_devsel_en_out,
	pciu_pciif_target_load_out,
	pciu_pciif_ad_out,
	pciu_pciif_ad_en_out,
	pciu_pciif_tabort_set_out,
    pciu_err_addr_out,   
    pciu_err_bc_out,
    pciu_err_data_out,
	pciu_err_be_out,
    pciu_err_signal_out, 
    pciu_err_source_out, 
    pciu_err_rty_exp_out,
    pciu_err_pending_in,
    pciu_conf_offset_out,
    pciu_conf_renable_out,
    pciu_conf_wenable_out,
    pciu_conf_be_out,     
    pciu_conf_data_out,   
    pciu_conf_select_out,
    pciu_pci_drcomp_pending_out,
    pciu_pciw_fifo_empty_out
);

input reset_in,
      wb_clock_in,
      pci_clock_in ;

output	[31:0]  ADR_O   ;
output  [31:0]  MDATA_O ;
input   [31:0]  MDATA_I ;
output          CYC_O   ;
output          STB_O   ;
output          WE_O    ;
output  [3:0]   SEL_O   ;
input           ACK_I   ;
input           RTY_I   ;
input           ERR_I   ;
output          CAB_O   ;

input           pciu_wbw_fifo_empty_in ;
input			pciu_wbu_frame_en_in ;

input           pciu_mem_enable_in ;
input           pciu_io_enable_in ;
input   [5:0]   pciu_map_in ;
input   [5:0]   pciu_pref_en_in ;
input   [31:0]  pciu_conf_data_in ;

input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar0_in ; 
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar1_in ; 
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar2_in ; 
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar3_in ; 
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar4_in ; 
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar5_in ; 
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am0_in ;  
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am1_in ;  
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am2_in ;  
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am3_in ;  
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am4_in ;  
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am5_in ;  
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta0_in ;  
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta1_in ;  
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta2_in ;  
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta3_in ;  
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta4_in ;  
input   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta5_in ;  
input   [5:0]                               pciu_at_en_in ;

input   [7:0]   pciu_cache_line_size_in ;

input           pciu_pciif_frame_in	;
input           pciu_pciif_irdy_in ;  
input           pciu_pciif_idsel_in ;
input           pciu_pciif_frame_reg_in ;
input           pciu_pciif_irdy_reg_in ;
input           pciu_pciif_idsel_reg_in ;
input  [31:0]   pciu_pciif_ad_reg_in ;
input   [3:0]   pciu_pciif_cbe_reg_in ;
input			pciu_pciif_bckp_trdy_en_in ;
input			pciu_pciif_bckp_devsel_in ;
input			pciu_pciif_bckp_trdy_in ;
input			pciu_pciif_bckp_stop_in ;


output          pciu_pciif_trdy_out ;   
output          pciu_pciif_stop_out ;   
output          pciu_pciif_devsel_out ; 
output          pciu_pciif_trdy_en_out ;
output          pciu_pciif_stop_en_out ;
output          pciu_pciif_devsel_en_out ;
output			pciu_pciif_target_load_out ;
output [31:0]   pciu_pciif_ad_out ;     
output          pciu_pciif_ad_en_out ; 
output			pciu_pciif_tabort_set_out ;

output  [31:0]  pciu_err_addr_out ;
output  [3:0]   pciu_err_bc_out ;
output	[31:0]  pciu_err_data_out ;
output	[3:0]   pciu_err_be_out	;
output          pciu_err_signal_out ;
output          pciu_err_source_out ;
output          pciu_err_rty_exp_out ;
input           pciu_err_pending_in ;

output			pciu_conf_select_out ;
output  [11:0]  pciu_conf_offset_out ;
output          pciu_conf_renable_out ;
output          pciu_conf_wenable_out ;
output  [3:0]   pciu_conf_be_out ;
output  [31:0]  pciu_conf_data_out ;

output			pciu_pci_drcomp_pending_out ;
output			pciu_pciw_fifo_empty_out ;


// pci target state machine and interface outputs
wire        pcit_sm_trdy_out ;
wire        pcit_sm_stop_out ;
wire        pcit_sm_devsel_out ;
wire        pcit_sm_trdy_en_out ;
wire        pcit_sm_stop_en_out ;
wire        pcit_sm_devsel_en_out ;
wire		pcit_sm_target_load_out ;
wire [31:0] pcit_sm_ad_out ;
wire        pcit_sm_ad_en_out ;
wire [31:0] pcit_sm_address_out ;
wire  [3:0] pcit_sm_bc_out ;
wire		pcit_sm_bc0_out ;
wire [31:0] pcit_sm_data_out ;
wire  [3:0] pcit_sm_be_out ;
wire        pcit_sm_req_out ;
wire        pcit_sm_rdy_out ;
wire		pcit_sm_addr_phase_out ;
wire		pcit_sm_bckp_trdy_out ;
wire		pcit_sm_last_reg_out ;
wire		pcit_sm_frame_reg_out ;
wire		pcit_sm_fetch_pcir_fifo_out ;
wire		pcit_sm_load_medium_reg_out ;
wire		pcit_sm_sel_fifo_mreg_out ;
wire		pcit_sm_sel_conf_fifo_out ;
wire		pcit_sm_fetch_conf_out ;
wire		pcit_sm_load_to_pciw_fifo_out ;
wire		pcit_sm_load_to_conf_out ;

wire		pcit_sm_target_abort_set_out ; // to conf space

assign	pciu_pciif_trdy_out           	=	pcit_sm_trdy_out ;     
assign	pciu_pciif_stop_out           	=	pcit_sm_stop_out ;     
assign	pciu_pciif_devsel_out         	=	pcit_sm_devsel_out ;   
assign	pciu_pciif_trdy_en_out        	=	pcit_sm_trdy_en_out ;  
assign	pciu_pciif_stop_en_out        	=	pcit_sm_stop_en_out ;  
assign	pciu_pciif_devsel_en_out      	=	pcit_sm_devsel_en_out ;
assign	pciu_pciif_target_load_out		=	pcit_sm_target_load_out ;
assign	pciu_pciif_ad_out             	=	pcit_sm_ad_out ;       
assign	pciu_pciif_ad_en_out          	=	pcit_sm_ad_en_out ;    
assign	pciu_pciif_tabort_set_out		=	pcit_sm_target_abort_set_out ;

wire        pcit_if_addr_claim_out ;
wire [31:0] pcit_if_data_out ;
wire        pcit_if_same_read_out ;
wire        pcit_if_norm_access_to_config_out ;
wire        pcit_if_read_completed_out ;
wire        pcit_if_read_processing_out ;
wire        pcit_if_target_abort_out ;
wire        pcit_if_disconect_wo_data_out ;
wire        pcit_if_pciw_fifo_full_out ;
wire        pcit_if_pcir_fifo_data_err_out ;
wire        pcit_if_wbw_fifo_empty_out ;
wire        pcit_if_req_out ;
wire        pcit_if_done_out ;
wire        pcit_if_in_progress_out ;
wire [31:0] pcit_if_addr_out ;
wire  [3:0] pcit_if_be_out ;
wire        pcit_if_we_out ;
wire  [3:0] pcit_if_bc_out ;
wire        pcit_if_burst_out ;
wire        pcit_if_pcir_fifo_renable_out ;
wire        pcit_if_pcir_fifo_flush_out ;
wire        pcit_if_pciw_fifo_wenable_out ;
wire [31:0] pcit_if_pciw_fifo_addr_data_out ;
wire  [3:0] pcit_if_pciw_fifo_cbe_out ;
wire  [3:0] pcit_if_pciw_fifo_control_out ;
wire        pcit_if_conf_hit_out ;
wire [11:0] pcit_if_conf_addr_out ;
wire [31:0] pcit_if_conf_data_out ;
wire  [3:0] pcit_if_conf_be_out ;
wire        pcit_if_conf_we_out ;
wire        pcit_if_conf_re_out ;

// pci target state machine outputs
// pci interface signals
assign	pciu_conf_select_out	=	pcit_if_conf_hit_out ;
assign	pciu_conf_offset_out	=	pcit_if_conf_addr_out ;
assign	pciu_conf_renable_out	=	pcit_if_conf_re_out ;
assign	pciu_conf_wenable_out	=	pcit_if_conf_we_out ;
assign	pciu_conf_be_out		=	pcit_if_conf_be_out ;
assign	pciu_conf_data_out		=	pcit_if_conf_data_out ;

// wishbone master state machine outputs
wire        wbm_sm_wb_read_done	;
wire        wbm_sm_pcir_fifo_wenable_out ;
wire [31:0] wbm_sm_pcir_fifo_data_out ;
wire  [3:0] wbm_sm_pcir_fifo_be_out ;
wire  [3:0] wbm_sm_pcir_fifo_control_out ;
wire        wbm_sm_pciw_fifo_renable_out ;
wire        wbm_sm_pci_error_sig_out ;
wire  [3:0] wbm_sm_pci_error_bc ;
wire        wbm_sm_write_rty_cnt_exp_out ;
wire        wbm_sm_read_rty_cnt_exp_out ;
wire        wbm_sm_cyc_out ;
wire        wbm_sm_stb_out ;
wire        wbm_sm_we_out ;
wire  [3:0] wbm_sm_sel_out ;
wire [31:0] wbm_sm_adr_out ;
wire [31:0] wbm_sm_mdata_out ;
wire        wbm_sm_cab_out ;

assign	pciu_err_addr_out		=	wbm_sm_adr_out ;
assign	pciu_err_bc_out			=	wbm_sm_pci_error_bc ;
assign	pciu_err_data_out		=	wbm_sm_mdata_out ;
assign	pciu_err_be_out			=	~wbm_sm_sel_out ;
assign	pciu_err_signal_out		=	wbm_sm_pci_error_sig_out ;
assign	pciu_err_source_out		=	wbm_sm_write_rty_cnt_exp_out ; // only for writing to WB !
assign	pciu_err_rty_exp_out	=	wbm_sm_write_rty_cnt_exp_out ;

assign	ADR_O  		=	wbm_sm_adr_out ;
assign	MDATA_O		=	wbm_sm_mdata_out ;
assign	CYC_O  		=	wbm_sm_cyc_out ;
assign	STB_O  		=	wbm_sm_stb_out ;
assign	WE_O   		=	wbm_sm_we_out ;
assign	SEL_O  		=	wbm_sm_sel_out ;
assign	CAB_O  		=	wbm_sm_cab_out ;

// pciw_pcir fifo outputs

// pciw_fifo_outputs:
wire [31:0] fifos_pciw_addr_data_out ;
wire [3:0]  fifos_pciw_cbe_out ;
wire [3:0]  fifos_pciw_control_out ;
wire		fifos_pciw_two_left_out ;
wire        fifos_pciw_almost_full_out ;
wire        fifos_pciw_full_out ;
wire		fifos_pciw_almost_empty_out ;
wire        fifos_pciw_empty_out ;
wire        fifos_pciw_transaction_ready_out ;

assign	pciu_pciw_fifo_empty_out = fifos_pciw_empty_out ;

// pcir_fifo_outputs
wire [31:0] fifos_pcir_data_out ;
wire [3:0]  fifos_pcir_be_out ;
wire [3:0]  fifos_pcir_control_out ;
wire        fifos_pcir_almost_full_out ;
wire        fifos_pcir_full_out ;
wire        fifos_pcir_almost_empty_out ;
wire		fifos_pcir_empty_out ;

// delayed transaction logic outputs
wire [31:0] del_sync_addr_out ;
wire [3:0]  del_sync_be_out ;
wire        del_sync_we_out ;
wire        del_sync_comp_req_pending_out ;
wire        del_sync_comp_comp_pending_out ;
wire        del_sync_req_req_pending_out ;
wire        del_sync_req_comp_pending_out ;
wire [3:0]  del_sync_bc_out ;
wire        del_sync_status_out ;
wire        del_sync_comp_flush_out ;
wire        del_sync_burst_out ;

assign	pciu_pci_drcomp_pending_out = del_sync_comp_comp_pending_out ;

// WISHBONE master interface inputs            
wire		wbm_sm_pci_tar_read_request				=	del_sync_comp_req_pending_out ;
wire [31:0] wbm_sm_pci_tar_address                  =	del_sync_addr_out ;
wire  [3:0] wbm_sm_pci_tar_cmd                      =	del_sync_bc_out ;
wire  [3:0] wbm_sm_pci_tar_be                       =	del_sync_be_out ;
wire		wbm_sm_pci_tar_prefetch_en              =	del_sync_burst_out ;
wire  [7:0] wbm_sm_pci_cache_line_size              =	pciu_cache_line_size_in ;
wire		wbm_sm_pcir_fifo_almost_full_in         =	fifos_pcir_almost_full_out ;
wire		wbm_sm_pcir_fifo_full_in                =	fifos_pcir_full_out ;
wire [31:0] wbm_sm_pciw_fifo_addr_data_in           =	fifos_pciw_addr_data_out ;
wire  [3:0] wbm_sm_pciw_fifo_cbe_in                 =	fifos_pciw_cbe_out ;
wire  [3:0] wbm_sm_pciw_fifo_control_in             =	fifos_pciw_control_out ;
wire		wbm_sm_pciw_fifo_almost_empty_in        =	fifos_pciw_almost_empty_out ;
wire		wbm_sm_pciw_fifo_empty_in               =	fifos_pciw_empty_out ;
wire		wbm_sm_pciw_fifo_transaction_ready_in	=	fifos_pciw_transaction_ready_out ;
wire		wbm_sm_pci_error_sig_set_in             =	pciu_err_pending_in ;
wire [31:0] wbm_sm_mdata_in                         =	MDATA_I ;
wire		wbm_sm_ack_in                           =	ACK_I ;
wire		wbm_sm_rty_in                           =	RTY_I ;
wire		wbm_sm_err_in                           =	ERR_I ;

// WISHBONE master interface instantiation
WB_MASTER wishbone_master
(
	.wb_clock_in					(wb_clock_in),
	.reset_in						(reset_in),
	.pci_tar_read_request			(wbm_sm_pci_tar_read_request),	//in
	.pci_tar_address				(wbm_sm_pci_tar_address),		//in
	.pci_tar_cmd					(wbm_sm_pci_tar_cmd),			//in
	.pci_tar_be						(wbm_sm_pci_tar_be),			//in
	.pci_tar_prefetch_en			(wbm_sm_pci_tar_prefetch_en),	//in
	.pci_cache_line_size			(wbm_sm_pci_cache_line_size),	//in
	.wb_read_done					(wbm_sm_wb_read_done),			//out
	.pcir_fifo_wenable_out			(wbm_sm_pcir_fifo_wenable_out),
	.pcir_fifo_data_out				(wbm_sm_pcir_fifo_data_out),
	.pcir_fifo_be_out				(wbm_sm_pcir_fifo_be_out),
	.pcir_fifo_control_out			(wbm_sm_pcir_fifo_control_out),
	.pcir_fifo_almost_full_in		(wbm_sm_pcir_fifo_almost_full_in),
	.pcir_fifo_full_in				(wbm_sm_pcir_fifo_full_in),
	.pciw_fifo_renable_out			(wbm_sm_pciw_fifo_renable_out),
	.pciw_fifo_addr_data_in			(wbm_sm_pciw_fifo_addr_data_in),
	.pciw_fifo_cbe_in				(wbm_sm_pciw_fifo_cbe_in),
	.pciw_fifo_control_in			(wbm_sm_pciw_fifo_control_in),
	.pciw_fifo_almost_empty_in		(wbm_sm_pciw_fifo_almost_empty_in),
	.pciw_fifo_empty_in				(wbm_sm_pciw_fifo_empty_in),
	.pciw_fifo_transaction_ready_in	(wbm_sm_pciw_fifo_transaction_ready_in),
	.pci_error_sig_set_in			(wbm_sm_pci_error_sig_set_in),
	.pci_error_sig_out				(wbm_sm_pci_error_sig_out),
	.pci_error_bc					(wbm_sm_pci_error_bc),
	.write_rty_cnt_exp_out			(wbm_sm_write_rty_cnt_exp_out),
	.read_rty_cnt_exp_out			(wbm_sm_read_rty_cnt_exp_out),
	.CYC_O							(wbm_sm_cyc_out),
	.STB_O							(wbm_sm_stb_out),
	.WE_O							(wbm_sm_we_out),
	.SEL_O							(wbm_sm_sel_out),
	.ADR_O							(wbm_sm_adr_out),
	.MDATA_I						(wbm_sm_mdata_in),
	.MDATA_O						(wbm_sm_mdata_out),
	.ACK_I							(wbm_sm_ack_in),
	.RTY_I							(wbm_sm_rty_in),
	.ERR_I							(wbm_sm_err_in),
	.CAB_O                 			(wbm_sm_cab_out)
);

// pciw_pcir_fifos inputs
// PCIW_FIFO inputs
wire        fifos_pciw_wenable_in   	=	pcit_if_pciw_fifo_wenable_out ;
wire [31:0] fifos_pciw_addr_data_in     =	pcit_if_pciw_fifo_addr_data_out ;
wire [3:0]  fifos_pciw_cbe_in           =	pcit_if_pciw_fifo_cbe_out ;
wire [3:0]  fifos_pciw_control_in       =	pcit_if_pciw_fifo_control_out ;
wire        fifos_pciw_renable_in       =	wbm_sm_pciw_fifo_renable_out ;
wire        fifos_pciw_flush_in         =	1'b0 ;

// PCIR_FIFO inputs
wire        fifos_pcir_wenable_in    	=	wbm_sm_pcir_fifo_wenable_out ;
wire [31:0] fifos_pcir_data_in          =	wbm_sm_pcir_fifo_data_out ;
wire [3:0]  fifos_pcir_be_in            =	wbm_sm_pcir_fifo_be_out ;
wire [3:0]  fifos_pcir_control_in       =	wbm_sm_pcir_fifo_control_out ;
wire        fifos_pcir_renable_in       =	pcit_if_pcir_fifo_renable_out ;
wire        fifos_pcir_flush_in         =	pcit_if_pcir_fifo_flush_out ;

// PCIW_FIFO and PCIR_FIFO instantiation
PCIW_PCIR_FIFOS fifos
(
	.wb_clock_in                (wb_clock_in),
	.pci_clock_in               (pci_clock_in),
	.reset_in                   (reset_in),
	.pciw_wenable_in            (fifos_pciw_wenable_in),      //for PCI Target !!!
	.pciw_addr_data_in          (fifos_pciw_addr_data_in),    //for PCI Target !!!
	.pciw_cbe_in                (fifos_pciw_cbe_in),          //for PCI Target !!!
	.pciw_control_in            (fifos_pciw_control_in),      //for PCI Target !!!
	.pciw_renable_in            (fifos_pciw_renable_in),      
	.pciw_addr_data_out         (fifos_pciw_addr_data_out),   
	.pciw_cbe_out               (fifos_pciw_cbe_out),         
	.pciw_control_out           (fifos_pciw_control_out), 
	.pciw_flush_in    			(fifos_pciw_flush_in),       
	.pciw_two_left_out			(fifos_pciw_two_left_out),	  //for PCI Target !!!
	.pciw_almost_full_out       (fifos_pciw_almost_full_out), //for PCI Target !!!
	.pciw_full_out              (fifos_pciw_full_out),        //for PCI Target !!!
	.pciw_almost_empty_out      (fifos_pciw_almost_empty_out),
	.pciw_empty_out             (fifos_pciw_empty_out),
	.pciw_transaction_ready_out (fifos_pciw_transaction_ready_out),
	.pcir_wenable_in            (fifos_pcir_wenable_in),
	.pcir_data_in               (fifos_pcir_data_in), 
	.pcir_be_in                 (fifos_pcir_be_in), 
	.pcir_control_in            (fifos_pcir_control_in), 
	.pcir_renable_in            (fifos_pcir_renable_in),      //for PCI Target !!!
	.pcir_data_out              (fifos_pcir_data_out),        //for PCI Target !!!
	.pcir_be_out                (fifos_pcir_be_out),          //for PCI Target !!!
	.pcir_control_out           (fifos_pcir_control_out),     //for PCI Target !!!
	.pcir_flush_in              (fifos_pcir_flush_in),        //for PCI Target !!!
	.pcir_almost_full_out 		(fifos_pcir_almost_full_out),
	.pcir_full_out  			(fifos_pcir_full_out),
	.pcir_almost_empty_out 		(fifos_pcir_almost_empty_out), //for PCI Target !!!
	.pcir_empty_out				(fifos_pcir_empty_out),		   //for PCI Target !!!
	.pcir_transaction_ready_out	()
) ;

// delayed transaction logic inputs
wire        del_sync_req_in             =	pcit_if_req_out ;              
wire        del_sync_comp_in            =	wbm_sm_wb_read_done ;
wire        del_sync_done_in            =	pcit_if_done_out ;
wire        del_sync_in_progress_in     =	pcit_if_in_progress_out ;
wire [31:0] del_sync_addr_in            =	pcit_if_addr_out ;
wire  [3:0] del_sync_be_in              =	pcit_if_be_out ;
wire        del_sync_we_in              =	pcit_if_we_out ;
wire  [3:0] del_sync_bc_in              =	pcit_if_bc_out ;
wire        del_sync_status_in          =	1'b0 ;
wire        del_sync_burst_in           =	pcit_if_burst_out ;
wire        del_sync_retry_expired_in   =	wbm_sm_read_rty_cnt_exp_out ;

// delayed transaction logic instantiation
DELAYED_SYNC				del_sync  
(
	.reset_in             	(reset_in),
	.req_clk_in           	(pci_clock_in),
	.comp_clk_in          	(wb_clock_in),
	.req_in               	(del_sync_req_in),
	.comp_in              	(del_sync_comp_in),
	.done_in              	(del_sync_done_in),
	.in_progress_in       	(del_sync_in_progress_in),
	.comp_req_pending_out 	(del_sync_comp_req_pending_out),
	.comp_comp_pending_out	(del_sync_comp_comp_pending_out),
	.req_req_pending_out  	(del_sync_req_req_pending_out),
	.req_comp_pending_out 	(del_sync_req_comp_pending_out),
	.addr_in              	(del_sync_addr_in),
	.be_in                	(del_sync_be_in),
	.addr_out             	(del_sync_addr_out),
	.be_out               	(del_sync_be_out),
	.we_in                	(del_sync_we_in),
	.we_out               	(del_sync_we_out),
	.bc_in                	(del_sync_bc_in),
	.bc_out               	(del_sync_bc_out),
	.status_in            	(del_sync_status_in),
	.status_out           	(del_sync_status_out),
	.comp_flush_out       	(del_sync_comp_flush_out),
	.burst_in             	(del_sync_burst_in),
	.burst_out            	(del_sync_burst_out),
	.retry_expired_in     	(del_sync_retry_expired_in)
);

// pci target interface inputs
wire [31:0] pcit_if_address_in						=	pcit_sm_address_out ;
wire  [3:0] pcit_if_bc_in							=	pcit_sm_bc_out ;
wire		pcit_if_bc0_in							=	pcit_sm_bc0_out ;
wire [31:0] pcit_if_data_in							=	pcit_sm_data_out ;
wire  [3:0] pcit_if_be_in							=	pcit_sm_be_out ;
wire		pcit_if_req_in							=	pcit_sm_req_out ;
wire		pcit_if_rdy_in							=	pcit_sm_rdy_out ;
wire		pcit_if_addr_phase_in					=	pcit_sm_addr_phase_out ;
wire		pcit_if_bckp_trdy_in					=	pcit_sm_bckp_trdy_out ;
wire		pcit_if_last_reg_in						=	pcit_sm_last_reg_out ;
wire		pcit_if_frame_reg_in					=	pcit_sm_frame_reg_out ;
wire		pcit_if_fetch_pcir_fifo_in				=	pcit_sm_fetch_pcir_fifo_out ;
wire		pcit_if_load_medium_reg_in				=	pcit_sm_load_medium_reg_out ;
wire		pcit_if_sel_fifo_mreg_in  				=	pcit_sm_sel_fifo_mreg_out ;
wire		pcit_if_sel_conf_fifo_in  				=	pcit_sm_sel_conf_fifo_out ;
wire		pcit_if_fetch_conf_in     				=	pcit_sm_fetch_conf_out ;
wire		pcit_if_load_to_pciw_fifo_in			=	pcit_sm_load_to_pciw_fifo_out ;
wire		pcit_if_load_to_conf_in   				=	pcit_sm_load_to_conf_out ;
wire		pcit_if_req_req_pending_in				=	del_sync_req_req_pending_out ;
wire		pcit_if_req_comp_pending_in				=	del_sync_req_comp_pending_out ;
wire        pcit_if_status_in						=	del_sync_status_out ;
wire [31:0]	pcit_if_strd_addr_in					=	del_sync_addr_out ;
wire  [3:0]	pcit_if_strd_bc_in						=	del_sync_bc_out ;
wire		pcit_if_comp_flush_in					=	del_sync_comp_flush_out ;
wire [31:0] pcit_if_pcir_fifo_data_in				=	fifos_pcir_data_out ;
wire  [3:0] pcit_if_pcir_fifo_be_in					=	fifos_pcir_be_out ;
wire  [3:0] pcit_if_pcir_fifo_control_in			=	fifos_pcir_control_out ;
wire		pcit_if_pcir_fifo_almost_empty_in		=	fifos_pcir_almost_empty_out ;
wire		pcit_if_pcir_fifo_empty_in				=	fifos_pcir_empty_out ;
wire		pcit_if_pciw_fifo_two_left_in			=	fifos_pciw_two_left_out ;
wire		pcit_if_pciw_fifo_almost_full_in		=	fifos_pciw_almost_full_out ;
wire		pcit_if_pciw_fifo_full_in				=	fifos_pciw_full_out ;
wire		pcit_if_wbw_fifo_empty_in				=	pciu_wbw_fifo_empty_in ;
wire [31:0] pcit_if_conf_data_in					=	pciu_conf_data_in ;
wire		pcit_if_mem_enable_in					=	pciu_mem_enable_in ;
wire		pcit_if_io_enable_in					=	pciu_io_enable_in ;
wire		pcit_if_mem_io_addr_space0_in			=	pciu_map_in[0] ;
wire		pcit_if_mem_io_addr_space1_in			=	pciu_map_in[1] ;
wire		pcit_if_mem_io_addr_space2_in			=	pciu_map_in[2] ;
wire		pcit_if_mem_io_addr_space3_in			=	pciu_map_in[3] ;
wire		pcit_if_mem_io_addr_space4_in			=	pciu_map_in[4] ;
wire		pcit_if_mem_io_addr_space5_in			=	pciu_map_in[5] ;
wire		pcit_if_pre_fetch_en0_in				=	pciu_pref_en_in[0] ;
wire		pcit_if_pre_fetch_en1_in				=	pciu_pref_en_in[1] ;
wire		pcit_if_pre_fetch_en2_in				=	pciu_pref_en_in[2] ;
wire		pcit_if_pre_fetch_en3_in				=	pciu_pref_en_in[3] ;
wire		pcit_if_pre_fetch_en4_in				=	pciu_pref_en_in[4] ;
wire		pcit_if_pre_fetch_en5_in				=	pciu_pref_en_in[5] ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_base_addr0_in	=	pciu_bar0_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_base_addr1_in	=	pciu_bar1_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_base_addr2_in	=	pciu_bar2_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_base_addr3_in	=	pciu_bar3_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_base_addr4_in	=	pciu_bar4_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_base_addr5_in	=	pciu_bar5_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_addr_mask0_in	=	pciu_am0_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_addr_mask1_in	=	pciu_am1_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_addr_mask2_in	=	pciu_am2_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_addr_mask3_in	=	pciu_am3_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_addr_mask4_in	=	pciu_am4_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_addr_mask5_in	=	pciu_am5_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_tran_addr0_in	=	pciu_ta0_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_tran_addr1_in	=	pciu_ta1_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_tran_addr2_in	=	pciu_ta2_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_tran_addr3_in	=	pciu_ta3_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_tran_addr4_in	=	pciu_ta4_in ;
wire [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] pcit_if_pci_tran_addr5_in	=	pciu_ta5_in ;
wire		pcit_if_addr_tran_en0_in				=	pciu_at_en_in[0] ;
wire		pcit_if_addr_tran_en1_in				=	pciu_at_en_in[1] ;
wire		pcit_if_addr_tran_en2_in				=	pciu_at_en_in[2] ;
wire		pcit_if_addr_tran_en3_in				=	pciu_at_en_in[3] ;
wire		pcit_if_addr_tran_en4_in				=	pciu_at_en_in[4] ;
wire		pcit_if_addr_tran_en5_in				=	pciu_at_en_in[5] ;

PCI_TARGET32_INTERFACE				pci_target_if
(
    .clk_in							(pci_clock_in),
    .reset_in						(reset_in),
    .address_in						(pcit_if_address_in),
    .addr_claim_out					(pcit_if_addr_claim_out),
    .bc_in							(pcit_if_bc_in),
    .bc0_in							(pcit_if_bc0_in),
    .data_in						(pcit_if_data_in),
    .data_out						(pcit_if_data_out),
    .be_in							(pcit_if_be_in),
    .req_in							(pcit_if_req_in),
    .rdy_in							(pcit_if_rdy_in),
    .addr_phase_in					(pcit_if_addr_phase_in),
    .bckp_trdy_in					(pcit_if_bckp_trdy_in),
    .last_reg_in					(pcit_if_last_reg_in),
    .frame_reg_in					(pcit_if_frame_reg_in),
    .fetch_pcir_fifo_in				(pcit_if_fetch_pcir_fifo_in),
    .load_medium_reg_in				(pcit_if_load_medium_reg_in),
    .sel_fifo_mreg_in  				(pcit_if_sel_fifo_mreg_in),
    .sel_conf_fifo_in  				(pcit_if_sel_conf_fifo_in),
    .fetch_conf_in     				(pcit_if_fetch_conf_in),
    .load_to_pciw_fifo_in			(pcit_if_load_to_pciw_fifo_in),
    .load_to_conf_in   				(pcit_if_load_to_conf_in),
    .same_read_out					(pcit_if_same_read_out),
	.norm_access_to_config_out		(pcit_if_norm_access_to_config_out),
	.read_completed_out				(pcit_if_read_completed_out),
	.read_processing_out			(pcit_if_read_processing_out),
	.target_abort_out				(pcit_if_target_abort_out),
	.disconect_wo_data_out			(pcit_if_disconect_wo_data_out),
	.pciw_fifo_full_out				(pcit_if_pciw_fifo_full_out),
	.pcir_fifo_data_err_out			(pcit_if_pcir_fifo_data_err_out),
	.wbw_fifo_empty_out				(pcit_if_wbw_fifo_empty_out),
	.req_out						(pcit_if_req_out),       
    .done_out						(pcit_if_done_out),      
    .in_progress_out				(pcit_if_in_progress_out),
	.req_req_pending_in				(pcit_if_req_req_pending_in), 
    .req_comp_pending_in			(pcit_if_req_comp_pending_in),
	.addr_out						(pcit_if_addr_out), 
    .be_out							(pcit_if_be_out),   
    .we_out							(pcit_if_we_out),   
    .bc_out							(pcit_if_bc_out), 
    .burst_out						(pcit_if_burst_out),
    .strd_addr_in					(pcit_if_strd_addr_in),
    .strd_bc_in						(pcit_if_strd_bc_in),
    .status_in						(pcit_if_status_in),
    .comp_flush_in					(pcit_if_comp_flush_in),
	.pcir_fifo_renable_out			(pcit_if_pcir_fifo_renable_out),  
	.pcir_fifo_data_in				(pcit_if_pcir_fifo_data_in),   	
	.pcir_fifo_be_in				(pcit_if_pcir_fifo_be_in), 		
	.pcir_fifo_control_in			(pcit_if_pcir_fifo_control_in),
	.pcir_fifo_flush_out			(pcit_if_pcir_fifo_flush_out), 	
	.pcir_fifo_almost_empty_in		(pcit_if_pcir_fifo_almost_empty_in), 	    
	.pcir_fifo_empty_in				(pcit_if_pcir_fifo_empty_in),
	.pciw_fifo_wenable_out			(pcit_if_pciw_fifo_wenable_out), 	    
	.pciw_fifo_addr_data_out		(pcit_if_pciw_fifo_addr_data_out),	
	.pciw_fifo_cbe_out				(pcit_if_pciw_fifo_cbe_out), 		    
	.pciw_fifo_control_out			(pcit_if_pciw_fifo_control_out),	    
	.pciw_fifo_two_left_in			(pcit_if_pciw_fifo_two_left_in),
	.pciw_fifo_almost_full_in		(pcit_if_pciw_fifo_almost_full_in),
	.pciw_fifo_full_in				(pcit_if_pciw_fifo_full_in),
	.wbw_fifo_empty_in				(pcit_if_wbw_fifo_empty_in),
	.conf_hit_out					(pcit_if_conf_hit_out),
	.conf_addr_out					(pcit_if_conf_addr_out),
	.conf_data_out					(pcit_if_conf_data_out),
	.conf_data_in					(pcit_if_conf_data_in),
	.conf_be_out					(pcit_if_conf_be_out),
	.conf_we_out					(pcit_if_conf_we_out),
	.conf_re_out					(pcit_if_conf_re_out),
	.mem_enable_in					(pcit_if_mem_enable_in),
	.io_enable_in					(pcit_if_io_enable_in),
	.mem_io_addr_space0_in			(pcit_if_mem_io_addr_space0_in),
	.mem_io_addr_space1_in			(pcit_if_mem_io_addr_space1_in),
	.mem_io_addr_space2_in			(pcit_if_mem_io_addr_space2_in),
	.mem_io_addr_space3_in			(pcit_if_mem_io_addr_space3_in),
	.mem_io_addr_space4_in			(pcit_if_mem_io_addr_space4_in),
	.mem_io_addr_space5_in			(pcit_if_mem_io_addr_space5_in),
	.pre_fetch_en0_in				(pcit_if_pre_fetch_en0_in),
	.pre_fetch_en1_in				(pcit_if_pre_fetch_en1_in),
	.pre_fetch_en2_in				(pcit_if_pre_fetch_en2_in),
	.pre_fetch_en3_in				(pcit_if_pre_fetch_en3_in),
	.pre_fetch_en4_in				(pcit_if_pre_fetch_en4_in),
	.pre_fetch_en5_in				(pcit_if_pre_fetch_en5_in),
	.pci_base_addr0_in				(pcit_if_pci_base_addr0_in),
	.pci_base_addr1_in				(pcit_if_pci_base_addr1_in),
	.pci_base_addr2_in				(pcit_if_pci_base_addr2_in),
	.pci_base_addr3_in				(pcit_if_pci_base_addr3_in),
	.pci_base_addr4_in				(pcit_if_pci_base_addr4_in),
	.pci_base_addr5_in				(pcit_if_pci_base_addr5_in),
	.pci_addr_mask0_in				(pcit_if_pci_addr_mask0_in),
	.pci_addr_mask1_in				(pcit_if_pci_addr_mask1_in),
	.pci_addr_mask2_in				(pcit_if_pci_addr_mask2_in),
	.pci_addr_mask3_in				(pcit_if_pci_addr_mask3_in),
	.pci_addr_mask4_in				(pcit_if_pci_addr_mask4_in),
	.pci_addr_mask5_in				(pcit_if_pci_addr_mask5_in),
	.pci_tran_addr0_in				(pcit_if_pci_tran_addr0_in),
	.pci_tran_addr1_in				(pcit_if_pci_tran_addr1_in),
	.pci_tran_addr2_in				(pcit_if_pci_tran_addr2_in),
	.pci_tran_addr3_in				(pcit_if_pci_tran_addr3_in),
	.pci_tran_addr4_in				(pcit_if_pci_tran_addr4_in),
	.pci_tran_addr5_in				(pcit_if_pci_tran_addr5_in),
	.addr_tran_en0_in				(pcit_if_addr_tran_en0_in),
	.addr_tran_en1_in				(pcit_if_addr_tran_en1_in),
	.addr_tran_en2_in				(pcit_if_addr_tran_en2_in),
	.addr_tran_en3_in				(pcit_if_addr_tran_en3_in),
	.addr_tran_en4_in				(pcit_if_addr_tran_en4_in),
	.addr_tran_en5_in				(pcit_if_addr_tran_en5_in)
) ;

// pci target state machine inputs
wire		pcit_sm_frame_in					=	pciu_pciif_frame_in ;
wire		pcit_sm_irdy_in                     =	pciu_pciif_irdy_in ;
wire		pcit_sm_idsel_in                    =	pciu_pciif_idsel_in ;
wire		pcit_sm_frame_reg_in                =	pciu_pciif_frame_reg_in ;
wire		pcit_sm_irdy_reg_in                 =	pciu_pciif_irdy_reg_in ;
wire		pcit_sm_idsel_reg_in                =	pciu_pciif_idsel_reg_in ;
wire [31:0] pcit_sm_ad_reg_in                   =	pciu_pciif_ad_reg_in ;
wire  [3:0] pcit_sm_cbe_reg_in                  =	pciu_pciif_cbe_reg_in ;
wire		pcit_sm_bckp_trdy_en_in				=	pciu_pciif_bckp_trdy_en_in ;
wire		pcit_sm_bckp_devsel_in				=	pciu_pciif_bckp_devsel_in ;
wire		pcit_sm_bckp_trdy_in				=	pciu_pciif_bckp_trdy_in ;
wire		pcit_sm_bckp_stop_in				=	pciu_pciif_bckp_stop_in ;
wire		pcit_sm_addr_claim_in               =	pcit_if_addr_claim_out ;
wire [31:0] pcit_sm_data_in                     =	pcit_if_data_out ;
wire		pcit_sm_same_read_in                =	pcit_if_same_read_out ;
wire		pcit_sm_norm_access_to_config_in    =	pcit_if_norm_access_to_config_out ;
wire		pcit_sm_read_completed_in           =	pcit_if_read_completed_out ;
wire		pcit_sm_read_processing_in          =	pcit_if_read_processing_out ;
wire		pcit_sm_target_abort_in             =	pcit_if_target_abort_out ;
wire		pcit_sm_disconect_wo_data_in        =	pcit_if_disconect_wo_data_out ;
wire		pcit_sm_pciw_fifo_full_in           =	pcit_if_pciw_fifo_full_out ;
wire		pcit_sm_pcir_fifo_data_err_in       =	pcit_if_pcir_fifo_data_err_out ;
wire		pcit_sm_wbw_fifo_empty_in           =	pcit_if_wbw_fifo_empty_out ;
wire		pcit_sm_wbu_frame_en_in				=	pciu_wbu_frame_en_in ;

PCI_TARGET32_SM					pci_target_sm
(
    .clk_in						(pci_clock_in),
    .reset_in					(reset_in),
    .pci_frame_in				(pcit_sm_frame_in),
    .pci_irdy_in				(pcit_sm_irdy_in),
    .pci_idsel_in				(pcit_sm_idsel_in),
    .pci_frame_reg_in			(pcit_sm_frame_reg_in),
    .pci_irdy_reg_in			(pcit_sm_irdy_reg_in),
    .pci_idsel_reg_in			(pcit_sm_idsel_reg_in),
    .pci_trdy_out				(pcit_sm_trdy_out),
    .pci_stop_out				(pcit_sm_stop_out),
    .pci_devsel_out				(pcit_sm_devsel_out),
    .pci_trdy_en_out			(pcit_sm_trdy_en_out),
    .pci_stop_en_out			(pcit_sm_stop_en_out),
    .pci_devsel_en_out			(pcit_sm_devsel_en_out),
    .pci_target_load_out		(pcit_sm_target_load_out),
    .pci_ad_reg_in				(pcit_sm_ad_reg_in),
    .pci_ad_out					(pcit_sm_ad_out),
    .pci_ad_en_out				(pcit_sm_ad_en_out),
    .pci_cbe_reg_in				(pcit_sm_cbe_reg_in),
    .bckp_trdy_en_in			(pcit_sm_bckp_trdy_en_in),
    .bckp_devsel_in				(pcit_sm_bckp_devsel_in),
    .bckp_trdy_in				(pcit_sm_bckp_trdy_in),
    .bckp_stop_in				(pcit_sm_bckp_stop_in),
    .address_out				(pcit_sm_address_out),
    .addr_claim_in				(pcit_sm_addr_claim_in),
    .bc_out						(pcit_sm_bc_out),
    .bc0_out					(pcit_sm_bc0_out),
    .data_out					(pcit_sm_data_out),
    .data_in					(pcit_sm_data_in),
    .be_out						(pcit_sm_be_out),
    .req_out					(pcit_sm_req_out),
    .rdy_out					(pcit_sm_rdy_out),
    .addr_phase_out				(pcit_sm_addr_phase_out),
    .bckp_trdy_out				(pcit_sm_bckp_trdy_out),
    .last_reg_out				(pcit_sm_last_reg_out),
    .frame_reg_out				(pcit_sm_frame_reg_out),
    .fetch_pcir_fifo_out		(pcit_sm_fetch_pcir_fifo_out),
    .load_medium_reg_out		(pcit_sm_load_medium_reg_out),
    .sel_fifo_mreg_out			(pcit_sm_sel_fifo_mreg_out),
    .sel_conf_fifo_out			(pcit_sm_sel_conf_fifo_out),
    .fetch_conf_out				(pcit_sm_fetch_conf_out),
    .load_to_pciw_fifo_out		(pcit_sm_load_to_pciw_fifo_out),
    .load_to_conf_out			(pcit_sm_load_to_conf_out),
	.same_read_in				(pcit_sm_same_read_in),
	.norm_access_to_config_in	(pcit_sm_norm_access_to_config_in),
	.read_completed_in			(pcit_sm_read_completed_in),
	.read_processing_in			(pcit_sm_read_processing_in),
	.target_abort_in			(pcit_sm_target_abort_in),
	.disconect_wo_data_in		(pcit_sm_disconect_wo_data_in),
	.target_abort_set_out		(pcit_sm_target_abort_set_out),
	.pciw_fifo_full_in			(pcit_sm_pciw_fifo_full_in),
	.pcir_fifo_data_err_in		(pcit_sm_pcir_fifo_data_err_in),
	.wbw_fifo_empty_in          (pcit_sm_wbw_fifo_empty_in),
	.wbu_frame_en_in			(pcit_sm_wbu_frame_en_in)
) ;

endmodule