//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "pci_bridge32.v"                                  ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - Miha Dolenc (mihad@opencores.org)                     ////
////      - Tadej Markovic (tadej@opencores.org)                  ////
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
// Revision 1.2  2001/10/05 08:14:28  mihad
// Updated all files with inclusion of timescale file for simulation purposes.
//
// Revision 1.1.1.1  2001/10/02 15:33:46  mihad
// New project directory structure
//
//

`include "pci_constants.v"

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

// this is top level module of pci bridge core
// it instantiates and connects other lower level modules
// check polarity of PCI output enables in file out_reg.v and change it according to IO interface specification

module PCI_BRIDGE32
(
    // WISHBONE system signals
    CLK_I,
    RST_I,
    RST_O,
    INT_I,
    INT_O,

    // WISHBONE slave interface
    ADR_I,
    SDAT_I,
    SDAT_O,
    SEL_I,
    CYC_I,
    STB_I,
    WE_I,
    CAB_I,
    ACK_O,
    RTY_O,
    ERR_O,

    // WISHBONE master interface
    ADR_O,
    MDAT_I,
    MDAT_O,
    SEL_O,
    CYC_O,
    STB_O,
    WE_O,
    CAB_O,
    ACK_I,
    RTY_I,
    ERR_I,

    // pci interface - system pins
    PCI_CLK_IN,
    PCI_RSTn_IN,
    PCI_RSTn_OUT,
    PCI_INTAn_IN,
    PCI_INTAn_OUT,
    PCI_RSTn_EN_OUT,
    PCI_INTAn_EN_OUT,

    // arbitration pins
    PCI_REQn_OUT,
    PCI_REQn_EN_OUT,

    PCI_GNTn_IN,

    // protocol pins
    PCI_FRAMEn_IN,
    PCI_FRAMEn_OUT,
    PCI_FRAMEn_EN_OUT,
    PCI_IRDYn_EN_OUT,
    PCI_DEVSELn_EN_OUT,
    PCI_TRDYn_EN_OUT,
    PCI_STOPn_EN_OUT,
    PCI_AD_EN_OUT,
    PCI_CBEn_EN_OUT,

    PCI_IRDYn_IN,
    PCI_IRDYn_OUT,

    PCI_IDSEL_IN,

    PCI_DEVSELn_IN,
    PCI_DEVSELn_OUT,


    PCI_TRDYn_IN,
    PCI_TRDYn_OUT,

    PCI_STOPn_IN,
    PCI_STOPn_OUT,

    // data transfer pins
    PCI_AD_IN,
    PCI_AD_OUT,

    PCI_CBEn_IN,
    PCI_CBEn_OUT,

    // parity generation and checking pins
    PCI_PAR_IN,
    PCI_PAR_OUT,
    PCI_PAR_EN_OUT,

    PCI_PERRn_IN,
    PCI_PERRn_OUT,
    PCI_PERRn_EN_OUT,

    // system error pin
    PCI_SERRn_OUT,
    PCI_SERRn_EN_OUT
);

// WISHBONE system signals
input   CLK_I ;
input   RST_I ;
output  RST_O ;
input   INT_I ;
output  INT_O ;

// WISHBONE slave interface
input   [31:0]  ADR_I ;
input   [31:0]  SDAT_I ;
output  [31:0]  SDAT_O ;
input   [3:0]   SEL_I ;
input           CYC_I ;
input           STB_I ;
input           WE_I  ;
input           CAB_I ;
output          ACK_O ;
output          RTY_O ;
output          ERR_O ;

// WISHBONE master interface
output  [31:0]  ADR_O ;
input   [31:0]  MDAT_I ;
output  [31:0]  MDAT_O ;
output  [3:0]   SEL_O ;
output          CYC_O ;
output          STB_O ;
output          WE_O  ;
output          CAB_O ;
input           ACK_I ;
input           RTY_I ;
input           ERR_I ;

// pci interface - system pins
input   PCI_CLK_IN ;
input   PCI_RSTn_IN ;
output  PCI_RSTn_OUT ;
output  PCI_RSTn_EN_OUT ;

input   PCI_INTAn_IN ;
output  PCI_INTAn_OUT ;
output  PCI_INTAn_EN_OUT ;

// arbitration pins
output  PCI_REQn_OUT ;
output  PCI_REQn_EN_OUT ;

input   PCI_GNTn_IN ;

// protocol pins
input   PCI_FRAMEn_IN ;
output  PCI_FRAMEn_OUT ;
output  PCI_FRAMEn_EN_OUT ;
output  PCI_IRDYn_EN_OUT ;
output  PCI_DEVSELn_EN_OUT ;
output  PCI_TRDYn_EN_OUT ;
output  PCI_STOPn_EN_OUT ;
output  [31:0]  PCI_AD_EN_OUT ;
output  [3:0]   PCI_CBEn_EN_OUT ;

input   PCI_IRDYn_IN ;
output  PCI_IRDYn_OUT ;

input   PCI_IDSEL_IN ;

input   PCI_DEVSELn_IN ;
output  PCI_DEVSELn_OUT ;

input   PCI_TRDYn_IN ;
output  PCI_TRDYn_OUT ;

input   PCI_STOPn_IN ;
output  PCI_STOPn_OUT ;

// data transfer pins
input   [31:0]  PCI_AD_IN ;
output  [31:0]  PCI_AD_OUT ;

input   [3:0]   PCI_CBEn_IN ;
output  [3:0]   PCI_CBEn_OUT ;

// parity generation and checking pins
input   PCI_PAR_IN ;
output  PCI_PAR_OUT ;
output  PCI_PAR_EN_OUT ;

input   PCI_PERRn_IN ;
output  PCI_PERRn_OUT ;
output  PCI_PERRn_EN_OUT ;

// system error pin
output  PCI_SERRn_OUT ;
output  PCI_SERRn_EN_OUT ;

// declare clock and reset wires
wire pci_clk = PCI_CLK_IN ;
wire wb_clk  = CLK_I ;
wire reset ; // assigned at pci bridge reset and interrupt logic

/*=========================================================================================================
First comes definition of all modules' outputs, so they can be assigned to any other module's input later
  in the file, when module is instantiated
=========================================================================================================*/
// PCI BRIDGE RESET AND INTERRUPT LOGIC OUTPUTS
wire    pci_reso_reset ;
wire    pci_reso_pci_rstn_out ;
wire    pci_reso_pci_rstn_en_out ;
wire    pci_reso_rst_o ;
wire    pci_into_pci_intan_out ;
wire    pci_into_pci_intan_en_out ;
wire    pci_into_int_o ;
wire    pci_into_conf_isr_int_prop_out ;

// assign pci bridge reset interrupt logic outputs to top outputs where possible
assign reset            = pci_reso_reset ;
assign PCI_RSTn_OUT     = pci_reso_pci_rstn_out ;
assign PCI_RSTn_EN_OUT  = pci_reso_pci_rstn_en_out ;
assign RST_O            = pci_reso_rst_o ;
assign PCI_INTAn_OUT    = pci_into_pci_intan_out ;
assign PCI_INTAn_EN_OUT = pci_into_pci_intan_en_out ;
assign INT_O            = pci_into_int_o ;

// WISHBONE SLAVE UNIT OUTPUTS
wire    [31:0]  wbu_sdata_out ;
wire            wbu_ack_out ;
wire            wbu_rty_out ;
wire            wbu_err_out ;
wire            wbu_pciif_req_out ;
wire            wbu_pciif_frame_out ;
wire            wbu_pciif_frame_en_out ;
wire            wbu_pciif_irdy_out ;
wire            wbu_pciif_irdy_en_out ;
wire    [31:0]  wbu_pciif_ad_out ;
wire            wbu_pciif_ad_en_out ;
wire    [3:0]   wbu_pciif_cbe_out ;
wire            wbu_pciif_cbe_en_out ;
wire    [31:0]  wbu_err_addr_out ;
wire    [3:0]   wbu_err_bc_out ;
wire            wbu_err_signal_out ;
wire            wbu_err_source_out ;
wire            wbu_err_rty_exp_out ;
wire            wbu_tabort_rec_out ;
wire            wbu_mabort_rec_out ;
wire    [11:0]  wbu_conf_offset_out ;
wire            wbu_conf_renable_out ;
wire            wbu_conf_wenable_out ;
wire    [3:0]   wbu_conf_be_out ;
wire    [31:0]  wbu_conf_data_out ;
wire            wbu_del_read_comp_pending_out ;
wire            wbu_wbw_fifo_empty_out ;
wire            wbu_ad_load_out ;
wire            wbu_ad_load_on_transfer_out ;
wire            wbu_pciif_frame_load_out ;

// assign wishbone slave unit's outputs to top outputs where possible
assign SDAT_O   =   wbu_sdata_out ;
assign ACK_O    =   wbu_ack_out ;
assign RTY_O    =   wbu_rty_out ;
assign ERR_O    =   wbu_err_out ;

// PCI TARGET UNIT OUTPUTS
wire    [31:0]  pciu_adr_out ;
wire    [31:0]  pciu_mdata_out ;
wire            pciu_cyc_out ;
wire            pciu_stb_out ;
wire            pciu_we_out ;
wire    [3:0]   pciu_sel_out ;
wire            pciu_cab_out ;
wire            pciu_pciif_trdy_out ;
wire            pciu_pciif_stop_out ;
wire            pciu_pciif_devsel_out ;
wire            pciu_pciif_trdy_en_out ;
wire            pciu_pciif_stop_en_out ;
wire            pciu_pciif_devsel_en_out ;
wire            pciu_ad_load_out ;
wire            pciu_ad_load_on_transfer_out ;
wire   [31:0]   pciu_pciif_ad_out ;
wire            pciu_pciif_ad_en_out ;
wire            pciu_pciif_tabort_set_out ;
wire    [31:0]  pciu_err_addr_out ;
wire    [3:0]   pciu_err_bc_out ;
wire    [31:0]  pciu_err_data_out ;
wire    [3:0]   pciu_err_be_out ;
wire            pciu_err_signal_out ;
wire            pciu_err_source_out ;
wire            pciu_err_rty_exp_out ;
wire            pciu_conf_select_out ;
wire    [11:0]  pciu_conf_offset_out ;
wire            pciu_conf_renable_out ;
wire            pciu_conf_wenable_out ;
wire    [3:0]   pciu_conf_be_out ;
wire    [31:0]  pciu_conf_data_out ;
wire            pciu_pci_drcomp_pending_out ;
wire            pciu_pciw_fifo_empty_out ;

// assign pci target unit's outputs to top outputs where possible
assign ADR_O    =   pciu_adr_out ;
assign MDAT_O   =   pciu_mdata_out ;
assign CYC_O    =   pciu_cyc_out ;
assign STB_O    =   pciu_stb_out ;
assign WE_O     =   pciu_we_out ;
assign SEL_O    =   pciu_sel_out ;
assign CAB_O    =   pciu_cab_out ;

// CONFIGURATION SPACE OUTPUTS
wire    [31:0]  conf_w_data_out ;
wire    [31:0]  conf_r_data_out ;
wire            conf_serr_enable_out ;
wire            conf_perr_response_out ;
wire            conf_pci_master_enable_out ;
wire            conf_mem_space_enable_out ;
wire            conf_io_space_enable_out ;
wire    [7:0]   conf_cache_line_size_to_pci_out ;
wire    [7:0]   conf_cache_line_size_to_wb_out ;
wire            conf_cache_lsize_not_zero_to_wb_out ;
wire    [7:0]   conf_latency_tim_out ;

wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ba0_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ba1_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ba2_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ba3_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ba4_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ba5_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ta0_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ta1_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ta2_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ta3_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ta4_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_ta5_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_am0_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_am1_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_am2_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_am3_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_am4_out ;
wire   [19:(20 - `PCI_NUM_OF_DEC_ADDR_LINES)]   conf_pci_am5_out ;

wire            conf_pci_mem_io0_out ;
wire            conf_pci_mem_io1_out ;
wire            conf_pci_mem_io2_out ;
wire            conf_pci_mem_io3_out ;
wire            conf_pci_mem_io4_out ;
wire            conf_pci_mem_io5_out ;

wire    [1:0]   conf_pci_img_ctrl0_out ;
wire    [1:0]   conf_pci_img_ctrl1_out ;
wire    [1:0]   conf_pci_img_ctrl2_out ;
wire    [1:0]   conf_pci_img_ctrl3_out ;
wire    [1:0]   conf_pci_img_ctrl4_out ;
wire    [1:0]   conf_pci_img_ctrl5_out ;

wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ba0_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ba1_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ba2_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ba3_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ba4_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ba5_out ;

wire            conf_wb_mem_io0_out ;
wire            conf_wb_mem_io1_out ;
wire            conf_wb_mem_io2_out ;
wire            conf_wb_mem_io3_out ;
wire            conf_wb_mem_io4_out ;
wire            conf_wb_mem_io5_out ;

wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_am0_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_am1_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_am2_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_am3_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_am4_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_am5_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ta0_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ta1_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ta2_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ta3_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ta4_out ;
wire    [19:(20 - `WB_NUM_OF_DEC_ADDR_LINES)]  conf_wb_ta5_out ;
wire    [2:0]   conf_wb_img_ctrl0_out ;
wire    [2:0]   conf_wb_img_ctrl1_out ;
wire    [2:0]   conf_wb_img_ctrl2_out ;
wire    [2:0]   conf_wb_img_ctrl3_out ;
wire    [2:0]   conf_wb_img_ctrl4_out ;
wire    [2:0]   conf_wb_img_ctrl5_out ;
wire    [23:0]  conf_ccyc_addr_out ;
wire            conf_soft_res_out ;
wire            conf_int_out ;

// PCI IO MUX OUTPUTS
wire        pci_mux_frame_out ;
wire        pci_mux_irdy_out ;
wire        pci_mux_devsel_out ;
wire        pci_mux_trdy_out ;
wire        pci_mux_stop_out ;
wire [3:0]  pci_mux_cbe_out ;
wire [31:0] pci_mux_ad_out ;
wire        pci_mux_ad_load_out ;

wire [31:0] pci_mux_ad_en_out ;
wire        pci_mux_ad_en_unregistered_out ;
wire        pci_mux_frame_en_out ;
wire        pci_mux_irdy_en_out ;
wire        pci_mux_devsel_en_out ;
wire        pci_mux_trdy_en_out ;
wire        pci_mux_stop_en_out ;
wire [3:0]  pci_mux_cbe_en_out ;

wire        pci_mux_par_out ;
wire        pci_mux_par_en_out ;
wire        pci_mux_perr_out ;
wire        pci_mux_perr_en_out ;
wire        pci_mux_serr_out ;
wire        pci_mux_serr_en_out ;

wire        pci_mux_req_out ;
wire        pci_mux_req_en_out ;

// assign outputs to top level outputs

assign PCI_AD_EN_OUT       = pci_mux_ad_en_out ;
assign PCI_FRAMEn_EN_OUT   = pci_mux_frame_en_out ;
assign PCI_IRDYn_EN_OUT    = pci_mux_irdy_en_out ;
assign PCI_CBEn_EN_OUT     = pci_mux_cbe_en_out ;

assign PCI_PAR_OUT         =   pci_mux_par_out ;
assign PCI_PAR_EN_OUT      =   pci_mux_par_en_out ;
assign PCI_PERRn_OUT       =   pci_mux_perr_out ;
assign PCI_PERRn_EN_OUT    =   pci_mux_perr_en_out ;
assign PCI_SERRn_OUT       =   pci_mux_serr_out ;
assign PCI_SERRn_EN_OUT    =   pci_mux_serr_en_out ;

assign PCI_REQn_OUT        =   pci_mux_req_out ;
assign PCI_REQn_EN_OUT     =   pci_mux_req_en_out ;

assign PCI_TRDYn_EN_OUT    = pci_mux_trdy_en_out ;
assign PCI_DEVSELn_EN_OUT  = pci_mux_devsel_en_out ;
assign PCI_STOPn_EN_OUT    = pci_mux_stop_en_out ;
assign PCI_TRDYn_OUT       =  pci_mux_trdy_out ;
assign PCI_DEVSELn_OUT     = pci_mux_devsel_out ;
assign PCI_STOPn_OUT       = pci_mux_stop_out ;

assign PCI_AD_OUT          = pci_mux_ad_out ;
assign PCI_FRAMEn_OUT      = pci_mux_frame_out ;
assign PCI_IRDYn_OUT       = pci_mux_irdy_out ;
assign PCI_CBEn_OUT        = pci_mux_cbe_out ;

// duplicate output register's outputs
wire            out_bckp_frame_out ;
wire            out_bckp_irdy_out ;
wire            out_bckp_devsel_out ;
wire            out_bckp_trdy_out ;
wire            out_bckp_stop_out ;
wire    [3:0]   out_bckp_cbe_out ;
wire            out_bckp_cbe_en_out ;
wire    [31:0]  out_bckp_ad_out ;
wire            out_bckp_ad_en_out ;
wire            out_bckp_irdy_en_out ;
wire            out_bckp_frame_en_out ;
wire            out_bckp_tar_ad_en_out ;
wire            out_bckp_mas_ad_en_out ;
wire            out_bckp_trdy_en_out ;

wire            out_bckp_par_out ;
wire            out_bckp_par_en_out ;
wire            out_bckp_perr_out ;
wire            out_bckp_perr_en_out ;
wire            out_bckp_serr_out ;
wire            out_bckp_serr_en_out ;


// PARITY CHECKER OUTPUTS
wire    parchk_pci_par_out ;
wire    parchk_pci_par_en_out ;
wire    parchk_pci_perr_out ;
wire    parchk_pci_perr_en_out ;
wire    parchk_pci_serr_out ;
wire    parchk_pci_serr_en_out ;
wire    parchk_par_err_detect_out ;
wire    parchk_perr_mas_detect_out ;
wire    parchk_sig_serr_out ;

// input register outputs
wire            in_reg_gnt_out ;
wire            in_reg_frame_out ;
wire            in_reg_irdy_out ;
wire            in_reg_trdy_out ;
wire            in_reg_stop_out ;
wire            in_reg_devsel_out ;
wire            in_reg_idsel_out ;
wire    [31:0]  in_reg_ad_out ;
wire    [3:0]   in_reg_cbe_out ;

/*=========================================================================================================
Now comes definition of all modules' and their appropriate inputs
=========================================================================================================*/
// PCI BRIDGE RESET AND INTERRUPT LOGIC INPUTS
wire    pci_resi_rst_i                  = RST_I ;
wire    pci_resi_pci_rstn_in            = PCI_RSTn_IN ;
wire    pci_resi_conf_soft_res_in       = conf_soft_res_out ;
wire    pci_inti_pci_intan_in           = PCI_INTAn_IN ;
wire    pci_inti_conf_int_in            = conf_int_out ;
wire    pci_inti_int_i                  = INT_I ;
wire    pci_inti_out_bckp_perr_en_in    = out_bckp_perr_en_out ;
wire    pci_inti_out_bckp_serr_en_in    = out_bckp_serr_en_out ;

PCI_RST_INT     pci_resets_and_interrupts
(
    .clk_in                 (pci_clk),
    .rst_i                  (pci_resi_rst_i),
    .pci_rstn_in            (pci_resi_pci_rstn_in),
    .conf_soft_res_in       (pci_resi_conf_soft_res_in),
    .reset                  (pci_reso_reset),
    .pci_rstn_out           (pci_reso_pci_rstn_out),
    .pci_rstn_en_out        (pci_reso_pci_rstn_en_out),
    .rst_o                  (pci_reso_rst_o),
    .pci_intan_in           (pci_inti_pci_intan_in),
    .conf_int_in            (pci_inti_conf_int_in),
    .int_i                  (pci_inti_int_i),
    .out_bckp_perr_en_in    (pci_inti_out_bckp_perr_en_in),
    .out_bckp_serr_en_in    (pci_inti_out_bckp_serr_en_in),
    .pci_intan_out          (pci_into_pci_intan_out),
    .pci_intan_en_out       (pci_into_pci_intan_en_out),
    .int_o                  (pci_into_int_o),
    .conf_isr_int_prop_out  (pci_into_conf_isr_int_prop_out)
);

// WISHBONE SLAVE UNIT INPUTS
wire    [31:0]  wbu_addr_in                     =   ADR_I ;
wire    [31:0]  wbu_sdata_in                    =   SDAT_I ;
wire            wbu_cyc_in                      =   CYC_I ;
wire            wbu_stb_in                      =   STB_I ;
wire            wbu_we_in                       =   WE_I ;
wire    [3:0]   wbu_sel_in                      =   SEL_I ;
wire            wbu_cab_in                      =   CAB_I ;

wire    [5:0]   wbu_map_in                      =   {
                                                     conf_wb_mem_io5_out,
                                                     conf_wb_mem_io4_out,
                                                     conf_wb_mem_io3_out,
                                                     conf_wb_mem_io2_out,
                                                     conf_wb_mem_io1_out,
                                                     conf_wb_mem_io0_out
                                                    } ;

wire    [5:0]   wbu_pref_en_in                  =   {
                                                     conf_wb_img_ctrl5_out[1],
                                                     conf_wb_img_ctrl4_out[1],
                                                     conf_wb_img_ctrl3_out[1],
                                                     conf_wb_img_ctrl2_out[1],
                                                     conf_wb_img_ctrl1_out[1],
                                                     conf_wb_img_ctrl0_out[1]
                                                    };
wire    [5:0]   wbu_mrl_en_in                   =   {
                                                     conf_wb_img_ctrl5_out[0],
                                                     conf_wb_img_ctrl4_out[0],
                                                     conf_wb_img_ctrl3_out[0],
                                                     conf_wb_img_ctrl2_out[0],
                                                     conf_wb_img_ctrl1_out[0],
                                                     conf_wb_img_ctrl0_out[0]
                                                    };

wire    [5:0]   wbu_at_en_in                    =   {
                                                     conf_wb_img_ctrl5_out[2],
                                                     conf_wb_img_ctrl4_out[2],
                                                     conf_wb_img_ctrl3_out[2],
                                                     conf_wb_img_ctrl2_out[2],
                                                     conf_wb_img_ctrl1_out[2],
                                                     conf_wb_img_ctrl0_out[2]
                                                    } ;

wire            wbu_pci_drcomp_pending_in       =   pciu_pci_drcomp_pending_out ;
wire            wbu_pciw_empty_in               =   pciu_pciw_fifo_empty_out ;

`ifdef HOST
    wire    [31:0]  wbu_conf_data_in            =   conf_w_data_out ;
`else
`ifdef GUEST
    wire    [31:0]  wbu_conf_data_in            =   conf_r_data_out ;
`endif
`endif

wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_bar0_in  =   conf_wb_ba0_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_bar1_in  =   conf_wb_ba1_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_bar2_in  =   conf_wb_ba2_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_bar3_in  =   conf_wb_ba3_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_bar4_in  =   conf_wb_ba4_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_bar5_in  =   conf_wb_ba5_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_am0_in   =   conf_wb_am0_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_am1_in   =   conf_wb_am1_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_am2_in   =   conf_wb_am2_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_am3_in   =   conf_wb_am3_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_am4_in   =   conf_wb_am4_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_am5_in   =   conf_wb_am5_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_ta0_in   =   conf_wb_ta0_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_ta1_in   =   conf_wb_ta1_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_ta2_in   =   conf_wb_ta2_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_ta3_in   =   conf_wb_ta3_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_ta4_in   =   conf_wb_ta4_out ;
wire   [(`WB_NUM_OF_DEC_ADDR_LINES - 1):0] wbu_ta5_in   =   conf_wb_ta5_out ;

wire    [23:0]  wbu_ccyc_addr_in                        =   conf_ccyc_addr_out ;
wire            wbu_master_enable_in                    =   conf_pci_master_enable_out ;
wire            wbu_cache_line_size_not_zero            =   conf_cache_lsize_not_zero_to_wb_out ;
wire    [7:0]   wbu_cache_line_size_in                  =   conf_cache_line_size_to_pci_out ;

wire            wbu_pciif_gnt_in                        = PCI_GNTn_IN ;
wire            wbu_pciif_frame_in                      = in_reg_frame_out ;
wire            wbu_pciif_irdy_in                       = in_reg_irdy_out ;
wire            wbu_pciif_trdy_in                       = PCI_TRDYn_IN ;
wire            wbu_pciif_stop_in                       = PCI_STOPn_IN ;
wire            wbu_pciif_devsel_in                     = PCI_DEVSELn_IN ;
wire    [31:0]  wbu_pciif_ad_reg_in                     = in_reg_ad_out ;
wire            wbu_pciif_trdy_reg_in                   = in_reg_trdy_out ;
wire            wbu_pciif_stop_reg_in                   = in_reg_stop_out ;
wire            wbu_pciif_devsel_reg_in                 = in_reg_devsel_out ;


wire    [7:0]   wbu_latency_tim_val_in                  = conf_latency_tim_out ;

wire            wbu_pciif_frame_en_in                   = out_bckp_frame_en_out ;
wire            wbu_pciif_frame_out_in                  = out_bckp_frame_out ;

WB_SLAVE_UNIT wishbone_slave_unit
(
    .reset_in                      (reset),
    .wb_clock_in                   (wb_clk),
    .pci_clock_in                  (pci_clk),
    .ADDR_I                        (wbu_addr_in),
    .SDATA_I                       (wbu_sdata_in),
    .SDATA_O                       (wbu_sdata_out),
    .CYC_I                         (wbu_cyc_in),
    .STB_I                         (wbu_stb_in),
    .WE_I                          (wbu_we_in),
    .SEL_I                         (wbu_sel_in),
    .ACK_O                         (wbu_ack_out),
    .RTY_O                         (wbu_rty_out),
    .ERR_O                         (wbu_err_out),
    .CAB_I                         (wbu_cab_in),
    .wbu_map_in                    (wbu_map_in),
    .wbu_pref_en_in                (wbu_pref_en_in),
    .wbu_mrl_en_in                 (wbu_mrl_en_in),
    .wbu_pci_drcomp_pending_in     (wbu_pci_drcomp_pending_in),
    .wbu_conf_data_in              (wbu_conf_data_in),
    .wbu_pciw_empty_in             (wbu_pciw_empty_in),
    .wbu_bar0_in                   (wbu_bar0_in),
    .wbu_bar1_in                   (wbu_bar1_in),
    .wbu_bar2_in                   (wbu_bar2_in),
    .wbu_bar3_in                   (wbu_bar3_in),
    .wbu_bar4_in                   (wbu_bar4_in),
    .wbu_bar5_in                   (wbu_bar5_in),
    .wbu_am0_in                    (wbu_am0_in),
    .wbu_am1_in                    (wbu_am1_in),
    .wbu_am2_in                    (wbu_am2_in),
    .wbu_am3_in                    (wbu_am3_in),
    .wbu_am4_in                    (wbu_am4_in),
    .wbu_am5_in                    (wbu_am5_in),
    .wbu_ta0_in                    (wbu_ta0_in),
    .wbu_ta1_in                    (wbu_ta1_in),
    .wbu_ta2_in                    (wbu_ta2_in),
    .wbu_ta3_in                    (wbu_ta3_in),
    .wbu_ta4_in                    (wbu_ta4_in),
    .wbu_ta5_in                    (wbu_ta5_in),
    .wbu_at_en_in                  (wbu_at_en_in),
    .wbu_ccyc_addr_in              (wbu_ccyc_addr_in),
    .wbu_master_enable_in          (wbu_master_enable_in),
    .wbu_cache_line_size_not_zero  (wbu_cache_line_size_not_zero),
    .wbu_cache_line_size_in        (wbu_cache_line_size_in),
    .wbu_pciif_gnt_in              (wbu_pciif_gnt_in),
    .wbu_pciif_frame_in            (wbu_pciif_frame_in),
    .wbu_pciif_frame_en_in         (wbu_pciif_frame_en_in),
    .wbu_pciif_frame_out_in        (wbu_pciif_frame_out_in),
    .wbu_pciif_irdy_in             (wbu_pciif_irdy_in),
    .wbu_pciif_trdy_in             (wbu_pciif_trdy_in),
    .wbu_pciif_stop_in             (wbu_pciif_stop_in),
    .wbu_pciif_devsel_in           (wbu_pciif_devsel_in),
    .wbu_pciif_ad_reg_in           (wbu_pciif_ad_reg_in),
    .wbu_pciif_req_out             (wbu_pciif_req_out),
    .wbu_pciif_frame_out           (wbu_pciif_frame_out),
    .wbu_pciif_frame_en_out        (wbu_pciif_frame_en_out),
    .wbu_pciif_frame_load_out      (wbu_pciif_frame_load_out),
    .wbu_pciif_irdy_out            (wbu_pciif_irdy_out),
    .wbu_pciif_irdy_en_out         (wbu_pciif_irdy_en_out),
    .wbu_pciif_ad_out              (wbu_pciif_ad_out),
    .wbu_pciif_ad_en_out           (wbu_pciif_ad_en_out),
    .wbu_pciif_cbe_out             (wbu_pciif_cbe_out),
    .wbu_pciif_cbe_en_out          (wbu_pciif_cbe_en_out),
    .wbu_err_addr_out              (wbu_err_addr_out),
    .wbu_err_bc_out                (wbu_err_bc_out),
    .wbu_err_signal_out            (wbu_err_signal_out),
    .wbu_err_source_out            (wbu_err_source_out),
    .wbu_err_rty_exp_out           (wbu_err_rty_exp_out),
    .wbu_tabort_rec_out            (wbu_tabort_rec_out),
    .wbu_mabort_rec_out            (wbu_mabort_rec_out),
    .wbu_conf_offset_out           (wbu_conf_offset_out),
    .wbu_conf_renable_out          (wbu_conf_renable_out),
    .wbu_conf_wenable_out          (wbu_conf_wenable_out),
    .wbu_conf_be_out               (wbu_conf_be_out),
    .wbu_conf_data_out             (wbu_conf_data_out),
    .wbu_del_read_comp_pending_out (wbu_del_read_comp_pending_out),
    .wbu_wbw_fifo_empty_out        (wbu_wbw_fifo_empty_out),
    .wbu_latency_tim_val_in        (wbu_latency_tim_val_in),
    .wbu_ad_load_out               (wbu_ad_load_out),
    .wbu_ad_load_on_transfer_out   (wbu_ad_load_on_transfer_out),
    .wbu_pciif_trdy_reg_in         (wbu_pciif_trdy_reg_in),
    .wbu_pciif_stop_reg_in         (wbu_pciif_stop_reg_in),
    .wbu_pciif_devsel_reg_in       (wbu_pciif_devsel_reg_in)
);

// PCI TARGET UNIT INPUTS
wire    [31:0]  pciu_mdata_in                   =   MDAT_I ;
wire            pciu_ack_in                     =   ACK_I ;
wire            pciu_rty_in                     =   RTY_I ;
wire            pciu_err_in                     =   ERR_I ;

wire    [5:0]   pciu_map_in                     =   {
                                                     conf_pci_mem_io5_out,
                                                     conf_pci_mem_io4_out,
                                                     conf_pci_mem_io3_out,
                                                     conf_pci_mem_io2_out,
                                                     conf_pci_mem_io1_out,
                                                     conf_pci_mem_io0_out
                                                    } ;

wire    [5:0]   pciu_pref_en_in                 =   {
                                                     conf_pci_img_ctrl5_out[0],
                                                     conf_pci_img_ctrl4_out[0],
                                                     conf_pci_img_ctrl3_out[0],
                                                     conf_pci_img_ctrl2_out[0],
                                                     conf_pci_img_ctrl1_out[0],
                                                     conf_pci_img_ctrl0_out[0]
                                                    };

wire    [5:0]   pciu_at_en_in                   =   {
                                                     conf_pci_img_ctrl5_out[1],
                                                     conf_pci_img_ctrl4_out[1],
                                                     conf_pci_img_ctrl3_out[1],
                                                     conf_pci_img_ctrl2_out[1],
                                                     conf_pci_img_ctrl1_out[1],
                                                     conf_pci_img_ctrl0_out[1]
                                                    } ;

wire            pciu_mem_enable_in              =   conf_mem_space_enable_out ;
wire            pciu_io_enable_in               =   conf_io_space_enable_out ;

wire            pciu_wbw_fifo_empty_in          =   wbu_wbw_fifo_empty_out ;
wire			pciu_wbu_del_read_comp_pending_in	=	wbu_del_read_comp_pending_out ;
wire            pciu_wbu_frame_en_in            =   out_bckp_frame_en_out ;

`ifdef HOST
    wire    [31:0]  pciu_conf_data_in           =   conf_r_data_out ;
`else
`ifdef GUEST
    wire    [31:0]  pciu_conf_data_in           =   conf_w_data_out ;
`endif
`endif

wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar0_in =   conf_pci_ba0_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar1_in =   conf_pci_ba1_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar2_in =   conf_pci_ba2_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar3_in =   conf_pci_ba3_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar4_in =   conf_pci_ba4_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_bar5_in =   conf_pci_ba5_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am0_in  =   conf_pci_am0_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am1_in  =   conf_pci_am1_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am2_in  =   conf_pci_am2_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am3_in  =   conf_pci_am3_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am4_in  =   conf_pci_am4_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_am5_in  =   conf_pci_am5_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta0_in  =   conf_pci_ta0_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta1_in  =   conf_pci_ta1_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta2_in  =   conf_pci_ta2_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta3_in  =   conf_pci_ta3_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta4_in  =   conf_pci_ta4_out ;
wire   [(`PCI_NUM_OF_DEC_ADDR_LINES - 1):0] pciu_ta5_in  =   conf_pci_ta5_out ;

wire    [7:0]   pciu_cache_line_size_in                 =   conf_cache_line_size_to_wb_out ;
wire            pciu_cache_lsize_not_zero_in            =   conf_cache_lsize_not_zero_to_wb_out ;

wire            pciu_pciif_frame_in                     =   PCI_FRAMEn_IN ;
wire            pciu_pciif_irdy_in                      =   PCI_IRDYn_IN ;
wire            pciu_pciif_idsel_in                     =   PCI_IDSEL_IN ;
wire            pciu_pciif_frame_reg_in                 =   in_reg_frame_out ;
wire            pciu_pciif_irdy_reg_in                  =   in_reg_irdy_out ;
wire            pciu_pciif_idsel_reg_in                 =   in_reg_idsel_out ;
wire    [31:0]  pciu_pciif_ad_reg_in                    =   in_reg_ad_out ;
wire    [3:0]   pciu_pciif_cbe_reg_in                   =   in_reg_cbe_out ;

wire            pciu_pciif_bckp_trdy_en_in              =   out_bckp_trdy_en_out ;
wire            pciu_pciif_bckp_devsel_in               =   out_bckp_devsel_out ;
wire            pciu_pciif_bckp_trdy_in                 =   out_bckp_trdy_out ;
wire            pciu_pciif_bckp_stop_in                 =   out_bckp_stop_out ;
wire            pciu_pciif_trdy_reg_in                  =   in_reg_trdy_out ;
wire            pciu_pciif_stop_reg_in                  =   in_reg_stop_out ;

PCI_TARGET_UNIT pci_target_unit
(
    .reset_in                       (reset),
    .wb_clock_in                    (wb_clk),
    .pci_clock_in                   (pci_clk),
    .ADR_O                          (pciu_adr_out),
    .MDATA_O                        (pciu_mdata_out),
    .MDATA_I                        (pciu_mdata_in),
    .CYC_O                          (pciu_cyc_out),
    .STB_O                          (pciu_stb_out),
    .WE_O                           (pciu_we_out),
    .SEL_O                          (pciu_sel_out),
    .ACK_I                          (pciu_ack_in),
    .RTY_I                          (pciu_rty_in),
    .ERR_I                          (pciu_err_in),
    .CAB_O                          (pciu_cab_out),
    .pciu_mem_enable_in             (pciu_mem_enable_in),
    .pciu_io_enable_in              (pciu_io_enable_in),
    .pciu_map_in                    (pciu_map_in),
    .pciu_pref_en_in                (pciu_pref_en_in),
    .pciu_conf_data_in              (pciu_conf_data_in),
    .pciu_wbw_fifo_empty_in         (pciu_wbw_fifo_empty_in),
    .pciu_wbu_del_read_comp_pending_in	(pciu_wbu_del_read_comp_pending_in),
    .pciu_wbu_frame_en_in           (pciu_wbu_frame_en_in),
    .pciu_bar0_in                   (pciu_bar0_in),
    .pciu_bar1_in                   (pciu_bar1_in),
    .pciu_bar2_in                   (pciu_bar2_in),
    .pciu_bar3_in                   (pciu_bar3_in),
    .pciu_bar4_in                   (pciu_bar4_in),
    .pciu_bar5_in                   (pciu_bar5_in),
    .pciu_am0_in                    (pciu_am0_in),
    .pciu_am1_in                    (pciu_am1_in),
    .pciu_am2_in                    (pciu_am2_in),
    .pciu_am3_in                    (pciu_am3_in),
    .pciu_am4_in                    (pciu_am4_in),
    .pciu_am5_in                    (pciu_am5_in),
    .pciu_ta0_in                    (pciu_ta0_in),
    .pciu_ta1_in                    (pciu_ta1_in),
    .pciu_ta2_in                    (pciu_ta2_in),
    .pciu_ta3_in                    (pciu_ta3_in),
    .pciu_ta4_in                    (pciu_ta4_in),
    .pciu_ta5_in                    (pciu_ta5_in),
    .pciu_at_en_in                  (pciu_at_en_in),
    .pciu_cache_line_size_in        (pciu_cache_line_size_in),
    .pciu_cache_lsize_not_zero_in   (pciu_cache_lsize_not_zero_in),
    .pciu_pciif_frame_in            (pciu_pciif_frame_in),
    .pciu_pciif_irdy_in             (pciu_pciif_irdy_in),
    .pciu_pciif_idsel_in            (pciu_pciif_idsel_in),
    .pciu_pciif_frame_reg_in        (pciu_pciif_frame_reg_in),
    .pciu_pciif_irdy_reg_in         (pciu_pciif_irdy_reg_in),
    .pciu_pciif_idsel_reg_in        (pciu_pciif_idsel_reg_in),
    .pciu_pciif_ad_reg_in           (pciu_pciif_ad_reg_in),
    .pciu_pciif_cbe_reg_in          (pciu_pciif_cbe_reg_in),
    .pciu_pciif_bckp_trdy_en_in     (pciu_pciif_bckp_trdy_en_in),
    .pciu_pciif_bckp_devsel_in      (pciu_pciif_bckp_devsel_in),
    .pciu_pciif_bckp_trdy_in        (pciu_pciif_bckp_trdy_in),
    .pciu_pciif_bckp_stop_in        (pciu_pciif_bckp_stop_in),
    .pciu_pciif_trdy_reg_in         (pciu_pciif_trdy_reg_in),
    .pciu_pciif_stop_reg_in         (pciu_pciif_stop_reg_in),
    .pciu_pciif_trdy_out            (pciu_pciif_trdy_out),
    .pciu_pciif_stop_out            (pciu_pciif_stop_out),
    .pciu_pciif_devsel_out          (pciu_pciif_devsel_out),
    .pciu_pciif_trdy_en_out         (pciu_pciif_trdy_en_out),
    .pciu_pciif_stop_en_out         (pciu_pciif_stop_en_out),
    .pciu_pciif_devsel_en_out       (pciu_pciif_devsel_en_out),
    .pciu_ad_load_out               (pciu_ad_load_out),
    .pciu_ad_load_on_transfer_out   (pciu_ad_load_on_transfer_out),
    .pciu_pciif_ad_out              (pciu_pciif_ad_out),
    .pciu_pciif_ad_en_out           (pciu_pciif_ad_en_out),
    .pciu_pciif_tabort_set_out      (pciu_pciif_tabort_set_out),
    .pciu_err_addr_out              (pciu_err_addr_out),
    .pciu_err_bc_out                (pciu_err_bc_out),
    .pciu_err_data_out              (pciu_err_data_out),
    .pciu_err_be_out                (pciu_err_be_out),
    .pciu_err_signal_out            (pciu_err_signal_out),
    .pciu_err_source_out            (pciu_err_source_out),
    .pciu_err_rty_exp_out           (pciu_err_rty_exp_out),
    .pciu_conf_offset_out           (pciu_conf_offset_out),
    .pciu_conf_renable_out          (pciu_conf_renable_out),
    .pciu_conf_wenable_out          (pciu_conf_wenable_out),
    .pciu_conf_be_out               (pciu_conf_be_out),
    .pciu_conf_data_out             (pciu_conf_data_out),
    .pciu_conf_select_out           (pciu_conf_select_out),
    .pciu_pci_drcomp_pending_out    (pciu_pci_drcomp_pending_out),
    .pciu_pciw_fifo_empty_out       (pciu_pciw_fifo_empty_out)
);


// CONFIGURATION SPACE INPUTS
`ifdef HOST

    wire    [11:0]  conf_w_addr_in          =       wbu_conf_offset_out ;
    wire    [31:0]  conf_w_data_in          =       wbu_conf_data_out ;
    wire            conf_w_we_in            =       wbu_conf_wenable_out ;
    wire            conf_w_re_in            =       wbu_conf_renable_out ;
    wire    [3:0]   conf_w_be_in            =       wbu_conf_be_out     ;
    wire            conf_w_clock            =       wb_clk ;
    wire    [11:0]  conf_r_addr_in          =       pciu_conf_offset_out ;
    wire            conf_r_re_in            =       pciu_conf_renable_out ;

`else
`ifdef GUEST

    wire    [11:0]  conf_r_addr_in          =       wbu_conf_offset_out ;
    wire            conf_r_re_in            =       wbu_conf_renable_out ;
    wire            conf_w_clock            =       pci_clk ;
    wire    [11:0]  conf_w_addr_in          =       pciu_conf_offset_out ;
    wire    [31:0]  conf_w_data_in          =       pciu_conf_data_out ;
    wire            conf_w_we_in            =       pciu_conf_wenable_out ;
    wire            conf_w_re_in            =       pciu_conf_renable_out ;
    wire    [3:0]   conf_w_be_in            =       pciu_conf_be_out ;

`endif
`endif


wire            conf_perr_in                            =   parchk_par_err_detect_out ;
wire            conf_serr_in                            =   parchk_sig_serr_out ;
wire            conf_master_abort_recv_in               =   wbu_mabort_rec_out ;
wire            conf_target_abort_recv_in               =   wbu_tabort_rec_out ;
wire            conf_target_abort_set_in                =   pciu_pciif_tabort_set_out ;

wire            conf_master_data_par_err_in             =   parchk_perr_mas_detect_out ;

wire    [3:0]   conf_pci_err_be_in      = pciu_err_be_out ;
wire    [3:0]   conf_pci_err_bc_in      = pciu_err_bc_out;
wire            conf_pci_err_es_in      = pciu_err_source_out ;
wire            conf_pci_err_rty_exp_in = pciu_err_rty_exp_out ;
wire            conf_pci_err_sig_in     = pciu_err_signal_out ;
wire    [31:0]  conf_pci_err_addr_in    = pciu_err_addr_out ;
wire    [31:0]  conf_pci_err_data_in    = pciu_err_data_out ;

wire    [3:0]   conf_wb_err_be_in       =   out_bckp_cbe_out ;
wire    [3:0]   conf_wb_err_bc_in       =   wbu_err_bc_out ;
wire            conf_wb_err_rty_exp_in  =   wbu_err_rty_exp_out ;
wire            conf_wb_err_es_in       =   wbu_err_source_out ;
wire            conf_wb_err_sig_in      =   wbu_err_signal_out ;
wire    [31:0]  conf_wb_err_addr_in     =   wbu_err_addr_out ;
wire    [31:0]  conf_wb_err_data_in     =   out_bckp_ad_out ;

wire            conf_isr_int_prop_in    =   pci_into_conf_isr_int_prop_out ;
wire            conf_par_err_int_in     =   parchk_perr_mas_detect_out ;
wire            conf_sys_err_int_in     =   parchk_sig_serr_out ;

CONF_SPACE configuration    (
                                .reset                      (reset),
                                .pci_clk                    (pci_clk),
                                .wb_clk                     (wb_clk),
                                .w_conf_address_in          (conf_w_addr_in),
                                .w_conf_data_in             (conf_w_data_in),
                                .w_conf_data_out            (conf_w_data_out),
                                .r_conf_address_in          (conf_r_addr_in),
                                .r_conf_data_out            (conf_r_data_out),
                                .w_we                       (conf_w_we_in),
                                .w_re                       (conf_w_re_in),
                                .r_re                       (conf_r_re_in),
                                .w_byte_en                  (conf_w_be_in),
                                .w_clock                    (conf_w_clock),
                                .serr_enable                (conf_serr_enable_out),
                                .perr_response              (conf_perr_response_out),
                                .pci_master_enable          (conf_pci_master_enable_out),
                                .memory_space_enable        (conf_mem_space_enable_out),
                                .io_space_enable            (conf_io_space_enable_out),
                                .perr_in                    (conf_perr_in),
                                .serr_in                    (conf_serr_in),
                                .master_abort_recv          (conf_master_abort_recv_in),
                                .target_abort_recv          (conf_target_abort_recv_in),
                                .target_abort_set           (conf_target_abort_set_in),
                                .master_data_par_err        (conf_master_data_par_err_in),
                                .cache_line_size_to_pci     (conf_cache_line_size_to_pci_out),
                                .cache_line_size_to_wb      (conf_cache_line_size_to_wb_out),
                                .cache_lsize_not_zero_to_wb (conf_cache_lsize_not_zero_to_wb_out),
                                .latency_tim                (conf_latency_tim_out),
                                .pci_base_addr0             (conf_pci_ba0_out),
                                .pci_base_addr1             (conf_pci_ba1_out),
                                .pci_base_addr2             (conf_pci_ba2_out),
                                .pci_base_addr3             (conf_pci_ba3_out),
                                .pci_base_addr4             (conf_pci_ba4_out),
                                .pci_base_addr5             (conf_pci_ba5_out),
                                .pci_memory_io0             (conf_pci_mem_io0_out),
                                .pci_memory_io1             (conf_pci_mem_io1_out),
                                .pci_memory_io2             (conf_pci_mem_io2_out),
                                .pci_memory_io3             (conf_pci_mem_io3_out),
                                .pci_memory_io4             (conf_pci_mem_io4_out),
                                .pci_memory_io5             (conf_pci_mem_io5_out),
                                .pci_addr_mask0             (conf_pci_am0_out),
                                .pci_addr_mask1             (conf_pci_am1_out),
                                .pci_addr_mask2             (conf_pci_am2_out),
                                .pci_addr_mask3             (conf_pci_am3_out),
                                .pci_addr_mask4             (conf_pci_am4_out),
                                .pci_addr_mask5             (conf_pci_am5_out),
                                .pci_tran_addr0             (conf_pci_ta0_out),
                                .pci_tran_addr1             (conf_pci_ta1_out),
                                .pci_tran_addr2             (conf_pci_ta2_out),
                                .pci_tran_addr3             (conf_pci_ta3_out),
                                .pci_tran_addr4             (conf_pci_ta4_out),
                                .pci_tran_addr5             (conf_pci_ta5_out),
                                .pci_img_ctrl0              (conf_pci_img_ctrl0_out),
                                .pci_img_ctrl1              (conf_pci_img_ctrl1_out),
                                .pci_img_ctrl2              (conf_pci_img_ctrl2_out),
                                .pci_img_ctrl3              (conf_pci_img_ctrl3_out),
                                .pci_img_ctrl4              (conf_pci_img_ctrl4_out),
                                .pci_img_ctrl5              (conf_pci_img_ctrl5_out),
                                .pci_error_be               (conf_pci_err_be_in),
                                .pci_error_bc               (conf_pci_err_bc_in),
                                .pci_error_rty_exp          (conf_pci_err_rty_exp_in),
                                .pci_error_es               (conf_pci_err_es_in),
                                .pci_error_sig              (conf_pci_err_sig_in),
                                .pci_error_addr             (conf_pci_err_addr_in),
                                .pci_error_data             (conf_pci_err_data_in),
                                .wb_base_addr0              (conf_wb_ba0_out),
                                .wb_base_addr1              (conf_wb_ba1_out),
                                .wb_base_addr2              (conf_wb_ba2_out),
                                .wb_base_addr3              (conf_wb_ba3_out),
                                .wb_base_addr4              (conf_wb_ba4_out),
                                .wb_base_addr5              (conf_wb_ba5_out),
                                .wb_memory_io0              (conf_wb_mem_io0_out),
                                .wb_memory_io1              (conf_wb_mem_io1_out),
                                .wb_memory_io2              (conf_wb_mem_io2_out),
                                .wb_memory_io3              (conf_wb_mem_io3_out),
                                .wb_memory_io4              (conf_wb_mem_io4_out),
                                .wb_memory_io5              (conf_wb_mem_io5_out),
                                .wb_addr_mask0              (conf_wb_am0_out),
                                .wb_addr_mask1              (conf_wb_am1_out),
                                .wb_addr_mask2              (conf_wb_am2_out),
                                .wb_addr_mask3              (conf_wb_am3_out),
                                .wb_addr_mask4              (conf_wb_am4_out),
                                .wb_addr_mask5              (conf_wb_am5_out),
                                .wb_tran_addr0              (conf_wb_ta0_out),
                                .wb_tran_addr1              (conf_wb_ta1_out),
                                .wb_tran_addr2              (conf_wb_ta2_out),
                                .wb_tran_addr3              (conf_wb_ta3_out),
                                .wb_tran_addr4              (conf_wb_ta4_out),
                                .wb_tran_addr5              (conf_wb_ta5_out),
                                .wb_img_ctrl0               (conf_wb_img_ctrl0_out),
                                .wb_img_ctrl1               (conf_wb_img_ctrl1_out),
                                .wb_img_ctrl2               (conf_wb_img_ctrl2_out),
                                .wb_img_ctrl3               (conf_wb_img_ctrl3_out),
                                .wb_img_ctrl4               (conf_wb_img_ctrl4_out),
                                .wb_img_ctrl5               (conf_wb_img_ctrl5_out),
                                .wb_error_be                (conf_wb_err_be_in),
                                .wb_error_bc                (conf_wb_err_bc_in),
                                .wb_error_rty_exp           (conf_wb_err_rty_exp_in),
                                .wb_error_es                (conf_wb_err_es_in),
                                .wb_error_sig               (conf_wb_err_sig_in),
                                .wb_error_addr              (conf_wb_err_addr_in),
                                .wb_error_data              (conf_wb_err_data_in),
                                .config_addr                (conf_ccyc_addr_out),
                                .icr_soft_res               (conf_soft_res_out),
                                .int_out                    (conf_int_out),
                                .isr_int_prop               (conf_isr_int_prop_in),
                                .isr_par_err_int            (conf_par_err_int_in),
                                .isr_sys_err_int            (conf_sys_err_int_in)
                            ) ;

// pci data io multiplexer inputs
wire            pci_mux_tar_ad_en_in            = pciu_pciif_ad_en_out ;
wire            pci_mux_tar_ad_en_reg_in        = out_bckp_tar_ad_en_out ;
wire    [31:0]  pci_mux_tar_ad_in               = pciu_pciif_ad_out ;
wire            pci_mux_devsel_in               = pciu_pciif_devsel_out ;
wire            pci_mux_devsel_en_in            = pciu_pciif_devsel_en_out ;
wire            pci_mux_trdy_in                 = pciu_pciif_trdy_out ;
wire            pci_mux_trdy_en_in              = pciu_pciif_trdy_en_out ;
wire            pci_mux_stop_in                 = pciu_pciif_stop_out ;
wire            pci_mux_stop_en_in              = pciu_pciif_stop_en_out ;
wire            pci_mux_tar_load_in             = pciu_ad_load_out ;
wire            pci_mux_tar_load_on_transfer_in = pciu_ad_load_on_transfer_out ;

wire            pci_mux_mas_ad_en_in    = wbu_pciif_ad_en_out ;
wire    [31:0]  pci_mux_mas_ad_in       = wbu_pciif_ad_out ;

wire            pci_mux_frame_in                = wbu_pciif_frame_out ;
wire            pci_mux_frame_en_in             = wbu_pciif_frame_en_out ;
wire            pci_mux_irdy_in                 = wbu_pciif_irdy_out;
wire            pci_mux_irdy_en_in              = wbu_pciif_irdy_en_out;
wire            pci_mux_mas_load_in             = wbu_ad_load_out ;
wire            pci_mux_mas_load_on_transfer_in = wbu_ad_load_on_transfer_out ;
wire [3:0]      pci_mux_cbe_in                  = wbu_pciif_cbe_out ;
wire            pci_mux_cbe_en_in               = wbu_pciif_cbe_en_out ;

wire            pci_mux_par_in              = parchk_pci_par_out ;
wire            pci_mux_par_en_in           = parchk_pci_par_en_out ;
wire            pci_mux_perr_in             = parchk_pci_perr_out ;
wire            pci_mux_perr_en_in          = parchk_pci_perr_en_out ;
wire            pci_mux_serr_in             = parchk_pci_serr_out ;
wire            pci_mux_serr_en_in          = parchk_pci_serr_en_out;

wire            pci_mux_req_in              =   wbu_pciif_req_out ;
wire            pci_mux_frame_load_in       =   wbu_pciif_frame_load_out ;

wire            pci_mux_pci_irdy_in         =   PCI_IRDYn_IN ;
wire            pci_mux_pci_trdy_in         =   PCI_TRDYn_IN ;
wire            pci_mux_pci_frame_in        =   PCI_FRAMEn_IN ;
wire            pci_mux_pci_stop_in         =   PCI_STOPn_IN ;

PCI_IO_MUX pci_io_mux
(
    .reset_in                   (reset),
    .clk_in                     (pci_clk),
    .frame_in                   (pci_mux_frame_in),
    .frame_en_in                (pci_mux_frame_en_in),
    .frame_load_in              (pci_mux_frame_load_in),
    .irdy_in                    (pci_mux_irdy_in),
    .irdy_en_in                 (pci_mux_irdy_en_in),
    .devsel_in                  (pci_mux_devsel_in),
    .devsel_en_in               (pci_mux_devsel_en_in),
    .trdy_in                    (pci_mux_trdy_in),
    .trdy_en_in                 (pci_mux_trdy_en_in),
    .stop_in                    (pci_mux_stop_in),
    .stop_en_in                 (pci_mux_stop_en_in),
    .master_load_in             (pci_mux_mas_load_in),
    .master_load_on_transfer_in (pci_mux_mas_load_on_transfer_in),
    .target_load_in             (pci_mux_tar_load_in),
    .target_load_on_transfer_in (pci_mux_tar_load_on_transfer_in),
    .cbe_in                     (pci_mux_cbe_in),
    .cbe_en_in                  (pci_mux_cbe_en_in),
    .mas_ad_in                  (pci_mux_mas_ad_in),
    .tar_ad_in                  (pci_mux_tar_ad_in),

    .mas_ad_en_in               (pci_mux_mas_ad_en_in),
    .tar_ad_en_in               (pci_mux_tar_ad_en_in),
    .tar_ad_en_reg_in           (pci_mux_tar_ad_en_reg_in),

    .par_in                     (pci_mux_par_in),
    .par_en_in                  (pci_mux_par_en_in),
    .perr_in                    (pci_mux_perr_in),
    .perr_en_in                 (pci_mux_perr_en_in),
    .serr_in                    (pci_mux_serr_in),
    .serr_en_in                 (pci_mux_serr_en_in),

    .frame_en_out               (pci_mux_frame_en_out),
    .irdy_en_out                (pci_mux_irdy_en_out),
    .devsel_en_out              (pci_mux_devsel_en_out),
    .trdy_en_out                (pci_mux_trdy_en_out),
    .stop_en_out                (pci_mux_stop_en_out),
    .cbe_en_out                 (pci_mux_cbe_en_out),
    .ad_en_out                  (pci_mux_ad_en_out),

    .frame_out                  (pci_mux_frame_out),
    .irdy_out                   (pci_mux_irdy_out),
    .devsel_out                 (pci_mux_devsel_out),
    .trdy_out                   (pci_mux_trdy_out),
    .stop_out                   (pci_mux_stop_out),
    .cbe_out                    (pci_mux_cbe_out),
    .ad_out                     (pci_mux_ad_out),
    .ad_load_out                (pci_mux_ad_load_out),

    .par_out                    (pci_mux_par_out),
    .par_en_out                 (pci_mux_par_en_out),
    .perr_out                   (pci_mux_perr_out),
    .perr_en_out                (pci_mux_perr_en_out),
    .serr_out                   (pci_mux_serr_out),
    .serr_en_out                (pci_mux_serr_en_out),
    .req_in                     (pci_mux_req_in),
    .req_out                    (pci_mux_req_out),
    .req_en_out                 (pci_mux_req_en_out),
    .pci_irdy_in                (pci_mux_pci_irdy_in),
    .pci_trdy_in                (pci_mux_pci_trdy_in),
    .pci_frame_in               (pci_mux_pci_frame_in),
    .pci_stop_in                (pci_mux_pci_stop_in),
    .ad_en_unregistered_out     (pci_mux_ad_en_unregistered_out)
);

CUR_OUT_REG output_backup
(
    .reset_in               (reset),
    .clk_in                 (pci_clk),
    .frame_in               (pci_mux_frame_in),
    .frame_en_in            (pci_mux_frame_en_in),
    .frame_load_in          (pci_mux_frame_load_in),
    .irdy_in                (pci_mux_irdy_in),
    .irdy_en_in             (pci_mux_irdy_en_in),
    .devsel_in              (pci_mux_devsel_in),
    .trdy_in                (pci_mux_trdy_in),
    .trdy_en_in             (pci_mux_trdy_en_in),
    .stop_in                (pci_mux_stop_in),
    .ad_load_in             (pci_mux_ad_load_out),
    .cbe_in                 (pci_mux_cbe_in),
    .cbe_en_in              (pci_mux_cbe_en_in),
    .mas_ad_in              (pci_mux_mas_ad_in),
    .tar_ad_in              (pci_mux_tar_ad_in),

    .mas_ad_en_in           (pci_mux_mas_ad_en_in),
    .tar_ad_en_in           (pci_mux_tar_ad_en_in),
    .ad_en_unregistered_in  (pci_mux_ad_en_unregistered_out),

    .par_in                 (pci_mux_par_in),
    .par_en_in              (pci_mux_par_en_in),
    .perr_in                (pci_mux_perr_in),
    .perr_en_in             (pci_mux_perr_en_in),
    .serr_in                (pci_mux_serr_in),
    .serr_en_in             (pci_mux_serr_en_in),

    .frame_out              (out_bckp_frame_out),
    .frame_en_out           (out_bckp_frame_en_out),
    .irdy_out               (out_bckp_irdy_out),
    .irdy_en_out            (out_bckp_irdy_en_out),
    .devsel_out             (out_bckp_devsel_out),
    .trdy_out               (out_bckp_trdy_out),
    .trdy_en_out            (out_bckp_trdy_en_out),
    .stop_out               (out_bckp_stop_out),
    .cbe_out                (out_bckp_cbe_out),
    .ad_out                 (out_bckp_ad_out),
    .ad_en_out              (out_bckp_ad_en_out),
    .cbe_en_out             (out_bckp_cbe_en_out),
    .tar_ad_en_out          (out_bckp_tar_ad_en_out),
    .mas_ad_en_out          (out_bckp_mas_ad_en_out),

    .par_out                (out_bckp_par_out),
    .par_en_out             (out_bckp_par_en_out),
    .perr_out               (out_bckp_perr_out),
    .perr_en_out            (out_bckp_perr_en_out),
    .serr_out               (out_bckp_serr_out),
    .serr_en_out            (out_bckp_serr_en_out)
) ;

// PARITY CHECKER INPUTS
wire            parchk_pci_par_in               =   PCI_PAR_IN ;
wire            parchk_pci_perr_in              =   PCI_PERRn_IN ;
wire            parchk_pci_frame_reg_in         =   in_reg_frame_out ;
wire            parchk_pci_frame_en_in          =   out_bckp_frame_en_out ;
wire            parchk_pci_irdy_en_in           =   out_bckp_irdy_en_out ;
wire            parchk_pci_irdy_reg_in          =   in_reg_irdy_out ;
wire            parchk_pci_trdy_reg_in          =   in_reg_trdy_out ;


wire            parchk_pci_trdy_en_in           =   out_bckp_trdy_en_out ;


wire    [31:0]  parchk_pci_ad_out_in            =   out_bckp_ad_out ;
wire    [31:0]  parchk_pci_ad_reg_in            =   in_reg_ad_out ;
wire    [3:0]   parchk_pci_cbe_in_in            =   PCI_CBEn_IN ;
wire    [3:0]   parchk_pci_cbe_reg_in           =   in_reg_cbe_out ;
wire    [3:0]   parchk_pci_cbe_out_in           =   out_bckp_cbe_out ;
wire            parchk_pci_ad_en_in             =   out_bckp_ad_en_out ;
wire            parchk_par_err_response_in      =   conf_perr_response_out ;
wire            parchk_serr_enable_in           =   conf_serr_enable_out ;

wire            parchk_pci_perr_out_in          =   out_bckp_perr_out ;
wire            parchk_pci_serr_en_in           =   out_bckp_serr_en_out ;
wire            parchk_pci_serr_out_in          =   out_bckp_serr_out ;
wire            parchk_pci_cbe_en_in            =   out_bckp_cbe_en_out ;

wire            parchk_pci_par_en_in            =   out_bckp_par_en_out ;

PCI_PARITY_CHECK parity_checker
(
    .reset_in               (reset),
    .clk_in                 (pci_clk),
    .pci_par_in             (parchk_pci_par_in),
    .pci_par_out            (parchk_pci_par_out),
    .pci_par_en_out         (parchk_pci_par_en_out),
    .pci_par_en_in          (parchk_pci_par_en_in),
    .pci_perr_in            (parchk_pci_perr_in),
    .pci_perr_out           (parchk_pci_perr_out),
    .pci_perr_en_out        (parchk_pci_perr_en_out),
    .pci_perr_out_in        (parchk_pci_perr_out_in),
    .pci_serr_out           (parchk_pci_serr_out),
    .pci_serr_out_in        (parchk_pci_serr_out_in),
    .pci_serr_en_out        (parchk_pci_serr_en_out),
    .pci_serr_en_in         (parchk_pci_serr_en_in),
    .pci_frame_reg_in       (parchk_pci_frame_reg_in),
    .pci_frame_en_in        (parchk_pci_frame_en_in),
    .pci_irdy_en_in         (parchk_pci_irdy_en_in),
    .pci_irdy_reg_in        (parchk_pci_irdy_reg_in),
    .pci_trdy_reg_in        (parchk_pci_trdy_reg_in),
    .pci_trdy_en_in         (parchk_pci_trdy_en_in),
    .pci_ad_out_in          (parchk_pci_ad_out_in),
    .pci_ad_reg_in          (parchk_pci_ad_reg_in),
    .pci_cbe_in_in          (parchk_pci_cbe_in_in),
    .pci_cbe_reg_in         (parchk_pci_cbe_reg_in),
    .pci_cbe_en_in          (parchk_pci_cbe_en_in),
    .pci_cbe_out_in         (parchk_pci_cbe_out_in),
    .pci_ad_en_in           (parchk_pci_ad_en_in),
    .par_err_response_in    (parchk_par_err_response_in),
    .par_err_detect_out     (parchk_par_err_detect_out),
    .perr_mas_detect_out    (parchk_perr_mas_detect_out),
    .serr_enable_in         (parchk_serr_enable_in),
    .sig_serr_out           (parchk_sig_serr_out)
);

wire            in_reg_gnt_in    = PCI_GNTn_IN ;
wire            in_reg_frame_in  = PCI_FRAMEn_IN ;
wire            in_reg_irdy_in   = PCI_IRDYn_IN ;
wire            in_reg_trdy_in   = PCI_TRDYn_IN ;
wire            in_reg_stop_in   = PCI_STOPn_IN ;
wire            in_reg_devsel_in = PCI_DEVSELn_IN ;
wire            in_reg_idsel_in  = PCI_IDSEL_IN ;
wire    [31:0]  in_reg_ad_in     = PCI_AD_IN ;
wire    [3:0]   in_reg_cbe_in    = PCI_CBEn_IN ;

PCI_IN_REG input_register
(
    .reset_in       (reset),
    .clk_in         (pci_clk),

    .pci_gnt_in     (in_reg_gnt_in),
    .pci_frame_in   (in_reg_frame_in),
    .pci_irdy_in    (in_reg_irdy_in),
    .pci_trdy_in    (in_reg_trdy_in),
    .pci_stop_in    (in_reg_stop_in),
    .pci_devsel_in  (in_reg_devsel_in),
    .pci_idsel_in   (in_reg_idsel_in),
    .pci_ad_in      (in_reg_ad_in),
    .pci_cbe_in     (in_reg_cbe_in),

    .pci_gnt_reg_out    (in_reg_gnt_out),
    .pci_frame_reg_out  (in_reg_frame_out),
    .pci_irdy_reg_out   (in_reg_irdy_out),
    .pci_trdy_reg_out   (in_reg_trdy_out),
    .pci_stop_reg_out   (in_reg_stop_out),
    .pci_devsel_reg_out (in_reg_devsel_out),
    .pci_idsel_reg_out  (in_reg_idsel_out),
    .pci_ad_reg_out     (in_reg_ad_out),
    .pci_cbe_reg_out    (in_reg_cbe_out)
);

endmodule
