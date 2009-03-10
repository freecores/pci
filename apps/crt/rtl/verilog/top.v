//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "top.v"                                           ////
////                                                              ////
////  This file is part of the PCI bridge sample aplication       ////
////  project (CRT controller).                                   ////
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
// Revision 1.1.1.1  2001/10/02 15:33:33  mihad
// New project directory structure
//
//

// This top module is used for simulation and synthesys of CRT controller
// sample aplication.

module TOP
(
    CLK,
    AD,
    CBE,
    RST,
    INTA,
    REQ,
    GNT,
    FRAME,
    IRDY,
    IDSEL,
    DEVSEL,
    TRDY,
    STOP,
    PAR,
    PERR,
    SERR,

/*    CLK_I,
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
    ERR_I    */

    CRT_CLK,
    HSYNC,
    VSYNC,
    RGB,
    LED
);

input           CLK ;
inout   [31:0]  AD ;
inout   [3:0]   CBE ;
inout           RST ;
inout           INTA ;
output          REQ ;
input           GNT ;
inout           FRAME ;
inout           IRDY ;
input           IDSEL ;
inout           DEVSEL ;
inout           TRDY ;
inout           STOP ;
inout           PAR ;
inout           PERR ;
output          SERR ;

input           CRT_CLK ;
// CRT outputs
output          HSYNC ;
output          VSYNC ;
output  [15:4]  RGB ;
output			LED ;

// WISHBONE system signals
wire    RST_I = 1'b0 ;
wire    RST_O ;
wire    INT_I = 1'b0 ;
wire    INT_O ;

wire [15:0] rgb_int ;
// WISHBONE slave interface
wire    [31:0]  ADR_I ;
wire    [31:0]  SDAT_I ;
wire    [31:0]  SDAT_O ;
wire    [3:0]   SEL_I ;
wire            CYC_I ;
wire            STB_I ;
wire            WE_I  ;
wire            CAB_I ;
wire            ACK_O ;
wire            RTY_O ;
wire            ERR_O ;

// WISHBONE master interface
wire    [31:0]  ADR_O ;
wire    [31:0]  MDAT_I ;
wire    [31:0]  MDAT_O ;
wire    [3:0]   SEL_O ;
wire            CYC_O ;
wire            STB_O ;
wire            WE_O  ;
wire            CAB_O ;
wire            ACK_I ;
wire            RTY_I ;
wire            ERR_I ;

wire    [31:0]  AD_out ;
wire    [31:0]  AD_en ;


wire    [31:0]  AD_in = AD ;

wire    [3:0]   CBE_in = CBE ;
wire    [3:0]   CBE_out ;
wire    [3:0]   CBE_en ;



wire            RST_in = RST ;
wire            RST_out ;
wire            RST_en ;

wire            INTA_in = INTA ;
wire            INTA_en ;
wire            INTA_out ;

wire            REQ_en ;
wire            REQ_out ;

wire            FRAME_in = FRAME ;
wire            FRAME_out ;
wire            FRAME_en ;

wire            IRDY_in = IRDY ;
wire            IRDY_out ;
wire            IRDY_en ;

wire            DEVSEL_in = DEVSEL ;
wire            DEVSEL_out ;
wire            DEVSEL_en ;

wire            TRDY_in = TRDY ;
wire            TRDY_out ;
wire            TRDY_en ;

wire            STOP_in = STOP ;
wire            STOP_out ;
wire            STOP_en ;

wire            PAR_in = PAR ;
wire            PAR_out ;
wire            PAR_en ;

wire            PERR_in = PERR ;
wire            PERR_out ;
wire            PERR_en ;

wire            SERR_out ;
wire            SERR_en ;

PCI_BRIDGE32 bridge
(
    // WISHBONE system signals
    .CLK_I(CRT_CLK),
    .RST_I(RST_I),
    .RST_O(RST_O),
    .INT_I(INT_I),
    .INT_O(INT_O),

    // WISHBONE slave interface
    .ADR_I(ADR_I),
    .SDAT_I(SDAT_I),
    .SDAT_O(SDAT_O),
    .SEL_I(SEL_I),
    .CYC_I(CYC_I),
    .STB_I(STB_I),
    .WE_I(WE_I),
    .CAB_I(CAB_I),
    .ACK_O(ACK_O),
    .RTY_O(RTY_O),
    .ERR_O(ERR_O),

    // WISHBONE master interface
    .ADR_O(ADR_O),
    .MDAT_I(MDAT_I),
    .MDAT_O(MDAT_O),
    .SEL_O(SEL_O),
    .CYC_O(CYC_O),
    .STB_O(STB_O),
    .WE_O(WE_O),
    .CAB_O(CAB_O),
    .ACK_I(ACK_I),
    .RTY_I(RTY_I),
    .ERR_I(ERR_I),

    // pci interface - system pins
    .PCI_CLK_IN (CLK),
    .PCI_RSTn_IN ( RST_in ),
    .PCI_RSTn_OUT ( RST_out ),
    .PCI_INTAn_IN ( INTA_in ),
    .PCI_INTAn_OUT( INTA_out),
    .PCI_RSTn_EN_OUT( RST_en),
    .PCI_INTAn_EN_OUT(INTA_en),

    // arbitration pins
    .PCI_REQn_OUT( REQ_out ),
    .PCI_REQn_EN_OUT ( REQ_en ),

    .PCI_GNTn_IN( GNT ),

    // protocol pins
    .PCI_FRAMEn_IN( FRAME_in),
    .PCI_FRAMEn_OUT( FRAME_out ),

    .PCI_FRAMEn_EN_OUT( FRAME_en ),
    .PCI_IRDYn_EN_OUT ( IRDY_en ),
    .PCI_DEVSELn_EN_OUT ( DEVSEL_en ),
    .PCI_TRDYn_EN_OUT ( TRDY_en ),
    .PCI_STOPn_EN_OUT ( STOP_en ),
    .PCI_AD_EN_OUT(AD_en),
    .PCI_CBEn_EN_OUT ( CBE_en) ,

    .PCI_IRDYn_IN ( IRDY_in ),
    .PCI_IRDYn_OUT ( IRDY_out ),

    .PCI_IDSEL_IN ( IDSEL ),

    .PCI_DEVSELn_IN( DEVSEL_in ),
    .PCI_DEVSELn_OUT ( DEVSEL_out ),

    .PCI_TRDYn_IN ( TRDY_in ),
    .PCI_TRDYn_OUT ( TRDY_out ),

    .PCI_STOPn_IN( STOP_in ),
    .PCI_STOPn_OUT ( STOP_out ),

    // data transfer pins
    .PCI_AD_IN(AD_in),
    .PCI_AD_OUT (AD_out),

    .PCI_CBEn_IN( CBE_in ),
    .PCI_CBEn_OUT ( CBE_out ),

    // parity generation and checking pins
    .PCI_PAR_IN ( PAR_in ),
    .PCI_PAR_OUT ( PAR_out ),
    .PCI_PAR_EN_OUT ( PAR_en ),

    .PCI_PERRn_IN ( PERR_in ),
    .PCI_PERRn_OUT ( PERR_out ),
    .PCI_PERRn_EN_OUT ( PERR_en ),

    // system error pin
    .PCI_SERRn_OUT ( SERR_out ),
    .PCI_SERRn_EN_OUT ( SERR_en )
);

// PCI IO buffers instantiation
bufif0 AD_buf0   ( AD[0],  AD_out[0], AD_en[0]) ;
bufif0 AD_buf1   ( AD[1],  AD_out[1], AD_en[1]) ;
bufif0 AD_buf2   ( AD[2],  AD_out[2], AD_en[2]) ;
bufif0 AD_buf3   ( AD[3],  AD_out[3], AD_en[3]) ;
bufif0 AD_buf4   ( AD[4],  AD_out[4], AD_en[4]) ;
bufif0 AD_buf5   ( AD[5],  AD_out[5], AD_en[5]) ;
bufif0 AD_buf6   ( AD[6],  AD_out[6], AD_en[6]) ;
bufif0 AD_buf7   ( AD[7],  AD_out[7], AD_en[7]) ;
bufif0 AD_buf8   ( AD[8],  AD_out[8], AD_en[8]) ;
bufif0 AD_buf9   ( AD[9],  AD_out[9], AD_en[9]) ;
bufif0 AD_buf10  ( AD[10], AD_out[10],AD_en[10] ) ;
bufif0 AD_buf11  ( AD[11], AD_out[11],AD_en[11] ) ;
bufif0 AD_buf12  ( AD[12], AD_out[12],AD_en[12] ) ;
bufif0 AD_buf13  ( AD[13], AD_out[13],AD_en[13] ) ;
bufif0 AD_buf14  ( AD[14], AD_out[14],AD_en[14] ) ;
bufif0 AD_buf15  ( AD[15], AD_out[15],AD_en[15] ) ;
bufif0 AD_buf16  ( AD[16], AD_out[16],AD_en[16] ) ;
bufif0 AD_buf17  ( AD[17], AD_out[17],AD_en[17] ) ;
bufif0 AD_buf18  ( AD[18], AD_out[18],AD_en[18] ) ;
bufif0 AD_buf19  ( AD[19], AD_out[19],AD_en[19] ) ;
bufif0 AD_buf20  ( AD[20], AD_out[20],AD_en[20] ) ;
bufif0 AD_buf21  ( AD[21], AD_out[21],AD_en[21] ) ;
bufif0 AD_buf22  ( AD[22], AD_out[22],AD_en[22] ) ;
bufif0 AD_buf23  ( AD[23], AD_out[23],AD_en[23] ) ;
bufif0 AD_buf24  ( AD[24], AD_out[24],AD_en[24] ) ;
bufif0 AD_buf25  ( AD[25], AD_out[25],AD_en[25] ) ;
bufif0 AD_buf26  ( AD[26], AD_out[26],AD_en[26] ) ;
bufif0 AD_buf27  ( AD[27], AD_out[27],AD_en[27] ) ;
bufif0 AD_buf28  ( AD[28], AD_out[28],AD_en[28] ) ;
bufif0 AD_buf29  ( AD[29], AD_out[29],AD_en[29] ) ;
bufif0 AD_buf30  ( AD[30], AD_out[30],AD_en[30] ) ;
bufif0 AD_buf31  ( AD[31], AD_out[31],AD_en[31] ) ;

bufif0 CBE_buf0 ( CBE[0], CBE_out[0], CBE_en[0] ) ;
bufif0 CBE_buf1 ( CBE[1], CBE_out[1], CBE_en[1] ) ;
bufif0 CBE_buf2 ( CBE[2], CBE_out[2], CBE_en[2] ) ;
bufif0 CBE_buf3 ( CBE[3], CBE_out[3], CBE_en[3] ) ;

bufif0 FRAME_buf    ( FRAME, FRAME_out, FRAME_en ) ;
bufif0 IRDY_buf     ( IRDY, IRDY_out, IRDY_en ) ;
bufif0 DEVSEL_buf   ( DEVSEL, DEVSEL_out, DEVSEL_en ) ;
bufif0 TRDY_buf     ( TRDY, TRDY_out, TRDY_en ) ;
bufif0 STOP_buf     ( STOP, STOP_out, STOP_en ) ;

bufif0 RST_buf      ( RST, RST_out, RST_en ) ;
bufif0 INTA_buf     ( INTA, INTA_out, INTA_en) ;
bufif0 REQ_buf      ( REQ, REQ_out, REQ_en ) ;
bufif0 PAR_buf      ( PAR, PAR_out, PAR_en ) ;
bufif0 PERR_buf     ( PERR, PERR_out, PERR_en ) ;
bufif0 SERR_buf     ( SERR, SERR_out, SERR_en ) ;

wire crt_hsync ;
wire crt_vsync ;

// CRT controler instance
ssvga_top CRT
(
	// Clock and reset
	.wb_clk_i(CRT_CLK),
    .wb_rst_i(RST_O),

	// WISHBONE Master I/F
	.wbm_cyc_o  (CYC_I),
    .wbm_stb_o  (STB_I),
    .wbm_sel_o  (SEL_I),
    .wbm_we_o   (WE_I),
	.wbm_adr_o  (ADR_I),
    .wbm_dat_o  (SDAT_I),
    .wbm_cab_o  (CAB_I),
	.wbm_dat_i  (SDAT_O),
    .wbm_ack_i  (ACK_O),
    .wbm_err_i  (ERR_O),
    .wbm_rty_i  (RTY_O),

	// WISHBONE Slave I/F
	.wbs_cyc_i  (CYC_O),
    .wbs_stb_i  (STB_O),
    .wbs_sel_i  (SEL_O),
    .wbs_we_i   (WE_O),
	.wbs_adr_i  (ADR_O),
    .wbs_dat_i  (MDAT_O),
    .wbs_cab_i  (CAB_O),
	.wbs_dat_o  (MDAT_I),
    .wbs_ack_o  (ACK_I),
    .wbs_err_o  (ERR_I),
    .wbs_rty_o  (RTY_I),

	// Signals to VGA display
	.pad_hsync_o    (crt_hsync),
    .pad_vsync_o    (crt_vsync),
    .pad_rgb_o      (rgb_int),
    .led_o			(LED)
);

CRTC_IOB crt_out_reg
(
    .reset_in(RST_O),
    .clk_in(CRT_CLK),
    .hsync_in(crt_hsync),
    .vsync_in(crt_vsync),
    .rgb_in(rgb_int[15:4]),
    .hsync_out(HSYNC),
    .vsync_out(VSYNC),
    .rgb_out(RGB)
) ;

endmodule