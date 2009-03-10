//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "constants.v"                                     ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - Miha Dolenc (mihad@opencores.org)                     ////
////      - Tadej Markovic (tadej@opencores.org)                  ////
////                                                              ////
////  All additional information is avaliable in the README.pdf   ////
////  file.                                                       ////
////                                                              ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000 Miha Dolenc, mihad@opencores.org          ////
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
// Revision 1.2  2001/07/17 15:11:14  mihad
// Added some WISHBONE slave defines
//
//

////////////////////////////////////////////////////////////////////////
////                                                                ////
//// FIFO parameters define behaviour of FIFO control logic and     ////
//// FIFO depths.                                                   ////
////                                                                ////
////////////////////////////////////////////////////////////////////////

//  FPGA implementation definitions :
//  FPGA definition is optional - if it's defined, BLOCK SelectRam+ will
//  be used for FIFO storage space. Implementation is SYNCHRONOUS regardles
//  of SYNCHRONOUS parameter definition. Smallest FPGA must have at leat 6
//  block select rams available. For bigger FPGAs, there is possibility of
//  defining BIG - only for FPGAs with 12 or more available block rams
//  Defining FPGA without BIG limits any FIFO to max depth of 128 ( address
//  length 7). Large FPGAs with BIG definition provide max 256 ( address length
//  8) depth for each FIFO. FIFO depth MUST be power of 2, so address length can
//  be defined
//  Minimum FIFO depth of any FIFO is 8 - control logic is such that address
// lengths less than 3 are not supported
`define FPGA
`define WBW_DEPTH 16
`define WBW_ADDR_LENGTH 4 
`define WBR_DEPTH 32
`define WBR_ADDR_LENGTH 5 
`define PCIW_DEPTH 64
`define PCIW_ADDR_LENGTH 6
`define PCIR_DEPTH 128
`define PCIR_ADDR_LENGTH 7
//`define BIG

// if FPGA is not defined (commented out), there can still be control logic
// for synchronous rams used by defining SYNCHRONOUS
`define SYNCHRONOUS

// if neither FPGA or SYNCRONOUS are defined, control logic for asynchronous
// rams is included


// control bus encoding definitions
`define ADDRESS 4'hf    // address entry
`define LAST 4'h0       // last data entry in transaction
`define DATA_ERROR 4'h8 // data was read with error signaled
`define DATA 4'h1       // intermediate data beat in a burst

// Flip flop delay included in assignements to every register
`define FF_DELAY 2 // FF propagation delay

`timescale 100ps/10ps

// PCI bridge HOST/GUEST implentation
// - for HOST implementation 'HOST' MUST be written othervise there is GUEST 
//   implementation and 'GUEST MUST be written !!!
`define HOST	

// MAX Retry counter value for WISHBONE Master state-machine
// 	This value is 8-bit because of 8-bit retry counter !!!
`define WB_RTY_CNT_MAX			8'hff

// no. of PCI Target IMAGES
// - The maximum number of images is "6". By default there are first two images
//   used and the first (PCI_IMAGE0) is assigned to Configuration space! With a
//   'define' PCI_IMAGEx you choose the number of used PCI IMAGES in a bridge
//   without PCI_IMAGE0 (e.g. PCI_IMAGE3 tells, that PCI_IMAGE1, PCI_IMAGE2 and
//   PCI_IMAGE3 are used for mapping the space from WB to PCI. Offcourse, 
//   PCI_IMAGE0 is assigned to Configuration space). That leave us PCI_IMAGE5 as
//   the maximum number of images.
//   There is one exeption, when the core is implemented as HOST. If so, then the
//   PCI specification allowes the Configuration space NOT to be visible on the
//   PCI bus. With `define PCI_IMAGE6 (and `define HOST), we assign PCI_IMAGE0
//   to normal WB to PCI image and not to configuration space!
`define PCI_IMAGE6

// no. of WISHBONE Slave IMAGES
// - The maximum number of images is "6". By default there are first two images
//   used and the first (WB_IMAGE0) is assigned to Configuration space! With a
//   'define' WB_IMAGEx you choose the number of used WB IMAGES in a bridge
//   without WB_IMAGE0 (e.g. WB_IMAGE3 tells, that WB_IMAGE1, WB_IMAGE2 and
//   WB_IMAGE3 are used for mapping the space from PCI to WB. Offcourse, 
//   WB_IMAGE0 is assigned to Configuration space). That leave us WB_IMAGE5 as
//   the maximum number of images.
`define WB_IMAGE5

// Configuration space base address for accesses from WISHBONE bus
`define WB_CONFIGURATION_BASE 20'hCCCC_C

// PCI target & WB slave ADDRESS names for configuration space !!!
`define P_IMG_CTRL0_ADDR		12'h100
`define P_BA0_ADDR				12'h104	// = PCI_CONF_SPC_BAR
`define P_AM0_ADDR				12'h108
`define P_TA0_ADDR				12'h10c
`define P_IMG_CTRL1_ADDR        12'h110
`define	P_BA1_ADDR				12'h114
`define	P_AM1_ADDR				12'h118
`define	P_TA1_ADDR				12'h11c
`define	P_IMG_CTRL2_ADDR		12'h120
`define	P_BA2_ADDR				12'h124
`define	P_AM2_ADDR				12'h128
`define	P_TA2_ADDR				12'h12c
`define	P_IMG_CTRL3_ADDR		12'h130
`define	P_BA3_ADDR				12'h134
`define	P_AM3_ADDR				12'h138
`define	P_TA3_ADDR				12'h13c
`define	P_IMG_CTRL4_ADDR		12'h140
`define	P_BA4_ADDR				12'h144
`define	P_AM4_ADDR				12'h148
`define	P_TA4_ADDR				12'h14c
`define	P_IMG_CTRL5_ADDR		12'h150
`define	P_BA5_ADDR				12'h154
`define	P_AM5_ADDR				12'h158
`define	P_TA5_ADDR				12'h15c
`define	P_ERR_CS_ADDR			12'h160
`define	P_ERR_ADDR_ADDR			12'h164
`define	P_ERR_DATA_ADDR			12'h168

`define	WB_CONF_SPC_BAR_ADDR	12'h800
`define	W_IMG_CTRL1_ADDR		12'h804
`define	W_BA1_ADDR				12'h808
`define	W_AM1_ADDR				12'h80c
`define	W_TA1_ADDR				12'h810
`define	W_IMG_CTRL2_ADDR		12'h814
`define	W_BA2_ADDR				12'h818
`define	W_AM2_ADDR				12'h81c
`define	W_TA2_ADDR				12'h820
`define	W_IMG_CTRL3_ADDR		12'h824
`define	W_BA3_ADDR				12'h828
`define	W_AM3_ADDR				12'h82c
`define	W_TA3_ADDR				12'h830
`define	W_IMG_CTRL4_ADDR		12'h834
`define	W_BA4_ADDR				12'h838
`define	W_AM4_ADDR				12'h83c
`define	W_TA4_ADDR				12'h840
`define	W_IMG_CTRL5_ADDR		12'h844
`define	W_BA5_ADDR				12'h848
`define	W_AM5_ADDR				12'h84c
`define	W_TA5_ADDR				12'h850
`define	W_ERR_CS_ADDR			12'h854
`define	W_ERR_ADDR_ADDR			12'h858
`define	W_ERR_DATA_ADDR			12'h85c
`define	CNF_ADDR_ADDR			12'h860
`define	CNF_DATA_ADDR			12'h864
`define	INT_ACK_ADDR			12'h868
`define	ICR_ADDR				12'hff8
`define	ISR_ADDR		        12'hffc

// the width of the registers
`define REG_WIDTH 32

// timing delays
`define DLY_L1 1
`define DLY_L2 2
`define DLY_L3 3
`define DLY_L4 4
`define DLY_L5 5

/*-----------------------------------------------------------------------------------------------------------
[000h-00Ch] First 4 DWORDs (32-bit) of PCI configuration header - the same regardless of the HEADER type !
			r_ prefix is a sign for read only registers
	Vendor_ID is an ID for a specific vendor defined by PCI_SIG - 2321h does not belong to anyone (e.g. 
	Xilinx's Vendor_ID is 10EEh and Altera's Vendor_ID is 1172h). Device_ID and Revision_ID should be used
	together by application. 
    66MHz goes into 66MHz capable bit and indicates that device can operate on 66MHz PCI bus.
-----------------------------------------------------------------------------------------------------------*/
`define HEADER_DEVICE_ID    16'h0001
`define HEADER_VENDOR_ID    16'h2321
`define HEADER_REVISION_ID  8'h01
`define HEADER_66MHz        1'b0 