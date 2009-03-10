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
////  All additional information is avaliable in the README.txt   ////
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
// Revision 1.2  2001/08/09 14:42:34  mihad
// Changed files during testing
//
// Revision 1.1  2001/08/06 18:12:43  mihad
// Pocasi delamo kompletno zadevo
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
`define WBW_DEPTH 32
`define WBW_ADDR_LENGTH 4 
`define WBR_DEPTH 32
`define WBR_ADDR_LENGTH 5 
`define PCIW_DEPTH 32
`define PCIW_ADDR_LENGTH 6
`define PCIR_DEPTH 32
`define PCIR_ADDR_LENGTH 7
//`define BIG

// if FPGA is not defined (commented out), there can still be control logic
// for synchronous rams used by defining SYNCHRONOUS
`define SYNCHRONOUS

// if neither FPGA or SYNCRONOUS are defined, control logic for asynchronous
// rams is included


// control bus encoding definitions
`define ADDRESS 4'h8    // address entry
`define LAST 4'h1       // last data entry in transaction
`define DATA_ERROR 4'h2 // data was read with error signaled
`define DATA 4'h0       // intermediate data beat in a burst

// defines on which bit in control bus means what
`define ADDR_CTRL_BIT 3
`define LAST_CTRL_BIT 0
`define DATA_ERROR_CTRL_BIT 1
`define UNUSED_CTRL_BIT 2
`define	BURST_BIT 2

// Flip flop delay included in assignements to every register
`define FF_DELAY 2 // FF propagation delay

`timescale 1ns/10ps

// PCI bridge HOST/GUEST implentation
// - for HOST implementation 'HOST' MUST be written othervise there is GUEST 
//   implementation and 'GUEST MUST be written !!!
`define GUEST

// MAX Retry counter value for WISHBONE Master state-machine
// 	This value is 8-bit because of 8-bit retry counter !!!
`define WB_RTY_CNT_MAX			8'hff

// MAX Retry counter value for PCI Master state-machine
// 	This value is 8-bit because of 8-bit retry counter !!!
`define PCI_RTY_CNT_MAX			8'h08

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
`define PCI_IMAGE3

`define PCI_AM0 20'hfff0_0
`define PCI_AM1 20'hfff0_0
`define PCI_AM2 20'h0000_0
`define PCI_AM3 20'h0000_0
`define PCI_AM4 20'h0000_0
`define PCI_AM5 20'h0000_0
// no. of WISHBONE Slave IMAGES
// - The maximum number of images is "6". By default there are first two images
//   used and the first (WB_IMAGE0) is assigned to Configuration space! With a
//   'define' WB_IMAGEx you choose the number of used WB IMAGES in a bridge
//   without WB_IMAGE0 (e.g. WB_IMAGE3 tells, that WB_IMAGE1, WB_IMAGE2 and
//   WB_IMAGE3 are used for mapping the space from PCI to WB. Offcourse, 
//   WB_IMAGE0 is assigned to Configuration space). That leave us WB_IMAGE5 as
//   the maximum number of images.
`define WB_IMAGE5
// if WB_CNF_IMAGE is commented out, than access to configuration space from WISHBONE for GUEST bridges is disabled alltogether ( even read only )
//`define WB_CNF_IMAGE

// decode speed for WISHBONE definition - initial cycle on WISHBONE bus will take 1 WS for FAST, 2 WSs for MEDIUM and 3 WSs for slow. 
// slower the decode speed, faster the WISHBONE clock can be
//`define WB_DECODE_FAST
`define WB_DECODE_MEDIUM
//`define WB_DECODE_SLOW

// definition of how many address lines are compared on address decoding for WISHBONE and PCI images. Put a number of smallest image used here.
// Minimum number is 1 and maximum number is 20 ( 1 = only 2GB images can be done, 20 - 4KB image is smallest possible) 
`define WB_NUM_OF_DEC_ADDR_LINES 20
`define PCI_NUM_OF_DEC_ADDR_LINES 20

// Configuration space base address for accesses from WISHBONE bus
`define WB_CONFIGURATION_BASE 20'hCCCC_C

// PCI target & WB slave ADDRESS names for configuration space !!!
//   ALL VALUES are without 2 LSBits AND there is required that address bit [8] is set while
//   accessing this registers, otherwise the configuration header will be accessed !!!
`define P_IMG_CTRL0_ADDR		6'h00
`define P_BA0_ADDR				6'h01	// = PCI_CONF_SPC_BAR
`define P_AM0_ADDR				6'h02
`define P_TA0_ADDR				6'h03
`define P_IMG_CTRL1_ADDR        6'h04
`define	P_BA1_ADDR				6'h05
`define	P_AM1_ADDR				6'h06
`define	P_TA1_ADDR				6'h07
`define	P_IMG_CTRL2_ADDR		6'h08
`define	P_BA2_ADDR				6'h09
`define	P_AM2_ADDR				6'h0a
`define	P_TA2_ADDR				6'h0b
`define	P_IMG_CTRL3_ADDR		6'h0c
`define	P_BA3_ADDR				6'h0d
`define	P_AM3_ADDR				6'h0e
`define	P_TA3_ADDR				6'h0f
`define	P_IMG_CTRL4_ADDR		6'h10
`define	P_BA4_ADDR				6'h11
`define	P_AM4_ADDR				6'h12
`define	P_TA4_ADDR				6'h13
`define	P_IMG_CTRL5_ADDR		6'h14
`define	P_BA5_ADDR				6'h15
`define	P_AM5_ADDR				6'h16
`define	P_TA5_ADDR				6'h17
`define	P_ERR_CS_ADDR			6'h18
`define	P_ERR_ADDR_ADDR			6'h19
`define	P_ERR_DATA_ADDR			6'h1a

`define	WB_CONF_SPC_BAR_ADDR	6'h20
`define	W_IMG_CTRL1_ADDR		6'h21
`define	W_BA1_ADDR				6'h22
`define	W_AM1_ADDR				6'h23
`define	W_TA1_ADDR				6'h24
`define	W_IMG_CTRL2_ADDR		6'h25
`define	W_BA2_ADDR				6'h26
`define	W_AM2_ADDR				6'h27
`define	W_TA2_ADDR				6'h28
`define	W_IMG_CTRL3_ADDR		6'h29
`define	W_BA3_ADDR				6'h2a
`define	W_AM3_ADDR				6'h2b
`define	W_TA3_ADDR				6'h2c
`define	W_IMG_CTRL4_ADDR		6'h2d
`define	W_BA4_ADDR				6'h2e
`define	W_AM4_ADDR				6'h2f
`define	W_TA4_ADDR				6'h30
`define	W_IMG_CTRL5_ADDR		6'h31
`define	W_BA5_ADDR				6'h32
`define	W_AM5_ADDR				6'h33
`define	W_TA5_ADDR				6'h34
`define	W_ERR_CS_ADDR			6'h35
`define	W_ERR_ADDR_ADDR			6'h36
`define	W_ERR_DATA_ADDR			6'h37
`define	CNF_ADDR_ADDR			6'h38
// Following two registers are not implemented in a configuration space but in a WishBone unit!
`define	CNF_DATA_ADDR			6'h39
`define	INT_ACK_ADDR			6'h3a
// ------------------------------------
`define	ICR_ADDR				6'h3b
`define	ISR_ADDR		        6'h3c

// the width of the registers
`define REG_WIDTH 32

// timing delays
`define DLY_L1 1
`define DLY_L2 2
`define DLY_L3 3
`define DLY_L4 4
`define DLY_L5 5

/*-----------------------------------------------------------------------------------------------------------
Core speed definition - used for simulation and bit in header register indicating 66MHz capable
defice
-----------------------------------------------------------------------------------------------------------*/

`define PCI33 
// define PCI66

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

`ifdef PCI33
    `define HEADER_66MHz        1'b0 
`else
`ifdef PCI66
    `define HEADER_66MHz        1'b1
`endif
`endif

// Implement address translation or not
`define ADDR_TRAN_IMPL

/*-----------------------------------------------------------------------------------------------------------
WISHBONE clock is specified in frequency in GHz
-----------------------------------------------------------------------------------------------------------*/
`define WB_FREQ                 0.1
