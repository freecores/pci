//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: conf_space.v                                     ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - tadej@opencores.org                                   ////
////      - Tadej Markovic                                        ////
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
//

`include "constants.v"

/*-----------------------------------------------------------------------------------------------------------
	w_ prefix is a sign for Write (and read) side of Dual-Port registers
	r_ prefix is a sign for Read only side of Dual-Port registers
In the first line there are DATA and ADDRESS ports, in the second line there are write enable and read 
enable signals with chip-select (conf_hit) for config. space.
In the third line there are output signlas from Command register of the PCI configuration header !!!
In the fourth line there are input signals to Status register of the PCI configuration header !!!
In the fifth line there is output from Latency Timer & r_Interrupt pin registers of the PCI conf. header !!!
Following are IMAGE specific registers, from which PCI_BASE_ADDR registers are the same as base address
registers from the PCI conf. header !!!
-----------------------------------------------------------------------------------------------------------*/
					// normal R/W address, data and control
module CONF_SPACE (	w_conf_address_in, w_conf_data_in, w_conf_data_out, r_conf_address_in, r_conf_data_out, 
					w_we, w_re, r_re, w_byte_en, w_clock, reset, pci_clk, wb_clk,
					// outputs from command register of the PCI header
					serr_enable, perr_response, pci_master_enable, memory_space_enable, io_space_enable,
					// inputs to status register of the PCI header
					perr_in, serr_in, master_abort_recv, target_abort_recv, target_abort_set, master_data_par_err,
					// output from cache_line_size, latency timer and r_interrupt_pin register of the PCI header
					cache_line_size, latency_tim, int_pin,
					// output from all pci IMAGE registers
					pci_base_addr0, pci_base_addr1, pci_base_addr2, pci_base_addr3, pci_base_addr4, pci_base_addr5, 
					pci_memory_io0, pci_memory_io1, pci_memory_io2, pci_memory_io3, pci_memory_io4, pci_memory_io5, 
					pci_addr_mask0, pci_addr_mask1, pci_addr_mask2, pci_addr_mask3, pci_addr_mask4, pci_addr_mask5, 
					pci_tran_addr0, pci_tran_addr1, pci_tran_addr2, pci_tran_addr3, pci_tran_addr4, pci_tran_addr5, 
					pci_img_ctrl0,  pci_img_ctrl1,  pci_img_ctrl2,  pci_img_ctrl3,  pci_img_ctrl4,  pci_img_ctrl5,
					// input to pci error control and status register, error address and error data registers
					pci_error_be, pci_error_bc, pci_error_rty_exp, pci_error_sig, pci_error_addr, pci_error_data,
					// output from pci error control and status register
					pci_error_en, pci_error_sig_set, pci_error_rty_exp_set, 
					// output from all wishbone IMAGE registers                                                                                          
					wb_base_addr0, wb_base_addr1, wb_base_addr2, wb_base_addr3, wb_base_addr4, wb_base_addr5,
					wb_memory_io0, wb_memory_io1, wb_memory_io2, wb_memory_io3, wb_memory_io4, wb_memory_io5, 
					wb_addr_mask0, wb_addr_mask1, wb_addr_mask2, wb_addr_mask3, wb_addr_mask4, wb_addr_mask5,
					wb_tran_addr0, wb_tran_addr1, wb_tran_addr2, wb_tran_addr3, wb_tran_addr4, wb_tran_addr5,
					wb_img_ctrl0,  wb_img_ctrl1,  wb_img_ctrl2,  wb_img_ctrl3,  wb_img_ctrl4,  wb_img_ctrl5, 
					// input to wb error control and status register, error address and error data registers
					wb_error_be, wb_error_bc, wb_error_rty_exp, wb_error_es, wb_error_sig, wb_error_addr, wb_error_data,
					// output from wb error control and status register
					wb_error_en, wb_error_sig_set, wb_error_rty_exp_set, 
					// output from conf. cycle generation register (sddress) & int. control register
					config_addr, icr_soft_res, serr_int_en, perr_int_en, error_int_en, int_prop_en,
					// input to interrupt status register
					isr_int_prop, isr_err_int, isr_par_err_int, isr_sys_err_int	) ;
                    
                    
/*###########################################################################################################
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Input and output ports
	======================
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
###########################################################################################################*/

// output data 
output	[31 : 0]				w_conf_data_out ;
output	[31 : 0]				r_conf_data_out ;
reg		[31 : 0]				w_conf_data_out ;
reg		[31 : 0]				r_conf_data_out ;
// input data
input	[31 : 0]				w_conf_data_in ;
// input address
input	[11 : 0]				w_conf_address_in ;
input	[11 : 0]				r_conf_address_in ;
// input control signals
input							w_we ;
input							w_re ;
input							r_re ;
input	[3 : 0]					w_byte_en ;
input							w_clock ;
input							reset ;
input							pci_clk ;
input							wb_clk ;
// PCI header outputs from command register
output							serr_enable ;
output							perr_response ;
output							pci_master_enable ;
output							memory_space_enable ;
output							io_space_enable ;
// PCI header inputs to status register
input							perr_in ;
input							serr_in ;
input							master_abort_recv ;
input							target_abort_recv ;
input							target_abort_set ;
input							master_data_par_err ;
// PCI header output from cache_line_size, latency timer and interrupt pin
output	[7 : 0]					cache_line_size ;
output	[7 : 0]					latency_tim ;
output	[2 : 0]					int_pin ; // only 3 LSbits are important!
// PCI output from image registers
output	[31 : 12]				pci_base_addr0 ; 
output  [31 : 12]               pci_base_addr1 ; 
output  [31 : 12]               pci_base_addr2 ; 
output  [31 : 12]               pci_base_addr3 ; 
output  [31 : 12]               pci_base_addr4 ; 
output  [31 : 12]               pci_base_addr5 ; 
output							pci_memory_io0 ;
output							pci_memory_io1 ;
output							pci_memory_io2 ;
output							pci_memory_io3 ;
output							pci_memory_io4 ;
output							pci_memory_io5 ;
output  [31 : 12]               pci_addr_mask0 ; 
output  [31 : 12]               pci_addr_mask1 ; 
output  [31 : 12]               pci_addr_mask2 ; 
output  [31 : 12]               pci_addr_mask3 ; 
output  [31 : 12]               pci_addr_mask4 ; 
output  [31 : 12]               pci_addr_mask5 ; 
output  [31 : 12]               pci_tran_addr0 ; 
output  [31 : 12]               pci_tran_addr1 ; 
output  [31 : 12]               pci_tran_addr2 ; 
output  [31 : 12]               pci_tran_addr3 ; 
output  [31 : 12]               pci_tran_addr4 ; 
output  [31 : 12]               pci_tran_addr5 ; 
output  [2 : 1]                 pci_img_ctrl0 ;  
output  [2 : 1]                 pci_img_ctrl1 ;  
output  [2 : 1]                 pci_img_ctrl2 ;  
output  [2 : 1]                 pci_img_ctrl3 ;  
output  [2 : 1]                 pci_img_ctrl4 ;  
output  [2 : 1]                 pci_img_ctrl5 ;  
// PCI input to pci error control and status register, error address and error data registers
input	[3 : 0]					pci_error_be ;     
input   [3 : 0]                 pci_error_bc ;     
input                           pci_error_rty_exp ;
input                           pci_error_sig ;    
input   [31 : 0]                pci_error_addr ;   
input   [31 : 0]                pci_error_data ;    
// PCI output from pci error control and status register
output							pci_error_en ; 
output							pci_error_sig_set ;
output							pci_error_rty_exp_set ;
// WISHBONE output from image registers
output	[31 : 12]   			wb_base_addr0 ;
output  [31 : 12]               wb_base_addr1 ;
output  [31 : 12]               wb_base_addr2 ;
output  [31 : 12]               wb_base_addr3 ;
output  [31 : 12]               wb_base_addr4 ;
output  [31 : 12]               wb_base_addr5 ;
output							wb_memory_io0 ;
output							wb_memory_io1 ;
output							wb_memory_io2 ;
output							wb_memory_io3 ;
output							wb_memory_io4 ;
output							wb_memory_io5 ;
output  [31 : 12]               wb_addr_mask0 ;
output  [31 : 12]               wb_addr_mask1 ;
output  [31 : 12]               wb_addr_mask2 ;
output  [31 : 12]               wb_addr_mask3 ;
output  [31 : 12]               wb_addr_mask4 ;
output  [31 : 12]               wb_addr_mask5 ;
output  [31 : 12]               wb_tran_addr0 ;
output  [31 : 12]               wb_tran_addr1 ;
output  [31 : 12]               wb_tran_addr2 ;
output  [31 : 12]               wb_tran_addr3 ;
output  [31 : 12]               wb_tran_addr4 ;
output  [31 : 12]               wb_tran_addr5 ;
output  [2 : 0]                 wb_img_ctrl0 ; 
output  [2 : 0]                 wb_img_ctrl1 ; 
output  [2 : 0]                 wb_img_ctrl2 ; 
output  [2 : 0]                 wb_img_ctrl3 ; 
output  [2 : 0]                 wb_img_ctrl4 ; 
output  [2 : 0]                 wb_img_ctrl5 ; 
// WISHBONE input to wb error control and status register, error address and error data registers
input	[3 : 0] 				wb_error_be ;     
input   [3 : 0]	                wb_error_bc ;     
input   		                wb_error_rty_exp ;
input                           wb_error_es ;     
input                           wb_error_sig ;    
input   [31 : 0]                wb_error_addr ;   
input   [31 : 0]                wb_error_data ;   
// WISHBONE output from wb error control and status register
output							wb_error_en ; 
output							wb_error_sig_set ;
output							wb_error_rty_exp_set ;
// GENERAL output from conf. cycle generation register & int. control register 
output	[23 : 0]				config_addr ; 		
output                          icr_soft_res ;
output                          serr_int_en ;
output                          perr_int_en ;
output                          error_int_en ;
output                          int_prop_en ;
// GENERAL input to interrupt status register       
input							isr_int_prop ;      
input                           isr_err_int ;       
input                           isr_par_err_int ;   
input                           isr_sys_err_int ;   


/*###########################################################################################################
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	REGISTERS definition
	====================
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
###########################################################################################################*/


/*###########################################################################################################
-------------------------------------------------------------------------------------------------------------
PCI CONFIGURATION SPACE HEADER (type 00h) registers

	BIST and some other registers are not implemented and therefor written in correct
	place with comment line. There are also some registers with NOT all bits implemented and therefor uses
	_bitX or _bitX2_X1 to sign which bit or range of bits are implemented. 
	Some special cases and examples are described below!
-------------------------------------------------------------------------------------------------------------
###########################################################################################################*/

/*-----------------------------------------------------------------------------------------------------------
[000h-00Ch] First 4 DWORDs (32-bit) of PCI configuration header - the same regardless of the HEADER type !
			r_ prefix is a sign for read only registers
	Vendor_ID is an ID for a specific vendor defined by PCI_SIG - 2321h does not belong to anyone (e.g. 
	Xilinx's Vendor_ID is 10EEh and Altera's Vendor_ID is 1172h). Device_ID and Revision_ID should be used
	together by application. Class_Code has 3 bytes to define BASE class (06h for PCI Bridge), SUB class 
	(00h for HOST type, 80h for Other Bridge type) and Interface type (00h for normal).
-----------------------------------------------------------------------------------------------------------*/
			parameter			r_vendor_id = `HEADER_VENDOR_ID ;	// 16'h2321 = 16'd8993 !!!
			parameter			r_device_id = `HEADER_DEVICE_ID ;
			reg					command_bit8 ;			
			reg					command_bit6 ;			
			reg		[2 : 0]		command_bit2_0 ;		
			reg		[15 : 11]	status_bit15_11 ;		
			parameter			r_status_bit10_9 = 2'b01 ;	// 2'b01 means MEDIUM devsel timing !!!
			reg					status_bit8 ;
			parameter			r_status_bit5 = `HEADER_66MHz ;   	// 1'b0 indicates 33 MHz capable !!!
			parameter			r_revision_id = `HEADER_REVISION_ID ;
`ifdef		HOST
			parameter			r_class_code = 24'h06_00_00 ;
`else
			parameter			r_class_code = 24'h06_80_00 ;
`endif
			reg		[7 : 0]		cache_line_size_reg	;
			reg		[7 : 0]		latency_timer ;
			parameter			r_header_type = 8'h00 ;
			// REG				bist							NOT implemented !!!

/*-----------------------------------------------------------------------------------------------------------
[010h-03Ch] all other DWORDs (32-bit) of PCI configuration header - only for HEADER type 00h !
			r_ prefix is a sign for read only registers
	BASE_ADDRESS_REGISTERS are the same as ones in the PCI Target configuration registers section. They
	are duplicated and therefor defined just ones and used with the same name as written below. If 
	IMAGEx is NOT defined there is only parameter image_X assigned to '0' and this parameter is used 
	elsewhere in the code. This parameter is defined in the INTERNAL SIGNALS part !!!
	Interrupt_Pin value 8'h01 is used for INT_A pin used.
	MIN_GNT and MAX_LAT are used for device's desired values for Latency Timer value. The value in boath
	registers specifies a period of time in units of 1/4 microsecond. ZERO indicates that there are no
	major requirements for the settings of Latency Timer. 
	MIN_GNT specifieshow how long a burst period the device needs at 33MHz. MAX_LAT specifies how often
	the device needs to gain access to the PCI bus. Values are choosen assuming that the target does not
	insert any wait states. Follow the expamle of settings for simple display card.
	If we use 64 (32-bit) FIFO locations for one burst then we need 8 x 1/4 microsecond periods at 33MHz
	clock rate => MIN_GNT = 08h ! Resolution is 1024 x 768 (= 786432 pixels for one frame) with 16-bit 
	color mode. We can transfere 2 16-bit pixels in one FIFO location. From that we calculate, that for
	one frame we need 6144 burst transferes in 1/25 second. So we need one burst every 6,51 microsecond
	and that is 26 x 1/4 microsecond or 1Ah x 1/4 microsecond => MAX_LAT = 1Ah !
-----------------------------------------------------------------------------------------------------------*/
			// REG x 6		base_address_register_X			IMPLEMENTED as		pci_ba_X !!!
			// REG			r_cardbus_cis_pointer			NOT implemented !!!
			// REG			r_subsystem_vendor_id			NOT implemented !!!
			// REG			r_subsystem_id					NOT implemented !!!
			// REG			r_expansion_rom_base_address	NOT implemented !!!
			// REG			r_cap_list_pointer				NOT implemented !!!
			reg		[7 : 0]	interrupt_line ;
			parameter		r_interrupt_pin = 8'h01 ;
			parameter		r_min_gnt = 8'h08 ;
			parameter		r_max_lat = 8'h1a ;


/*###########################################################################################################
-------------------------------------------------------------------------------------------------------------
PCI Bridge default image SIZE parameters
	This parameters are not part of any register group, but are needed for default image size configuration
	used in PCI Target and WISHBONE Slave configuration registers!
-------------------------------------------------------------------------------------------------------------
###########################################################################################################*/

/*-----------------------------------------------------------------------------------------------------------
	PCI Target default image size parameters are defined with masked bits for address mask registers of 
	each image space. By default there are 1MByte of address space defined for def_pci_imageX_addr_map 
	parameters!
-----------------------------------------------------------------------------------------------------------*/
			parameter		def_pci_image0_addr_map = `PCI_AM0 ; //20'hfff0_0
			parameter		def_pci_image1_addr_map = `PCI_AM1 ; //20'hfff0_0
			parameter		def_pci_image2_addr_map = `PCI_AM2 ; //20'h0000_0
			parameter		def_pci_image3_addr_map = `PCI_AM3 ; //20'h0000_0
			parameter		def_pci_image4_addr_map = `PCI_AM4 ; //20'h0000_0
			parameter		def_pci_image5_addr_map = `PCI_AM5 ; //20'h0000_0

/*-----------------------------------------------------------------------------------------------------------
	WISHBONE Slave default image size parameters are defined with masked bits for address mask registers 
	of each image space. By default there are 1MByte of address space defined for def_wb_imageX_addr_map 
	parameters except for def_wb_image0_addr_map which is used for configuration space!
-----------------------------------------------------------------------------------------------------------*/
			// PARAMETER	def_wb_image0_addr_map	IMPLEMENTED as r_wb_am0 parameter for CONF. space !!!
			parameter		def_wb_image1_addr_map = 20'h0000_0 ;
			parameter		def_wb_image2_addr_map = 20'h0000_0 ;
			parameter		def_wb_image3_addr_map = 20'h0000_0 ;
			parameter		def_wb_image4_addr_map = 20'h0000_0 ;
			parameter		def_wb_image5_addr_map = 20'h0000_0 ;


/*###########################################################################################################
-------------------------------------------------------------------------------------------------------------
PCI Target configuration registers
	There are also some registers with NOT all bits implemented and therefor uses _bitX or _bitX2_X1 to 
	sign which bit or range of bits are implemented. Some special cases and examples are described below!
-------------------------------------------------------------------------------------------------------------
###########################################################################################################*/

/*-----------------------------------------------------------------------------------------------------------
[100h-168h] 
	Depending on defines (PCI_IMAGE1 or .. or PCI_IMAGE5 or (PCI_IMAGE6 and HOST)) in constants.v file, 
	there are registers corresponding to each IMAGE defined to REG and parameter pci_image_X assigned to '1'. 
	The maximum number of images is "6". By default there are first two images used and the first (PCI_IMAGE0) 
	is assigned to Configuration space! With a 'define' PCI_IMAGEx you choose the number of used PCI IMAGES 
	in a bridge without PCI_IMAGE0 (e.g. PCI_IMAGE3 tells, that PCI_IMAGE1, PCI_IMAGE2 and PCI_IMAGE3 are 
	used for mapping the space from WB to PCI. Offcourse, PCI_IMAGE0 is assigned to Configuration space). 
	That leave us PCI_IMAGE5 as the maximum number of images.
	There is one exeption, when the core is implemented as HOST. If so, then the PCI specification allowes 
	the Configuration space NOT to be visible on the PCI bus. With `define PCI_IMAGE6 (and `define HOST), we 
	assign PCI_IMAGE0 to normal WB to PCI image and not to configuration space!                   

	When error occurs, PCI ERR_ADDR and ERR_DATA registers stores address and data on the bus that
	caused error. While ERR_CS register stores Byte Enables and Bus Command in the MSByte. In bits 10
	and 8 it reports Retry Counter Expired (for posted writes), Error Source (Master Abort) and Error
	Report Signal (signals that error has occured) respectively. With bit 0 we enable Error Reporting 
	mechanism.
-----------------------------------------------------------------------------------------------------------*/
			parameter			pci_image0 = 1 ;
			reg		[31 : 12]	pci_ba0_bit31_12 ;
`ifdef		HOST
	`ifdef	PCI_IMAGE6	// if PCI bridge is HOST and IMAGE0 is assigned as general image space
			parameter			pci_image0_conf = 1 ;
			reg		[2 : 1]		pci_img_ctrl0_bit2_1 ;
			reg					pci_ba0_bit0 ;
			reg		[31 : 12]	pci_am0 ;
			reg		[31 : 12]	pci_ta0 ;
	`else // if PCI bridge is HOST and IMAGE0 is assigned to PCI configuration space
			parameter			pci_image0_conf = 0 ;
			wire	[2 : 1]		pci_img_ctrl0_bit2_1 = 2'b00 ; // NO pre-fetch and read line support
			wire				pci_ba0_bit0 = 0 ; // config. space is MEMORY space
			wire	[31 : 12]	pci_am0 = 20'hffff_f ; // 4KBytes of configuration space
			wire	[31 : 12]	pci_ta0 = 20'h0000_0 ; // NO address translation needed
	`endif
`else // if PCI bridge is GUEST, then IMAGE0 is assigned to PCI configuration space
			parameter			pci_image0_conf = 0 ;
			wire	[2 : 1]		pci_img_ctrl0_bit2_1 = 2'b00 ; // NO addr.transl. and pre-fetch
			wire				pci_ba0_bit0 = 0 ; // config. space is MEMORY space
			wire	[31 : 12]	pci_am0 = 20'hffff_f ; // 4KBytes of configuration space
			wire	[31 : 12]	pci_ta0 = 20'h0000_0 ; // NO address translation needed
`endif
			parameter			pci_image1 = 1 ;
			reg		[2 : 1]		pci_img_ctrl1_bit2_1 ;
			reg		[31 : 12]	pci_ba1_bit31_12 ;
			reg					pci_ba1_bit0 ;
			reg		[31 : 12]	pci_am1 ;
			reg		[31 : 12]	pci_ta1 ;

// IMAGE0 and IMAGE1 are included by default, meanwhile other IMAGEs are optional !!!
`ifdef		PCI_IMAGE1              
			parameter			pci_image2 = 0 ;
            wire	[2 : 1]		pci_img_ctrl2_bit2_1 = 2'b00 ;
			wire	[31 : 12]	pci_ba2_bit31_12 = 20'h0000_0 ;    
            wire				pci_ba2_bit0 = 1'b0 ;        
            wire	[31 : 12]	pci_am2 = 20'h0000_0 ;             
            wire	[31 : 12]	pci_ta2 = 20'h0000_0 ;             

 			parameter			pci_image3 = 0 ;
            wire	[2 : 1]		pci_img_ctrl3_bit2_1 = 2'b00 ;
			wire	[31 : 12]	pci_ba3_bit31_12 = 20'h0000_0 ;    
            wire				pci_ba3_bit0 = 1'b0 ;        
            wire	[31 : 12]	pci_am3 = 20'h0000_0 ;             
            wire	[31 : 12]	pci_ta3 = 20'h0000_0 ;             

  			parameter			pci_image4 = 0 ;
            wire	[2 : 1]		pci_img_ctrl4_bit2_1 = 2'b00 ;
			wire	[31 : 12]	pci_ba4_bit31_12 = 20'h0000_0 ;    
            wire				pci_ba4_bit0 = 1'b0 ;        
            wire	[31 : 12]	pci_am4 = 20'h0000_0 ;             
            wire	[31 : 12]	pci_ta4 = 20'h0000_0 ;             

			parameter			pci_image5 = 0 ;
            wire	[2 : 1]		pci_img_ctrl5_bit2_1 = 2'b00 ;
			wire	[31 : 12]	pci_ba5_bit31_12 = 20'h0000_0 ;    
            wire				pci_ba5_bit0 = 1'b0 ;        
            wire	[31 : 12]	pci_am5 = 20'h0000_0 ;             
            wire	[31 : 12]	pci_ta5 = 20'h0000_0 ;             
`endif
`ifdef		PCI_IMAGE2              
			parameter			pci_image2 = 1 ;
			reg		[2 : 1]		pci_img_ctrl2_bit2_1 ;
			reg		[31 : 12]	pci_ba2_bit31_12 ;
			reg					pci_ba2_bit0 ;
			reg		[31 : 12]	pci_am2 ;
			reg		[31 : 12]	pci_ta2 ;

 			parameter			pci_image3 = 0 ;
            wire	[2 : 1]		pci_img_ctrl3_bit2_1 = 2'b00 ;
			wire	[31 : 12]	pci_ba3_bit31_12 = 20'h0000_0 ;    
            wire				pci_ba3_bit0 = 1'b0 ;        
            wire	[31 : 12]	pci_am3 = 20'h0000_0 ;             
            wire	[31 : 12]	pci_ta3 = 20'h0000_0 ;             

  			parameter			pci_image4 = 0 ;
            wire	[2 : 1]		pci_img_ctrl4_bit2_1 = 2'b00 ;
			wire	[31 : 12]	pci_ba4_bit31_12 = 20'h0000_0 ;    
            wire				pci_ba4_bit0 = 1'b0 ;        
            wire	[31 : 12]	pci_am4 = 20'h0000_0 ;             
            wire	[31 : 12]	pci_ta4 = 20'h0000_0 ;             

			parameter			pci_image5 = 0 ;
            wire	[2 : 1]		pci_img_ctrl5_bit2_1 = 2'b00 ;
			wire	[31 : 12]	pci_ba5_bit31_12 = 20'h0000_0 ;    
            wire				pci_ba5_bit0 = 1'b0 ;        
            wire	[31 : 12]	pci_am5 = 20'h0000_0 ;             
            wire	[31 : 12]	pci_ta5 = 20'h0000_0 ;             
`endif
`ifdef		PCI_IMAGE3              
			parameter			pci_image2 = 1 ;
			reg		[2 : 1]		pci_img_ctrl2_bit2_1 ;
			reg		[31 : 12]	pci_ba2_bit31_12 ;
			reg					pci_ba2_bit0 ;
			reg		[31 : 12]	pci_am2 ;
			reg		[31 : 12]	pci_ta2 ;

			parameter			pci_image3 = 1 ;
			reg		[2 : 1]		pci_img_ctrl3_bit2_1 ;
			reg		[31 : 12]	pci_ba3_bit31_12 ;
			reg					pci_ba3_bit0 ;
			reg		[31 : 12]	pci_am3 ;
			reg		[31 : 12]	pci_ta3 ;

  			parameter			pci_image4 = 0 ;
            wire	[2 : 1]		pci_img_ctrl4_bit2_1 = 2'b00 ;
			wire	[31 : 12]	pci_ba4_bit31_12 = 20'h0000_0 ;    
            wire				pci_ba4_bit0 = 1'b0 ;        
            wire	[31 : 12]	pci_am4 = 20'h0000_0 ;             
            wire	[31 : 12]	pci_ta4 = 20'h0000_0 ;             

			parameter			pci_image5 = 0 ;
            wire	[2 : 1]		pci_img_ctrl5_bit2_1 = 2'b00 ;
			wire	[31 : 12]	pci_ba5_bit31_12 = 20'h0000_0 ;    
            wire				pci_ba5_bit0 = 1'b0 ;        
            wire	[31 : 12]	pci_am5 = 20'h0000_0 ;             
            wire	[31 : 12]	pci_ta5 = 20'h0000_0 ;             
`endif
`ifdef		PCI_IMAGE4 
			parameter			pci_image2 = 1 ;
			reg		[2 : 1]		pci_img_ctrl2_bit2_1 ;
			reg		[31 : 12]	pci_ba2_bit31_12 ;
			reg					pci_ba2_bit0 ;
			reg		[31 : 12]	pci_am2 ;
			reg		[31 : 12]	pci_ta2 ;

			parameter			pci_image3 = 1 ;
			reg		[2 : 1]		pci_img_ctrl3_bit2_1 ;
			reg		[31 : 12]	pci_ba3_bit31_12 ;
			reg					pci_ba3_bit0 ;
			reg		[31 : 12]	pci_am3 ;
			reg		[31 : 12]	pci_ta3 ;
            
			parameter			pci_image4 = 1 ;
			reg		[2 : 1]		pci_img_ctrl4_bit2_1 ;
			reg		[31 : 12]	pci_ba4_bit31_12 ;
			reg					pci_ba4_bit0 ;
			reg		[31 : 12]	pci_am4 ;
			reg		[31 : 12]	pci_ta4 ;

			parameter			pci_image5 = 0 ;
            wire	[2 : 1]		pci_img_ctrl5_bit2_1 = 2'b00 ;
			wire	[31 : 12]	pci_ba5_bit31_12 = 20'h0000_0 ;    
            wire				pci_ba5_bit0 = 1'b0 ;        
            wire	[31 : 12]	pci_am5 = 20'h0000_0 ;             
            wire	[31 : 12]	pci_ta5 = 20'h0000_0 ;             
`endif
`ifdef		PCI_IMAGE5              
			parameter			pci_image2 = 1 ;
			reg		[2 : 1]		pci_img_ctrl2_bit2_1 ;
			reg		[31 : 12]	pci_ba2_bit31_12 ;
			reg					pci_ba2_bit0 ;
			reg		[31 : 12]	pci_am2 ;
			reg		[31 : 12]	pci_ta2 ;

			parameter			pci_image3 = 1 ;
			reg		[2 : 1]		pci_img_ctrl3_bit2_1 ;
			reg		[31 : 12]	pci_ba3_bit31_12 ;
			reg					pci_ba3_bit0 ;
			reg		[31 : 12]	pci_am3 ;
			reg		[31 : 12]	pci_ta3 ;
            
			parameter			pci_image4 = 1 ;
			reg		[2 : 1]		pci_img_ctrl4_bit2_1 ;
			reg		[31 : 12]	pci_ba4_bit31_12 ;
			reg					pci_ba4_bit0 ;
			reg		[31 : 12]	pci_am4 ;
			reg		[31 : 12]	pci_ta4 ;

			parameter			pci_image5 = 1 ;
			reg		[2 : 1]		pci_img_ctrl5_bit2_1 ;
			reg		[31 : 12]	pci_ba5_bit31_12 ;
			reg					pci_ba5_bit0 ;
			reg		[31 : 12]	pci_am5 ;
			reg		[31 : 12]	pci_ta5 ;
`endif
`ifdef		PCI_IMAGE6              
			parameter			pci_image2 = 1 ;
			reg		[2 : 1]		pci_img_ctrl2_bit2_1 ;
			reg		[31 : 12]	pci_ba2_bit31_12 ;
			reg					pci_ba2_bit0 ;
			reg		[31 : 12]	pci_am2 ;
			reg		[31 : 12]	pci_ta2 ;

			parameter			pci_image3 = 1 ;
			reg		[2 : 1]		pci_img_ctrl3_bit2_1 ;
			reg		[31 : 12]	pci_ba3_bit31_12 ;
			reg					pci_ba3_bit0 ;
			reg		[31 : 12]	pci_am3 ;
			reg		[31 : 12]	pci_ta3 ;
            
			parameter			pci_image4 = 1 ;
			reg		[2 : 1]		pci_img_ctrl4_bit2_1 ;
			reg		[31 : 12]	pci_ba4_bit31_12 ;
			reg					pci_ba4_bit0 ;
			reg		[31 : 12]	pci_am4 ;
			reg		[31 : 12]	pci_ta4 ;

			parameter			pci_image5 = 1 ;
			reg		[2 : 1]		pci_img_ctrl5_bit2_1 ;
			reg		[31 : 12]	pci_ba5_bit31_12 ;
			reg					pci_ba5_bit0 ;
			reg		[31 : 12]	pci_am5 ;
			reg		[31 : 12]	pci_ta5 ;
`endif
			reg		[31 : 24]	pci_err_cs_bit31_24 ;
			reg					pci_err_cs_bit10 ;
			reg					pci_err_cs_bit8 ;
			reg					pci_err_cs_bit0 ;
			reg		[31 : 0]	pci_err_addr ;
			reg		[31 : 0]	pci_err_data ;
                                
                                
/*###########################################################################################################
-------------------------------------------------------------------------------------------------------------
WISHBONE Slave configuration registers
	There are also some registers with NOT all bits implemented and therefor uses _bitX or _bitX2_X1 to 
	sign which bit or range of bits are implemented. Some special cases and examples are described below!
-------------------------------------------------------------------------------------------------------------
###########################################################################################################*/
                                
/*-----------------------------------------------------------------------------------------------------------
[800h-85Ch]                     
	Depending on defines (WB_IMAGE1 or .. or WB_IMAGE4 or WB_IMAGE5) in constants.v file, there are 
	registers corresponding to each IMAGE defined to REG and parameter wb_image_X assigned to '1'. 
	The maximum number of images is "6". By default there are first two images used and the first (WB_IMAGE0) 
	is assigned to Configuration space! With a 'define' WB_IMAGEx you choose the number of used WB IMAGES in 
	a bridge without WB_IMAGE0 (e.g. WB_IMAGE3 tells, that WB_IMAGE1, WB_IMAGE2 and WB_IMAGE3 are used for 
	mapping the space from PCI to WB. Offcourse, WB_IMAGE0 is assigned to Configuration space). That leave 
	us WB_IMAGE5 as the maximum number of images.                                             
          
	When error occurs, WISHBONE ERR_ADDR and ERR_DATA registers stores address and data on the bus that
	caused error. While ERR_CS register stores Byte Enables and Bus Command in the MSByte. In bits 10, 9
	and 8 it reports Retry Counter Expired (for posted writes), Error Source (Master Abort) and Error
	Report Signal (signals that error has occured) respectively. With bit 0 we enable Error Reporting 
	mechanism.                  
-----------------------------------------------------------------------------------------------------------*/
// WB_IMAGE0 is always assigned to config. space
			parameter			wb_image0 = 1 ; 
			wire	[2 : 0]		wb_img_ctrl0_bit2_0 = 3'b000 ; // NO addr.transl., pre-fetch and read-line 
			wire	[31 : 12]	wb_ba0_bit31_12 = `WB_CONFIGURATION_BASE ;
			wire				wb_ba0_bit0 = 0 ; // config. space is MEMORY space                 
			wire	[31 : 12]	wb_am0 = 20'hffff_f ; // 4KBytes of configuration space            
			wire	[31 : 12]	wb_ta0 = 20'h0000_0 ; // NO address translation needed             
// WB_IMAGE0 and WB_IMAGE1 are included by default meanwhile others are optional !
			parameter			wb_image1 = 1 ;
			reg		[2 : 0]		wb_img_ctrl1_bit2_0 ;
			reg		[31 : 12]	wb_ba1_bit31_12 ;
			reg					wb_ba1_bit0 ;
			reg		[31 : 12]	wb_am1 ;
			reg		[31 : 12]	wb_ta1 ;
`ifdef		WB_IMAGE1          
			parameter			wb_image2 = 0 ; 
            wire	[2 : 0]		wb_img_ctrl2_bit2_0 = 3'b000 ;
			wire	[31 : 12]	wb_ba2_bit31_12 = 20'h0000_0 ;    
            wire				wb_ba2_bit0 = 1'b0 ;        
            wire	[31 : 12]	wb_am2 = 20'h0000_0 ;             
            wire	[31 : 12]	wb_ta2 = 20'h0000_0 ;             

 			parameter			wb_image3 = 0 ;
            wire	[2 : 0]		wb_img_ctrl3_bit2_0 = 3'b000 ;
			wire	[31 : 12]	wb_ba3_bit31_12 = 20'h0000_0 ;    
            wire				wb_ba3_bit0 = 1'b0 ;        
            wire	[31 : 12]	wb_am3 = 20'h0000_0 ;             
            wire	[31 : 12]	wb_ta3 = 20'h0000_0 ;             

  			parameter			wb_image4 = 0 ;
            wire	[2 : 0]		wb_img_ctrl4_bit2_0 = 3'b000 ;
			wire	[31 : 12]	wb_ba4_bit31_12 = 20'h0000_0 ;    
            wire				wb_ba4_bit0 = 1'b0 ;        
            wire	[31 : 12]	wb_am4 = 20'h0000_0 ;             
            wire	[31 : 12]	wb_ta4 = 20'h0000_0 ;             

			parameter			wb_image5 = 0 ;
            wire	[2 : 0]		wb_img_ctrl5_bit2_0 = 3'b000 ;
			wire	[31 : 12]	wb_ba5_bit31_12 = 20'h0000_0 ;    
            wire				wb_ba5_bit0 = 1'b0 ;        
            wire	[31 : 12]	wb_am5 = 20'h0000_0 ;             
            wire	[31 : 12]	wb_ta5 = 20'h0000_0 ;             
`endif                        
`ifdef		WB_IMAGE2           
			parameter			wb_image2 = 1 ; 
			reg		[2 : 0]		wb_img_ctrl2_bit2_0 ;
			reg		[31 : 12]	wb_ba2_bit31_12 ;
			reg					wb_ba2_bit0 ;
			reg		[31 : 12]	wb_am2 ;
			reg		[31 : 12]	wb_ta2 ;

 			parameter			wb_image3 = 0 ;
            wire	[2 : 0]		wb_img_ctrl3_bit2_0 = 3'b000 ;
			wire	[31 : 12]	wb_ba3_bit31_12 = 20'h0000_0 ;    
            wire				wb_ba3_bit0 = 1'b0 ;        
            wire	[31 : 12]	wb_am3 = 20'h0000_0 ;             
            wire	[31 : 12]	wb_ta3 = 20'h0000_0 ;             

  			parameter			wb_image4 = 0 ;
            wire	[2 : 0]		wb_img_ctrl4_bit2_0 = 3'b000 ;
			wire	[31 : 12]	wb_ba4_bit31_12 = 20'h0000_0 ;    
            wire				wb_ba4_bit0 = 1'b0 ;        
            wire	[31 : 12]	wb_am4 = 20'h0000_0 ;             
            wire	[31 : 12]	wb_ta4 = 20'h0000_0 ;             

			parameter			wb_image5 = 0 ;
            wire	[2 : 0]		wb_img_ctrl5_bit2_0 = 3'b000 ;
			wire	[31 : 12]	wb_ba5_bit31_12 = 20'h0000_0 ;    
            wire				wb_ba5_bit0 = 1'b0 ;        
            wire	[31 : 12]	wb_am5 = 20'h0000_0 ;             
            wire	[31 : 12]	wb_ta5 = 20'h0000_0 ;             
`endif                        
`ifdef		WB_IMAGE3           
			parameter			wb_image2 = 1 ; 
			reg		[2 : 0]		wb_img_ctrl2_bit2_0 ;
			reg		[31 : 12]	wb_ba2_bit31_12 ;
			reg					wb_ba2_bit0 ;
			reg		[31 : 12]	wb_am2 ;
			reg		[31 : 12]	wb_ta2 ;

			parameter			wb_image3 = 1 ; 
			reg		[2 : 0]		wb_img_ctrl3_bit2_0 ;
			reg		[31 : 12]	wb_ba3_bit31_12 ;
			reg					wb_ba3_bit0 ;
			reg		[31 : 12]	wb_am3 ;
			reg		[31 : 12]	wb_ta3 ;

  			parameter			wb_image4 = 0 ;
            wire	[2 : 0]		wb_img_ctrl4_bit2_0 = 3'b000 ;
			wire	[31 : 12]	wb_ba4_bit31_12 = 20'h0000_0 ;    
            wire				wb_ba4_bit0 = 1'b0 ;        
            wire	[31 : 12]	wb_am4 = 20'h0000_0 ;             
            wire	[31 : 12]	wb_ta4 = 20'h0000_0 ;             

			parameter			wb_image5 = 0 ;
            wire	[2 : 0]		wb_img_ctrl5_bit2_0 = 3'b000 ;
			wire	[31 : 12]	wb_ba5_bit31_12 = 20'h0000_0 ;    
            wire				wb_ba5_bit0 = 1'b0 ;        
            wire	[31 : 12]	wb_am5 = 20'h0000_0 ;             
            wire	[31 : 12]	wb_ta5 = 20'h0000_0 ;             
`endif                        
`ifdef		WB_IMAGE4           
			parameter			wb_image2 = 1 ; 
			reg		[2 : 0]		wb_img_ctrl2_bit2_0 ;
			reg		[31 : 12]	wb_ba2_bit31_12 ;
			reg					wb_ba2_bit0 ;
			reg		[31 : 12]	wb_am2 ;
			reg		[31 : 12]	wb_ta2 ;

			parameter			wb_image3 = 1 ; 
			reg		[2 : 0]		wb_img_ctrl3_bit2_0 ;
			reg		[31 : 12]	wb_ba3_bit31_12 ;
			reg					wb_ba3_bit0 ;
			reg		[31 : 12]	wb_am3 ;
			reg		[31 : 12]	wb_ta3 ;

			parameter			wb_image4 = 1 ; 
			reg		[2 : 0]		wb_img_ctrl4_bit2_0 ;
			reg		[31 : 12]	wb_ba4_bit31_12 ;
			reg					wb_ba4_bit0 ;
			reg		[31 : 12]	wb_am4 ;
			reg		[31 : 12]	wb_ta4 ;

			parameter			wb_image5 = 0 ;
            wire	[2 : 0]		wb_img_ctrl5_bit2_0 = 3'b000 ;
			wire	[31 : 12]	wb_ba5_bit31_12 = 20'haa00_0 ;    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            wire				wb_ba5_bit0 = 1'b0 ;        
            wire	[31 : 12]	wb_am5 = 20'h0000_0 ;             
            wire	[31 : 12]	wb_ta5 = 20'h0000_0 ;             
`endif                        
`ifdef		WB_IMAGE5           
			parameter			wb_image2 = 1 ; 
			reg		[2 : 0]		wb_img_ctrl2_bit2_0 ;
			reg		[31 : 12]	wb_ba2_bit31_12 ;
			reg					wb_ba2_bit0 ;
			reg		[31 : 12]	wb_am2 ;
			reg		[31 : 12]	wb_ta2 ;

			parameter			wb_image3 = 1 ; 
			reg		[2 : 0]		wb_img_ctrl3_bit2_0 ;
			reg		[31 : 12]	wb_ba3_bit31_12 ;
			reg					wb_ba3_bit0 ;
			reg		[31 : 12]	wb_am3 ;
			reg		[31 : 12]	wb_ta3 ;

			parameter			wb_image4 = 1 ; 
			reg		[2 : 0]		wb_img_ctrl4_bit2_0 ;
			reg		[31 : 12]	wb_ba4_bit31_12 ;
			reg					wb_ba4_bit0 ;
			reg		[31 : 12]	wb_am4 ;
			reg		[31 : 12]	wb_ta4 ;

			parameter			wb_image5 = 1 ; 
			reg		[2 : 0]		wb_img_ctrl5_bit2_0 ;
			reg		[31 : 12]	wb_ba5_bit31_12 ;
			reg					wb_ba5_bit0 ;
			reg		[31 : 12]	wb_am5 ;
			reg		[31 : 12]	wb_ta5 ;
`endif                         
			reg		[31 : 24]	wb_err_cs_bit31_24 ;
			reg		[10 : 8]	wb_err_cs_bit10_8 ;
			reg					wb_err_cs_bit0 ;
			reg		[31 : 0]	wb_err_addr ;
			reg		[31 : 0]	wb_err_data ;
                                
                                
/*###########################################################################################################
-------------------------------------------------------------------------------------------------------------
Configuration Cycle address register
	There are also some registers with NOT all bits implemented and therefor uses _bitX or _bitX2_X1 to 
	sign which bit or range of bits are implemented. 
-------------------------------------------------------------------------------------------------------------
###########################################################################################################*/
                                
/*-----------------------------------------------------------------------------------------------------------
[860h-868h]                     
	PCI bridge must ignore Type 1 configuration cycles (Master Abort) since they are used for PCI to PCI 
	bridges. This is single function device, that means responding on configuration cycles to all functions 
	(or responding only to function 0). Configuration address register for generating configuration cycles 
	is prepared for all options (it includes Bus Number, Device, Function, Offset and Type).
	Interrupt acknowledge register stores interrupt vector data returned during Interrupt Acknowledge cycle.
-----------------------------------------------------------------------------------------------------------*/
			reg		[23 : 2]	cnf_addr_bit23_2 ;
			reg					cnf_addr_bit0 ;
			// reg	[31 : 0]	cnf_data ;		IMPLEMENTED elsewhere !!!!!
			// reg	[31 : 0]	int_ack ;		IMPLEMENTED elsewhere !!!!!
                                
                                
/*###########################################################################################################
-------------------------------------------------------------------------------------------------------------
General Interrupt registers     
	There are also some registers with NOT all bits implemented and therefor uses _bitX or _bitX2_X1 to 
	sign which bit or range of bits are implemented. 
-------------------------------------------------------------------------------------------------------------
###########################################################################################################*/
                                
/*-----------------------------------------------------------------------------------------------------------
[FF8h-FFCh]                     
	Bit 31 in the Interrupt Control register is set by software and used to generate SOFT RESET. Other 4
	bits are used to enable interrupt generations.
	4 LSbits in the Interrupt Status register are indicating System Error Int, Parity Error Int, Error
	Int and Inerrupt respecively.
-----------------------------------------------------------------------------------------------------------*/
			reg					icr_bit31 ;
			reg		[3 : 0]		icr_bit3_0 ;
			reg		[3 : 0]		isr_bit3_0 ;
        
                                
/*###########################################################################################################
-------------------------------------------------------------------------------------------------------------


-----------------------------------------------------------------------------------------------------------*/
                                
always@(r_re or 
		r_conf_address_in or
		status_bit15_11 or status_bit8 or command_bit8 or command_bit6 or command_bit2_0 or
		latency_timer or cache_line_size_reg or 
		pci_ba0_bit31_12 or 
		pci_img_ctrl0_bit2_1 or pci_am0 or pci_ta0 or pci_ba0_bit0 or  
		pci_img_ctrl1_bit2_1 or pci_am1 or pci_ta1 or pci_ba1_bit31_12 or pci_ba1_bit0 or 
		pci_img_ctrl2_bit2_1 or pci_am2 or pci_ta2 or pci_ba2_bit31_12 or pci_ba2_bit0 or 
		pci_img_ctrl3_bit2_1 or pci_am3 or pci_ta3 or pci_ba3_bit31_12 or pci_ba3_bit0 or 
		pci_img_ctrl4_bit2_1 or pci_am4 or pci_ta4 or pci_ba4_bit31_12 or pci_ba4_bit0 or 
		pci_img_ctrl5_bit2_1 or pci_am5 or pci_ta5 or pci_ba5_bit31_12 or pci_ba5_bit0 or 
		interrupt_line or 
		pci_err_cs_bit31_24 or pci_err_cs_bit10 or pci_err_cs_bit8 or pci_err_cs_bit0 or 
		pci_err_addr or pci_err_data or 
		wb_ba0_bit31_12 or wb_ba0_bit0 or 
		wb_img_ctrl1_bit2_0 or wb_ba1_bit31_12 or wb_ba1_bit0 or wb_am1 or wb_ta1 or 
		wb_img_ctrl2_bit2_0 or wb_ba2_bit31_12 or wb_ba2_bit0 or wb_am2 or wb_ta2 or 
		wb_img_ctrl3_bit2_0 or wb_ba3_bit31_12 or wb_ba3_bit0 or wb_am3 or wb_ta3 or 
		wb_img_ctrl4_bit2_0 or wb_ba4_bit31_12 or wb_ba4_bit0 or wb_am4 or wb_ta4 or 
		wb_img_ctrl5_bit2_0 or wb_ba5_bit31_12 or wb_ba5_bit0 or wb_am5 or wb_ta5 or 
		wb_err_cs_bit31_24 or wb_err_cs_bit10_8 or wb_err_cs_bit0 or wb_err_addr or wb_err_data or 
		cnf_addr_bit23_2 or cnf_addr_bit0 or icr_bit31 or icr_bit3_0 or isr_bit3_0
		)     
begin                       
	if (r_re == 1'b1)
	begin    
		case (r_conf_address_in[8])
		1'b0 :
		begin
		  case ({r_conf_address_in[7], r_conf_address_in[6]})
		  2'b00 :
		  begin
			// PCI header - configuration space
			case (r_conf_address_in[5:2])
			4'h0: r_conf_data_out = { r_device_id, r_vendor_id } ;
			4'h1: r_conf_data_out = { status_bit15_11, r_status_bit10_9, status_bit8, 2'h0, r_status_bit5, 
										 5'h00, 7'h00, command_bit8, 1'h0, command_bit6, 3'h0, command_bit2_0 } ;
			4'h2: r_conf_data_out = { r_class_code, r_revision_id } ;
			4'h3: r_conf_data_out = { 8'h00, r_header_type, latency_timer, cache_line_size_reg } ;
			4'h4: r_conf_data_out = { pci_ba0_bit31_12, 11'h000, pci_ba0_bit0 } & { pci_am0[31:12], 12'h001 } ;
			4'h5: r_conf_data_out = { pci_ba1_bit31_12, 11'h000, pci_ba1_bit0 } & { pci_am1[31:12], 12'h001 } ;                            
			4'h6: r_conf_data_out = { pci_ba2_bit31_12, 11'h000, pci_ba2_bit0 } & { pci_am2[31:12], 12'h001 } ;
			4'h7: r_conf_data_out = { pci_ba3_bit31_12, 11'h000, pci_ba3_bit0 } & { pci_am3[31:12], 12'h001 } ;                            
			4'h8: r_conf_data_out = { pci_ba4_bit31_12, 11'h000, pci_ba4_bit0 } & { pci_am4[31:12], 12'h001 } ;
			4'h9: r_conf_data_out = { pci_ba5_bit31_12, 11'h000, pci_ba5_bit0 } & { pci_am5[31:12], 12'h001 } ;
			4'hf: r_conf_data_out = { r_max_lat, r_min_gnt, r_interrupt_pin, interrupt_line } ;
			default	: r_conf_data_out = 32'h0000_0000 ;
			endcase
		  end
		  default :
		    r_conf_data_out = 32'h0000_0000 ;
		  endcase
		end
		default :
		begin
			// PCI target - configuration space
			case (r_conf_address_in[7:2])
			`P_IMG_CTRL0_ADDR: r_conf_data_out = { 29'h00000000, pci_img_ctrl0_bit2_1, 1'h0 } ;
            `P_BA0_ADDR		 : r_conf_data_out = { pci_ba0_bit31_12, 11'h000, pci_ba0_bit0 } & { pci_am0[31:12], 12'h001 } ;
            `P_AM0_ADDR		 : r_conf_data_out = { pci_am0, 12'h000 } ;
            `P_TA0_ADDR		 : r_conf_data_out = { pci_ta0, 12'h000 } ;
            `P_IMG_CTRL1_ADDR: r_conf_data_out = { 29'h00000000, pci_img_ctrl1_bit2_1, 1'h0 } ;
            `P_BA1_ADDR		 : r_conf_data_out = { pci_ba1_bit31_12, 11'h000, pci_ba1_bit0 } & { pci_am1[31:12], 12'h001 } ;
            `P_AM1_ADDR		 : r_conf_data_out = { pci_am1, 12'h000 } ;
            `P_TA1_ADDR		 : r_conf_data_out = { pci_ta1, 12'h000 } ;
            `P_IMG_CTRL2_ADDR: r_conf_data_out = { 29'h00000000, pci_img_ctrl2_bit2_1, 1'h0 } ;
            `P_BA2_ADDR		 : r_conf_data_out = { pci_ba2_bit31_12, 11'h000, pci_ba2_bit0 } & { pci_am2[31:12], 12'h001 } ;
            `P_AM2_ADDR		 : r_conf_data_out = { pci_am2, 12'h000 } ;
            `P_TA2_ADDR		 : r_conf_data_out = { pci_ta2, 12'h000 } ;
            `P_IMG_CTRL3_ADDR: r_conf_data_out = { 29'h00000000, pci_img_ctrl3_bit2_1, 1'h0 } ;
            `P_BA3_ADDR		 : r_conf_data_out = { pci_ba3_bit31_12, 11'h000, pci_ba3_bit0 } & { pci_am3[31:12], 12'h001 } ;
            `P_AM3_ADDR		 : r_conf_data_out = { pci_am3, 12'h000 } ;
            `P_TA3_ADDR		 : r_conf_data_out = { pci_ta3, 12'h000 } ;
            `P_IMG_CTRL4_ADDR: r_conf_data_out = { 29'h00000000, pci_img_ctrl4_bit2_1, 1'h0 } ;
            `P_BA4_ADDR		 : r_conf_data_out = { pci_ba4_bit31_12, 11'h000, pci_ba4_bit0 } & { pci_am4[31:12], 12'h001 } ;
            `P_AM4_ADDR		 : r_conf_data_out = { pci_am4, 12'h000 } ;
            `P_TA4_ADDR		 : r_conf_data_out = { pci_ta4, 12'h000 } ;
            `P_IMG_CTRL5_ADDR: r_conf_data_out = { 29'h00000000, pci_img_ctrl5_bit2_1, 1'h0 } ;
            `P_BA5_ADDR		 : r_conf_data_out = { pci_ba5_bit31_12, 11'h000, pci_ba5_bit0 } & { pci_am5[31:12], 12'h001 } ;
            `P_AM5_ADDR		 : r_conf_data_out = { pci_am5, 12'h000 } ;
            `P_TA5_ADDR		 : r_conf_data_out = { pci_ta5, 12'h000 } ;
            `P_ERR_CS_ADDR	 : r_conf_data_out = { pci_err_cs_bit31_24, 13'h0000, pci_err_cs_bit10, 1'h0, 
            									   pci_err_cs_bit8, 7'h00, pci_err_cs_bit0 } ;
            `P_ERR_ADDR_ADDR : r_conf_data_out = pci_err_addr ;
            `P_ERR_DATA_ADDR : r_conf_data_out = pci_err_data ;
			// WB slave - configuration space
			`WB_CONF_SPC_BAR_ADDR: r_conf_data_out = { wb_ba0_bit31_12, 11'h000, wb_ba0_bit0 } ;
			`W_IMG_CTRL1_ADDR: r_conf_data_out = { 29'h00000000, wb_img_ctrl1_bit2_0 } ;
			`W_BA1_ADDR		 : r_conf_data_out = { wb_ba1_bit31_12, 11'h000, wb_ba1_bit0 } & { wb_am1[31:12], 12'h001 } ;                                        
			`W_AM1_ADDR		 : r_conf_data_out = { wb_am1, 12'h000 } ;                                                               
			`W_TA1_ADDR		 : r_conf_data_out = { wb_ta1, 12'h000 } ;                                                               
			`W_IMG_CTRL2_ADDR: r_conf_data_out = { 29'h00000000, wb_img_ctrl2_bit2_0 } ;
			`W_BA2_ADDR		 : r_conf_data_out = { wb_ba2_bit31_12, 11'h000, wb_ba2_bit0 } & { wb_am2[31:12], 12'h001 } ;                                        
			`W_AM2_ADDR		 : r_conf_data_out = { wb_am2, 12'h000 } ;                                                               
			`W_TA2_ADDR		 : r_conf_data_out = { wb_ta2, 12'h000 } ;                                                               
			`W_IMG_CTRL3_ADDR: r_conf_data_out = { 29'h00000000, wb_img_ctrl3_bit2_0 } ;
			`W_BA3_ADDR		 : r_conf_data_out = { wb_ba3_bit31_12, 11'h000, wb_ba3_bit0 } & { wb_am3[31:12], 12'h001 } ;                                        
			`W_AM3_ADDR		 : r_conf_data_out = { wb_am3, 12'h000 } ;                                                               
			`W_TA3_ADDR		 : r_conf_data_out = { wb_ta3, 12'h000 } ;                                                               
			`W_IMG_CTRL4_ADDR: r_conf_data_out = { 29'h00000000, wb_img_ctrl4_bit2_0 } ;
			`W_BA4_ADDR		 : r_conf_data_out = { wb_ba4_bit31_12, 11'h000, wb_ba4_bit0 } & { wb_am4[31:12], 12'h001 } ;                                        
			`W_AM4_ADDR		 : r_conf_data_out = { wb_am4, 12'h000 } ;                                                               
			`W_TA4_ADDR		 : r_conf_data_out = { wb_ta4, 12'h000 } ;                                                               
			`W_IMG_CTRL5_ADDR: r_conf_data_out = { 29'h00000000, wb_img_ctrl5_bit2_0 } ;
			`W_BA5_ADDR		 : r_conf_data_out = { wb_ba5_bit31_12, 11'h000, wb_ba5_bit0 } & { wb_am5[31:12], 12'h001 } ;
			`W_AM5_ADDR		 : r_conf_data_out = { wb_am5, 12'h000 } ;                       
			`W_TA5_ADDR		 : r_conf_data_out = { wb_ta5, 12'h000 } ;                       
			`W_ERR_CS_ADDR	 : r_conf_data_out = { wb_err_cs_bit31_24, 13'h0000, wb_err_cs_bit10_8, 
            									   7'h00, wb_err_cs_bit0 } ;
			`W_ERR_ADDR_ADDR : r_conf_data_out = wb_err_addr ;
			`W_ERR_DATA_ADDR : r_conf_data_out = wb_err_data ;
			
			`CNF_ADDR_ADDR	 : r_conf_data_out = { 8'h00, cnf_addr_bit23_2, 1'h0, cnf_addr_bit0 } ;
			// `CNF_DATA_ADDR: implemented elsewhere !!! 
			// `INT_ACK_ADDR : implemented elsewhere !!!
            `ICR_ADDR		 : r_conf_data_out = { icr_bit31, 27'h0000_000, icr_bit3_0 } ;
            `ISR_ADDR		 : r_conf_data_out = { 28'h0000_000, isr_bit3_0 } ;
                               
			default	: r_conf_data_out = 32'h0000_0000 ;
			endcase
		end
		endcase
	end
	else
		r_conf_data_out = 32'h0000_0000 ;
end
	                   
always@(w_re or 
		w_conf_address_in or
		status_bit15_11 or status_bit8 or command_bit8 or command_bit6 or command_bit2_0 or
		latency_timer or cache_line_size_reg or 
		pci_ba0_bit31_12 or 
		pci_img_ctrl0_bit2_1 or pci_am0 or pci_ta0 or pci_ba0_bit0 or  
		pci_img_ctrl1_bit2_1 or pci_am1 or pci_ta1 or pci_ba1_bit31_12 or pci_ba1_bit0 or 
		pci_img_ctrl2_bit2_1 or pci_am2 or pci_ta2 or pci_ba2_bit31_12 or pci_ba2_bit0 or 
		pci_img_ctrl3_bit2_1 or pci_am3 or pci_ta3 or pci_ba3_bit31_12 or pci_ba3_bit0 or 
		pci_img_ctrl4_bit2_1 or pci_am4 or pci_ta4 or pci_ba4_bit31_12 or pci_ba4_bit0 or 
		pci_img_ctrl5_bit2_1 or pci_am5 or pci_ta5 or pci_ba5_bit31_12 or pci_ba5_bit0 or 
		interrupt_line or 
		pci_err_cs_bit31_24 or pci_err_cs_bit10 or pci_err_cs_bit8 or pci_err_cs_bit0 or 
		pci_err_addr or pci_err_data or 
		wb_ba0_bit31_12 or wb_ba0_bit0 or 
		wb_img_ctrl1_bit2_0 or wb_ba1_bit31_12 or wb_ba1_bit0 or wb_am1 or wb_ta1 or 
		wb_img_ctrl2_bit2_0 or wb_ba2_bit31_12 or wb_ba2_bit0 or wb_am2 or wb_ta2 or 
		wb_img_ctrl3_bit2_0 or wb_ba3_bit31_12 or wb_ba3_bit0 or wb_am3 or wb_ta3 or 
		wb_img_ctrl4_bit2_0 or wb_ba4_bit31_12 or wb_ba4_bit0 or wb_am4 or wb_ta4 or 
		wb_img_ctrl5_bit2_0 or wb_ba5_bit31_12 or wb_ba5_bit0 or wb_am5 or wb_ta5 or 
		wb_err_cs_bit31_24 or wb_err_cs_bit10_8 or wb_err_cs_bit0 or wb_err_addr or wb_err_data or 
		cnf_addr_bit23_2 or cnf_addr_bit0 or icr_bit31 or icr_bit3_0 or isr_bit3_0
		)  
begin                       
	if (w_re == 1'b1)
	begin
		case (w_conf_address_in[8])
		1'b0 :
		begin
		  case ({w_conf_address_in[7], w_conf_address_in[6]})
		  2'b00 :
		  begin
			// PCI header - configuration space
			case (w_conf_address_in[5:2])
			4'h0: w_conf_data_out = { r_device_id, r_vendor_id } ;
			4'h1: w_conf_data_out = { status_bit15_11, r_status_bit10_9, status_bit8, 2'h0, r_status_bit5, 
									 5'h00, 7'h00, command_bit8, 1'h0, command_bit6, 3'h0, command_bit2_0 } ;
			4'h2: w_conf_data_out = { r_class_code, r_revision_id } ;
			4'h3: w_conf_data_out = { 8'h00, r_header_type, latency_timer, cache_line_size_reg } ;
			4'h4: w_conf_data_out = { pci_ba0_bit31_12, 11'h000, pci_ba0_bit0 } & { pci_am0[31:12], 12'h001 } ;
			4'h5: w_conf_data_out = { pci_ba1_bit31_12, 11'h000, pci_ba1_bit0 } & { pci_am1[31:12], 12'h001 } ;
			4'h6: w_conf_data_out = { pci_ba2_bit31_12, 11'h000, pci_ba2_bit0 } & { pci_am2[31:12], 12'h001 } ;
			4'h7: w_conf_data_out = { pci_ba3_bit31_12, 11'h000, pci_ba3_bit0 } & { pci_am3[31:12], 12'h001 } ;
			4'h8: w_conf_data_out = { pci_ba4_bit31_12, 11'h000, pci_ba4_bit0 } & { pci_am4[31:12], 12'h001 } ;
			4'h9: w_conf_data_out = { pci_ba5_bit31_12, 11'h000, pci_ba5_bit0 } & { pci_am5[31:12], 12'h001 } ;
			4'hf: w_conf_data_out = { r_max_lat, r_min_gnt, r_interrupt_pin, interrupt_line } ;
			default	: w_conf_data_out = 32'h0000_0000 ;
			endcase
		  end
		  default :
		    w_conf_data_out = 32'h0000_0000 ;
		  endcase
		end
		default :
		begin
			// PCI target - configuration space
			case (w_conf_address_in[7:2])
			`P_IMG_CTRL0_ADDR: w_conf_data_out = { 29'h00000000, pci_img_ctrl0_bit2_1, 1'h0 } ;
            `P_BA0_ADDR		 : w_conf_data_out = { pci_ba0_bit31_12, 11'h000, pci_ba0_bit0 } & { pci_am0[31:12], 12'h001 } ;
            `P_AM0_ADDR		 : w_conf_data_out = { pci_am0, 12'h000 } ;
            `P_TA0_ADDR		 : w_conf_data_out = { pci_ta0, 12'h000 } ;
            `P_IMG_CTRL1_ADDR: w_conf_data_out = { 29'h00000000, pci_img_ctrl1_bit2_1, 1'h0 } ;
            `P_BA1_ADDR		 : w_conf_data_out = { pci_ba1_bit31_12, 11'h000, pci_ba1_bit0 } & { pci_am1[31:12], 12'h001 } ;
            `P_AM1_ADDR		 : w_conf_data_out = { pci_am1, 12'h000 } ;
            `P_TA1_ADDR		 : w_conf_data_out = { pci_ta1, 12'h000 } ;
            `P_IMG_CTRL2_ADDR: w_conf_data_out = { 29'h00000000, pci_img_ctrl2_bit2_1, 1'h0 } ;
            `P_BA2_ADDR		 : w_conf_data_out = { pci_ba2_bit31_12, 11'h000, pci_ba2_bit0 } & { pci_am2[31:12], 12'h001 } ;
            `P_AM2_ADDR		 : w_conf_data_out = { pci_am2, 12'h000 } ;
            `P_TA2_ADDR		 : w_conf_data_out = { pci_ta2, 12'h000 } ;
            `P_IMG_CTRL3_ADDR: w_conf_data_out = { 29'h00000000, pci_img_ctrl3_bit2_1, 1'h0 } ;
            `P_BA3_ADDR		 : w_conf_data_out = { pci_ba3_bit31_12, 11'h000, pci_ba3_bit0 } & { pci_am3[31:12], 12'h001 } ;
            `P_AM3_ADDR		 : w_conf_data_out = { pci_am3, 12'h000 } ;
            `P_TA3_ADDR		 : w_conf_data_out = { pci_ta3, 12'h000 } ;
            `P_IMG_CTRL4_ADDR: w_conf_data_out = { 29'h00000000, pci_img_ctrl4_bit2_1, 1'h0 } ;
            `P_BA4_ADDR		 : w_conf_data_out = { pci_ba4_bit31_12, 11'h000, pci_ba4_bit0 } & { pci_am4[31:12], 12'h001 } ;
            `P_AM4_ADDR		 : w_conf_data_out = { pci_am4, 12'h000 } ;
            `P_TA4_ADDR		 : w_conf_data_out = { pci_ta4, 12'h000 } ;
            `P_IMG_CTRL5_ADDR: w_conf_data_out = { 29'h00000000, pci_img_ctrl5_bit2_1, 1'h0 } ;
            `P_BA5_ADDR		 : w_conf_data_out = { pci_ba5_bit31_12, 11'h000, pci_ba5_bit0 } & { pci_am5[31:12], 12'h001 } ;
            `P_AM5_ADDR		 : w_conf_data_out = { pci_am5, 12'h000 } ;
            `P_TA5_ADDR		 : w_conf_data_out = { pci_ta5, 12'h000 } ;
            `P_ERR_CS_ADDR	 : w_conf_data_out = { pci_err_cs_bit31_24, 13'h0000, pci_err_cs_bit10, 1'h0, 
            									   pci_err_cs_bit8, 7'h00, pci_err_cs_bit0 } ;
            `P_ERR_ADDR_ADDR : w_conf_data_out = pci_err_addr ;
            `P_ERR_DATA_ADDR : w_conf_data_out = pci_err_data ;
			// WB slave - configuration space
			`WB_CONF_SPC_BAR_ADDR: w_conf_data_out = { wb_ba0_bit31_12, 11'h000, wb_ba0_bit0 } ;
			`W_IMG_CTRL1_ADDR: w_conf_data_out = { 29'h00000000, wb_img_ctrl1_bit2_0 } ;
			`W_BA1_ADDR		 : w_conf_data_out = { wb_ba1_bit31_12, 11'h000, wb_ba1_bit0 } & { wb_am1[31:12], 12'h001 } ;                                        
			`W_AM1_ADDR		 : w_conf_data_out = { wb_am1, 12'h000 } ;                                                               
			`W_TA1_ADDR		 : w_conf_data_out = { wb_ta1, 12'h000 } ;                                                               
			`W_IMG_CTRL2_ADDR: w_conf_data_out = { 29'h00000000, wb_img_ctrl2_bit2_0 } ;
			`W_BA2_ADDR		 : w_conf_data_out = { wb_ba2_bit31_12, 11'h000, wb_ba2_bit0 } & { wb_am2[31:12], 12'h001 } ;                                        
			`W_AM2_ADDR		 : w_conf_data_out = { wb_am2, 12'h000 } ;                                                               
			`W_TA2_ADDR		 : w_conf_data_out = { wb_ta2, 12'h000 } ;                                                               
			`W_IMG_CTRL3_ADDR: w_conf_data_out = { 29'h00000000, wb_img_ctrl3_bit2_0 } ;
			`W_BA3_ADDR		 : w_conf_data_out = { wb_ba3_bit31_12, 11'h000, wb_ba3_bit0 } & { wb_am3[31:12], 12'h001 } ;                                        
			`W_AM3_ADDR		 : w_conf_data_out = { wb_am3, 12'h000 } ;                                                               
			`W_TA3_ADDR		 : w_conf_data_out = { wb_ta3, 12'h000 } ;                                                               
			`W_IMG_CTRL4_ADDR: w_conf_data_out = { 29'h00000000, wb_img_ctrl4_bit2_0 } ;
			`W_BA4_ADDR		 : w_conf_data_out = { wb_ba4_bit31_12, 11'h000, wb_ba4_bit0 } & { wb_am4[31:12], 12'h001 } ;                                        
			`W_AM4_ADDR		 : w_conf_data_out = { wb_am4, 12'h000 } ;                                                               
			`W_TA4_ADDR		 : w_conf_data_out = { wb_ta4, 12'h000 } ;                                                               
			`W_IMG_CTRL5_ADDR: w_conf_data_out = { 29'h00000000, wb_img_ctrl5_bit2_0 } ;
			`W_BA5_ADDR		 : w_conf_data_out = { wb_ba5_bit31_12, 11'h000, wb_ba5_bit0 } & { wb_am5[31:12], 12'h001 } ;
			`W_AM5_ADDR		 : w_conf_data_out = { wb_am5, 12'h000 } ;                       
			`W_TA5_ADDR		 : w_conf_data_out = { wb_ta5, 12'h000 } ;                       
			`W_ERR_CS_ADDR	 : w_conf_data_out = { wb_err_cs_bit31_24, 13'h0000, wb_err_cs_bit10_8, 
            									   7'h00, wb_err_cs_bit0 } ;
			`W_ERR_ADDR_ADDR : w_conf_data_out = wb_err_addr ;
			`W_ERR_DATA_ADDR : w_conf_data_out = wb_err_data ;
			
			`CNF_ADDR_ADDR	 : w_conf_data_out = { 8'h00, cnf_addr_bit23_2, 1'h0, cnf_addr_bit0 } ;
			// `CNF_DATA_ADDR: implemented elsewhere !!! 
			// `INT_ACK_ADDR : implemented elsewhere !!!
            `ICR_ADDR		 : w_conf_data_out = { icr_bit31, 27'h0000_000, icr_bit3_0 } ;
            `ISR_ADDR		 : w_conf_data_out = { 28'h0000_000, isr_bit3_0 } ;
			default	: w_conf_data_out = 32'h0000_0000 ;
			endcase
		end
		endcase
	end 
	else
		w_conf_data_out = 32'h0000_0000 ;
end

always@(posedge w_clock or posedge reset)     
begin         
	// Here are implemented all registers that are reset with RESET signal otherwise they can be normaly written!!!
	// Registers that are commented are implemented after this alwasy statement, because they are e.g. reset with
	//   RESET signal, set with some status signal and they are erased with writting '1' into them !!!
	if (reset)
	begin
		/*status_bit15_11 ; status_bit8 ;*/ command_bit8 <= 1'h0 ; command_bit6 <= 1'h0 ; command_bit2_0 <= 3'h0 ;
		latency_timer <= 8'h00 ; cache_line_size_reg <= 8'h00 ;
		// ALL pci_base address registers are the same as pci_baX registers !
		interrupt_line <= 8'h00 ;

		`ifdef	HOST
		 `ifdef	PCI_IMAGE6	// if PCI bridge is HOST and IMAGE0 is assigned as general image space
					pci_img_ctrl0_bit2_1 <= 2'h0 ;
		//			pci_ba0_bit31_12 is always a register (never parameter) !!! 
					pci_ba0_bit0 <= 1'h0 ; 
					pci_am0 <= def_pci_image0_addr_map ;
					pci_ta0 <= 20'h0000_0 ;
		 `endif
		`endif
		pci_ba0_bit31_12 <= 20'h0000_0 ; 
		
		pci_img_ctrl1_bit2_1 <= 2'h0 ;
		pci_ba1_bit31_12 <= 20'h0000_0 ; pci_ba1_bit0 <= 1'h0 ;
		pci_am1 <= def_pci_image1_addr_map ;
		pci_ta1 <= 20'h0000_0 ;
		`ifdef	PCI_IMAGE2
        			pci_img_ctrl2_bit2_1 <= 2'h0 ;
					pci_ba2_bit31_12 <= 20'h0000_0 ; pci_ba2_bit0 <= 1'h0 ;
					pci_am2 <= def_pci_image2_addr_map ;
					pci_ta2 <= 20'h0000_0 ;
		`endif
		`ifdef	PCI_IMAGE3
        			pci_img_ctrl2_bit2_1 <= 2'h0 ;
					pci_ba2_bit31_12 <= 20'h0000_0 ; pci_ba2_bit0 <= 1'h0 ;
					pci_am2 <= def_pci_image2_addr_map ;
					pci_ta2 <= 20'h0000_0 ;
					pci_img_ctrl3_bit2_1 <= 2'h0 ;
        			pci_ba3_bit31_12 <= 20'h0000_0 ; pci_ba3_bit0 <= 1'h0 ;
        			pci_am3 <= def_pci_image3_addr_map ;
					pci_ta3 <= 20'h0000_0 ;
		`endif
		`ifdef	PCI_IMAGE4
        			pci_img_ctrl2_bit2_1 <= 2'h0 ;
					pci_ba2_bit31_12 <= 20'h0000_0 ; pci_ba2_bit0 <= 1'h0 ;
					pci_am2 <= def_pci_image2_addr_map ;
					pci_ta2 <= 20'h0000_0 ;
					pci_img_ctrl3_bit2_1 <= 2'h0 ;
        			pci_ba3_bit31_12 <= 20'h0000_0 ; pci_ba3_bit0 <= 1'h0 ;
        			pci_am3 <= def_pci_image3_addr_map ;
					pci_ta3 <= 20'h0000_0 ;
					pci_img_ctrl4_bit2_1 <= 2'h0 ;
					pci_ba4_bit31_12 <= 20'h0000_0 ; pci_ba4_bit0 <= 1'h0 ;
					pci_am4 <= def_pci_image4_addr_map ;
					pci_ta4 <= 20'h0000_0 ;
		`endif
		`ifdef	PCI_IMAGE5
        			pci_img_ctrl2_bit2_1 <= 2'h0 ;
					pci_ba2_bit31_12 <= 20'h0000_0 ; pci_ba2_bit0 <= 1'h0 ;
					pci_am2 <= def_pci_image2_addr_map ;
					pci_ta2 <= 20'h0000_0 ;
					pci_img_ctrl3_bit2_1 <= 2'h0 ;
        			pci_ba3_bit31_12 <= 20'h0000_0 ; pci_ba3_bit0 <= 1'h0 ;
        			pci_am3 <= def_pci_image3_addr_map ;
					pci_ta3 <= 20'h0000_0 ;
					pci_img_ctrl4_bit2_1 <= 2'h0 ;
					pci_ba4_bit31_12 <= 20'h0000_0 ; pci_ba4_bit0 <= 1'h0 ;
					pci_am4 <= def_pci_image4_addr_map ;
					pci_ta4 <= 20'h0000_0 ;
					pci_img_ctrl5_bit2_1 <= 2'h0 ;
					pci_ba5_bit31_12 <= 20'h0000_0 ; pci_ba5_bit0 <= 1'h0 ;
					pci_am5 <= def_pci_image5_addr_map ;
					pci_ta5 <= 20'h0000_0 ;
		`endif
		`ifdef	PCI_IMAGE6
        			pci_img_ctrl2_bit2_1 <= 2'h0 ;
					pci_ba2_bit31_12 <= 20'h0000_0 ; pci_ba2_bit0 <= 1'h0 ;
					pci_am2 <= def_pci_image2_addr_map ;
					pci_ta2 <= 20'h0000_0 ;
					pci_img_ctrl3_bit2_1 <= 2'h0 ;
        			pci_ba3_bit31_12 <= 20'h0000_0 ; pci_ba3_bit0 <= 1'h0 ;
        			pci_am3 <= def_pci_image3_addr_map ;
					pci_ta3 <= 20'h0000_0 ;
					pci_img_ctrl4_bit2_1 <= 2'h0 ;
					pci_ba4_bit31_12 <= 20'h0000_0 ; pci_ba4_bit0 <= 1'h0 ;
					pci_am4 <= def_pci_image4_addr_map ;
					pci_ta4 <= 20'h0000_0 ;
					pci_img_ctrl5_bit2_1 <= 2'h0 ;
					pci_ba5_bit31_12 <= 20'h0000_0 ; pci_ba5_bit0 <= 1'h0 ;
					pci_am5 <= def_pci_image5_addr_map ;
					pci_ta5 <= 20'h0000_0 ;
		`endif
		/*pci_err_cs_bit31_24 ; pci_err_cs_bit10 ; pci_err_cs_bit8 ;*/ pci_err_cs_bit0 <= 1'h0 ;
		/*pci_err_addr ;*/
        /*pci_err_data ;*/
		// 
		wb_img_ctrl1_bit2_0 <= 3'h0 ;
		wb_ba1_bit31_12 <= 20'h0000_0 ; wb_ba1_bit0 <= 1'h0 ;
		wb_am1 <= def_wb_image1_addr_map ;
		wb_ta1 <= 20'h0000_0 ;
        `ifdef	WB_IMAGE2
					wb_img_ctrl2_bit2_0 <= 3'h0 ;
					wb_ba2_bit31_12 <= 20'h0000_0 ; wb_ba2_bit0 <= 1'h0 ;
					wb_am2 <= def_wb_image2_addr_map ;
					wb_ta2 <= 20'h0000_0 ;
		`endif
		`ifdef	WB_IMAGE3
					wb_img_ctrl2_bit2_0 <= 3'h0 ;
					wb_ba2_bit31_12 <= 20'h0000_0 ; wb_ba2_bit0 <= 1'h0 ;
					wb_am2 <= def_wb_image2_addr_map ;
					wb_ta2 <= 20'h0000_0 ;
					wb_img_ctrl3_bit2_0 <= 3'h0 ;
					wb_ba3_bit31_12 <= 20'h0000_0 ; wb_ba3_bit0 <= 1'h0 ;
					wb_am3 <= def_wb_image3_addr_map ;
					wb_ta3 <= 20'h0000_0 ;
		`endif
		`ifdef	WB_IMAGE4
					wb_img_ctrl2_bit2_0 <= 3'h0 ;
					wb_ba2_bit31_12 <= 20'h0000_0 ; wb_ba2_bit0 <= 1'h0 ;
					wb_am2 <= def_wb_image2_addr_map ;
					wb_ta2 <= 20'h0000_0 ;
					wb_img_ctrl3_bit2_0 <= 3'h0 ;
					wb_ba3_bit31_12 <= 20'h0000_0 ; wb_ba3_bit0 <= 1'h0 ;
					wb_am3 <= def_wb_image3_addr_map ;
					wb_ta3 <= 20'h0000_0 ;
					wb_img_ctrl4_bit2_0 <= 3'h0 ;
					wb_ba4_bit31_12 <= 20'h0000_0 ; wb_ba4_bit0 <= 1'h0 ;
					wb_am4 <= def_wb_image4_addr_map ;
					wb_ta4 <= 20'h0000_0 ;
		`endif
		`ifdef	WB_IMAGE5
					wb_img_ctrl2_bit2_0 <= 3'h0 ;
					wb_ba2_bit31_12 <= 20'h0000_0 ; wb_ba2_bit0 <= 1'h0 ;
					wb_am2 <= def_wb_image2_addr_map ;
					wb_ta2 <= 20'h0000_0 ;
					wb_img_ctrl3_bit2_0 <= 3'h0 ;
					wb_ba3_bit31_12 <= 20'h0000_0 ; wb_ba3_bit0 <= 1'h0 ;
					wb_am3 <= def_wb_image3_addr_map ;
					wb_ta3 <= 20'h0000_0 ;
					wb_img_ctrl4_bit2_0 <= 3'h0 ;
					wb_ba4_bit31_12 <= 20'h0000_0 ; wb_ba4_bit0 <= 1'h0 ;
					wb_am4 <= def_wb_image4_addr_map ;
					wb_ta4 <= 20'h0000_0 ;
					wb_img_ctrl5_bit2_0 <= 3'h0 ;
        			wb_ba5_bit31_12 <= 20'h0000_0 ; wb_ba5_bit0 <= 1'h0 ;
					wb_am5 <= def_wb_image5_addr_map ;
					wb_ta5 <= 20'h0000_0 ;
		`endif
		/*wb_err_cs_bit31_24 ; wb_err_cs_bit10_8 ;*/ wb_err_cs_bit0 <= 1'h0 ;
		/*wb_err_addr ;*/
		/*wb_err_data ;*/
        
        cnf_addr_bit23_2 <= 22'h0000_00 ; cnf_addr_bit0 <= 1'h0 ;
		icr_bit31 <= 1'h0 ; icr_bit3_0 <= 4'h0 ;
		/*isr_bit3_0 ;*/
	end
/* -----------------------------------------------------------------------------------------------------------
Following register bits should have asynchronous RESET & SET! That is why they are IMPLEMENTED separately
after this ALWAYS block!!! (for every register bit, there are two D-FF implemented)
		status_bit15_11[15] <= 1'b1 ;
		status_bit15_11[14] <= 1'b1 ;
		status_bit15_11[13] <= 1'b1 ;
		status_bit15_11[12] <= 1'b1 ;
		status_bit15_11[11] <= 1'b1 ;
		status_bit8 <= 1'b1 ;
		pci_err_cs_bit10 <= 1'b1 ;
		pci_err_cs_bit8 <= 1'b1 ;
		pci_err_cs_bit31_24 <= { pci_error_be, pci_error_bc } ;
		pci_err_addr <= pci_error_addr ;
		pci_err_data <= pci_error_data ;
		wb_err_cs_bit10_8[10] <= 1'b1 ;
		wb_err_cs_bit10_8[9] <= 1'b1 ;
		wb_err_cs_bit10_8[8] <= 1'b1 ;
		wb_err_cs_bit31_24 <= { wb_error_be, wb_error_bc } ;
		wb_err_addr <= wb_error_addr ;
		wb_err_data <= wb_error_data ;
		isr_bit3_0[3] <= 1'b1 & icr_bit3_0[3] ;
		isr_bit3_0[2] <= 1'b1 & icr_bit3_0[2] ;
		isr_bit3_0[1] <= 1'b1 & icr_bit3_0[1] ;
		isr_bit3_0[0] <= 1'b1 & icr_bit3_0[0] ;
-----------------------------------------------------------------------------------------------------------*/
	// Here follows normal writting to registers (only to their valid bits) !
	else
	begin
		if (w_we == 1'b1)
		begin               
			case (w_conf_address_in[8])
			1'b0 :
			begin
			  case ({w_conf_address_in[7], w_conf_address_in[6]})
			  2'b00 :
			  begin
				// PCI header - configuration space
				case (w_conf_address_in[5:2])
				4'h1: 
				begin
					if (w_byte_en[1] == 1'b0)
						command_bit8 <= #`FF_DELAY w_conf_data_in[8] ;
					if (w_byte_en[0] == 1'b0)
					begin
						command_bit6 <= #`FF_DELAY w_conf_data_in[6] ;
						command_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
					end
				end
				4'h3: 
				begin
					if (w_byte_en[1] == 1'b0)
						latency_timer <= #`FF_DELAY w_conf_data_in[15:8] ;
					if (w_byte_en[0] == 1'b0)
						cache_line_size_reg <= #`FF_DELAY w_conf_data_in[7:0] ;
				end
	            4'h4: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba0_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba0_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba0_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            4'h5: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba1_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba1_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba1_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba1_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
`ifdef		PCI_IMAGE2
	            4'h6: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
`endif
`ifdef		PCI_IMAGE3
	            4'h6: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            4'h7: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba3_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba3_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba3_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba3_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
`endif
`ifdef		PCI_IMAGE4
	            4'h6: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            4'h7: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba3_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba3_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba3_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba3_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            4'h8: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba4_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba4_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba4_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba4_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
`endif
`ifdef		PCI_IMAGE5
	            4'h6: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            4'h7: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba3_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba3_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba3_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba3_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            4'h8: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba4_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba4_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba4_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba4_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            4'h9: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba5_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba5_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba5_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba5_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
`endif
`ifdef		PCI_IMAGE6
	            4'h6: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            4'h7: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba3_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba3_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba3_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba3_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            4'h8: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba4_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba4_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba4_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba4_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            4'h9: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba5_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba5_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba5_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba5_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
`endif
				4'hf: 
				begin
					if (w_byte_en[0] == 1'b0)
						interrupt_line <= #`FF_DELAY w_conf_data_in[7:0] ;
				end
				default :
				begin
				end
				endcase
			  end
			  default :
			  begin
			  end
			  endcase
			end
			default :
			begin
				// PCI target - configuration space
				case (w_conf_address_in[7:2])
`ifdef		HOST
 `ifdef		PCI_IMAGE6	// if PCI bridge is HOST and IMAGE0 is assigned as general image space
				`P_IMG_CTRL0_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl0_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
 `endif
`endif
			// pci_ba0_bit31_12 & pci_ba0_bit0 are always registers (never parameters) !!! 
	            `P_BA0_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba0_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba0_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba0_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
`ifdef		HOST
 `ifdef		PCI_IMAGE6	// if PCI bridge is HOST and IMAGE0 is assigned as general image space
					if (w_byte_en[0] == 1'b0)
						pci_ba0_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
 `endif
`endif
				end
`ifdef		HOST
 `ifdef		PCI_IMAGE6	// if PCI bridge is HOST and IMAGE0 is assigned as general image space
	            `P_AM0_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am0[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am0[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am0[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA0_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta0[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta0[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta0[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
 `endif
`endif
	            `P_IMG_CTRL1_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl1_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA1_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba1_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba1_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba1_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba1_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM1_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am1[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am1[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am1[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA1_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta1[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta1[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta1[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
`ifdef		PCI_IMAGE2
	            `P_IMG_CTRL2_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl2_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
`endif
`ifdef		PCI_IMAGE3
	            `P_IMG_CTRL2_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl2_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_IMG_CTRL3_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl3_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba3_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba3_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba3_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba3_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
`endif
`ifdef		PCI_IMAGE4
	            `P_IMG_CTRL2_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl2_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_IMG_CTRL3_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl3_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba3_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba3_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba3_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba3_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_IMG_CTRL4_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl4_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba4_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba4_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba4_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba4_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am4[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am4[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am4[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta4[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta4[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta4[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
`endif
`ifdef		PCI_IMAGE5
	            `P_IMG_CTRL2_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl2_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_IMG_CTRL3_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl3_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba3_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba3_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba3_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba3_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_IMG_CTRL4_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl4_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba4_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba4_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba4_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba4_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am4[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am4[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am4[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta4[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta4[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta4[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_IMG_CTRL5_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl5_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA5_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba5_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba5_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba5_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba5_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM5_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am5[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am5[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am5[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA5_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta5[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta5[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta5[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
`endif
`ifdef		PCI_IMAGE6
	            `P_IMG_CTRL2_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl2_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_IMG_CTRL3_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl3_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba3_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba3_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba3_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba3_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_IMG_CTRL4_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl4_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba4_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba4_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba4_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba4_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am4[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am4[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am4[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta4[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta4[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta4[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_IMG_CTRL5_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_img_ctrl5_bit2_1 <= #`FF_DELAY w_conf_data_in[2:1] ;
				end
	            `P_BA5_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ba5_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ba5_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ba5_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						pci_ba5_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
	            `P_AM5_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_am5[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_am5[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_am5[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
	            `P_TA5_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						pci_ta5[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						pci_ta5[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						pci_ta5[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
`endif
	            `P_ERR_CS_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						pci_err_cs_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
			// WB slave - configuration space
				`W_IMG_CTRL1_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_img_ctrl1_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
				end
				`W_BA1_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ba1_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ba1_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ba1_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						wb_ba1_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
				`W_AM1_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_am1[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_am1[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_am1[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_TA1_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ta1[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ta1[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ta1[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
`ifdef		WB_IMAGE2
				`W_IMG_CTRL2_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_img_ctrl2_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
				end
				`W_BA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						wb_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
				`W_AM2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_am2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_am2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_am2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_TA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ta2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ta2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ta2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
`endif
`ifdef		WB_IMAGE3
				`W_IMG_CTRL2_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_img_ctrl2_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
				end
				`W_BA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						wb_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
				`W_AM2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_am2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_am2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_am2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_TA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ta2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ta2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ta2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_IMG_CTRL3_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_img_ctrl3_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
				end
				`W_BA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ba3_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ba3_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ba3_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						wb_ba3_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
				`W_AM3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_am3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_am3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_am3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_TA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ta3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ta3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ta3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
`endif
`ifdef		WB_IMAGE4
				`W_IMG_CTRL2_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_img_ctrl2_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
				end
				`W_BA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						wb_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
				`W_AM2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_am2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_am2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_am2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_TA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ta2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ta2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ta2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_IMG_CTRL3_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_img_ctrl3_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
				end
				`W_BA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ba3_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ba3_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ba3_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						wb_ba3_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
				`W_AM3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_am3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_am3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_am3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_TA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ta3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ta3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ta3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_IMG_CTRL4_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_img_ctrl4_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
				end
				`W_BA4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ba4_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ba4_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ba4_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						wb_ba4_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
				`W_AM4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_am4[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_am4[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_am4[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_TA4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ta4[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ta4[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ta4[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
`endif
`ifdef		WB_IMAGE5
				`W_IMG_CTRL2_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_img_ctrl2_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
				end
				`W_BA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ba2_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ba2_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ba2_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						wb_ba2_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
				`W_AM2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_am2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_am2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_am2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_TA2_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ta2[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ta2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ta2[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_IMG_CTRL3_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_img_ctrl3_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
				end
				`W_BA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ba3_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ba3_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ba3_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						wb_ba3_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
				`W_AM3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_am3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_am3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_am3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_TA3_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ta3[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ta3[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ta3[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_IMG_CTRL4_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_img_ctrl4_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
				end
				`W_BA4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ba4_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ba4_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ba4_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						wb_ba4_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
				`W_AM4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_am4[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_am4[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_am4[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_TA4_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ta4[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ta4[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ta4[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_IMG_CTRL5_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_img_ctrl5_bit2_0 <= #`FF_DELAY w_conf_data_in[2:0] ;
				end
				`W_BA5_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ba5_bit31_12[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ba5_bit31_12[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ba5_bit31_12[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
					if (w_byte_en[0] == 1'b0)
						wb_ba5_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end
				`W_AM5_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_am5[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_am5[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_am5[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
				`W_TA5_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						wb_ta5[31:24] <= #`FF_DELAY w_conf_data_in[31:24] ;
					if (w_byte_en[2] == 1'b0)
						wb_ta5[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						wb_ta5[15:12] <= #`FF_DELAY w_conf_data_in[15:12] ;
				end
`endif
				`W_ERR_CS_ADDR: 
				begin
					if (w_byte_en[0] == 1'b0)
						wb_err_cs_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
				end

				`CNF_ADDR_ADDR: 
				begin
					if (w_byte_en[2] == 1'b0)
						cnf_addr_bit23_2[23:16] <= #`FF_DELAY w_conf_data_in[23:16] ;
					if (w_byte_en[1] == 1'b0)
						cnf_addr_bit23_2[15:8] <= #`FF_DELAY w_conf_data_in[15:8] ;
					if (w_byte_en[0] == 1'b0)
					begin
						cnf_addr_bit23_2[7:2] <= #`FF_DELAY w_conf_data_in[7:2] ;
						cnf_addr_bit0 <= #`FF_DELAY w_conf_data_in[0] ;
					end
				end
				// `CNF_DATA_ADDR: implemented elsewhere !!! 
				// `INT_ACK_ADDR : implemented elsewhere !!!
	            `ICR_ADDR: 
				begin
					if (w_byte_en[3] == 1'b0)
						icr_bit31 <= #`FF_DELAY w_conf_data_in[31] ;
					if (w_byte_en[0] == 1'b0)
						icr_bit3_0 <= #`FF_DELAY w_conf_data_in[3:0] ;
				end
				default	: 
				begin
				end
				endcase
			end
			endcase
		end
	end 
end

// This signals are synchronous resets for registers, whic occures when asynchronous RESET is '1' or
// data '1' is synchronously written into them! 
reg			delete_status_bit15 ;
reg			delete_status_bit14 ;
reg			delete_status_bit13 ;
reg			delete_status_bit12 ;
reg			delete_status_bit11 ;
reg			delete_status_bit8 ;
reg			delete_pci_err_cs_bit10 ;
reg			delete_pci_err_cs_bit8 ; 
reg			delete_wb_err_cs_bit10 ;
reg			delete_wb_err_cs_bit8 ;
reg			delete_isr_bit3 ;
reg			delete_isr_bit2 ;
reg			delete_isr_bit1 ;
reg			delete_isr_bit0 ;   

// If nothing is written into, then the value is '0' (W_WE = 0) !!! 
wire		delete_status_bit15_in		= w_we ? w_conf_data_in[31] : 1'b0 ;
wire		delete_status_bit14_in		= w_we ? w_conf_data_in[30] : 1'b0 ;
wire		delete_status_bit13_in		= w_we ? w_conf_data_in[29] : 1'b0 ;
wire		delete_status_bit12_in		= w_we ? w_conf_data_in[28] : 1'b0 ;
wire		delete_status_bit11_in		= w_we ? w_conf_data_in[27] : 1'b0 ;
wire		delete_status_bit8_in   	= w_we ? w_conf_data_in[24] : 1'b0 ;
wire		delete_pci_err_cs_bit10_in	= w_we ? w_conf_data_in[10] : 1'b0 ;
wire		delete_pci_err_cs_bit8_in	= w_we ? w_conf_data_in[8]  : 1'b0 ; 
wire		delete_wb_err_cs_bit10_in	= w_we ? w_conf_data_in[10] : 1'b0 ;
wire		delete_wb_err_cs_bit8_in	= w_we ? w_conf_data_in[8]  : 1'b0 ;
wire		delete_isr_bit3_in			= w_we ? w_conf_data_in[3]  : 1'b0 ;
wire		delete_isr_bit2_in			= w_we ? w_conf_data_in[2]  : 1'b0 ;
wire		delete_isr_bit1_in			= w_we ? w_conf_data_in[1]  : 1'b0 ;
wire		delete_isr_bit0_in			= w_we ? w_conf_data_in[0]  : 1'b0 ;   

// This are aditional register bits, which are resets when their value is '1' !!!
always@(posedge w_clock or posedge reset)
begin
	if (reset) // Asynchronous RESET sets signals to '1'
	begin
		delete_status_bit15 <= #`FF_DELAY 1'b1 ;
		delete_status_bit14 <= #`FF_DELAY 1'b1 ;
		delete_status_bit13 <= #`FF_DELAY 1'b1 ;
		delete_status_bit12 <= #`FF_DELAY 1'b1 ;
		delete_status_bit11 <= #`FF_DELAY 1'b1 ;
		delete_status_bit8 <= #`FF_DELAY 1'b1 ;
		delete_pci_err_cs_bit10 <= #`FF_DELAY 1'b1 ;
		delete_pci_err_cs_bit8 <= #`FF_DELAY 1'b1 ; 
		delete_wb_err_cs_bit10 <= #`FF_DELAY 1'b1 ;
		delete_wb_err_cs_bit8 <= #`FF_DELAY 1'b1 ;
		delete_isr_bit3 <= #`FF_DELAY 1'b1 ;
		delete_isr_bit2 <= #`FF_DELAY 1'b1 ;
		delete_isr_bit1 <= #`FF_DELAY 1'b1 ;
		delete_isr_bit0 <= #`FF_DELAY 1'b1 ;   
	end
	else
	begin // If '1' is written into, then it also sets signals to '1'
		case (w_conf_address_in[8])
		1'b0 :
		// if (~w_conf_address_in[8])
		begin
		  case ({w_conf_address_in[7], w_conf_address_in[6]})
		  2'b00 :
		  //  if ((~w_conf_address_in[7]) && (~w_conf_address_in[6]))
		  begin
			// PCI header - configuration space
			case (w_conf_address_in[5:2])
			4'h1: 
			begin
				if (w_byte_en[3] == 1'b0)
				begin
					delete_status_bit15 <= #`FF_DELAY delete_status_bit15_in ;
					delete_status_bit14 <= #`FF_DELAY delete_status_bit14_in ;
					delete_status_bit13 <= #`FF_DELAY delete_status_bit13_in ;
					delete_status_bit12 <= #`FF_DELAY delete_status_bit12_in ;
					delete_status_bit11 <= #`FF_DELAY delete_status_bit11_in ;
					delete_status_bit8  <= #`FF_DELAY delete_status_bit8_in ;
				end
			end
			default :
			begin
			end
			endcase
		  end
		  default :
		  begin
		  end
		  endcase
		end
		default :
		//  else
		begin
			// PCI target - configuration space
			case (w_conf_address_in[7:2])
	        `P_ERR_CS_ADDR: 
			begin
				if (w_byte_en[1] == 1'b0)
				begin
					delete_pci_err_cs_bit10 <= #`FF_DELAY delete_pci_err_cs_bit10_in ;
					delete_pci_err_cs_bit8  <= #`FF_DELAY delete_pci_err_cs_bit8_in ;
				end
			end
			`W_ERR_CS_ADDR: 
			begin
				if (w_byte_en[1] == 1'b0)
				begin
					delete_wb_err_cs_bit10 <= #`FF_DELAY delete_wb_err_cs_bit10_in ;
					delete_wb_err_cs_bit8  <= #`FF_DELAY delete_wb_err_cs_bit8_in ;
				end
			end
			`ISR_ADDR: 
			begin
				if (w_byte_en[0] == 1'b0)
				begin
					delete_isr_bit3 <= #`FF_DELAY delete_isr_bit3_in ;
					delete_isr_bit2 <= #`FF_DELAY delete_isr_bit2_in ;
					delete_isr_bit1 <= #`FF_DELAY delete_isr_bit1_in ;
					delete_isr_bit0 <= #`FF_DELAY delete_isr_bit0_in ;
				end
			end
			default	: 
			begin
			end
			endcase
		end
		endcase
	end
end

// Following are REGISTERS , which have "asynchronous & synchronous RESET" and synchronous SET on pci_clk !
always@(posedge pci_clk or posedge delete_status_bit15)
begin
	if (delete_status_bit15) // Asynchronous reset
		status_bit15_11[15] <= #`FF_DELAY 1'b0 ;
	else
		if (perr_in)
			status_bit15_11[15] <= #`FF_DELAY 1'b1 ;
end
always@(posedge pci_clk or posedge delete_status_bit14)
begin
	if (delete_status_bit14) // Asynchronous reset
		status_bit15_11[14] <= #`FF_DELAY 1'b0 ;
	else
		if (serr_in)
			status_bit15_11[14] <= #`FF_DELAY 1'b1 ;
end
always@(posedge pci_clk or posedge delete_status_bit13)
begin
	if (delete_status_bit13) // Asynchronous reset
		status_bit15_11[13] <= #`FF_DELAY 1'b0 ;
	else
		if (master_abort_recv)
			status_bit15_11[13] <= #`FF_DELAY 1'b1 ;
end
always@(posedge pci_clk or posedge delete_status_bit12)
begin
	if (delete_status_bit12) // Asynchronous reset
		status_bit15_11[12] <= #`FF_DELAY 1'b0 ;
	else
		if (target_abort_recv)
			status_bit15_11[12] <= #`FF_DELAY 1'b1 ;
end
always@(posedge pci_clk or posedge delete_status_bit11)
begin
	if (delete_status_bit11) // Asynchronous reset
		status_bit15_11[11] <= #`FF_DELAY 1'b0 ;
	else
		if (target_abort_set)
			status_bit15_11[11] <= #`FF_DELAY 1'b1 ;
end
always@(posedge pci_clk or posedge delete_status_bit8)
begin
	if (delete_status_bit8) // Asynchronous reset
		status_bit8 <= #`FF_DELAY 1'b0 ;
	else
		if (master_data_par_err && command_bit6)
			status_bit8 <= #`FF_DELAY 1'b1 ;
end

// Following are REGISTERS , which have "asynchronous & synchronous RESET" and synchronous SET on w_clock !
always@(posedge wb_clk or posedge delete_pci_err_cs_bit10)
begin
	if (delete_pci_err_cs_bit10) // Asynchronous reset
		pci_err_cs_bit10 <= #`FF_DELAY 1'b0 ;
	else
		if (pci_error_rty_exp && pci_err_cs_bit0)
			pci_err_cs_bit10 <= #`FF_DELAY 1'b1 ;
end
always@(posedge wb_clk or posedge delete_pci_err_cs_bit8)
begin
	if (delete_pci_err_cs_bit8) // Asynchronous reset
		pci_err_cs_bit8 <= #`FF_DELAY 1'b0 ;
	else
		if (pci_error_sig && pci_err_cs_bit0)
			pci_err_cs_bit8 <= #`FF_DELAY 1'b1 ;
end
always@(posedge wb_clk or posedge reset)
begin
	if (reset) // Asynchronous reset
    begin
		pci_err_cs_bit31_24 <= #`FF_DELAY 8'h00 ;
		pci_err_addr <= #`FF_DELAY 32'h0000_0000 ;
		pci_err_data <= #`FF_DELAY 32'h0000_0000 ;
    end
	else
		if (pci_error_sig && pci_err_cs_bit0)
		begin
			pci_err_cs_bit31_24 <= #`FF_DELAY { pci_error_be, pci_error_bc } ;
			pci_err_addr <= #`FF_DELAY pci_error_addr ;
			pci_err_data <= #`FF_DELAY pci_error_data ;
		end
end

always@(posedge pci_clk or posedge delete_wb_err_cs_bit10)
begin
	if (delete_wb_err_cs_bit10) // Asynchronous reset
		wb_err_cs_bit10_8[10] <= #`FF_DELAY 1'b0 ;
	else
		if (wb_error_rty_exp && wb_err_cs_bit0)
			wb_err_cs_bit10_8[10] <= #`FF_DELAY 1'b1 ;
end
always@(posedge pci_clk or posedge reset)
begin
	if (reset) // Asynchronous reset
		wb_err_cs_bit10_8[9] <= #`FF_DELAY 1'b0 ;
	else
		if (wb_error_sig && wb_err_cs_bit0)
			wb_err_cs_bit10_8[9] <= #`FF_DELAY wb_error_es ;
end
always@(posedge pci_clk or posedge delete_wb_err_cs_bit8)
begin
	if (delete_wb_err_cs_bit8) // Asynchronous reset
		wb_err_cs_bit10_8[8] <= #`FF_DELAY 1'b0 ;
	else
		if (wb_error_sig && wb_err_cs_bit0)
			wb_err_cs_bit10_8[8] <= #`FF_DELAY 1'b1 ;
end
always@(posedge pci_clk or posedge reset)
begin
	if (reset) // Asynchronous reset
    begin
		wb_err_cs_bit31_24 <= #`FF_DELAY 8'h00 ;
		wb_err_addr <= #`FF_DELAY 32'h0000_0000 ;
		wb_err_data <= #`FF_DELAY 32'h0000_0000 ;
    end
	else
		if (wb_error_sig && wb_err_cs_bit0)
		begin
			wb_err_cs_bit31_24 <= #`FF_DELAY { wb_error_be, wb_error_bc } ;
			wb_err_addr <= #`FF_DELAY wb_error_addr ;
			wb_err_data <= #`FF_DELAY wb_error_data ;
		end
end

always@(posedge w_clock or posedge delete_isr_bit3)
begin
	if (delete_isr_bit3) // Asynchronous reset
		isr_bit3_0[3] <= #`FF_DELAY 1'b0 ;
	else
		if (isr_int_prop && icr_bit3_0[3])
			isr_bit3_0[3] <= #`FF_DELAY 1'b1 ;
end
always@(posedge w_clock or posedge delete_isr_bit2)
begin
	if (delete_isr_bit2) // Asynchronous reset
		isr_bit3_0[2] <= #`FF_DELAY 1'b0 ;
	else
		if (isr_err_int && icr_bit3_0[2])
			isr_bit3_0[2] <= #`FF_DELAY 1'b1 ;
end
always@(posedge w_clock or posedge delete_isr_bit1)
begin
	if (delete_isr_bit1) // Asynchronous reset
		isr_bit3_0[1] <= #`FF_DELAY 1'b0 ;
	else
		if (isr_par_err_int && icr_bit3_0[1])
			isr_bit3_0[1] <= #`FF_DELAY 1'b1 ;
end
always@(posedge w_clock or posedge delete_isr_bit0)
begin
	if (delete_isr_bit0) // Asynchronous reset
		isr_bit3_0[0] <= #`FF_DELAY 1'b0 ;
	else
		if (isr_sys_err_int && icr_bit3_0[0])
			isr_bit3_0[0] <= #`FF_DELAY 1'b1 ;
end


/*-----------------------------------------------------------------------------------------------------------
        OUTPUTs from registers !!!             
-----------------------------------------------------------------------------------------------------------*/
// PCI header outputs from command register
assign		serr_enable = command_bit8 ;				
assign		perr_response = command_bit6 ;              
assign		pci_master_enable = command_bit2_0[2] ;          
assign		memory_space_enable = command_bit2_0[1] ;
assign		io_space_enable = command_bit2_0[0] ;
// PCI header output from cache_line_size, latency timer and interrupt pin
assign		cache_line_size[7 : 0] = cache_line_size_reg ;                         
assign		latency_tim[7 : 0] = latency_timer ;               
assign		int_pin[2 : 0] = r_interrupt_pin ;                                          
// PCI output from image registers
assign		pci_base_addr0[31 : 12] = pci_ba0_bit31_12 ; 
assign		pci_base_addr1[31 : 12] = pci_ba1_bit31_12 ; 
assign		pci_base_addr2[31 : 12] = pci_ba2_bit31_12 ; 
assign		pci_base_addr3[31 : 12] = pci_ba3_bit31_12 ; 
assign		pci_base_addr4[31 : 12] = pci_ba4_bit31_12 ; 
assign		pci_base_addr5[31 : 12] = pci_ba5_bit31_12 ; 
assign		pci_memory_io0 = pci_ba0_bit0 ;
assign		pci_memory_io1 = pci_ba1_bit0 ;
assign		pci_memory_io2 = pci_ba2_bit0 ;
assign		pci_memory_io3 = pci_ba3_bit0 ;
assign		pci_memory_io4 = pci_ba4_bit0 ;
assign		pci_memory_io5 = pci_ba5_bit0 ;
assign		pci_addr_mask0[31 : 12] = pci_am0 ; 
assign		pci_addr_mask1[31 : 12] = pci_am1 ; 
assign		pci_addr_mask2[31 : 12] = pci_am2 ; 
assign		pci_addr_mask3[31 : 12] = pci_am3 ; 
assign		pci_addr_mask4[31 : 12] = pci_am4 ; 
assign		pci_addr_mask5[31 : 12] = pci_am5 ; 
assign		pci_tran_addr0[31 : 12] = pci_ta0 ; 
assign		pci_tran_addr1[31 : 12] = pci_ta1 ; 
assign		pci_tran_addr2[31 : 12] = pci_ta2 ; 
assign		pci_tran_addr3[31 : 12] = pci_ta3 ; 
assign		pci_tran_addr4[31 : 12] = pci_ta4 ; 
assign		pci_tran_addr5[31 : 12] = pci_ta5 ; 
assign		pci_img_ctrl0[2 : 1] = pci_img_ctrl0_bit2_1 ;  
assign		pci_img_ctrl1[2 : 1] = pci_img_ctrl1_bit2_1 ;  
assign		pci_img_ctrl2[2 : 1] = pci_img_ctrl2_bit2_1 ;  
assign		pci_img_ctrl3[2 : 1] = pci_img_ctrl3_bit2_1 ;  
assign		pci_img_ctrl4[2 : 1] = pci_img_ctrl4_bit2_1 ;  
assign		pci_img_ctrl5[2 : 1] = pci_img_ctrl5_bit2_1 ;  
// PCI output from pci error control and status register
assign		pci_error_en = pci_err_cs_bit0 ; 
assign		pci_error_sig_set = pci_err_cs_bit8 ;
assign		pci_error_rty_exp_set = pci_err_cs_bit10 ;
// WISHBONE output from image registers
assign		wb_base_addr0[31 : 12] = wb_ba0_bit31_12 ;
assign		wb_base_addr1[31 : 12] = wb_ba1_bit31_12 ;
assign		wb_base_addr2[31 : 12] = wb_ba2_bit31_12 ;
assign		wb_base_addr3[31 : 12] = wb_ba3_bit31_12 ;
assign		wb_base_addr4[31 : 12] = wb_ba4_bit31_12 ;
assign		wb_base_addr5[31 : 12] = wb_ba5_bit31_12 ;
assign		wb_memory_io0 = wb_ba0_bit0 ;
assign		wb_memory_io1 = wb_ba1_bit0 ;
assign		wb_memory_io2 = wb_ba2_bit0 ;
assign		wb_memory_io3 = wb_ba3_bit0 ;
assign		wb_memory_io4 = wb_ba4_bit0 ;
assign		wb_memory_io5 = wb_ba5_bit0 ;
assign		wb_addr_mask0[31 : 12] = wb_am0 ;
assign		wb_addr_mask1[31 : 12] = wb_am1 ;
assign		wb_addr_mask2[31 : 12] = wb_am2 ;
assign		wb_addr_mask3[31 : 12] = wb_am3 ;
assign		wb_addr_mask4[31 : 12] = wb_am4 ;
assign		wb_addr_mask5[31 : 12] = wb_am5 ;
assign		wb_tran_addr0[31 : 12] = wb_ta0 ;
assign		wb_tran_addr1[31 : 12] = wb_ta1 ;
assign		wb_tran_addr2[31 : 12] = wb_ta2 ;
assign		wb_tran_addr3[31 : 12] = wb_ta3 ;
assign		wb_tran_addr4[31 : 12] = wb_ta4 ;
assign		wb_tran_addr5[31 : 12] = wb_ta5 ;
assign		wb_img_ctrl0[2 : 0] = wb_img_ctrl0_bit2_0 ; 
assign		wb_img_ctrl1[2 : 0] = wb_img_ctrl1_bit2_0 ; 
assign		wb_img_ctrl2[2 : 0] = wb_img_ctrl2_bit2_0 ; 
assign		wb_img_ctrl3[2 : 0] = wb_img_ctrl3_bit2_0 ; 
assign		wb_img_ctrl4[2 : 0] = wb_img_ctrl4_bit2_0 ; 
assign		wb_img_ctrl5[2 : 0] = wb_img_ctrl5_bit2_0 ; 
// WISHBONE output from wb error control and status register
assign		wb_error_en = wb_err_cs_bit0 ; 
assign		wb_error_sig_set = wb_err_cs_bit10_8[8] ;
assign		wb_error_rty_exp_set = wb_err_cs_bit10_8[10] ;
// GENERAL output from conf. cycle generation register & int. control register 
assign		config_addr[23 : 0] = { cnf_addr_bit23_2, 1'b0, cnf_addr_bit0 } ; 		
assign		icr_soft_res = icr_bit31 ; 
assign		serr_int_en = icr_bit3_0[3] ;
assign		perr_int_en = icr_bit3_0[2] ;
assign		error_int_en = icr_bit3_0[1] ;
assign		int_prop_en = icr_bit3_0[0] ;

                                
endmodule                       
			                    