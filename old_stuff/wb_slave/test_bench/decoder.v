//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: decoder.v                                        ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - Tadej Markovic, tadej@opencores.org                   ////
////      - Tilen Novak, tilen@opencores.org                      ////
////                                                              ////
////  All additional information is avaliable in the README.txt   ////
////  file.                                                       ////
////                                                              ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000 Tadej Markovic, tadej@opencores.org       ////
////                    Tilen Novak, tilen@opencores.org          ////
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
// Revision 1.2  2001/07/19 09:52:33  mihad
// Only delayed write storage is provided after merging delayed requests.
// All delayed reads now pass through FIFO.
//
//

`include "constants.v"

module DECODER (hit, addr_out, addr_in, base_addr, mask_addr, tran_addr, at_en) ;

/*-----------------------------------------------------------------------------------------------------------
DECODER interface decodes input address (ADDR_IN); what means that it validates (HIT), if input address 
falls within the defined image space boundaries. Image space boundarie is defined with image base address
register (BASE_ADDR) and address mask register (MASK_ADDR).
Beside that, it also translates (maps) the input address to the output address (ADDR_OUT), regarding the
translation address register (TRAN_ADDR) and the address mask register.
-----------------------------------------------------------------------------------------------------------*/

// output control  
output	hit ;
// output address 
output	[(`REG_WIDTH - 1) : 0] addr_out ;
// input address
input	[(`REG_WIDTH - 1) : 0] addr_in ;

// input registers - 12 LSbits are not valid since the smallest possible size is 4KB !
input	[(`REG_WIDTH - 1) : 12] base_addr ;
input	[(`REG_WIDTH - 1) : 12] mask_addr ;
input	[(`REG_WIDTH - 1) : 12] tran_addr ;

// input bit[2] of the Image Control register used to enable the address translation !
input	at_en ;

/*-----------------------------------------------------------------------------------------------------------
Internal signals !
-----------------------------------------------------------------------------------------------------------*/

// corrected mask_addr value and its negation
reg		[(`REG_WIDTH - 1) : 12] mask_addr_corret ;
wire	[(`REG_WIDTH - 1) : 12] mask_addr_corret_neg ;

// bit[31] if address mask register is IMAGE ENABLE bit (img_en)
wire	img_en ;

// addr_in_compare are masked input address bits that are compared with masked base_addr
wire	[(`REG_WIDTH - 1) : 12] addr_in_compare ;
// base_addr_compare are masked base address bits that are compared with masked addr_in
wire	[(`REG_WIDTH - 1) : 12] base_addr_comapre ;

// addr_in_combine input address bits are not replaced with translation address!
wire	[(`REG_WIDTH - 1) : 12] addr_in_combine ;
// tran_addr_combine are masked and combined with addr_in_combine!
reg		[(`REG_WIDTH - 1) : 12] tran_addr_combine ;

// index for loops
integer	i ;

/*-----------------------------------------------------------------------------------------------------------
Correction logic of the Address Mask register value!
This does not affect device operation latency, since correction is made only when mask_addr register is
changed !
There are only 20 MSbits of address mask corrected, since 12 LSbits (4 KB) of input address can not be
measked!

Bit 12 is allways correct! Between MASK bits (ones) there must be no zeros (otherwise there will be two
base addresses for one base address register). That is corrected with logic OR function. The corrected
bit with weight [n] is a result of a logic OR of mask_addr bits from [n] to [12]!
-----------------------------------------------------------------------------------------------------------*/

always @ (mask_addr)
	begin
		mask_addr_corret[12] = mask_addr[12] ;
//		for (i = 13; i < (`REG_WIDTH - 1); i = i + 1)
//			begin
//				mask_addr_corret[i] = | (mask_addr[i : 12]) ;
//			end

// compiler does not accept FOR loop, because of index [i] on the right side :(
		mask_addr_corret[13] = | (mask_addr[13 : 12]) ;
		mask_addr_corret[14] = | (mask_addr[14 : 12]) ;
		mask_addr_corret[15] = | (mask_addr[15 : 12]) ;
		mask_addr_corret[16] = | (mask_addr[16 : 12]) ;
		mask_addr_corret[17] = | (mask_addr[17 : 12]) ;
		mask_addr_corret[18] = | (mask_addr[18 : 12]) ;
		mask_addr_corret[19] = | (mask_addr[19 : 12]) ;
		mask_addr_corret[20] = | (mask_addr[20 : 12]) ;
		mask_addr_corret[21] = | (mask_addr[21 : 12]) ;
		mask_addr_corret[22] = | (mask_addr[22 : 12]) ;
		mask_addr_corret[23] = | (mask_addr[23 : 12]) ;
		mask_addr_corret[24] = | (mask_addr[24 : 12]) ;
		mask_addr_corret[25] = | (mask_addr[25 : 12]) ;
		mask_addr_corret[26] = | (mask_addr[26 : 12]) ;
		mask_addr_corret[27] = | (mask_addr[27 : 12]) ;
		mask_addr_corret[28] = | (mask_addr[28 : 12]) ;
		mask_addr_corret[29] = | (mask_addr[29 : 12]) ;
		mask_addr_corret[30] = | (mask_addr[30 : 12]) ;
		mask_addr_corret[31] = | (mask_addr[31 : 12]) ;

	end

assign # `DLY_L1 mask_addr_corret_neg = ~ mask_addr_corret ;

/*-----------------------------------------------------------------------------------------------------------
Decoding the input address!
This logic produces the loghest path in this module!

20 MSbits of input addres are as well as base address (20 bits) masked with corrected address mask. Only
masked bits of each vector are actually logically compared.
Bit[31] of address mask register is used to enable the image space !
-----------------------------------------------------------------------------------------------------------*/

assign # `DLY_L1 addr_in_compare = (addr_in[(`REG_WIDTH - 1) : 12] & mask_addr_corret) ;

assign # `DLY_L1 base_addr_comapre = (base_addr & mask_addr_corret) ;

assign img_en = mask_addr_corret[31] ;

assign # `DLY_L4 hit = { 1'b1, addr_in_compare } == { img_en, base_addr_comapre } ;

/*-----------------------------------------------------------------------------------------------------------
Translating the input address!

20 MSbits of input address are masked with negated value of the corrected address mask in order to get
address bits of the input address which won't be replaced with translation address bits.
Translation address bits (20 bits) are masked with corrected address mask. Only masked bits of vector are 
actually valid, all others are zero. 
Boath vectors are bit-wise ORed in order to get the valid translation address with an offset of an input
address.
12 LSbits of an input address are assigned to 12 LSbits of an output addres.
-----------------------------------------------------------------------------------------------------------*/

assign # `DLY_L1 addr_in_combine = (addr_in[(`REG_WIDTH - 1) : 12] & mask_addr_corret_neg) ;

// if Address Translation Enable bit is set, then translation address is used othervise input address is used!
always
	begin
	    if (at_en) 
			begin
				# `DLY_L1 tran_addr_combine = (tran_addr & mask_addr_corret) ;
    		end
    	else
			begin
				# `DLY_L1 tran_addr_combine = (addr_in[(`REG_WIDTH - 1) : 12] & mask_addr_corret) ;
			end
	end

assign # `DLY_L2 addr_out[(`REG_WIDTH - 1) : 12] = addr_in_combine | tran_addr_combine ;

assign # `DLY_L1 addr_out[11 : 0] = addr_in [11 : 0] ;

endmodule

