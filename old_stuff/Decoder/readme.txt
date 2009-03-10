//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: readme.txt                                       ////
////  Module name: DECODER                                        ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - Tadej Markovic, tadej@opencores.org                   ////
////                                                              ////
////  List of files:                                              ////
////      - decoder.v (source code)                               ////
////      - defines.v (defines and constants for source code)     ////
////      - tb_decoder.v (test bench)                             ////
////      - tb_defines.v (defines and constants for test bench)   ////
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
// Revision 1.1.1.1  2001/06/06 15:11:35  tadej
// Initial release
//
//

DECODER is a module used in a PCI Target and in a WISHBONE 
slave units. It's purpose is to decode an input address and
translate it, but for that the input address must fall into
the image size.

The base address is written into the Base Address register. 
How many Most Significant bits are masked and replaced with 
translation address bits, depends on Address Mask register, 
which also defines the size of the image. There is one rule, 
how to set the Address Mask register. Address bits that can 
be masked, must start with MSbit (bit[31]) and follow till 
twelfth bit (bit[11]). All the allowed masked bits defines 
the smallest size of 4kBytes, that can be assigned. Between 
mask bits there must be no zeros, otherwise this image will 
have two base addresses with one base address register, what 
do not complies with PCI specification. Because of that, the 
output of Address Mask register is connected to Mask Correction 
unit. Bit[31] of the Mask register must be set to one to
enable the image space. Therefor the maximum image size is
2 GB. 
To enable the translation of the input address, the bit[2]
of the Image Control register must be set to one, othervise
the output address is equal to input address.
To find out if address falls into the correct address range, 
the masked bits of an input address and of the base address 
must be compared (the number of masked bits defines the 
unchanging address of current address range, and with that 
the size of this image). Masking of the input and the base 
addresses is made with AND logic function between each one 
of the address and address mask.
To get the correct output address, there must be combined 
(with OR logic function) the unmasked bits of the input address 
and masked bits of the translation address. The unmasked bits 
of the input address can be achieved with masking (with AND 
logic function) the input address with negated mask bits.

