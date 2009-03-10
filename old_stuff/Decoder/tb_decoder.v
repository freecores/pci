//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: tb_decoder.v                                     ////
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
// Revision 1.1.1.1  2001/06/06 15:11:37  tadej
// Initial release
//
//

//`include "defines.v"
`include "tb_defines.v"
`include "decoder.v"

module TB_decoder () ;

// output control from UUT  
wire	image_hit ;
// output address from UUT and input address to UUT
wire	[(`REG_WIDTH - 1) : 0] address_out ;
reg		[(`REG_WIDTH - 1) : 0] address_in ;
// input register values to UUT
reg		[(`REG_WIDTH - 1) : 12] base_address ;
wire	[(`REG_WIDTH - 1) : 12] mask_address ;
reg		[(`REG_WIDTH - 1) : 12] tran_address ;
// input register bit address translation enable to UUT
reg		at_enable ;

// clock for TB environment
reg		clock ;

// mask for setting up mask register
reg		[(`REG_WIDTH - 1) : 12] mask ;
reg		[(`REG_WIDTH - 1) : 12] mask_correct ;

// expected image hit
reg		expected_hit ;
// expected output address
reg		[(`REG_WIDTH - 1) : 0] expected_addr_out ;

// starting values to UUT
initial
	begin
// clock starts with logic zero
		clock <= 1'b0 ;
// input address is predefined
		address_in <= 32'h0000_0000 ;
// image mapping registers are predefined
		base_address <= 20'h0004_0 ;
		mask <= 20'hffff_f ;
		tran_address <= 20'h0400_0 ;
		at_enable <= 1'b1 ;
// start with calling tasks
		# 20 call_task ;
// simulation stops after 500 time units
		# 20 $stop ;
	end 

// clock generation with period of 2*T
// it is not necessary, but othervise testbench doesn't work
always 
	begin
		#`T clock = ~clock ;
	end

// set up the mask_addr register
assign	mask_address = mask ;

task call_task ;
	begin
		# 0 fall_into_image_space_tran_en ;
		# 4 fall_into_image_space_tran_dis ;
		# 4 fall_across_image_space_tran_en ;
		# 4 fall_across_image_space_tran_dis ;
	end
endtask
		
// testing when input address falls in and out of the image space, when address translation is performed
task fall_into_image_space_tran_en ;
	begin
		fork
			mask <= 20'hf0ff_0 ;
			mask_correct <= 20'hffff_0 ;
			at_enable <= 1'b1 ;
            
			address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			#  6 bus_monitor ;
			# 10 address_in <= { base_address, 12'h000 } ;
			# 16 bus_monitor ;
			# 20 address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			# 26 bus_monitor ;
			# 30 address_in <= { base_address, 12'h000 } + 32'h0000_0001 ;
			# 36 bus_monitor ;
			# 40 address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			# 46 bus_monitor ;
			# 50 address_in <= { base_address, 12'h000 } + 32'h0000_8000 ;
			# 56 bus_monitor ;
			# 60 address_in <= { base_address, 12'h000 } + 32'h0000_7ffc ;
			# 66 bus_monitor ;
			# 70 address_in <= { base_address, 12'h000 } - 32'h0000_f000 ;
			# 76 bus_monitor ;
			# 80 address_in <= { base_address, 12'h000 } + 32'h0000_fefe ;
			# 86 bus_monitor ;
// here we put mask to ZERO in order to disable image space !!!
			# 90 mask <= 20'h0000_0 ;
			# 90 mask_correct <= 20'h0000_0 ;
			# 100 address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			# 106 bus_monitor ;
			# 110 address_in <= { base_address, 12'h000 } + 32'h0000_8000 ;
			# 116 bus_monitor ;
			# 120 address_in <= { base_address, 12'h000 } - 32'h0000_f000 ;
			# 126 bus_monitor ;
			# 130 address_in <= { base_address, 12'h000 } + 32'h0000_fefe ;
			# 136 bus_monitor ;
		join
	end
endtask

// testing when input address falls in and out of the image space, when address translation is NOT performed
task fall_into_image_space_tran_dis ;
	begin
		fork
			mask <= 20'hf0ff_0 ;
			mask_correct <= 20'hffff_0 ;
			at_enable <= 1'b0 ;

			address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			#  6 bus_monitor ;
			# 10 address_in <= { base_address, 12'h000 } ;
			# 16 bus_monitor ;               
			# 20 address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			# 26 bus_monitor ;               
			# 30 address_in <= { base_address, 12'h000 } + 32'h0000_0001 ;
			# 36 bus_monitor ;               
			# 40 address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			# 46 bus_monitor ;               
			# 50 address_in <= { base_address, 12'h000 } + 32'h0000_8000 ;
			# 56 bus_monitor ;               
			# 60 address_in <= { base_address, 12'h000 } - 32'h0000_f000 ;
			# 66 bus_monitor ;               
			# 70 address_in <= { base_address, 12'h000 } + 32'h0000_fefe ;
			# 76 bus_monitor ;
		join
	end
endtask

// testing when input address falls across image space, when address translation is performed
task fall_across_image_space_tran_en ;
	begin
		fork
			mask <= 20'h0fff_0 ;
			mask_correct <= 20'hffff_0 ;
			at_enable <= 1'b1 ;
            
			address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			#  6 bus_monitor ;
			# 10 address_in <= { base_address, 12'h000 } + 32'h0001_0000 ;
			# 16 bus_monitor ;               
			# 20 address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			# 26 bus_monitor ;               
			# 30 address_in <= { base_address, 12'h000 } + 32'h0003_0001 ;
			# 36 bus_monitor ;               
			# 40 address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			# 46 bus_monitor ;               
			# 50 address_in <= { base_address, 12'h000 } + 32'h2000_8000 ;
			# 56 bus_monitor ;               
			# 60 address_in <= { base_address, 12'h000 } - 32'h0000_f000 ;
			# 66 bus_monitor ;               
			# 70 address_in <= { base_address, 12'h000 } + 32'hf000_fefe ;
			# 76 bus_monitor ;
		join                  
	end
endtask                       
                              
// testing when input address falls across image space, when address translation is NOT performed
task fall_across_image_space_tran_dis ;
	begin
		fork
			mask <= 20'hf0ff_0 ;
			mask_correct <= 20'hffff_0 ;
			at_enable <= 1'b0 ;
                 
			address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			#  6 bus_monitor ;        
			# 10 address_in <= { base_address, 12'h000 } + 32'h0001_0000 ;
			# 16 bus_monitor ;            
			# 20 address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			# 26 bus_monitor ;            
			# 30 address_in <= { base_address, 12'h000 } + 32'h0003_0001 ;
			# 36 bus_monitor ;            
			# 40 address_in <= { base_address, 12'h000 } - 32'h0000_0001 ;
			# 46 bus_monitor ;            
			# 50 address_in <= { base_address, 12'h000 } + 32'h00f0_8000 ;
			# 56 bus_monitor ;            
			# 60 address_in <= { base_address, 12'h000 } - 32'h0000_f000 ;
			# 66 bus_monitor ;            
			# 70 address_in <= { base_address, 12'h000 } + 32'h0fc0_fefe ;
			# 76 bus_monitor ;
		join
	end
endtask

// HIT and ADDR_OUT prediction !
always @ (address_in or mask_correct or at_enable)
	begin
		if ((base_address & mask_correct) == (address_in[(`REG_WIDTH - 1) : 12] & mask_correct))
			begin
				expected_hit <= mask_correct[31] ;
			end
		else
			begin
				expected_hit <= 1'b0 ;
			end
		if (at_enable)
			begin
				expected_addr_out <= { ((address_in[(`REG_WIDTH - 1) : 12] & (~mask_correct)) | (tran_address[(`REG_WIDTH - 1) : 12] & mask_correct)), address_in [11 : 0] } ;
			end
		else
			begin
				expected_addr_out <= address_in ;
			end
	end

// BUS monitor
task bus_monitor ;
	begin
		if (image_hit == expected_hit)
			begin
				$display("Image hit OK") ;
			end
		else
			begin
				$display("Image hit ERROR at time %t:, with address_in: %h, and address_out: %h ", $time, address_in, address_out) ;
			end
		if (address_out == expected_addr_out)
			begin
				$display("Output address OK") ;
			end
		else
			begin
				$display("Output address ERROR at time %t:, with address_in: %h, and address_out: %h ", $time, address_in, address_out) ;
			end
	end
endtask


DECODER dec1 (	.hit(image_hit), .addr_out(address_out), .addr_in(address_in), .base_addr(base_address),
				.mask_addr(mask_address), .tran_addr(tran_address), .at_en(at_enable) ) ;

endmodule 

        
    

