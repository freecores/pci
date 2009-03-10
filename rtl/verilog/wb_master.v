//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: wb_master.v                                      ////
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
//

`define FSM_BITS 3 // number of bits needed for FSM states


`include "bus_commands.v"
`include "constants.v"

module WB_MASTER (  wb_clock_in,		// CLK_I
                    reset_in,			// RST_I
                    
                    pci_tar_read_request,
					pci_tar_address,
					pci_tar_cmd,
					pci_tar_be,
					pci_tar_prefetch_en,
					pci_cache_line_size,
					wb_read_done,

					pcir_fifo_wenable_out, 
					pcir_fifo_data_out,    
					pcir_fifo_be_out, 
					pcir_fifo_control_out,  
					//pcir_fifo_renable_out,			for PCI Target !!!
					//pcir_fifo_data_in,   				for PCI Target !!!
					//pcir_fifo_be_in, 					for PCI Target !!!
					//pcir_fifo_control_in,				for PCI Target !!!
					//pcir_fifo_flush_out, 				for PCI Target !!!
					pcir_fifo_almost_full_in, 
					pcir_fifo_full_in,               
					//pcir_fifo_almost_empty_in, 		for PCI Target !!!
					//pcir_fifo_empty_in, 				NOT used
					//pcir_fifo_transaction_ready_in,	NOT used
					//pciw_fifo_wenable_out, 			for PCI Target !!!
					//pciw_fifo_addr_data_out,			for PCI Target !!!   
					//pciw_fifo_cbe_out, 				for PCI Target !!!
					//pciw_fifo_control_out,			for PCI Target !!!       
					pciw_fifo_renable_out, 
					pciw_fifo_addr_data_in,                              
					pciw_fifo_cbe_in, 
					pciw_fifo_control_in,                                   
					//pciw_fifo_flush_out, 				NOT used
					//pciw_fifo_almost_full_in, 		for PCI Target !!!
					//pciw_fifo_full_in,       			for PCI Target !!!
					pciw_fifo_almost_empty_in, 
					pciw_fifo_empty_in, 
					pciw_fifo_transaction_ready_in,

					pci_error_sig_set_in,
					pci_error_sig_out,
					pci_error_bc,
					write_rty_cnt_exp_out,
					read_rty_cnt_exp_out,

                    CYC_O,
                    STB_O,
                    WE_O,
                    SEL_O,
                    ADR_O,
                    MDATA_I,
                    MDATA_O,
                    ACK_I,
                    RTY_I,
                    ERR_I,
                    CAB_O
                );

/*----------------------------------------------------------------------------------------------------------------------
Various parameters needed for state machine and other stuff
----------------------------------------------------------------------------------------------------------------------*/
parameter		S_IDLE			= `FSM_BITS'h0 ; 
parameter		S_WRITE			= `FSM_BITS'h1 ;
parameter		S_WRITE_ERR_RTY	= `FSM_BITS'h2 ;
parameter		S_READ			= `FSM_BITS'h3 ;
parameter		S_READ_RTY		= `FSM_BITS'h4 ;
parameter		S_TURN_ARROUND	= `FSM_BITS'h5 ;

/*----------------------------------------------------------------------------------------------------------------------
System signals inputs
wb_clock_in - WISHBONE bus clock input
reset_in    - system reset input controlled by bridge's reset logic
----------------------------------------------------------------------------------------------------------------------*/
input 			wb_clock_in ; 
input			reset_in ;

/*----------------------------------------------------------------------------------------------------------------------
Control signals from PCI Target for READS to PCIR_FIFO
---------------------------------------------------------------------------------------------------------------------*/
input			pci_tar_read_request ;		// read request from PCI Target
input	[31:0]	pci_tar_address ;			// address for requested read from PCI Target					
input	[3:0]	pci_tar_cmd ;				// command for requested read from PCI Target					
input	[3:0]	pci_tar_be ;				// byte enables for requested read from PCI Target              
input			pci_tar_prefetch_en ;		// pre-fetch enable for requested read from PCI Target          
input	[7:0]	pci_cache_line_size ;		// CACHE line size register value for burst length              
output			wb_read_done ;				// read done and PCIR_FIFO has data ready

reg				wb_read_done ;

/*----------------------------------------------------------------------------------------------------------------------
PCIR_FIFO control signals used for sinking data into PCIR_FIFO and status monitoring		 
---------------------------------------------------------------------------------------------------------------------*/
output			pcir_fifo_wenable_out ;		// PCIR_FIFO write enable output
output	[31:0]	pcir_fifo_data_out ;		// data output to PCIR_FIFO
output	[3:0]	pcir_fifo_be_out ;			// byte enable output to PCIR_FIFO
output	[3:0]	pcir_fifo_control_out ;		// control bus output to PCIR_FIFO
input			pcir_fifo_almost_full_in ;	// almost full status indicator from PCIR_FIFO
input			pcir_fifo_full_in ;			// full status indicator from PCIR_FIFO

reg				pcir_fifo_wenable_out ;
reg		[3:0]	pcir_fifo_control_out ;

/*----------------------------------------------------------------------------------------------------------------------
PCIW_FIFO control signals used for fetching data from PCIW_FIFO and status monitoring
---------------------------------------------------------------------------------------------------------------------*/
output			pciw_fifo_renable_out ;		// read enable for PCIW_FIFO output
input	[31:0]	pciw_fifo_addr_data_in ;	// address and data input from PCIW_FIFO
input	[3:0]	pciw_fifo_cbe_in ;			// command and byte_enables from PCIW_FIFO
input	[3:0]	pciw_fifo_control_in ;		// control bus input from PCIW_FIFO
input			pciw_fifo_almost_empty_in ;	// almost empty status indicator from PCIW_FIFO
input			pciw_fifo_empty_in ;		// empty status indicator from PCIW_FIFO
input			pciw_fifo_transaction_ready_in ;	// write transaction is ready in PCIW_FIFO

reg				pciw_fifo_renable_out ;

/*----------------------------------------------------------------------------------------------------------------------
Control INPUT / OUTPUT signals for configuration space reporting registers !!!
---------------------------------------------------------------------------------------------------------------------*/
input			pci_error_sig_set_in ;		// When error signaled bit in PCI_ERR_CS register is set
output			pci_error_sig_out ;			// When error on WB bus occures
output  [3:0]   pci_error_bc ;				// bus command at which error occured !
output			write_rty_cnt_exp_out ;		// Signaling that RETRY counter has expired during write transaction!
output			read_rty_cnt_exp_out ;		// Signaling that RETRY counter has expired during read transaction!

reg				pci_error_sig_out ;
reg				write_rty_cnt_exp_out ;
reg				read_rty_cnt_exp_out ;

/*----------------------------------------------------------------------------------------------------------------------
WISHBONE bus interface signals - can be connected directly to WISHBONE bus
---------------------------------------------------------------------------------------------------------------------*/
output          CYC_O ;   		// cycle indicator output
output          STB_O ;   		// strobe output - data is valid when strobe and cycle indicator are high
output          WE_O  ;   		// write enable output - 1 - write operation, 0 - read operation
output  [3:0]   SEL_O ;   		// Byte select outputs
output  [31:0]  ADR_O ;   		// WISHBONE address output
input   [31:0]  MDATA_I ; 		// WISHBONE slave interface input data bus
output  [31:0]  MDATA_O ; 		// WISHBONE slave interface output data bus
input           ACK_I ;   		// Acknowledge input - qualifies valid data on data output bus or received data on data input bus
input           RTY_I ;   		// retry input - signals from WISHBONE slave that cycle should be terminated and retried later
input           ERR_I ;   		// Signals from WISHBONE slave that access resulted in an error
output          CAB_O ;   		// consecutive address burst output - indicated that master will do a serial address transfer in current cycle

reg		[3:0]   SEL_O ;


/*###########################################################################################################
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	LOGIC, COUNTERS, STATE MACHINE and some control register bits
	=============================================================
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
###########################################################################################################*/

// wire for write attempt - 1 when PCI Target attempt to write and PCIW_FIFO has a write transaction ready
wire w_attempt = ( pciw_fifo_transaction_ready_in && ~pciw_fifo_empty_in ) ; 

// wire for read attempt - 1 when PCI Target is attempting a read and PCIR_FIFO is not full !
// because of transaction ordering, PCI Master must not start read untill all writes are done -> at that
//   moment PCIW_FIFO is empty !!! (when read is pending PCI Target will block new reads and writes)
wire r_attempt = ( pci_tar_read_request && ~pcir_fifo_full_in && pciw_fifo_empty_in ) ; 

// Signal is used for reads on WB, when there is retry!
reg				first_wb_data_access ;

reg				last_data_from_pciw_fifo ;	// signal tells when there is last data in pciw_fifo
reg				last_data_to_pcir_fifo ;	// signal tells when there will be last data for pcir_fifo

// Logic used in State Machine logic implemented out of State Machine because of less delay!
always@(pciw_fifo_control_in or pciw_fifo_almost_empty_in)
begin
	//	(pciw_fifo_control_in == `LAST) is the same as (pciw_fifo_control_in[0] == 1'b1)
	if (pciw_fifo_control_in[0] || pciw_fifo_almost_empty_in) // if last data is going to be transfered
		last_data_from_pciw_fifo <= 1'b1 ; // signal for last data from PCIW_FIFO
	else
		last_data_from_pciw_fifo <= 1'b0 ;
end

wire	[7:0]	cache_line_wire ; // wire assigned directly from cache_line register !!!
// Logic used in State Machine logic implemented out of State Machine because of less delay!
//   definition of signal telling, when there is last data written into FIFO
always@(pci_tar_cmd or pci_tar_prefetch_en or cache_line_wire or pcir_fifo_almost_full_in)
begin
	case ({pci_tar_cmd, pci_tar_prefetch_en})
	{`BC_IO_READ, 1'b0},
	{`BC_IO_READ, 1'b1},
	{`BC_MEM_READ, 1'b0} :
	begin	// when single cycle
		last_data_to_pcir_fifo <= 1'b1 ;
	end
	{`BC_MEM_READ, 1'b1},
	{`BC_MEM_READ_LN, 1'b0},
	{`BC_MEM_READ_LN, 1'b1},
	{`BC_MEM_READ_MUL, 1'b0} :
	begin	// when burst cycle
		if ((cache_line_wire == 8'h1) || pcir_fifo_almost_full_in)
			last_data_to_pcir_fifo <= 1'b1 ;
		else
			last_data_to_pcir_fifo <= 1'b0 ;
	end
	{`BC_MEM_READ_MUL, 1'b1} :
	begin	// when burst cycle
		if (pcir_fifo_almost_full_in)
			last_data_to_pcir_fifo <= 1'b1 ;
		else
			last_data_to_pcir_fifo <= 1'b0 ;
	end
	default :
	begin
		last_data_to_pcir_fifo <= 1'b0 ;
	end
	endcase
end

reg		[3:0]	wb_no_response_cnt ;
reg		[3:0]	wb_response_value ;
reg				wait_for_wb_response ;
reg				set_retry ; // 

// internal WB no response retry generator counter!
always@(posedge reset_in or posedge wb_clock_in)
begin
	if (reset_in)
		wb_no_response_cnt <= #`FF_DELAY 4'h0 ; 
	else
		wb_no_response_cnt <= #`FF_DELAY wb_response_value ;
end
// internal WB no response retry generator logic
always@(wait_for_wb_response or wb_no_response_cnt or ACK_I or ERR_I or RTY_I)
begin
	if (wb_no_response_cnt == 4'h8) // when there isn't response for 8 clocks, set internal retry
	begin
		wb_response_value <= 4'h0 ;
		set_retry <= 1'b1 ;
	end
	else
	begin
		if (ACK_I || ERR_I || RTY_I) // if device responded, then ZERO must be written into counter
			wb_response_value <= 4'h0 ;
		else
		if (wait_for_wb_response)
			wb_response_value <= wb_no_response_cnt + 1 ; // count clocks when no response
		else
			wb_response_value <= wb_no_response_cnt ;
		set_retry <= 1'b0 ;
	end
end

wire	retry = RTY_I || set_retry ; // retry signal - logic OR function between RTY_I and internal WB no response retry!
reg		[7:0]	rty_counter ; // output from retry counter
reg		[7:0]	rty_counter_in ; // input value - output value + 1 OR output value
reg				rty_counter_max_value ; // signal tells when retry counter riches maximum value!
reg				reset_rty_cnt ; // signal for asynchronous reset of retry counter after each complete transfere
reg				last_data_transferred ; // signal is set by STATE MACHINE after each complete transfere !

// sinchronous signal after each transfere and asynchronous signal 'reset_rty_cnt' after reset  
//   for reseting the retry counter
always@(posedge reset_in or posedge wb_clock_in)
begin
	if (reset_in)
		reset_rty_cnt <= 1'b1 ; // asynchronous set when reset signal is active
	else
		reset_rty_cnt <= ACK_I || ERR_I ; // synchronous set after completed transfere
end

// clock enable for retry counter - when there is sinchronous reset OR retry
wire			rty_cnt_clk_en = reset_rty_cnt || retry ;
// data written into retry counter - ZEROS when sinchronous reset OR incremented value when retry
wire	[7:0]	rty_cnt_data = reset_rty_cnt ? 8'h00 : rty_counter_in ;
// Retry counter register control
always@(posedge reset_in or posedge wb_clock_in)
begin
	if (reset_in)
		rty_counter <= #`FF_DELAY 8'h00 ;
	else
	begin
		if (rty_cnt_clk_en)
			rty_counter <= #`FF_DELAY rty_cnt_data ; // count-up or hold value depending on retry counter logic
	end
end
// Retry counter logic
always@(rty_counter or rty_counter_in)
begin
	if(rty_counter == `WB_RTY_CNT_MAX) // stop counting
	begin
       	rty_counter_in <= rty_counter ;
       	rty_counter_max_value <= 1'b1 ;
	end
	else
	begin
       	rty_counter_in <= rty_counter_in + 1'b1 ; // count up
       	rty_counter_max_value <= 1'b0 ;
	end
end		

reg		[7:0]	cache_line ; // output from cache line down-counter
reg		[7:0]	cache_line_in ; // input to cache line counter
reg				cache_line_into_cnt ; // control signal for loading cache line size to counter
reg				cache_line_count ; // control signal for count enable
reg				cache_line_reg_used ; // if pci_cache_line_size is ZERO then all PCIR_FIFO is read

assign	cache_line_wire = cache_line ;
// cache line size down-counter register control
always@(posedge wb_clock_in or posedge reset_in)
begin
	if (reset_in) // reset counter
		cache_line <= #`FF_DELAY 8'h00 ;
	else
		cache_line <= #`FF_DELAY cache_line_in ; // count down or hold value depending on cache line counter logic
end
// cache line size down-counter logic
always@(cache_line_into_cnt or cache_line_count or pci_cache_line_size or cache_line)
begin
	if (cache_line_into_cnt) // load cache line size into counter
	begin
		if (pci_cache_line_size == 8'h0)
			cache_line_in = 8'h01 ;
		else
			cache_line_in = pci_cache_line_size ;
	end
	else
	if (cache_line_count)
	begin
		cache_line_in = cache_line - 1'h1 ; // count down
	end
	else
	begin
		cache_line_in = cache_line ;
	end
end


reg		[31:0]	addr_cnt_out ;	// output value from address counter to WB ADDRESS output
reg		[31:0]	addr_cnt_in ;	// input address value to address counter
reg				addr_into_cnt ; // control signal for loading starting address into counter
reg				addr_count ; // control signal for count enable
reg		[3:0]	bc_register ; // used when error occures during writes!

// wb address counter register control
always@(posedge wb_clock_in or posedge reset_in)
begin
	if (reset_in) // reset counter
	begin
		addr_cnt_out <= #`FF_DELAY 32'h0000_0000 ;
		bc_register  <= #`FF_DELAY 4'h0 ;
	end
	else
	begin
		addr_cnt_out <= #`FF_DELAY addr_cnt_in ; // count up or hold value depending on cache line counter logic
		if (addr_into_cnt)
			bc_register  <= #`FF_DELAY pciw_fifo_cbe_in ;
	end
end
// wb address counter logic
always@(addr_into_cnt or r_attempt or addr_count or pciw_fifo_addr_data_in or pci_tar_address or addr_cnt_out)
begin
	if (addr_into_cnt) // load starting address into counter
	begin
		if (r_attempt)
		begin
			addr_cnt_in = pci_tar_address ; // if read request, then load read addresss from PCI Target
		end
		else
		begin
			addr_cnt_in = pciw_fifo_addr_data_in ; // if not read request, then load write address from PCIW_FIFO
		end
	end
	else
	if (addr_count)
	begin
		addr_cnt_in = addr_cnt_out + 3'h4 ; // count up for 32-bit alligned address 
	end
	else
	begin
		addr_cnt_in = addr_cnt_out ;
	end
end

reg wb_stb_o ; // Internal signal for driwing STB_O on WB bus
reg wb_we_o ; // Internal signal for driwing WE_O on WB bus
reg wb_cyc_o ; // Internal signal for driwing CYC_O on WB bus and for enableing burst signal generation

reg	retried ; // Signal is output value from FF and is set for one clock period after retried_d is set
reg	retried_d ; // Signal is set whenever cycle is retried and is input to FF for delaying -> used in S_IDLE state

reg	first_data_is_burst ; // Signal is set in S_WRITE or S_READ states, when data transfere is burst!
reg	burst_transfer ; // This signal is set when data transfere is burst and is reset with RESET or last data transfered

// FFs output signals tell, when there is first data out from FIFO (for BURST checking)
//   and for delaying retried signal
always@(posedge wb_clock_in or posedge reset_in)
begin
	if (reset_in) // reset signals
	begin
		retried <= #`FF_DELAY 1'b0 ;
	end
	else
	begin
		retried <= #`FF_DELAY retried_d ; // delaying retried signal  
	end
end

// Determinig if first data is a part of BURST or just a single transfere!
always@(addr_into_cnt or r_attempt or pci_tar_cmd or pci_tar_prefetch_en or pciw_fifo_control_in)
begin
	if (addr_into_cnt)
	begin
		if (r_attempt)
		begin
			if (((pci_tar_cmd == `BC_MEM_READ) && pci_tar_prefetch_en) ||
   	   			(pci_tar_cmd == `BC_MEM_READ_LN) ||
   	   			(pci_tar_cmd == `BC_MEM_READ_MUL))
				first_data_is_burst <= 1'b1 ; 
   	   		else
				first_data_is_burst <= 1'b0 ;
		end
		else
		begin
			// When first data is not LAST, then there will be burst transfere
			if (pciw_fifo_control_in[`BURST_BIT]) // (pciw_fifo_control_in != `LAST)
				first_data_is_burst <= 1'b1 ; 
   	   		else
				first_data_is_burst <= 1'b0 ;
		end
	end
	else
		first_data_is_burst <= 1'b0 ;
end

// clock enable for burst_transfere signal when last data is transfered OR if first data is burst
wire burst_transfer_clk_en = last_data_transferred || first_data_is_burst ;
// data written into burst_transfere register is ZERO when last data is transfered OR ONE when first data is burst
wire burst_transfer_data = last_data_transferred ? 1'b0 : 1'b1 ;
// FF for seting and reseting burst_transfere signal
always@(posedge wb_clock_in or posedge reset_in)
begin
	if (reset_in) 
		burst_transfer <= #`FF_DELAY 1'b0 ;
	else
	begin
		if (burst_transfer_clk_en)
			burst_transfer <= #`FF_DELAY burst_transfer_data ;
	end
end

reg [(`FSM_BITS - 1):0]  c_state ; //current state register
reg [(`FSM_BITS - 1):0]  n_state ; //next state input to current state register

// state machine register control
always@(posedge wb_clock_in or posedge reset_in)
begin
    if (reset_in) // reset state machine ti S_IDLE state
        c_state <= #`FF_DELAY S_IDLE ;
    else
        c_state <= #`FF_DELAY n_state ;
end

// state machine logic
always@(c_state or
		ACK_I or 
		RTY_I or 
		ERR_I or 
		w_attempt or
		r_attempt or
		pci_error_sig_set_in or 
		retried or 
		pciw_fifo_control_in or 
		pciw_fifo_almost_empty_in or 
		pci_tar_cmd or 
		pci_tar_prefetch_en or 
		cache_line or
		rty_counter or
		pci_tar_read_request or
		pcir_fifo_almost_full_in or 
		rty_counter_max_value or 
		last_data_to_pcir_fifo or 
		first_wb_data_access or
		last_data_from_pciw_fifo
		)
begin
    case (c_state)
    S_IDLE:
        begin
        	// Default values for signals not used in this state
       		pcir_fifo_wenable_out <= 1'b0 ;
       		pcir_fifo_control_out <= 4'h0 ;
       		addr_count <= 1'b0 ;
       		cache_line_count <= 1'b0 ;
       		pci_error_sig_out <= 1'b0 ;
       		retried_d <= 1'b0 ;
       		last_data_transferred <= 1'b0 ;
       		wb_read_done <= 1'b0 ;
			write_rty_cnt_exp_out <= 1'b0 ;
			read_rty_cnt_exp_out <= 1'b0 ;
			wait_for_wb_response <= 1'b0 ;
			// If ERROR SIGNALED bit in pci_error_cs register is set, then state machine must be frozen
			case (pci_error_sig_set_in)
			1'b1 :
       		begin
        		pciw_fifo_renable_out <= 1'b0 ;
     			addr_into_cnt <= 1'b0 ;
       			cache_line_into_cnt <= 1'b0 ;
       			n_state <= S_IDLE ;
       		end
       		default :
       		begin
       			case ({w_attempt, r_attempt, retried})
       			3'b101 : // Write request for PCIW_FIFO to WB bus transaction
        		begin	 // If there was retry, the same transaction must be initiated
        			pciw_fifo_renable_out <= 1'b0 ; // the same data
        			addr_into_cnt <= 1'b0 ; // the same address
        			cache_line_into_cnt <= 1'b0 ; // no need for cache line when there is write
        			n_state <= S_WRITE ;
        		end
       			3'b100 : // Write request for PCIW_FIFO to WB bus transaction
				begin	 // If there is new transaction
        			pciw_fifo_renable_out <= 1'b1 ; // first location is address (in FIFO), next will be data
        			addr_into_cnt <= 1'b1 ; // address must be latched into address counter
        			cache_line_into_cnt <= 1'b0 ; // no need for cache line when there is write
        			n_state <= S_WRITE ;
        		end
       			3'b011 : // Read request from PCI Target for WB bus to PCIR_FIFO transaction
				begin	 // If there was retry, the same transaction must be initiated
	            	addr_into_cnt <= 1'b0 ; // the same address
	            	cache_line_into_cnt <= 1'b0 ; // cache line counter must not be changed for retried read
					pciw_fifo_renable_out <= 1'b0 ; // don't read from FIFO, when read transaction from WB to FIFO
        			n_state <= S_READ ;
				end
       			3'b010 : // Read request from PCI Target for WB bus to PCIR_FIFO transaction
				begin	 // If there is new transaction
	            	addr_into_cnt <= 1'b1 ; // address must be latched into counter from separate request bus
	            	cache_line_into_cnt <= 1'b1 ; // cache line size must be latched into its counter
					pciw_fifo_renable_out <= 1'b0 ; // don't read from FIFO, when read transaction from WB to FIFO
        			n_state <= S_READ ;
	            end
       			default : // stay in IDLE state
				begin
	       	   		pciw_fifo_renable_out <= 1'b0 ;
        			addr_into_cnt <= 1'b0 ;
        			cache_line_into_cnt <= 1'b0 ;
        			n_state <= S_IDLE ;
				end
       			endcase
			end
			endcase
			wb_stb_o <= 1'b0 ;
			wb_we_o <= 1'b0 ;
			wb_cyc_o <= 1'b0 ;
		end
	S_WRITE: // WRITE from PCIW_FIFO to WB bus
		begin
       		// Default values for signals not used in this state
       		pcir_fifo_wenable_out <= 1'b0 ;
       		pcir_fifo_control_out <= 4'h0 ;
       		addr_into_cnt <= 1'b0 ;
       		cache_line_into_cnt <= 1'b0 ;
       		cache_line_count <= 1'b0 ;
       		wb_read_done <= 1'b0 ;
			read_rty_cnt_exp_out <= 1'b0 ;
			case ({ACK_I, ERR_I, RTY_I})
			3'b100 : // If writting of one data is acknowledged
       		begin
       			pciw_fifo_renable_out <= 1'b1 ; // prepare next value (address when new trans., data when burst tran.)
       			addr_count <= 1'b1 ; // prepare next address if there will be burst
				pci_error_sig_out <= 1'b0 ; // there was no error
				retried_d <= 1'b0 ; // there was no retry
				write_rty_cnt_exp_out <= 1'b0 ; // there was no retry
				wait_for_wb_response <= 1'b0 ;
       			if (last_data_from_pciw_fifo) // if last data was transfered
				begin       			
       				n_state <= S_IDLE ;
       				last_data_transferred <= 1'b1 ; // signal for last data transfered
				end
       			else
				begin
       				n_state <= S_WRITE ;
       				last_data_transferred <= 1'b0 ;
				end
       		end
			3'b010 : // If writting of one data is terminated with ERROR
       		begin
       			pciw_fifo_renable_out <= 1'b1 ; // prepare next value (address when new trans., data when cleaning FIFO)
       			addr_count <= 1'b0 ; // no need for new address
       			retried_d <= 1'b0 ; // there was no retry
       			last_data_transferred <= 1'b1 ; // signal for last data transfered
       			pci_error_sig_out <= 1'b1 ; // segnal for error reporting
				write_rty_cnt_exp_out <= 1'b0 ; // there was no retry
				wait_for_wb_response <= 1'b0 ;
	   			if (last_data_from_pciw_fifo) // if last data was transfered
	   				n_state <= S_IDLE ; // go to S_IDLE for new transfere
	   			else // if there wasn't last data of transfere
	   				n_state <= S_WRITE_ERR_RTY ; // go here to clean this write transaction from PCIW_FIFO
       		end
			3'b001 : // If writting of one data is retried
			begin
				pciw_fifo_renable_out <= 1'b0 ;
				addr_count <= 1'b0 ;
				pci_error_sig_out <= 1'b0 ;
				last_data_transferred <= 1'b0 ;
				retried_d <= 1'b1 ; // there was a retry
				wait_for_wb_response <= 1'b0 ;
				if(rty_counter_max_value) // If retry counter reached maximum allowed value
				begin
					n_state <= S_WRITE_ERR_RTY ; // go here to clean this write transaction from PCIW_FIFO
					write_rty_cnt_exp_out <= 1'b1 ; // signal for reporting write counter expired
				end
				else
				begin
					n_state <= S_IDLE ; // go to S_IDLE state for retrying the transaction
					write_rty_cnt_exp_out <= 1'b0 ; // retry counter hasn't expired yet
				end
			end
			default :
			begin
				pciw_fifo_renable_out <= 1'b0 ;
				addr_count <= 1'b0 ;
				last_data_transferred <= 1'b0 ;
				retried_d <= 1'b0 ;
				wait_for_wb_response <= 1'b1 ; // wait for WB device to response (after 8 clocks RTY CNT is incremented)
				if(rty_counter_max_value) // when no WB response and RTY CNT reached maximum allowed value 
				begin
					n_state <= S_WRITE_ERR_RTY ; // go here to clean this write transaction from PCIW_FIFO
					write_rty_cnt_exp_out <= 1'b1 ; // signal for reporting write counter expired
    	   			pci_error_sig_out <= 1'b1 ; // signal for error reporting
				end
				else
				begin
					n_state <= S_WRITE ; // stay in S_WRITE state to wait WB to response
					write_rty_cnt_exp_out <= 1'b0 ; // retry counter hasn't expired yet
					pci_error_sig_out <= 1'b0 ;
				end
			end
			endcase
			wb_stb_o <= 1'b1 ;
			wb_we_o <= 1'b1 ;
			wb_cyc_o <= 1'b1 ;
		end
	S_WRITE_ERR_RTY: // Clean current write transaction from PCIW_FIFO if ERROR or Retry counter expired occures
		begin
   			pciw_fifo_renable_out <= 1'b1 ; // put out next data (untill last data or FIFO empty)
  			last_data_transferred <= 1'b1 ; // after exiting this state, negedge of this signal is used
  			// Default values for signals not used in this state
       		pcir_fifo_wenable_out <= 1'b0 ;
       		pcir_fifo_control_out <= 4'h0 ;
       		addr_into_cnt <= 1'b0 ;
       		cache_line_into_cnt <= 1'b0 ;
       		cache_line_count <= 1'b0 ;
       		addr_count <= 1'b0 ;
			pci_error_sig_out <= 1'b0 ;
			retried_d <= 1'b0 ;
   			wb_read_done <= 1'b0 ;
			write_rty_cnt_exp_out <= 1'b0 ;
			read_rty_cnt_exp_out <= 1'b0 ;
			wait_for_wb_response <= 1'b0 ;
			// If last data is cleaned out from PCIW_FIFO
   			if (last_data_from_pciw_fifo)
   				n_state <= S_IDLE ;
   			else
   				n_state <= S_WRITE_ERR_RTY ; // Clean until last data is cleaned out from FIFO
			wb_stb_o <= 1'b0 ;
			wb_we_o <= 1'b0 ;
			wb_cyc_o <= 1'b0 ;
		end
	S_READ: // READ from WB bus to PCIR_FIFO
		begin
  			// Default values for signals not used in this state
       		pciw_fifo_renable_out <= 1'b0 ;
       		addr_into_cnt <= 1'b0 ;
       		cache_line_into_cnt <= 1'b0 ;
       		pci_error_sig_out <= 1'b0 ;
			write_rty_cnt_exp_out <= 1'b0 ;
			case ({ACK_I, ERR_I, RTY_I})
			3'b100 : // If reading of one data is acknowledged
       		begin
       			pcir_fifo_wenable_out <= 1'b1 ; // enable writting data into PCIR_FIFO
       			addr_count <= 1'b1 ; // prepare next address if there will be burst
				cache_line_count <= 1'b1 ; // decrease counter value for cache line size
       			retried_d <= 1'b0 ; // there was no retry
				read_rty_cnt_exp_out <= 1'b0 ; // there was no retry
				wait_for_wb_response <= 1'b0 ;
				// if last data was transfered
       			if (last_data_to_pcir_fifo)
       			begin
       				pcir_fifo_control_out <= `LAST ; // control code in FIFO must indicate LAST data transfered
       				last_data_transferred <= 1'b1 ; // signal for last data transfered
       				wb_read_done <= 1'b1 ; // signal last data of read transaction for PCI Target
       				n_state <= S_TURN_ARROUND ;
       			end
       			else // if not last data transfered
       			begin
   					pcir_fifo_control_out <= 4'h0 ; // ZERO for control code
   					last_data_transferred <= 1'b0 ; // not last data transfered
   					wb_read_done <= 1'b0 ; // read is not done yet
   					n_state <= S_READ ;
   				end
       		end
       		3'b010 : // If reading of one data is terminated with ERROR
       		begin
       			pcir_fifo_wenable_out <= 1'b1 ; // enable for writting to FIFO data with ERROR
       			addr_count <= 1'b0 ; // no need for new address
   				pcir_fifo_control_out <= `DATA_ERROR ; // control code in FIFO must indicate the DATA with ERROR
   				last_data_transferred <= 1'b1 ; // signal for last data transfered
   				wb_read_done <= 1'b1 ; // signal last data of read transaction for PCI Target
  				cache_line_count <= 1'b0 ; // no need for cache line, when error occures
   				n_state <= S_TURN_ARROUND ;
   				retried_d <= 1'b0 ; // there was no retry
				wait_for_wb_response <= 1'b0 ;
				read_rty_cnt_exp_out <= 1'b0 ; // there was no retry
       		end
			3'b001 : // If reading of one data is retried
       		begin
				pcir_fifo_wenable_out <= 1'b0 ;
				pcir_fifo_control_out <= 4'h0 ;
				addr_count <= 1'b0 ;
				cache_line_count <= 1'b0 ;
				wait_for_wb_response <= 1'b0 ;
				case ({first_wb_data_access, rty_counter_max_value})
				2'b10 :
				begin
					n_state <= S_IDLE ; // go to S_IDLE state for retrying the transaction
					read_rty_cnt_exp_out <= 1'b0 ; // retry counter hasn't expired yet   
					last_data_transferred <= 1'b0 ;
					wb_read_done <= 1'b0 ;
					retried_d <= 1'b1 ; // there was a retry
				end
				2'b11 :
				begin
					n_state <= S_READ_RTY ; // go here to wait for PCI Target to remove read request
					read_rty_cnt_exp_out <= 1'b1 ; // signal for reporting read counter expired  
					last_data_transferred <= 1'b0 ;
					wb_read_done <= 1'b0 ;
					retried_d <= 1'b1 ; // there was a retry
				end
				default :
				begin
					n_state <= S_TURN_ARROUND ; // go to S_TURN_ARROUND state 
					read_rty_cnt_exp_out <= 1'b0 ; // retry counter hasn't expired  
					last_data_transferred <= 1'b1 ;
					wb_read_done <= 1'b1 ;
					retried_d <= 1'b0 ; // retry must not be retried, since there is not a first data
				end
				endcase
			end
      		default :
       		begin
				addr_count <= 1'b0 ;
				cache_line_count <= 1'b0 ;
				read_rty_cnt_exp_out <= 1'b0 ;
				retried_d <= 1'b0 ;
				wait_for_wb_response <= 1'b1 ; // wait for WB device to response (after 8 clocks RTY CNT is incremented)
				if(rty_counter_max_value) // when no WB response and RTY CNT reached maximum allowed value 
				begin
					n_state <= S_TURN_ARROUND ; // go here to stop read request
					pcir_fifo_wenable_out <= 1'b1 ;
					pcir_fifo_control_out <= `DATA_ERROR ;
					last_data_transferred <= 1'b1 ;
					wb_read_done <= 1'b1 ;
				end
				else
				begin
					n_state <= S_READ ; // stay in S_READ state to wait WB to response
					pcir_fifo_wenable_out <= 1'b0 ;
					pcir_fifo_control_out <= 4'h0 ;
					last_data_transferred <= 1'b0 ;
					wb_read_done <= 1'b0 ;
				end
			end
			endcase
			wb_stb_o <= 1'b1 ;
			wb_we_o <= 1'b0 ;
			wb_cyc_o <= 1'b1 ;
		end
	S_READ_RTY: // Wait for PCI Target to remove read request, when retry counter reaches maximum value!
		begin
  			// Default values for signals not used in this state
   			pciw_fifo_renable_out <= 1'b0 ;
       		pcir_fifo_wenable_out <= 1'b0 ;
       		pcir_fifo_control_out <= 4'h0 ;
       		addr_into_cnt <= 1'b0 ;
       		cache_line_into_cnt <= 1'b0 ;
       		cache_line_count <= 1'b0 ;
       		addr_count <= 1'b0 ;
			pci_error_sig_out <= 1'b0 ;
			retried_d <= 1'b0 ;
   			wb_read_done <= 1'b0 ;
			write_rty_cnt_exp_out <= 1'b0 ;
			read_rty_cnt_exp_out <= 1'b0 ;
			wait_for_wb_response <= 1'b0 ;
			// wait for PCI Target to remove read request
			if (pci_tar_read_request)
			begin
				n_state <= S_READ_RTY ; // stay in this state until read request is removed
	   			last_data_transferred <= 1'b0 ;
			end
			else // when read request is removed
			begin
				n_state <= S_IDLE ;
	   			last_data_transferred <= 1'b1 ; // when read request is removed, there is "last" data
			end
			wb_stb_o <= 1'b0 ;
			wb_we_o <= 1'b0 ;
			wb_cyc_o <= 1'b0 ;
		end		
	S_TURN_ARROUND: // Turn arround cycle after writting to PCIR_FIFO (for correct data when reading from PCIW_FIFO) 
		begin
  			// Default values for signals not used in this state
   			pciw_fifo_renable_out <= 1'b0 ;
       		pcir_fifo_wenable_out <= 1'b0 ;
       		pcir_fifo_control_out <= 4'h0 ;
       		addr_into_cnt <= 1'b0 ;
       		cache_line_into_cnt <= 1'b0 ;
       		cache_line_count <= 1'b0 ;
       		addr_count <= 1'b0 ;
			pci_error_sig_out <= 1'b0 ;
			retried_d <= 1'b0 ;
   			last_data_transferred <= 1'b1 ;
   			wb_read_done <= 1'b0 ;
			write_rty_cnt_exp_out <= 1'b0 ;
			read_rty_cnt_exp_out <= 1'b0 ;
			wait_for_wb_response <= 1'b0 ;
			n_state <= S_IDLE ;
			wb_stb_o <= 1'b0 ;
			wb_we_o <= 1'b0 ;
			wb_cyc_o <= 1'b0 ;
		end		
	default	: 
		begin
  			// Default values for signals not used in this state
   			pciw_fifo_renable_out <= 1'b0 ;
       		pcir_fifo_wenable_out <= 1'b0 ;
       		pcir_fifo_control_out <= 4'h0 ;
       		addr_into_cnt <= 1'b0 ;
       		cache_line_into_cnt <= 1'b0 ;
       		cache_line_count <= 1'b0 ;
       		addr_count <= 1'b0 ;
			pci_error_sig_out <= 1'b0 ;
			retried_d <= 1'b0 ;
   			last_data_transferred <= 1'b0 ;
   			wb_read_done <= 1'b0 ;
			write_rty_cnt_exp_out <= 1'b0 ;
			read_rty_cnt_exp_out <= 1'b0 ;
			wait_for_wb_response <= 1'b0 ;
			n_state <= S_IDLE ;
			wb_stb_o <= 1'b0 ;
			wb_we_o <= 1'b0 ;
			wb_cyc_o <= 1'b0 ;
		end
	endcase
end

// Signal for retry monitor in state machine when there is read and first (or single) data access
wire ack_rty_response = ACK_I || RTY_I ;

// clock enable for first_wb_data_access signal when NO WB cycle present OR there was acknowledge or retry termination
wire wb_access_clk_en = ~wb_cyc_o || ack_rty_response ;
// data written into first_wb_data_access signal register is ONE when no WB cycle present OR ZERO durin WB cycle (when
//   acknowledge or retry termination occures)
wire wb_access_data = ~wb_cyc_o ? 1'b1 : 1'b0 ;
// Signal first_wb_data_access is set when no WB cycle present till end of first data access of WB cycle on WB bus
always@(posedge wb_clock_in or posedge reset_in)
begin
	if (reset_in)
		first_wb_data_access = 1'b1 ;
	else
	begin
		if (wb_access_clk_en)
			first_wb_data_access = wb_access_data ;
	end
end


// Signals to FIFO
assign	pcir_fifo_be_out = pci_tar_be ;
assign	pcir_fifo_data_out = MDATA_I ;

// OUTPUT signals
assign	pci_error_bc = bc_register ;

assign	STB_O = wb_stb_o ;
assign	WE_O = wb_we_o ;
assign	CYC_O = wb_cyc_o ;
assign	CAB_O = wb_cyc_o & burst_transfer ;
assign	ADR_O = addr_cnt_out ;
assign	MDATA_O = pciw_fifo_addr_data_in ;

always@(pciw_fifo_cbe_in or pci_tar_be or wb_we_o)
begin
	if (wb_we_o)
		SEL_O = ~pciw_fifo_cbe_in ;
	else
		SEL_O = ~pci_tar_be ;
end


endmodule

