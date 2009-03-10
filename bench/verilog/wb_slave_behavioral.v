//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name: wb_slave_behavioral.v                            ////
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
// Revision 1.1  2002/02/01 13:39:43  mihad
// Initial testbench import. Still under development
//
//

`include "pci_testbench_defines.v"
`include "timescale.v"
`include "pci_constants.v"
module WB_SLAVE_BEHAVIORAL
(
	CLK_I,
	RST_I,
	ACK_O,
	ADR_I,
	CYC_I,
	DAT_O,
	DAT_I,
	ERR_O,
	RTY_O,
	SEL_I,
	STB_I,
	WE_I,
	CAB_I
);

/*----------------------------------------------------------------------------------------------------------------------
WISHBONE signals
----------------------------------------------------------------------------------------------------------------------*/
input					CLK_I ;
input					RST_I ;
output					ACK_O ;
input	`WB_ADDR_TYPE	ADR_I ;
input					CYC_I ;
output	`WB_DATA_TYPE	DAT_O ;
input	`WB_DATA_TYPE	DAT_I ;
output					ERR_O ;
output					RTY_O ;
input	`WB_SEL_TYPE	SEL_I ;
input					STB_I ;
input					WE_I ;
input					CAB_I ;

//reg				ACK_O ;
//reg				ERR_O ;
//reg				RTY_O ;
reg		[31:0]	DAT_O ;

/*----------------------------------------------------------------------------------------------------------------------
Asynchronous dual-port RAM signals for storing and fetching the data
----------------------------------------------------------------------------------------------------------------------*/
reg 	[31:0] 	wb_memory [0:1023] ; // data for WB memory - 16 LSB addresses are connected
reg		[31:0]	mem_wr_data_out ;
reg		[31:0]	mem_rd_data_in ;

/*----------------------------------------------------------------------------------------------------------------------
Maximum values for WAIT and RETRY counters and which response !!!
----------------------------------------------------------------------------------------------------------------------*/
reg		[2:0]	a_e_r_resp ; 		// tells with which cycle_termination_signal must wb_slave respond !
reg				wait_cyc ;
reg		[7:0]	max_retry ;

// assign registers to default state while in reset
always@(RST_I)
begin
    if (RST_I)
    begin
        a_e_r_resp	<= 3'b000 ; // do not respond
    	wait_cyc	<= 1'b0 ;	// no wait cycles
        max_retry	<= 8'h0 ;	// no retries
    end
end //reset

task cycle_response ;
	input [2:0]		ack_err_rty_resp ;	// acknowledge, error or retry response input flags
	input 			wait_cycles ;		// if wait cycles before each data termination cycle (ack, err or rty)
	input [7:0]		retry_cycles ;		// noumber of retry cycles before acknowledge cycle
begin
    // assign values
    a_e_r_resp	<= #1 ack_err_rty_resp ;
	wait_cyc	<= #1 wait_cycles ;
    max_retry	<= #1 retry_cycles ;
end
endtask // cycle_response

/*----------------------------------------------------------------------------------------------------------------------
Internal signals and logic
----------------------------------------------------------------------------------------------------------------------*/
reg				calc_ack ;
reg				calc_err ;
reg				calc_rty ;

reg		[7:0]	retry_cnt ;
reg		[7:0]	retry_num ;
reg				retry_expired ;
reg				retry_rst ;

// RESET retry counter
always@(posedge RST_I or posedge CLK_I)
begin
	if (RST_I)
		retry_rst <= 1'b1 ;
	else
		retry_rst <= calc_ack || calc_err ;
end

// Retry counter
always@(posedge retry_rst or negedge calc_rty)
begin
	if (retry_rst)
		retry_cnt <= #`FF_DELAY 8'h00 ;
	else
		retry_cnt <= #`FF_DELAY retry_num ;
end
always@(retry_cnt or max_retry)
begin
	if (retry_cnt < max_retry)
	begin
		retry_num = retry_cnt + 1'b1 ;
		retry_expired = #10 1'b0 ;
	end
	else
	begin
		retry_num = retry_cnt ;
		retry_expired = #10 1'b1 ;
	end
end

reg		[1:0]	wait_cnt ;
reg		[1:0]	wait_num ;
reg				wait_expired ;
reg				reset_wait ;

always@(posedge RST_I or posedge CLK_I)
begin
	if (RST_I)
		reset_wait <= #`FF_DELAY 1'b1 ;
	else
		reset_wait <= #`FF_DELAY (wait_expired || ~STB_I) ;
end

// Wait counter
always@(posedge reset_wait or posedge CLK_I)
begin
	if (reset_wait)
		wait_cnt <= #`FF_DELAY 4'h0 ;
	else
		wait_cnt <= #`FF_DELAY wait_num ;
end
always@(wait_cnt or wait_cyc or STB_I or a_e_r_resp or retry_expired)
begin
	if ((wait_cyc) && (STB_I))
	begin
		if (wait_cnt < 2'h2)
		begin
			wait_num = wait_cnt + 1'b1 ;
			wait_expired = 1'b0 ;
			calc_ack = 1'b0 ;
			calc_err = 1'b0 ;
			calc_rty = 1'b0 ;
		end
		else
		begin
			wait_num = wait_cnt ;
			wait_expired = 1'b1 ;
			if (a_e_r_resp == 3'b100)
			begin
				calc_ack = 1'b1 ;
				calc_err = 1'b0 ;
				calc_rty = 1'b0 ;
			end
			else
			if (a_e_r_resp == 3'b010)
			begin
				calc_ack = 1'b0 ;
				calc_err = 1'b1 ;
				calc_rty = 1'b0 ;
			end
			else
			if (a_e_r_resp == 3'b001)
			begin
				calc_err = 1'b0 ;
				if (retry_expired)
				begin
					calc_ack = 1'b1 ;
					calc_rty = 1'b0 ;
				end
				else
				begin
					calc_ack = 1'b0 ;
					calc_rty = 1'b1 ;
				end
			end
			else
			begin
				calc_ack = 1'b0 ;
				calc_err = 1'b0 ;
				calc_rty = 1'b0 ;
			end
		end
	end
	else
	if ((~wait_cyc) && (STB_I))
	begin
		wait_num = 2'h0 ;
		wait_expired = 1'b1 ;
		if (a_e_r_resp == 3'b100)
		begin
			calc_ack = 1'b1 ;
			calc_err = 1'b0 ;
			calc_rty = 1'b0 ;
		end
		else
		if (a_e_r_resp == 3'b010)
		begin
			calc_ack = 1'b0 ;
			calc_err = 1'b1 ;
			calc_rty = 1'b0 ;
		end
		else
		if (a_e_r_resp == 3'b001)
		begin
			calc_err = 1'b0 ;
			if (retry_expired)
			begin
				calc_ack = 1'b1 ;
				calc_rty = 1'b0 ;
			end
			else
			begin
				calc_ack = 1'b0 ;
				calc_rty = 1'b1 ;
			end
		end
		else
		begin
			calc_ack = 1'b0 ;
			calc_err = 1'b0 ;
			calc_rty = 1'b0 ;
		end
	end
	else
	begin
		wait_num = 2'h0 ;
		wait_expired = 1'b0 ;
		calc_ack = 1'b0 ;
		calc_err = 1'b0 ;
		calc_rty = 1'b0 ;
	end
end

wire	rd_sel = (CYC_I && STB_I && ~WE_I) ;
wire	wr_sel = (CYC_I && STB_I && WE_I) ;

// Generate cycle termination signals
assign	ACK_O = calc_ack && STB_I ;
assign	ERR_O = calc_err && STB_I ;
assign	RTY_O = calc_rty && STB_I ;

// Assign address to asynchronous memory
always@(ADR_I or RST_I)
begin
	if (RST_I) // this is added because at start of test bench we need address change in order to get data!
		mem_rd_data_in = 32'hxxxx_xxxx ;
	else
		mem_rd_data_in = wb_memory[ADR_I[11:2]] ;
end

// assign outputs to unknown state while in reset
always@(RST_I)
begin
    if (RST_I)
    begin
		DAT_O <= 32'hxxxx_xxxx ;
    end
end //reset

// Data input/output interface
always@(rd_sel or wr_sel or mem_rd_data_in or DAT_I or SEL_I or mem_wr_data_out)
begin
	if (rd_sel)
	begin
		DAT_O = mem_rd_data_in ;
	end
end
always@(posedge CLK_I)
begin
    if (wr_sel)
    begin
        mem_wr_data_out         = wb_memory[ADR_I[11:2]] ;

        if ( SEL_I[3] )
            mem_wr_data_out[31:24] = DAT_I[31:24] ;

        if ( SEL_I[2] )
            mem_wr_data_out[23:16] = DAT_I[23:16] ;

        if ( SEL_I[1] )
            mem_wr_data_out[15: 8] = DAT_I[15: 8] ;

        if ( SEL_I[0] )
            mem_wr_data_out[ 7: 0] = DAT_I[ 7: 0] ;

        wb_memory[ADR_I[11:2]] <= mem_wr_data_out ;
    end
end

endmodule
