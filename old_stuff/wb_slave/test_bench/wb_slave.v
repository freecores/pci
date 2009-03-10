//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "wb_slave.v"                                      ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - Miha Dolenc (mihad@opencores.org)                     ////
////                                                              ////
////  All additional information is avaliable in the README.pdf   ////
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
// Revision 1.6  2001/07/30 15:24:17  mihad
// Updated selects checking on WISHBONE bus, so it does not pass any
// requests to pci which conflict with pci specification.
//
//

`define FSM_BITS 4 // number of bits needed for FSM states

// WISHBONE data output selections encoding
`include "bus_commands.v"
`include "constants.v"

module WB_SLAVE(    wb_clock_in,
                    reset_in,
                    wb_hit_in, 
                    wb_conf_hit_in,
                    wb_map_in,
                    wb_pref_en_in,
                    wb_mrl_en_in,
                    wb_addr_in,
                    del_bc_in,  
                    wb_del_req_pending_in,
                    wb_del_comp_pending_in,
                    pci_drcomp_pending_in,
                    del_bc_out,
                    del_req_out,
                    del_done_out,
                   	del_burst_out,
                    del_write_out,
                    del_write_in,
                    del_error_in,
                    del_in_progress_out,
                    ccyc_addr_in,
                    wb_del_addr_in,
                    wb_del_be_in,
                    wb_conf_offset_out,
                    wb_conf_renable_out,
                    wb_conf_wenable_out,
                    wb_conf_be_out,
                    wb_conf_data_in,
                    wb_conf_data_out,
                    wb_data_out,
                    wb_cbe_out,    
                    wbw_fifo_wenable_out,
                    wbw_fifo_control_out,
                    wbw_fifo_almost_full_in,
                    wbw_fifo_full_in,
                    wbr_fifo_renable_out, 
                    wbr_fifo_be_in, 
                    wbr_fifo_data_in,
                    wbr_fifo_control_in,
                    wbr_fifo_flush_out, 
                    wbr_fifo_almost_empty_in, 
                    wbr_fifo_empty_in,
                    pciw_fifo_empty_in,
                    wbs_lock_in, 
                    CYC_I,
                    STB_I,
                    WE_I,
                    SEL_I,
                    SDATA_I,
                    SDATA_O,
                    ACK_O,
                    RTY_O,
                    ERR_O,
                    CAB_I
                );

/*----------------------------------------------------------------------------------------------------------------------
Various parameters needed for state machine and other stuff
----------------------------------------------------------------------------------------------------------------------*/
parameter WBR_SEL  = 1'b0 ;
parameter CONF_SEL = 1'b1 ;

parameter S_IDLE         = `FSM_BITS'h0 ; 
parameter S_W_ADDR_DATA  = `FSM_BITS'h1 ;
parameter S_TURN_ARROUND = `FSM_BITS'h2 ;
parameter S_READ         = `FSM_BITS'h3 ;
parameter S_CONF_WRITE   = `FSM_BITS'h4 ;
parameter S_CONF_READ    = `FSM_BITS'h5 ;

/*----------------------------------------------------------------------------------------------------------------------
System signals inputs
wb_clock_in - WISHBONE bus clock input
reset_in    - system reset input controlled by bridge's reset logic
----------------------------------------------------------------------------------------------------------------------*/
input wb_clock_in, reset_in ;

/*----------------------------------------------------------------------------------------------------------------------
Inputs from address decoding logic
wb_hit_in - Decoder logic indicates if address is in a range of one of images
wb_conf_hit_in - Decoder logic indicates that address is in configuration space range
wb_map_in   - Decoder logic provides information about image mapping - memory mapped image   - wb_map_in = 0
                                                                       IO space mapped image - wb_map_in = 1 
wb_pref_en_in - Prefetch enable signal from currently selected image - used for PCI bus command usage
wb_addr_in - Address already transalted from WB bus to PCI bus input
wb_mrl_en_in - Memory read line enable input for each image
----------------------------------------------------------------------------------------------------------------------*/
input [4:0]     wb_hit_in ;         // hit indicators
input           wb_conf_hit_in ;    // configuration hit indicator
input [4:0]     wb_pref_en_in ;     // prefetch enable from all images
input [4:0]     wb_mrl_en_in ;      // Memory Read line command enable from images 
input [4:0]     wb_map_in ;         // address space mapping indicators - 1 memory space mapping, 0-IO space mapping
input [31:0]    wb_addr_in ;        // Translated address input

/*----------------------------------------------------------------------------------------------------------------------
Delayed transaction control inputs and outputs:
Used for locking particular accesses when delayed transactions are in progress:
wb_del_addr_in - delayed transaction address input - when completion is ready it's used for transaction decoding
wb_del_be_in   - delayed transaction byte enable input - when completion is ready it's used for transaction decoding
----------------------------------------------------------------------------------------------------------------------*/
input  [31:0] wb_del_addr_in ;
input  [3:0]  wb_del_be_in ;

input [3:0] del_bc_in ;           // delayed request bus command used
input       wb_del_req_pending_in ;   // delayed request pending indicator
input       wb_del_comp_pending_in ;  // delayed completion pending indicator
input       pci_drcomp_pending_in ; // PCI initiated delayed read completion pending 

output [3:0] del_bc_out ; // delayed transaction bus command output

output del_req_out ; // output for issuing delayed transaction requests

output del_done_out ; // output indicating current delayed completion finished on WISHBONE bus

output del_burst_out ; // delayed burst transaction indicator

output del_in_progress_out ; // delayed in progress indicator - since delayed transaction can be a burst transaction, progress indicator must be used for proper operation

output del_write_out ;   // write enable for delayed transaction - used for indicating that transaction is a write

input  del_write_in ;    // indicates that current delayed completion is from a write request
input  del_error_in ;    // indicate that delayed request terminated with an error - used for write requests

input  [31:0] ccyc_addr_in ; // configuration cycle address input - it's separate from other addresses, since it is stored separately and decoded for type 0 configuration access

/*----------------------------------------------------------------------------------------------------------------------
Configuration space access control and data signals
wb_conf_offset_out  - lower 12 bits of address input provided for register offset
wb_conf_renable     - read enable signal for configuration space accesses
wb_conf_wenable     - write enable signal for configuration space accesses
wb_conf_be_out      - byte enable signals for configuration space accesses
wb_conf_data_in     - data from configuration space
wb_conf_data_in     - data provided for configuration space
----------------------------------------------------------------------------------------------------------------------*/
output [11:0]   wb_conf_offset_out ;  // register offset output
output          wb_conf_renable_out,  // configuration read and write enable outputs
                wb_conf_wenable_out ; 
output [3:0]    wb_conf_be_out ;      // byte enable outputs for configuration space
input  [31:0]   wb_conf_data_in ;     // configuration data input from configuration space
output [31:0]   wb_conf_data_out ;    // configuration data output for configuration space

/*----------------------------------------------------------------------------------------------------------------------
Data from WISHBONE bus output to interiror of the core:
Data output is used for normal and configuration accesses.
---------------------------------------------------------------------------------------------------------------------*/
output [31:0] wb_data_out ;

/*----------------------------------------------------------------------------------------------------------------------
Bus command - byte enable output - during address phase of image access this bus holds information about PCI
bus command that should be used, during dataphases ( configuration or image access ) this bus contains inverted
SEL_I signals
---------------------------------------------------------------------------------------------------------------------*/
output [3:0] wb_cbe_out ;

/*----------------------------------------------------------------------------------------------------------------------
WBW_FIFO control signals used for sinking data into WBW_FIFO and status monitoring
---------------------------------------------------------------------------------------------------------------------*/
output       wbw_fifo_wenable_out ;    // write enable for WBW_FIFO output
output [3:0] wbw_fifo_control_out ;    // control bus output for WBW_FIFO
input        wbw_fifo_almost_full_in ; // almost full status indicator from WBW_FIFO
input        wbw_fifo_full_in ;        // full status indicator from WBW_FIFO

/*----------------------------------------------------------------------------------------------------------------------
WBR_FIFO control signals used for fetching data from WBR_FIFO and status monitoring
---------------------------------------------------------------------------------------------------------------------*/
output          wbr_fifo_renable_out ;      // WBR_FIFO read enable output
input   [3:0]   wbr_fifo_be_in ;            // byte enable input from WBR_FIFO
input   [31:0]  wbr_fifo_data_in ;          // data input from WBR_FIFO
input   [3:0]   wbr_fifo_control_in ;       // control bus input from WBR_FIFO
output          wbr_fifo_flush_out ;        // flush signal for WBR_FIFO
input           wbr_fifo_almost_empty_in ;  // almost empty status indicator from WBR_FIFO
input           wbr_fifo_empty_in ;         // empty status indicator from WBR_FIFO

// used for transaction ordering requirements - WISHBONE read cannot complete until writes from PCI are completed
input           pciw_fifo_empty_in ;        // empty status indicator from PCIW_FIFO

/*----------------------------------------------------------------------------------------------------------------------
wbs_lock_in - internal signal - when error reporting is enabled and WISHBONE master detects an error while completing
posted write on PCI, then WISHBONE slave unit doesn't accept any new requests or posted writes. Delayed completions
are allowed to complete on WISHBONE if all other requirements are satisfied also. 
---------------------------------------------------------------------------------------------------------------------*/
input           wbs_lock_in ;

/*----------------------------------------------------------------------------------------------------------------------
WISHBONE bus interface signals - can be connected directly to WISHBONE bus
---------------------------------------------------------------------------------------------------------------------*/
input           CYC_I ;     // cycle indicator
input           STB_I ;     // strobe input - input data is valid when strobe and cycle indicator are high
input           WE_I  ;     // write enable input - 1 - write operation, 0 - read operation
input   [3:0]   SEL_I ;     // Byte select inputs
input   [31:0]  SDATA_I ;   // WISHBONE slave interface input data bus
output  [31:0]  SDATA_O ;   // WISHBONE slave interface output data bus
output          ACK_O ;     // Acknowledge output - qualifies valid data on data output bus or received data on data input bus
output          RTY_O ;     // retry output - signals to WISHBONE master that cycle should be terminated and retried later
output          ERR_O ;     // Signals to WISHBONE master that access resulted in an error
input           CAB_I ;     // consecutive address burst input - indicates that master will do a serial address transfer in current cycle

reg [(`FSM_BITS - 1):0]  c_state ; //current state register
reg [(`FSM_BITS - 1):0]  n_state ; //next state input to current state register

// state machine register control
always@(posedge wb_clock_in or posedge reset_in)
begin
    if (reset_in)
        c_state <= #`FF_DELAY S_IDLE ;
    else
        c_state <= #`FF_DELAY n_state ;
end


// write operation indicator for delayed transaction requests
assign del_write_out = WE_I ;

// variable for bus command multiplexer logic output for delayed requests
reg [3:0] del_bc ;

//register for intermediate data and select storage
reg [35:0] d_incoming ;

// enable for incoming data register
reg d_incoming_ena ;

// incoming data register control logic
always@(posedge wb_clock_in or posedge reset_in)
begin
	if (reset_in)
		d_incoming <= #`FF_DELAY {35{1'b0}} ;
	else if (d_incoming_ena)
		d_incoming <= #`FF_DELAY {SEL_I, SDATA_I} ;
end

/*===================================================================================================================================================================================
Write allow for image accesses. Writes through images are allowed when all of following are true:
- WBW_FIFO musn't be almost full nor full for image writes to be allowed - Every transaction takes at least two locations in the FIFO
- delayed read from from WISHBONE to PCI request musn't be present
- delayed read from PCI to WISHBONE completion musn't be present
- lock input musn't be set - it can be set because of error reporting or because PCI master state machine is disabled
===================================================================================================================================================================================*/
wire img_wallow           = ~|{ wbw_fifo_almost_full_in , wbw_fifo_full_in, (wb_del_req_pending_in && ~del_write_in) , pci_drcomp_pending_in, wbs_lock_in } ;

/*===================================================================================================================================================================================
WISHBONE slave can request an image read accesses when all of following are true:
- delayed completion is not present
- delayed request is not present
- operation is not locked because of error reporting mechanism or because PCI master is disabled
===================================================================================================================================================================================*/
wire do_dread_request     = ~|{ wb_del_req_pending_in, wb_del_comp_pending_in, wbs_lock_in } ;

/*===================================================================================================================================================================================
WISHBONE slave can complete an image read accesses when all of following are true:
- delayed read completion is present
- delayed read completion is the same as current read access ( dread_completion_hit is 1 )
- PCI Write FIFO is empty - no posted write is waiting to be finished in PCIW_FIFO
- WBR_FIFO empty status is active
===================================================================================================================================================================================*/
wire select_and_bc_hit = ( SEL_I == wb_del_be_in ) && ( del_bc == del_bc_in ) ;
wire dread_completion_hit = ( wb_del_addr_in == wb_addr_in ) && select_and_bc_hit && ~del_write_in ;
wire do_dread_completion  = wb_del_comp_pending_in && pciw_fifo_empty_in && dread_completion_hit && ~wbr_fifo_empty_in ;

`ifdef GUEST

    // wires indicating allowance for configuration cycle generation requests
    wire do_ccyc_req  = 1'b0 ;
    wire do_ccyc_comp = 1'b0 ;
    
    // wires indicating allowance for interrupt acknowledge cycle generation requests
    wire do_iack_req  = 1'b0 ;
    wire do_iack_comp = 1'b0 ;

    // variables for configuration access control signals
    reg conf_wenable ;
    assign wb_conf_wenable_out = 1'b0 ;

    // configuration cycle data register hit
    wire ccyc_hit = 1'b0 ;
    wire iack_hit = 1'b0 ;

`else
`ifdef HOST
    // only host implementation has access for generating interrupt acknowledge and configuration cycles

    // wires indicating allowance for configuration cycle generation requests
    wire do_ccyc_req  = ~|{ wb_del_comp_pending_in, wb_del_req_pending_in, wbs_lock_in } ;
    wire do_ccyc_comp = wb_del_comp_pending_in && select_and_bc_hit && ((pciw_fifo_empty_in && ~del_write_in && ~wbr_fifo_empty_in) || del_write_in) ;
    
    // wires indicating allowance for interrupt acknowledge cycle generation requests
    wire do_iack_req  = ~|{ wb_del_comp_pending_in, wb_del_req_pending_in, wbs_lock_in } ;
    wire do_iack_comp = wb_del_comp_pending_in && select_and_bc_hit && pciw_fifo_empty_in && ~del_write_in && ~wbr_fifo_empty_in;

    // variables for configuration access control signals
    reg conf_wenable ;
    assign wb_conf_wenable_out = conf_wenable ;

    // configuration cycle data register hit
    wire ccyc_hit = ({wb_addr_in[11:2], 2'b00} == `CNF_DATA_ADDR) ;
    wire iack_hit = ({wb_addr_in[11:2], 2'b00} == `INT_ACK_ADDR) ;
`endif
`endif

// configuration read enable - supplied for host and guest bridges
reg conf_renable ;
assign wb_conf_renable_out = conf_renable ;

// wire for write attempt - 1 when external WB master is attempting a write
wire wattempt = ( CYC_I && STB_I && WE_I ) ; // write is qualified when cycle, strobe and write enable inputs are all high

// wire for read attempt - 1 when external WB master is attempting a read
wire rattempt = ( CYC_I && STB_I && ~WE_I ) ; // read is qualified when cycle and strobe are high and write enable is low

// burst access indicator
wire burst_transfer = CYC_I && CAB_I ;

// address allignement indicator
wire alligned_address = wb_addr_in[1:0] == 2'b00 ;

// SEL_I error indicator for IO and configuration accesses - select lines must be alligned with address
reg sel_error ;
always@(wb_addr_in or SEL_I)
begin
    case (wb_addr_in[1:0]) 
        2'b00: sel_error <= ~SEL_I[0] ; // select 0 must be 1, all others are don't cares.
        2'b01: sel_error <= ~SEL_I[1] || SEL_I[0]  ; // byte 0 can't be selected, byte 1 must be selected
        2'b10: sel_error <= ~SEL_I[2] || SEL_I[1] || SEL_I[0] ; // bytes 0 and 1 can't be selected, byte 2 must be selected
        2'b11: sel_error <= ~SEL_I[3] || SEL_I[2] || SEL_I[1] || SEL_I[0] ; // bytes 0, 1 and 2 can't be selected, byte 3 must be selected
    endcase
end

// WBW_FIFO control output
reg [3:0] wbw_fifo_control ;
assign wbw_fifo_control_out = wbw_fifo_control ; //control bus output for WBW_FIFO

// WBW_FIFO wenable output assignment
reg wbw_fifo_wenable ;
assign wbw_fifo_wenable_out = wbw_fifo_wenable ; //write enable for WBW_FIFO

// WBR_FIFO control outputs
reg wbr_fifo_flush, wbr_fifo_renable ; // flush and read enable outputs
assign wbr_fifo_renable_out = wbr_fifo_renable ; //read enable for wbr_fifo
assign wbr_fifo_flush_out   = wbr_fifo_flush ; // flush for wbr_fifo

// delayed transaction request control signals
reg del_req, del_done ;
assign del_req_out  = del_req ; // read request
assign del_done_out = del_done ; // read done
                    
// WISHBONE handshaking control outputs
reg ack, rty, err ;
assign ACK_O = ack ;
assign RTY_O = rty ;
assign ERR_O = err ;

/*----------------------------------------------------------------------------------------------------------------------
Control logic for image hits
img_hit - state of wb_hit_in bus when first data is acknowledged
same_hit - comparator output that compares first data phase's wb_hit_input and current wb_hit_input
---------------------------------------------------------------------------------------------------------------------*/
reg [4:0] img_hit ;

wire hit_latch_en = ( c_state == S_IDLE ) && ack ;

always@(posedge wb_clock_in or posedge reset_in)
begin
    if (reset_in)
        img_hit <= #`FF_DELAY 5'h00 ;
    else
        if (hit_latch_en)
            img_hit <= #`FF_DELAY wb_hit_in ;
end

wire same_hit  = ( img_hit == wb_hit_in ) ;

/*----------------------------------------------------------------------------------------------------------------------
Control logic for image control signals
pref_en - prefetch enable of currently selected image
mrl_en  - Memory read line enable of currently selected image
map     - Address space mapping for currently selected image
---------------------------------------------------------------------------------------------------------------------*/
reg pref_en, mrl_en, map ;

// hit error indicator
reg hit_error ;

always@(wb_hit_in or wb_pref_en_in or wb_mrl_en_in or wb_map_in)
begin
    case (wb_hit_in)
        5'h01:  begin
                    pref_en   <= wb_pref_en_in[0] ;
                    mrl_en    <= wb_mrl_en_in[0] ;
                    map       <= wb_map_in[0] ;
                    hit_error <= 1'b0 ;
                end
        
        5'h02:  begin
                   pref_en   <= wb_pref_en_in[1] ;
                   mrl_en    <= wb_mrl_en_in[1] ;
                   map       <= wb_map_in[1] ;
                   hit_error <= 1'b0 ;
                end

        5'h04:  begin
                    pref_en   <= wb_pref_en_in[2] ;
                    mrl_en    <= wb_mrl_en_in[2] ;
                    map       <= wb_map_in[2] ;
                    hit_error <= 1'b0 ;
                end
    
        5'h08:  begin
                    pref_en   <= wb_pref_en_in[3] ;
                    mrl_en    <= wb_mrl_en_in[3] ;
                    map       <= wb_map_in[3] ;
                    hit_error <= 1'b0 ;
                end

        5'h10:  begin
                    pref_en   <= wb_pref_en_in[4] ;
                    mrl_en    <= wb_mrl_en_in[4] ;
                    map       <= wb_map_in[4] ;
                    hit_error <= 1'b0 ;
                end

        default:begin
                    pref_en <= 1'b0 ;
                    mrl_en  <= 1'b0 ;
                    map     <= 1'b0 ;
                    hit_error <= |(wb_hit_in) ;
                end
    endcase     
end

assign del_burst_out = CAB_I && pref_en && ~WE_I; // delayed burst indicator - only when WB master attempts CAB transfer and prefetch enable of corresponding image is set - 
                                                  // applies for reads only - delayed write cannot be a burst

/*----------------------------------------------------------------------------------------------------------------------
Delayed transaction bus command generation
Bus command for delayed reads depends on image's address space mapping and control bits and
whether or not these are interrupt acknowledge requests or configuration cycle requests
---------------------------------------------------------------------------------------------------------------------*/
assign del_bc_out = del_bc ;

always@(map or CAB_I or mrl_en or pref_en or iack_hit or ccyc_hit or WE_I or wb_conf_hit_in)
begin
    `ifdef HOST
    if (wb_conf_hit_in)
    begin
        if (iack_hit)
        begin
            del_bc <= `BC_IACK ;
        end
        else
        begin
            if (WE_I)
                del_bc <= `BC_CONF_WRITE ;
            else
                del_bc <= `BC_CONF_READ ;
        end
    end
    else
    `endif
    begin
	    if (map) // map = 1 - IO space
	    begin
		    del_bc <= `BC_IO_READ ;
	    end
	    else 
	    if ( CAB_I && mrl_en && pref_en) // burst and memory read line enable
	    begin
		    del_bc <= `BC_MEM_READ_LN ;
	    end
	    else // normal single memory access
	    begin
		    del_bc <= `BC_MEM_READ ;
	    end
    end
end

// WISHBONE data output select lines for output multiplexor
reg sdata_o_sel ;

reg del_in_progress_out ; // state machine indicates whether current read completion is in progress on WISHBONE bus

// state machine logic
always@(
        c_state                     or 
        wattempt                    or
        img_wallow                  or
        burst_transfer              or 
        wb_hit_in                   or 
        map                         or 
        same_hit                    or
        alligned_address            or
        rattempt                    or
        do_dread_request            or
        do_dread_completion         or
        wbr_fifo_almost_empty_in    or
        wbr_fifo_control_in         or
        wb_conf_hit_in              or
        sel_error                   or
        do_ccyc_req                 or 
        do_ccyc_comp                or
        ccyc_hit                    or
        del_write_in                or
        del_error_in                or
        do_iack_req                 or
        do_iack_comp                or
        iack_hit                    or
        hit_error
       )
begin    
    case (c_state)
    S_IDLE:begin
                if (wattempt)
                begin
                    // read signals in inactive state when writes are in progress
                    wbr_fifo_flush   <= 1'b0 ;
                    wbr_fifo_renable <= 1'b0 ;
                    
                    // configuration read enable control signal inactive
                    conf_renable <= 1'b0 ;
                        
                    // WISHBONE data output selection - since this is a write, hold wbr output
                    sdata_o_sel <= WBR_SEL ;

                    // read cannot be in progress while write is attempted
                    del_in_progress_out <= 1'b0 ;

                    // delayed request signals inactive
                    del_req  <= 1'b0 ;
                    del_done <= 1'b0 ;
        
                    // configuration access control signals inactive
                    conf_wenable <= 1'b0 ;
                    conf_renable <= 1'b0 ;

                    if(wb_hit_in)
                    begin

                        // check error conditions for image writes
                        if ( 
                            (map && (burst_transfer || sel_error)) || // IO write is a burst or has wrong select lines active= Error
                            (~map && ~alligned_address) ||  // Mem write to nonaligned address = error
                            hit_error                       // images overlaping - error
                           )
                        begin
                            n_state <= S_IDLE ; // stay in idle state because of an error condition

                            // respond with an error
                            ack         <= 1'b0 ;
                            rty         <= 1'b0 ;
                            err         <= 1'b1 ;

                            // write signals in inactive state
                            wbw_fifo_control <= `ADDRESS ;
                            wbw_fifo_wenable <= 1'b0 ;            
                            d_incoming_ena   <= 1'b0 ;
           
                        end // error conditions
                        else
                        // check for retry conditions for image writes
                        if ( ~img_wallow ) // write to image not allowed at this time = retry
                        begin
                            n_state <= S_IDLE ; // stay in IDLE
                            
                            // respond with a retry
                            ack         <= 1'b0 ;
                            rty         <= 1'b1 ;
                            err         <= 1'b0 ;

                            // write signals in inactive state
                            wbw_fifo_control <= `ADDRESS ;
                            wbw_fifo_wenable <= 1'b0 ;            
                            d_incoming_ena   <= 1'b0 ;
                            
                        end //retry
                        else // everything OK - proceed
                        begin
                            n_state <= S_W_ADDR_DATA ; // goto write transfer state
            
                            // respond with acknowledge
                            ack         <= 1'b1 ;
                            rty         <= 1'b0 ;
                            err         <= 1'b0 ;

                            // write control signals
                            // fifo is enabled - address is written to it with appropriate control encoding
                            wbw_fifo_control <= `ADDRESS ;
                            wbw_fifo_wenable <= 1'b1 ;            
                            // data is latched to data incoming intermidiate stage - it will be put in FIFO later
                            d_incoming_ena   <= 1'b1 ;
                            
                        end // image write OK
                    end //wb_hit_in
                    else // no image hit

                    if (wb_conf_hit_in) // configuration space hit
                    begin
                        // image write control signals inactive - give outputs one clock cycle setup time
                        n_state <= S_CONF_WRITE ; // go to conf. write state
                            
                        // don't respond yet
                        ack         <= 1'b0 ;
                        rty         <= 1'b0 ;
                        err         <= 1'b0 ;

                        // write signals in inactive state
                        wbw_fifo_control <= `ADDRESS ;
                        wbw_fifo_wenable <= 1'b0 ;            
                        d_incoming_ena   <= 1'b0 ;

                    end // wb_conf_hit_in*/
                    else
                    begin // no hit
                        n_state <= S_IDLE ; // stay in IDLE
                            
                        // don't respond
                        ack         <= 1'b0 ;
                        rty         <= 1'b0 ;
                        err         <= 1'b0 ;

                        // write signals in inactive state
                        wbw_fifo_control <= `ADDRESS ;
                        wbw_fifo_wenable <= 1'b0 ;            
                        d_incoming_ena   <= 1'b0 ;

                    end // no hit                        
                end //wattempt
                else
                if (rattempt)
                begin
                    // write signals in inactive state
                    wbw_fifo_control <= `ADDRESS ;
                    wbw_fifo_wenable <= 1'b0 ;            
                    d_incoming_ena   <= 1'b0 ;

                    // configuration access signals inactive
                    conf_wenable <= 1'b0 ;
                    conf_renable <= 1'b0 ;
						
                    if(wb_hit_in)
                    begin
						// WISHBONE data output selection - drive wbr_output
	                    sdata_o_sel <= WBR_SEL ;
	                    
                        // check error conditions for image reads
                        if ( 
                            (map && (burst_transfer || sel_error)) || // IO read is a burst or has wrong select lines active= Error
                            (~map && ~alligned_address) || // Mem read from nonaligned address = error
                            hit_error                      // images overlaping - error
                           )
                        begin
                            n_state <= S_IDLE ; // stay in idle because of an error
                            
                            // respond with error
                            ack         <= 1'b0 ;
                            rty         <= 1'b0 ;
                            err         <= 1'b1 ;
    
                            // read controls inactive
                            wbr_fifo_flush   <= 1'b0 ;
                            wbr_fifo_renable <= 1'b0 ;
                            del_req      <= 1'b0 ;
                            del_done     <= 1'b0 ;

                            // read is not in progress
                            del_in_progress_out <= 1'b0 ;
                            
                        end // error conditions
                        else
                        
                        // check for retry conditions for image reads
                        if ( ~do_dread_request && ~do_dread_completion) // read through image not allowed at this time = retry
                        begin
                            n_state <= S_IDLE ; // stay in idle state
                            
                            // respond with retry
                            ack         <= 1'b0 ;
                            rty         <= 1'b1 ;
                            err         <= 1'b0 ;
    
                            // read controls inactive
                            wbr_fifo_flush   <= 1'b0 ;
                            wbr_fifo_renable <= 1'b0 ;
                            del_req      <= 1'b0 ;
                            del_done     <= 1'b0 ;

                            // read is not in progress
                            del_in_progress_out <= 1'b0 ;
                        
                        end //retry
                        else
                        if ( do_dread_request ) // read request can be accepted
                        begin
                            n_state <= S_IDLE ; // stay in IDLE

                            // respond with retry
                            ack         <= 1'b0 ; 
                            rty         <= 1'b1 ;
                            err         <= 1'b0 ;

                            // FIFO signals inactive
                            wbr_fifo_flush   <= 1'b0 ;
                            wbr_fifo_renable <= 1'b0 ;
                    
                            // signal read request
                            del_req      <= 1'b1 ;
                            del_done     <= 1'b0 ;

                            // read is not in progress
                            del_in_progress_out <= 1'b0 ;
                            
                        end //do_dread_request
                        else // do_dread_completion
                        begin
                            // check if data is error message
                            if (wbr_fifo_control_in == `DATA_ERROR)
                            begin
                                n_state <= S_IDLE ; // stay in idle state
                            
                                // respond with error
                                ack         <= 1'b0 ;
                                rty         <= 1'b0 ;
                                err         <= 1'b1 ;

                                // read from FIFO to free up location that error is taking up
                                wbr_fifo_flush   <= 1'b0 ;
                                wbr_fifo_renable <= 1'b1 ;

                                // respond that read is finished
                                del_req      <= 1'b0 ;
                                del_done     <= 1'b1 ;

                                // read is in progress nevertheless it was terminated with an error
                                del_in_progress_out <= 1'b1 ;

                            end // DATA_ERROR
                            else
                            if (wbr_fifo_almost_empty_in)
                            begin
                                // only one location is in the FIFO - acknowledge the transfer and stay in this state
                                n_state <= S_IDLE ; // stay in idle state
                            
                                // respond with acknowledge
                                ack         <= 1'b1 ;
                                rty         <= 1'b0 ;
                                err         <= 1'b0 ;

                                // read from FIFO
                                wbr_fifo_flush   <= 1'b0 ;
                                wbr_fifo_renable <= 1'b1 ;

                                // respond that read is finished
                                del_req      <= 1'b0 ;
                                del_done     <= 1'b1 ;

                                // read is in progress
                                del_in_progress_out <= 1'b1 ;
                            end
                            else
                            begin
                                n_state <= S_READ ; // go to read state
                            
                                // respond with acknowledge since data output from FIFO is always prepared
                                ack         <= 1'b1 ;
                                rty         <= 1'b0 ;
                                err         <= 1'b0 ;

                                //  enable read from FIFO - on posedge of clock new data will be provided
                                wbr_fifo_flush   <= 1'b0 ;
                                wbr_fifo_renable <= 1'b1 ;
                                del_req      <= 1'b0 ;
                                del_done     <= 1'b0 ;

                                // read is in progress
                                del_in_progress_out <= 1'b1 ;

                            end // no DATA_ERROR

                        end // do_dread_completion
                    end //wb_hit_in
                    else //~wb_hit
                    if(wb_conf_hit_in) // read from configuration space
                    begin 
                        n_state <= S_CONF_READ ; // go to configuration space read state - give outputs one clock cycle for setup

                        // image read signals inactive
                        wbr_fifo_flush   <= 1'b0 ;
                        wbr_fifo_renable <= 1'b0 ;
                        del_req      <= 1'b0 ;
                        del_done     <= 1'b0 ;

                        // do not respond yet
                        ack         <= 1'b0 ;
                        rty         <= 1'b0 ;
                        err         <= 1'b0 ;

						// WISHBONE data output selection - drive configuration space output
	                    sdata_o_sel <= CONF_SEL ;

                        // read is not in progress since this is configuration read
                        del_in_progress_out <= 1'b0 ;
	                    
                    end //read from configuration space

                    else // no hit

                    begin
                        n_state <= S_IDLE ; 
                        wbr_fifo_flush   <= 1'b0 ;
                        wbr_fifo_renable <= 1'b0 ;
                        del_req      <= 1'b0 ;
                        del_done     <= 1'b0 ;
                        ack         <= 1'b0 ;
                        rty         <= 1'b0 ;
                        err         <= 1'b0 ;

                        // WISHBONE data output selection - drive wbr_output
	                    sdata_o_sel <= WBR_SEL ;
	                    
                        // read is not in progress since there is no image hit
                        del_in_progress_out <= 1'b0 ;

                    end // no hit
                end //if (rattempt)
                else // no write and no read attempt - do nothing
                begin
                    n_state <= S_IDLE ; 
    
                    // write signals - all in inactive state
                    wbw_fifo_control <= `ADDRESS ;
                    wbw_fifo_wenable <= 1'b0 ;            
                    d_incoming_ena   <= 1'b0 ;
            
                    // handshaking signals inactive
                    ack         <= 1'b0 ;
                    rty         <= 1'b0 ;
                    err         <= 1'b0 ;

                    //  read signals inactive
                    wbr_fifo_flush   <= 1'b0 ;
                    wbr_fifo_renable <= 1'b0 ;
                    del_req          <= 1'b0 ;
                    del_done         <= 1'b0 ;

                    // configuration space control signals inactive
                    conf_wenable <= 1'b0 ;
                    conf_renable <= 1'b0 ;

                    // WISHBONE data output selection - drive wbr_output
	                sdata_o_sel <= WBR_SEL ;

                    // read is not in progress
                    del_in_progress_out <= 1'b0 ;
	                    
                end //no write and no read attempt                
            end //`S_IDLE
    S_W_ADDR_DATA: begin
                        // read signals inactive
                        wbr_fifo_flush   <= 1'b0 ;
                        wbr_fifo_renable <= 1'b0 ;
                        del_req          <= 1'b0 ;
                        del_done         <= 1'b0 ;

                        // configuration space control signals inactive
                        conf_wenable <= 1'b0 ;
                        conf_renable <= 1'b0 ;

                       	// WISHBONE data output selection - drive wbr_output
	                    sdata_o_sel <= WBR_SEL ;

                        // delayed transaction is not in progress when receiving posted burst write	                    
                        del_in_progress_out <= 1'b0 ;

                        if (burst_transfer)
                            if (wattempt)
                            begin
                                // check for error conditions
                                if ( ~same_hit ||             // burst transfer crossed image range
                                     ~alligned_address )    // address is not alligned
                                begin
                                    n_state <= S_TURN_ARROUND ; // goto turn-arround state
                                    
                                    // respond with an error
                                    ack         <= 1'b0 ;
                                    rty         <= 1'b0 ;
                                    err         <= 1'b1 ;
    
                                    // write data latched in itermediate register and tag it as last
                                    wbw_fifo_control <= `LAST ;
                                    wbw_fifo_wenable <= 1'b1 ;            
                                    d_incoming_ena   <= 1'b0 ;

                                end // error
                                else //no error
                                // check for retry condition
                                if (~img_wallow)
                                begin
                                    n_state <= S_TURN_ARROUND ; // FIFO was filled or error lock occurred - goto turnarround state

                                    // respond with retry
                                    ack         <= 1'b0 ;
                                    rty         <= 1'b1 ;
                                    err         <= 1'b0 ;

                                    // write last data latched in intermediate register
                                    wbw_fifo_control <= `LAST ;
                                    wbw_fifo_wenable <= 1'b1 ;            
                                    d_incoming_ena   <= 1'b0 ;
                                    
                                end // retry    
                                else // no error and no retry - acknowledge
                                begin
                                    n_state <= S_W_ADDR_DATA ; // stay in current state
                                    
                                    // respond with acknowledge
                                    ack         <= 1'b1 ;
                                    rty         <= 1'b0 ;
                                    err         <= 1'b0 ;
                    
                                    // write data from intermediate register to fifo and latch new data provided
                                    wbw_fifo_control <= `DATA ;
                                    wbw_fifo_wenable <= 1'b1 ;            
                                    d_incoming_ena   <= 1'b1 ;
                                    
                                end // no error and no retry - acknowledge
                            end // wattempt
                            else // no write attempt
                            begin
                                n_state <= S_W_ADDR_DATA ; // master is inserting WS - stay in current state

                                // response signals inactive
                                ack         <= 1'b0 ;
                                rty         <= 1'b0 ;
                                err         <= 1'b0 ;

                                //write signals inactive
                                wbw_fifo_control <= `DATA ;
                                wbw_fifo_wenable <= 1'b0 ;            
                                d_incoming_ena   <= 1'b0 ;
                                
                            end // no write attempt
                        else // ~burst transfer
                        begin
                            n_state <= S_TURN_ARROUND ; // no burst transfer - go back to idle
                            
                            // do not respond
                            ack         <= 1'b0 ;
                            rty         <= 1'b0 ;
                            err         <= 1'b0 ;
                            
                            // write last data to FIFO and don't latch new data
                            wbw_fifo_control <= `LAST ;
                            wbw_fifo_wenable <= 1'b1 ;            
                            d_incoming_ena   <= 1'b0 ;
                            
                        end // burst_transfer
                    end // S_W_ADDR_DATA

    S_READ:begin
                // write signals in inactive state
                wbw_fifo_control <= `ADDRESS ;
                wbw_fifo_wenable <= 1'b0 ;            
                d_incoming_ena   <= 1'b0 ;
                
                // configuration space control signals inactive
                conf_wenable <= 1'b0 ;
                conf_renable <= 1'b0 ;

				// WISHBONE data output selection - drive wbr_output
	            sdata_o_sel <= WBR_SEL ;
	            
                // this state is for reads only - in this state read is in progress all the time
                del_in_progress_out <= 1'b1 ;

                if(burst_transfer)
                begin
                    if(rattempt)
                    begin
                        // check for error conditions
                        if (~same_hit || // read passed image's range
                            ~alligned_address ) // address is not alligned within burst transfer
                        begin
                            // return to idle state
                            n_state <= S_IDLE ;
                    
                            // signal an error
                            ack         <= 1'b0 ;
                            rty         <= 1'b0 ;
                            err         <= 1'b1 ;

                            // read signals : signal that read completion has finished on WISHBONE and flush FIFO
                            wbr_fifo_flush   <= 1'b1 ;
                            wbr_fifo_renable <= 1'b0 ;
                            del_req          <= 1'b0 ;
                            del_done         <= 1'b1 ;
                        end // error
                        else
                        if (wbr_fifo_control_in == `DATA_ERROR)
                        begin
                            // signal an error has occured on PCI bus during this location read
                            n_state <= S_IDLE ; // go back to idle state
                            
                            // respond with error
                            ack         <= 1'b0 ;
                            rty         <= 1'b0 ;
                            err         <= 1'b1 ;

                            // read controls - enable fifo read and signal that read is finished
                            wbr_fifo_flush   <= 1'b0 ;
                            wbr_fifo_renable <= 1'b1 ;
                            del_req          <= 1'b0 ;
                            del_done         <= 1'b1 ;
                        end
                        else
                        if (wbr_fifo_almost_empty_in) // WB is now reading last data from FIFO
                        begin
                            n_state <= S_IDLE ; // go back to idle state
                            
                            // acknowledge the transaction
                            ack         <= 1'b1 ;
                            rty         <= 1'b0 ;
                            err         <= 1'b0 ;

                            // read controls - enable fifo read and signal that read is finished
                            wbr_fifo_flush   <= 1'b0 ;
                            wbr_fifo_renable <= 1'b1 ;
                            del_req          <= 1'b0 ;
                            del_done         <= 1'b1 ;

                        end //wbr_fifo_almost_empty_in
                        else //~wbr_fifo_almost_empty_in
                        begin
                            n_state <= S_READ ; // stay in this state
                            
                            // acknowledge the transaction
                            ack         <= 1'b1 ;
                            rty         <= 1'b0 ;
                            err         <= 1'b0 ;

                            // read controls - enable fifo read
                            wbr_fifo_flush   <= 1'b0 ;
                            wbr_fifo_renable <= 1'b1 ;
                            del_req          <= 1'b0 ;
                            del_done         <= 1'b0 ;
                        end //~wbr_fifo_almost_empty_in
                    end //rattempt
                    else 
                    begin //~rattempt
                        n_state <= S_READ ; // stay in this state
                            
                        // response signals inactive
                        ack         <= 1'b0 ;
                        rty         <= 1'b0 ;
                        err         <= 1'b0 ;

                        // read controls - inactive - master inserting WS
                        wbr_fifo_flush   <= 1'b0 ;
                        wbr_fifo_renable <= 1'b0 ;
                        del_req          <= 1'b0 ;
                        del_done         <= 1'b0 ;
                    end //~rattempt
                end //burst_transfer
                else //~burst_transfer
                begin
                    // this isn't a burst transfer - return to IDLE state
                    n_state <= S_IDLE ;
                    
                    // do not respond with any signal
                    ack         <= 1'b0 ;
                    rty         <= 1'b0 ;
                    err         <= 1'b0 ;

                    // read signals : signal that read completion has finished on WISHBONE and flush FIFO
                    wbr_fifo_flush   <= 1'b1 ;
                    wbr_fifo_renable <= 1'b0 ;
                    del_req          <= 1'b0 ;
                    del_done         <= 1'b1 ;
                    
                end //~burst_transfer
            end // S_READ

    S_TURN_ARROUND:begin
                        // turn-arround is provided for FIFO - it does everything necesarry by itself
                        n_state <= S_IDLE ; // next state is always idle

                        // response signals inactive
                        ack         <= 1'b0 ;
                        rty         <= 1'b0 ;
                        err         <= 1'b0 ;

                        //write signals inactive
                        wbw_fifo_control <= `ADDRESS ;
                        wbw_fifo_wenable <= 1'b0 ;            
                        d_incoming_ena   <= 1'b0 ;
                        
                        // read signals inactive
                        wbr_fifo_flush   <= 1'b0 ;
                        wbr_fifo_renable <= 1'b0 ;
                        del_req          <= 1'b0 ;
                        del_done         <= 1'b0 ;

                        // configuration space control signals inactive
                        conf_wenable <= 1'b0 ;
                        conf_renable <= 1'b0 ;

						// WISHBONE data output selection - drive wbr_output
	                    sdata_o_sel <= WBR_SEL ;

                        // read is not in progress
                        del_in_progress_out <= 1'b0 ;
	                    
                    end // S_TURN_ARROUND
    S_CONF_WRITE:  begin
                        n_state <= S_IDLE ; // next state after configuration access is always idle

                        //image write signals inactive
                        wbw_fifo_control <= `ADDRESS ;
                        wbw_fifo_wenable <= 1'b0 ;            
                        d_incoming_ena   <= 1'b0 ;
                        
                        // image read signals inactive
                        wbr_fifo_flush   <= 1'b0 ;
                        wbr_fifo_renable <= 1'b0 ;
                        
                        // configuration space read enable control signal inactive
                        conf_renable <= 1'b0 ;

						// WISHBONE data output selection - drive wbr_output
	                    sdata_o_sel <= WBR_SEL ;

                        if (wb_conf_hit_in && wattempt)
                        begin
                            if (burst_transfer || // bursts not allowed to configuration space
                                sel_error )       // illegal address and select lines combination
                            begin
                                // burst transfer to configuration space
                                // respond with an error
                                ack         <= 1'b0 ;
                                rty         <= 1'b0 ;
                                err         <= 1'b1 ;

                                // configuration access write enable control signal inactive
                                conf_wenable <= 1'b0 ;
                                
                                // delayed request signals inactive
                                del_req  <= 1'b0 ;
                                del_done <= 1'b0 ;

                                // delayed is not in progress since this is an error
                                del_in_progress_out <= 1'b0 ;

                            end //error
                            else
                        
                        `ifdef GUEST
                            begin
                                // guest bridge doesn't have write access to configuration space
                                conf_wenable <= 1'b0 ;
                            
                                // acknowledge the cycle
                                rty <= 1'b0 ;
                                ack <= 1'b1 ;
                                err <= 1'b0 ;
                                
                                // delayed request signals inactive
                                del_req  <= 1'b0 ;
                                del_done <= 1'b0 ;

                                // delayed transaction is not in progress
                                del_in_progress_out <= 1'b0 ;
                             end
                    
                        `else
                        `ifdef HOST
                            // check whether this is a write to conf. cycle register
                            if ( ccyc_hit ) 
                            begin
                                conf_wenable <= 1'b0 ; 
                                // retry
                                if (~do_ccyc_req && ~do_ccyc_comp) // neither request or completion can be performed at this time - retry
                                begin

                                    rty <= 1'b1 ;
                                    ack <= 1'b0 ;
                                    err <= 1'b0 ;
    
                                    // delayed request signals inactive
                                    del_req  <= 1'b0 ;
                                    del_done <= 1'b0 ;
                            
                                    // delayed transaction is not in progress
                                    del_in_progress_out <= 1'b0 ;

                                end //retry
                                else
                                if (do_ccyc_req)
                                begin
                                    // request can be issued - respond with retry
                                    rty <= 1'b1 ;
                                    ack <= 1'b0 ;
                                    err <= 1'b0 ;
    
                                    // issue delayed request
                                    del_req  <= 1'b1 ;
                                    del_done <= 1'b0 ;

                                    // delayed completion is not in progress
                                    del_in_progress_out <= 1'b0 ;
                                end
                                else // do ccyc completion
                                begin 
                                    
                                    // signal completion done and in progress
                                    del_req  <= 1'b0 ;
                                    del_done <= 1'b1 ;
                                    del_in_progress_out <= 1'b1 ;
                                    
                                    if (del_error_in) // request was finished with target abort - signal an error
                                    begin
                                        rty <= 1'b0 ;
                                        ack <= 1'b0 ;
                                        err <= 1'b1 ;
                                    
                                    end // del_error_in
                                    else
                                    begin // ccyc master abort or normal
                                        rty <= 1'b0 ;
                                        ack <= 1'b1 ;
                                        err <= 1'b0 ;
                                    end
                                end //ccyc completion
                            end //conf_cyc_hit
                            else //ordinary configuration hit                    
                            begin
    
                                // enable configuration space write
                                conf_wenable <= 1'b1 ;
    
                                // acknowledge the cycle
                                rty <= 1'b0 ;
                                ack <= 1'b1 ;
                                err <= 1'b0 ;
                                
                                // delayed request controls inactive
                                del_req  <= 1'b0 ;
                                del_done <= 1'b0 ;
                                del_in_progress_out <= 1'b0 ;
    
                            end //ordinary configuration hit
                        `endif
                        `endif
                        end // wb_conf_hit_in
                      
                        else // no conf hit
                        begin
                            rty <= 1'b0 ;
                            ack <= 1'b0 ;
                            err <= 1'b0 ;

                            // configuration write enable inactive
                            conf_wenable <= 1'b0 ;

                            // configuration cycle controls inactive
                            del_req  <= 1'b0 ;
                            del_done <= 1'b0 ;
                            del_in_progress_out <= 1'b0 ;

                        end // no conf hit
                    
                    end // S_CONF_WRITE

    S_CONF_READ:   begin
                        n_state <= S_IDLE ; // next state after configuration access is always idle

                        //image write signals inactive
                        wbw_fifo_control <= `ADDRESS ;
                        wbw_fifo_wenable <= 1'b0 ;            
                        d_incoming_ena   <= 1'b0 ;
                        
                        // image read signals inactive
                        wbr_fifo_flush   <= 1'b0 ;
                        
                        // configuration space write enable control signal inactive
                        conf_wenable <= 1'b0 ;
                        
                        if(wb_conf_hit_in && rattempt) // read from configuration space
                        begin 
    
                            if (burst_transfer || // bursts not allowed to configuration space
                                sel_error )       // illegal address and select lines combination
                            begin
                                // respond with an error
                                ack         <= 1'b0 ;
                                rty         <= 1'b0 ;
                                err         <= 1'b1 ;
                                
                                // delayed request control inactive
                                del_req  <= 1'b0 ;
                                del_done <= 1'b0 ;                        
                                
                                // delayed transaction is not in progress
                                del_in_progress_out <= 1'b0 ;

                                // configuration read enable inactive
                                conf_renable <= 1'b0 ;
    
    							// WISHBONE data output selection - drive configuration output
			                    sdata_o_sel <= CONF_SEL ;

                                // fifo read enable inactive
                                wbr_fifo_renable <= 1'b0 ;
			                    
                            end //error
                            else
                            
                        `ifdef GUEST
                            
                            begin // ordinary configuration read
                                ack         <= 1'b1 ;
                                rty         <= 1'b0 ;
                                err         <= 1'b0 ;
                            
                                // configuration read enable active
                                conf_renable <= 1'b1 ;
    
    							// WISHBONE data output selection - drive configuration output
			                    sdata_o_sel <= CONF_SEL ;

                                // delayed request signals inactive
                                del_req <= 1'b0 ;
                                del_done <= 1'b0 ;
                                del_in_progress_out <= 1'b0 ;

                                // fifo read enable inactive
                                wbr_fifo_renable <= 1'b0 ;
			                    
                            end // ordinary configuration read
    
                        `else
                        `ifdef HOST
                            // check whether this is a read from conf. cycle register
                            if ( ccyc_hit ) 
                            begin
   
                                // configuration read enable inactive
                                conf_renable <= 1'b0 ;
                                
                                // WISHBONE data output selection - drive configuration cycle output
			                    sdata_o_sel <= WBR_SEL ;
    
                                // retry
                                if (~do_ccyc_req && ~do_ccyc_comp) // neither request or completion can be performed at this time - retry
                                begin
                                    rty <= 1'b1 ;
                                    ack <= 1'b0 ;
                                    err <= 1'b0 ;
    
                                    // do not issue new request
                                    del_req <= 1'b0 ;
                                    del_done <= 1'b0 ;
                                    del_in_progress_out <= 1'b0 ;
    
                                    // fifo read enable inactive
                                    wbr_fifo_renable <= 1'b0 ;

                                end //retry
                                else
                                if (do_ccyc_req)
                                begin
                                    // request can be issued - respond with retry
                                    rty <= 1'b1 ;
                                    ack <= 1'b0 ;
                                    err <= 1'b0 ;
    
                                    // initiate configuration write request
                                    del_req <= 1'b1 ;
                                    del_done <= 1'b0 ;
                                    del_in_progress_out <= 1'b0 ;

                                    // fifo read enable inactive
                                    wbr_fifo_renable <= 1'b0 ;
                                end
                                else // do ccyc completion
                                begin 
                                    
                                    // signal completion done and in progress
                                    del_done <= 1'b1 ;
                                    del_in_progress_out <= 1'b1 ;
                                    
                                    // fifo read enable
                                    wbr_fifo_renable <= 1'b1 ;

                                    // don't issue new request
                                    del_req <= 1'b0 ;
                                    
                                    if (del_error_in) // request was finished with target abort - signal an error
                                    begin
                                        rty <= 1'b0 ;
                                        ack <= 1'b0 ;
                                        err <= 1'b1 ;
                                    
                                    end // ccyc_tabort
                                    else
                                    begin // ccyc master abort or normal
                                        rty <= 1'b0 ;
                                        ack <= 1'b1 ;
                                        err <= 1'b0 ;
                                    end
                                end // ccyc_completion
                            end //conf_cyc_hit
                            else
                            // check if this is an interrupt acknowledge cycle request
                            if(iack_hit)
                            begin

                                // configuration read enable inactive
                                conf_renable <= 1'b0 ;
    
    							// WISHBONE data output selection - drive interrupt acknowledge output
			                    sdata_o_sel <= WBR_SEL ;
			                    
                                // retry
                                if (~do_iack_req && ~do_iack_comp) // neither request or completion can be performed at this time - retry
                                begin
                                    rty <= 1'b1 ;
                                    ack <= 1'b0 ;
                                    err <= 1'b0 ;
                
                                    // delayed control signals inactive
                                    del_req <= 1'b0 ;
                                    del_done <= 1'b0 ;
                                    del_in_progress_out <= 1'b0 ;

                                    // fifo read enable inactive
                                    wbr_fifo_renable <= 1'b0 ;
    
                                end //retry
                                else
                                if (do_iack_req)
                                begin
                                    // request can be issued - respond with retry
                                    rty <= 1'b1 ;
                                    ack <= 1'b0 ;
                                    err <= 1'b0 ;
    
                                    // initiate interrupt acknowledge request
                                    del_req <= 1'b1 ;
                                    del_done <= 1'b0 ;
                                    del_in_progress_out <= 1'b0 ;

                                    // fifo read enable inactive
                                    wbr_fifo_renable <= 1'b0 ;

                                end
                                else
                                begin //do iack completion
                                    del_req <= 1'b0 ;
                                    del_done <= 1'b1 ; // signal done
                                    del_in_progress_out <= 1'b1 ;

                                    // fifo read enable
                                    wbr_fifo_renable <= 1'b1 ;
                                    
                                    rty <= 1'b0 ; // retry inactive - completion can be terminated with either acknowledge or error

                                    if (del_error_in) // terminated with target abort
                                    begin
                                        ack <= 1'b0 ;
                                        err <= 1'b1 ;
                                    end // target abort
                                    else
                                    begin
                                        ack <= 1'b1 ;
                                        err <= 1'b0 ;
                                    end // target termination normal
    
                                end // do iack completion
    
                            end // iack_hit
                            else
                            begin // configuration read from ordinary configuration space
                                // configuration cycle controls inactive
                                del_req <= 1'b0 ;
                                del_done <= 1'b0 ;
                                del_in_progress_out <= 1'b0 ;

                                // fifo read enable inactive
                                wbr_fifo_renable <= 1'b0 ;
                            
                                // configuration read enable active
                                conf_renable <= 1'b1 ;
                                
                                // acknowledge this cycle
                                rty <= 1'b0 ;
                                ack <= 1'b1 ;
                                err <= 1'b0 ;
                                
                                // WISHBONE data output selection - drive configuration output
			                    sdata_o_sel <= CONF_SEL ;
                            end 
                        `endif
                        `endif
    
                        end //read from configuration space
    
                        else // no hit
                        begin
                            // response signals inactive
                            ack         <= 1'b0 ;
                            rty         <= 1'b0 ;
                            err         <= 1'b0 ;
    
                            // configuration cycle controls inactive
                            del_req <= 1'b0 ;
                            del_done <= 1'b0 ;                        
                            del_in_progress_out <= 1'b0 ;

                            // fifo read enable inactive
                            wbr_fifo_renable <= 1'b0 ;
    
                            // configuration read enable inactive
                            conf_renable <= 1'b0 ;
                            
                            // WISHBONE data output selection - drive configuration output
    	                    sdata_o_sel <= CONF_SEL ;
    	                    
                        end // no hit
                    end //S_CONF_READ
    default:begin
                n_state <= S_IDLE ; // return to idle state

                // response signals inactive
                ack         <= 1'b0 ;
                rty         <= 1'b0 ;
                err         <= 1'b0 ;

                //write signals inactive
                wbw_fifo_control <= `ADDRESS ;
                wbw_fifo_wenable <= 1'b0 ;            
                d_incoming_ena   <= 1'b0 ;
                        
                // read signals inactive
                wbr_fifo_flush   <= 1'b0 ;
                wbr_fifo_renable <= 1'b0 ;
                del_req          <= 1'b0 ;
                del_done         <= 1'b0 ;
        
                // configuration space control signals inactive
                conf_wenable <= 1'b0 ;
                conf_renable <= 1'b0 ;
                
				// WISHBONE data output selection - drive wbr output
                sdata_o_sel <= WBR_SEL ;

                // read is not in progress
                del_in_progress_out <= 1'b0 ;
                
            end //default
    endcase
end

// configuration space offset output assignment
assign wb_conf_offset_out = {wb_addr_in[11:2], 2'b00} ; // upper 10 bits of address input and two zeros

// Configuration space byte enables output
assign wb_conf_be_out = SEL_I ; // just route select lines from WISHBONE to conf space

// data output assignment - for image writes, first data is address, subsequent data comes from intermediate register
reg [31:0] wb_data_out ;
always@(ccyc_hit or c_state or wb_addr_in or ccyc_addr_in or d_incoming)
begin
    wb_data_out <= d_incoming ;
    if ( c_state == S_IDLE )
        wb_data_out <= wb_addr_in ;
    else
    if ( ((c_state == S_CONF_WRITE) || (c_state == S_CONF_READ)) && ccyc_hit )
        wb_data_out <= ccyc_addr_in ;
end

// command / byte enable assignment - with address, bus command is provided, with data - byte enables are provided
reg [3:0] wb_cbe ;
assign wb_cbe_out = wb_cbe ;

always@(c_state or d_incoming or map)
begin
	if (c_state == S_IDLE)
	begin
        if (map)
		    wb_cbe <= `BC_IO_WRITE ;
        else
            wb_cbe <= `BC_MEM_WRITE ;
	end
	else
		wb_cbe <= ~(d_incoming[35:32]) ;
end

// for configuration writes, data output is always data from WISHBONE - in guest implementation data is all 0.
`ifdef GUEST
	assign wb_conf_data_out = 32'h00000000 ;	
`else
`ifdef HOST
	assign wb_conf_data_out = SDATA_I ;
`endif
`endif

// WISHBONE data output multiplexor
reg [31:0] sdata ;
assign SDATA_O = sdata ;

always@(sdata_o_sel or wbr_fifo_data_in or wb_conf_data_in)
begin
	case (sdata_o_sel)
		WBR_SEL :sdata <= wbr_fifo_data_in ;
		CONF_SEL:sdata <= wb_conf_data_in ;
	endcase
end

endmodule //WB_SLAVE
