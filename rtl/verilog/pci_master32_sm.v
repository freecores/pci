//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "pci_master32_sm.v"                               ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
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
// Revision 1.1.1.1  2001/10/02 15:33:46  mihad
// New project directory structure
//
//

// module includes pci master state machine and surrounding logic
`include "bus_commands.v"
`include "constants.v"
`include "timescale.v"

module PCI_MASTER32_SM
(
    // system inputs
    clk_in,
    reset_in,
    // arbitration
    pci_req_out,
    pci_gnt_in,
    // master in/outs
    pci_frame_in,
    pci_frame_out,
    pci_frame_out_in,
    pci_frame_load_out,
    pci_frame_en_in,
    pci_frame_en_out,
    pci_irdy_in,
    pci_irdy_out,
    pci_irdy_en_out,
    
    // target response inputs
    pci_trdy_in,
    pci_trdy_reg_in,
    pci_stop_in,
    pci_stop_reg_in,
    pci_devsel_in,
    pci_devsel_reg_in,

    // address, data, bus command, byte enable in/outs
    pci_ad_reg_in,
    pci_ad_out,
    pci_ad_en_out,
    pci_cbe_out,
    pci_cbe_en_out,

    // other side of state machine
    address_in,
    bc_in,
    data_in,
    data_out,
    be_in,
    req_in,
    rdy_in,
    last_in,
    next_data_in,
    next_be_in,
    next_last_in,
    load_next_out,
    wait_out,
    wtransfer_out,
    rtransfer_out,
    retry_out,
    werror_out,
    rerror_out,
    first_out,
    mabort_out,
    latency_tim_val_in
) ;

// system inputs
input   clk_in,
        reset_in ;

/*==================================================================================================================
PCI interface signals - bidirectional signals are divided to inputs and outputs in I/O cells instantiation
module. Enables are separate signals.
==================================================================================================================*/
// arbitration
output  pci_req_out ;

input   pci_gnt_in ;

// master in/outs
input   pci_frame_in ;
input   pci_frame_en_in ;
input   pci_frame_out_in ;

output  pci_frame_out,
        pci_frame_en_out ;

output  pci_frame_load_out ;

input   pci_irdy_in ;
output  pci_irdy_out,
        pci_irdy_en_out;
    
// target response inputs
input   pci_trdy_in,
        pci_trdy_reg_in,
        pci_stop_in,
        pci_stop_reg_in,
        pci_devsel_in,
        pci_devsel_reg_in ;
    
// address, data, bus command, byte enable in/outs
input   [31:0]  pci_ad_reg_in ;
output  [31:0]  pci_ad_out ;

reg     [31:0]  pci_ad_out ;

output          pci_ad_en_out ;

output  [3:0]   pci_cbe_out ;

reg     [3:0]   pci_cbe_out ;

output          pci_cbe_en_out ;

input   [31:0]  address_in ; // current request address input

input   [3:0]   bc_in ;      // current request bus command input

input   [31:0]  data_in ;    // current dataphase data input

output  [31:0]  data_out ;    // for read operations - current request data output

reg     [31:0]  data_out ;    

input   [3:0]   be_in ;      // current dataphase byte enable inputs

input           req_in ;     // initiator cycle is requested
input           rdy_in ;     // requestor indicates that data is ready to be sent for write transaction and ready to
                            // be received on read transaction
input           last_in ;    // last dataphase in current transaction indicator

// status outputs
output wait_out,            // wait indicates to the backend that dataphases are not in progress on PCI bus
       wtransfer_out,       // on any rising clock edge that this status is 1, data is transferred - heavy constraints here
       rtransfer_out,       // registered transfer indicator - when 1 indicates that data was transfered on previous clock cycle
       retry_out,           // retry status output - when target signals a retry
       werror_out,          // error output - when 1 indicates that error (target abort) occured on current dataphase - heavy constraints
       rerror_out,          // registered error output - when 1 indicates that error was signalled by a target on previous clock cycle
       first_out ,          // indicates whether or not any data was transfered in current transaction
       mabort_out;          // master abort indicator

reg wait_out ;

// latency timer value input - state machine starts latency timer whenever it starts a transaction and last is not
// asserted ( meaning burst transfer ). 
input [7:0] latency_tim_val_in ;

// next data, byte enable and last inputs
input [31:0] next_data_in ;
input [3:0]  next_be_in ;
input        next_last_in ;

// clock enable for data output flip-flops - whenever data is transfered, sm loads next data to those flip flops
output       load_next_out ;

// parameters - states - one hot
// idle state
parameter S_IDLE            = 4'h1 ;

// address state
parameter S_ADDRESS         = 4'h2 ;

// transfer state - dataphases
parameter S_TRANSFER        = 4'h4 ;

// turn arround state
parameter S_TA_END          = 4'h8 ;

// change state - clock enable for sm state register
wire change_state ;
// next state for state machine
reg [4:0] next_state ;
// SM state register
reg [4:0] cur_state ;

// variables for indicating which state state machine is in
// this variables are used to reduce logic levels in case of heavily constrained PCI signals
reg sm_idle            ;
reg sm_address         ;
reg sm_data_phases     ;
reg sm_turn_arround    ;

// state machine register control logic with clock enable
always@(posedge reset_in or posedge clk_in)
begin
    if (reset_in)
        cur_state <= #`FF_DELAY S_IDLE ;
    else
    if ( change_state )
        cur_state <= #`FF_DELAY next_state ;
end

// parameters - data selector - ad and bc lines switch between address/data and bus command/byte enable respectively
parameter SEL_ADDR_BC      = 2'b01 ;
parameter SEL_DATA_BE      = 2'b00 ;
parameter SEL_NEXT_DATA_BE = 2'b11 ;

reg [1:0] wdata_selector ;

wire u_dont_have_pci_bus = pci_gnt_in || ~pci_frame_in || ~pci_irdy_in ;    // pci master can't start a transaction when GNT is deasserted ( 1 ) or
                                                                            // bus is not in idle state ( FRAME and IRDY both 1 )
wire u_have_pci_bus      = ~pci_gnt_in && pci_frame_in && pci_irdy_in ;

// decode count enable - counter that counts cycles passed since address phase
wire        sm_decode_count_enable = sm_data_phases ;                                                               // counter is enabled when master wants to transfer 
wire        decode_count_enable    = sm_decode_count_enable && pci_trdy_in && pci_stop_in && pci_devsel_in ;        // and target is not responding
wire        decode_count_load      = ~decode_count_enable ;                 
reg [2:0]   decode_count ;

wire decode_to = ~( decode_count[2] || decode_count[1]) ;

always@(posedge reset_in or posedge clk_in)
begin
    if ( reset_in )
        // initial value of counter is 4
        decode_count <= #`FF_DELAY 3'h4 ;
    else
    if ( decode_count_load )
        decode_count <= #`FF_DELAY 3'h4 ;
    else
    if ( decode_count_enable )
        decode_count <= #`FF_DELAY decode_count - 1'b1 ;
end

// Bus commands LSbit indicates whether operation is a read or a write
wire do_write = bc_in[0] ;

// latency timer
reg [7:0]   latency_timer ;


wire latency_timer_enable = sm_data_phases ;
wire latency_timer_load   = ~sm_address && ~sm_data_phases ;
wire latency_timer_exp    = ~( 
                               (latency_timer[7] || latency_timer[6] || latency_timer[5] || latency_timer[4]) || 
                               (latency_timer[3] || latency_timer[2] || latency_timer_load)
                             ) ;

// flip flop for registering latency timer timeout
reg         latency_time_out ;
always@(posedge clk_in or posedge reset_in)
begin
    if (reset_in)
        latency_time_out <= #`FF_DELAY 1'b0 ;
    else
        latency_time_out <= #`FF_DELAY latency_timer_exp ;
end

always@(posedge clk_in or posedge reset_in)
begin
    if (reset_in)
        latency_timer <= #`FF_DELAY 8'hFF ;
    else
    if ( latency_timer_load )
        latency_timer <= #`FF_DELAY latency_tim_val_in ;
    else
    if ( latency_timer_enable && ~latency_time_out)         // latency timer counts down until it expires - then it stops
        latency_timer <= #`FF_DELAY latency_timer - 1'b1 ;
end

// master abort indicators - when decode time out occurres and still no target response is received
wire do_master_abort = decode_to && pci_trdy_in && pci_stop_in && pci_devsel_in ;
reg mabort1 ;
always@(posedge reset_in or posedge clk_in)
begin
    if (reset_in)
        mabort1 <= #`FF_DELAY 1'b0 ;
    else
        mabort1 <= #`FF_DELAY do_master_abort ;
end

reg mabort2 ;
always@(posedge reset_in or posedge clk_in)
begin
    if ( reset_in )
        mabort2 <= #`FF_DELAY 1'b0 ;
    else
        mabort2 <= #`FF_DELAY mabort1 ;
end

// master abort is only asserted for one clock cycle
assign mabort_out = mabort1 && ~mabort2 ;

// register indicating when master should do timeout termination (latency timer expires)
reg timeout ;
always@(posedge reset_in or posedge clk_in)
begin
    if (reset_in)
        timeout <= #`FF_DELAY 1'b0 ;
    else 
        timeout <= #`FF_DELAY (latency_time_out && ~pci_frame_out_in && pci_gnt_in || timeout ) && ~wait_out ;
end

wire timeout_termination = sm_turn_arround && timeout && pci_stop_reg_in ;

// frame control logic
// frame is forced to 0 (active) when state machine is in idle state, since only possible next state is address state which always drives frame active
wire force_frame = ~sm_idle ;
// slow signal for frame calculated from various registers in the core
wire slow_frame  = last_in || timeout || (next_last_in && sm_data_phases) || mabort1 ;
// critical timing frame logic in separate module - some combinations of target signals force frame to inactive state immediately after sampled asserted
// (STOP)
FRAME_CRIT frame_iob_feed
(
    .pci_frame_out      (pci_frame_out),
    .force_frame_in     (force_frame),
    .slow_frame_in      (slow_frame),
    .pci_stop_in        (pci_stop_in)
) ;

// frame IOB flip flop's clock enable signal
// slow clock enable - calculated from internal - non critical paths
wire frame_load_slow = sm_idle || sm_address || mabort1 ;

// critical clock enable for frame IOB in separate module - target response signals actually allow frame value change - critical timing
FRAME_LOAD_CRIT frame_iob_ce
(
    .pci_frame_load_out (pci_frame_load_out),
    .sm_data_phases_in  (sm_data_phases),
    .frame_load_slow_in (frame_load_slow),
    .pci_trdy_in        (pci_trdy_in),
    .pci_stop_in        (pci_stop_in)
) ;

// IRDY driving
// non critical path for IRDY calculation
wire irdy_slow = pci_frame_out_in && mabort1 || mabort2 ;

// critical path in separate module
IRDY_OUT_CRIT irdy_iob_feed
(
    .pci_irdy_out       (pci_irdy_out),
    .irdy_slow_in       (irdy_slow),
    .pci_frame_out_in   (pci_frame_out_in),
    .pci_trdy_in        (pci_trdy_in),
    .pci_stop_in        (pci_stop_in)
) ;
            
// transfer FF indicator - when first transfer occurs it is set to 1 so backend can distinguish between disconnects and retries.
wire sm_transfer = sm_data_phases ;
reg transfer ;

wire transfer_input = sm_transfer && (~(pci_trdy_in || pci_devsel_in) || transfer) ;

always@(posedge clk_in or posedge reset_in)
begin
    if (reset_in)
        transfer <= #`FF_DELAY 1'b0 ;
    else
        transfer <= #`FF_DELAY transfer_input ;
end

assign first_out = ~transfer ;

// fast transfer status output - it's only negated target ready, since wait indicator qualifies valid transfer
assign wtransfer_out = ~pci_trdy_in ;

// registered transfer status output - calculated from registered target response inputs
assign rtransfer_out = ~(pci_trdy_reg_in || pci_devsel_reg_in) ;

// current error status - calculated directly from target signals and therefore critical
assign werror_out    = (~pci_stop_in && pci_devsel_in) ;

// registered error status - calculated from registered target response inputs
assign rerror_out    = (~pci_stop_reg_in && pci_devsel_reg_in) ;

// retry is signalled to backend depending on registered target response or when latency timer expires
assign retry_out = timeout_termination || (~pci_stop_reg_in && ~pci_devsel_reg_in) ;

// AD output flip flops' clock enable
// new data is loaded to AD outputs whenever state machine is idle, bus was granted and bus is in idle state or
// when address phase is about to be finished
wire load_force = (sm_idle && u_have_pci_bus) || sm_address ;

// next data loading is allowed when state machine is in transfer state and operation is a write
wire load_allow = sm_data_phases && do_write ;

// actual loading during data phases is done by monitoring critical target response signals - separate module
MAS_LOAD_NEXT_CRIT ad_iob_ce
(
    .load_next_out      (load_next_out),
    .load_force_in      (load_force),
    .load_allow_in      (load_allow),
    .pci_trdy_in        (pci_trdy_in)
) ;

// request for a bus is issued anytime when backend is requesting a transaction and state machine is in idle state
assign pci_req_out = ~(req_in && sm_idle) ;

// change state signal is actually clock enable for state register
// Non critical path for state change enable:
// state is always changed when:
// - address phase is finishing
// - state machine is in turn arround state
// - state machine is in transfer state and master abort termination is in progress

wire ch_state_slow = sm_address || sm_turn_arround || sm_data_phases && ( pci_frame_out_in && mabort1 || mabort2 ) ;

// a bit more critical change state enable is calculated with GNT signal
wire ch_state_med  = ch_state_slow || sm_idle && u_have_pci_bus && req_in && rdy_in ;

// most critical change state enable - calculated from target response signals
MAS_CH_STATE_CRIT state_machine_ce
(
    .change_state_out   (change_state),
    .ch_state_med_in    (ch_state_med),
    .sm_data_phases_in  (sm_data_phases),
    .pci_trdy_in        (pci_trdy_in),
    .pci_stop_in        (pci_stop_in)
) ;

// ad enable driving
// also divided in several categories - from less critical to most critical in separate module
wire ad_en_slowest  = do_write && (sm_address || sm_data_phases && ~pci_frame_out_in) ;
wire ad_en_on_grant = sm_idle && pci_frame_in && pci_irdy_in || sm_turn_arround ;
wire ad_en_slow     = ad_en_on_grant && ~pci_gnt_in || ad_en_slowest ;
wire ad_en_keep     = sm_data_phases && do_write && (pci_frame_out_in && ~mabort1 && ~mabort2) ;

// critical timing ad enable - calculated from target response inputs
MAS_AD_EN_CRIT ad_iob_oe_feed
(
    .pci_ad_en_out      (pci_ad_en_out),
    .ad_en_slow_in      (ad_en_slow),
    .ad_en_keep_in      (ad_en_keep),
    .pci_stop_in        (pci_stop_in),
    .pci_trdy_in        (pci_trdy_in)
) ;

// cbe enable driving
wire cbe_en_on_grant = sm_idle && pci_frame_in && pci_irdy_in || sm_turn_arround ;
wire cbe_en_slow     = cbe_en_on_grant && ~pci_gnt_in || sm_address || sm_data_phases && ~pci_frame_out_in ;
wire cbe_en_keep     = sm_data_phases && pci_frame_out_in && ~mabort1 && ~mabort2 ;

// most critical cbe enable in separate module - calculated with most critical target inputs
CBE_EN_CRIT cbe_iob_feed
(
    .pci_cbe_en_out     (pci_cbe_en_out),
    .cbe_en_slow_in     (cbe_en_slow),
    .cbe_en_keep_in     (cbe_en_keep),
    .pci_stop_in        (pci_stop_in),
    .pci_trdy_in        (pci_trdy_in)

) ;

// IRDY enable is equal to FRAME enable delayed for one clock
assign pci_irdy_en_out   = pci_frame_en_in ;

// frame enable driving - sometimes it's calculated from non critical paths
wire frame_en_slow = (sm_idle && u_have_pci_bus && req_in && rdy_in) || sm_address || (sm_data_phases && ~pci_frame_out_in) ;
wire frame_en_keep = sm_data_phases && pci_frame_out_in && ~mabort1 && ~mabort2 ;

// most critical frame enable - calculated from heavily constrained target inputs in separate module
FRAME_EN_CRIT frame_iob_en_feed
(
    .pci_frame_en_out   (pci_frame_en_out),
    .frame_en_slow_in   (frame_en_slow),
    .frame_en_keep_in   (frame_en_keep),
    .pci_stop_in        (pci_stop_in),
    .pci_trdy_in        (pci_trdy_in) 
) ;

// state machine next state definitions
always@(
    cur_state or
    do_write or
    pci_frame_out_in
)
begin
    // default values for state machine outputs
    wait_out                = 1'b1 ;
    wdata_selector          = SEL_ADDR_BC ;
    sm_idle                 = 1'b0 ;
    sm_address              = 1'b0 ;
    sm_data_phases          = 1'b0 ;
    sm_turn_arround         = 1'b0 ;

    case ( cur_state )

        S_IDLE: begin
                    // indicate the state
                    sm_idle      = 1'b1 ;
                    // assign next state - only possible is address - if state machine is supposed to stay in idle state
                    // outside signals disable the clock
                    next_state   = S_ADDRESS ;
                end

        S_ADDRESS:  begin
                        // indicate the state
                        sm_address  = 1'b1 ;
                        // select appropriate data/be for outputs
                        wdata_selector = SEL_DATA_BE ;
                        // only possible next state is transfer state
                        next_state = S_TRANSFER ;
                    end

        S_TRANSFER: begin
                        // during transfers wait indicator is inactive - all status signals are now valid
                        wait_out               = 1'b0 ;
                        // indicate the state
                        sm_data_phases         = 1'b1 ;
                        // select appropriate data/be for outputs
                        wdata_selector = SEL_NEXT_DATA_BE ;
                        if ( pci_frame_out_in )
                        begin
                            // when frame is inactive next state will be turn arround
                            next_state = S_TA_END ;
                        end
                        else
                            // while frame is active state cannot be anything else then transfer
                            next_state = S_TRANSFER ;
                    end
            
        S_TA_END:   begin
                        // wait is still inactive because of registered statuses
                        wait_out = 1'b0 ;
                        // indicate the state
                        sm_turn_arround = 1'b1 ;
                        // next state is always idle
                        next_state = S_IDLE ;
                    end
        default:    next_state = S_IDLE ;
    endcase
end

// ad and cbe lines multiplexer for write data
always@(wdata_selector or address_in or bc_in or data_in or be_in or next_data_in or next_be_in)
begin
    case ( wdata_selector )
        SEL_ADDR_BC:    begin
                            pci_ad_out  = address_in ;
                            pci_cbe_out = bc_in ;
                        end

        SEL_DATA_BE:    begin
                            pci_ad_out  = data_in ;
                            pci_cbe_out = be_in ;
                        end
        SEL_NEXT_DATA_BE,
        2'b10:              begin
                                pci_ad_out  = next_data_in ;
                                pci_cbe_out = next_be_in ;
                            end
    endcase
end

// data output mux for reads
always@(mabort_out or pci_ad_reg_in)
begin
    if ( mabort_out )
        data_out = 32'hFFFF_FFFF ;
    else
        data_out = pci_ad_reg_in ;
end
endmodule