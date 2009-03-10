//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "test_bench.v"                                    ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - mihad@opencores.org                                   ////
////      - Miha Dolenc                                           ////
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
// Revision 1.5  2001/07/30 15:24:34  mihad
// no message
//
//

// Common definitions for simulation purposes
`define Tpci 75 // Tp/2 = 7.5ns => Tp = 15ns => Fpci = 66MHz
`define Twb 25  // Tp/2 = 2.5ns =>   Tp = 5ns  => Fwb  = 200MHz

//Running this test bench requires all include files in same directory
`include "constants.v"
`include "conf_space.v"
`include "wb_slave.v"
`include "wb_master32.v"
`include "wbw_wbr_fifos.v"
`include "wb_bus_mon.v"
`include "bus_commands.v"
`include "decoder.v"
`include "delayed_sync.v"
`include "delayed_write_reg.v"

/*===================================================================================================
This test bench is intended for HOST bridge implementation simulation. ( definition in constants.v )
The reason is that configuration space operation, configuration and interrupt acknowledge cycles
request generation are also tested in this testbench. This features are only implemented in HOST 
bridge implementation. 
===================================================================================================*/

module TEST_BENCH () ;

// clocks
reg wb_clock ;
reg pci_clock ;

// reset
reg reset ;

/*==================================================================================================
Simulated signals - signals on PCI side of the bridge are simulated in behavioral fashion
==================================================================================================*/
// read enable for wbw_fifo
reg wbw_renable ;

// write enable for wbr_fifo
reg wbr_wenable ;

// data input to wbr fifo
reg [31:0] wbr_data_in ;

// byte enable input to wbr fifo
reg [3:0] wbr_be_in ;

// control input to wbr fifo
reg [3:0] wbr_control_in ;

// pciw fifo empty indicator
reg pciw_empty ;

// variable for feeding delayed trans. logic with status signal
reg del_error_signal ;

// variable for communicating to completing task that it should terminate with an error
reg error_terminate ;

// pci delayed read completion pending
reg pci_drcomp_pending ;

// lock simulation - signal locks out all but configuration accesses from WISHBONE bus
reg wbs_lock ;

// pci side configuration space address, data and read enable signals
reg  [11:0] pci_conf_raddr ;
reg         pci_conf_renable ;
wire [31:0] pci_conf_data_probe ;

// pci side completion simulation variables
reg del_comp_done, del_rty_exp;

// variables for various error reporting mechanisms
reg perr_set, serr_set, master_abort_recv, target_abort_recv, target_abort_set, master_data_par_err ;

// outputs from configuration space for pci interface
wire [7:0] cache_line_size_probe ;
wire [7:0] latency_tim_probe ;
wire [2:0] int_pin_probe ;

// pci address decoding values - just for checking since PCI side is only simulated
wire [19:0] pci_ba0, pci_ba1, pci_ba2, pci_ba3, pci_ba4, pci_ba5 ;
wire [19:0] pci_am0, pci_am1, pci_am2, pci_am3, pci_am4, pci_am5 ;
wire [19:0] pci_ta0, pci_ta1, pci_ta2, pci_ta3, pci_ta4, pci_ta5 ;
wire [5:0]  pci_map ;
wire [1:0]  pci_img_ctrl0, pci_img_ctrl1, pci_img_ctrl2, pci_img_ctrl3, pci_img_ctrl4, pci_img_ctrl5 ;

// pci error reporting variables - simulated
reg [3:0]  pci_error_be ;
reg [3:0]  pci_error_bc ;
reg        pci_error_rty_exp ;
reg        pci_error_sig ;
reg [31:0] pci_error_addr ;
reg [31:0] pci_error_data ;
wire pci_error_rty_exp_set ;

// error log enable signal
wire pci_error_en ;

// WISHBONE error reporting variables
reg [3:0] wb_error_be ;
reg [3:0] wb_error_bc ;
reg wb_error_rty_exp ;
reg wb_error_es ;
reg wb_error_sig ;
reg [31:0] wb_error_addr ;
reg [31:0] wb_error_data ;
wire wb_error_rty_exp_set ;

// interrupt status generation
reg isr_int_prop ;
reg isr_err_int  ;
reg isr_par_err_int ;
reg isr_sys_err_int ;


/*============================================================================================
Module interconnect signals - wires, buses, muxes for module interconnects
============================================================================================*/

// configuration hit wire
wire conf_hit ;

// delayed read bus command bus
wire [3:0] del_bc ;

// data for delayed read requests and completions
wire [31:0] del_addr ;
wire [3:0]  del_be ;

// delayed read flag wires
wire    del_req, 
        del_comp, 
        read_in_burst, 
        pci_req ;

// configuration space data input - from WISHBONE slave, since this is HOST implementation simulation
wire [31:0] conf_data;

// address from MUX, WISHBONE slave output and input data buses and SEL bus
reg     [31:0]  address ;

// image access data and bus command/byte enable buses
wire [31:0] image_data ;
wire [3:0]  image_cbe ;

// WISHBONE Write FIFO outputs
wire [31:0] wbw_data_probe ;
wire [3:0]  wbw_cbe_probe ;
wire [3:0]  wbw_control_probe ;

// WISHBONE Read FIFO - WISHBONE slave interconnect
wire [31:0] wbr_data ;
wire [3:0]  wbr_be ;
wire [3:0]  wbr_control ;

// delayed read request bus command output
wire [3:0] del_bc_probe ;

// configuration offset and byte enable interconnect
wire [11:0] conf_offset_probe ;
wire [3:0] conf_be_probe ;

// configuration cycle address interconnect
wire [23:0] config_addr ;

// control bus interconnect between WBW_FIFO and WISHBONE slave
wire [3:0] wbw_control ;

// configuration space data output
wire [31:0] conf_data_probe ;

// image hit signals
wire [4:0] hit ;

// WISHBONE bus interconnect between behav. master and WISHBONE slave
wire    [31:0]  addr_o ;            // address
wire    [31:0]  sdat_o ;            // DAT_O from slave
wire    [31:0]  sdat_i ;            // DAT_I to slave
wire    [3:0]   sel ;               // SEL_I to slave
wire            cyc ;               // CYC_I to slave
wire            stb ;               // STB_I to slave
wire            we  ;               // WE_I to slave
wire            ack ;               // ACK_O from slave
wire            rty ;               // RTY_O from slave
wire            err ;               // ERR_O from slave
wire            cab ;               // CAB_I to slave

// map wires
wire [5:1] wb_map ;

// wb image control signals
wire [2:0] wb_img_ctrl0, wb_img_ctrl1, wb_img_ctrl2, wb_img_ctrl3, wb_img_ctrl4, wb_img_ctrl5 ;
wire [4:0] wb_mrl = { wb_img_ctrl5[0], wb_img_ctrl4[0], wb_img_ctrl3[0], wb_img_ctrl2[0], wb_img_ctrl1[0] } ;
wire [4:0] wb_pref = { wb_img_ctrl5[1], wb_img_ctrl4[1], wb_img_ctrl3[1], wb_img_ctrl2[1], wb_img_ctrl1[1] } ;

`ifdef FPGA
    assign glbl.GSR = reset ;
`endif

// WISHBONE read FIFO flush wire
wire wbs_wbr_flush, delayed_sync_wbr_flush ;
wire wbr_flush = wbs_wbr_flush || delayed_sync_wbr_flush ;


// memory for storing various data
/*-----------------------------------------------------------------------------------------------------------------------------
Memory is devided into four pieces - two for writes and two for reads - one of each for IO and one for MEMORY accesses.
Test bench first copies data from Write memory, through WISHBONE slave and WBW_FIFO to simulated side of the bridge. This side
fills read memory. Then Test Bench reads data through WBR_FIFO and WISHBONE slave module and compares it to written data
-----------------------------------------------------------------------------------------------------------------------------*/
reg [31:0] wmem_data [0:1023] ; // data for memory mapped image writes
reg [31:0] wio_data  [0:1023] ; // data for IO mapped image writes

reg [7:0] rmem_data [0:4095] ;  // data for memory mapped image reads
reg [7:0] rio_data  [0:4095] ;  // data for IO mapped image reads

reg [31:0] conf_read_data ;     // data for configuration cycle read request

reg [31:0] iack_read_data ;     // data for interrupt acknowledge cycle read request

// memory array for FIFO contents checking - FIFO contents is checked whenever something is read from it
reg [39:0] wbw_contents[0:`WBW_DEPTH - 1] ;
reg [`WBW_ADDR_LENGTH - 1:0] wbw_write_pointer ;
reg [`WBW_ADDR_LENGTH - 1:0] wbw_read_pointer ;

// WISHBONE slave interface instantiation
WB_SLAVE wishbone_slave(
                        .wb_clock_in              (wb_clock) ,
                        .reset_in                 (reset) ,
                        .wb_hit_in                (hit) ,
                        .wb_conf_hit_in           (conf_hit) ,
                        .wb_map_in                (wb_map) ,
                        .wb_pref_en_in            (wb_pref) ,
                        .wb_mrl_en_in             (wb_mrl) ,
                        .wb_addr_in               (address) ,
                        .del_bc_in                (del_bc) ,
                        .wb_del_req_pending_in    (del_req) ,
                        .wb_del_comp_pending_in   (del_comp) ,
                        .pci_drcomp_pending_in    (pci_drcomp_pending) ,
                        .del_bc_out               (del_bc_probe) ,
                        .del_req_out              (del_req_probe) ,
                        .del_done_out             (del_done_probe) ,
                       	.del_burst_out            (del_burst_probe) ,
                        .del_write_out            (del_write_probe),
                        .del_write_in             (del_write),
                        .del_error_in             (del_error),
                        .wb_del_addr_in           (del_addr) ,
                        .wb_del_be_in             (del_be) ,
                        .wb_conf_offset_out       (conf_offset_probe) ,
                        .wb_conf_renable_out      (conf_renable_probe) ,
                        .wb_conf_wenable_out      (conf_wenable_probe) ,
                        .wb_conf_be_out           (conf_be_probe) ,
                        .wb_conf_data_in          (conf_data) ,
                        .wb_conf_data_out         (conf_data_probe) ,
                        .wb_data_out              (image_data) ,
                        .wb_cbe_out               (image_cbe) ,
                        .wbw_fifo_wenable_out     (wbw_wenable) ,
                        .wbw_fifo_control_out     (wbw_control) ,
                        .wbw_fifo_almost_full_in  (wbw_almost_full) ,
                        .wbw_fifo_full_in         (wbw_full) ,
                        .wbr_fifo_renable_out     (wbr_renable) ,
                        .wbr_fifo_be_in           (wbr_be) ,
                        .wbr_fifo_data_in         (wbr_data) ,
                        .wbr_fifo_control_in      (wbr_control) ,
                        .wbr_fifo_flush_out       (wbs_wbr_flush) ,
                        .wbr_fifo_almost_empty_in (wbr_almost_empty), 
                        .wbr_fifo_empty_in        (wbr_empty),     
                        .pciw_fifo_empty_in       (pciw_empty),
                        .wbs_lock_in              (wbs_lock),
                        .del_in_progress_out      (del_in_progress),
                        .ccyc_addr_in             ({8'h00, config_addr}),
                        .CYC_I                    (cyc),
                        .STB_I                    (stb),
                        .WE_I                     (we),
                        .SEL_I                    (sel),
                        .SDATA_I                  (sdat_i),
                        .SDATA_O                  (sdat_o),
                        .ACK_O                    (ack),
                        .RTY_O                    (rty),
                        .ERR_O                    (err),
                        .CAB_I                    (cab)
                       );

// behavioral WISHBONE master instantiation
WB_MASTER32 wishbone_master(
                            .CLK_I(wb_clock), 
                            .RST_I(reset), 
                            .TAG_I(4'b0), 
                            .TAG_O(),
		                    .ACK_I(ack), 
                            .ADR_O(addr_o), 
                            .CYC_O(cyc), 
                            .DAT_I(sdat_o), 
                            .DAT_O(sdat_i), 
                            .ERR_I(err), 
                            .RTY_I(rty), 
                            .SEL_O(sel), 
                            .STB_O(stb), 
                            .WE_O (we),
                            .CAB_O(cab)
                           ) ;

// WBW_FIFO and WBR_FIFO instantiation
WBW_WBR_FIFOS fifos(
                    .wb_clock_in               (wb_clock),
                    .pci_clock_in              (pci_clock),
                    .reset_in                  (reset),
                    .wbw_wenable_in            (wbw_wenable),
                    .wbw_addr_data_in          (image_data),
                    .wbw_cbe_in                (image_cbe),
                    .wbw_control_in            (wbw_control),         
                    .wbw_renable_in            (wbw_renable),
                    .wbw_addr_data_out         (wbw_data_probe),
                    .wbw_cbe_out               (wbw_cbe_probe),
                    .wbw_control_out           (wbw_control_probe),          
                    .wbw_flush_in              (1'b0),
                    .wbw_almost_full_out       (wbw_almost_full),
                    .wbw_full_out              (wbw_full),
                    .wbw_almost_empty_out      (wbw_almost_empty_probe),
                    .wbw_empty_out             (wbw_empty_probe),
                    .wbw_transaction_ready_out (wbw_transaction_probe),
                    .wbr_wenable_in            (wbr_wenable), 
                    .wbr_data_in               (wbr_data_in),
                    .wbr_be_in                 (wbr_be_in),
                    .wbr_control_in            (wbr_control_in),         
                    .wbr_renable_in            (wbr_renable),
                    .wbr_data_out              (wbr_data),
                    .wbr_be_out                (wbr_be),
                    .wbr_control_out           (wbr_control),          
                    .wbr_flush_in              (wbr_flush),
                    .wbr_almost_full_out       (wbr_almost_full_probe),
                    .wbr_full_out              (wbr_full_probe),
                    .wbr_almost_empty_out      (wbr_almost_empty),
                    .wbr_empty_out             (wbr_empty),
                    .wbr_transaction_ready_out (wbr_transaction_probe)
                   ) ;

// bus monitor instantiation
WB_BUS_MON wishbone_monitor(
                            .CLK_I  (wb_clock),
                            .RST_I  (reset),
		                    .ACK_I  (ack),
                            .ADDR_O (addr_o), 
                            .CYC_O  (cyc),
                            .DAT_I  (sdat_o),
                            .DAT_O  (sdat_i),
                            .ERR_I  (err),
                            .RTY_I  (rty),
                            .SEL_O  (sel),
                            .STB_O  (stb),
                            .WE_O   (we),
                            .TAG_I  (1'b0),
                            .TAG_O  (1'b0),
                            .CAB_O  (cab)
                           ) ;

// address outputs from decoders
wire [31:0] conf_addr ;
wire [31:0] img_addr1 ;
wire [31:0] img_addr2 ;
wire [31:0] img_addr3 ;
wire [31:0] img_addr4 ;
wire [31:0] img_addr5 ;

// input wires to decoders
wire [19:0] wb_ba0, wb_ba1, wb_ba2, wb_ba3, wb_ba4, wb_ba5 ; 
wire [19:0] wb_am0, wb_am1, wb_am2, wb_am3, wb_am4, wb_am5 ; 
wire [19:0] wb_ta0, wb_ta1, wb_ta2, wb_ta3, wb_ta4, wb_ta5 ; 

//address decoder instantiation
DECODER dec1(
                .hit       (hit[0]),
                .addr_out  (img_addr1),
                .addr_in   (addr_o),
                .base_addr (wb_ba1),
                .mask_addr (wb_am1),
                .tran_addr (wb_ta1),
                .at_en     (wb_img_ctrl1[2])
            ) ;

DECODER dec2(
                .hit       (hit[1]),
                .addr_out  (img_addr2),
                .addr_in   (addr_o),
                .base_addr (wb_ba2),
                .mask_addr (wb_am2),
                .tran_addr (wb_ta2),
                .at_en     (wb_img_ctrl2[2])
            ) ;

DECODER dec3(
                .hit       (hit[2]),
                .addr_out  (img_addr3),
                .addr_in   (addr_o),
                .base_addr (wb_ba3),
                .mask_addr (wb_am3),
                .tran_addr (wb_ta3),
                .at_en     (wb_img_ctrl3[2])
            ) ;

DECODER dec4(
                .hit       (hit[3]),
                .addr_out  (img_addr4),
                .addr_in   (addr_o),
                .base_addr (wb_ba4),
                .mask_addr (wb_am4),
                .tran_addr (wb_ta4),
                .at_en     (wb_img_ctrl4[2])
            ) ;

DECODER dec5(
                .hit       (hit[4]),
                .addr_out  (img_addr5),
                .addr_in   (addr_o),
                .base_addr (wb_ba5),
                .mask_addr (wb_am5),
                .tran_addr (wb_ta5),
                .at_en     (wb_img_ctrl5[2])
            ) ;

// address mux
always@(hit or conf_hit or img_addr1 or img_addr2 or img_addr3 or img_addr4 or img_addr5 or conf_addr)
begin
    case(hit)
        5'h01:address <= img_addr1 ;    
        5'h02:address <= img_addr2 ;
        5'h04:address <= img_addr3 ;
        5'h08:address <= img_addr4 ;
        5'h10:address <= img_addr5 ;
        default: address <= conf_addr ;
    endcase
end

// configuration space decoder
DECODER dec0(
                .hit       (conf_hit),
                .addr_out  (conf_addr),
                .addr_in   (addr_o),
                .base_addr (wb_ba0),
                .mask_addr (wb_am0),
                .tran_addr (wb_ta0),
                .at_en     (wb_img_ctrl0[2])
            ) ;

// delayed transaction logic instantiation
DELAYED_SYNC del_sync  (
                                .reset_in             (reset),
                                .req_clk_in           (wb_clock),
                                .comp_clk_in          (pci_clock),
                                .req_in               (del_req_probe),
                                .comp_in              (del_comp_done),
                                .done_in              (del_done_probe),
                                .in_progress_in       (del_in_progress),
                                .comp_req_pending_out (pci_req),
                                .req_req_pending_out  (del_req),
                                .req_comp_pending_out (del_comp),
                                .addr_in              (image_data),
                                .be_in                (conf_be_probe),
                                .addr_out             (del_addr),
                                .be_out               (del_be),
                                .we_in                (del_write_probe),
                                .we_out               (del_write),
                                .bc_in                (del_bc_probe),
                                .bc_out               (del_bc),
                                .status_in            (del_error_signal),
                                .status_out           (del_error),
                                .comp_flush_out       (delayed_sync_wbr_flush),
                                .burst_in             (del_burst_probe),
                                .burst_out            (read_in_burst),
                                .retry_expired_in     (del_rty_exp)
                            );

wire [31:0] del_write_data ;
DELAYED_WRITE_REG delayed_write_data
(
	.reset_in       (reset),
	.req_clk_in     (wb_clock),
	.comp_wdata_out (del_write_data),
	.req_we_in      (del_req_probe && del_write_probe),
	.req_wdata_in   (conf_data_probe)
);




CONF_SPACE configuration    (	
                                .w_conf_address_in   (conf_offset_probe),
                                .w_conf_data_in      (conf_data_probe),
                                .w_conf_data_out     (conf_data),
                                .r_conf_address_in   (pci_conf_raddr),
                                .r_conf_data_out     (pci_conf_data_probe),
					            .w_we                (conf_wenable_probe),
                                .w_re                (conf_renable_probe),
                                .r_re                (pci_conf_renable),
                                .w_byte_en           (~conf_be_probe),
                                .conf_hit            (conf_hit),
                                .w_clock             (wb_clock),
                                .reset               (reset),
                                .pci_clk             (pci_clock),
                                .wb_clk              (wb_clock),
                                // outputs from command register of the PCI header  
					            .serr_enable         (serr_enable),
                                .perr_response       (perr_response),
                                .pci_master_enable   (pci_master_enable),
                                .memory_space_enable (memory_space_enable),
                                .io_space_enable     (io_space_enable),
                                // inputs to status register of the PCI header      
					            .perr_in             (perr_set),
                                .serr_in             (serr_set),
                                .master_abort_recv   (master_abort_recv),
                                .target_abort_recv   (target_abort_recv),
                                .target_abort_set    (target_abort_set),
                                .master_data_par_err (master_data_par_err),
                                // output from cache_line_size, latency timer and   
                                // r_interrupt_pin register of the PCI header       
					            .cache_line_size     (cache_line_size_probe),
                                .latency_tim         (latency_tim_probe),
                                .int_pin             (int_pin_probe),
                                // output from all pci IMAGE registers
					            .pci_base_addr0 (pci_ba0),
                                .pci_base_addr1 (pci_ba1),
                                .pci_base_addr2 (pci_ba2),
                                .pci_base_addr3 (pci_ba3),
                                .pci_base_addr4 (pci_ba4),
                                .pci_base_addr5 (pci_ba5),
					            .pci_memory_io0 (pci_map[0]),
                                .pci_memory_io1 (pci_map[1]),
                                .pci_memory_io2 (pci_map[2]),
                                .pci_memory_io3 (pci_map[3]),
                                .pci_memory_io4 (pci_map[4]),
                                .pci_memory_io5 (pci_map[5]),
					            .pci_addr_mask0 (pci_am0),
                                .pci_addr_mask1 (pci_am1),
                                .pci_addr_mask2 (pci_am2),
                                .pci_addr_mask3 (pci_am3),
                                .pci_addr_mask4 (pci_am4),
                                .pci_addr_mask5 (pci_am5),
					            .pci_tran_addr0 (pci_ta0),
                                .pci_tran_addr1 (pci_ta1),
                                .pci_tran_addr2 (pci_ta2),
                                .pci_tran_addr3 (pci_ta3),
                                .pci_tran_addr4 (pci_ta4),
                                .pci_tran_addr5 (pci_ta5),
					            .pci_img_ctrl0  (pci_img_ctrl0),
                                .pci_img_ctrl1  (pci_img_ctrl1),
                                .pci_img_ctrl2  (pci_img_ctrl2),
                                .pci_img_ctrl3  (pci_img_ctrl3),
                                .pci_img_ctrl4  (pci_img_ctrl4),
                                .pci_img_ctrl5  (pci_img_ctrl5),
                                // input to pci error control and status register, 
                                // error address and error data registers
					            .pci_error_be      (pci_error_be),
                                .pci_error_bc      (pci_error_bc),
                                .pci_error_rty_exp (pci_error_rty_exp),
                                .pci_error_sig     (pci_error_sig),
                                .pci_error_addr    (pci_error_addr),
                                .pci_error_data    (pci_error_data),
                                .pci_error_rty_exp_set (pci_error_rty_exp_set),
                                // output from pci error control and status register
					            .pci_error_en     (pci_error_en),
                                // output from all wishbone IMAGE registers                                                                                          
					            .wb_base_addr0    (wb_ba0),
                                .wb_base_addr1    (wb_ba1),
                                .wb_base_addr2    (wb_ba2),
                                .wb_base_addr3    (wb_ba3),
                                .wb_base_addr4    (wb_ba4),
                                .wb_base_addr5    (wb_ba5),
					            .wb_memory_io0    (),
                                .wb_memory_io1    (wb_map[1]),
                                .wb_memory_io2    (wb_map[2]),
                                .wb_memory_io3    (wb_map[3]),
                                .wb_memory_io4    (wb_map[4]),
                                .wb_memory_io5    (wb_map[5]),
					            .wb_addr_mask0    (wb_am0),
                                .wb_addr_mask1    (wb_am1),
                                .wb_addr_mask2    (wb_am2),
                                .wb_addr_mask3    (wb_am3),
                                .wb_addr_mask4    (wb_am4),
                                .wb_addr_mask5    (wb_am5),
					            .wb_tran_addr0    (wb_ta0),
                                .wb_tran_addr1    (wb_ta1),
                                .wb_tran_addr2    (wb_ta2),
                                .wb_tran_addr3    (wb_ta3),
                                .wb_tran_addr4    (wb_ta4),
                                .wb_tran_addr5    (wb_ta5),
					            .wb_img_ctrl0     (wb_img_ctrl0),
                                .wb_img_ctrl1     (wb_img_ctrl1),
                                .wb_img_ctrl2     (wb_img_ctrl2),
                                .wb_img_ctrl3     (wb_img_ctrl3),
                                .wb_img_ctrl4     (wb_img_ctrl4),
                                .wb_img_ctrl5     (wb_img_ctrl5),
                                // input to wb error control and status register,
                                // error address and error data registers        (),
					            .wb_error_be      (wb_error_be),
                                .wb_error_bc      (wb_error_bc),
                                .wb_error_rty_exp (wb_error_rty_exp),
                                .wb_error_es      (wb_error_es),
                                .wb_error_sig     (wb_error_sig),
                                .wb_error_addr    (wb_error_addr),
                                .wb_error_data    (wb_error_data),
                                .wb_error_rty_exp_set (wb_error_rty_exp_set), 
                                // output from wb error control and status register
					            .wb_error_en      (wb_error_en),
                                // output from conf. cycle generation register 
                                // (sddress) & int. control register
					            .config_addr  (config_addr),
                                .icr_soft_res (icr_soft_res),
                                .serr_int_en  (serr_int_en), 
                                .perr_int_en  (perr_int_en),
                                .error_int_en (error_int_en),
                                .int_prop_en  (int_prop_en),
                                // input to interrupt status register
					            .isr_int_prop (isr_int_prop),
                                .isr_err_int  (isr_err_int), 
                                .isr_par_err_int (isr_par_err_int),
                                .isr_sys_err_int (isr_sys_err_int),
                                .pci_error_sig_set (pci_error_sig_set),
                                .wb_error_sig_set (wb_error_sig_set)
                            ) ;

integer temp_index ;
// initial state
initial
begin
    // guest implementation not supported in this testbench
    `ifdef GUEST
        $display("GUEST implementation not supported in this testbench") ;
        $finish ;
    `endif
    // clocks
    wb_clock <= 1'b0 ;
    pci_clock <= 1'b1 ;
      
    // read enable for wbw_fifo
    wbw_renable <= 1'b0 ;
    
    // write enable for wbr_fifo
    wbr_wenable <= 1'b0 ;
    
    // byte enable input to wbr fifo
    wbr_be_in <= 4'hF ;
    
    // control input to wbr fifo
    wbr_control_in <= 4'h0 ;
    
    // pciw fifo empty indicator
    pciw_empty <= 1'b1 ;
    
    // pci side delayed read completion pending flag
    pci_drcomp_pending <= 1'b0 ;
    
    // lock variable
    wbs_lock  <= 1'b0 ;
    
    // various inputs to configuration space are initialized to 0s
    perr_set <= 1'b0 ;
    serr_set <= 1'b0 ; 
    master_abort_recv <= 1'b0 ; 
    target_abort_recv <= 1'b0 ; 
    target_abort_set <= 1'b0 ; 
    master_data_par_err <= 1'b0 ;
    pci_error_sig <= 1'b0 ;
    wb_error_rty_exp <= 1'b0 ;
    wb_error_es <= 1'b0 ;
    wb_error_sig <= 1'b0 ;

    isr_int_prop <= 1'b0 ;
    isr_err_int <= 1'b0  ;
    isr_par_err_int <= 1'b0 ;
    isr_sys_err_int <= 1'b0 ;

    pci_error_rty_exp <= 1'b0 ;

    pci_error_be <= 4'h0 ;
    pci_error_bc <= 4'h0 ;
    pci_error_addr <= 32'h00000000 ;
    pci_error_data <= 32'h00000000 ;

    wb_error_be <= 4'h0 ;
    wb_error_bc <= 4'h0 ;
    wb_error_addr <= 32'h00000000 ;
    wb_error_data <= 32'h00000000 ;

    // error terminate variable for delayed reads
    del_error_signal <= 1'b0 ;

    error_terminate  <= 1'b0 ;

    // read completion signal
    del_comp_done <= 1'b0 ;

    // retry counter expired variable
    del_rty_exp <= 1'b0 ;

    // fill write memories with random data
    for( temp_index = 0; temp_index <=1023; temp_index = temp_index + 1 )
    begin
        wmem_data[temp_index[9:0]] = $random ;
        wio_data[temp_index[9:0]]  = $random ;
    end

    // run tests
    run_tests ;
    $stop ;

end //initial

// clocks generation
always 
begin
   #`Tpci pci_clock = ~pci_clock ;
end

always 
begin
   #`Twb wb_clock = ~wb_clock ;
end

task run_tests ;
begin
    // first - reset logic
    do_reset ;
    conf_space_test ;
    wb_error_log_test ;
    pci_error_log_test ;
    pci_status_reg_test ;
    interrupt_status_reg_test ;
    image_testing ;
    conf_cycle_test ;
    iack_cycle_test ;
    error_termination_test ;
    dlyd_req_rty_exp_test ;
    force_write_with_read_test ;
end
endtask //run_tests

// task for warning displays
task display_warning;
    input [31:0] error_address ;
    input [31:0] expected_data ;
    input [31:0] actual ;
begin
    $display("Read from address %h produced wrong result! \nExpected value was %h\tread value was %h!", error_address, expected_data, actual) ;
end
endtask // display warning

// reset task
task do_reset;
begin
    reset <= 1'b0 ;
    #1 reset <= 1'b1 ;
    #100 reset <= 1'b0 ;
end
endtask

// constant for control and status register check
wire [31:0] header_cs = { 10'b0000_0010_00, `HEADER_66MHz, 5'b0_0000, 16'h0147 };
task conf_space_test ;
    reg [31:0] current_address ;
    integer i ;
    reg [2:0] result ;
    reg [31:0] write_data ;
    reg [31:0] read_data ;
    reg [3:0] read_select ;
    
    begin 
        $display("Starting configuration space test") ;

        // apply start address
        current_address = {`WB_CONFIGURATION_BASE, 12'h000} ;

        // loop for performing 4KB (1Kx4) of writes - whole configuration space address range
        i = 0 ;
        result = 3'b000 ;
        while ( (i < 1024) && (result[1:0] == 2'b00) )
        begin
            // check if this is configuration cycle generation access - skip it by inserting wait state
            if ( current_address[11:0] == `CNF_DATA_ADDR )
                write_data = 32'hxxxx_xxxx ;
            else
                write_data = 32'hffff_ffff ;
            
            // do a write
            wishbone_master.blkwr( current_address, write_data, 4'hF, 1'b0, (i == 0), ((i + 1) == 1024), result ) ;
            
            // increment burst address
            current_address = current_address + 4 ;
            
            i = i + 1 ;
        end //while

        // check if whole image has been done
        if (i < 1024)
            $display("WISHBONE slave responded with error or retry to early! It didn't allow whole configuration space access!") ;
    
        // check outputs from conf space 
        // wait one clock edge for cycle to finish
        @(posedge wb_clock) ;

        // check if every output from configuration space is as it should be
        if (~serr_enable)
            $display("System error enable bit wasn't set although written one to it!") ;
        if (~perr_response)
            $display("Parity error response bit wasn't set although written one to it!") ;
        if (~pci_master_enable)
            $display("PCI master enable bit wasn't set although written one to it!") ;
        if (~memory_space_enable)
            $display("Memory space enable bit wasn't set although written one to it!") ;
        if (~io_space_enable)
            $display("I/O space enable bit wasn't set although written one to it!") ;

        if (cache_line_size_probe !== 8'hFF)
            $display("Cacheline size output didn't reflect a value written to cacheline size register!") ;
        if (latency_tim_probe !== 8'hFF)
            $display("Latency timer output didn't reflect a value written to latency timer register!") ;
        if (int_pin_probe !== 3'b001)
            $display("Interrupt pin output didn't reflect a value written to interrupt pin register!") ;

        if (pci_ba0 !== 20'hFFFF_F)
            $display("PCI base address 0 register didn't reflect a value written to it!") ;
        if (pci_ba1 !== 20'hFFFF_F)
            $display("PCI base address 1 register didn't reflect a value written to it!") ;
        if (pci_ba2 !== 20'hFFFF_F)
            $display("PCI base address 2 register didn't reflect a value written to it!") ;
        if (pci_ba3 !== 20'hFFFF_F)
            $display("PCI base address 3 register didn't reflect a value written to it!") ;
        if (pci_ba4 !== 20'hFFFF_F)
            $display("PCI base address 4 register didn't reflect a value written to it!") ;
        if (pci_ba5 !== 20'hFFFF_F)
            $display("PCI base address 5 register didn't reflect a value written to it!") ;
        if (pci_map[5:0] !== 6'b1111_11)
            $display("PCI address space mapping values didn't reflect values written to them!") ;
        if (pci_am0 !== 20'hFFFF_F)
            $display("PCI address mask 0 register didn't reflect a value written to it!") ;
        if (pci_am1 !== 20'hFFFF_F)
            $display("PCI address mask 1 register didn't reflect a value written to it!") ;
        if (pci_am2 !== 20'hFFFF_F)
            $display("PCI address mask 2 register didn't reflect a value written to it!") ;
        if (pci_am3 !== 20'hFFFF_F)
            $display("PCI address mask 3 register didn't reflect a value written to it!") ;
        if (pci_am4 !== 20'hFFFF_F)
            $display("PCI address mask 4 register didn't reflect a value written to it!") ;
        if (pci_am5 !== 20'hFFFF_F)
            $display("PCI address mask 5 register didn't reflect a value written to it!") ;
		if (pci_ta0 !== 20'hFFFF_F)
            $display("PCI translation address 0 register didn't reflect a value written to it!") ;
        if (pci_ta1 !== 20'hFFFF_F)
            $display("PCI translation address 1 register didn't reflect a value written to it!") ;
        if (pci_ta2 !== 20'hFFFF_F)
            $display("PCI translation address 2 register didn't reflect a value written to it!") ;
        if (pci_ta3 !== 20'hFFFF_F)
            $display("PCI translation address 3 register didn't reflect a value written to it!") ;
        if (pci_ta4 !== 20'hFFFF_F)
            $display("PCI translation address 4 register didn't reflect a value written to it!") ;
        if (pci_ta5 !== 20'hFFFF_F)
            $display("PCI translation address 5 register didn't reflect a value written to it!") ;
		if (pci_img_ctrl0 !== 2'b11)
            $display("PCI image control 0 register didn't reflect a value written to it!") ;
        if (pci_img_ctrl1 !== 2'b11)
            $display("PCI image control 1 register didn't reflect a value written to it!") ;
        if (pci_img_ctrl2 !== 2'b11)
            $display("PCI image control 2 register didn't reflect a value written to it!") ;
        if (pci_img_ctrl3 !== 2'b11)
            $display("PCI image control 3 register didn't reflect a value written to it!") ;
        if (pci_img_ctrl4 !== 2'b11)
            $display("PCI image control 4 register didn't reflect a value written to it!") ;
        if (pci_img_ctrl5 !== 2'b11)
            $display("PCI image control 5 register didn't reflect a value written to it!") ;

        if (~pci_error_en)
            $display("PCI error loging enable output didn't reflect a value written to it!") ;

        if (wb_ba0 !== `WB_CONFIGURATION_BASE)
            $display("WISHBONE base address 0 register didn't reflect a value of conf. space base address!") ;
        if (wb_ba1 !== 20'hFFFF_F)
            $display("WISHBONE base address 1 register didn't reflect a value written to it!") ;
        if (wb_ba2 !== 20'hFFFF_F)
            $display("WISHBONE base address 2 register didn't reflect a value written to it!") ;
        if (wb_ba3 !== 20'hFFFF_F)
            $display("WISHBONE base address 3 register didn't reflect a value written to it!") ;
        if (wb_ba4 !== 20'hFFFF_F)
            $display("WISHBONE base address 4 register didn't reflect a value written to it!") ;
        if (wb_ba5 !== 20'hFFFF_F)
            $display("WISHBONE base address 5 register didn't reflect a value written to it!") ;
        if (wb_map[5:1] !== 5'b1111_1)
            $display("WISHBONE address space mapping values didn't reflect values written to them!") ;
        if (wb_am0 !== 20'hFFFF_F)
            $display("WISHBONE address mask 0 register didn't reflect a value corresponding to 4KB size of conf. space!") ;
        if (wb_am1 !== 20'hFFFF_F)
            $display("WISHBONE address mask 1 register didn't reflect a value written to it!") ;
        if (wb_am2 !== 20'hFFFF_F)
            $display("WISHBONE address mask 2 register didn't reflect a value written to it!") ;
        if (wb_am3 !== 20'hFFFF_F)
            $display("WISHBONE address mask 3 register didn't reflect a value written to it!") ;
        if (wb_am4 !== 20'hFFFF_F)
            $display("WISHBONE address mask 4 register didn't reflect a value written to it!") ;
        if (wb_am5 !== 20'hFFFF_F)
            $display("WISHBONE address mask 5 register didn't reflect a value written to it!") ;
		if (wb_ta0 !== 20'h0000_0)
            $display("WISHBONE translation address 0 register didn't reflect a value of 0x00000!") ;
        if (wb_ta1 !== 20'hFFFF_F)
            $display("WISHBONE translation address 1 register didn't reflect a value written to it!") ;
        if (wb_ta2 !== 20'hFFFF_F)
            $display("WISHBONE translation address 2 register didn't reflect a value written to it!") ;
        if (wb_ta3 !== 20'hFFFF_F)
            $display("WISHBONE translation address 3 register didn't reflect a value written to it!") ;
        if (wb_ta4 !== 20'hFFFF_F)
            $display("WISHBONE translation address 4 register didn't reflect a value written to it!") ;
        if (wb_ta5 !== 20'hFFFF_F)
            $display("WISHBONE translation address 5 register didn't reflect a value written to it!") ;
		if (wb_img_ctrl0 !== 3'b000)
            $display("WISHBONE image control 0 register didn't reflect a value right for configuration space!") ;
        if (wb_img_ctrl1 !== 3'b111)
            $display("WISHBONE image control 1 register didn't reflect a value written to it!") ;
        if (wb_img_ctrl2 !== 3'b111)
            $display("WISHBONE image control 2 register didn't reflect a value written to it!") ;
        if (wb_img_ctrl3 !== 3'b111)
            $display("WISHBONE image control 3 register didn't reflect a value written to it!") ;
        if (wb_img_ctrl4 !== 3'b111)
            $display("WISHBONE image control 4 register didn't reflect a value written to it!") ;
        if (wb_img_ctrl5 !== 3'b111)
            $display("WISHBONE image control 5 register didn't reflect a value written to it!") ;

        if (~wb_error_en)
            $display("WB error logging enable output didn't reflect a value written to it!") ;
		if ({8'h00, config_addr} !== 32'h00FF_FFFD)
            $display("Configuration cycle address output didn't reflect a value written to register!") ;
        if (~icr_soft_res)
            $display("Software reset output didn't reflect a value written to it!") ;
        if (~serr_int_en)
            $display("System error interrupt enable output didn't reflect a value written to it!") ;
        if (~perr_int_en)
            $display("Parity error interrupt enable output didn't reflect a value written to it!") ;
        if (~error_int_en)
            $display("WB error interrupt enable output didn't reflect a value written to it!") ;        
        if (~int_prop_en)
            $display("Interrupt propagation interrupt enable output didn't reflect a value written to it!") ;        
        
        // do actual reads from conf_space and compare read values with expected ones
        // apply start address
        current_address = {`WB_CONFIGURATION_BASE, 12'h000} ;

        // loop for performing 4KB (1Kx4) of reads - whole configuration space address range
        i = 0 ;
        result = 3'b000 ;
        while ( (i < 1024) && (result[1:0] == 2'b00) )
        begin
            // check if this is configuration cycle or interrupt acknowledge generation access - skip them by inserting wait states
            if ( (current_address[11:0] == `CNF_DATA_ADDR) || (current_address[11:0] == `INT_ACK_ADDR) )
                read_select = 4'hx ;
            else
                read_select = 4'hf ;
            
            // do a write
            wishbone_master.blkrd( current_address, read_select, 1'b0, (i == 0), ((i + 1) == 1024), {result, read_data} ) ;
            
            // check response
            if (result[2]) // transfer acknowledged
            begin
                // check what address was currently read and see, if data is OK
                case (current_address)
                    // type0 header test
                    { `WB_CONFIGURATION_BASE, 12'h000 }:if (read_data !== {`HEADER_DEVICE_ID, `HEADER_VENDOR_ID})
                                                            display_warning(current_address, {`HEADER_DEVICE_ID, `HEADER_VENDOR_ID}, read_data) ;

                    { `WB_CONFIGURATION_BASE, 12'h004 }:if (read_data !== header_cs)
                                                            display_warning(current_address, header_cs, read_data) ;

                    { `WB_CONFIGURATION_BASE, 12'h008 }:`ifdef HOST
                                                            if (read_data !== {24'h06_00_00, `HEADER_REVISION_ID} )
                                                                display_warning(current_address, {24'h06_00_00, `HEADER_REVISION_ID}, read_data) ;
                                                        `else
                                                            if (read_data !== {24'h06_80_00, `HEADER_REVISION_ID} )
                                                                display_warning(current_address, {24'h06_80_00, `HEADER_REVISION_ID}, read_data) ;
                                                        `endif

                    { `WB_CONFIGURATION_BASE, 12'h00C }:if (read_data !== 32'h0000_FFFF)
                                                            display_warning(current_address, 32'h0000_FFFF, read_data) ;

                    { `WB_CONFIGURATION_BASE, 12'h010 }, 
                    { `WB_CONFIGURATION_BASE, 12'h014 }, 
                    { `WB_CONFIGURATION_BASE, 12'h018 }, 
                    { `WB_CONFIGURATION_BASE, 12'h01C }, 
                    { `WB_CONFIGURATION_BASE, 12'h020 }, 
                    { `WB_CONFIGURATION_BASE, 12'h024 }:if (read_data !== 32'hFFFF_F001)
                                                            display_warning(current_address, 32'hFFFF_F001, read_data) ;

                    { `WB_CONFIGURATION_BASE, 12'h03C }:if (read_data !== 32'h1A08_01FF)
                                                            display_warning(current_address, 32'h1A08_01FF, read_data) ;

                    // all pci image control registers must return same value
                    { `WB_CONFIGURATION_BASE, `P_IMG_CTRL0_ADDR }, 
                    { `WB_CONFIGURATION_BASE, `P_IMG_CTRL1_ADDR }, 
                    { `WB_CONFIGURATION_BASE, `P_IMG_CTRL2_ADDR }, 
                    { `WB_CONFIGURATION_BASE, `P_IMG_CTRL3_ADDR }, 
                    { `WB_CONFIGURATION_BASE, `P_IMG_CTRL4_ADDR }, 
                    { `WB_CONFIGURATION_BASE, `P_IMG_CTRL5_ADDR }:  if (read_data !== 32'h0000_0006)
                                                                        display_warning(current_address, 32'h0000_0006, read_data) ;
                                                                            
                    // all pci base address registers must return same value
                    { `WB_CONFIGURATION_BASE, `P_BA0_ADDR },				
                    { `WB_CONFIGURATION_BASE, `P_BA1_ADDR },				
                    { `WB_CONFIGURATION_BASE, `P_BA2_ADDR },				
                    { `WB_CONFIGURATION_BASE, `P_BA3_ADDR },				
                    { `WB_CONFIGURATION_BASE, `P_BA4_ADDR },				
                    { `WB_CONFIGURATION_BASE, `P_BA5_ADDR }:if (read_data !== 32'hFFFF_F001)
                                                                display_warning(current_address, 32'hFFFF_F001, read_data) ;    

                    // all pci address mask registers must return same value
                    { `WB_CONFIGURATION_BASE, `P_AM0_ADDR },				
                    { `WB_CONFIGURATION_BASE, `P_AM1_ADDR },				
                    { `WB_CONFIGURATION_BASE, `P_AM2_ADDR },				
                    { `WB_CONFIGURATION_BASE, `P_AM3_ADDR },				
                    { `WB_CONFIGURATION_BASE, `P_AM4_ADDR },				
        			{ `WB_CONFIGURATION_BASE, `P_AM5_ADDR }:if (read_data !== 32'hFFFF_F000)
                                                                display_warning(current_address, 32'hFFFF_F000, read_data) ;    

                    // all pci translation address registers must return same value            
                    { `WB_CONFIGURATION_BASE, `P_TA0_ADDR },
				    { `WB_CONFIGURATION_BASE, `P_TA1_ADDR },
                    { `WB_CONFIGURATION_BASE, `P_TA2_ADDR },
                    { `WB_CONFIGURATION_BASE, `P_TA3_ADDR },
                    { `WB_CONFIGURATION_BASE, `P_TA4_ADDR },
				    { `WB_CONFIGURATION_BASE, `P_TA5_ADDR }:if (read_data !== 32'hFFFF_F000)
                                                                display_warning(current_address, 32'hFFFF_F000, read_data) ;    
                    
                    { `WB_CONFIGURATION_BASE, `P_ERR_CS_ADDR }: if (read_data !== 32'h0000_0001)
                                                                    display_warning(current_address, 32'h0000_0001, read_data) ;    
                    
                    { `WB_CONFIGURATION_BASE, `P_ERR_ADDR_ADDR }:   if (read_data !== 32'h0000_0000)
                                                                        display_warning(current_address, 32'h0000_0000, read_data) ;    
                    
                    { `WB_CONFIGURATION_BASE, `P_ERR_DATA_ADDR }:   if (read_data !== 32'h0000_0000)
                                                                        display_warning(current_address, 32'h0000_0000, read_data) ;       

                    { `WB_CONFIGURATION_BASE, `WB_CONF_SPC_BAR_ADDR }:  if (read_data !== {`WB_CONFIGURATION_BASE, 12'h000})
                                                                            display_warning(current_address, {`WB_CONFIGURATION_BASE, 12'h000}, read_data) ;       

                    // all WISHBONE image configuration registers must return same value
				    { `WB_CONFIGURATION_BASE, `W_IMG_CTRL1_ADDR },
				    { `WB_CONFIGURATION_BASE, `W_IMG_CTRL2_ADDR },		
                    { `WB_CONFIGURATION_BASE, `W_IMG_CTRL3_ADDR },		
			        { `WB_CONFIGURATION_BASE, `W_IMG_CTRL4_ADDR },		
                    { `WB_CONFIGURATION_BASE, `W_IMG_CTRL5_ADDR }:  if (read_data !== 32'h0000_0007)
                                                                        display_warning(current_address, 32'h0000_0007, read_data) ;      
                    
                    // all WISHBONE base address registers must return same value
                    { `WB_CONFIGURATION_BASE, `W_BA1_ADDR },
                    { `WB_CONFIGURATION_BASE, `W_BA2_ADDR },
                    { `WB_CONFIGURATION_BASE, `W_BA3_ADDR },
                    { `WB_CONFIGURATION_BASE, `W_BA4_ADDR },
                    { `WB_CONFIGURATION_BASE, `W_BA5_ADDR }:if (read_data !== 32'hFFFF_F001)
                                                                display_warning(current_address, 32'hFFFF_F001, read_data) ;    

                    // all WISHBONE address mask registers must return same value
                    { `WB_CONFIGURATION_BASE, `W_AM1_ADDR },
                    { `WB_CONFIGURATION_BASE, `W_AM2_ADDR },
				    { `WB_CONFIGURATION_BASE, `W_AM3_ADDR },
                    { `WB_CONFIGURATION_BASE, `W_AM4_ADDR },
                    { `WB_CONFIGURATION_BASE, `W_AM5_ADDR }:if (read_data !== 32'hFFFF_F000)
                                                                display_warning(current_address, 32'hFFFF_F000, read_data) ;    

                    // all WISHBONE translation address registers must return same value
                    { `WB_CONFIGURATION_BASE, `W_TA1_ADDR },
                    { `WB_CONFIGURATION_BASE, `W_TA2_ADDR },				
                    { `WB_CONFIGURATION_BASE, `W_TA3_ADDR },				
                    { `WB_CONFIGURATION_BASE, `W_TA4_ADDR },				
                    { `WB_CONFIGURATION_BASE, `W_TA5_ADDR }:if (read_data !== 32'hFFFF_F000)
                                                                display_warning(current_address, 32'hFFFF_F000, read_data) ;    

                    { `WB_CONFIGURATION_BASE, `W_ERR_CS_ADDR }:if (read_data !== 32'h0000_0001)
                                                                    display_warning(current_address, 32'h0000_0001, read_data) ;     
                                                                        
                    { `WB_CONFIGURATION_BASE, `W_ERR_ADDR_ADDR }:   if (read_data !== 32'h0000_0000)
                                                                        display_warning(current_address, 32'h0000_0000, read_data) ;    
    
                    { `WB_CONFIGURATION_BASE, `W_ERR_DATA_ADDR }:   if (read_data !== 32'h0000_0000)
                                                                        display_warning(current_address, 32'h0000_0000, read_data) ;    

                    { `WB_CONFIGURATION_BASE, `CNF_ADDR_ADDR }: if (read_data !== 32'h00FF_FFFD)
                                                                    display_warning(current_address, 32'h00FF_FFFD, read_data) ;    

                    { `WB_CONFIGURATION_BASE, `ICR_ADDR }:  if (read_data !== 32'h8000_000F)
                                                                display_warning(current_address, 32'h8000_000F, read_data) ;    

                    { `WB_CONFIGURATION_BASE, `ISR_ADDR }:  if (read_data !== 32'h0000_0000)
                                                                display_warning(current_address, 32'h0000_0000, read_data) ;    

                    default:// access to unimplemented space  
                            if (read_data !== 32'h0000_0000)
                                display_warning(current_address, 32'h0000_0000, read_data) ;    
                endcase
            end // transfer acknowledged
            
            // increment burst address
            current_address = current_address + 4 ;
            
            i = i + 1 ;
        end //while
        
        // check if whole access was done succesfully
        if (i < 1024)
            $display("WISHBONE slave responded with error or retry too early! It didn't allow whole configuration space access!") ;
        
        @(posedge wb_clock) ;

        $display("Configuration space test done") ;

    end
endtask //conf_space_test 

task wb_error_log_test;
    reg [31:0] current_address ;
    reg [2:0] result ;
    reg [31:0] read_data ;

begin
        $display("Starting WISHBONE error reporting test!") ;
        
        // error is set on pci_clock, since PCI Master is reporting it
        @(posedge pci_clock) ;
        
        // signal an error
        wb_error_sig  <= #`FF_DELAY 1'b1 ;
        // define error source
        wb_error_es   <= #`FF_DELAY 1'b1 ;

        //provide address and data of the access when error occured
        wb_error_addr <= #`FF_DELAY 32'hFFFF_FFFF ;
        wb_error_data <= #`FF_DELAY 32'hFFFF_FFFF ;
        
        // provide byte enables and bus command of an access
        wb_error_be <= #`FF_DELAY 4'hF ;
        wb_error_bc <= #`FF_DELAY 4'hF ;

        // wait for posedge
        // here error should be written to registers
        @(posedge pci_clock) ;
    
        // dismiss error generation and change data values also
        wb_error_sig  <= #`FF_DELAY 1'b0 ;
        wb_error_es   <= #`FF_DELAY 1'b0 ;
        wb_error_addr <= #`FF_DELAY 32'h0000_0000 ;
        wb_error_data <= #`FF_DELAY 32'h0000_0000 ;
        wb_error_be <= #`FF_DELAY 4'h0 ;
        wb_error_bc <= #`FF_DELAY 4'h0 ;

        // wait for negedge - give FFs and outputs enough setup time
        @(negedge pci_clock) ;
        
        // first check corresponding outputs from configuration space
        if (~wb_error_sig_set)
            $display("WISHBONE error pending output didn't have a value of 1 when error was signalled!") ;

        // now do a read from these registers and see if right data is provided
        // read from error control and status register
        current_address = { `WB_CONFIGURATION_BASE, `W_ERR_CS_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b0, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFF00_0301)
                display_warning(current_address, 32'hFF00_0301, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        //read from error data register
        current_address = { `WB_CONFIGURATION_BASE, `W_ERR_DATA_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b0, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFFFF_FFFF)
                display_warning(current_address, 32'hFFFF_FFFF, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        // read from error address register
        current_address = { `WB_CONFIGURATION_BASE, `W_ERR_ADDR_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b1, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFFFF_FFFF)
                display_warning(current_address, 32'hFFFF_FFFF, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        // now write one to corresponding status bit and see if it turns off
        // do a write
        current_address = { `WB_CONFIGURATION_BASE, `W_ERR_CS_ADDR } ;
        wishbone_master.blkwr( current_address + 1, 32'h0000_0100, 4'b0010, 1'b0, 1'b1, 1'b0, result ) ;
        
        // check if access was acknowledged
        if (~result[2])
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;
        
        @(negedge wb_clock) ;

        // probe for error pending output
        if (wb_error_sig_set)
            $display("WISHBONE error pending output didn't have a value of 0 when cleared by writing 1 to it!") ;
        
        // now do a read from same register
        current_address = { `WB_CONFIGURATION_BASE, `W_ERR_CS_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b1, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFF00_0201)
                display_warning(current_address, 32'hFF00_0201, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        // do a test of retry error signalization
        @(posedge pci_clock) ;

        // signal an error
        wb_error_rty_exp <= #`FF_DELAY 1'b1 ;
        
        // wait for posedge
        // here error is written to register
        @(posedge pci_clock) ;
    
        // dismiss error generation
        wb_error_rty_exp <= #`FF_DELAY 1'b0 ;

        // wait for negedge - give FFs and outputs enough setup time
        @(negedge pci_clock) ;
        
        // first check corresponding outputs from configuration space
        if (~wb_error_rty_exp_set)
            $display("WISHBONE retry expired output didn't show a value of 1 when retry expired was signaled") ;

        // now do a read from these registers and see if right data is provided
        
        // read from error control and status register
        current_address = { `WB_CONFIGURATION_BASE, `W_ERR_CS_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b0, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFF00_0601)
                display_warning(current_address, 32'hFF00_0601, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        //read from error data register
        current_address = { `WB_CONFIGURATION_BASE, `W_ERR_DATA_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b0, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFFFF_FFFF)
                display_warning(current_address, 32'hFFFF_FFFF, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        // read from error address register
        current_address = { `WB_CONFIGURATION_BASE, `W_ERR_ADDR_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b1, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFFFF_FFFF)
                display_warning(current_address, 32'hFFFF_FFFF, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        // now write one to corresponding status bit and see if it turns off
        // do a write
        current_address = { `WB_CONFIGURATION_BASE, `W_ERR_CS_ADDR } ;
        wishbone_master.blkwr( current_address, 32'hFFFF_FFFF, 4'hF, 1'b0, 1'b1, 1'b0, result ) ;

        // check if transfer was acknowledged
        if (~result[2])
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;
        
        @(negedge pci_clock) ;
        // probe for error pending output
        if (wb_error_rty_exp_set)
            $display("WISHBONE retry expired error output wasn't cleared when 1 was written to its location") ;
        
        // now do a read from same register
        current_address = { `WB_CONFIGURATION_BASE, `W_ERR_CS_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFF00_0201)
                display_warning(current_address, 32'hFF00_0201, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        $display("WISHBONE error reporting test finished!") ;
end
endtask //wb_error_log_test

task pci_error_log_test;
    reg [31:0] current_address ;
    reg [2:0] result ;
    reg [31:0] read_data ;
begin
        // PCI error signaling test - WISHBONE master does it so it's sync'd to WISHBONE clock
        
        $display("Starting PCI error reporting test!") ;
        
        @(posedge wb_clock) ;
        
        // signal an error
        pci_error_sig  <= #`FF_DELAY 1'b1 ;
        
        //provide address and data of the access when error occured
        pci_error_addr <= #`FF_DELAY 32'hFFFF_FFFF ;
        pci_error_data <= #`FF_DELAY 32'hFFFF_FFFF ;
        
        // provide byte enables and bus command of an access
        pci_error_be <= #`FF_DELAY 4'hF ;
        pci_error_bc <= #`FF_DELAY 4'hF ;

        // wait for posedge
        // here error is written to registers
        @(posedge wb_clock) ;
    
        // dismiss error generation and data inputs
        pci_error_sig  <= #`FF_DELAY 1'b0 ;
        pci_error_addr <= #`FF_DELAY 32'h0000_0000 ;
        pci_error_data <= #`FF_DELAY 32'h0000_0000 ;
        pci_error_be <= #`FF_DELAY 4'h0 ;
        pci_error_bc <= #`FF_DELAY 4'h0 ;

        // wait for negedge - give FFs and outputs enough setup time
        @(negedge wb_clock) ;
        
        // first check corresponding outputs from configuration space
        if (~pci_error_sig_set)
            $display("PCI error pending output didn't have a value of 1 when error was signalled!") ;

        // now do a read from these registers and see if right data is provided
        // read from error control and status register
        current_address = { `WB_CONFIGURATION_BASE, `P_ERR_CS_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b0, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFF00_0101)
                display_warning(current_address, 32'hFF00_0101, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        //read from error data register
        current_address = { `WB_CONFIGURATION_BASE, `P_ERR_DATA_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b0, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFFFF_FFFF)
                display_warning(current_address, 32'hFFFF_FFFF, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        // read from error address register
        current_address = { `WB_CONFIGURATION_BASE, `P_ERR_ADDR_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b1, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFFFF_FFFF)
                display_warning(current_address, 32'hFFFF_FFFF, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;
        
        // now write one to corresponding status bit and see if it turns off
        // do a write
        current_address = { `WB_CONFIGURATION_BASE, `P_ERR_CS_ADDR } ;
        wishbone_master.blkwr( current_address + 1, 32'h0000_0100, 4'b0010, 1'b0, 1'b1, 1'b0, result ) ;

        @(negedge wb_clock) ;
        // probe for error pending output
        if (pci_error_sig_set)
            $display("PCI error pending output didn't have a value of 0 when cleared by writing 1 to it!") ;

        if (~result[2])
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;
        
        // now do a read from same register
        current_address = { `WB_CONFIGURATION_BASE, `P_ERR_CS_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b1, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFF00_0001)
                display_warning(current_address, 32'hFF00_0001, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        @(posedge wb_clock) ;
        
        // signal an error
        pci_error_rty_exp <= #`FF_DELAY 1'b1 ;

        // wait for posedge
        // here error is written to registers
        @(posedge wb_clock) ;
    
        // dismiss error generation
        pci_error_rty_exp <= #`FF_DELAY 1'b0 ;

        // wait for negedge - give FFs and outputs enough setup time
        @(negedge wb_clock) ;
        
        // first check corresponding outputs from configuration space
        if (~pci_error_rty_exp_set)
            $display("PCI retry expired error output didn't have a value of 1 when retry error was signalled!") ;

        // now do a read from these registers and see if right data is provided
        // read from error control and status register
        current_address = { `WB_CONFIGURATION_BASE, `P_ERR_CS_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b0, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFF00_0401)
                display_warning(current_address, 32'hFF00_0401, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        //read from error data register
        current_address = { `WB_CONFIGURATION_BASE, `P_ERR_DATA_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b0, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFFFF_FFFF)
                display_warning(current_address, 32'hFFFF_FFFF, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        // read from error address register
        current_address = { `WB_CONFIGURATION_BASE, `P_ERR_ADDR_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b1, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFFFF_FFFF)
                display_warning(current_address, 32'hFFFF_FFFF, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;
        
        // now write one to corresponding status bit and see if it turns off
        // do a write
        current_address = { `WB_CONFIGURATION_BASE, `P_ERR_CS_ADDR } ;
        wishbone_master.blkwr( current_address, 32'hFFFF_FFFF, 4'hF, 1'b0, 1'b1, 1'b0, result ) ;

        @(negedge wb_clock) ;

        // probe for error pending output
        if (pci_error_rty_exp_set)
            $display("PCI retry expired error output didn't have a value of 0 when retry error bit was cleared by writing 1 to it!") ;

        if (~result[2])
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;
        
        // now do a read from same register
        current_address = { `WB_CONFIGURATION_BASE, `P_ERR_CS_ADDR } ;
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b1, {result, read_data} ) ;

        if (result[2]) 
        begin
            if (read_data !== 32'hFF00_0001)
                display_warning(current_address, 32'hFF00_0001, read_data) ;    
        end
        else
            $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

        $display("PCI error reporting test finished!") ;

end
endtask // pci_error_log_test

task pci_status_reg_test;
    reg [31:0] current_address ;
    reg [31:0] read_data ;
    reg [2:0] result ;
begin
    $display("Starting PCI header status register test!") ;

    // test if status register in type0 header is functioning
    // set all error and status signals
    @(posedge pci_clock) ;
    perr_set <= #`FF_DELAY 1'b1 ;
    serr_set <= #`FF_DELAY 1'b1 ;
    master_abort_recv <= #`FF_DELAY 1'b1 ;
    target_abort_recv <= #`FF_DELAY 1'b1 ;
    target_abort_set  <= #`FF_DELAY 1'b1 ;
    master_data_par_err <= #`FF_DELAY 1'b1 ;
    
    // at posedge of clock this signals are latched
    @(posedge pci_clock);
    
    // clear signals
    perr_set <= #`FF_DELAY 1'b0 ;
    serr_set <= #`FF_DELAY 1'b0 ;
    master_abort_recv <= #`FF_DELAY 1'b0 ;
    target_abort_recv <= #`FF_DELAY 1'b0 ;
    target_abort_set  <= #`FF_DELAY 1'b0 ;
    master_data_par_err <= #`FF_DELAY 1'b0 ;

    // read from type0 header
    current_address = { `WB_CONFIGURATION_BASE, 12'h004 } ;
    wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;

    if (result[2]) 
    begin
        if (read_data[31:24] !== 8'hFB)
            display_warning(current_address, {8'hFB, read_data[23:0]}, read_data) ;    
    end
    else
        $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

    // Now write ones to status bits to clear them
    wishbone_master.blkwr( current_address + 3, 32'hFF00_0000, 4'b1000, 1'b0, 1'b1, 1'b0, result ) ;

    if (~result[2])
        $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;
    
    // now read value back
    wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b1, {result, read_data} ) ;

    if (result[2]) 
    begin
        if (read_data[31:24] !== 8'h02)
            display_warning(current_address, {8'h02, read_data[23:0]}, read_data) ;    
    end
    else
        $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

    $display("PCI header status register test finished!") ;
end
endtask //pci_status_reg_test

task interrupt_status_reg_test;
    reg [31:0] current_address ;
    reg [31:0] read_data ;
    reg [2:0] result ;
begin
    $display("Starting interrupt status register test!") ;

    // test if interrupt status register is functioning
    // set all interrupt sources
    @(posedge wb_clock) ;
    isr_int_prop <= #`FF_DELAY 1'b1 ;
    isr_err_int  <= #`FF_DELAY 1'b1 ;
    isr_par_err_int <= #`FF_DELAY 1'b1 ;
    isr_sys_err_int <= #`FF_DELAY 1'b1 ;
    
    // at posedge of clock this signals are latched
    @(posedge wb_clock);
    
    // clear signals
    isr_int_prop <= #`FF_DELAY 1'b0 ;
    isr_err_int  <= #`FF_DELAY 1'b0 ;
    isr_par_err_int <= #`FF_DELAY 1'b0 ;
    isr_sys_err_int <= #`FF_DELAY 1'b0 ;

    // read from interrupt status register
    current_address = { `WB_CONFIGURATION_BASE, `ISR_ADDR } ;
    wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;

    if (result[2]) 
    begin
        if (read_data !== 32'h0000_000F)
            display_warning(current_address, 32'h0000_000F, read_data) ;    
    end
    else
        $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

    // Now write ones to status bits to clear them
    wishbone_master.blkwr( current_address, 32'hFFFF_FFFF, 4'hF, 1'b0, 1'b1, 1'b0, result ) ;

    if (~result[2])
        $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

    // now read value back
    wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b0, 1'b1, {result, read_data} ) ;

    if (result[2]) 
    begin
        if (read_data !== 32'h0000_0000)
            display_warning(current_address, 32'h0000_0000, read_data) ;    
    end
    else
        $display("WISHBONE slave didn't acknowledge an access to address %h although it was expected to!", current_address) ;

    $display("Interrupt status register test finished!") ;
end
endtask //interrupt_status_reg_test

// task for testing normal operation - reads and writes through images
task image_testing ;
    reg [31:0] current_address ;
    reg [2:0] result ;
begin

    $display("Starting image accesses testing!") ;
    $display("Configuring images!") ;

    // first configure 4 images that will be used in testing and disable fifth
    // configure image 1
    // write image control register
    current_address = { `WB_CONFIGURATION_BASE, `W_IMG_CTRL1_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_0003, 4'h1, 1'b0, 1'b1, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write base address register
    current_address = { `WB_CONFIGURATION_BASE, `W_BA1_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_0000, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write address mask register
    current_address = { `WB_CONFIGURATION_BASE, `W_AM1_ADDR } ;
    wishbone_master.blkwr( current_address, 32'hFFFF_F000, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // no need for translation addrress because translation is disabled
    
    // configure image 2
    // write image control register
    current_address = { `WB_CONFIGURATION_BASE, `W_IMG_CTRL2_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_0004, 4'h1, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write base address register
    current_address = { `WB_CONFIGURATION_BASE, `W_BA2_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_1000, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write address mask register
    current_address = { `WB_CONFIGURATION_BASE, `W_AM2_ADDR } ;
    wishbone_master.blkwr( current_address, 32'hFFFF_F000, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write address translation register - point to same address space as first image
    current_address = { `WB_CONFIGURATION_BASE, `W_TA2_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_0000, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // configure image 3
    // write image control register
    current_address = { `WB_CONFIGURATION_BASE, `W_IMG_CTRL3_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_0000, 4'h1, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write base address register - map to IO space
    current_address = { `WB_CONFIGURATION_BASE, `W_BA3_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_2001, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // write address mask register
    current_address = { `WB_CONFIGURATION_BASE, `W_AM3_ADDR } ;
    wishbone_master.blkwr( current_address, 32'hFFFF_F000, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // no need for translation addrress because translation is disabled
    

    // configure image 4
    // write image control register
    current_address = { `WB_CONFIGURATION_BASE, `W_IMG_CTRL4_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_0004, 4'h1, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write base address register
    current_address = { `WB_CONFIGURATION_BASE, `W_BA4_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_3001, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write address mask register
    current_address = { `WB_CONFIGURATION_BASE, `W_AM4_ADDR } ;
    wishbone_master.blkwr( current_address, 32'hFFFF_F000, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write address translation register - point to same address space as first I/O image
    current_address = { `WB_CONFIGURATION_BASE, `W_TA4_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_2000, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    
    // disable image 5
    // write image control register
    current_address = { `WB_CONFIGURATION_BASE, `W_IMG_CTRL5_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_0000, 4'h1, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write base address register
    current_address = { `WB_CONFIGURATION_BASE, `W_BA5_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_0000, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write address mask register
    current_address = { `WB_CONFIGURATION_BASE, `W_AM5_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_0000, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write address translation register
    current_address = { `WB_CONFIGURATION_BASE, `W_TA5_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_0000, 4'hf, 1'b0, 1'b0, 1'b1, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    $display("Image configurations finished!") ;

    $display("Starting writes through memory mapped images!") ;

    // perform first burst write through image 1
    burst_write(32'h0000_0000, 10'h200) ;
    
    // perform second burst write through image 2 - start at offset 800 where previous finished
    burst_write(32'h0000_1800, 10'h200) ;

    $display("Writes through memory mapped images finished!") ;

    $display("Starting reads through same memory mapped images!") ;
    // now read written locations through one image
    burst_read(32'h0000_0000, 11'h400, 4'hF) ;

    // read written locations through second image
    burst_read(32'h0000_1000, 11'h400, 4'hF) ;
        
    $display("Reads through same memory mapped images finished!") ;
    
    test_io_transfers ;
    
    $display("Image accesses testing finished!") ;
end
endtask // image_testing

task test_io_transfers;
    reg [2:0] result ;
begin

    $display("Testing IO mapped images") ;
    $display("Starting writes through IO mapped images!") ;
    wishbone_master.blkwr( 32'h0000_2000, wio_data[10'h000], 4'hF, 1'b0, 1'b1, 1'b1, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", 32'h0000_2000) ;

    wishbone_master.blkwr( 32'h0000_2004, wio_data[10'h001], 4'hF, 1'b0, 1'b1, 1'b1, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", 32'h0000_2004) ;

    wishbone_master.blkwr( 32'h0000_3008, wio_data[10'h002], 4'hF, 1'b0, 1'b1, 1'b1, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", 32'h0000_3008) ;

    wishbone_master.blkwr( 32'h0000_300C, wio_data[10'h003], 4'hF, 1'b0, 1'b1, 1'b1, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", 32'h0000_300C) ;

    $display("Writes through IO mapped images finished!") ;

    $display("Starting reads through same IO mapped images!") ;
    
    // first read all four entries through one image
    read_io(32'h0000_2000, 4'hF) ;
    read_io(32'h0000_2004, 4'hF) ;
    read_io(32'h0000_2008, 4'hF) ;
    read_io(32'h0000_200C, 4'hF) ;

    // first read all four entries through second image
    read_io(32'h0000_3000, 4'hF) ;
    read_io(32'h0000_3004, 4'hF) ;
    read_io(32'h0000_3008, 4'hF) ;
    read_io(32'h0000_300C, 4'hF) ;

    $display("Reads through same IO mapped images finished!") ;
    $display("IO mapped images testing complete") ;

end
endtask //test_io_transfers

task conf_cycle_test ;
    reg [31:0] cnf_wdata ;
    reg [2:0] result ;
    reg [31:0] read_data ;
begin
    $display("Starting configuration cycle test!") ;
        
    // get random data
    cnf_wdata = $random ;

    // first write to conf_data register
    // retry it until acknowledge is signalled
    result = 3'b001 ;
    while (result[0]) 
        wishbone_master.blkwr( {`WB_CONFIGURATION_BASE, `CNF_DATA_ADDR}, cnf_wdata, 4'hF, 1'b0, 1'b1, 1'b1, result ) ;
    
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", {`WB_CONFIGURATION_BASE, `CNF_DATA_ADDR}) ;

    // now read and compare data written by conf write
    result = 3'b001 ;
    while (result[0]) 
        wishbone_master.blkrd( {`WB_CONFIGURATION_BASE, `CNF_DATA_ADDR}, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;
    
    if (result[2])
    begin
        if (read_data !== cnf_wdata)
            display_warning({`WB_CONFIGURATION_BASE, `CNF_DATA_ADDR}, cnf_wdata, read_data) ;
    end
    else
        $display("WISHBONE slave didn't acknowledge read access to address %h although it was expected to!", {`WB_CONFIGURATION_BASE, `CNF_DATA_ADDR}) ;

    // test operation if target abort is signalled on completing a request
    error_terminate = 1'b1 ;

    result = 3'b001 ;
    while (result[0]) 
        wishbone_master.blkwr( {`WB_CONFIGURATION_BASE, `CNF_DATA_ADDR}, cnf_wdata, 4'hF, 1'b0, 1'b1, 1'b1, result ) ;
    
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on TARGET ABORT termination of configuration cycle!") ;

    // now read and see if error is signalled
    result = 3'b001 ;
    while (result[0]) 
        wishbone_master.blkrd( {`WB_CONFIGURATION_BASE, `CNF_DATA_ADDR}, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;
    
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on TARGET ABORT termination of configuration cycle!") ;
    
    error_terminate = 1'b0 ;
    $display("Configuration cycle test finished!") ;
    
end
endtask //conf_cycle_test

task iack_cycle_test ;
    reg [2:0] result ;
    reg [31:0] read_data ;
begin
    $display("Starting interrupt acknowledge cycle test!") ;
        
    // get random data
    iack_read_data = $random ;

    // first write to iack register
    // retry it until acknowledge is signalled
    // write should be acknowledged immediately since writes to this register musn't have any effect
    result = 3'b001 ;
    while (result[0]) 
        wishbone_master.blkwr( {`WB_CONFIGURATION_BASE, `INT_ACK_ADDR}, 32'h0000_0000, 4'hF, 1'b0, 1'b1, 1'b1, result ) ;
    
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", {`WB_CONFIGURATION_BASE, `INT_ACK_ADDR}) ;

    // now read from interrupt acknowledge register and see if right data is provided by WB_SLAVE
    result = 3'b001 ;
    while (result[0]) 
        wishbone_master.blkrd( {`WB_CONFIGURATION_BASE, `INT_ACK_ADDR}, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;
    
    if (result[2])
    begin
        if (read_data !== iack_read_data)
            display_warning({`WB_CONFIGURATION_BASE, `INT_ACK_ADDR}, iack_read_data, read_data) ;
    end
    else
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", {`WB_CONFIGURATION_BASE, `INT_ACK_ADDR}) ;

    // test operation if target abort is signalled on iack request completing on PCI
    error_terminate = 1'b1 ;
    result = 3'b001 ;
    while (result[0]) 
        wishbone_master.blkwr( {`WB_CONFIGURATION_BASE, `INT_ACK_ADDR}, 32'h0000_0000, 4'hF, 1'b0, 1'b1, 1'b1, result ) ;
    
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", {`WB_CONFIGURATION_BASE, `INT_ACK_ADDR}) ;

    // now read and see if error is signalled
    result = 3'b001 ;
    while (result[0]) 
        wishbone_master.blkrd( {`WB_CONFIGURATION_BASE, `INT_ACK_ADDR}, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;
    
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on TARGET ABORT termination of interrupt acknowledge cycle!") ;
    
    error_terminate = 1'b0 ;
    $display("Interrupt acknowledge cycle test finished!") ;
    
end
endtask //iack_cycle_test

task error_termination_test ;
    reg [31:0] write_data;
    reg [31:0] read_data;
    reg [2:0] result;
    reg [31:0] current_address ;
    
begin
    // test various conditions on which WISHBONE slave should signal an error - other testbench code is monitoring this conditions
    $display("Starting error termination tests!") ;

    // first test access to nonalligned memory addresses
    result = 3'b000 ;
    wishbone_master.blkwr( 32'h0000_0001, 32'h0000_0000, 4'hF, 1'b0, 1'b1, 1'b1, result ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to memory mapped image with nonalligned address") ;

    wishbone_master.blkwr( 32'h0000_0002, 32'h0000_0000, 4'hF, 1'b0, 1'b1, 1'b1, result ) ;
    
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to memory mapped image with nonalligned address") ;

    wishbone_master.blkwr( 32'h0000_0003, 32'h0000_0000, 4'hF, 1'b0, 1'b1, 1'b1, result ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to memory mapped image with nonalligned address") ;

    // now check if anything is in a wbw_fifo - there shouldn't be anything in there since all of these were error terminations
    @(posedge pci_clock) ;
    @(posedge pci_clock) ;
    @(posedge pci_clock) ;
    if (~wbw_empty_probe)
        $display("Although access was an error, it was accepted and written to wbw_fifo!") ;

    // do reads to same addresses
    result = 3'b000 ;
    wishbone_master.blkrd( 32'h0000_0001, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to memory mapped image with nonalligned address") ;

    wishbone_master.blkrd( 32'h0000_0002, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;
    
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to memory mapped image with nonalligned address") ;

    wishbone_master.blkrd( 32'h0000_0003, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to memory mapped image with nonalligned address") ;
    
    // see if read request was generated - it shouldn't since it was an error
    if (del_req)
        $display("Read request was accepted although read access was an error!") ;

    // try one more read - this time a burst one - it has to be an error also
    wishbone_master.blkrd( 32'h0000_0003, 4'hF, 1'b1, 1'b1, 1'b0, {result, read_data} ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to memory mapped image with nonalligned address") ;
    
    // see if read request was generated - it shouldn't since it was an error
    if (del_req)
        $display("Read request was accepted although read access was an error!") ;

    // now test accesses to IO space with wrong byte enables

    result = 3'b000 ;
    wishbone_master.blkwr( 32'h0000_2001, 32'h0000_0000, 4'h1, 1'b0, 1'b1, 1'b1, result ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to IO mapped image with wrong byte enables") ;

    wishbone_master.blkwr( 32'h0000_2002, 32'h0000_0000, 4'h3, 1'b0, 1'b1, 1'b1, result ) ;
    
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to IO mapped image with wrong byte enables") ;

    wishbone_master.blkwr( 32'h0000_2003, 32'h0000_0000, 4'h7, 1'b0, 1'b1, 1'b1, result ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to IO mapped image with wrong byte enables") ;

    // now check if anything is in a wbw_fifo - there shouldn't be anything in there since all of these were error terminations
    @(posedge pci_clock) ;
    @(posedge pci_clock) ;
    @(posedge pci_clock) ;
    
    if (~wbw_empty_probe)
        $display("Although access was an error, it was accepted and written to wbw_fifo!") ;

    // now do reads through this image
    result = 3'b000 ;
    wishbone_master.blkrd( 32'h0000_2001, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to IO mapped image with wrong byte enables") ;

    wishbone_master.blkrd( 32'h0000_2002, 4'h7, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;
    
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to IO mapped image with wrong byte enables") ;

    wishbone_master.blkrd( 32'h0000_2003, 4'h1, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to IO mapped image with wrong byte enables") ;
    
    // see if read request was generated - it shouldn't since it was an error
    if (del_req)
        $display("Read request was accepted although read access was an error!") ;

    // test burst accesses to IO space
    wishbone_master.blkrd( 32'h0000_2000, 4'hF, 1'b1, 1'b1, 1'b0, {result, read_data} ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on burst access to IO mapped image!") ;
    
    // see if read request was generated - it shouldn't since it was an error
    if (del_req)
        $display("Read request was accepted although read access was an error!") ;

    wishbone_master.blkwr( 32'h0000_2000, 32'h0000_0000, 4'hF, 1'b1, 1'b1, 1'b0, result ) ;
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on burst access to IO mapped image!") ;

    // now test accesses to configuration space with wrong byte enables

    result = 3'b000 ;
    wishbone_master.blkwr( {`WB_CONFIGURATION_BASE, 12'h001}, 32'h0000_0000, 4'h1, 1'b0, 1'b1, 1'b1, result ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to configuration space with wrong byte enables") ;

    wishbone_master.blkwr( {`WB_CONFIGURATION_BASE, 12'h002}, 32'h0000_0000, 4'h3, 1'b0, 1'b1, 1'b1, result ) ;
    
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to configuration space with wrong byte enables") ;

    wishbone_master.blkwr( {`WB_CONFIGURATION_BASE, 12'h003}, 32'h0000_0000, 4'h7, 1'b0, 1'b1, 1'b1, result ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to configuration space with wrong byte enables") ;

    // do one read from conf_space with wrong BE
    wishbone_master.blkrd( {`WB_CONFIGURATION_BASE, 12'h003}, 4'h7, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;

    if (~result[1])
        $display("WISHBONE slave didn't signal an error on access to configuration space with wrong byte enables") ;

    // now attempt a burst write and read to configuration space
    wishbone_master.blkwr( {`WB_CONFIGURATION_BASE, 12'h000}, 32'h0000_0000, 4'hF, 1'b1, 1'b1, 1'b0, result ) ;
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on burst access to configuration space!") ;

    wishbone_master.blkrd( {`WB_CONFIGURATION_BASE, 12'h000}, 4'hF, 1'b1, 1'b1, 1'b0, {result, read_data} ) ;
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on burst access to configuration space!") ;

    // now do a delayed read and complete it with an error
    error_terminate = 1'b1 ;

    result = 3'b001 ;
    // retry or perform reads while they are acknowledged or retried
    current_address = 32'h0000_0000 ;
    while (result[0] || result[2])
    begin
        wishbone_master.blkrd( current_address, 4'hF, 1'b1, 1'b1, 1'b0, {result, read_data} ) ;
        if (result[2])
            current_address = current_address + 4 ;
    end

    // check if result of last read was an error
    if (~result[1])
        $display("WISHBONE slave didn't signal an error on burst read when last data was marked as error") ;

    error_terminate = 1'b0 ;

    $display("Error termination tests finished!") ;
end
endtask //error_termination_test

task dlyd_req_rty_exp_test ;
    reg [31:0] current_address ;
    reg [2:0]  result ;
    reg [31:0] read_data ;
    integer i ;
begin
    $display("Starting delayed transaction expiration test!") ;

    // configure fifth image for accessing target which signals retry
    current_address = { `WB_CONFIGURATION_BASE, `W_IMG_CTRL5_ADDR } ;
    wishbone_master.blkwr( current_address, 32'h0000_0003, 4'h1, 1'b0, 1'b1, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write base address register
    current_address = { `WB_CONFIGURATION_BASE, `W_BA5_ADDR } ;
    wishbone_master.blkwr( current_address, 32'hFFFF_F000, 4'hf, 1'b0, 1'b0, 1'b0, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // write address mask register
    current_address = { `WB_CONFIGURATION_BASE, `W_AM5_ADDR } ;
    wishbone_master.blkwr( current_address, 32'hFFFF_F000, 4'hf, 1'b0, 1'b0, 1'b1, result ) ;

    // check if write succeeded
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    // now do a read through this image - it should be terminated with retry
    current_address = 32'hFFFF_F000 ;
    wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;
    if (~result[0])
        $display("WISHBONE slave didn't signal a retry on read request!") ;

    i = 0 ;
    // now do some other read and see if it comes through - it should go through since retry expired is signalled on previous read request
    current_address = 32'h0000_0000 ;
    while (result[0] && i < 1000)
    begin
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;
        i = i + 1 ;
    end
    
    if ( (~result[2]) && (i==1000) )
        $display("WISHBONE slave didn't provide new data in 1000 retries! Retry expired test probably failed") ;
    else
    if (result[2])
    begin
        // check if OK data was provided
        if (read_data !== wmem_data[0])
            display_warning(current_address, wmem_data[0], read_data) ;    
    end 
    else
        $display("Unknown slave response during retry expired test!") ;

    // now do a test where read is not repeated
    // issue read request to one address and then repeat second one until it's finished - it should finish when previous completion expires
    $display("Testing completion expiration") ;
    current_address = 32'h0000_0800 ;
    wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;
    if (~result[0])
        $display("WISHBONE slave didn't signal a retry on read request!") ;

    i = 0 ;
    current_address = 32'h0000_0100 ;
    while(result[0] && i < 32'h0001_FFFF)
    begin
        
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;
        i = i + 1;
    end
    
    if (result[2])
    begin
    // last access was acknowledged - check if right data is provided
        if (read_data !== wmem_data[current_address[11:2]])
            display_warning(current_address, wmem_data[current_address[11:2]], read_data) ;    
    end
    else
        $display("Completion expired test failed - completion has expired, but other reads didn't pass through WISHBONE slave interface") ;
        
    $display("Completion expiration test finished") ;

    $display("Delayed transaction expiration test finished!") ;
end
endtask //dlyd_req_rty_exp_test

task force_write_with_read_test ;
    integer write_data ;
    integer write_addr ;
    integer current_address ;
    reg [2:0] result ;
    integer read_data ;
begin
    // this performs a test if reads and writes proceede through bridge in correct order!
    $display("Starting force write with read test!") ;
    write_data = $random ;
    write_addr = $random ;
    // do a write through first image
    current_address = {20'h0_0000, write_addr[11:2], 2'b00} ;
    wishbone_master.blkwr( current_address, write_data, 4'hf, 1'b0, 1'b1, 1'b1, result ) ;
    if (~result[2])
        $display("WISHBONE slave didn't acknowledge write access to address %h although it was expected to!", current_address) ;

    result[0] = 1'b1 ;
    while (result[0])
        wishbone_master.blkrd( current_address, 4'hF, 1'b0, 1'b1, 1'b1, {result, read_data} ) ;    
    
    if (~result[2])
        $display("Read from address %h didn't succeed passing a bridge!", current_address) ;
    else
    begin
        if (read_data !== write_data)
            display_warning(current_address, write_data, read_data) ;
    end

    $display("Force write with read test finished!") ;
end
endtask //force_write_with_read_test

reg cycle_end ;
task burst_write ;
    input [31:0] start_address ;
    input [10:0] num_of_cycles ;
    reg   [10:0] mem_pointer ;
    reg [10:0]  i ;
    reg [31:0] burst_address ;
    reg [2:0] result ;
    reg [39:0] temp ;
    reg [9:0] retry_count ;
    
    begin 

        // apply start address
        burst_address = start_address ;
        
        // set retry counter to 0
        retry_count = 10'h000 ;

        // initialize index and result
        i = 11'h000 ;
        result = 3'b001 ;

        // assign just offset for memory pointer
        mem_pointer = start_address[11:2] ;

        while ( (i < num_of_cycles) && (result[1] == 1'b0) )
        begin
            // do a write
            cycle_end   = ((i + 1) == num_of_cycles) ;
            wishbone_master.blkwr( burst_address, wmem_data[mem_pointer + i], 4'hF, 1'b1, result[0], cycle_end, result ) ;
            
            if (result[2])
            begin
                // increment burst address
                burst_address = burst_address + 4 ;
                
                // increment counter
                i = i + 1'b1 ;

                // set retry counter to 0
                retry_count = 10'h000 ;

                // first data is address
                if (i == 0)
                begin
                    // write address to fifo contents monitor
                    wbw_contents[wbw_write_pointer] = {`ADDRESS, `BC_MEM_WRITE, address} ;
                    wbw_write_pointer = wbw_write_pointer + 1 ;
                end
                
                if (cycle_end)
                begin
                    // write data and appropriate control indication to fifo contents monitor
                    wbw_contents[wbw_write_pointer] = {`LAST, ~sel, sdat_i} ;
                    wbw_write_pointer = wbw_write_pointer + 1 ;
                end    
            end
            else
            if(result[1:0] && (i > 0))
            begin   
                // repair entry only once
                if (~retry_count)
                begin
                    // result of operation was either retry or error
                    temp = wbw_contents[wbw_write_pointer - 1];
                    wbw_contents[wbw_write_pointer-1] = {`LAST, temp[35:0]} ;
                end

                if (result[0])
                begin
                    retry_count = retry_count + 1 ;
                    if (retry_count === 10'h000)
                    begin
                        $display("Warning! Over 1000 retries were signalled from WB_SLAVE when attempting an image write! Possibility of wrong operation exists!") ;
                        $stop ;
                    end
                end
                   
            end
        end //while
    end
endtask //burst write

// pci side of FIFOs simulation
// write transaction signals on PCI side
reg [31:0] write_pci_address ;
reg [3:0] write_pci_bc ;

always@(posedge pci_clock)
begin
    // is there anything in a wbw_fifo
    if (~wbw_empty_probe)
    begin
        if (wbw_transaction_probe)
        begin
            // // transaction is ready in the fifo - simulate pci write
            pci_write_sim ;
        end
    end
    else
    if ( pci_req )
        // simulate completing of pci read
        pci_delayed_comp ;
end

task pci_write_sim ;
    reg write_to_mem ;
begin
    // write flag - if first entry of fifo is not OK, data will not get written to simulated PCI memory - default = 1
    write_to_mem = 1'b1 ;
    
    // activate renable signal for wbw_fifo
    wbw_renable <= #`FF_DELAY 1'b1 ;

    // all entries are read at posedge of clock
    @(posedge pci_clock) ;

    if (wbw_control_probe !== `ADDRESS)
    begin
        $display("First entry of transaction was not marked as address entry - pulling out data!") ;
        write_to_mem = 1'b0 ;
    end
    else
    if ((wbw_cbe_probe !== `BC_MEM_WRITE) && (wbw_cbe_probe !== `BC_IO_WRITE))
    begin
        $display("First entry of transaction didn't provide valid bus command - pulling out data!") ;
        write_to_mem = 1'b0 ;
    end
    else
    if ( ((wbw_cbe_probe == `BC_MEM_WRITE ) && (wbw_data_probe[31:12] !== 20'h0000_0)) ||
         ((wbw_cbe_probe == `BC_IO_WRITE ) && (wbw_data_probe[31:12] !== 20'h0000_2)) )
    begin
        $display("Wrong address provided for transaction!") ;
        write_to_mem = 1'b0 ;
    end
    else
    begin
        write_pci_address <= #`FF_DELAY wbw_data_probe ;
        write_pci_bc      <= #`FF_DELAY wbw_cbe_probe ;
    end
    
    while (wbw_control_probe !== `LAST)
    begin
        @(posedge pci_clock) ;
        if (write_to_mem)
        begin
            //write this data to read memory
            if (write_pci_bc === `BC_MEM_WRITE) 
            begin
                // this is a memory write - store it to memory space
                if (~wbw_cbe_probe[0])
                    rmem_data[{write_pci_address[11:2], 2'b00}] = wbw_data_probe[7:0] ;
                else
                    rmem_data[{write_pci_address[11:2], 2'b00}] = 8'h00 ;
                
                if (~wbw_cbe_probe[1])
                    rmem_data[{write_pci_address[11:2], 2'b01}] = wbw_data_probe[15:8] ;
                else
                    rmem_data[{write_pci_address[11:2], 2'b01}] = 8'h00 ;

                if (~wbw_cbe_probe[2])
                    rmem_data[{write_pci_address[11:2], 2'b10}] = wbw_data_probe[23:16] ;
                else
                    rmem_data[{write_pci_address[11:2], 2'b10}] = 8'h00 ;

                if (~wbw_cbe_probe[3])
                    rmem_data[{write_pci_address[11:2], 2'b11}] = wbw_data_probe[31:24] ;
                else
                    rmem_data[{write_pci_address[11:2], 2'b11}] = 8'h00 ;
            end // mem write
            else
            begin
                // io write - same as mem write except write to io space
                if (~wbw_cbe_probe[0])
                    rio_data[{write_pci_address[11:2], 2'b00}] = wbw_data_probe[7:0] ;
                else
                    rio_data[{write_pci_address[11:2], 2'b00}] = 8'h00 ;
                
                if (~wbw_cbe_probe[1])
                    rio_data[{write_pci_address[11:2], 2'b01}] = wbw_data_probe[15:8] ;
                else
                    rio_data[{write_pci_address[11:2], 2'b01}] = 8'h00 ;

                if (~wbw_cbe_probe[2])
                    rio_data[{write_pci_address[11:2], 2'b10}] = wbw_data_probe[23:16] ;
                else
                    rio_data[{write_pci_address[11:2], 2'b10}] = 8'h00 ;

                if (~wbw_cbe_probe[3])
                    rio_data[{write_pci_address[11:2], 2'b11}] = wbw_data_probe[31:24] ;
                else
                    rio_data[{write_pci_address[11:2], 2'b11}] = 8'h00 ;
            end // io write

            write_pci_address = write_pci_address + 4 ;
        
        end // write_to_mem
        
        // check control bus
        if ((wbw_control_probe !== `DATA) && (wbw_control_probe !== `LAST))
            $display("Control bus encoding during image write contained invalid value") ;

    end // while

    wbw_renable <= #`FF_DELAY 1'b0 ;

end
endtask //pci_write_sim

task burst_read ;
    input [31:0] start_address ;
    input [10:0] num_of_cycles ;
    input [3:0] selects ;
    reg [10:0]  i ;
    reg [31:0] burst_address ;
    
    reg [34:0] status_w_read_data ;
    reg [31:0] read_data ;
    reg [2:0] result ;
    reg [31:0] temp_data ;

    reg warning_flag ;

    begin 
        warning_flag = 1'b0 ;

        // apply start address
        burst_address = start_address ;
    
        // apply memory address

        // apply number of writes
        i = 11'h000 ;
        result = 3'b001 ;
        
        while ( (i < num_of_cycles) && (result[1] == 1'b0) )
        begin
            // do a read
            cycle_end = ((i + 1) == num_of_cycles) ;

            wishbone_master.blkrd( burst_address, selects, 1'b1, result[0], cycle_end, status_w_read_data ) ;

            // extract status from return data
            result = status_w_read_data[34:32] ;
        
            // extract data
            read_data = status_w_read_data[31:0] ;
            if (result[2])
            begin
                
                temp_data = wmem_data[burst_address[11:2]] ;
                
                // see if data read is OK
                if (
                        (selects[3] && (read_data[31:24] !== temp_data[31:24])) ||
                        (selects[2] && (read_data[23:16] !== temp_data[23:16])) ||
                        (selects[1] && (read_data[15:8] !== temp_data[15:8])) ||
                        (selects[0] && (read_data[7:0] !== temp_data[7:0]))
                    )
                    warning_flag = 1'b1 ;
               else
                    warning_flag = 1'b0 ;

                if (warning_flag)
                    display_warning(burst_address, temp_data, read_data) ;

                // increment address
                burst_address = burst_address + 4 ;

                

                i = i + 1 ;
            end
        end //while
    end
endtask //burst read

task read_io;
    input [31:0] cur_addr ;
    input [3:0]  selects ;
    reg[31:0] temp_data ;
    reg[2:0] result ;
    reg [31:0] read_data ;
    reg [34:0] status_w_read_data ;
    reg warning_flag ;

    begin         

        result = 3'b001 ;
        while (result[0])
        begin
            wishbone_master.blkrd( cur_addr, selects, 1'b0, 1'b1, 1'b1, status_w_read_data ) ;
            
            // extract status from return data
            result = status_w_read_data[34:32] ;
        end
        
        if (result[2])
        begin
            // extract data
            read_data = status_w_read_data[31:0] ;    
            
            // see what is in IO space
            temp_data = wio_data[cur_addr[11:2]] ;
        
            // see if data read is OK
            if (
                (selects[3] && (read_data[31:24] !== temp_data[31:24])) ||
                (selects[2] && (read_data[23:16] !== temp_data[23:16])) ||
                (selects[1] && (read_data[15:8] !== temp_data[15:8])) ||
                (selects[0] && (read_data[7:0] !== temp_data[7:0]))
               )
                warning_flag = 1'b1 ;
            else
                warning_flag = 1'b0 ;

            if (warning_flag)
                display_warning(cur_addr, temp_data, read_data) ;
        end

    end
endtask // read_io

// completions checking block
// monitors for completions pending in WBR_FIFO - if there is completion present,
// check slave response - only two possible - error or acknowledge
always@(posedge wb_clock or posedge reset or posedge wbr_flush or hit)
begin
    if (~reset && ~wbr_flush)
    begin
        if (hit && cyc && stb && ~we)
        begin
            // read through an image is attempted - see if there is completion present in the FIFO
            if (del_comp)
            begin
                if ((del_addr == address) && (del_be == sel) && (del_bc == del_bc_probe) && ~wbr_empty)
                begin
                    if (~ack && (wbr_control != `DATA_ERROR))
                        $display("WISHBONE slave didn't acknowledge read completion stored in wbr_fifo") ;
                    else
                    if (~err && (wbr_control == `DATA_ERROR))
                    begin
                        $display("WISHBONE didn't respond with an error although data was marked as an error");
                    end
                end // read completion
            end // read comp     
        end
    end
end

// function for determining selects error
function selects_error ;
    input [1:0] addr_lsb ;
    input [3:0] selects ;
    input [5:0] hit ;
    input [5:0] map ;
begin
    case (addr_lsb)
        2'b00 : selects_error = 1'b0 ; // alligned address - any combination of sel is valid
        2'b01 : if (map & hit)
                    selects_error = selects[0] ; // sel 0 must be inactiove
                else
                    selects_error = 1'b1 ; // unaligned memory space access is always an error
        2'b10 : if (map & hit)
                     selects_error = selects[0] || selects[1] ; // sel 0 and 1 must be 0
                else
                    selects_error = 1'b1 ; 
        2'b11 : if (map & hit)
                    selects_error = selects[0] || selects[1] || selects[2] ; // sel 0, 1 and 2 must be 0
                else
                    selects_error = 1'b1 ;

    endcase
end         
endfunction // check_selects

// monitor for image accesses - it monitors slave response when FIFOs are full, empty, hit is changed etc.
always@(posedge wb_clock)
begin
    // transfer was acknowledged - see if it was allowed to
    if (ack)
    begin

        if (selects_error( addr_o[1:0], sel, {conf_hit, hit}, {1'b1, wb_map[5:1]} ))
            $display("Access with illegal address / select combination acknowledged by WISHBONE slave") ;

        // Image accesses
        if (hit)
        begin
            // is this a write?
            if (we)
            begin
            
                // check if fifo is almost full or full
                if ( ((wbw_write_pointer + 1) == wbw_read_pointer) || ((wbw_write_pointer + 2) == wbw_read_pointer))
                    $display("Wrong slave response - it acknowledged image write when FIFO was full") ;

                if (del_req)
                    $display("Wrong slave response - it acknowledged image write when delayed read request was pending") ;
                
                if (wbs_lock)
                    $display("Wrong slave response - it should block out writes when error is reported") ;

            end // write
            else
            begin //read
                if (~del_comp)
                    $display("WISHBONE slave acknowledged delayed read, when it was not yet completed on PCI bus") ;
                
                if (~pciw_empty)
                    $display("WISHBONE slave allowed a read to complete on WISHBONE, before writes from PCI to WISHBONE were completed") ;
            end // read
                
        end //hit
        else
        if (conf_hit)
        begin
            // configuration accesses monitored elsewhere
        end // conf_hit
        else
            $display("WISHBONE slave acknowledged transfer while none of its address ranges was selected") ;
        
    end // acknowledge
end

// delayed read control logic monitor
always@(posedge wb_clock or posedge reset)
begin
    // issuing request when one is in process is illegal
    if (del_req && del_req_probe)
        $display("WISHBONE slave issued another read request when one was pending already") ;
    else
    // issuing request when completion is pending is illegal
    if (del_req_probe && del_comp)
        $display("WISHBONE slave issued read request when completion was pending already") ;
    else
    // issuing done without request pending is illegal
    if (del_done_probe && ~del_comp)
        $display("WISHBONE slave signaled read request done when no completion was pending") ;
end

// pci side of the bridge simulation - when writes are pulled out of WBW_FIFO this block monitors if contents is OK
always@(posedge pci_clock or posedge reset)
begin
    if (reset)
        wbw_read_pointer =  {`WBW_ADDR_LENGTH{1'b0}};
    if (wbw_renable)
    begin
        // compare data from FIFO with expected values
        if (wbw_contents[wbw_read_pointer] != {wbw_control_probe, wbw_cbe_probe, wbw_data_probe})
            $display("Data read from wbw_fifo was %h, while expected was %h!", {wbw_control_probe, wbw_cbe_probe, wbw_data_probe}, wbw_contents[wbw_read_pointer]) ;
        wbw_contents[wbw_read_pointer] = 40'hxxxxxxxxxx ;
        wbw_read_pointer = wbw_read_pointer + 1 ;
    end
end

// address write monitor - check if OK addresses are written to write FIFO
always@(posedge wb_clock)
begin
    if (wbw_wenable)
    begin
        // something is going into FIFO - check hits, check bus command and address during address entry
        case (hit)
            5'b00001:   begin
                            if (wbw_control == `ADDRESS) 
                            begin
                                // address entry - check address provided and bus command
                                if (image_data != img_addr1)
                                    $display("WISHBONE slave didn't provide right address in FIFO entry") ;
                                // check bus command
                                if ((wb_map[1] && (image_cbe != `BC_IO_WRITE)) ||
                                     (~wb_map[1] && (image_cbe != `BC_MEM_WRITE)))

                                    $display("WISHBONE slave provided wrong bus command") ;
                            end
                        end
            5'b00010:   begin
                            if (wbw_control == `ADDRESS) 
                            begin
                                // address entry - check address provided and bus command
                                if (image_data != img_addr2)
                                    $display("WISHBONE slave didn't provide right address in FIFO entry") ;
                                // check bus command
                                if ((wb_map[2] && (image_cbe != `BC_IO_WRITE)) ||
                                     (~wb_map[2] && (image_cbe != `BC_MEM_WRITE)))

                                    $display("WISHBONE slave provided wrong bus command") ;
                            end
                        end
            5'b00100:   begin
                            if (wbw_control == `ADDRESS) 
                            begin
                                // address entry - check address provided and bus command
                                if (image_data != img_addr3)
                                    $display("WISHBONE slave didn't provide right address in FIFO entry") ;
                                // check bus command
                                if ((wb_map[3] && (image_cbe != `BC_IO_WRITE)) ||
                                     (~wb_map[3] && (image_cbe != `BC_MEM_WRITE)))

                                    $display("WISHBONE slave provided wrong bus command") ;
                            end
                        end
            5'b01000:   begin
                            if (wbw_control == `ADDRESS) 
                            begin
                                // address entry - check address provided and bus command
                                if (image_data != img_addr4)
                                    $display("WISHBONE slave didn't provide right address in FIFO entry") ;
                                // check bus command
                                if ((wb_map[4] && (image_cbe != `BC_IO_WRITE)) ||
                                     (~wb_map[4] && (image_cbe != `BC_MEM_WRITE)))

                                    $display("WISHBONE slave provided wrong bus command") ;
                            end
                        end
            5'b10000:   begin
                            if (wbw_control == `ADDRESS) 
                            begin
                                // address entry - check address provided and bus command
                                if (image_data != img_addr5)
                                    $display("WISHBONE slave didn't provide right address in FIFO entry") ;
                                // check bus command
                                if ((wb_map[5] && (image_cbe != `BC_IO_WRITE)) ||
                                     (~wb_map[5] && (image_cbe != `BC_MEM_WRITE)))

                                    $display("WISHBONE slave provided wrong bus command") ;
                            end
                        end
            default:    begin
                            // only last data can be written when hit is not OK - anything else is an error
                            if (wbw_control != `LAST)
                                $display("WISHBONE slave wrote illegal data to FIFO. When hit is undefined only last data can be valid entry") ;
                        end
        endcase
    end
end
        
// pci read completion simulation
task pci_delayed_comp ;
    reg [7:0] num_of_reads ;
    reg [7:0] i ;
    reg [31:0] cur_address ;
    reg [31:0] provide_data ;
begin
    i = 8'h00 ;
    // check if read address is the one that signals retry
    if (del_addr[31:12] === 20'hFFFF_F)
    begin
        num_of_reads = 0 ;
        del_rty_exp <= #`FF_DELAY 1'b1 ;
        // wait for end of cycle
        @(posedge pci_clock) ;
        del_rty_exp <= #`FF_DELAY 1'b0 ;
    end
    else
    begin
    
        cur_address = del_addr ;
        if (~read_in_burst)
            num_of_reads = 8'h01 ;
        else
            // simulated burst size is 8
            num_of_reads = 8'h8 ;
        
        // enable writes to wbr_fifo
        if (~del_write)
            wbr_wenable <= #`FF_DELAY 1'b1 ;

        // byte enables must be the same as in request
        wbr_be_in   <= #`FF_DELAY del_be ;
    end
        
    while (i <  num_of_reads && ~wbr_almost_full_probe)
    fork
    begin:data_control
        if ( (del_bc === `BC_MEM_READ) || (del_bc === `BC_MEM_READ_LN) )
            provide_data = {
                            rmem_data[{cur_address[11:2], 2'b11}],
                            rmem_data[{cur_address[11:2], 2'b10}],
                            rmem_data[{cur_address[11:2], 2'b01}],
                            rmem_data[{cur_address[11:2], 2'b00}]
                           } ;
        else
        if (del_bc === `BC_IO_READ)
            provide_data = {
                            rio_data[{cur_address[11:2], 2'b11}],
                            rio_data[{cur_address[11:2], 2'b10}],
                            rio_data[{cur_address[11:2], 2'b01}],
                            rio_data[{cur_address[11:2], 2'b00}]
                           } ;
        else
        if (del_bc === `BC_IACK)
            provide_data = iack_read_data ;
        else
        if (del_bc === `BC_CONF_READ)
            provide_data = conf_read_data ;
        else
        if (del_bc === `BC_CONF_WRITE)
            provide_data = del_write_data ;
        else
        begin
            $display("Invalid bus command provided for delayed read request! Request will be terminated with an error") ;
            i = num_of_reads - 1;
        end
        
        if (~del_write)
        begin
        
            if (del_be[0])
                wbr_data_in[7:0] <= #`FF_DELAY provide_data[7:0] ;
            else
                wbr_data_in[7:0] <= #`FF_DELAY 8'h00 ;
    
            if (del_be[1])
                wbr_data_in[15:8] <= #`FF_DELAY provide_data[15:8] ;
            else
                wbr_data_in[15:8] <= #`FF_DELAY 8'h00 ;
        
            if (del_be[2])
                wbr_data_in[23:16] <= #`FF_DELAY provide_data[23:16] ;
            else
                wbr_data_in[23:16] <= #`FF_DELAY 8'h00 ;
    
            if (del_be[3])
                wbr_data_in[31:24] <= #`FF_DELAY provide_data[31:24] ;
            else
                wbr_data_in[31:24] <= #`FF_DELAY 8'h00 ;
        
        end
        else
        begin
            if (del_bc !== `BC_CONF_WRITE)
                $display("Invalid write indicator for delayed transaction! Only Configuration Write can be delayed write request!");
        end    
        
        // wait for data to get written to FIFO
        @(posedge pci_clock) ;
        
        if (del_write && (del_bc === `BC_CONF_WRITE))
        begin
            if (del_be[0])
                conf_read_data[7:0] <= #`FF_DELAY provide_data[7:0] ;
            else
                conf_read_data[7:0] <= #`FF_DELAY 8'h00 ;
    
            if (del_be[1])
                conf_read_data[15:8] <= #`FF_DELAY provide_data[15:8] ;
            else
                conf_read_data[15:8] <= #`FF_DELAY 8'h00 ;
        
            if (del_be[2])
                conf_read_data[23:16] <= #`FF_DELAY provide_data[23:16] ;
            else
                conf_read_data[23:16] <= #`FF_DELAY 8'h00 ;
    
            if (del_be[3])
                conf_read_data[31:24] <= #`FF_DELAY provide_data[31:24] ;
            else
                conf_read_data[31:24] <= #`FF_DELAY 8'h00 ;
        end
        // increment vectors address
        cur_address = cur_address + 4 ;

        i = i + 1 ;
    end //data_control
    begin:done_and_error_control
    
        if ( wbr_almost_full_probe )
            del_comp_done <= #`FF_DELAY 1'b1 ;

        if ((i + 1) == num_of_reads)
        begin
            // signal done
            del_comp_done <= #`FF_DELAY 1'b1 ;

            // check how to terminate (error or normal)
            if (error_terminate)
            begin
                del_error_signal <= #`FF_DELAY 1'b1 ;
                wbr_control_in <= #`FF_DELAY `DATA_ERROR ;
            end
            else
                wbr_control_in <= #`FF_DELAY `LAST ;
        end
        else
            wbr_control_in <= #`FF_DELAY `DATA ;

        // wait for data to be written
        @(posedge pci_clock) ;
        
        // wait for negedge of clock - give time for outputs to setup and propagate
        @(negedge pci_clock) ;

    end // done_control
    join
    
    // disable writes to wbr_fifo
    wbr_wenable <= #`FF_DELAY 1'b0 ;
    del_comp_done <= #`FF_DELAY 1'b0 ;
    del_error_signal <= #`FF_DELAY 1'b0 ;
end
endtask // pci_delayed_comp

endmodule // TEST_BENCH
