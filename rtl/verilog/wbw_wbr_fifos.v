//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "wbw_wbr_fifos.v"                                 ////
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
// Revision 1.1.1.1  2001/10/02 15:33:47  mihad
// New project directory structure
//
//

`include "constants.v"
`include "timescale.v"

module WBW_WBR_FIFOS( 
                        wb_clock_in, 
                        pci_clock_in, 
                        reset_in, 
                        wbw_wenable_in, 
                        wbw_addr_data_in, 
                        wbw_cbe_in, 
                        wbw_control_in,                     
                        wbw_renable_in, 
                        wbw_addr_data_out, 
                        wbw_cbe_out, 
                        wbw_control_out,                     
                        wbw_flush_in, 
                        wbw_almost_full_out, 
                        wbw_full_out,
                        wbw_empty_out, 
                        wbw_transaction_ready_out, 
                        wbr_wenable_in, 
                        wbr_data_in, 
                        wbr_be_in, 
                        wbr_control_in,                     
                        wbr_renable_in, 
                        wbr_data_out, 
                        wbr_be_out, 
                        wbr_control_out,                     
                        wbr_flush_in, 
                        wbr_empty_out
) ;

/*-----------------------------------------------------------------------------------------------------------
System inputs:
wb_clock_in - WISHBONE bus clock
pci_clock_in - PCI bus clock
reset_in - reset from control logic
-------------------------------------------------------------------------------------------------------------*/
input wb_clock_in, pci_clock_in, reset_in ;

/*-----------------------------------------------------------------------------------------------------------
WISHBONE WRITE FIFO interface signals prefixed with wbw_ - FIFO is used for posted writes initiated by
WISHBONE master, traveling through FIFO and are completed on PCI by PCI master interface 

write enable signal:
wbw_wenable_in = write enable input for WBW_FIFO - driven by WISHBONE slave interface

data input signals:
wbw_addr_data_in = data input - data from WISHBONE bus - first entry of transaction is address others are data entries
wbw_cbe_in       = bus command/byte enable(~SEL[3:0]) input - first entry of transaction is bus command, other are byte enables
wbw_control_in   = control input - encoded control bus input

read enable signal:
wbw_renable_in = read enable input driven by PCI master interface

data output signals:
wbw_addr_data_out = data output - data from WISHBONE bus - first entry of transaction is address, others are data entries
wbw_cbe_out      = bus command/byte enable output - first entry of transaction is bus command, others are byte enables
wbw_control_out = control input - encoded control bus input

status signals - monitored by various resources in the core
wbw_flush_in = flush signal input for WBW_FIFO - when asserted, fifo is flushed(emptied)
wbw_almost_full_out = almost full output from WBW_FIFO
wbw_full_out = full output from WBW_FIFO
wbw_empty_out = empty output from WBW_FIFO
wbw_transaction_ready_out = output indicating that one complete transaction is waiting in WBW_FIFO
-----------------------------------------------------------------------------------------------------------*/
// input control and data
input        wbw_wenable_in ;
input [31:0] wbw_addr_data_in ;
input [3:0]  wbw_cbe_in ;
input [3:0]  wbw_control_in ;

// output control and data
input         wbw_renable_in ;
output [31:0] wbw_addr_data_out ;
output [3:0]  wbw_cbe_out ;
output [3:0]  wbw_control_out ;                    

// flush input
input wbw_flush_in ; 

// status outputs
output wbw_almost_full_out ;
output wbw_full_out ;
output wbw_empty_out ;
output wbw_transaction_ready_out ;

/*-----------------------------------------------------------------------------------------------------------
WISHBONE READ FIFO interface signals prefixed with wbr_ - FIFO is used for holding delayed read completions 
initiated by master on WISHBONE bus and completed on PCI bus, 

write enable signal:
wbr_wenable_in = write enable input for WBR_FIFO - driven by PCI master interface

data input signals:
wbr_data_in      = data input - data from PCI bus - there is no address entry here, since address is stored in separate register
wbr_be_in        = byte enable(~BE#[3:0]) input - byte enables - same through one transaction
wbr_control_in   = control input - encoded control bus input

read enable signal:
wbr_renable_in = read enable input driven by WISHBONE slave interface

data output signals:
wbr_data_out = data output - data from PCI bus
wbr_be_out      = byte enable output(~#BE)
wbr_control_out = control output - encoded control bus output

status signals - monitored by various resources in the core
wbr_flush_in = flush signal input for WBR_FIFO - when asserted, fifo is flushed(emptied)
wbr full_out = full output from WBR_FIFO
wbr_empty_out = empty output from WBR_FIFO
-----------------------------------------------------------------------------------------------------------*/
// input control and data
input        wbr_wenable_in ;
input [31:0] wbr_data_in ;
input [3:0]  wbr_be_in ;
input [3:0]  wbr_control_in ;                     

// output control and data
input         wbr_renable_in ;
output [31:0] wbr_data_out ;
output [3:0]  wbr_be_out ;
output [3:0]  wbr_control_out ;                    

// flush input
input wbr_flush_in ; 

output wbr_empty_out ;

/*-----------------------------------------------------------------------------------------------------------
FIFO depth parameters:
WBW_DEPTH = defines WBW_FIFO depth
WBR_DEPTH = defines WBR_FIFO depth
WBW_ADDR_LENGTH = defines WBW_FIFO's location address length = log2(WBW_DEPTH)
WBR_ADDR_LENGTH = defines WBR_FIFO's location address length = log2(WBR_DEPTH)
-----------------------------------------------------------------------------------------------------------*/
parameter WBW_DEPTH = `WBW_DEPTH ;
parameter WBW_ADDR_LENGTH = `WBW_ADDR_LENGTH ;
parameter WBR_DEPTH = `WBR_DEPTH ;
parameter WBR_ADDR_LENGTH = `WBR_ADDR_LENGTH ;

// obvious
wire vcc = 1'b1 ;
wire gnd = 1'b0 ;

/*-----------------------------------------------------------------------------------------------------------
wbw_wallow = WBW_FIFO write allow wire - writes to FIFO are allowed when FIFO isn't full and write enable is 1
wbw_rallow = WBW_FIFO read allow wire - reads from FIFO are allowed when FIFO isn't empty and read enable is 1
-----------------------------------------------------------------------------------------------------------*/
wire wbw_wallow ;
wire wbw_rallow ;

/*-----------------------------------------------------------------------------------------------------------
wbr_wallow = WBR_FIFO write allow wire - writes to FIFO are allowed when FIFO isn't full and write enable is 1
wbr_rallow = WBR_FIFO read allow wire - reads from FIFO are allowed when FIFO isn't empty and read enable is 1
-----------------------------------------------------------------------------------------------------------*/
wire wbr_wallow ;
wire wbr_rallow ;

/*-----------------------------------------------------------------------------------------------------------
wires for address port conections from WBW_FIFO control logic to RAM blocks used for WBW_FIFO
-----------------------------------------------------------------------------------------------------------*/
wire [(WBW_ADDR_LENGTH - 1):0] wbw_raddr ;
wire [(WBW_ADDR_LENGTH - 1):0] wbw_waddr ;

/*-----------------------------------------------------------------------------------------------------------
wires for address port conections from WBR_FIFO control logic to RAM blocks used for WBR_FIFO
-----------------------------------------------------------------------------------------------------------*/
wire [(WBR_ADDR_LENGTH - 1):0] wbr_raddr ;
wire [(WBR_ADDR_LENGTH - 1):0] wbr_waddr ;

/*-----------------------------------------------------------------------------------------------------------
WBW_FIFO transaction counters: used to count incoming transactions and outgoing transactions. When number of
input transactions is equal to number of output transactions, it means that there isn't any complete transaction
currently present in the FIFO.
-----------------------------------------------------------------------------------------------------------*/
reg [(WBW_ADDR_LENGTH - 2):0] wbw_inTransactionCount ;
reg [(WBW_ADDR_LENGTH - 2):0] wbw_outTransactionCount ;

/*-----------------------------------------------------------------------------------------------------------
FlipFlops for indicating if complete delayed read completion is present in the FIFO
-----------------------------------------------------------------------------------------------------------*/
/*reg wbr_inTransactionCount ;
reg wbr_outTransactionCount ;*/
/*-----------------------------------------------------------------------------------------------------------
wires monitoring control bus. When control bus on a write transaction has a value of `LAST, it means that
complete transaction is in the FIFO. When control bus on a read transaction has a value of `LAST,
it means that there was one complete transaction taken out of FIFO.
-----------------------------------------------------------------------------------------------------------*/
wire wbw_last_in  = wbw_control_in[`LAST_CTRL_BIT]  ;
wire wbw_last_out = wbw_control_out[`LAST_CTRL_BIT] ;

/*wire wbr_last_in  = wbr_wallow && wbr_control_in[`LAST_CTRL_BIT] ;
wire wbr_last_out = wbr_rallow && wbr_control_out[`LAST_CTRL_BIT] ;*/

wire wbw_empty ;
wire wbr_empty ;

assign wbw_empty_out = wbw_empty ;
assign wbr_empty_out = wbr_empty ;

// clear wires for fifos
wire wbw_clear = reset_in || wbw_flush_in ; // WBW_FIFO clear
wire wbr_clear = reset_in || wbr_flush_in ; // WBR_FIFO clear

`ifdef FPGA
/*-----------------------------------------------------------------------------------------------------------
this code is included only for FPGA core usage - somewhat different logic because of sharing
one block selectRAM+ between two FIFOs
-----------------------------------------------------------------------------------------------------------*/
    `ifdef BIG
        /*-----------------------------------------------------------------------------------------------------------
        Big FPGAs
        WBW_FIFO and WBR_FIFO address prefixes - used for extending read and write addresses because of varible
        FIFO depth and fixed SelectRAM+ size. Addresses are zero paded on the left to form long enough address
        -----------------------------------------------------------------------------------------------------------*/
        wire [(7 - WBW_ADDR_LENGTH):0] wbw_addr_prefix = {( 8 - WBW_ADDR_LENGTH){1'b0}} ;
        wire [(7 - WBR_ADDR_LENGTH):0] wbr_addr_prefix = {( 8 - WBR_ADDR_LENGTH){1'b0}} ;

        // compose addresses
        wire [7:0] wbw_whole_waddr = {wbw_addr_prefix, wbw_waddr} ;
        wire [7:0] wbw_whole_raddr = {wbw_addr_prefix, wbw_raddr} ;
        
        wire [7:0] wbr_whole_waddr = {wbr_addr_prefix, wbr_waddr} ;
        wire [7:0] wbr_whole_raddr = {wbr_addr_prefix, wbr_raddr} ;

        /*-----------------------------------------------------------------------------------------------------------
        Only 8 bits out of 16 are used in ram3 and ram6 - wires for referencing them
        -----------------------------------------------------------------------------------------------------------*/
        wire [15:0] dpram3_portB_output ;
        wire [15:0] dpram6_portA_output ;

        /*-----------------------------------------------------------------------------------------------------------
        Control out assignements from ram3 output
        -----------------------------------------------------------------------------------------------------------*/
        assign wbw_control_out = dpram3_portB_output[15:12] ;
        assign wbr_control_out = dpram6_portA_output[15:12] ;

        assign wbw_cbe_out = dpram3_portB_output[3:0] ;
        assign wbr_be_out  = dpram6_portA_output[3:0] ;

        wire wbw_read_enable = 1'b1 ;
        wire wbr_read_enable = 1'b1 ;
        
        // Block SelectRAM+ cells instantiation
        RAMB4_S16_S16 dpram16_1 (.ADDRA(wbw_whole_waddr), .DIA(wbw_addr_data_in[15:0]), 
                                 .ENA(vcc), .RSTA(reset_in),
                                 .CLKA(wb_clock_in), .WEA(wbw_wallow), 
                                 .DOA(),
                                 .ADDRB(wbw_whole_raddr), .DIB(16'h0000), 
                                 .ENB(wbw_read_enable), .RSTB(reset_in),
                                 .CLKB(pci_clock_in), .WEB(gnd), 
                                 .DOB(wbw_addr_data_out[15:0])) ;

        RAMB4_S16_S16 dpram16_2 (.ADDRA(wbw_whole_waddr), .DIA(wbw_addr_data_in[31:16]), 
                                 .ENA(vcc), .RSTA(reset_in),
                                 .CLKA(wb_clock_in), .WEA(wbw_wallow), 
                                 .DOA(),
                                 .ADDRB(wbw_whole_raddr), .DIB(16'h0000), 
                                 .ENB(wbw_read_enable), .RSTB(reset_in),
                                 .CLKB(pci_clock_in), .WEB(gnd), 
                                 .DOB(wbw_addr_data_out[31:16])) ;
    
        RAMB4_S16_S16 dpram16_3 (.ADDRA(wbw_whole_waddr), .DIA({wbw_control_in, 8'h00, wbw_cbe_in}), 
                                 .ENA(vcc), .RSTA(reset_in),
                                 .CLKA(wb_clock_in), .WEA(wbw_wallow), 
                                 .DOA(),
                                 .ADDRB(wbw_whole_raddr), .DIB(16'h0000), 
                                 .ENB(wbw_read_enable), .RSTB(reset_in),
                                 .CLKB(pci_clock_in), .WEB(gnd), 
                                 .DOB(dpram3_portB_output)) ;

        RAMB4_S16_S16 dpram16_4 (.ADDRA(wbr_whole_raddr), .DIA(16'h0000), 
                                 .ENA(1'b1), .RSTA(reset_in),
                                 .CLKA(wb_clock_in), .WEA(gnd), 
                                 .DOA(wbr_data_out[15:0]),
                                 .ADDRB(wbr_whole_waddr), .DIB(wbr_data_in[15:0]), 
                                 .ENB(wbr_read_enable), .RSTB(reset_in),
                                 .CLKB(pci_clock_in), .WEB(wbr_wallow), 
                                 .DOB()) ;

        RAMB4_S16_S16 dpram16_5 (.ADDRA(wbr_whole_raddr), .DIA(16'h0000), 
                                 .ENA(1'b1), .RSTA(reset_in),
                                 .CLKA(wb_clock_in), .WEA(gnd), 
                                 .DOA(wbr_data_out[31:16]),
                                 .ADDRB(wbr_whole_waddr), .DIB(wbr_data_in[31:16]), 
                                 .ENB(wbr_read_enable), .RSTB(reset_in),
                                 .CLKB(pci_clock_in), .WEB(wbr_wallow), 
                                 .DOB()) ;
    
        RAMB4_S16_S16 dpram16_6 (.ADDRA(wbr_whole_raddr), .DIA(16'h0000), 
                                 .ENA(1'b1), .RSTA(reset_in),
                                 .CLKA(wb_clock_in), .WEA(gnd), 
                                 .DOA(dpram6_portA_output),
                                 .ADDRB(wbr_whole_waddr), .DIB({wbr_control_in, 8'h00, wbr_be_in}), 
                                 .ENB(wbr_read_enable), .RSTB(reset_in),
                                 .CLKB(pci_clock_in), .WEB(wbr_wallow), 
                                 .DOB()) ;

    `else // SMALL FPGAs

        /*-----------------------------------------------------------------------------------------------------------
        Small FPGAs
        WBW_FIFO and WBR_FIFO address prefixes - used for extending read and write addresses because of varible
        FIFO depth and fixed SelectRAM+ size. Addresses are always paded, because of RAM sharing between FIFOs
        WBW addresses are zero padded on the left, WBR addresses are padded
        with ones on the left
        -----------------------------------------------------------------------------------------------------------*/
        wire [(7 - WBW_ADDR_LENGTH):0] wbw_addr_prefix = {( 8 - WBW_ADDR_LENGTH){1'b0}} ;
        wire [(7 - WBR_ADDR_LENGTH):0] wbr_addr_prefix = {( 8 - WBR_ADDR_LENGTH){1'b1}} ;

        /*-----------------------------------------------------------------------------------------------------------
        Only 8 bits out of 16 are used in ram3 - wires for referencing them
        -----------------------------------------------------------------------------------------------------------*/
        wire [15:0] dpram3_portA_output ;
        wire [15:0] dpram3_portB_output ;

        /*-----------------------------------------------------------------------------------------------------------
        Control out assignements from ram3 output
        -----------------------------------------------------------------------------------------------------------*/
        assign wbw_control_out = dpram3_portB_output[15:12] ;
        assign wbr_control_out = dpram3_portA_output[15:12] ;

        assign wbw_cbe_out = dpram3_portB_output[3:0] ;
        assign wbr_be_out  = dpram3_portA_output[3:0] ;
        
        /*-----------------------------------------------------------------------------------------------------------
        Port A address generation for block SelectRam+ in SpartanII or Virtex
        Port A is clocked by WISHBONE clock, DIA is input for wbw_fifo, DOA is output for wbr_fifo. Address is multiplexed
        between two values.
        Address multiplexing:
        wbw_wenable == 1 => ADDRA = wbw_waddr (write pointer of WBW_FIFO)
        else                ADDRA = wbr_raddr (read pointer of WBR_FIFO)
        -----------------------------------------------------------------------------------------------------------*/
        wire [7:0] portA_addr = wbw_wallow ? {wbw_addr_prefix, wbw_waddr} : {wbr_addr_prefix, wbr_raddr} ;
    
        /*-----------------------------------------------------------------------------------------------------------
        Port B address generation for block SelectRam+ in SpartanII or Virtex
        Port B is clocked by PCI clock, DIB is input for wbr_fifo, DOB is output for wbw_fifo. Address is multiplexed
        between two values.
        Address multiplexing:
        wbr_wenable == 1 => ADDRB = wbr_waddr (write pointer of WBR_FIFO)
        else                ADDRB = wbw_raddr (read pointer of WBW_FIFO)
        -----------------------------------------------------------------------------------------------------------*/
        wire [7:0] portB_addr  = wbr_wallow ? {wbr_addr_prefix, wbr_waddr} : {wbw_addr_prefix, wbw_raddr} ;
    
        wire portA_enable      = 1'b1 ;

        wire portB_enable      = 1'b1 ;

        // Block SelectRAM+ cells instantiation
        RAMB4_S16_S16 dpram16_1 (.ADDRA(portA_addr), .DIA(wbw_addr_data_in[15:0]), 
                                 .ENA(portA_enable), .RSTA(reset_in),
                                 .CLKA(wb_clock_in), .WEA(wbw_wallow), 
                                 .DOA(wbr_data_out[15:0]),
                                 .ADDRB(portB_addr), .DIB(wbr_data_in[15:0]), 
                                 .ENB(portB_enable), .RSTB(reset_in),
                                 .CLKB(pci_clock_in), .WEB(wbr_wallow), 
                                 .DOB(wbw_addr_data_out[15:0])) ;

        RAMB4_S16_S16 dpram16_2 (.ADDRA(portA_addr), .DIA(wbw_addr_data_in[31:16]), 
                                 .ENA(portA_enable), .RSTA(reset_in),
                                 .CLKA(wb_clock_in), .WEA(wbw_wallow), 
                                 .DOA(wbr_data_out[31:16]),
                                 .ADDRB(portB_addr), .DIB(wbr_data_in[31:16]), 
                                 .ENB(portB_enable), .RSTB(reset_in),
                                 .CLKB(pci_clock_in), .WEB(wbr_wallow), 
                                 .DOB(wbw_addr_data_out[31:16])) ;
    
        RAMB4_S16_S16 dpram16_3 (.ADDRA(portA_addr), .DIA({wbw_control_in, 8'h00, wbw_cbe_in}), 
                                 .ENA(portA_enable), .RSTA(reset_in),
                                 .CLKA(wb_clock_in), .WEA(wbw_wallow), 
                                 .DOA(dpram3_portA_output),
                                 .ADDRB(portB_addr), .DIB({wbr_control_in, 8'h00, wbr_be_in}), 
                                 .ENB(portB_enable), .RSTB(reset_in),
                                 .CLKB(pci_clock_in), .WEB(wbr_wallow), 
                                 .DOB(dpram3_portB_output)) ;
    `endif
    
    
    
    

`else
    wire [39:0] wbw_ram_data_out ;
    wire [39:0] wbw_ram_data_in = {wbw_control_in, wbw_cbe_in, wbw_addr_data_in} ;
    wire [39:0] wbr_ram_data_in = {wbr_control_in, wbr_be_in, wbr_data_in} ;
    wire [39:0] wbr_ram_data_out ;
    assign wbw_control_out   = wbw_ram_data_out[39:36] ;
    assign wbw_cbe_out       = wbw_ram_data_out[35:32] ;
    assign wbw_addr_data_out = wbw_ram_data_out [31:0] ;
    
    assign wbr_control_out   = wbr_ram_data_out[39:36] ;
    assign wbr_be_out        = wbr_ram_data_out[35:32] ;
    assign wbr_data_out      = wbr_ram_data_out [31:0] ;
    
    `ifdef SYNCHRONOUS
    /*-----------------------------------------------------------------------------------------------------------
    ASIC memory primitives will be added here in the near future - currently there is only some generic, 
    behavioral dual port ram here 
    -----------------------------------------------------------------------------------------------------------*/
    DP_SRAM #(WBW_ADDR_LENGTH, WBW_DEPTH) wbw_ram (.reset_in(reset_in), .wclock_in(wb_clock_in), .rclock_in(pci_clock_in), .data_in(wbw_ram_data_in), 
                    .raddr_in(wbw_raddr), .waddr_in(wbw_waddr), .data_out(wbw_ram_data_out), .renable_in(1'b1), .wenable_in(wbw_wallow));
    
    DP_SRAM #(WBR_ADDR_LENGTH, WBR_DEPTH) wbr_ram (.reset_in(reset_in), .wclock_in(pci_clock_in), .rclock_in(wb_clock_in), .data_in(wbr_ram_data_in),
                    .raddr_in(wbr_raddr), .waddr_in(wbr_waddr), .data_out(wbr_ram_data_out), .renable_in(1'b1), .wenable_in(wbr_wallow));
    
    `else //ASYNCHRONOUS RAM
        DP_ASYNC_RAM #(WBW_ADDR_LENGTH, WBW_DEPTH) wbw_ram (.reset_in(reset_in), .wclock_in(wb_clock_in), .data_in(wbw_ram_data_in), 
                    .raddr_in(wbw_raddr), .waddr_in(wbw_waddr), .data_out(wbw_ram_data_out), .wenable_in(wbw_wallow));
    
        DP_ASYNC_RAM #(WBR_ADDR_LENGTH, WBR_DEPTH) wbr_ram (.reset_in(reset_in), .wclock_in(pci_clock_in), .data_in(wbr_ram_data_in),
                    .raddr_in(wbr_raddr), .waddr_in(wbr_waddr), .data_out(wbr_ram_data_out), .wenable_in(wbr_wallow));
    `endif
`endif

/*-----------------------------------------------------------------------------------------------------------
Instantiation of two control logic modules - one for WBW_FIFO and one for WBR_FIFO
-----------------------------------------------------------------------------------------------------------*/
WBW_FIFO_CONTROL #(WBW_ADDR_LENGTH) wbw_fifo_ctrl
              (.rclock_in(pci_clock_in), .wclock_in(wb_clock_in), .renable_in(wbw_renable_in), 
               .wenable_in(wbw_wenable_in), .reset_in(reset_in), .flush_in(wbw_flush_in), 
               .almost_full_out(wbw_almost_full_out), .full_out(wbw_full_out), 
               .empty_out(wbw_empty), 
               .waddr_out(wbw_waddr), .raddr_out(wbw_raddr), 
               .rallow_out(wbw_rallow), .wallow_out(wbw_wallow)); 

WBR_FIFO_CONTROL #(WBR_ADDR_LENGTH) wbr_fifo_ctrl
                  (.rclock_in(wb_clock_in), .wclock_in(pci_clock_in), .renable_in(wbr_renable_in), 
                   .wenable_in(wbr_wenable_in), .reset_in(reset_in), .flush_in(wbr_flush_in), 
                   .empty_out(wbr_empty), 
                   .waddr_out(wbr_waddr), .raddr_out(wbr_raddr), 
                   .rallow_out(wbr_rallow), .wallow_out(wbr_wallow)); 


// in and out transaction counters and grey codes
reg  [(WBW_ADDR_LENGTH-2):0] inGreyCount ;
reg  [(WBW_ADDR_LENGTH-2):0] outGreyCount ;
wire [(WBW_ADDR_LENGTH-2):0] inNextGreyCount = {wbw_inTransactionCount[(WBW_ADDR_LENGTH-2)], wbw_inTransactionCount[(WBW_ADDR_LENGTH-2):1] ^ wbw_inTransactionCount[(WBW_ADDR_LENGTH-3):0]} ;
wire [(WBW_ADDR_LENGTH-2):0] outNextGreyCount = {wbw_outTransactionCount[(WBW_ADDR_LENGTH-2)], wbw_outTransactionCount[(WBW_ADDR_LENGTH-2):1] ^ wbw_outTransactionCount[(WBW_ADDR_LENGTH-3):0]} ;

wire in_count_en  = wbw_wallow && wbw_last_in ;
wire out_count_en = wbw_renable_in && wbw_last_out ;

always@(posedge wb_clock_in or posedge wbw_clear)
begin
    if (wbw_clear)
    begin
        inGreyCount[(WBW_ADDR_LENGTH-2)] <= #`FF_DELAY 1'b1 ;
        inGreyCount[(WBW_ADDR_LENGTH-3):0] <= #`FF_DELAY {(WBW_ADDR_LENGTH-2),1'b0} ;
    end
    else
    if (in_count_en)
        inGreyCount <= #`FF_DELAY inNextGreyCount ;
end

always@(posedge pci_clock_in or posedge wbw_clear)
begin
    if (wbw_clear)
    begin
        outGreyCount[(WBW_ADDR_LENGTH-2)]   <= #`FF_DELAY 1'b1 ;
        outGreyCount[(WBW_ADDR_LENGTH-3):0] <= #`FF_DELAY {(WBW_ADDR_LENGTH-2),1'b0} ;
    end
    else
    if (out_count_en)
        outGreyCount <= #`FF_DELAY outNextGreyCount ;
end

always@(posedge wb_clock_in or posedge wbw_clear)
begin
    if (wbw_clear)
        wbw_inTransactionCount <= #`FF_DELAY {(WBW_ADDR_LENGTH-1){1'b0}} ;
    else
    if (in_count_en)
        wbw_inTransactionCount <= #`FF_DELAY wbw_inTransactionCount + 1'b1 ;
end

always@(posedge pci_clock_in or posedge wbw_clear)
begin
    if (wbw_clear)
        wbw_outTransactionCount <= #`FF_DELAY {(WBW_ADDR_LENGTH-1){1'b0}} ;
    else
    if (out_count_en)
        wbw_outTransactionCount <= #`FF_DELAY wbw_outTransactionCount + 1'b1 ;
end

/*always@(posedge pci_clock_in or posedge wbr_clear)
begin
    if (wbr_clear)
        wbr_inTransactionCount <= #`FF_DELAY 1'b0 ;
    else
        if (wbr_last_in && wbr_wallow)
            wbr_inTransactionCount <= #`FF_DELAY ~wbr_inTransactionCount ;
end
        
always@(posedge wb_clock_in or posedge wbr_clear)
begin
    if (wbr_clear)
        wbr_outTransactionCount <= #`FF_DELAY 1'b0 ;
    else
        if (wbr_last_out)
            wbr_outTransactionCount <= #`FF_DELAY ~wbr_outTransactionCount ;
end
*/

// synchronize transaction ready output to reading clock
reg wbw_transaction_ready_out ;
always@(posedge pci_clock_in or posedge wbw_clear)
begin
    if (wbw_clear)
        wbw_transaction_ready_out <= #`FF_DELAY 1'b0 ;
    else
    if ( out_count_en )
        wbw_transaction_ready_out <= #`FF_DELAY 1'b0 ;
    else
        wbw_transaction_ready_out <= #`FF_DELAY inGreyCount != outGreyCount ;
end

endmodule

