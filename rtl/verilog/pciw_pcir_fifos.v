//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "pciw_pcir_fifos.v"                               ////
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
//// Copyright (C) 2000 Miha Dolenc, mihad@opencores.org          ////
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

module PCIW_PCIR_FIFOS
( 
    wb_clock_in, 
    pci_clock_in, 
    reset_in, 
    pciw_wenable_in, 
    pciw_addr_data_in, 
    pciw_cbe_in, 
    pciw_control_in,                     
    pciw_renable_in, 
    pciw_addr_data_out, 
    pciw_cbe_out, 
    pciw_control_out,                     
    pciw_flush_in, 
    pciw_two_left_out, 
    pciw_almost_full_out, 
    pciw_full_out,
    pciw_almost_empty_out, 
    pciw_empty_out, 
    pciw_transaction_ready_out, 
    pcir_wenable_in, 
    pcir_data_in, 
    pcir_be_in, 
    pcir_control_in,                     
    pcir_renable_in, 
    pcir_data_out, 
    pcir_be_out, 
    pcir_control_out,                     
    pcir_flush_in, 
    pcir_almost_full_out, 
    pcir_full_out,
    pcir_almost_empty_out, 
    pcir_empty_out, 
    pcir_transaction_ready_out
) ;

/*-----------------------------------------------------------------------------------------------------------
System inputs:
wb_clock_in - WISHBONE bus clock
pci_clock_in - PCI bus clock
reset_in - reset from control logic
-------------------------------------------------------------------------------------------------------------*/
input wb_clock_in, pci_clock_in, reset_in ;

/*-----------------------------------------------------------------------------------------------------------
PCI WRITE FIFO interface signals prefixed with pciw_ - FIFO is used for posted writes initiated by external
PCI master through PCI target interface, traveling through FIFO and are completed on WISHBONE by 
WISHBONE master interface 

write enable signal:
pciw_wenable_in = write enable input for PCIW_FIFO - driven by PCI TARGET interface

data input signals:
pciw_addr_data_in = data input - data from PCI bus - first entry of transaction is address others are data entries
pciw_cbe_in       = bus command/byte enable(~#BE[3:0]) input - first entry of transaction is bus command, other are byte enables
pciw_control_in   = control input - encoded control bus input

read enable signal:
pciw_renable_in = read enable input driven by WISHBONE master interface

data output signals:
pciw_addr_data_out = data output - data from PCI bus - first entry of transaction is address, others are data entries
pciw_cbe_out      = bus command/byte enable output - first entry of transaction is bus command, others are byte enables
pciw_control_out = control input - encoded control bus input

status signals - monitored by various resources in the core
pciw_flush_in = flush signal input for PCIW_FIFO - when asserted, fifo is flushed(emptied)
pciw_almost_full_out = almost full output from PCIW_FIFO
pciw_full_out = full output from PCIW_FIFO
pciw_almost_empty_out = almost empty output from PCIW_FIFO
pciw_empty_out = empty output from PCIW_FIFO
pciw_transaction_ready_out = output indicating that one complete transaction is waiting in PCIW_FIFO
-----------------------------------------------------------------------------------------------------------*/
// input control and data
input        pciw_wenable_in ;
input [31:0] pciw_addr_data_in ;
input [3:0]  pciw_cbe_in ;
input [3:0]  pciw_control_in ;

// output control and data
input         pciw_renable_in ;
output [31:0] pciw_addr_data_out ;
output [3:0]  pciw_cbe_out ;
output [3:0]  pciw_control_out ;                    

// flush input
input pciw_flush_in ; 

// status outputs
output pciw_two_left_out ;
output pciw_almost_full_out ;
output pciw_full_out ;
output pciw_almost_empty_out ;
output pciw_empty_out ;
output pciw_transaction_ready_out ;

/*-----------------------------------------------------------------------------------------------------------
PCI READ FIFO interface signals prefixed with pcir_ - FIFO is used for holding delayed read completions 
initiated by master on PCI bus and completed on WISHBONE bus, 

write enable signal:
pcir_wenable_in = write enable input for PCIR_FIFO - driven by WISHBONE master interface

data input signals:
pcir_data_in      = data input - data from WISHBONE bus - there is no address entry here, since address is stored in separate register
pcir_be_in        = byte enable(~SEL[3:0]) input - byte enables - same through one transaction
pcir_control_in   = control input - encoded control bus input

read enable signal:
pcir_renable_in = read enable input driven by PCI target interface

data output signals:
pcir_data_out = data output - data from WISHBONE bus
pcir_be_out      = byte enable output(~SEL)
pcir_control_out = control output - encoded control bus output

status signals - monitored by various resources in the core
pcir_flush_in = flush signal input for PCIR_FIFO - when asserted, fifo is flushed(emptied)
pcir_almost_full_out = almost full output from PCIR_FIFO
pcir full_out = full output from PCIR_FIFO
pcir_almost_empty_out = almost empty output from PCIR_FIFO
pcir_empty_out = empty output from PCIR_FIFO
pcir_transaction_ready_out = output indicating that one complete transaction is waiting in PCIR_FIFO
-----------------------------------------------------------------------------------------------------------*/
// input control and data
input        pcir_wenable_in ;
input [31:0] pcir_data_in ;
input [3:0]  pcir_be_in ;
input [3:0]  pcir_control_in ;                     

// output control and data
input         pcir_renable_in ;
output [31:0] pcir_data_out ;
output [3:0]  pcir_be_out ;
output [3:0]  pcir_control_out ;                    

// flush input
input pcir_flush_in ; 

// status outputs
output pcir_almost_full_out ;
output pcir_full_out ;
output pcir_almost_empty_out ;
output pcir_empty_out ;
output pcir_transaction_ready_out ;

/*-----------------------------------------------------------------------------------------------------------
Address length parameters:
PCIW_DEPTH = defines PCIW_FIFO depth
PCIR_DEPTH = defines PCIR_FIFO depth
PCIW_ADDR_LENGTH = defines PCIW_FIFO's location address length - log2(PCIW_DEPTH)
PCIR_ADDR_LENGTH = defines PCIR_FIFO's location address length - log2(PCIR_DEPTH)
-----------------------------------------------------------------------------------------------------------*/
parameter PCIW_DEPTH = `PCIW_DEPTH ;
parameter PCIW_ADDR_LENGTH = `PCIW_ADDR_LENGTH ;
parameter PCIR_DEPTH = `PCIR_DEPTH ;
parameter PCIR_ADDR_LENGTH = `PCIR_ADDR_LENGTH ;

// obvious
wire vcc = 1'b1 ;
wire gnd = 1'b0 ;

/*-----------------------------------------------------------------------------------------------------------
pciw_wallow = PCIW_FIFO write allow wire - writes to FIFO are allowed when FIFO isn't full and write enable is 1
pciw_rallow = PCIW_FIFO read allow wire - reads from FIFO are allowed when FIFO isn't empty and read enable is 1
-----------------------------------------------------------------------------------------------------------*/
wire pciw_wallow ;
wire pciw_rallow ;

/*-----------------------------------------------------------------------------------------------------------
pcir_wallow = PCIR_FIFO write allow wire - writes to FIFO are allowed when FIFO isn't full and write enable is 1
pcir_rallow = PCIR_FIFO read allow wire - reads from FIFO are allowed when FIFO isn't empty and read enable is 1
-----------------------------------------------------------------------------------------------------------*/
wire pcir_wallow ;
wire pcir_rallow ;

/*-----------------------------------------------------------------------------------------------------------
wires for address port conections from PCIW_FIFO control logic to RAM blocks used for PCIW_FIFO
-----------------------------------------------------------------------------------------------------------*/
wire [(PCIW_ADDR_LENGTH - 1):0] pciw_raddr ;
wire [(PCIW_ADDR_LENGTH - 1):0] pciw_waddr ;

/*-----------------------------------------------------------------------------------------------------------
wires for address port conections from PCIR_FIFO control logic to RAM blocks used for PCIR_FIFO
-----------------------------------------------------------------------------------------------------------*/
wire [(PCIR_ADDR_LENGTH - 1):0] pcir_raddr ;
wire [(PCIR_ADDR_LENGTH - 1):0] pcir_waddr ;

/*-----------------------------------------------------------------------------------------------------------
PCIW_FIFO transaction counters: used to count incoming transactions and outgoing transactions. When number of
input transactions is equal to number of output transactions, it means that there isn't any complete transaction
currently present in the FIFO.
-----------------------------------------------------------------------------------------------------------*/
reg [(PCIW_ADDR_LENGTH - 1):0] pciw_inTransactionCount ;
reg [(PCIW_ADDR_LENGTH - 1):0] pciw_outTransactionCount ;

/*-----------------------------------------------------------------------------------------------------------
FlipFlops for indicating if complete delayed read completion is present in the FIFO
-----------------------------------------------------------------------------------------------------------*/
/*reg pcir_inTransactionCount ;
reg pcir_outTransactionCount ;*/
/*-----------------------------------------------------------------------------------------------------------
wires monitoring control bus. When control bus on a write transaction has a value of `LAST, it means that
complete transaction is in the FIFO. When control bus on a read transaction has a value of `LAST,
it means that there was one complete transaction taken out of FIFO.
-----------------------------------------------------------------------------------------------------------*/
wire pciw_last_in  = pciw_control_in[`LAST_CTRL_BIT] ;
wire pciw_last_out = pciw_control_out[`LAST_CTRL_BIT] ;

/*wire pcir_last_in  = pcir_wallow && (pcir_control_in == `LAST) ;
wire pcir_last_out = pcir_rallow && (pcir_control_out == `LAST) ;*/

wire pciw_empty ;
wire pcir_empty ;

assign pciw_empty_out = pciw_empty ;
assign pcir_empty_out = pcir_empty ;

// clear wires for clearing FFs and registers
wire pciw_clear = reset_in || pciw_flush_in ; // PCIW_FIFO's clear signal
wire pcir_clear = reset_in || pcir_flush_in ; // PCIR_FIFO's clear signal

`ifdef FPGA
/*-----------------------------------------------------------------------------------------------------------
this code is included only for FPGA core usage - somewhat different logic because of sharing
one block selectRAM+ between two FIFOs
-----------------------------------------------------------------------------------------------------------*/
    `ifdef BIG
        /*-----------------------------------------------------------------------------------------------------------
        Big FPGAs
        PCIW_FIFO and PCIR_FIFO address prefixes - used for extending read and write addresses because of varible
        FIFO depth and fixed SelectRAM+ size. Addresses are zero paded on the left to form long enough address
        -----------------------------------------------------------------------------------------------------------*/
        wire [(7 - PCIW_ADDR_LENGTH):0] pciw_addr_prefix = {( 8 - PCIW_ADDR_LENGTH){1'b0}} ;
        wire [(7 - PCIR_ADDR_LENGTH):0] pcir_addr_prefix = {( 8 - PCIR_ADDR_LENGTH){1'b0}} ;

        // compose addresses
        wire [7:0] pciw_whole_waddr = {pciw_addr_prefix, pciw_waddr} ;
        wire [7:0] pciw_whole_raddr = {pciw_addr_prefix, pciw_raddr} ;
        
        wire [7:0] pcir_whole_waddr = {pcir_addr_prefix, pcir_waddr} ;
        wire [7:0] pcir_whole_raddr = {pcir_addr_prefix, pcir_raddr} ;

        /*-----------------------------------------------------------------------------------------------------------
        Only 8 bits out of 16 are used in ram3 and ram6 - wires for referencing them
        -----------------------------------------------------------------------------------------------------------*/
        wire [15:0] dpram3_portB_output ;
        wire [15:0] dpram6_portA_output ;

        /*-----------------------------------------------------------------------------------------------------------
        Control out assignements from ram3 output
        -----------------------------------------------------------------------------------------------------------*/
        assign pciw_control_out = dpram3_portB_output[15:12] ;
        assign pcir_control_out = dpram6_portA_output[15:12] ;

        assign pciw_cbe_out = dpram3_portB_output[3:0] ;
        assign pcir_be_out  = dpram6_portA_output[3:0] ;

        wire pciw_read_enable = pciw_rallow || pciw_empty ;
        wire pcir_read_enable = pcir_rallow || pcir_empty ;
        
        // Block SelectRAM+ cells instantiation
        RAMB4_S16_S16 dpram16_1 (.ADDRA(pciw_whole_waddr), .DIA(pciw_addr_data_in[15:0]), 
                                 .ENA(vcc), .RSTA(reset_in),
                                 .CLKA(pci_clock_in), .WEA(pciw_wallow), 
                                 .DOA(),
                                 .ADDRB(pciw_whole_raddr), .DIB(16'h0000), 
                                 .ENB(pciw_read_enable), .RSTB(reset_in),
                                 .CLKB(wb_clock_in), .WEB(gnd), 
                                 .DOB(pciw_addr_data_out[15:0])) ;

        RAMB4_S16_S16 dpram16_2 (.ADDRA(pciw_whole_waddr), .DIA(pciw_addr_data_in[31:16]), 
                                 .ENA(vcc), .RSTA(reset_in),
                                 .CLKA(pci_clock_in), .WEA(pciw_wallow), 
                                 .DOA(),
                                 .ADDRB(pciw_whole_raddr), .DIB(16'h0000), 
                                 .ENB(pciw_read_enable), .RSTB(reset_in),
                                 .CLKB(wb_clock_in), .WEB(gnd), 
                                 .DOB(pciw_addr_data_out[31:16])) ;
    
        RAMB4_S16_S16 dpram16_3 (.ADDRA(pciw_whole_waddr), .DIA({pciw_control_in, 8'h00, pciw_cbe_in}), 
                                 .ENA(vcc), .RSTA(reset_in),
                                 .CLKA(pci_clock_in), .WEA(pciw_wallow), 
                                 .DOA(),
                                 .ADDRB(pciw_whole_raddr), .DIB(16'h0000), 
                                 .ENB(pciw_read_enable), .RSTB(reset_in),
                                 .CLKB(wb_clock_in), .WEB(gnd), 
                                 .DOB(dpram3_portB_output)) ;

        RAMB4_S16_S16 dpram16_4 (.ADDRA(pcir_whole_raddr), .DIA(16'h0000), 
                                 .ENA(pcir_read_enable), .RSTA(reset_in),
                                 .CLKA(pci_clock_in), .WEA(gnd), 
                                 .DOA(pcir_data_out[15:0]),
                                 .ADDRB(pcir_whole_waddr), .DIB(pcir_data_in[15:0]), 
                                 .ENB(vcc), .RSTB(reset_in),
                                 .CLKB(wb_clock_in), .WEB(pcir_wallow), 
                                 .DOB()) ;

        RAMB4_S16_S16 dpram16_5 (.ADDRA(pcir_whole_raddr), .DIA(16'h0000), 
                                 .ENA(pcir_read_enable), .RSTA(reset_in),
                                 .CLKA(pci_clock_in), .WEA(gnd), 
                                 .DOA(pcir_data_out[31:16]),
                                 .ADDRB(pcir_whole_waddr), .DIB(pcir_data_in[31:16]), 
                                 .ENB(vcc), .RSTB(reset_in),
                                 .CLKB(wb_clock_in), .WEB(pcir_wallow), 
                                 .DOB()) ;
    
        RAMB4_S16_S16 dpram16_6 (.ADDRA(pcir_whole_raddr), .DIA(16'h0000), 
                                 .ENA(pcir_read_enable), .RSTA(reset_in),
                                 .CLKA(pci_clock_in), .WEA(gnd), 
                                 .DOA(dpram6_portA_output),
                                 .ADDRB(pcir_whole_waddr), .DIB({pcir_control_in, 8'h00, pcir_be_in}), 
                                 .ENB(vcc), .RSTB(reset_in),
                                 .CLKB(wb_clock_in), .WEB(pcir_wallow), 
                                 .DOB()) ;

    `else // SMALL FPGAs

        /*-----------------------------------------------------------------------------------------------------------
        Small FPGAs
        PCIW_FIFO and PCIR_FIFO address prefixes - used for extending read and write addresses because of varible
        FIFO depth and fixed SelectRAM+ size. Addresses are always paded, because of RAM sharing between FIFOs
        PCIW addresses are zero padded on the left, PCIR addresses are padded
        with ones on the left
        -----------------------------------------------------------------------------------------------------------*/
        wire [(7 - PCIW_ADDR_LENGTH):0] pciw_addr_prefix = {( 8 - PCIW_ADDR_LENGTH){1'b0}} ;
        wire [(7 - PCIR_ADDR_LENGTH):0] pcir_addr_prefix = {( 8 - PCIR_ADDR_LENGTH){1'b1}} ;

        /*-----------------------------------------------------------------------------------------------------------
        Only 8 bits out of 16 are used in ram3 - wires for referencing them
        -----------------------------------------------------------------------------------------------------------*/
        wire [15:0] dpram3_portA_output ;
        wire [15:0] dpram3_portB_output ;

        /*-----------------------------------------------------------------------------------------------------------
        Control out assignements from ram3 output
        -----------------------------------------------------------------------------------------------------------*/
        assign pciw_control_out = dpram3_portB_output[15:12] ;
        assign pcir_control_out = dpram3_portA_output[15:12] ;

        assign pciw_cbe_out = dpram3_portB_output[3:0] ;
        assign pcir_be_out  = dpram3_portA_output[3:0] ;
        
        /*-----------------------------------------------------------------------------------------------------------
        Logic used for extending port's enable input for one clock cycle to allow address and date change from 
        PCI write fifo's write address and data back to PCI read fifo's address and data ( turnaround cycle )
        -----------------------------------------------------------------------------------------------------------*/
        reg pciw_write_performed ;
        always@(posedge pci_clock_in or posedge reset_in)
        begin
            if (reset_in)
                pciw_write_performed <= #`FF_DELAY 1'b0 ;
            else 
                pciw_write_performed <= #`FF_DELAY pciw_wallow ;
        end

        /*-----------------------------------------------------------------------------------------------------------
        Logic used for extending port's enable input for one clock cycle to allow address and date change from 
        PCI read fifo's write address and data back to PCI write fifo's address and data ( turnaround cycle )
        -----------------------------------------------------------------------------------------------------------*/
        reg pcir_write_performed ;
        always@(posedge wb_clock_in or posedge reset_in)
        begin
            if (reset_in)
                pcir_write_performed <= #`FF_DELAY 1'b0 ;
            else 
                pcir_write_performed <= #`FF_DELAY pcir_wallow ;
        end

        /*-----------------------------------------------------------------------------------------------------------
        Additional register storing actual PCIW read address. It must be applied to port B during turnaround cycle
        -----------------------------------------------------------------------------------------------------------*/
        reg [(PCIW_ADDR_LENGTH - 1):0] pciw_raddr_0 ;

        always@(posedge wb_clock_in or posedge pciw_clear)
        begin
            if (pciw_clear)
                pciw_raddr_0 <= #`FF_DELAY {PCIW_ADDR_LENGTH{1'b0}} ;
            else
                if(pciw_rallow)
                    pciw_raddr_0 <= #`FF_DELAY pciw_raddr ;
        end 
        
        wire [(PCIW_ADDR_LENGTH - 1):0] pciw_raddr_calc = pcir_write_performed ? pciw_raddr_0 : pciw_raddr ;

        /*-----------------------------------------------------------------------------------------------------------
        Additional register storing actual PCIR read address. It must be applied to port A during turnaround cycle
        -----------------------------------------------------------------------------------------------------------*/
        reg [(PCIR_ADDR_LENGTH - 1):0] pcir_raddr_0 ;

        always@(posedge pci_clock_in or posedge pcir_clear)
        begin
            if(pcir_clear)
                pcir_raddr_0 <= #`FF_DELAY {PCIR_ADDR_LENGTH{1'b0}} ;
            else
                if(pcir_rallow)
                    pcir_raddr_0 <= #`FF_DELAY pcir_raddr ;
        end 
    
        wire [(PCIR_ADDR_LENGTH - 1):0] pcir_raddr_calc = pciw_write_performed ? pcir_raddr_0 : pcir_raddr ;
    
        /*-----------------------------------------------------------------------------------------------------------
        Port A and B enables
        -----------------------------------------------------------------------------------------------------------*/
        wire portA_enable = pciw_wallow || pcir_rallow || pcir_empty || pciw_write_performed ;
        wire portB_enable = pcir_wallow || pciw_rallow || pciw_empty || pcir_write_performed ;

        /*-----------------------------------------------------------------------------------------------------------
        Port A address generation for block SelectRam+ in SpartanII or Virtex
        Port A is clocked by PCI clock, DIA is input for pciw_fifo, DOA is output for pcir_fifo. Address is multiplexed
        between two values.
        Address multiplexing:
        pciw_wenable == 1 => ADDRA = pciw_waddr (write pointer of PCIW_FIFO)
        else                ADDRA = pcir_raddr (read pointer of PCIR_FIFO)
        -----------------------------------------------------------------------------------------------------------*/
        wire [7:0] portA_addr = pciw_wallow ? {pciw_addr_prefix, pciw_waddr} : {pcir_addr_prefix, pcir_raddr_calc} ;
    
        /*-----------------------------------------------------------------------------------------------------------
        Port B address generation for block SelectRam+ in SpartanII or Virtex
        Port B is clocked by PCI clock, DIB is input for pcir_fifo, DOB is output for pciw_fifo. Address is multiplexed
        between two values.
        Address multiplexing:
        pcir_wenable == 1 => ADDRB = pcir_waddr (write pointer of PCIR_FIFO)
        else                ADDRB = pciw_raddr (read pointer of PCIW_FIFO)
        -----------------------------------------------------------------------------------------------------------*/
        wire [7:0] portB_addr = pcir_wallow ? {pcir_addr_prefix, pcir_waddr} : {pciw_addr_prefix, pciw_raddr_calc} ;
    
        // Block SelectRAM+ cells instantiation
        RAMB4_S16_S16 dpram16_1 (.ADDRA(portA_addr), .DIA(pciw_addr_data_in[15:0]), 
                                 .ENA(portA_enable), .RSTA(reset_in),
                                 .CLKA(pci_clock_in), .WEA(pciw_wallow), 
                                 .DOA(pcir_data_out[15:0]),
                                 .ADDRB(portB_addr), .DIB(pcir_data_in[15:0]), 
                                 .ENB(portB_enable), .RSTB(reset_in),
                                 .CLKB(wb_clock_in), .WEB(pcir_wallow), 
                                 .DOB(pciw_addr_data_out[15:0])) ;

        RAMB4_S16_S16 dpram16_2 (.ADDRA(portA_addr), .DIA(pciw_addr_data_in[31:16]), 
                                 .ENA(portA_enable), .RSTA(reset_in),
                                 .CLKA(pci_clock_in), .WEA(pciw_wallow), 
                                 .DOA(pcir_data_out[31:16]),
                                 .ADDRB(portB_addr), .DIB(pcir_data_in[31:16]), 
                                 .ENB(portB_enable), .RSTB(reset_in),
                                 .CLKB(wb_clock_in), .WEB(pcir_wallow), 
                                 .DOB(pciw_addr_data_out[31:16])) ;
    
        RAMB4_S16_S16 dpram16_3 (.ADDRA(portA_addr), .DIA({pciw_control_in, 8'h00, pciw_cbe_in}), 
                                 .ENA(portA_enable), .RSTA(reset_in),
                                 .CLKA(pci_clock_in), .WEA(pciw_wallow), 
                                 .DOA(dpram3_portA_output),
                                 .ADDRB(portB_addr), .DIB({pcir_control_in, 8'h00, pcir_be_in}), 
                                 .ENB(portB_enable), .RSTB(reset_in),
                                 .CLKB(wb_clock_in), .WEB(pcir_wallow), 
                                 .DOB(dpram3_portB_output)) ;
    `endif
    
    
    
    

`else
    wire [39:0] pciw_ram_data_out ;
    wire [39:0] pciw_ram_data_in = {pciw_control_in, pciw_cbe_in, pciw_addr_data_in} ;
    wire [39:0] pcir_ram_data_in = {pcir_control_in, pcir_be_in, pcir_data_in} ;
    wire [39:0] pcir_ram_data_out ;
    assign pciw_control_out   = pciw_ram_data_out[39:36] ;
    assign pciw_cbe_out       = pciw_ram_data_out[35:32] ;
    assign pciw_addr_data_out = pciw_ram_data_out [31:0] ;
    
    assign pcir_control_out   = pcir_ram_data_out[39:36] ;
    assign pcir_be_out        = pcir_ram_data_out[35:32] ;
    assign pcir_data_out      = pcir_ram_data_out [31:0] ;
    
    `ifdef SYNCHRONOUS
    /*-----------------------------------------------------------------------------------------------------------
    ASIC memory primitives will be added here in the near future - currently there is only some generic, 
    behavioral dual port ram here 
    -----------------------------------------------------------------------------------------------------------*/

    wire pciw_read_enable = pciw_rallow || pciw_empty ;
    wire pcir_read_enable = pcir_rallow || pcir_empty ;

    DP_SRAM #(PCIW_ADDR_LENGTH, PCIW_DEPTH) pciw_ram (.reset_in(reset_in), .wclock_in(pci_clock_in), .rclock_in(wb_clock_in), .data_in(pciw_ram_data_in), 
                    .raddr_in(pciw_raddr), .waddr_in(pciw_waddr), .data_out(pciw_ram_data_out), .renable_in(pciw_read_enable), .wenable_in(pciw_wallow));
    
    DP_SRAM #(PCIR_ADDR_LENGTH, PCIR_DEPTH) pcir_ram (.reset_in(reset_in), .wclock_in(wb_clock_in), .rclock_in(pci_clock_in), .data_in(pcir_ram_data_in),
                    .raddr_in(pcir_raddr), .waddr_in(pcir_waddr), .data_out(pcir_ram_data_out), .renable_in(pcir_read_enable), .wenable_in(pcir_wallow));
    
    `else //ASYNCHRONOUS RAM
        DP_ASYNC_RAM #(PCIW_ADDR_LENGTH, PCIW_DEPTH) pciw_ram (.reset_in(reset_in), .wclock_in(pci_clock_in), .data_in(pciw_ram_data_in), 
                    .raddr_in(pciw_raddr), .waddr_in(pciw_waddr), .data_out(pciw_ram_data_out), .wenable_in(pciw_wallow));
    
        DP_ASYNC_RAM #(PCIR_ADDR_LENGTH, PCIR_DEPTH) pcir_ram (.reset_in(reset_in), .wclock_in(wb_clock_in), .data_in(pcir_ram_data_in),
                    .raddr_in(pcir_raddr), .waddr_in(pcir_waddr), .data_out(pcir_ram_data_out), .wenable_in(pcir_wallow));
    `endif
`endif

/*-----------------------------------------------------------------------------------------------------------
Instantiation of two control logic modules - one for PCIW_FIFO and one for PCIR_FIFO
-----------------------------------------------------------------------------------------------------------*/
PCIW_FIFO_CONTROL #(PCIW_ADDR_LENGTH) pciw_fifo_ctrl
(
    .rclock_in(wb_clock_in), 
    .wclock_in(pci_clock_in), 
    .renable_in(pciw_renable_in), 
    .wenable_in(pciw_wenable_in), 
    .reset_in(reset_in), 
    .flush_in(pciw_flush_in), 
    .two_left_out(pciw_two_left_out),
    .almost_full_out(pciw_almost_full_out), 
    .full_out(pciw_full_out), 
    .almost_empty_out(pciw_almost_empty_out), 
    .empty_out(pciw_empty), 
    .waddr_out(pciw_waddr), 
    .raddr_out(pciw_raddr), 
    .rallow_out(pciw_rallow), 
    .wallow_out(pciw_wallow)
); 

FIFO_CONTROL #(PCIR_ADDR_LENGTH) pcir_fifo_ctrl
(
    .rclock_in(pci_clock_in), 
    .wclock_in(wb_clock_in), 
    .renable_in(pcir_renable_in), 
    .wenable_in(pcir_wenable_in), 
    .reset_in(reset_in), 
    .flush_in(pcir_flush_in), 
    .almost_full_out(pcir_almost_full_out), 
    .full_out(pcir_full_out), 
    .almost_empty_out(pcir_almost_empty_out), 
    .empty_out(pcir_empty), 
    .waddr_out(pcir_waddr), 
    .raddr_out(pcir_raddr), 
    .rallow_out(pcir_rallow), 
    .wallow_out(pcir_wallow)
); 


// in and out transaction counters and grey codes
reg  [(PCIW_ADDR_LENGTH-2):0] inGreyCount ;
reg  [(PCIW_ADDR_LENGTH-2):0] outGreyCount ;
wire [(PCIW_ADDR_LENGTH-2):0] inNextGreyCount  = {pciw_inTransactionCount[(PCIW_ADDR_LENGTH-2)], pciw_inTransactionCount[(PCIW_ADDR_LENGTH-2):1] ^ pciw_inTransactionCount[(PCIW_ADDR_LENGTH-3):0]} ;
wire [(PCIW_ADDR_LENGTH-2):0] outNextGreyCount = {pciw_outTransactionCount[(PCIW_ADDR_LENGTH-2)], pciw_outTransactionCount[(PCIW_ADDR_LENGTH-2):1] ^ pciw_outTransactionCount[(PCIW_ADDR_LENGTH-3):0]} ;

wire in_count_en  = pciw_wallow     && pciw_last_in ;
wire out_count_en = pciw_renable_in && pciw_last_out ;

always@(posedge pci_clock_in or posedge pciw_clear)
begin
    if (pciw_clear)
    begin
        inGreyCount[(PCIW_ADDR_LENGTH-2)] <= #`FF_DELAY 1'b1 ;
        inGreyCount[(PCIW_ADDR_LENGTH-3):0] <= #`FF_DELAY {(PCIW_ADDR_LENGTH-2),1'b0} ;
    end
    else
    if (in_count_en)
        inGreyCount <= #`FF_DELAY inNextGreyCount ;
end

always@(posedge wb_clock_in or posedge pciw_clear)
begin
    if (pciw_clear)
    begin
        outGreyCount[(PCIW_ADDR_LENGTH-2)]   <= #`FF_DELAY 1'b1 ;
        outGreyCount[(PCIW_ADDR_LENGTH-3):0] <= #`FF_DELAY {(PCIW_ADDR_LENGTH-2),1'b0} ;
    end
    else
    if (out_count_en)
        outGreyCount <= #`FF_DELAY outNextGreyCount ;
end

always@(posedge pci_clock_in or posedge pciw_clear)
begin
    if (pciw_clear)
        pciw_inTransactionCount <= #`FF_DELAY {(PCIW_ADDR_LENGTH-1){1'b0}} ;
    else
    if (in_count_en)
        pciw_inTransactionCount <= #`FF_DELAY pciw_inTransactionCount + 1'b1 ;
end

always@(posedge wb_clock_in or posedge pciw_clear)
begin
    if (pciw_clear)
        pciw_outTransactionCount <= #`FF_DELAY {(PCIW_ADDR_LENGTH-1){1'b0}} ;
    else
    if (out_count_en)
        pciw_outTransactionCount <= #`FF_DELAY pciw_outTransactionCount + 1'b1 ;
end

/*always@(posedge wb_clock_in or posedge pcir_clear)
begin
    if (pcir_clear)
        pcir_inTransactionCount <= #`FF_DELAY 1'b0 ;
    else
        if (pcir_last_in && pcir_wallow)
            pcir_inTransactionCount <= #`FF_DELAY ~pcir_inTransactionCount ;
end
        
always@(posedge pci_clock_in or posedge pcir_clear)
begin
    if (pcir_clear)
        pcir_outTransactionCount <= #`FF_DELAY 1'b0 ;
    else
        if (pcir_last_out)
            pcir_outTransactionCount <= #`FF_DELAY ~pcir_outTransactionCount ;
end
*/

reg pciw_transaction_ready_out ;
always@(posedge wb_clock_in or posedge pciw_clear)
begin
    if (pciw_clear)
        pciw_transaction_ready_out <= #`FF_DELAY 1'b0 ;
    else
    if ( out_count_en )
        pciw_transaction_ready_out <= #`FF_DELAY 1'b0 ;
    else
        pciw_transaction_ready_out <= #`FF_DELAY inGreyCount != outGreyCount ;
end

assign pcir_transaction_ready_out  = 1'b0 ;

endmodule

