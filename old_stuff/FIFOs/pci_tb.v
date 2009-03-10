//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "pci_tb.v"                                        ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - mihad@opencores.org                                   ////
////      - Miha Dolenc                                           ////
////                                                              ////
////  All additional information is avaliable in the README.txt   ////
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
// Revision 1.3  2001/06/12 11:15:11  mihad
// Changed module parameters
//
//

`include "constants.v"
`include "pciw_pcir_fifos.v"

// Common definitions for simulation purposes
`define Tpci 75 // Tp/2 = 7.5ns => Tp = 15ns => Fpci = 66MHz
`define Twb 25  // Tp/2 = 2.5ns =>   Tp = 5ns  => Fwb  = 200MHz

module PCI_TB() ;

reg [31:0] pciw_data_in ;
wire [31:0] pciw_data_out;
reg [3:0]  pciw_cbe_in ;
wire [3:0] pciw_cbe_out ;
reg [3:0] pciw_control_in ;
wire [3:0] pciw_control_out ;

wire [31:0] pcir_data_out;
reg [31:0] pcir_data_in;
wire [3:0] pcir_be_out ;
reg [3:0] pcir_be_in ;
reg [3:0] pcir_control_in ;
wire [3:0] pcir_control_out ;

wire pciw_full;
wire pciw_empty ;
wire pciw_almost_full ;
wire pciw_almost_empty ;
reg pciw_wenable ;
reg pciw_renable ;
wire pciw_transaction_ready ;

wire pcir_full;
wire pcir_empty ;
wire pcir_almost_full ;
wire pcir_almost_empty ;
reg pcir_wenable ;
reg pcir_renable ;
wire pcir_transaction_ready ;

reg reset ;
reg pci_clock ;
reg wb_clock ;

reg pcir_flush ;
reg pciw_flush ;

`ifdef FPGA
    assign glbl.GSR = reset ;
`endif

// initial values
initial
begin
    reset <= 1'b1 ;

    pci_clock <= 1'b1 ;
    wb_clock <= 1'b0 ;
    pciw_data_in <= 32'h00000001 ;
    pciw_cbe_in  <= 4'h0 ;
    pcir_be_in  <= 4'h0 ;
    pciw_control_in <= 4'h1 ;
    pciw_wenable <= 1'b0;
    pciw_renable <= 1'b0 ;
    pcir_control_in <= 4'h1 ;
    pcir_wenable <= 1'b0;
    pcir_renable <= 1'b0 ;
    pcir_data_in <= 32'h00000001 ;
    pcir_flush <= 1'b0 ;

    #10 reset = 1'b0 ;
    run_tests ;
end 

// clock generation
always 
begin
   #`Tpci pci_clock = ~pci_clock ;
end

always 
begin
   #`Twb wb_clock = ~wb_clock ;
end

task run_tests;
begin
    pciw_empty_nempty_transition ;
    $display("Empty/not empty transition test for PCIW completed") ;
    pciw_full_empty ;
    $display("Full/empty status generation test for PCIW completed") ;
    
    pcir_empty_nempty_transition ;
    $display("Empty/not empty transition test for PCIR completed") ;
    pcir_full_empty ;
    $display("Full/empty status generation test for PCIR completed") ;

    simultaneous_operation ;
    $stop ;
end
endtask

task pciw_empty_nempty_transition ;
    integer index ;
    integer read_data ;
begin
    read_data = 1 ;
    fork
    begin:write_pciw_once
        // repeat the test several times, so clock to clock transition time changes
        for(index = 2; index <= 101; index = index + 1)
        begin
            // wait for PCIW to become empty
            wait(pciw_empty);
            // sample empty on posedge of clock
            @(posedge pci_clock)
                pciw_wenable <= #`FF_DELAY 1'b1;
                
            // data gets written at clock edge following the one empty was asserted
            @(posedge pci_clock)
            begin
                // prepare data for next write
                pciw_wenable <= #`FF_DELAY 1'b0 ;
                pciw_data_in <= #`FF_DELAY index ;
                pciw_control_in <= #`FF_DELAY index[3:0] ;
                pciw_cbe_in <= #`FF_DELAY index[7:4] ;
                // wait for PCIW to be empty again
                wait(~pciw_empty) ;
            end
        end
    end

    begin:read_pciw
        // repeat test several times
        while(read_data <= 100)
        begin
            wait (~pciw_empty) ;
            @(posedge wb_clock)
                // at first clock edge empty is deasserted assert read enable, so read will be performed on the next clk edge
                pciw_renable <= #`FF_DELAY 1'b1 ;
            @(posedge wb_clock)
            begin
                // sample read data
                if ((pciw_data_out != read_data) || (read_data[3:0] != pciw_control_out) || (read_data[7:4] != pciw_cbe_out))
                begin
                    $display("Empty/not empty transition test failed for PCIW fifo! On read number %d !", index) ;
                    $stop ;
                end
                // prepare data for next read sampling
                read_data <= read_data + 32'd1 ;
                pciw_renable <= #`FF_DELAY 1'b0 ;
                @(posedge wb_clock) ;
            end
        end
    end
    join
end
endtask

task pciw_full_empty;
    integer windex ;
    integer rindex ;
begin
    // task fills FIFO until it is full and monitors status outputs if they are asserted correctly
    windex = 1 ;
    pciw_data_in <= #`FF_DELAY 32'h00000001 ;
    pciw_control_in <= #`FF_DELAY 4'h1 ;
    pciw_cbe_in <= #`FF_DELAY 4'h0 ;
    begin:fill_FIFO
        @(posedge pci_clock)
            pciw_wenable <= #`FF_DELAY 1'b1 ;
        while (windex < `PCIW_DEPTH)
        begin
            @(posedge pci_clock)
            begin
                windex = windex + 1 ;
                pciw_data_in <= #`FF_DELAY windex ;
                pciw_control_in <= #`FF_DELAY windex[3:0] ;
                pciw_cbe_in <= #`FF_DELAY windex[7:4] ;
                
                if (windex == (`PCIW_DEPTH))
                begin
                    if (~pciw_almost_full)
                    begin
                        $display("Almost full status generation test for PCIW failed") ;
                        $stop ;
                    end
                end
            end
        end
        
        pciw_wenable <= #`FF_DELAY 1'b0 ;
        @(posedge pci_clock)
        begin
            if (~pciw_full)
            begin
                $display("Full status generation test for PCIW failed") ;
                $stop ;
            end
        end
    end
    
    // statements read from FIFO, monitor data output and status outputs on proper clock edge
    begin:empty_FIFO
        rindex = 0 ;
        wait (pciw_full) ;
        @(posedge wb_clock)
            pciw_renable <= #`FF_DELAY 1'b1 ;
        while(rindex < (`PCIW_DEPTH - 1))
        begin
            rindex = rindex + 1 ;
            @(posedge wb_clock)
            begin
                if ((pciw_data_out != rindex) || (rindex[3:0] != pciw_control_out) || (rindex[7:4] != pciw_cbe_out))
                begin
                    $display("Full/Empty status generation test failed for PCIW fifo!");
                    $stop ;
                end
                if (rindex == `PCIW_DEPTH - 1)
                begin
                    if (~pciw_almost_empty)
                    begin
                        $display("Almost empty status generation test for PCIW failed") ;
                        $stop ;
                    end
                end
            end
        end

        pciw_renable <= #`FF_DELAY 1'b0 ;
        @(posedge wb_clock)
        begin
            if (~pciw_empty)
            begin
                $display("Empty status generation test for PCIW failed") ;
                $stop ;
            end
        end
    end    
end
endtask


task pcir_empty_nempty_transition ;
    integer index ;
    integer read_data ;
begin
    read_data = 1 ;
    fork
    begin:write_pcir_once
        for(index = 2; index <= 101; index = index + 1)
        begin
            wait(pcir_empty);
            @(posedge wb_clock)
                pcir_wenable <= #`FF_DELAY 1'b1;
                
            // data gets written at next rising clock edge
            @(posedge wb_clock)
            begin
                pcir_wenable <= #`FF_DELAY 1'b0 ;
                pcir_data_in <= #`FF_DELAY index ;
                pcir_control_in <= #`FF_DELAY index[3:0] ;
                pcir_be_in <= #`FF_DELAY index[7:4] ;
                wait(~pcir_empty) ;
            end
        end
    end

    begin:read_pcir
        while(read_data <= 100)
        begin
            wait (~pcir_empty) ;
            @(posedge pci_clock)
                pcir_renable <= #`FF_DELAY 1'b1 ;
            @(posedge pci_clock)
            begin
                if ((pcir_data_out != read_data) || (read_data[3:0] != pcir_control_out) || (read_data[7:4] != pcir_be_out))
                begin
                    $display("Empty/not empty transition test failed for PCIR fifo! On read number %d !", index) ;
                    $stop ;
                end
                read_data <= read_data + 32'd1 ;
                pcir_renable <= #`FF_DELAY 1'b0 ;
                @(posedge pci_clock) ;
            end
        end
    end
    join
end
endtask

task pcir_full_empty;
    integer windex ;
    integer rindex ;
begin
    windex = 1 ;
    pcir_data_in <= #`FF_DELAY 32'h00000001 ;
    pcir_control_in <= #`FF_DELAY 4'h1 ;
    pcir_be_in <= #`FF_DELAY 4'h0 ;
    begin:fill_FIFO
        @(posedge wb_clock)
            pcir_wenable <= #`FF_DELAY 1'b1 ;
        while (windex < `PCIR_DEPTH)
        begin
            @(posedge wb_clock)
            begin
                windex = windex + 1 ;
                pcir_data_in <= #`FF_DELAY windex ;
                pcir_control_in <= #`FF_DELAY windex[3:0] ;
                pcir_be_in <= #`FF_DELAY windex[7:4] ;
                
                if (windex == (`PCIR_DEPTH))
                begin
                    if (~pcir_almost_full)
                    begin
                        $display("Almost full status generation test for PCIR failed") ;
                        $stop ;
                    end
                end
            end
        end
        
        pcir_wenable <= #`FF_DELAY 1'b0 ;
        @(posedge wb_clock)
        begin
            if (~pcir_full)
            begin
                $display("Full status generation test for PCIR failed") ;
                $stop ;
            end
        end
    end

    begin:empty_FIFO
        rindex = 0 ;
        wait (pcir_full) ;
        @(posedge pci_clock)
            pcir_renable <= #`FF_DELAY 1'b1 ;
        while(rindex < (`PCIR_DEPTH - 1))
        begin
            rindex = rindex + 1 ;
            @(posedge pci_clock)
            begin
                if ((pcir_data_out != rindex) || (rindex[3:0] != pcir_control_out) || (rindex[7:4] != pcir_be_out))
                begin
                    $display("Full/Empty status generation test failed for PCIR fifo!");
                    $stop ;
                end
                if (rindex == `PCIR_DEPTH - 1)
                begin
                    if (~pcir_almost_empty)
                    begin
                        $display("Almost empty status generation test for PCIR failed") ;
                        $stop ;
                    end
                end
            end
        end

        pcir_renable <= #`FF_DELAY 1'b0 ;
        @(posedge pci_clock)
        begin
            if (~pcir_empty)
            begin
                $display("Empty status generation test for PCIR failed") ;
                $stop ;
            end
        end
    end    
end
endtask

task simultaneous_operation;
    integer i ;
    integer pciw_data_incoming ;
    reg[(`PCIW_ADDR_LENGTH - 1):0] pci_num_of_writes ;
    reg[(`PCIR_ADDR_LENGTH - 1):0] pci_num_of_reads ;
    reg[(`PCIR_ADDR_LENGTH - 1):0] wb_num_of_writes ;
    reg[3:0] pciw_cbe_outgoing ;
    reg[31:0] pciw_data_outgoing ;
    reg[3:0] pcir_be_outgoing ;
    reg[31:0] pcir_data_outgoing ;
    integer decision ;
    reg read_request ;
begin
    // initialize data
    pciw_data_in <= 32'd0 ;
    pciw_control_in <= 4'h0 ;
    pciw_cbe_in <= 4'he ;
    pciw_data_outgoing <= 32'd1;
    pciw_cbe_outgoing <= 4'hf;
    
    read_request <= 1'b0 ;
    pcir_be_outgoing <= 4'hf ;
    pcir_data_outgoing <= 32'd1 ;
    pcir_data_in <= 32'd0 ;
    pcir_control_in <= 4'h1 ;
    pcir_be_in <= 4'he ;

fork
begin:pci_requests
    forever
    begin
        // random decision on whether current request is a read or a write
        decision = $random(pciw_data_incoming) ;
        if (decision <= 32'd2_147_000_000)
        begin
            // write operation
            if(~pciw_full && ~pciw_almost_full && ~read_request)
            begin
                // PCIW fifo is not full or almost full, so it can accomodate one transaction at least
                // get random transaction length
                pci_num_of_writes = $random ;
                @(posedge pci_clock)
                begin
                    pciw_wenable <= #`FF_DELAY 1'b1 ; //assert write enable and assign control to address
                    pciw_data_in <= #`FF_DELAY pciw_data_in + 1 ;
                    pciw_cbe_in <= #`FF_DELAY pciw_cbe_in + 1 ;
                    pciw_control_in <= #`FF_DELAY `ADDRESS ;
                end
            
                @(posedge pci_clock) ; // address is written on the first clock edge
                @(negedge pci_clock) ; // wait 1/2 clock to see if after address was written fifo is almost empty
                while ((pci_num_of_writes > 1) && (~pciw_almost_full))
                begin
                    // prepare new data
                    pciw_data_in <= #`FF_DELAY pciw_data_in + 1 ;
                    pciw_control_in <= #`FF_DELAY 4'h1 ;
                    pciw_cbe_in <= #`FF_DELAY pciw_cbe_in + 1 ;
                    @(posedge pci_clock)  // write new data
                        pci_num_of_writes <= pci_num_of_writes - 1 ;
                    @(negedge pci_clock) ; // combinatorial monitoring of almost full avoided by waiting half a cycle
                end //while
                // prepare last data
                pciw_data_in <= #`FF_DELAY pciw_data_in + 1 ;
                pciw_control_in <= #`FF_DELAY `LAST ;
                pciw_cbe_in <= #`FF_DELAY pciw_cbe_in + 1 ;
                @(posedge pci_clock) //last data of transaction written
                    pciw_wenable <= #`FF_DELAY 1'b0 ;
                @(negedge pci_clock) ;

            end //if
            else
                // since it was a write request and FIFO cannot accommodate another transaction, wait one cycle
                @(posedge pci_clock) ;
        end
        else
        begin
            // it's a read request
        
            // only accept read request if no other is pending           
            if (~read_request)
            begin
            @(posedge pci_clock)
            // initiate read request
                read_request <= #`FF_DELAY 1'b1 ;

            // wait for FIFO to be filled and transaction be ready 
            wait(~pcir_empty && pcir_transaction_ready) ;
           
            // random number of reads pci is prepared to do
            pci_num_of_reads = $random ;
    
            // first posedge of pci clock when transaction is ready - assert read enable
            @(posedge pci_clock)
                pcir_renable <= #`FF_DELAY 1'b1 ;
            // until whole transaction or random number of reads is performed do a read
            @(negedge pci_clock) ; //wait if maybe first transaction is the last
    
           while((pci_num_of_reads > 1) && (~pcir_almost_empty) && (pcir_control_out != `LAST))
           begin
               @(posedge pci_clock)
               begin
                   if ((pcir_data_out != pcir_data_outgoing) || (pcir_be_out != pcir_be_outgoing))
                   begin
                       $display("Operational test for WBR FIFO failed") ;
                       $stop ;
                   end
                      
                   // prepare new set of outgoing data for comparison  
                   pcir_data_outgoing <= #`FF_DELAY pcir_data_outgoing + 1;
                   pcir_be_outgoing   <= #`FF_DELAY pcir_be_outgoing + 1 ;
                   pci_num_of_reads   <= pci_num_of_reads - 1 ;
                        
                   @(negedge pci_clock) ; // avoiding combinatorial logic for monitoring almost full or control out by waiting 1/2 cycle
               end
           end
           
            // read last data entry
            @(posedge pci_clock)
            begin
                if ((pcir_data_out != pcir_data_outgoing) || (pcir_be_out != pcir_be_outgoing))
                begin
                    $display("Operational test for WBR FIFO failed") ;
                    $stop ;
                end
                pcir_renable <= #`FF_DELAY 1'b0 ;
            end
            
            // after last intended data is read, flush pcir_fifo
            @(posedge pci_clock)
            begin
                if(~pcir_empty)
                begin
                    pcir_flush <= #`FF_DELAY 1'b1 ;
                    @(posedge pci_clock)
                        pcir_flush <= #`FF_DELAY 1'b0 ;
                end //if
            end // posedge
        end
        else
            @(posedge pci_clock) ;
        end //else
    end //forever
end //pci_requests

begin:wb_completions
    // wait until read request is present or at least one write is posted
    forever
    begin
    wait ((~pciw_empty && pciw_transaction_ready) || (read_request && pcir_empty)) ;
    
    // all the writes must finish before a read can be processed
    if (~pciw_empty && pciw_transaction_ready)
    begin
        // first posedge of wb clock when transaction is ready - assert read enable
        @(posedge wb_clock)
            pciw_renable <= #`FF_DELAY 1'b1 ;
            
        // first entry must be address
        @(posedge wb_clock)
        begin
            if ((pciw_data_out != pciw_data_outgoing) || (pciw_cbe_out != pciw_cbe_outgoing) || (pciw_control_out != `ADDRESS))
            begin
                 $display("Operational test for PCIW FIFO failed") ;
                 $stop ;
            end //if
            
            pciw_data_outgoing <= #`FF_DELAY pciw_data_outgoing + 1 ;
            pciw_cbe_outgoing  <= #`FF_DELAY pciw_cbe_outgoing + 1 ;
        end //posedge
           
        // wait for negedge - maybe first data is also last
        @(negedge wb_clock) ;           
               
        while ((pciw_control_out != `LAST) && (~pciw_almost_empty))
        begin
            @(posedge wb_clock)
            begin
                if ((pciw_data_out != pciw_data_outgoing) || (pciw_cbe_out != pciw_cbe_outgoing))
                begin
                    $display("Operational test for PCIW FIFO failed") ;
                    $stop ;
                end
                     
                // prepare new set of outgoing data for comparison  
                pciw_data_outgoing <= #`FF_DELAY pciw_data_outgoing + 1;
                pciw_cbe_outgoing   <= #`FF_DELAY pciw_cbe_outgoing + 1 ;
                        
                @(negedge wb_clock) ; // avoiding combinatorial logic for monitoring almost full or control out by waiting 1/2 cycle
            end
        end
          
        // read last data entry
        @(posedge wb_clock)
        begin
            if ((pciw_data_out != pciw_data_outgoing) || (pciw_cbe_out != pciw_cbe_outgoing) || (pciw_control_out != `LAST))
            begin
                $display("Operational test for PCIW FIFO failed") ;
                $stop ;
            end
            pciw_renable <= #`FF_DELAY 1'b0 ;
            // prepare new set of outgoing data for comparison  
            pciw_data_outgoing <= #`FF_DELAY pciw_data_outgoing + 1;
            pciw_cbe_outgoing   <= #`FF_DELAY pciw_cbe_outgoing + 1 ;
            
        end //posedge
        
        // turnaround cycle
        @(posedge wb_clock) ;

    end // if
    else if (read_request && pcir_empty)
    begin
        // read request is present - fill the FIFO
        wb_num_of_writes = $random ; // reads are of random length
        @(posedge wb_clock)
        begin
            if (wb_num_of_writes < `WBR_DEPTH'h2)
                pcir_control_in <= #`FF_DELAY `LAST ;
            else
                pcir_control_in <= #`FF_DELAY 4'h1 ;

            pcir_wenable <= #`FF_DELAY 1'b1 ;
            pcir_data_in <= #`FF_DELAY pcir_data_in + 1 ;
            pcir_be_in   <= #`FF_DELAY pcir_be_in + 1 ;

            // set pcir outgoing to new value
            pcir_data_outgoing <= pcir_data_in + 1 ;
            pcir_be_outgoing   <= pcir_be_in  + 1;
        end // posedge
        
        @(negedge wb_clock) ;

        if (pcir_control_in != `LAST)
        begin
            while ((wb_num_of_writes > 1) && (~pcir_almost_full))
            begin
                @(posedge wb_clock)
                begin
                    // prepare data for next write
                    pcir_data_in <= #`FF_DELAY pcir_data_in + 1 ;
                    pcir_be_in   <= #`FF_DELAY pcir_be_in + 1 ;
                    wb_num_of_writes <= wb_num_of_writes - 1 ;
                end // posedge
                @(negedge wb_clock) ; // avoid combinatorial logic for almost full monitoring
            end //while
        
            // write last data phase
            pcir_control_in <= #`FF_DELAY `LAST ;
            @(posedge wb_clock)
            begin
                // last data is written
                pcir_wenable <= #`FF_DELAY 1'b0 ;
                read_request <= #`FF_DELAY 1'b0 ;
            end //posedge
        end //if
        else
        begin
            // first is also the last data - wait for posedge and finish
            @(posedge wb_clock)
            begin
                pcir_wenable <= #`FF_DELAY 1'b0 ;
                read_request <= #`FF_DELAY 1'b0 ;
            end //posedge
        end //else
        @(posedge wb_clock) ; // turnaround
    end //else if
    else
        @(posedge wb_clock) ;
    end //forever
end //wb_completions

begin:timing
    #1000000 $stop ;
end //timing
join
end 

endtask
    
PCIW_PCIR_FIFOS #(`PCIW_DEPTH, `PCIW_ADDR_LENGTH, `PCIR_DEPTH, `PCIR_ADDR_LENGTH) pciw_pcir(.wb_clock_in(wb_clock), .pci_clock_in(pci_clock), .reset_in(reset), 
                      .pciw_wenable_in(pciw_wenable), .pciw_addr_data_in(pciw_data_in), 
                      .pciw_cbe_in(pciw_cbe_in), .pciw_control_in(pciw_control_in),                     
                      .pciw_renable_in(pciw_renable), .pciw_addr_data_out(pciw_data_out), 
                      .pciw_cbe_out(pciw_cbe_out), .pciw_control_out(pciw_control_out),                     
                      .pciw_flush_in(1'b0), .pciw_almost_full_out(pciw_almost_full), .pciw_full_out(pciw_full),
                      .pciw_almost_empty_out(pciw_almost_empty), .pciw_empty_out(pciw_empty), .pciw_transaction_ready_out(pciw_transaction_ready), 
                      .pcir_wenable_in(pcir_wenable), .pcir_data_in(pcir_data_in), 
                      .pcir_be_in(pcir_be_in), .pcir_control_in(pcir_control_in),                     
                      .pcir_renable_in(pcir_renable), .pcir_data_out(pcir_data_out), 
                      .pcir_be_out(pcir_be_out), .pcir_control_out(pcir_control_out),                     
                      .pcir_flush_in(pcir_flush), .pcir_almost_full_out(pcir_almost_full), .pcir_full_out(pcir_full),
                      .pcir_almost_empty_out(pcir_almost_empty), .pcir_empty_out(pcir_empty), .pcir_transaction_ready_out(pcir_transaction_ready)) ;

endmodule 

        
    

