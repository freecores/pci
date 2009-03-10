//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "wb_tb.v"                                            ////
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
// Revision 1.3  2001/06/12 11:15:10  mihad
// Changed module parameters
//
//

`include "constants.v"
`include "wbw_wbr_fifos.v"

// Common definitions for simulation purposes
`define Tpci 75 // Tp/2 = 7.5ns => Tp = 15ns => Fpci = 66MHz
`define Twb 25  // Tp/2 = 2.5ns =>   Tp = 5ns  => Fwb  = 200MHz

module WB_TB() ;

reg [31:0] wbw_data_in ;
wire [31:0] wbw_data_out;
reg [3:0]  wbw_cbe_in ;
wire [3:0] wbw_cbe_out ;
reg [3:0] wbw_control_in ;
wire [3:0] wbw_control_out ;

wire [31:0] wbr_data_out;
reg [31:0] wbr_data_in;
wire [3:0] wbr_be_out ;
reg [3:0] wbr_be_in ;
reg [3:0] wbr_control_in ;
wire [3:0] wbr_control_out ;

wire wbw_full;
wire wbw_empty ;
wire wbw_almost_full ;
wire wbw_almost_empty ;
reg wbw_wenable ;
reg wbw_renable ;
wire wbw_transaction_ready ;

wire wbr_full;
wire wbr_empty ;
wire wbr_almost_full ;
wire wbr_almost_empty ;
reg wbr_wenable ;
reg wbr_renable ;
wire wbr_transaction_ready ;

reg reset ;
reg pci_clock ;
reg wb_clock ;

reg wbr_flush ;
reg wbw_flush ;

`ifdef FPGA
    assign glbl.GSR = reset ;
`endif

// initial values
initial
begin
    reset <= 1'b1 ;

    pci_clock <= 1'b1 ;
    wb_clock <= 1'b0 ;
    wbw_data_in <= 32'h00000001 ;
    wbw_cbe_in  <= 4'h0 ;
    wbr_be_in  <= 4'h0 ;
    wbw_control_in <= 4'h1 ;
    wbw_wenable <= 1'b0;
    wbw_renable <= 1'b0 ;
    wbr_control_in <= 4'h1 ;
    wbr_wenable <= 1'b0;
    wbr_renable <= 1'b0 ;
    wbr_data_in <= 32'h00000001 ;
    wbr_flush <= 1'b0 ;

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
    wbw_empty_nempty_transition ;
    $display("Empty/not empty transition test for WBW completed") ;
    wbw_full_empty ;
    $display("Full/empty status generation test for WBW completed") ;
    
    wbr_empty_nempty_transition ;
    $display("Empty/not empty transition test for WBR completed") ;
    wbr_full_empty ;
    $display("Full/empty status generation test for WBR completed") ;

    simultaneous_operation ;
    $stop ;
end
endtask

task wbw_empty_nempty_transition ;
    integer index ;
    integer read_data ;
begin
    read_data = 1 ;
    fork
    begin:write_wbw_once
        // repeat the test several times, so clock to clock transition time changes
        for(index = 2; index <= 101; index = index + 1)
        begin
            // wait for WBW to become empty
            wait(wbw_empty);
            // sample empty on posedge of clock
            @(posedge wb_clock)
                wbw_wenable <= #`FF_DELAY 1'b1;
                
            // data gets written at clock edge following the one empty was asserted
            @(posedge wb_clock)
            begin
                // prepare data for next write
                wbw_wenable <= #`FF_DELAY 1'b0 ;
                wbw_data_in <= #`FF_DELAY index ;
                wbw_control_in <= #`FF_DELAY index[3:0] ;
                wbw_cbe_in <= #`FF_DELAY index[7:4] ;
                // wait for WBW to be empty again
                wait(~wbw_empty) ;
            end
        end
    end

    begin:read_wbw
        // repeat test several times
        while(read_data <= 100)
        begin
            wait (~wbw_empty) ;
            @(posedge pci_clock)
                // at first clock edge empty is deasserted assert read enable, so read will be performed on the next clk edge
                wbw_renable <= #`FF_DELAY 1'b1 ;
            @(posedge pci_clock)
            begin
                // sample read data
                if ((wbw_data_out != read_data) || (read_data[3:0] != wbw_control_out) || (read_data[7:4] != wbw_cbe_out))
                begin
                    $display("Empty/not empty transition test failed for WBW fifo! On read number %d !", index) ;
                    $stop ;
                end
                // prepare data for next read sampling
                read_data <= read_data + 32'd1 ;
                wbw_renable <= #`FF_DELAY 1'b0 ;
                @(posedge pci_clock) ;
            end
        end
    end
    join
end
endtask

task wbw_full_empty;
    integer windex ;
    integer rindex ;
begin
    // task fills FIFO until it is full and monitors status outputs if they are asserted correctly
    windex = 1 ;
    wbw_data_in <= #`FF_DELAY 32'h00000001 ;
    wbw_control_in <= #`FF_DELAY 4'h1 ;
    wbw_cbe_in <= #`FF_DELAY 4'h0 ;
    begin:fill_FIFO
        @(posedge wb_clock)
            wbw_wenable <= #`FF_DELAY 1'b1 ;
        while (windex < `WBW_DEPTH)
        begin
            @(posedge wb_clock)
            begin
                windex = windex + 1 ;
                wbw_data_in <= #`FF_DELAY windex ;
                wbw_control_in <= #`FF_DELAY windex[3:0] ;
                wbw_cbe_in <= #`FF_DELAY windex[7:4] ;
                
                if (windex == (`WBW_DEPTH))
                begin
                    if (~wbw_almost_full)
                    begin
                        $display("Almost full status generation test for WBW failed") ;
                        $stop ;
                    end
                end
            end
        end
        
        wbw_wenable <= #`FF_DELAY 1'b0 ;
        @(posedge wb_clock)
        begin
            if (~wbw_full)
            begin
                $display("Full status generation test for WBW failed") ;
                $stop ;
            end
        end
    end
    
    // statements read from FIFO, monitor data output and status outputs on proper clock edge
    begin:empty_FIFO
        rindex = 0 ;
        wait (wbw_full) ;
        @(posedge pci_clock)
            wbw_renable <= #`FF_DELAY 1'b1 ;
        while(rindex < (`WBW_DEPTH - 1))
        begin
            rindex = rindex + 1 ;
            @(posedge pci_clock)
            begin
                if ((wbw_data_out != rindex) || (rindex[3:0] != wbw_control_out) || (rindex[7:4] != wbw_cbe_out))
                begin
                    $display("Full/Empty status generation test failed for WBW fifo!");
                    $stop ;
                end
                if (rindex == `WBW_DEPTH - 1)
                begin
                    if (~wbw_almost_empty)
                    begin
                        $display("Almost empty status generation test for WBW failed") ;
                        $stop ;
                    end
                end
            end
        end

        wbw_renable <= #`FF_DELAY 1'b0 ;
        @(posedge pci_clock)
        begin
            if (~wbw_empty)
            begin
                $display("Empty status generation test for WBW failed") ;
                $stop ;
            end
        end
    end    
end
endtask


task wbr_empty_nempty_transition ;
    integer index ;
    integer read_data ;
begin
    read_data = 1 ;
    fork
    begin:write_wbr_once
        for(index = 2; index <= 101; index = index + 1)
        begin
            wait(wbr_empty);
            @(posedge pci_clock)
                wbr_wenable <= #`FF_DELAY 1'b1;
                
            // data gets written at next rising clock edge
            @(posedge pci_clock)
            begin
                wbr_wenable <= #`FF_DELAY 1'b0 ;
                wbr_data_in <= #`FF_DELAY index ;
                wbr_control_in <= #`FF_DELAY index[3:0] ;
                wbr_be_in <= #`FF_DELAY index[7:4] ;
                wait(~wbr_empty) ;
            end
        end
    end

    begin:read_wbr
        while(read_data <= 100)
        begin
            wait (~wbr_empty) ;
            @(posedge wb_clock)
                wbr_renable <= #`FF_DELAY 1'b1 ;
            @(posedge wb_clock)
            begin
                if ((wbr_data_out != read_data) || (read_data[3:0] != wbr_control_out) || (read_data[7:4] != wbr_be_out))
                begin
                    $display("Empty/not empty transition test failed for WBR fifo! On read number %d !", index) ;
                    $stop ;
                end
                read_data <= read_data + 32'd1 ;
                wbr_renable <= #`FF_DELAY 1'b0 ;
                @(posedge wb_clock) ;
            end
        end
    end
    join
end
endtask

task wbr_full_empty;
    integer windex ;
    integer rindex ;
begin
    windex = 1 ;
    wbr_data_in <= #`FF_DELAY 32'h00000001 ;
    wbr_control_in <= #`FF_DELAY 4'h1 ;
    wbr_be_in <= #`FF_DELAY 4'h0 ;
    begin:fill_FIFO
        @(posedge pci_clock)
            wbr_wenable <= #`FF_DELAY 1'b1 ;
        while (windex < `WBR_DEPTH)
        begin
            @(posedge pci_clock)
            begin
                windex = windex + 1 ;
                wbr_data_in <= #`FF_DELAY windex ;
                wbr_control_in <= #`FF_DELAY windex[3:0] ;
                wbr_be_in <= #`FF_DELAY windex[7:4] ;
                
                if (windex == (`WBR_DEPTH))
                begin
                    if (~wbr_almost_full)
                    begin
                        $display("Almost full status generation test for WBR failed") ;
                        $stop ;
                    end
                end
            end
        end
        
        wbr_wenable <= #`FF_DELAY 1'b0 ;
        @(posedge pci_clock)
        begin
            if (~wbr_full)
            begin
                $display("Full status generation test for WBR failed") ;
                $stop ;
            end
        end
    end

    begin:empty_FIFO
        rindex = 0 ;
        wait (wbr_full) ;
        @(posedge wb_clock)
            wbr_renable <= #`FF_DELAY 1'b1 ;
        while(rindex < (`WBR_DEPTH - 1))
        begin
            rindex = rindex + 1 ;
            @(posedge wb_clock)
            begin
                if ((wbr_data_out != rindex) || (rindex[3:0] != wbr_control_out) || (rindex[7:4] != wbr_be_out))
                begin
                    $display("Full/Empty status generation test failed for WBR fifo!");
                    $stop ;
                end
                if (rindex == `WBR_DEPTH - 1)
                begin
                    if (~wbr_almost_empty)
                    begin
                        $display("Almost empty status generation test for WBR failed") ;
                        $stop ;
                    end
                end
            end
        end

        wbr_renable <= #`FF_DELAY 1'b0 ;
        @(posedge wb_clock)
        begin
            if (~wbr_empty)
            begin
                $display("Empty status generation test for WBR failed") ;
                $stop ;
            end
        end
    end    
end
endtask

task simultaneous_operation;
    integer i ;
    integer wbw_data_incoming ;
    reg[(`WBW_ADDR_LENGTH - 1):0] wb_num_of_writes ;
    reg[(`WBR_ADDR_LENGTH - 1):0] wb_num_of_reads ;
    reg[(`WBR_ADDR_LENGTH - 1):0] pci_num_of_writes ;
    reg[3:0] wbw_cbe_outgoing ;
    reg[31:0] wbw_data_outgoing ;
    reg[3:0] wbr_be_outgoing ;
    reg[31:0] wbr_data_outgoing ;
    integer decision ;
    reg read_request ;
begin
    // initialize data
    wbw_data_in <= 32'd0 ;
    wbw_control_in <= 4'h0 ;
    wbw_cbe_in <= 4'he ;
    wbw_data_outgoing <= 32'd1;
    wbw_cbe_outgoing <= 4'hf;
    
    read_request <= 1'b0 ;
    wbr_be_outgoing <= 4'hf ;
    wbr_data_outgoing <= 32'd1 ;
    wbr_data_in <= 32'd0 ;
    wbr_control_in <= 4'h1 ;
    wbr_be_in <= 4'he ;

fork
begin:wb_requests
    forever
    begin
        // random decision on whether current request is a read or a write
        decision = $random(wbw_data_incoming) ;
        if (decision <= 32'd2_147_000_000)
        begin
            // write operation
            if(~wbw_full && ~wbw_almost_full && ~read_request)
            begin
                // WBW fifo is not full or almost full, so it can accomodate one transaction at least
                // get random transaction length
                wb_num_of_writes = $random ;
                @(posedge wb_clock)
                begin
                    wbw_wenable <= #`FF_DELAY 1'b1 ; //assert write enable and assign control to address
                    wbw_data_in <= #`FF_DELAY wbw_data_in + 1 ;
                    wbw_cbe_in <= #`FF_DELAY wbw_cbe_in + 1 ;
                    wbw_control_in <= #`FF_DELAY `ADDRESS ;
                end
            
                @(posedge wb_clock) ; // address is written on the first clock edge
                @(negedge wb_clock) ; // wait 1/2 clock to see if after address was written fifo is almost empty
                while ((wb_num_of_writes > 1) && (~wbw_almost_full))
                begin
                    // prepare new data
                    wbw_data_in <= #`FF_DELAY wbw_data_in + 1 ;
                    wbw_control_in <= #`FF_DELAY 4'h1 ;
                    wbw_cbe_in <= #`FF_DELAY wbw_cbe_in + 1 ;
                    @(posedge wb_clock)  // write new data
                        wb_num_of_writes <= wb_num_of_writes - 1 ;
                    @(negedge wb_clock) ; // combinatorial monitoring of almost full avoided by waiting half a cycle
                end //while
                // prepare last data
                wbw_data_in <= #`FF_DELAY wbw_data_in + 1 ;
                wbw_control_in <= #`FF_DELAY `LAST ;
                wbw_cbe_in <= #`FF_DELAY wbw_cbe_in + 1 ;
                @(posedge wb_clock) //last data of transaction written
                    wbw_wenable <= #`FF_DELAY 1'b0 ;
                @(negedge wb_clock) ;

            end //if
            else
                // since it was a write request and FIFO cannot accommodate another transaction, wait one cycle
                @(posedge wb_clock) ;
        end
        else
        begin
            // it's a read request
        
            // only accept read request if no other is pending           
            if (~read_request)
            begin
            @(posedge wb_clock)
            // initiate read request
                read_request <= #`FF_DELAY 1'b1 ;

            // wait for FIFO to be filled and transaction be ready 
            wait(~wbr_empty && wbr_transaction_ready) ;
           
            // random number of reads wb is prepared to do
            wb_num_of_reads = $random ;
    
            // first posedge of wb clock when transaction is ready - assert read enable
            @(posedge wb_clock)
                wbr_renable <= #`FF_DELAY 1'b1 ;
            // until whole transaction or random number of reads is performed do a read
            @(negedge wb_clock) ; //wait if maybe first transaction is the last
    
           while((wb_num_of_reads > 1) && (~wbr_almost_empty) && (wbr_control_out != `LAST))
           begin
               @(posedge wb_clock)
               begin
                   if ((wbr_data_out != wbr_data_outgoing) || (wbr_be_out != wbr_be_outgoing))
                   begin
                       $display("Operational test for WBR FIFO failed") ;
                       $stop ;
                   end
                      
                   // prepare new set of outgoing data for comparison  
                   wbr_data_outgoing <= #`FF_DELAY wbr_data_outgoing + 1;
                   wbr_be_outgoing   <= #`FF_DELAY wbr_be_outgoing + 1 ;
                   wb_num_of_reads   <= wb_num_of_reads - 1 ;
                        
                   @(negedge wb_clock) ; // avoiding combinatorial logic for monitoring almost full or control out by waiting 1/2 cycle
               end
           end
           
            // read last data entry
            @(posedge wb_clock)
            begin
                if ((wbr_data_out != wbr_data_outgoing) || (wbr_be_out != wbr_be_outgoing))
                begin
                    $display("Operational test for WBR FIFO failed") ;
                    $stop ;
                end
                wbr_renable <= #`FF_DELAY 1'b0 ;
            end
            
            // after last intended data is read, flush wbr_fifo
            @(posedge wb_clock)
            begin
                if(~wbr_empty)
                begin
                    wbr_flush <= #`FF_DELAY 1'b1 ;
                    @(posedge wb_clock)
                        wbr_flush <= #`FF_DELAY 1'b0 ;
                end //if
            end // posedge
        end
        else
            @(posedge wb_clock) ;
        end //else
    end //forever
end //wb_requests

begin:pci_completions
    // wait until read request is present or at least one write is posted
    forever
    begin
    wait ((~wbw_empty && wbw_transaction_ready) || (read_request && wbr_empty)) ;
    
    // all the writes must finish before a read can be processed
    if (~wbw_empty && wbw_transaction_ready)
    begin
        // first posedge of pci clock when transaction is ready - assert read enable
        @(posedge pci_clock)
            wbw_renable <= #`FF_DELAY 1'b1 ;
            
        // first entry must be address
        @(posedge pci_clock)
        begin
            if ((wbw_data_out != wbw_data_outgoing) || (wbw_cbe_out != wbw_cbe_outgoing) || (wbw_control_out != `ADDRESS))
            begin
                 $display("Operational test for WBW FIFO failed") ;
                 $stop ;
            end //if
            
            wbw_data_outgoing <= #`FF_DELAY wbw_data_outgoing + 1 ;
            wbw_cbe_outgoing  <= #`FF_DELAY wbw_cbe_outgoing + 1 ;
        end //posedge
           
        // wait for negedge - maybe first data is also last
        @(negedge pci_clock) ;           
               
        while ((wbw_control_out != `LAST) && (~wbw_almost_empty))
        begin
            @(posedge pci_clock)
            begin
                if ((wbw_data_out != wbw_data_outgoing) || (wbw_cbe_out != wbw_cbe_outgoing))
                begin
                    $display("Operational test for WBW FIFO failed") ;
                    $stop ;
                end
                     
                // prepare new set of outgoing data for comparison  
                wbw_data_outgoing <= #`FF_DELAY wbw_data_outgoing + 1;
                wbw_cbe_outgoing   <= #`FF_DELAY wbw_cbe_outgoing + 1 ;
                        
                @(negedge pci_clock) ; // avoiding combinatorial logic for monitoring almost full or control out by waiting 1/2 cycle
            end
        end
          
        // read last data entry
        @(posedge pci_clock)
        begin
            if ((wbw_data_out != wbw_data_outgoing) || (wbw_cbe_out != wbw_cbe_outgoing) || (wbw_control_out != `LAST))
            begin
                $display("Operational test for WBW FIFO failed") ;
                $stop ;
            end
            wbw_renable <= #`FF_DELAY 1'b0 ;
            // prepare new set of outgoing data for comparison  
            wbw_data_outgoing <= #`FF_DELAY wbw_data_outgoing + 1;
            wbw_cbe_outgoing   <= #`FF_DELAY wbw_cbe_outgoing + 1 ;
            
        end //posedge
        
        // turnaround cycle
        @(posedge pci_clock) ;

    end // if
    else if (read_request && wbr_empty)
    begin
        // read request is present - fill the FIFO
        pci_num_of_writes = $random ; // reads are of random length
        @(posedge pci_clock)
        begin
            if (pci_num_of_writes < `WBR_DEPTH'h2)
                wbr_control_in <= #`FF_DELAY `LAST ;
            else
                wbr_control_in <= #`FF_DELAY 4'h1 ;

            wbr_wenable <= #`FF_DELAY 1'b1 ;
            wbr_data_in <= #`FF_DELAY wbr_data_in + 1 ;
            wbr_be_in   <= #`FF_DELAY wbr_be_in + 1 ;

            // set wbr outgoing to new value
            wbr_data_outgoing <= wbr_data_in + 1 ;
            wbr_be_outgoing   <= wbr_be_in  + 1;
        end // posedge
        
        @(negedge pci_clock) ;

        if (wbr_control_in != `LAST)
        begin
            while ((pci_num_of_writes > 1) && (~wbr_almost_full))
            begin
                @(posedge pci_clock)
                begin
                    // prepare data for next write
                    wbr_data_in <= #`FF_DELAY wbr_data_in + 1 ;
                    wbr_be_in   <= #`FF_DELAY wbr_be_in + 1 ;
                    pci_num_of_writes <= pci_num_of_writes - 1 ;
                end // posedge
                @(negedge pci_clock) ; // avoid combinatorial logic for almost full monitoring
            end //while
        
            // write last data phase
            wbr_control_in <= #`FF_DELAY `LAST ;
            @(posedge pci_clock)
            begin
                // last data is written
                wbr_wenable <= #`FF_DELAY 1'b0 ;
                read_request <= #`FF_DELAY 1'b0 ;
            end //posedge
        end //if
        else
        begin
            // first is also the last data - wait for posedge and finish
            @(posedge pci_clock)
            begin
                wbr_wenable <= #`FF_DELAY 1'b0 ;
                read_request <= #`FF_DELAY 1'b0 ;
            end //posedge
        end //else
        @(posedge pci_clock) ; // turnaround
    end //else if
    else
        @(posedge pci_clock) ;
    end //forever
end //pci_completions

begin:timing
    #1000000 $stop ;
end //timing
join
end 

endtask
    
WBW_WBR_FIFOS #(`WBW_DEPTH, `WBW_ADDR_LENGTH, `WBR_DEPTH, `WBR_ADDR_LENGTH) wbw_wbr
                     (.wb_clock_in(wb_clock), .pci_clock_in(pci_clock), .reset_in(reset), 
                      .wbw_wenable_in(wbw_wenable), .wbw_addr_data_in(wbw_data_in), 
                      .wbw_cbe_in(wbw_cbe_in), .wbw_control_in(wbw_control_in),                     
                      .wbw_renable_in(wbw_renable), .wbw_addr_data_out(wbw_data_out), 
                      .wbw_cbe_out(wbw_cbe_out), .wbw_control_out(wbw_control_out),                     
                      .wbw_flush_in(1'b0), .wbw_almost_full_out(wbw_almost_full), .wbw_full_out(wbw_full),
                      .wbw_almost_empty_out(wbw_almost_empty), .wbw_empty_out(wbw_empty), .wbw_transaction_ready_out(wbw_transaction_ready), 
                      .wbr_wenable_in(wbr_wenable), .wbr_data_in(wbr_data_in), 
                      .wbr_be_in(wbr_be_in), .wbr_control_in(wbr_control_in),                     
                      .wbr_renable_in(wbr_renable), .wbr_data_out(wbr_data_out), 
                      .wbr_be_out(wbr_be_out), .wbr_control_out(wbr_control_out),                     
                      .wbr_flush_in(wbr_flush), .wbr_almost_full_out(wbr_almost_full), .wbr_full_out(wbr_full),
                      .wbr_almost_empty_out(wbr_almost_empty), .wbr_empty_out(wbr_empty), .wbr_transaction_ready_out(wbr_transaction_ready)) ;

endmodule 

        
    

