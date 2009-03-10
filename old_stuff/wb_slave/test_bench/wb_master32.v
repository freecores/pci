//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "wb_master32.v"                                   ////
////                                                              ////
////  This file is part of the "PCI bridge" project               ////
////  http://www.opencores.org/cores/pci/                         ////
////                                                              ////
////  Author(s):                                                  ////
////      - Miha Dolenc (mihad@opencores.org)                     ////
////      - Winefred Washington                                   ////
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
// Revision 1.2  2001/07/19 09:50:08  mihad
// Only delayed write storage is provided after merging delayed requests.
// All delayed reads now pass through FIFO.
//
//

`define Tp 1

module WB_MASTER32(CLK_I, RST_I, TAG_I, TAG_O,
		   ACK_I, ADR_O, CYC_O, DAT_I, DAT_O, ERR_I, RTY_I, SEL_O, STB_O, WE_O, CAB_O);

    input	    	CLK_I;
    input		    RST_I;
    input    [3:0] 	TAG_I;
    output   [3:0]  TAG_O;
    input		    ACK_I;
    output   [31:0] ADR_O;
    output		    CYC_O;
    input    [31:0] DAT_I;
    output   [31:0] DAT_O;
    input	    	ERR_I;
    input		    RTY_I;
    output   [3:0] 	SEL_O;
    output	    	STB_O;
    output		    WE_O;
    output          CAB_O ;
    
    reg [31:0]  ADR_O;
    reg [3:0] 	SEL_O;
    reg 		CYC_O;
    reg 		WE_O;
    reg [31:0] 	DAT_O;
    reg         CAB_O ;
    reg         STB_O ;
 
    // timeout counter and timeout flag for canceling deadlocked cycles 
    reg [7:0]   tocount ;
    reg         toflag ;

    // assign outputs to unknown state while in reset
    always@(RST_I)
    begin
        if (RST_I)
        begin
   	        ADR_O <= 32'hxxxx_xxxx ;
    	    SEL_O <= 4'hx ;
            CYC_O <= 1'b0 ;
   		    WE_O  <= 1'bx ;
   		    DAT_O <= 32'hxxxx_xxxx ;
            CAB_O <= 1'b0 ;
            STB_O <= 1'b0 ;
        end
    end //reset
   
    task blkwr ;
        input [31:0]    address ;   // address input
        input [31:0]    data ;      // data input 
        input [3:0]     select ;    // selects input
        input           burst ;     // burst (cab) input flag
        input           start ;     // start flag
        input           stop ;      // stop flag - terminates cycle
        output [2:0]    result ;    // result output: result[2] = acknowledge, result[1] = error, result[0] = retry
    begin
        // if this is the first cycle, synchronize operation to clock
        if (start)
            @(posedge CLK_I) ;
        
        // assign outputs
        ADR_O <= #`Tp address ;
    	SEL_O <= #`Tp select ;
        CYC_O <= #`Tp 1'b1 ;
   		WE_O  <= #`Tp 1'b1 ;
   		DAT_O <= #`Tp data ;
        CAB_O <= #`Tp burst ;
        
        // if data is not valid insert a wait cycle - STB = 0
        if (data  === 32'hxxxxxxxx)
            STB_O <= #`Tp 1'b0 ;
        else
            STB_O <= #`Tp 1'b1 ;

        // wait for current cycle to end
        @(posedge CLK_I) ;

        if (STB_O)
            while (~(ACK_I || ERR_I || RTY_I || toflag))
                @(posedge CLK_I) ;
        
        result = {ACK_I, ERR_I, RTY_I} ;

        STB_O <= #`Tp 1'b0 ;
    
        // terminate the cycle if stop flag is set or error, retry or timeout is signalled
        if(stop || result[1:0])
        begin   
            CYC_O <= #`Tp 1'b0 ;
            CAB_O <= #`Tp 1'b0 ;
        end
        
        #`Tp ;
    end
    endtask // blkwr
    
    task blkrd ;
        input   [31:0]  address ;       // read address
        input   [3:0]   select ;        // byte selects
        input           burst ;         // burst flag
        input           start ;         // start flag
        input           stop ;          // stop flag
        output  [34:0]  status_w_data ; // status with data - [34:32] = status[34]=acknowledge, status[33]=error, status[32]=retry
        reg     [2:0]   result ;        
    begin
        // if this is the first cycle, synchronize operation to clock
        if (start)
            @(posedge CLK_I) ;
        
        // assign outputs
        ADR_O <= #`Tp address ;
    	SEL_O <= #`Tp select ;
        CYC_O <= #`Tp 1'b1 ;
   		WE_O  <= #`Tp 1'b0 ;
        CAB_O <= #`Tp burst ;
        STB_O <= #`Tp 1'b1 ;
        
        // invalid selects insert wait cycles
        if (select === 4'bxxxx)
            STB_O <= #`Tp 1'b0 ;
        else
            STB_O <= #`Tp 1'b1 ;

        // wait for current cycle to end
        @(posedge CLK_I) ;
        if (STB_O) 
            while (~(ACK_I || ERR_I || RTY_I || toflag))
                @(posedge CLK_I) ;
        
        result = {ACK_I, ERR_I, RTY_I} ;
        status_w_data = {result, DAT_I} ;
        
        STB_O <= #`Tp 1'b0 ;
    
        if(stop || result[1:0] || toflag)
        begin   
            CYC_O <= #`Tp 1'b0 ;
            CAB_O <= #`Tp 1'b0 ;
        end
        
        #`Tp ;
    end
    endtask // blkrd

    // timeout flag generation
    always@(posedge CLK_I or posedge RST_I)
    begin
        if (RST_I)
        begin
            toflag  <= 1'b0 ;
            tocount <= 8'h00 ;
        end
        else
        begin
            // whenever cycle is in progress and data is qualified count cycles with no slave response
            if (CYC_O && STB_O && ~(ACK_I || ERR_I || RTY_I))
            begin
                tocount <= tocount + 1 ;
            end
            else
            // reset counter and flag when cycle or strobe are inactive or slave responds whith whatever response signal
            begin
                tocount <= 8'h00 ;
                toflag  <= 1'b0 ;
            end

            if ( tocount == 8'hFF)
                toflag <= 1'b1 ;
        end
        
        
    end
endmodule





