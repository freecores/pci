//////////////////////////////////////////////////////////////////////
////                                                              ////
////  File name "wb_master32.v"                                   ////
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
// Revision 1.3  2003/07/29 08:19:47  mihad
// Found and simulated the problem in the synchronization logic.
// Repaired the synchronization logic in the FIFOs.
//
// Revision 1.2  2003/06/12 02:30:39  mihad
// Update!
//
// Revision 1.1  2002/02/01 13:39:43  mihad
// Initial testbench import. Still under development
//
//

`include "pci_testbench_defines.v"
`include "timescale.v"
module WB_MASTER32
(
    CLK_I,
    RST_I,
    TAG_I,
    TAG_O,
    ACK_I,
    ADR_O,
    CYC_O,
    DAT_I,
    DAT_O,
    ERR_I,
    RTY_I,
    SEL_O,
    STB_O,
    WE_O,
    CAB_O
);

    input	             CLK_I;
    input                    RST_I;
    input    `WB_TAG_TYPE    TAG_I;
    output   `WB_TAG_TYPE    TAG_O;
    input                    ACK_I;
    output   `WB_ADDR_TYPE   ADR_O;
    output                   CYC_O;
    input    `WB_DATA_TYPE   DAT_I;
    output   `WB_DATA_TYPE   DAT_O;
    input                    ERR_I;
    input                    RTY_I;
    output   `WB_SEL_TYPE    SEL_O;
    output                   STB_O;
    output                   WE_O;
    output                   CAB_O ;

    // period length
    real Tp ;

    reg    `WB_ADDR_TYPE   ADR_O;
    reg    `WB_SEL_TYPE    SEL_O;
    reg    `WB_TAG_TYPE    TAG_O;
    reg                    CYC_O;
    reg                    WE_O;
    reg    `WB_DATA_TYPE   DAT_O;
    reg                    CAB_O ;
    reg                    STB_O ;

    // variable used for indication on whether cycle was already started
    reg in_use ;

    // because of non-blocking assignments CYC_O is not sufficient indicator for cycle starting - this var is used in its place
    reg cycle_in_progress ;

    // same goes for CAB_O signal
    reg cab ;

    reg we ;

    task start_cycle ;
        input is_cab  ;
        input write   ;
        input fastb2b ;
        output ok ;      // ok indicates to the caller that cycle was started succesfully - if not, caller must take appropriate action
    begin:main

        ok  = 1 ;

        // just check if valid value is provided for CAB_O signal (no x's or z's allowed)
        if ( (is_cab !== 1'b0) && (is_cab !== 1'b1) )
        begin
            $display("*E, invalid CAB value for cycle! Requested CAB_O value = %b, Time %t ", is_cab, $time) ;
            ok = 0 ;
            disable main ;
        end

        if ( (cycle_in_progress === 1) || (CYC_O === 1))
        begin
            // cycle was previously started - allow cycle to continue if CAB and WE values match
            if (fastb2b !== 1'b1)
                $display("*W, cycle already in progress when start_cycle routine was called! Time %t ", $time) ;

            if ((CAB_O !== is_cab) || (WE_O !== write) )
            begin
                ok = 0 ;
                if ( is_cab === 1 )
                    $display("*E, cab cycle start attempted when non-cab cycle was in progress! Time %t", $time) ;
                else
                    $display("*E, non-cab cycle start attempted when cab cycle was in progress! Time %t", $time) ;

                if ( we === 1 )
                    $display("*E, write cycle start attempted when read cycle was in progress! Time %t", $time) ;
                else
                    $display("*E, read cycle start attempted when write cycle was in progress! Time %t", $time) ;

                disable main ;
            end
        end

        CYC_O <= #(Tp - `Tsetup) 1'b1 ;
        CAB_O <= #(Tp - `Tsetup) is_cab ;
        WE_O  <= #(Tp - `Tsetup) write ;

        // this non-blocking assignments are made to internal variables, so read and write tasks can be called immediately after cycle start task
        cycle_in_progress = 1'b1 ;
        cab               = is_cab ;
        we                = write ;
    end
    endtask //start_cycle

    task end_cycle ;
    begin
        if ( CYC_O !== 1'b1 )
            $display("*W, end_cycle routine called when CYC_O value was %b! Time %t ", CYC_O, $time) ;

        CYC_O <= #`Thold 1'b0 ;
        CAB_O <= #`Thold 1'b0 ;
        cycle_in_progress = 1'b0 ;
    end
    endtask //end_cycle

    task modify_cycle ;
    begin
        if ( CYC_O !== 1'b1 )
            $display("*W, modify_cycle routine called when CYC_O value was %b! Time %t ", CYC_O, $time) ;

        we = ~we ;
        WE_O <= #(Tp - `Tsetup) we ;
    end
    endtask //modify_cycle

    task wbm_read ;
        input  `READ_STIM_TYPE input_data ;
        inout `READ_RETURN_TYPE   output_data ;
        reg    `WB_ADDR_TYPE           address ;
        reg    `WB_DATA_TYPE           data ;
        reg    `WB_SEL_TYPE            sel ;
        reg    `WB_TAG_TYPE            tag ;
        integer                        num_of_cyc ;
    begin:main
        output_data`TB_ERROR_BIT = 1'b0 ;

        // check if task was called before previous call to read or write finished
        if ( in_use === 1 )
        begin
            $display("*E, wbm_read routine re-entered or called concurently with write routine! Time %t ", $time) ;
            output_data`TB_ERROR_BIT = 1'b1 ;
            disable main ;
        end

        if ( cycle_in_progress !== 1 )
        begin
            $display("*E, wbm_read routine called without start_cycle routine being called first! Time %t ", $time) ;
            output_data`TB_ERROR_BIT = 1'b1 ;
            disable main ;
        end

        if ( we !== 0 )
        begin
            $display("*E, wbm_read routine called after write cycle was started! Time %t ", $time) ;
            output_data`TB_ERROR_BIT = 1'b1 ;
            disable main ;
        end

        // this branch contains timing controls - claim the use of WISHBONE
        in_use = 1 ;

        num_of_cyc = `WAIT_FOR_RESPONSE ;

        // assign data outputs
        ADR_O      <= #(Tp - `Tsetup) input_data`READ_ADDRESS ;
        SEL_O      <= #(Tp - `Tsetup) input_data`READ_SEL ;
        TAG_O      <= #(Tp - `Tsetup) input_data`READ_TAG_STIM ;

        // assign control output
        STB_O      <= #(Tp - `Tsetup) 1'b1 ;

        output_data`CYC_ACK = 0 ;
        output_data`CYC_RTY = 0 ;
        output_data`CYC_ERR = 0 ;

        @(posedge CLK_I) ;
        output_data`CYC_ACK = ACK_I ;
        output_data`CYC_RTY = RTY_I ;
        output_data`CYC_ERR = ERR_I ;

        while ( (num_of_cyc > 0) && (output_data`CYC_RESPONSE === 0) )
        begin
	    @(posedge CLK_I) ;
	    output_data`CYC_ACK = ACK_I ;
	    output_data`CYC_RTY = RTY_I ;
	    output_data`CYC_ERR = ERR_I ;
	    num_of_cyc = num_of_cyc - 1 ;
	end

	output_data`READ_DATA    = DAT_I ;
	output_data`READ_TAG_RET = TAG_I ;

        if ( output_data`CYC_RESPONSE === 0 )
	begin

            $display("*W, Terminating read cycle because no response was received in %d cycles! Time %t ", `WAIT_FOR_RESPONSE, $time) ;
        end

        if ( output_data`CYC_ACK === 1 && output_data`CYC_RTY === 0 && output_data`CYC_ERR === 0 )
            output_data`CYC_ACTUAL_TRANSFER = output_data`CYC_ACTUAL_TRANSFER + 1 ;

        STB_O <= #`Thold 1'b0 ;
        ADR_O <= #`Thold {`WB_ADDR_WIDTH{1'bx}} ;
        SEL_O <= #`Thold {`WB_SEL_WIDTH{1'bx}} ;
        TAG_O <= #`Thold {`WB_TAG_WIDTH{1'bx}} ;

        in_use = 0 ;
    end
    endtask // wbm_read

    task wbm_write ;
        input  `WRITE_STIM_TYPE input_data ;
        inout  `WRITE_RETURN_TYPE   output_data ;
        reg    `WB_ADDR_TYPE        address ;
        reg    `WB_DATA_TYPE        data ;
        reg    `WB_SEL_TYPE         sel ;
        reg    `WB_TAG_TYPE         tag ;
        integer                     num_of_cyc ;
    begin:main
        output_data`TB_ERROR_BIT = 1'b0 ;

        // check if task was called before previous call to read or write finished
        if ( in_use === 1 )
        begin
            $display("*E, wbm_write routine re-entered or called concurently with read routine! Time %t ", $time) ;
            output_data`TB_ERROR_BIT = 1'b1 ;
            disable main ;
        end

        if ( cycle_in_progress !== 1 )
        begin
            $display("*E, wbm_write routine called without start_cycle routine being called first! Time %t ", $time) ;
            output_data`TB_ERROR_BIT = 1'b1 ;
            disable main ;
        end

        if ( we !== 1 )
        begin
            $display("*E, wbm_write routine after read cycle was started! Time %t ", $time) ;
            output_data`TB_ERROR_BIT = 1'b1 ;
            disable main ;
        end

        // this branch contains timing controls - claim the use of WISHBONE
        in_use = 1 ;

        num_of_cyc = `WAIT_FOR_RESPONSE ;

        ADR_O      <= #(Tp - `Tsetup) input_data`WRITE_ADDRESS ;

        begin:dat_o_assign_blk
            reg [`WB_SEL_WIDTH - 1:0] cur_sel ;
            reg [`WB_DATA_WIDTH - 1:0] cur_dat ;
            reg [`WB_DATA_WIDTH - 1:0] rnd_dat ;
            integer cur_bit ;

            cur_dat = input_data`WRITE_DATA ;
            cur_sel = input_data`WRITE_SEL  ;
            rnd_dat = $random ;

            for(cur_bit = 0 ; cur_bit < `WB_DATA_WIDTH ; cur_bit = cur_bit + 1)
            begin
                if (cur_sel[cur_bit/8] === 1'b1)
                    DAT_O[cur_bit] <= #(Tp - `Tsetup) cur_dat[cur_bit] ;
                else
                    DAT_O[cur_bit] <= #(Tp - `Tsetup) rnd_dat[cur_bit] ;
            end
        end

        SEL_O      <= #(Tp - `Tsetup) input_data`WRITE_SEL ;
        TAG_O      <= #(Tp - `Tsetup) input_data`WRITE_TAG_STIM ;

        STB_O      <= #(Tp - `Tsetup) 1'b1 ;

        output_data`CYC_ACK = 0 ;
        output_data`CYC_RTY = 0 ;
        output_data`CYC_ERR = 0 ;

        @(posedge CLK_I) ;
        output_data`CYC_ACK = ACK_I ;
        output_data`CYC_RTY = RTY_I ;
        output_data`CYC_ERR = ERR_I ;

        while ( (num_of_cyc > 0) && (output_data`CYC_RESPONSE === 0) )
        begin
            @(posedge CLK_I) ;
            output_data`CYC_ACK = ACK_I ;
            output_data`CYC_RTY = RTY_I ;
            output_data`CYC_ERR = ERR_I ;
            num_of_cyc = num_of_cyc - 1 ;
        end

        output_data`WRITE_TAG_RET = TAG_I ;
        if ( output_data`CYC_RESPONSE === 0 )
        begin
            $display("*W, Terminating write cycle because no response was received in %d cycles! Time %t ", `WAIT_FOR_RESPONSE, $time) ;
        end

        if ( output_data`CYC_ACK === 1 && output_data`CYC_RTY === 0 && output_data`CYC_ERR === 0 )
            output_data`CYC_ACTUAL_TRANSFER = output_data`CYC_ACTUAL_TRANSFER + 1 ;

        ADR_O <= #`Thold {`WB_ADDR_WIDTH{1'bx}} ;
        DAT_O <= #`Thold {`WB_DATA_WIDTH{1'bx}} ;
        SEL_O <= #`Thold {`WB_SEL_WIDTH{1'bx}} ;
        TAG_O <= #`Thold {`WB_TAG_WIDTH{1'bx}} ;

        STB_O <= #`Thold 1'b0 ;

        in_use = 0 ;
    end
    endtask //wbm_write

    initial
    begin
        Tp = `WB_PERIOD ;
        in_use = 0 ;
        cycle_in_progress = 0 ;
        cab = 0 ;
        ADR_O <= {`WB_ADDR_WIDTH{1'bx}} ;
        DAT_O <= {`WB_DATA_WIDTH{1'bx}} ;
        SEL_O <= {`WB_SEL_WIDTH{1'bx}} ;
        TAG_O <= {`WB_TAG_WIDTH{1'bx}} ;
        CYC_O <= 1'b0 ;
        STB_O <= 1'b0 ;
        CAB_O <= 1'b0 ;
        WE_O  <= 1'b0 ;
        if ( `Tsetup > Tp || `Thold >= Tp )
        begin
            $display("Either Tsetup or Thold values for WISHBONE BFMs are too large!") ;
            $stop ;
        end
    end

endmodule
