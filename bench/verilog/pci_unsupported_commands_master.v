`include "bus_commands.v"
module pci_unsupported_commands_master
(
    CLK,
    AD,
    CBE,
    RST,
    REQ,
    GNT,
    FRAME,
    IRDY,
    DEVSEL,
    TRDY,
    STOP,
    PAR
);

input CLK ;
output [31:0] AD ;
output [3:0]  CBE ;
input  RST ;
output REQ ;
input  GNT ;
output FRAME ;
output IRDY ;
input  DEVSEL ;
input  TRDY ;
input  STOP ;
output PAR ;

reg [31:0]  AD ;
reg [3:0]   CBE ;
reg         REQ ;
reg         FRAME ;
reg         IRDY ;
reg         PAR ;

initial
begin
    REQ   = 1'b1 ;
    AD    = 32'hzzzz_zzzz ;
    CBE   = 4'bzzzz ;
    FRAME = 1'bz ;
    IRDY  = 1'bz ;
    PAR   = 1'bz ;
end

task master_reference ;
    input [31:0] addr1 ;
    input [31:0] addr2 ;
    input [3:0]  bc1 ;
    input [3:0]  bc2 ;
    input [3:0]  be ;
    input [31:0] data ;
    input        make_addr_perr1 ;
    input        make_addr_perr2 ;
    output       ok ;
    integer      i ;
    reg          dual_address ;
begin
    ok = 1 ;
    dual_address = (bc1 == `BC_DUAL_ADDR_CYC) ;
    @(posedge CLK) ;
    while( GNT == 1 )
    begin
        REQ <= #1 1'b0 ;
        @(posedge CLK) ;
    end

    REQ   <= #1 1'b1 ;
    FRAME <= #1 1'b0 ;
    AD    <= #1 addr1 ;
    CBE   <= #1 bc1 ;

    // end of first address cycle
    @(posedge CLK) ;
    PAR <= #1 ^{AD, CBE, make_addr_perr1} ;
    if ( dual_address )
    begin
        IRDY <= #1 1'b1 ;
        AD   <= #1 addr2 ;
        CBE  <= #1 bc2 ;
    end
    else
    begin
        IRDY  <= #1 1'b0 ;
        FRAME <= #1 1'b1 ;
        CBE   <= #1 be ;
        if ( bc1[0] )
            AD <= #1 data ;
        else
            AD <= #1 32'hzzzz_zzzz ;
    end

    @(posedge CLK) ;
    CBE <= #1 be ;
    if ( dual_address )
    begin
        PAR   <= #1 ^{AD, CBE, make_addr_perr2} ;
        IRDY  <= #1 1'b0 ;
        FRAME <= #1 1'b1 ;
        if ( bc2[0] )
            AD <= #1 data ;
        else
            AD <= #1 32'hzzzz_zzzz ;
    end
    else
    begin
        if ( bc1[0] )
            PAR <= #1 ^{AD, CBE} ;
        else
            PAR <= #1 1'bz ;
    end

    @(posedge CLK) ;
    if ( AD[31] !== 1'bz )
        PAR <= #1 ^{AD, CBE} ;
    else
        PAR <= #1 1'bz ;

    i = 0 ; // Checking for Master Abort
    while ( (DEVSEL === 1) && (STOP === 1) && (i < 6) )
    begin
        @(posedge CLK) ;
        i = i + 1 ;
    end

    if ( (DEVSEL !== 1) || (STOP !== 1) )
    begin
        ok = 0 ; // If NO Master abort, then NOT OK!
    end

    FRAME <= #1 1'bz ;
    IRDY  <= #1 1'b1 ;
    AD    <= #1 32'hzzzz_zzzz ;
    CBE   <= #1 4'hz ;

    @(posedge CLK) ;
    IRDY <= #1 1'bz ;
    PAR  <= #1 1'bz ;
end
endtask // master_reference
endmodule

