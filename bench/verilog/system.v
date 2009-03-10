`include "pci_constants.v"
`include "bus_commands.v"
`include "pci_testbench_defines.v"
`include "timescale.v"

module SYSTEM ;

`include "pci_blue_constants.vh"
`include "pci_blue_options.vh"

integer tests_successfull ;
integer tests_failed ;
integer tb_log_file ;
reg [799:0] test_name ;

reg pci_clock ;
reg wb_clock ;
reg reset ;

wire [4:0] arb_grant_out ;

wire [31:0] AD ;
wire [3:0]  CBE ;
pullup(INTA) ;
pullup(MAS0_REQ) ;
pullup(MAS1_REQ) ;
pullup(MAS2_REQ) ;
pullup(MAS3_REQ) ;

wire MAS0_GNT = ~arb_grant_out[0];
wire MAS1_GNT = ~arb_grant_out[1] ;
wire MAS2_GNT = ~arb_grant_out[2] ;
wire MAS3_GNT = ~arb_grant_out[3] ;

pullup(FRAME) ;
pullup(IRDY) ;
wire        TAR0_IDSEL = AD[11] ;
`define     TAR0_IDSEL_INDEX    11
`define     TAR0_IDSEL_ADDR     32'h0000_0800
pullup(DEVSEL) ;
pullup(TRDY) ;
pullup(STOP) ;
wire   PAR ;
pullup(PERR) ;
pullup(SERR) ;
wire [3:0] MAS1_IDSEL ;

pullup lockpu ( LOCK ) ;

wire        RST_O ;
wire        INT_O ;
reg         INT_I ;
wire [31:0] ADR_I ;
wire [31:0] SDAT_I ;
wire [31:0] SDAT_O ;
wire [3:0]  SEL_I ;
wire        CYC_I ;
wire        STB_I ;
wire        WE_I ;
wire        CAB_I ;
wire        ACK_O ;
wire        RTY_O ;
wire        ERR_O ;

wire [31:0] ADR_O ;
wire [31:0] MDAT_I ;
wire [31:0] MDAT_O ;
wire [3:0]  SEL_O ;
wire        CYC_O ;
wire        STB_O ;
wire        WE_O ;
wire        CAB_O ;
wire        ACK_I ;
wire        RTY_I ;
wire        ERR_I ;
wire        TAR1_IDSEL = AD[12] ;
`define     TAR1_IDSEL_INDEX    12
`define     TAR1_IDSEL_ADDR     32'h0000_1000
wire        TAR2_IDSEL = AD[13] ;
`define     TAR2_IDSEL_INDEX    13
`define     TAR2_IDSEL_ADDR     32'h0000_2000

wire        reset_wb ; // reset to Wb devices

`ifdef GUEST
    wire    RST = ~reset ;
    assign  reset_wb = RST_O ;
`else
    pullup(RST) ;
    assign  reset_wb = reset ;
`endif

`define PCI_BRIDGE_INSTANCE bridge32_top

TOP `PCI_BRIDGE_INSTANCE
(
    .CLK    ( pci_clock),
    .AD     ( AD ),
    .CBE    ( CBE ),
    .RST    ( RST ),
    .INTA   ( INTA ),
    .REQ    ( MAS0_REQ ),
    .GNT    ( MAS0_GNT ),
    .FRAME  ( FRAME ),
    .IRDY   ( IRDY ),
    .IDSEL  ( TAR0_IDSEL),
    .DEVSEL ( DEVSEL ),
    .TRDY   ( TRDY ),
    .STOP   ( STOP ),
    .PAR    ( PAR ),
    .PERR   ( PERR ),
    .SERR   ( SERR ),

    .CLK_I  ( wb_clock ),
    .RST_I  ( reset ),
    .RST_O  ( RST_O ),
    .INT_I  ( INT_I ),
    .INT_O  ( INT_O ),

    // WISHBONE slave interface
    .ADR_I  ( ADR_I ),
    .SDAT_I ( SDAT_I ),
    .SDAT_O ( SDAT_O ),
    .SEL_I  ( SEL_I ),
    .CYC_I  ( CYC_I ),
    .STB_I  ( STB_I ),
    .WE_I   ( WE_I ),
    .CAB_I  ( CAB_I),
    .ACK_O  ( ACK_O ),
    .RTY_O  ( RTY_O ),
    .ERR_O  ( ERR_O ),

    // WISHBONE master interface
    .ADR_O  ( ADR_O ),
    .MDAT_I ( MDAT_I ),
    .MDAT_O ( MDAT_O ),
    .SEL_O  ( SEL_O ),
    .CYC_O  ( CYC_O ),
    .STB_O  ( STB_O ),
    .WE_O   ( WE_O ),
    .CAB_O  ( CAB_O ),
    .ACK_I  ( ACK_I ),
    .RTY_I  ( RTY_I ),
    .ERR_I  ( ERR_I )
) ;

WB_MASTER_BEHAVIORAL wishbone_master
(
    .CLK_I(wb_clock),
    .RST_I(reset_wb),
    .TAG_I(4'b0000),
    .TAG_O(),
    .ACK_I(ACK_O),
    .ADR_O(ADR_I),
    .CYC_O(CYC_I),
    .DAT_I(SDAT_O),
    .DAT_O(SDAT_I),
    .ERR_I(ERR_O),
    .RTY_I(RTY_O),
    .SEL_O(SEL_I),
    .STB_O(STB_I),
    .WE_O (WE_I),
    .CAB_O(CAB_I)
);

WB_SLAVE_BEHAVIORAL wishbone_slave
(
    .CLK_I              (wb_clock),
    .RST_I              (reset_wb),
    .ACK_O              (ACK_I),
    .ADR_I              (ADR_O),
    .CYC_I              (CYC_O),
    .DAT_O              (MDAT_I),
    .DAT_I              (MDAT_O),
    .ERR_O              (ERR_I),
    .RTY_O              (RTY_I),
    .SEL_I              (SEL_O),
    .STB_I              (STB_O),
    .WE_I               (WE_O),
    .CAB_I              (CAB_O)
);

integer wbu_mon_log_file_desc ;
integer pciu_mon_log_file_desc ;
WB_BUS_MON wbu_wb_mon(
                    .CLK_I(wb_clock),
                    .RST_I(reset_wb),
                    .ACK_I(ACK_O),
                    .ADDR_O(ADR_I),
                    .CYC_O(CYC_I),
                    .DAT_I(SDAT_O),
                    .DAT_O(SDAT_I),
                    .ERR_I(ERR_O),
                    .RTY_I(RTY_O),
                    .SEL_O(SEL_I),
                    .STB_O(STB_I),
                    .WE_O (WE_I),
                    .TAG_I({`WB_TAG_WIDTH{1'b0}}),
                    .TAG_O(),
                    .CAB_O(CAB_I),
                    .log_file_desc ( wbu_mon_log_file_desc )
                  ) ;

WB_BUS_MON pciu_wb_mon(
                    .CLK_I(wb_clock),
                    .RST_I(reset_wb),
                    .ACK_I(ACK_I),
                    .ADDR_O(ADR_O),
                    .CYC_O(CYC_O),
                    .DAT_I(MDAT_I),
                    .DAT_O(MDAT_O),
                    .ERR_I(ERR_I),
                    .RTY_I(RTY_I),
                    .SEL_O(SEL_O),
                    .STB_O(STB_O),
                    .WE_O (WE_O),
                    .TAG_I({`WB_TAG_WIDTH{1'b0}}),
                    .TAG_O(),
                    .CAB_O(CAB_O),
                    .log_file_desc( pciu_mon_log_file_desc )
                  ) ;

reg irq_respond ;
reg [31:0] irq_vector ;
PCI_BEHAVIORAL_IACK_TARGET interrupt_control
(
    .CLK              ( pci_clock),
    .AD               ( AD ),
    .CBE              ( CBE ),
    .RST              ( RST ),
    .FRAME            ( FRAME ),
    .IRDY             ( IRDY ),
    .DEVSEL           ( DEVSEL ),
    .TRDY             ( TRDY ),
    .STOP             ( STOP ),
    .PAR              ( PAR ),
    .respond          ( irq_respond ),
    .interrupt_vector ( irq_vector)
);

// some aditional signals are needed here because of the arbiter
reg [3:0] pci_ext_req_prev ;
always@(posedge pci_clock)
begin
    pci_ext_req_prev <= {~MAS3_REQ, ~MAS2_REQ, ~MAS1_REQ, ~MAS0_REQ} ;
end
reg pci_frame_prev ;
always@(posedge pci_clock)
begin
    pci_frame_prev <= FRAME ;
end
reg pci_irdy_prev ;
always@(posedge pci_clock)
begin
    pci_irdy_prev <= IRDY ;
end

pci_blue_arbiter pci_arbiter
(
  .pci_int_req_direct(1'b0),
  .pci_ext_req_prev(pci_ext_req_prev),
  .pci_int_gnt_direct_out(arb_grant_out[4]),
  .pci_ext_gnt_direct_out(arb_grant_out[3:0]),
  .pci_frame_prev(~pci_frame_prev),
  .pci_irdy_prev(~pci_irdy_prev),
  .pci_irdy_now(~IRDY),
  .arbitration_enable(1'b1),
  .pci_clk(pci_clock),
  .pci_reset_comb(~RST)
);

reg [31:0] target_message ;

// define output enable signals for monitor inputs
// real output enable signals
//{frame_oe, irdy_oe, devsel_t_s_oe, ad_oe, cbe_oe, perr_oe}
`ifdef ACTIVE_LOW_OE
wire devsel_t_s_oe = `PCI_BRIDGE_INSTANCE.DEVSEL_en && `PCI_BRIDGE_INSTANCE.TRDY_en && `PCI_BRIDGE_INSTANCE.STOP_en ;
wire ad_oe         = &(`PCI_BRIDGE_INSTANCE.AD_en) ;
wire cbe_oe        = &(`PCI_BRIDGE_INSTANCE.CBE_en) ;
wire [5:0] r_oe_sigs = {!`PCI_BRIDGE_INSTANCE.FRAME_en,
                        !`PCI_BRIDGE_INSTANCE.IRDY_en,
                        !devsel_t_s_oe,
                        !ad_oe,
                        !cbe_oe,
                        !`PCI_BRIDGE_INSTANCE.PERR_en}
                        ;
`else
`ifdef ACTIVE_HIGH_OE
wire devsel_t_s_oe = `PCI_BRIDGE_INSTANCE.DEVSEL_en || `PCI_BRIDGE_INSTANCE.TRDY_en || `PCI_BRIDGE_INSTANCE.STOP_en ;
wire ad_oe         = |(`PCI_BRIDGE_INSTANCE.AD_en) ;
wire cbe_oe        = |(`PCI_BRIDGE_INSTANCE.CBE_en) ;
wire [5:0] r_oe_sigs = {`PCI_BRIDGE_INSTANCE.FRAME_en,
                        `PCI_BRIDGE_INSTANCE.IRDY_en,
                        devsel_t_s_oe,
                        ad_oe,
                        cbe_oe,
                        `PCI_BRIDGE_INSTANCE.PERR_en}
                        ;
`endif
`endif
/*wire [5:0] oe_sigs_0 = {1'b0,
                        1'b0,
                        pci_target32.ctl_enable | pci_target32.r_ctl_enable,
                        pci_target32.ad_enable,
                        1'b0,
                        pci_target32.err_enable | pci_target32.r_err_enable
                       } ;
*/

wire [5:0] oe_sigs_2 ;
wire [5:0] oe_sigs_1 ;

// signals which are used by test modules to know what to do
triand  test_accepted_l_int, error_event_int;
pullup  (test_accepted_l_int), (error_event_int);

wire    pci_reset_comb  = ~RST;
wire    pci_ext_clk     = pci_clock;

integer pci_mon_log_file_desc ;
pci_bus_monitor monitor32
(
    .pci_ext_ad                 (AD),
    .pci_ext_cbe_l              (CBE),
    .pci_ext_par                (PAR),
    .pci_ext_frame_l            (FRAME),
    .pci_ext_irdy_l             (IRDY),
    .pci_ext_devsel_l           (DEVSEL),
    .pci_ext_trdy_l             (TRDY),
    .pci_ext_stop_l             (STOP),
    .pci_ext_perr_l             (PERR),
    .pci_ext_serr_l             (SERR),
    .pci_real_req_l             (MAS0_REQ),
    .pci_real_gnt_l             (MAS0_GNT),
    .pci_ext_req_l              ({1'b1, MAS3_REQ, MAS2_REQ, MAS1_REQ}),
    .pci_ext_gnt_l              (~arb_grant_out[4:1]),
    .test_error_event           (error_event_int),
    .test_observe_r_oe_sigs     (r_oe_sigs),
    .test_observe_0_oe_sigs     (6'h00),
    .test_observe_1_oe_sigs     (oe_sigs_1),
    .test_observe_2_oe_sigs     (oe_sigs_2),
    .test_observe_3_oe_sigs     (6'h00),
    .pci_ext_reset_l            (RST),
    .pci_ext_clk                (pci_clock),
    .log_file_desc              (pci_mon_log_file_desc)
) ;

reg [2:0]  test_master_number ;
reg [31:0] test_address ;
reg [3:0]  test_command ;
reg [31:0] test_data ;
reg [3:0]  test_byte_enables_l ;
reg [9:0]  test_size ;
reg        test_make_addr_par_error ;
reg        test_make_data_par_error ;
reg [3:0]  test_master_initial_wait_states ;
reg [3:0]  test_master_subsequent_wait_states ;
reg [3:0]  test_target_initial_wait_states ;
reg [3:0]  test_target_subsequent_wait_states ;
reg [1:0]  test_target_devsel_speed ;
reg        test_fast_back_to_back ;
reg [2:0]  test_target_termination ;
reg        test_expect_master_abort ;
reg        test_start ;
reg [25:0] test_target_response ;

wire [31:0] master2_received_data ;
wire        master2_received_data_valid ;
reg         master2_check_received_data ;
pci_behaviorial_device pci_behaviorial_device2
(
    .pci_ext_ad(AD),
    .pci_ext_cbe_l(CBE),
    .pci_ext_par(PAR),
    .pci_ext_frame_l(FRAME),
    .pci_ext_irdy_l(IRDY),
    .pci_ext_devsel_l(DEVSEL),
    .pci_ext_trdy_l(TRDY),
    .pci_ext_stop_l(STOP),
    .pci_ext_perr_l(PERR),
    .pci_ext_serr_l(SERR),
    .pci_ext_idsel(TAR2_IDSEL),
    .pci_ext_inta_l(INTA),
    .pci_ext_req_l(MAS2_REQ),
    .pci_ext_gnt_l(MAS2_GNT),
    .pci_ext_reset_l(RST),
    .pci_ext_clk(pci_clock),

// Signals used by the test bench instead of using "." notation
    .test_observe_oe_sigs               (oe_sigs_2[5:0]),               //
    .test_master_number                 (test_master_number[2:0]),
    .test_address                       (test_address[PCI_BUS_DATA_RANGE:0]),
    .test_command                       (test_command[3:0]),
    .test_data                          (test_data[PCI_BUS_DATA_RANGE:0]),
    .test_byte_enables_l                (test_byte_enables_l[PCI_BUS_CBE_RANGE:0]),
    .test_size                          (test_size),
    .test_make_addr_par_error           (test_make_addr_par_error),
    .test_make_data_par_error           (test_make_data_par_error),
    .test_master_initial_wait_states    (test_master_initial_wait_states[3:0]),
    .test_master_subsequent_wait_states (test_master_subsequent_wait_states[3:0]),
    .test_target_initial_wait_states    (test_target_initial_wait_states[3:0]),
    .test_target_subsequent_wait_states (test_target_subsequent_wait_states[3:0]),
    .test_target_devsel_speed           (test_target_devsel_speed[1:0]),
    .test_fast_back_to_back             (test_fast_back_to_back),
    .test_target_termination            (test_target_termination[2:0]),
    .test_expect_master_abort           (test_expect_master_abort),
    .test_start                         (test_start),
    .test_accepted_l                    (test_accepted_l_int),
    .test_error_event                   (error_event_int),
    .test_device_id                     (`Test_Master_2),
    .test_target_response               (test_target_response),

    .master_received_data               (master2_received_data),
    .master_received_data_valid         (master2_received_data_valid),
    .master_check_received_data         (master2_check_received_data)
);

wire [31:0] master1_received_data ;
wire        master1_received_data_valid ;
reg         master1_check_received_data ;
pci_behaviorial_device pci_behaviorial_device1
(
    .pci_ext_ad(AD),
    .pci_ext_cbe_l(CBE),
    .pci_ext_par(PAR),
    .pci_ext_frame_l(FRAME),
    .pci_ext_irdy_l(IRDY),
    .pci_ext_devsel_l(DEVSEL),
    .pci_ext_trdy_l(TRDY),
    .pci_ext_stop_l(STOP),
    .pci_ext_perr_l(PERR),
    .pci_ext_serr_l(SERR),
    .pci_ext_idsel(TAR1_IDSEL),
    .pci_ext_inta_l(INTA),
    .pci_ext_req_l(MAS1_REQ),
    .pci_ext_gnt_l(MAS1_GNT),
    .pci_ext_reset_l(RST),
    .pci_ext_clk(pci_clock),

// Signals used by the test bench instead of using "." notation
    .test_observe_oe_sigs           (oe_sigs_1[5:0]),               //
    .test_master_number                 (test_master_number[2:0]),
    .test_address                               (test_address[PCI_BUS_DATA_RANGE:0]),
    .test_command                       (test_command[3:0]),
    .test_data                          (test_data[PCI_BUS_DATA_RANGE:0]),
    .test_byte_enables_l                (test_byte_enables_l[PCI_BUS_CBE_RANGE:0]),
    .test_size                          (test_size),
    .test_make_addr_par_error           (test_make_addr_par_error),
    .test_make_data_par_error           (test_make_data_par_error),
    .test_master_initial_wait_states    (test_master_initial_wait_states[3:0]),
    .test_master_subsequent_wait_states (test_master_subsequent_wait_states[3:0]),
    .test_target_initial_wait_states    (test_target_initial_wait_states[3:0]),
    .test_target_subsequent_wait_states (test_target_subsequent_wait_states[3:0]),
    .test_target_devsel_speed           (test_target_devsel_speed[1:0]),
    .test_fast_back_to_back             (test_fast_back_to_back),
    .test_target_termination            (test_target_termination[2:0]),
    .test_expect_master_abort           (test_expect_master_abort),
    .test_start                         (test_start),
    .test_accepted_l                    (test_accepted_l_int),
    .test_error_event                   (error_event_int),
    .test_device_id                     (`Test_Master_1),
    .test_target_response               (test_target_response),

    .master_received_data               (master1_received_data),
    .master_received_data_valid         (master1_received_data_valid),
    .master_check_received_data         (master1_check_received_data)
);

pci_unsupported_commands_master ipci_unsupported_commands_master
(
    .CLK    ( pci_clock),
    .AD     ( AD ),
    .CBE    ( CBE ),
    .RST    ( RST ),
    .REQ    ( MAS3_REQ ),
    .GNT    ( MAS3_GNT ),
    .FRAME  ( FRAME ),
    .IRDY   ( IRDY ),
    .DEVSEL ( DEVSEL ),
    .TRDY   ( TRDY ),
    .STOP   ( STOP ),
    .PAR    ( PAR )
) ;

// pci clock generator
always
`ifdef PCI33
    #15 pci_clock = ~pci_clock ;
`else
`ifdef PCI66
    #7.5 pci_clock = ~pci_clock ;
`endif
`endif

// WISHBONE clock generation
always
    #(((1/`WB_FREQ)/2)) wb_clock = ~wb_clock ;

// Make test name visible when the Master starts working on it
reg     [79:0] present_test_name;
reg     [79:0] next_test_name;
wire    test_accepted = ~test_accepted_l_int;
always @(posedge test_accepted)
begin
    present_test_name <= next_test_name;
end

reg [31:0] wmem_data [0:1023] ; // data for memory mapped image writes
reg [31:0] wio_data  [0:1023] ; // data for IO mapped image writes

// basic configuration parameters for both behavioral devices
parameter [2:0] Master_ID_A                           = `Test_Master_1;
parameter [PCI_BUS_DATA_RANGE:0] Target_Config_Addr_A = `TAR2_IDSEL_ADDR;
parameter [PCI_BUS_DATA_RANGE:0] Target_Base_Addr_A0  = `BEH_TAR2_MEM_START;
parameter [PCI_BUS_DATA_RANGE:0] Target_Base_Addr_A1  = `BEH_TAR2_IO_START;

parameter [2:0] Master_ID_B                           = `Test_Master_2;
parameter [PCI_BUS_DATA_RANGE:0] Target_Config_Addr_B = `TAR1_IDSEL_ADDR;
parameter [PCI_BUS_DATA_RANGE:0] Target_Base_Addr_B0  = `BEH_TAR1_MEM_START;
parameter [PCI_BUS_DATA_RANGE:0] Target_Base_Addr_B1  = `BEH_TAR1_IO_START;

// basic configuration parameters for REAL device
reg [PCI_BUS_DATA_RANGE:0] Target_Config_Addr_R ;
reg [PCI_BUS_DATA_RANGE:0] Target_Base_Addr_R [0:5];
reg [PCI_BUS_DATA_RANGE:0] Target_Addr_Mask_R [0:5];
reg [PCI_BUS_DATA_RANGE:0] Target_Tran_Addr_R [0:5];

// reg  [2:0]   ack_err_rty_termination ;
// reg          wait_cycles ;
// reg  [7:0]   num_of_retries ;

//reg [19:0] pci_config_base ;
reg [7:0] system_burst_size ;
reg [7:0] bridge_latency ;
integer   target_mem_image ;
integer   target_io_image ;

initial
begin
    next_test_name[79:0] <= "Nowhere___";
    reset = 1'b1 ;
    pci_clock = 1'b0 ;
    wb_clock  = 1'b1 ;
    target_message = 32'h0000_0000 ;
//  num_of_retries = 8'h01 ;
//  ack_err_rty_termination = 3'b100 ;
//  wait_cycles = 1'b0 ;

    // system paameters
    system_burst_size = 16 ;
    bridge_latency    = 8 ;

    // set initial values for controling the behavioral PCI master
    Target_Config_Addr_R  = `TAR0_IDSEL_ADDR ;
    Target_Base_Addr_R[0] = `TAR0_BASE_ADDR_0;
    Target_Base_Addr_R[1] = `TAR0_BASE_ADDR_1;
    Target_Base_Addr_R[2] = `TAR0_BASE_ADDR_2;
    Target_Base_Addr_R[3] = `TAR0_BASE_ADDR_3;
    Target_Base_Addr_R[4] = `TAR0_BASE_ADDR_4;
    Target_Base_Addr_R[5] = `TAR0_BASE_ADDR_5;

    Target_Addr_Mask_R[0] = `TAR0_ADDR_MASK_0;
    Target_Addr_Mask_R[1] = `TAR0_ADDR_MASK_1;
    Target_Addr_Mask_R[2] = `TAR0_ADDR_MASK_2;
    Target_Addr_Mask_R[3] = `TAR0_ADDR_MASK_3;
    Target_Addr_Mask_R[4] = `TAR0_ADDR_MASK_4;
    Target_Addr_Mask_R[5] = `TAR0_ADDR_MASK_5;

    Target_Tran_Addr_R[0] = `TAR0_TRAN_ADDR_0;
    Target_Tran_Addr_R[1] = `TAR0_TRAN_ADDR_1;
    Target_Tran_Addr_R[2] = `TAR0_TRAN_ADDR_2;
    Target_Tran_Addr_R[3] = `TAR0_TRAN_ADDR_3;
    Target_Tran_Addr_R[4] = `TAR0_TRAN_ADDR_4;
    Target_Tran_Addr_R[5] = `TAR0_TRAN_ADDR_5;

    test_master_number = `Test_Master_2 ;
    test_address = 32'h0000_0000 ;
    test_command = `BC_RESERVED0 ;
    test_data = 32'h0000_0000 ;
    test_byte_enables_l   = 4'hF ;
    test_size = 0 ;
    test_make_addr_par_error = 0 ;
    test_make_data_par_error = 0;
    test_master_initial_wait_states = 0 ;
    test_master_subsequent_wait_states = 0 ;
    test_target_initial_wait_states = 0 ;
    test_target_subsequent_wait_states = 0;
    test_target_devsel_speed = `Test_Devsel_Fast ;
    test_fast_back_to_back = 0 ;
    test_target_termination = `Test_Target_Normal_Completion ;
    test_expect_master_abort = 0 ;
    test_start = 0 ;
    test_target_response = 0 ;

    master1_check_received_data = 0 ;
    master2_check_received_data = 0 ;

    irq_respond = 1 ;
    irq_vector  = 32'hAAAA_AAAA ;

    // fill memory and IO data with random values
    fill_memory ;

    INT_I = 0 ;

    // extract from constants which target image can be used as IO and which as memory
    `ifdef HOST
        target_mem_image = 1 ;
        target_io_image  = 1 ;
    `else
        target_mem_image = -1 ;
        target_io_image     = -1 ;
        if ( `PCI_BA1_MEM_IO === 0 )
            target_mem_image = 1 ;
        else
            target_io_image = 1 ;

        if ( target_mem_image === -1 )
        begin
            `ifdef PCI_IMAGE2
                if ( `PCI_BA2_MEM_IO === 0 )
                    target_mem_image = 2 ;
                else if ( target_io_image === -1 )
                    target_io_image = 2 ;
            `endif
        end

        if ( target_mem_image === -1 )
        begin
            `ifdef PCI_IMAGE3
                if ( `PCI_BA3_MEM_IO === 0 )
                    target_mem_image = 3 ;
                else if ( target_io_image === -1 )
                    target_io_image = 3 ;
            `endif
        end

        if ( target_mem_image === -1 )
        begin
            `ifdef PCI_IMAGE4
                if ( `PCI_BA4_MEM_IO === 0 )
                    target_mem_image = 4 ;
                else if ( target_io_image === -1 )
                    target_io_image = 4 ;
            `endif
        end

        if ( target_mem_image === -1 )
        begin
            `ifdef PCI_IMAGE5
                if ( `PCI_BA5_MEM_IO === 0 )
                    target_mem_image = 5 ;
                else if ( target_io_image === -1 )
                    target_io_image = 5 ;
            `endif
        end
    `endif

    tests_successfull = 0 ;
    tests_failed = 0 ;

    tb_log_file = $fopen("../log/pci_tb.log") ;

    if ( tb_log_file < 2 )
    begin
        $display(" Could not open/create testbench log file in ../log/ directory! " ) ;
        $finish ;
    end

    $fdisplay( tb_log_file, "************************ PCI IP Core Testbench Test results ************************") ;
    $fdisplay( tb_log_file,"" ) ;

    wbu_mon_log_file_desc  = $fopen("../log/wbu_mon.log") ;
    pciu_mon_log_file_desc = $fopen("../log/pciu_mon.log") ;

    if ( (wbu_mon_log_file_desc < 2) || (pciu_mon_log_file_desc < 2 ) )
    begin
        $fdisplay(tb_log_file, "Could not open WISHBONE monitors' log files!") ;
        $finish ;
    end

    $fdisplay(wbu_mon_log_file_desc, "*********************************** Start WISHBONE Slave Bus Monitor log file *******************************************") ;
    $fdisplay(pciu_mon_log_file_desc, "*********************************** Start WISHBONE Master Bus Monitor log file ******************************************") ;

    pci_mon_log_file_desc = $fopen("../log/pci_mon.log") ;
    if ( pci_mon_log_file_desc < 2 )
    begin
        $fdisplay(tb_log_file, "Could not open PCI monitor's log file!") ;
        $finish ;
    end

    $fdisplay(pci_mon_log_file_desc, "*********************************** Start PCI Bus Monitor log file ******************************************") ;

    run_tests ;
end

task fill_memory ;
    integer temp_index ;
begin
    // fill write memories with random data
    for( temp_index = 0; temp_index <=1023; temp_index = temp_index + 1 )
    begin
        wmem_data[temp_index[9:0]] = $random ;
        # 1;
        wio_data[temp_index[9:0]]  = $random ;
        # 1;
    end
    // fill WB slave behavioral MEMORY
    for( temp_index = 0; temp_index <=65535; temp_index = temp_index + 1 )
    begin
        wishbone_slave.wb_memory[temp_index[15:0]] = $random ;
        # 1;
    end
end
endtask // fill_memory

reg [2:0] tb_init_waits ;
reg [2:0] tb_subseq_waits ;
reg [2:0] tb_target_decode_speed ;

task run_tests ;
begin
    // first - reset logic
    do_reset ;
    next_test_name[79:0] <= "Initing...";
    test_target_response[`TARGET_ENCODED_PARAMATERS_ENABLE] = 1 ;

    for ( tb_init_waits = 0 ; tb_init_waits <= 4 ; tb_init_waits = tb_init_waits + 1 )
    begin
        for ( tb_subseq_waits = 0 ; tb_subseq_waits <= 4 ; tb_subseq_waits = tb_subseq_waits + 1 )
        begin

            test_target_response[`TARGET_ENCODED_INIT_WAITSTATES] = 4 - tb_init_waits ;
            test_target_response[`TARGET_ENCODED_SUBS_WAITSTATES] = 4 - tb_subseq_waits ;

            for ( tb_target_decode_speed = 0 ; tb_target_decode_speed <= 3 ; tb_target_decode_speed = tb_target_decode_speed + 1 )
            begin
                test_target_response[`TARGET_ENCODED_DEVSEL_SPEED] = tb_target_decode_speed ;

                `ifdef HOST
                    configure_bridge_target ;
                    find_pci_devices ;
                `endif

                @(posedge pci_clock) ;
                configure_target(1) ;
                @(posedge pci_clock) ;
                configure_target(2) ;

                `ifdef GUEST
                    configure_bridge_target ;
                `endif

               next_test_name[79:0] <= "WB_SLAVE..";

                $display("Testing WISHBONE slave images' features!") ;
                test_wb_image(1) ;

                `ifdef WB_IMAGE2
                    test_wb_image(2) ;
                `else
                    $display(" WB IMAGE 2 not implemented! ") ;
                `endif

                `ifdef WB_IMAGE3
                    test_wb_image(3) ;
                `else
                    $display(" WB IMAGE 3 not implemented! ") ;
                `endif

                `ifdef WB_IMAGE4
                    test_wb_image(4) ;
                `else
                    $display(" WB IMAGE 4 not implemented! ") ;
                `endif

                `ifdef WB_IMAGE5
                    test_wb_image(5) ;
                `else
                    $display(" WB IMAGE 5 not implemented! ") ;
                `endif

                wb_slave_errors ;
                wb_to_pci_error_handling ;

                parity_checking ;

                wb_to_pci_transactions ;

                `ifdef HOST
                iack_cycle ;
                `endif

            end
            $display(" ") ;
            $display("WB slave images' tests finished!") ;

            $display("########################################################################") ;
            $display("########################################################################") ;
            $display("||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||") ;
            $display("########################################################################") ;
            $display("########################################################################") ;

            $display("Testing PCI target images' features!") ;
            configure_bridge_target_base_addresses ;

            `ifdef HOST
             `ifdef NO_CNF_IMAGE
              `ifdef PCI_IMAGE0
            $display("PCI bridge implemented as HOST device, PCI image 0 implemented to access WB bus!") ;
            test_pci_image(0) ;
              `else
            $display("PCI bridge implemented as HOST device, P_BA0 NOT used (no access to Configuration space)!") ;
              `endif
             `else
            $display("PCI bridge implemented as HOST device, P_BA0 used for Read-only access to Configuration space!") ;
             `endif
            `endif

            $display("PCI image 1 is ALWAYS implemented!") ;
            test_pci_image(1) ;

            `ifdef PCI_IMAGE2
            $display("PCI image 2 is implemented!") ;
            test_pci_image(2) ;
            `else
            $display("PCI image 2 is NOT implemented!") ;
            `endif

            `ifdef PCI_IMAGE3
            $display("PCI image 3 is implemented!") ;
            test_pci_image(3) ;
            `else
            $display("PCI image 3 is NOT implemented!") ;
            `endif

            `ifdef PCI_IMAGE4
            $display("PCI image 4 is implemented!") ;
            test_pci_image(4) ;
            `else
            $display("PCI image 4 is NOT implemented!") ;
            `endif

            `ifdef PCI_IMAGE5
            $display("PCI image 5 is implemented!") ;
            test_pci_image(5) ;
            `else
            $display("PCI image 5 is NOT implemented!") ;
            `endif

            test_wb_error_rd ;

            target_fast_back_to_back ;
            target_disconnects ;

            if ( target_io_image !== -1 )
                test_target_abort( target_io_image ) ;
            $display(" ") ;
            $display("PCI target images' tests finished!") ;

            transaction_ordering ;

            target_completion_expiration ;
            $display(" ") ;
            $display("PCI transaction ordering tests finished!") ;
        end
    end

    test_summary ;

    $fclose(wbu_mon_log_file_desc | pciu_mon_log_file_desc | pci_mon_log_file_desc) ;
    $stop ;
end
endtask // run_tests

task do_reset;
begin
    next_test_name[79:0] <= "Reset.....";

    reset = 1'b1 ;
    #100 ;
    `ifdef HOST
        @(posedge wb_clock) ;
    `else
    `ifdef GUEST
        @(posedge pci_clock) ;
    `endif
    `endif

    reset <= 1'b0 ;

end
endtask

/*############################################################################
WB SLAVE UNIT tasks
===================
############################################################################*/

task configure_target ;
    input [1:0]  device_num ;
    reg   [31:0] base_address1 ;
    reg   [31:0] base_address2 ;
    reg   [2:0]  Master_ID;
    reg   [31:0] Target_Config_Addr;
begin
    if (device_num === 1)
    begin
        base_address1       = `BEH_TAR1_MEM_START ;
        base_address2       = `BEH_TAR1_IO_START  ;
        Master_ID           = `Test_Master_2 ;
        Target_Config_Addr  = `TAR1_IDSEL_ADDR ;
    end
    else
    if (device_num === 2)
    begin
        base_address1       = `BEH_TAR2_MEM_START ;
        base_address2       = `BEH_TAR2_IO_START  ;
        Master_ID           = `Test_Master_1 ;
        Target_Config_Addr  = `TAR2_IDSEL_ADDR ;
    end

    // write target's base addresses
    // bus number 0, device number, function number 0, register number = 4 = base address register 0,
    // type 0 cycle, byte enables, base address
    configuration_cycle_write(0, device_num, 0, 4, 0, 4'hF, base_address1) ;
    configuration_cycle_write(0, device_num, 0, 5, 0, 4'hF, base_address2) ;

    // enable target's response and master
    // enable parity errors, disable system error

    configuration_cycle_write(0, device_num, 0, 1, 0, 4'h3, 32'h0000_0047) ;

end
endtask //configure_target

task test_wb_image ;
    input [2:0]  image_num ;
    reg   [11:0] ctrl_offset ;
    reg   [11:0] ba_offset ;
    reg   [11:0] am_offset ;
    reg   [11:0] ta_offset ;
    reg   [11:0] err_cs_offset ;
    reg `WRITE_STIM_TYPE write_data ;
    reg `READ_STIM_TYPE  read_data ;
    reg `READ_RETURN_TYPE read_status ;

    reg `WRITE_RETURN_TYPE write_status ;
    reg `WB_TRANSFER_FLAGS write_flags ;
    reg [31:0] temp_val ;
    reg        ok   ;
    reg [11:0] pci_ctrl_offset ;
    reg [31:0] image_base ;
    reg [31:0] target_address ;
    reg [31:0] translation_address ;
    integer    i ;
    integer    j ;
begin:main
    pci_ctrl_offset = 12'h4 ;
    err_cs_offset   = {4'h1, `W_ERR_CS_ADDR, 2'b00} ;
    // image 0 can only be configuration image - start with 1
    if (image_num === 1)
    begin
        ctrl_offset = {4'h1, `W_IMG_CTRL1_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `W_BA1_ADDR, 2'b00} ;
        am_offset   = {4'h1, `W_AM1_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `W_TA1_ADDR, 2'b00} ;
        test_name   = "WB IMAGE 1 FEATURES TEST" ;
    end
    else if (image_num === 2)
    begin
        ctrl_offset = {4'h1, `W_IMG_CTRL2_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `W_BA2_ADDR, 2'b00} ;
        am_offset   = {4'h1, `W_AM2_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `W_TA2_ADDR, 2'b00} ;
        test_name   = "WB IMAGE 2 FEATURES TEST" ;
    end
    else if (image_num === 3)
    begin
        ctrl_offset = {4'h1, `W_IMG_CTRL3_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `W_BA3_ADDR, 2'b00} ;
        am_offset   = {4'h1, `W_AM3_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `W_TA3_ADDR, 2'b00} ;
        test_name   = "WB IMAGE 3 FEATURES TEST" ;
    end
    else if (image_num === 4)
    begin
        ctrl_offset = {4'h1, `W_IMG_CTRL4_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `W_BA4_ADDR, 2'b00} ;
        am_offset   = {4'h1, `W_AM4_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `W_TA4_ADDR, 2'b00} ;
        test_name   = "WB IMAGE 4 FEATURES TEST" ;
    end
    else if (image_num === 5)
    begin
        ctrl_offset = {4'h1, `W_IMG_CTRL5_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `W_BA5_ADDR, 2'b00} ;
        am_offset   = {4'h1, `W_AM5_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `W_TA5_ADDR, 2'b00} ;
        test_name   = "WB IMAGE 5 FEATURES TEST" ;
    end
    else
    begin
        test_name   = "WB IMAGES' FEATURES TEST" ;
        test_fail("invalid image number was provided for call to task test_wb_image") ;
        disable main ;
    end

    target_address  = `BEH_TAR1_MEM_START ;
    image_base      = 0 ;
    image_base[`PCI_BASE_ADDR0_MATCH_RANGE] = target_address[`PCI_BASE_ADDR0_MATCH_RANGE] ;

    target_address[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] = image_base[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] ;
    target_address[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0]  = image_base[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] ;
    write_flags                      = 0 ;
    write_flags`INIT_WAITS           = tb_init_waits ;
    write_flags`SUBSEQ_WAITS         = tb_subseq_waits ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    test_name = "WB IMAGE CONFIGURATION" ;
    // enable master & target operation
    config_write( pci_ctrl_offset, 32'h0000_0007, 4'h1, ok) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write PCI Device Control register! Time %t ", image_num, $time) ;
        test_fail("write to PCI Device Control register didn't succeede");
        disable main ;
    end

    config_write( err_cs_offset, 32'h0000_0000, 4'h1, ok) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write WB Error Control and Status register! Time %t ", image_num, $time) ;
        test_fail("write to WB Error Control and Status register didn't succeede");
        disable main ;
    end

    // prepare image control register
    config_write( ctrl_offset, 32'h0000_0000, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write W_IMG_CTRL%d register! Time %t ", image_num, $time) ;
        test_fail("write to WB Image Control register didn't succeede");
        disable main ;
    end

    // prepare base address register
    config_write( ba_offset, image_base, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write W_BA%d register! Time %t ", image_num, $time) ;
        test_fail("write to WB Base Address register didn't succeede");
        disable main ;
    end

    // write address mask register
    config_write( am_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write W_AM%d register! Time %t ", image_num, $time) ;
        test_fail("write to WB Address Mask register didn't succeede");
        disable main ;
    end

    fork
    begin
        write_data`WRITE_ADDRESS = target_address ;
        write_data`WRITE_DATA    = wmem_data[0] ;
        write_data`WRITE_SEL     = 4'hF ;

        // handle retries from now on
        write_flags`WB_TRANSFER_AUTO_RTY = 1 ;

        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        test_name = "NORMAL SINGLE MEMORY WRITE THROUGH WB IMAGE TO PCI" ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Image testing failed! Bridge failed to process single memory write! Time %t ", $time) ;
            test_fail("WB Slave state machine failed to post single memory write");
            disable main ;
        end

        // read written data back
        read_data`READ_ADDRESS  = target_address ;
        read_data`READ_SEL      = 4'hF ;
        read_data`READ_TAG_STIM = 0 ;
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
        if (read_status`CYC_ACTUAL_TRANSFER !== 1)
        begin
            $display("Image testing failed! Bridge failed to process single memory read! Time %t ", $time) ;
            test_fail("PCI bridge didn't process the read as expected");
            disable main ;
        end

        if (read_status`READ_DATA !== wmem_data[0])
        begin
            display_warning(target_address, wmem_data[0], read_status`READ_DATA) ;
            test_fail("PCI bridge returned unexpected Read Data");
        end
        else
            test_ok ;
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 1, 0, 1, 0, 0, ok ) ;
        if ( ok !== 1 )
        begin
            test_fail("because single memory write from WB to PCI didn't engage expected transaction on PCI bus") ;
        end
        else
            test_ok ;

        test_name = "NORMAL SINGLE MEMORY READ THROUGH WB IMAGE FROM PCI" ;
        pci_transaction_progress_monitor( target_address, `BC_MEM_READ, 1, 0, 1, 0, 0, ok ) ;
        if ( ok !== 1 )
        begin
            test_fail("because single memory read from WB to PCI didn't engage expected transaction on PCI bus") ;
        end
    end
    join

    // if address translation is implemented - try it out
    translation_address = image_base ;
    `ifdef ADDR_TRAN_IMPL
    test_name = "CONFIGURE ADDRESS TRANSLATION FOR WISHBONE IMAGE" ;
    config_write( ta_offset, translation_address, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write W_TA%d register! Time %t ", image_num, $time) ;
        test_fail("write to WB Image Translation Address Register failed") ;
        disable main ;
    end

    target_address  = `BEH_TAR2_MEM_START ;
    image_base      = 0 ;
    image_base[`PCI_BASE_ADDR0_MATCH_RANGE] = target_address[`PCI_BASE_ADDR0_MATCH_RANGE] ;

    target_address[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] = image_base[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] ;
    target_address[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0]  = translation_address[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] ;

    write_flags                      = 0 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    test_name = "CHANGE WB IMAGE BASE ADDRESS" ;
    config_write( ba_offset, image_base, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write W_BA%d register! Time %t ", image_num, $time) ;
        test_fail("write to WB Image Base Address Register failed") ;
        disable main ;
    end

    test_name = "ENABLE WB IMAGE ADDRESS TRANSLATION" ;
    // enable address translation
    config_write( ctrl_offset, 4, 4'h1, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write W_IMG_CTRL%d register! Time %t ", image_num, $time) ;
        test_fail("write to WB Image Control Register failed") ;
        disable main ;
    end

    `endif

    fork
    begin
        write_data`WRITE_ADDRESS = target_address + 4 ;
        write_data`WRITE_DATA    = wmem_data[1] ;
        write_data`WRITE_SEL     = 4'hF ;

        write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

        `ifdef ADDR_TRAN_IMPL
        test_name = "NORMAL SINGLE MEMORY WRITE THROUGH WB IMAGE TO PCI WITH ADDRESS TRANSLATION" ;
        `else
        test_name = "NORMAL SINGLE MEMORY WRITE THROUGH WB IMAGE TO PCI" ;
        `endif

        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Image testing failed! Bridge failed to process single memory write! Time %t ", $time) ;
            test_fail("WB Slave state machine failed to post single memory write") ;
            disable main ;
        end

        // read written data back
        read_data`READ_ADDRESS  = target_address + 4 ;
        read_data`READ_SEL      = 4'hF ;
        read_data`READ_TAG_STIM = 0 ;

        write_flags`WB_TRANSFER_AUTO_RTY = 1 ;

        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
        if (read_status`CYC_ACTUAL_TRANSFER !== 1)
        begin
            $display("Image testing failed! Bridge failed to process single memory read! Time %t ", $time) ;
            test_fail("PCI bridge failed to process single delayed memory read") ;
            disable main ;
        end

        if (read_status`READ_DATA !== wmem_data[1])
        begin
            display_warning(target_address + 4, wmem_data[1], read_status`READ_DATA) ;
            test_fail("PCI bridge returned unexpected Read Data");
        end
        else
            test_ok ;
    end
    begin
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 4, translation_address, 1'b1 ), `BC_MEM_WRITE, 1, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
        begin
            test_fail("single memory write didn't engage expected transaction on PCI bus") ;
        end
        else
            test_ok ;

        test_name = "NORMAL SINGLE MEMORY READ THROUGH WB IMAGE FROM PCI" ;
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 4, translation_address, 1'b1 ), `BC_MEM_READ, 1, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
        begin
            test_fail("single memory read didn't engage expected transaction on PCI bus") ;
        end
    end
    join

    // now do one burst write - length of 6 - minimum depth of fifo is 8, one loc. is always free and one is taken up by address entry
    // prepare write data
    for ( i = 0 ; i < 6 ; i = i + 1 )
    begin
        write_data`WRITE_DATA    = wmem_data[2 + i] ;
        write_data`WRITE_ADDRESS = target_address + 8 + 4*i ;
        write_data`WRITE_SEL     = 4'hF ;
        wishbone_master.blk_write_data[i] = write_data ;
    end

    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
    write_flags`WB_TRANSFER_CAB    = 1 ;
    write_flags`WB_TRANSFER_SIZE   = 6 ;

    fork
    begin
        test_name = "CAB MEMORY WRITE THROUGH WB SLAVE TO PCI" ;
        wishbone_master.wb_block_write(write_flags, write_status) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 6 )
        begin
            $display("Image testing failed! Bridge failed to process CAB memory write! Time %t ", $time) ;
            test_fail("WB Slave state machine failed to post CAB memory write") ;
            disable main ;
        end
    end
    begin
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 8, translation_address, 1'b1 ), `BC_MEM_WRITE, 6, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
        begin
            test_fail("CAB memory write didn't engage expected transaction on PCI bus") ;
        end
        else
            test_ok ;
    end
    join

    // set burst size and latency timer
    config_write( 12'h00C, {bridge_latency, 8'd4}, 4'b1111, ok ) ;

    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_CAB    = 1 ;
    write_flags`WB_TRANSFER_SIZE   = 4 ;

    // prepare read data
    for ( i = 0 ; i < 4 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + 8 + 4*i ;
        read_data`READ_SEL     = 4'hF ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    fork
    begin
        test_name = "CAB MEMORY READ THROUGH WB SLAVE FROM PCI" ;
        wishbone_master.wb_block_read(write_flags, read_status) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 4 )
        begin
            $display("Image testing failed! Bridge failed to process CAB memory read! Time %t ", $time) ;
            test_fail("PCI Bridge Failed to process delayed CAB read") ;
            disable main ;
        end

        // check data read from target
        for ( i = 0 ; i < 4 ; i = i + 1 )
        begin
            read_status = wishbone_master.blk_read_data_out[i] ;
            if (read_status`READ_DATA !== wmem_data[2 + i])
            begin
                display_warning(target_address + 8 + 4 * i, wmem_data[2 + i], read_status`READ_DATA) ;
                test_fail("data returned by PCI bridge during completion of Delayed Read didn't have expected value") ;
            end
        end
    end
    begin
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 8, translation_address, 1'b1 ), `BC_MEM_READ, 1, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("CAB memory read divided into single transactions didn't engage expected transaction on PCI bus") ;
        else
            test_ok ;

        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 12, translation_address, 1'b1 ), `BC_MEM_READ, 1, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("CAB memory read divided into single transactions didn't engage expected transaction on PCI bus") ;
        else
            test_ok ;

        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 16, translation_address, 1'b1 ), `BC_MEM_READ, 1, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("CAB memory read divided into single transactions didn't engage expected transaction on PCI bus") ;
        else
            test_ok ;

        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 20, translation_address, 1'b1 ), `BC_MEM_READ, 1, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("CAB memory read divided into single transactions didn't engage expected transaction on PCI bus") ;
        else
            test_ok ;

    end
    join

    // now repeat this same burst read with various image features enabled or disabled
    test_name   = "ENABLING/DISABLING IMAGE'S FEATURES" ;
    config_write( ctrl_offset, 5, 4'b1, ok ) ;
    if (ok !== 1)
    begin
        $display("WISHBONE image %d test failed! Couldn't write image's control register! Time %t ", image_num, $time) ;
        test_fail("write to WB Image control register failed") ;
        disable main ;
    end

    fork
    begin
        test_name = "CAB MEMORY READ THROUGH WB IMAGE WITH READ BURSTING ENABLED" ;
        wishbone_master.wb_block_read(write_flags, read_status) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 4 )
        begin
            $display("Image testing failed! Bridge failed to process CAB memory read! Time %t ", $time) ;
            test_fail("delayed CAB memory read wasn't processed as expected") ;
            disable main ;
        end

        // check data read from target
        for ( i = 0 ; i < 4 ; i = i + 1 )
        begin
            read_status = wishbone_master.blk_read_data_out[i] ;
            if (read_status`READ_DATA !== wmem_data[2 + i])
            begin
                display_warning(target_address + 8 + 4 * i, wmem_data[2 + i], read_status`READ_DATA) ;
                test_fail("delayed CAB memory read data value returned was not as expected") ;
            end
            else
                test_ok ;
        end
    end
    begin
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 8, translation_address, 1'b1 ), `BC_MEM_READ_LN, 4, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail( "burst Delayed Read didn't engage expected transaction on PCI bus" ) ;
    end
    join

    read_data`READ_ADDRESS  = target_address ;
    read_data`READ_SEL      = 4'hF ;
    read_data`READ_TAG_STIM = 0 ;

    test_name = "SINGLE MEMORY READ THROUGH WB IMAGE WITH READ BURSTING ENABLED" ;
    fork
    begin
        wishbone_master.wb_single_read(read_data, write_flags, read_status) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Image testing failed! Bridge failed to process single memory read! Time %t ", $time) ;
            test_fail("delayed single memory read wasn't processed as expected") ;
            disable main ;
        end

        if (read_status`READ_DATA !== wmem_data[0])
        begin
            display_warning(target_address, wmem_data[0], read_status`READ_DATA) ;
            test_fail("delayed single memory read data value returned was not as expected") ;
        end
        else
            test_ok ;
    end
    begin
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address, translation_address, 1'b1 ), `BC_MEM_READ, 1, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail( "single Delayed Read didn't engage expected transaction on PCI bus" ) ;
    end
    join

    test_name   = "ENABLING/DISABLING IMAGE'S FEATURES" ;
    config_write( ctrl_offset, 6, 4'b1, ok ) ;
    if (ok !== 1)
    begin
        $display("WISHBONE image %d test failed! Couldn't write image's control register! Time %t ", image_num, $time) ;
        test_fail("write to WB Image control register failed") ;
        disable main ;
    end

    test_name = "CAB MEMORY READ THROUGH WB IMAGE WITH READ BURSTING ENABLED" ;
    fork
    begin
        wishbone_master.wb_block_read(write_flags, read_status) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 4 )
        begin
            $display("Image testing failed! Bridge failed to process CAB memory read! Time %t ", $time) ;
            test_fail("delayed CAB memory read wasn't processed as expected") ;
            disable main ;
        end

        // check data read from target
        for ( i = 0 ; i < 4 ; i = i + 1 )
        begin
            read_status = wishbone_master.blk_read_data_out[i] ;
            if (read_status`READ_DATA !== wmem_data[2 + i])
            begin
                display_warning(target_address + 8 + 4 * i, wmem_data[2 + i], read_status`READ_DATA) ;
                test_fail("delayed CAB memory read data value returned was not as expected") ;
            end
            else
                test_ok ;
        end
    end
    begin
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 8, translation_address, 1'b1 ), `BC_MEM_READ, 4, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail( "burst Delayed Read didn't engage expected transaction on PCI bus" ) ;
    end
    join

    read_data`READ_ADDRESS  = target_address + 4 ;
    read_data`READ_SEL      = 4'hF ;
    read_data`READ_TAG_STIM = 0 ;

    test_name = "SINGLE MEMORY READ THROUGH WB IMAGE WITH READ BURSTING ENABLED" ;
    fork
    begin
        wishbone_master.wb_single_read(read_data, write_flags, read_status) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Image testing failed! Bridge failed to process single memory read! Time %t ", $time) ;
            test_fail("delayed single memory read wasn't processed as expected") ;
            disable main ;
        end

        if (read_status`READ_DATA !== wmem_data[1])
        begin
            display_warning(target_address, wmem_data[1], read_status`READ_DATA) ;
            test_fail("delayed single memory read data value returned was not as expected") ;
        end
        else
            test_ok ;
    end
    begin
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 4, translation_address, 1'b1 ), `BC_MEM_READ, 1, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail( "single Delayed Read didn't engage expected transaction on PCI bus" ) ;
    end
    join

    test_name   = "ENABLING/DISABLING IMAGE'S FEATURES" ;
    config_write( ctrl_offset, 7, 4'b1, ok ) ;
    if (ok !== 1)
    begin
        $display("WISHBONE image %d test failed! Couldn't write image's control register! Time %t ", image_num, $time) ;
        test_fail("write to WB Image control register failed") ;
        disable main ;
    end

    test_name = "CAB MEMORY READ THROUGH WB IMAGE WITH READ BURSTING ENABLED" ;
    fork
    begin
        wishbone_master.wb_block_read(write_flags, read_status) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 4 )
        begin
            $display("Image testing failed! Bridge failed to process CAB memory read! Time %t ", $time) ;
            test_fail("delayed CAB memory read wasn't processed as expected") ;
            disable main ;
        end

        // check data read from target
        for ( i = 0 ; i < 4 ; i = i + 1 )
        begin
            read_status = wishbone_master.blk_read_data_out[i] ;
            if (read_status`READ_DATA !== wmem_data[2 + i])
            begin
                display_warning(target_address + 8 + 4 * i, wmem_data[2 + i], read_status`READ_DATA) ;
                test_fail("delayed CAB memory read data value returned was not as expected") ;
            end
            else
                test_ok ;
        end
    end
    begin
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 8, translation_address, 1'b1 ), `BC_MEM_READ_MUL, `WBR_DEPTH - 1, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail( "burst Delayed Read didn't engage expected transaction on PCI bus" ) ;
    end
    join

    read_data`READ_ADDRESS  = target_address + 8 ;
    read_data`READ_SEL      = 4'hF ;
    read_data`READ_TAG_STIM = 0 ;

    test_name = "SINGLE MEMORY READ THROUGH WB IMAGE WITH READ BURSTING ENABLED" ;
    fork
    begin
        wishbone_master.wb_single_read(read_data, write_flags, read_status) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Image testing failed! Bridge failed to process single memory read! Time %t ", $time) ;
            test_fail("delayed single memory read wasn't processed as expected") ;
            disable main ;
        end

        if (read_status`READ_DATA !== wmem_data[2])
        begin
            display_warning(target_address, wmem_data[2], read_status`READ_DATA) ;
            test_fail("delayed single memory read data value returned was not as expected") ;
        end
        else
            test_ok ;
    end
    begin
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 8, translation_address, 1'b1 ), `BC_MEM_READ, 1, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail( "single Delayed Read didn't engage expected transaction on PCI bus" ) ;
    end
    join

    // map image to IO space
    test_name   = "ENABLING/DISABLING IMAGE'S FEATURES" ;
    config_write( ba_offset, image_base | 1, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write W_BA%d register! Time %t ", image_num, $time) ;
        test_fail("write to WB Image Base Address register failed") ;
        disable main ;
    end

    write_data`WRITE_ADDRESS = target_address ;
    write_data`WRITE_DATA    = wmem_data[11] ;
    write_data`WRITE_SEL     = 4'hF ;

    // handle retries from now on
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    test_name   = "I/O WRITE TRANSACTION FROM WB TO PCI TEST" ;
    fork
    begin
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Image testing failed! Bridge failed to process single IO write! Time %t ", $time) ;
            test_fail("single I/O write was not posted by WB Slave state machine as expected");
            disable main ;
        end
    end
    begin
        // currently IO commands not supported in behavioral models - master abort
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address, translation_address, 1'b1 ), `BC_IO_WRITE, 0, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail( "single I/O write didn't engage expected transaction on PCI bus" ) ;
        else
            test_ok ;
    end
    join

    read_data`READ_ADDRESS  = target_address ;
    read_data`READ_SEL      = 4'hF ;
    read_data`READ_TAG_STIM = 0 ;

    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;

    // currently io commands are not supported by behavioral target - transfer should not be completed
    test_name   = "I/O READ TRANSACTION FROM WB TO PCI TEST" ;
    fork
    begin
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
        if (read_status`CYC_ERR !== 1)
        begin
            $display("Image testing failed! Bridge failed to process single IO read! Time %t ", $time) ;
            test_fail("single I/O read was not finished by WB Slave state machine as expected");
            disable main ;
        end
        else
            test_ok ;
    end
    begin
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address, translation_address, 1'b1 ), `BC_IO_READ, 0, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail( "single I/O read didn't engage expected transaction on PCI bus" ) ;
    end
    join

    // test byte addressing
    read_data`READ_ADDRESS = target_address + 2 ;
    read_data`READ_SEL     = 4'b1100 ;

    fork
    begin
        // currently io commands are not supported by behavioral target - transfer should not be completed
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
        if (read_status`CYC_ERR !== 1)
        begin
            $display("Image testing failed! Bridge failed to process single IO read! Time %t ", $time) ;
            test_fail("single I/O read was not finished by WB Slave state machine as expected");
            disable main ;
        end
        else test_ok ;
    end
    begin
        pci_transaction_progress_monitor( wb_to_pci_addr_convert( target_address + 2, translation_address, 1'b1 ), `BC_IO_READ, 0, 0, 1'b1, 0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail( "single I/O read didn't engage expected transaction on PCI bus" ) ;
    end
    join

    test_name = "CHECK MAXIMUM IMAGE SIZE" ;
    target_address = ~({`WB_CONFIGURATION_BASE, 12'hFFF}) ;
    config_write(ba_offset, target_address + 1, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        test_fail("WB Base address register could not be written") ;
        disable main ;
    end

    config_write(am_offset, 32'h8000_0000, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        test_fail("WB Address Mask register could not be written") ;
        disable main ;
    end

    config_write(ctrl_offset, 32'h0000_0000, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        test_fail("WB Image Control register could not be written") ;
        disable main ;
    end

    write_data`WRITE_ADDRESS = {target_address[31], 31'h7FFF_FFFF} ;
    write_data`WRITE_DATA    = wmem_data[11] ;
    write_data`WRITE_SEL     = 4'b1000 ;

    // handle retries from now on
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    fork
    begin
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Image testing failed! Bridge failed to process single IO write! Time %t ", $time) ;
            test_fail("single I/O write was not posted by WB Slave state machine as expected");
            disable main ;
        end
    end
    begin
        // currently IO commands not supported in behavioral models - master abort
        pci_transaction_progress_monitor( write_data`WRITE_ADDRESS, `BC_IO_WRITE, 0, 0, 1'b0, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail( "single I/O write didn't engage expected transaction on PCI bus" ) ;
        else
            test_ok ;
    end
    join

    read_data`READ_ADDRESS = write_data`WRITE_ADDRESS ;
    read_data`READ_SEL     = write_data`WRITE_SEL ;
    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;

    fork
    begin
        // currently io commands are not supported by behavioral target - transfer should not be completed
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
        if (read_status`CYC_ERR !== 1)
        begin
            $display("Image testing failed! Bridge failed to process single IO read! Time %t ", $time) ;
            test_fail("single I/O read was not finished by WB Slave state machine as expected");
            disable main ;
        end
        else test_ok ;
    end
    begin
        pci_transaction_progress_monitor( read_data`READ_ADDRESS, `BC_IO_READ, 0, 0, 1'b0, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail( "single I/O read didn't engage expected transaction on PCI bus" ) ;
    end
    join

    test_name = "DISABLING WB IMAGE" ;

    // disable current image
    config_write( am_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write W_AM%d register! Time %t ", image_num, $time) ;
        test_fail("write to WB Image Address Mask register not accepted as expected") ;
        disable main ;
    end

    // clear master abort status bit
    config_write( pci_ctrl_offset, 32'hFFFF_0000, 4'hC, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write W_AM%d register! Time %t ", image_num, $time) ;
        test_fail("write to PCI Device Status register not accepted as expected") ;
        disable main ;
    end

end //main
endtask //test_wb_image

task wb_slave_errors ;
    reg   [11:0] ctrl_offset ;
    reg   [11:0] ba_offset ;
    reg   [11:0] am_offset ;
    reg   [11:0] ta_offset ;
    reg `WRITE_STIM_TYPE write_data ;
    reg `READ_STIM_TYPE  read_data ;
    reg `READ_RETURN_TYPE read_status ;

    reg `WRITE_RETURN_TYPE write_status ;
    reg `WB_TRANSFER_FLAGS write_flags ;
    reg [31:0] temp_val1 ;
    reg [31:0] temp_val2 ;
    reg        ok   ;
    reg [11:0] pci_ctrl_offset ;
    reg [31:0] image_base ;
    reg [31:0] target_address ;
    integer    i ;
    reg skip ;
fork
begin:main

    `ifdef GUEST
        skip = 1 ;
    `else
        skip = 0 ;
    `endif

    pci_ctrl_offset = 12'h4 ;

    // image 1 is used for error testing, since it is always implemented
    ctrl_offset = {4'h1, `W_IMG_CTRL1_ADDR, 2'b00} ;
    ba_offset   = {4'h1, `W_BA1_ADDR, 2'b00} ;
    am_offset   = {4'h1, `W_AM1_ADDR, 2'b00} ;
    ta_offset   = {4'h1, `W_TA1_ADDR, 2'b00} ;

    target_address  = `BEH_TAR1_MEM_START ;
    image_base      = 0 ;
    image_base[`PCI_BASE_ADDR0_MATCH_RANGE] = target_address[`PCI_BASE_ADDR0_MATCH_RANGE] ;

    target_address[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] = image_base[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] ;
    target_address[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0]  = image_base[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] ;
    write_flags                    = 0 ;
    write_flags`INIT_WAITS         = tb_init_waits ;
    write_flags`SUBSEQ_WAITS       = tb_subseq_waits ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    // enable master & target operation
    test_name = "CONFIGURING IMAGE FOR WB SLAVE ERROR TERMINATION TESTING" ;
    config_write( pci_ctrl_offset, 32'h0000_0007, 4'h1, ok) ;
    if ( ok !== 1 )
    begin
        $display("WISHBONE slave error termination testing failed! Failed to write PCI Device Control register! Time %t ", $time) ;
        test_fail("PCI Device Control register couldn't be written") ;
        disable no_transaction ;
        disable main ;
    end

    // prepare image control register
    config_write( ctrl_offset, 32'h0000_0000, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("WISHBONE slave error termination testing failed! Failed to write W_IMG_CTRL1 register! Time %t ", $time) ;
        test_fail("WB Image Control register couldn't be written") ;
        disable no_transaction ;
        disable main ;
    end

    // prepare base address register
    config_write( ba_offset, image_base, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("WISHBONE slave error termination testing failed! Failed to write W_BA1 register! Time %t ", $time) ;
        test_fail("WB Base Address register couldn't be written") ;
        disable no_transaction ;
        disable main ;
    end

    // write address mask register
    config_write( am_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("WISHBONE slave error termination testing failed! Failed to write W_AM1 register! Time %t ", $time) ;
        test_fail("WB Address Mask register couldn't be written") ;
        disable no_transaction ;
        disable main ;
    end

    $display("************************* Testing WISHBONE Slave's response to erroneous memory accesses **************************") ;

    skip = 0 ;

    // memory mapped image - access is erroneous when address is not alligned
    write_data`WRITE_ADDRESS = target_address + 1 ;
    write_data`WRITE_DATA    = wmem_data[0] ;
    write_data`WRITE_SEL     = 4'hF ;

    // handle retries from now on
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    test_name = "SINGLE ERRONEOUS MEMORY WRITE TO WB SLAVE" ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on access to non-alligned memory address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous memory write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    write_data`WRITE_ADDRESS = target_address + 2 ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on access to non-alligned memory address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous memory write as expected") ;
        disable no_transaction ;
        disable main ;

    end

    write_data`WRITE_ADDRESS = target_address + 3 ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on access to non-alligned memory address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous memory write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    test_ok ;

    // perform same tests for read accesses
    test_name = "SINGLE ERRONEOUS MEMORY READ TO WB SLAVE" ;

    read_data`READ_ADDRESS  = target_address + 2 ;
    read_data`READ_SEL      = 4'hF ;
    read_data`READ_TAG_STIM = 0 ;

    wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
    if ( (read_status`CYC_ACTUAL_TRANSFER !== 0) || (read_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on access to non-alligned memory address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous memory read as expected") ;
        disable no_transaction ;
        disable main ;
    end

    test_ok ;

    // prepare write data
    for ( i = 0 ; i < 6 ; i = i + 1 )
    begin
        write_data`WRITE_DATA    = wmem_data[i] ;
        write_data`WRITE_ADDRESS = target_address +  4*i + 2 ;
        write_data`WRITE_SEL     = 4'hF ;
        wishbone_master.blk_write_data[i] = write_data ;
    end

    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
    write_flags`WB_TRANSFER_CAB    = 1 ;
    write_flags`WB_TRANSFER_SIZE   = 6 ;

    test_name = "ERRONEOUS CAB MEMORY WRITE TO WB SLAVE" ;
    wishbone_master.wb_block_write(write_flags, write_status) ;

    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on access to non-alligned memory address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous CAB memory write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    // prepare read data
    for ( i = 0 ; i < 6 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + 4*i + 3 ;
        read_data`READ_SEL     = 4'hF ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    test_name = "ERRONEOUS CAB MEMORY READ TO WB SLAVE" ;
    wishbone_master.wb_block_read(write_flags, read_status) ;
    if ( (read_status`CYC_ACTUAL_TRANSFER !== 0) || (read_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on access to non-alligned memory address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous CAB memory read as expected") ;
        disable no_transaction ;
        disable main ;
    end

    test_ok ;

    $display("************************* DONE testing WISHBONE Slave's response to erroneous memory accesses **************************") ;
    $display("***************************** Testing WISHBONE Slave's response to erroneous IO accesses ******************************") ;

    // map image to IO space
    `ifdef GUEST
        skip = 1 ;
    `endif

    test_name = "RECONFIGURING IMAGE TO I/O MAPPED ADDRESS SPACE" ;
    config_write( ba_offset, image_base | 1, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("WISHBONE slave error termination testing failed! Failed to write W_BA1 register! Time %t ", $time) ;
        test_fail("WB Image Base Address register couldn't be written") ;
        disable no_transaction ;
        disable main ;
    end

    skip = 0 ;

    write_data`WRITE_ADDRESS = target_address ;
    write_data`WRITE_DATA    = wmem_data[0] ;
    write_data`WRITE_SEL     = 4'b1010 ;

    // don't handle retries
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    test_name = "ERRONEOUS I/O WRITE TO WB SLAVE" ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on erroneous access to IO mapped address! Time %t ", $time) ;
        $display("ADDRESS : %h, SEL : %b ", write_data`WRITE_ADDRESS, write_data`WRITE_SEL) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous I/O write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    write_data`WRITE_ADDRESS = target_address + 1 ;
    write_data`WRITE_SEL     = 4'b0011 ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on erroneous access to IO mapped address! Time %t ", $time) ;
        $display("ADDRESS : %h, SEL : %b ", write_data`WRITE_ADDRESS, write_data`WRITE_SEL) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous I/O write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    write_data`WRITE_SEL     = 4'b1100 ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on erroneous access to IO mapped address! Time %t ", $time) ;
        $display("ADDRESS : %h, SEL : %b ", write_data`WRITE_ADDRESS, write_data`WRITE_SEL) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous I/O write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    write_data`WRITE_ADDRESS = target_address + 2 ;
    write_data`WRITE_SEL     = 4'b0101 ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on erroneous access to IO mapped address! Time %t ", $time) ;
        $display("ADDRESS : %h, SEL : %b ", write_data`WRITE_ADDRESS, write_data`WRITE_SEL) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous I/O write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    write_data`WRITE_SEL     = 4'b1000 ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on erroneous access to IO mapped address! Time %t ", $time) ;
        $display("ADDRESS : %h, SEL : %b ", write_data`WRITE_ADDRESS, write_data`WRITE_SEL) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous I/O write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    write_data`WRITE_ADDRESS = target_address + 3 ;
    write_data`WRITE_SEL     = 4'b1010 ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on erroneous access to IO mapped address! Time %t ", $time) ;
        $display("ADDRESS : %h, SEL : %b ", write_data`WRITE_ADDRESS, write_data`WRITE_SEL) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous I/O write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    write_data`WRITE_SEL     = 4'b0110 ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on erroneous access to IO mapped address! Time %t ", $time) ;
        $display("ADDRESS : %h, SEL : %b ", write_data`WRITE_ADDRESS, write_data`WRITE_SEL) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous I/O write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    test_ok ;

    test_name = "ERRONEOUS I/O READ TO WB SLAVE" ;

    read_data`READ_ADDRESS  = target_address + 3 ;
    read_data`READ_SEL      = 4'hF ;
    read_data`READ_TAG_STIM = 0 ;

    wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
    if ( (read_status`CYC_ACTUAL_TRANSFER !== 0) || (read_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on erroneous access to IO mapped address! Time %t ", $time) ;
        $display("ADDRESS : %h, SEL : %b ", write_data`WRITE_ADDRESS, write_data`WRITE_SEL) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous I/O read as expected") ;
        disable no_transaction ;
        disable main ;
    end

    test_ok ;

    test_name = "CAB I/O WRITE TO WB SLAVE" ;
    // prepare write data
    for ( i = 0 ; i < 6 ; i = i + 1 )
    begin
        write_data`WRITE_DATA    = wmem_data[i] ;
        write_data`WRITE_ADDRESS = target_address + 4*i ;
        write_data`WRITE_SEL     = 4'hF ;
        wishbone_master.blk_write_data[i] = write_data ;
    end

    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
    write_flags`WB_TRANSFER_CAB    = 1 ;
    write_flags`WB_TRANSFER_SIZE   = 6 ;

    wishbone_master.wb_block_write(write_flags, write_status) ;

    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on CAB access to IO mapped address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject CAB I/O write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    test_ok ;

    test_name = "CAB I/O READ TO WB SLAVE" ;
    // prepare read data
    for ( i = 0 ; i < 6 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + 4*i ;
        read_data`READ_SEL     = 4'hF ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    wishbone_master.wb_block_read(write_flags, read_status) ;

    if ( (read_status`CYC_ACTUAL_TRANSFER !== 0) || (read_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on CAB access to IO mapped address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject CAB I/O read as expected") ;
        disable no_transaction ;
        disable main ;
    end

    test_ok ;

    $display("**************************** DONE testing WISHBONE Slave's response to erroneous IO accesses ***************************") ;

    $display("************************* Testing WISHBONE Slave's response to erroneous configuration accesses **************************") ;

    target_address = {`WB_CONFIGURATION_BASE, 12'h0} ;
    write_data`WRITE_ADDRESS = target_address + 1 ;
    write_data`WRITE_DATA    = wmem_data[0] ;
    write_data`WRITE_SEL     = 4'hF ;

    // don't handle retries
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    `ifdef HOST
        `define DO_W_CONF_TEST
        `define DO_R_CONF_TEST
    `else
        `ifdef WB_CNF_IMAGE
             `define DO_R_CONF_TEST
        `endif
    `endif

    `ifdef DO_W_CONF_TEST
    test_name = "ERRONEOUS WB CONFIGURATION WRITE ACCESS" ;
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on access to non-alligned configuration address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous Configuration write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    write_data`WRITE_ADDRESS = target_address + 2 ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on access to non-alligned configuration address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous Configuration write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    write_data`WRITE_ADDRESS = target_address + 3 ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on access to non-alligned configuration address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous Configuration write as expected") ;
        disable no_transaction ;
        disable main ;
    end

    test_ok ;
    `endif

    `ifdef DO_R_CONF_TEST
    test_name = "ERRONEOUS WB CONFIGURATION READ ACCESS" ;
    read_data`READ_ADDRESS  = target_address + 3 ;
    read_data`READ_SEL      = 4'hF ;
    read_data`READ_TAG_STIM = 0 ;

    wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
    if ( (read_status`CYC_ACTUAL_TRANSFER !== 0) || (read_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on access to non-alligned configuration address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject erroneous Configuration read as expected") ;
        disable no_transaction ;
        disable main ;
    end

    test_ok ;
    `endif

    `ifdef DO_W_CONF_TEST
    // prepare write data
    test_name = "WB CAB CONFIGURATION WRITE ACCESS" ;
    for ( i = 0 ; i < 6 ; i = i + 1 )
    begin
        write_data`WRITE_DATA    = wmem_data[i] ;
        write_data`WRITE_ADDRESS = target_address + 4*i ;
        write_data`WRITE_SEL     = 4'hF ;
        wishbone_master.blk_write_data[i] = write_data ;
    end

    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
    write_flags`WB_TRANSFER_CAB    = 1 ;
    write_flags`WB_TRANSFER_SIZE   = 6 ;

    wishbone_master.wb_block_write(write_flags, write_status) ;

    if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on CAB access to configuration address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject CAB Configuration write as expected") ;
        disable no_transaction ;
        disable main ;
    end
    test_ok ;
    `endif

    `ifdef DO_R_CONF_TEST
    // prepare read data
    test_name = "WB CAB CONFIGURATION READ ACCESS" ;
    for ( i = 0 ; i < 6 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + 4*i ;
        read_data`READ_SEL     = 4'hF ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    wishbone_master.wb_block_read(write_flags, read_status) ;

    if ( (read_status`CYC_ACTUAL_TRANSFER !== 0) || (read_status`CYC_ERR !== 1) )
    begin
        $display("WISHBONE slave error termination testing failed! WB Slave didn't signal error on CAB access to configuration address! Time %t ", $time) ;
        $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
        test_fail("WB Slave state machine didn't reject CAB Configuration read as expected") ;
        disable no_transaction ;
        disable main ;
    end
    test_ok ;
    `endif

    `ifdef GUEST
        skip = 1 ;
    `endif

    // disable image
    test_name = "DISABLE IMAGE" ;
    config_write( am_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("WISHBONE slave error termination testing failed! Failed to write W_AM1 register! Time %t ", $time) ;
        test_fail("WB Address Mask register couldn't be written") ;
        disable no_transaction ;
        disable main ;
    end

    skip = 0 ;

    $display("*********************** DONE testing WISHBONE Slave's response to erroneous configuration accesses ***********************") ;

    disable no_transaction ;
end
begin:no_transaction
    // this block of statements monitors PCI bus for initiated transaction - no transaction can be initiated on PCI bus by PCI bridge,
    // since all WISHBONE transactions in this test should be terminated with error on WISHBONE and should not proceede to PCI bus
    forever
    begin
        @(posedge pci_clock) ;
        if ( skip !== 1 )
        begin
            if ( FRAME !== 1 )
            begin
                $display("WISHBONE slave error termination testing failed! Transaction terminated with error on WISHBONE should not proceede to PCI bus! Time %t ", $time) ;
                $display("Address = %h, Bus command = %b ", AD, CBE) ;
                test_fail("access that should be terminated with error by WB Slave state machine proceeded to PCI bus") ;
            end
        end
    end
end
join
endtask //wb_slave_errors

task wb_to_pci_error_handling ;
    reg   [11:0] ctrl_offset ;
    reg   [11:0] ba_offset ;
    reg   [11:0] am_offset ;
    reg   [11:0] ta_offset ;
    reg   [11:0] err_cs_offset ;
    reg `WRITE_STIM_TYPE write_data ;
    reg `READ_STIM_TYPE  read_data ;
    reg `READ_RETURN_TYPE read_status ;

    reg `WRITE_RETURN_TYPE write_status ;
    reg `WB_TRANSFER_FLAGS write_flags ;
    reg [31:0] temp_val1 ;
    reg [31:0] temp_val2 ;
    reg        ok   ;
    reg [11:0] pci_ctrl_offset ;
    reg [31:0] image_base ;
    reg [31:0] target_address ;
    integer    num_of_trans ;
    integer    current ;
    integer    i ;
begin:main

    $display("************************** Testing handling of PCI bus errors *******************************************") ;

    pci_ctrl_offset = 12'h4 ;

    // disable error interrupts and disable error reporting
    test_name = "CONFIGURING BRIDGE FOR PCI ERROR TERMINATION RESPONSE BY WB SLAVE TESTING" ;
    config_write( {4'h1, `ICR_ADDR, 2'b00}, 32'h0000_0000, 4'h1, ok) ;
    if ( ok !== 1 )
    begin
        $display("PCI bus error handling testing failed! Failed to write ICR register! Time %t ", $time) ;
        test_fail("PCI Device Control register couldn't be written") ;
        disable main ;
    end

    // image 1 is used for error testing, since it is always implemented
    ctrl_offset = {4'h1, `W_IMG_CTRL1_ADDR, 2'b00} ;
    ba_offset   = {4'h1, `W_BA1_ADDR, 2'b00} ;
    am_offset   = {4'h1, `W_AM1_ADDR, 2'b00} ;
    ta_offset   = {4'h1, `W_TA1_ADDR, 2'b00} ;

    // set master abort testing address to address that goes out of target's range
    target_address  = `BEH_TAR1_MEM_START ;
    image_base      = 0 ;
    image_base[`PCI_BASE_ADDR0_MATCH_RANGE] = target_address[`PCI_BASE_ADDR0_MATCH_RANGE] ;
    target_address[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] = image_base[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] ;
    target_address[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0]  = image_base[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] ;

    write_flags                    = 0 ;
    write_flags`INIT_WAITS         = tb_init_waits ;
    write_flags`SUBSEQ_WAITS       = tb_subseq_waits ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    err_cs_offset = {4'h1, `W_ERR_CS_ADDR, 2'b00} ;

    // enable master & target operation
    config_write( pci_ctrl_offset, 32'h0000_0007, 4'h1, ok) ;
    if ( ok !== 1 )
    begin
        $display("PCI bus error handling testing failed! Failed to write PCI Device Control register! Time %t ", $time) ;
        test_fail("PCI Device Control register couldn't be written") ;
        disable main ;
    end

    // prepare image control register
    config_write( ctrl_offset, 32'h0000_0002, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("PCI bus error handling testing failed! Failed to write W_IMG_CTRL1 register! Time %t ", $time) ;
        test_fail("WB Image Control register couldn't be written") ;
        disable main ;
    end

    // prepare base address register
    config_write( ba_offset, image_base, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("PCI bus error handling testing failed! Failed to write W_BA1 register! Time %t ", $time) ;
        test_fail("WB Image Base Address register couldn't be written") ;
        disable main ;
    end

    // write address mask register
    config_write( am_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("PCI bus error handling testing failed! Failed to write W_AM1 register! Time %t ", $time) ;
        test_fail("WB Image Address Mask register couldn't be written") ;
        disable main ;
    end

    // disable error reporting
    config_write( err_cs_offset, 32'h0000_0000, 4'hF, ok ) ;
    if ( ~ok )
    begin
        $display("PCI bus error handling testing failed! Failed to write W_ERR_CS register! Time %t ", $time) ;
        test_fail("WB Error Control and Status register couldn't be written") ;
        disable main ;
    end

    // perform two writes - one to error address and one to OK address
    // prepare write buffer

    write_data`WRITE_ADDRESS = target_address ;
    write_data`WRITE_DATA    = wmem_data[100] ;
    write_data`WRITE_SEL     = 4'hF ;

    wishbone_master.blk_write_data[0] = write_data ;

    write_flags`WB_TRANSFER_SIZE = 2 ;

    // don't handle retries
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
    write_flags`WB_TRANSFER_CAB    = 0 ;

    $display("Introducing master abort error on single WB to PCI write!") ;

    test_name = "MASTER ABORT ERROR HANDLING DURING WB TO PCI WRITES" ;
    // first disable target 1

    configuration_cycle_write(0,             // bus number
                              1,             // device number
                              0,             // function number
                              1,             // register number
                              0,             // type of configuration cycle
                              4'b0001,       // byte enables
                              32'h0000_0000  // data
                             ) ;

    fork
    begin
        // start no response monitor in parallel with writes
        musnt_respond(ok) ;
        if ( ok !== 1 )
        begin
            $display("PCI bus error handling testing failed! Test of master abort handling got one target to respond! Time %t ", $time) ;
            $display("Testbench is configured wrong!") ;
            test_fail("transaction wasn't initiated by PCI Master state machine or Target responded and Master Abort didn't occur");
        end
        else
            test_ok ;
    end
    begin
       wishbone_master.wb_single_write(write_data, write_flags, write_status) ;
       if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
       begin
           $display("PCI bus error handling testing failed! WB slave didn't acknowledge single write cycle! Time %t ", $time) ;
           $display("WISHBONE slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
           test_fail("WB Slave state machine failed to post single memory write");
           disable main ;
       end
    end
    join

    /*// read data from second write
    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    read_data`READ_ADDRESS = target_address ;
    read_data`READ_SEL     = 4'hF ;
    read_data`READ_TAG_STIM = 0 ;

    wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;

    if ( read_status`READ_DATA !== wmem_data[101] )
    begin
        display_warning( target_address, wmem_data[101], read_status`READ_DATA ) ;
    end
    */

    // read error status register - no errors should be reported since reporting was disabled
    test_name = "CHECKING ERROR REPORTING FUNCTIONS AFTER MASTER ABORT ERROR" ;

    @(posedge pci_clock) ;
    // wait for two WB clocks for synchronization to be finished
    repeat (2)
        @(posedge wb_clock) ;

    config_read( err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error reporting was disabled, but error was reported anyway!") ;
        test_fail("Error reporting was disabled, but error was reported anyway") ;
        disable main ;
    end
    test_ok ;

    test_name = "CHECKING INTERRUPT REQUESTS AFTER MASTER ABORT ERROR" ;
    // check for interrupts - there should be no interrupt requests active
    `ifdef HOST
        repeat(4)
            @(posedge wb_clock) ;

        if ( INT_O !== 0 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("WISHBONE error interrupt enable is cleared, error signal bit is cleared, but interrupt was signalled on WISHBONE bus!") ;
            test_fail("WISHBONE error interrupts were disabled, error signal bit is clear, but interrupt was signalled on WISHBONE bus") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat( 4 )
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("WISHBONE error interrupt enable is cleared, error signal bit is cleared, but interrupt was signalled on PCI bus!") ;
            test_fail("WISHBONE error interrupts were disabled, error signal bit is clear, but interrupt was signalled on PCI bus") ;
        end
        else
            test_ok ;
    `endif
    `endif

    test_name = "CHECKING PCI DEVICE STATUS REGISTER VALUE AFTER MASTER ABORT" ;
    // check PCI status register
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[29] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Master Abort bit was not set when write was terminated with Master Abort!") ;
        test_fail("Received Master Abort bit was not set when write was terminated with Master Abort") ;
    end
    else
        test_ok ;

    // clear
    config_write( pci_ctrl_offset, temp_val1, 4'b1100, ok ) ;

    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    $display("Introducing master abort error to CAB write!") ;
    // now enable error reporting mechanism
    config_write( err_cs_offset, 1, 4'h1, ok ) ;
    // enable error interrupts
    config_write( {4'h1, `ICR_ADDR, 2'b00}, 32'h0000_0002, 4'h1, ok) ;

    // configure flags for CAB transfer
    write_flags`WB_TRANSFER_CAB = 1 ;
    write_flags`WB_TRANSFER_SIZE = 3 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    // prepare data for erroneous write
    for ( i = 0 ; i < 3 ; i = i + 1 )
    begin
        write_data`WRITE_ADDRESS = target_address + 4*i ;
        write_data`WRITE_DATA    = wmem_data[110 + i] ;
        write_data`WRITE_SEL     = 4'hF ;
        wishbone_master.blk_write_data[i] = write_data ;
    end

    test_name = "CHECKING MASTER ABORT ERROR HANDLING ON CAB MEMORY WRITE" ;
    fork
    begin
        wishbone_master.wb_block_write(write_flags, write_status) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 3 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("Complete burst write through WB slave didn't succeed!") ;
            test_fail("WB Slave state machine failed to post CAB Memory write") ;
            disable main ;
        end
    end
    begin
        musnt_respond(ok) ;
        if ( ok !== 1 )
        begin
            $display("PCI bus error handling testing failed! Test of master abort handling got one target to respond! Time %t ", $time) ;
            $display("Testbench is configured wrong!") ;
            test_fail("transaction wasn't initiated by PCI Master state machine or Target responded and Master Abort didn't occur");
        end
        else
            test_ok ;
    end
    join

    // check error status address, data, byte enables and bus command
    // error status bit is signalled on PCI clock and synchronized to WB clock
    // wait one PCI clock cycle
    test_name = "CHECKING ERROR REPORTING REGISTERS' VALUES AFTER MASTER ABORT ERROR" ;
    ok = 1 ;
    @(posedge pci_clock) ;

    // wait for two WB clocks for synchronization to be finished
    repeat (2)
        @(posedge wb_clock) ;

    // read registers
    config_read( err_cs_offset, 4'h2, temp_val1 ) ;
    if ( temp_val1[8] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Error reporting was enabled, but error was not reported! Time %t ", $time) ;
        test_fail("Error reporting was enabled, but error wasn't reported") ;
        ok = 0 ;
    end

    if ( temp_val1[9] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Error source bit value incorrect! Time %t ", $time) ;
        $display("Error source bit should be 1 to indicate master is reporting error!") ;
        test_fail("Error source bit should be 1 to indicate master is reporting error after Master Abort") ;
        ok = 0 ;
    end

    if ( temp_val1[31:28] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Byte enable field in W_ERR_CS bit has incorrect value! Time %t ", $time) ;
        $display("Expected value %b, actualy read value %b !", 4'h0, temp_val1[31:28]) ;
        test_fail("BE field in WB Error Control and Status register was wrong after Master Abort") ;
        ok = 0 ;
    end

    if ( temp_val1[27:24] !== `BC_MEM_WRITE )
    begin
        $display("PCI bus error handling testing failed! Bus command field in W_ERR_CS bit has incorrect value! Time %t ", $time) ;
        $display("Expected value %b, actualy read value %b !", `BC_MEM_WRITE, temp_val1[27:24]) ;
        test_fail("BC field in WB Error Control and Status register was wrong after Master Abort") ;
        ok = 0 ;
    end

    // read error address register
    config_read( {4'h1, `W_ERR_ADDR_ADDR, 2'b00}, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== target_address )
    begin
        $display("PCI bus error handling testing failed! Erroneous address register has incorrect value! Time %t ", $time) ;
        $display("Expected value %h, actualy read value %h !", target_address, temp_val1) ;
        test_fail("address provided in WB Erroneous Address register was wrong after Master Abort") ;
        ok = 0 ;
    end

    config_read( {4'h1, `W_ERR_DATA_ADDR, 2'b00}, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== wmem_data[110] )
    begin
        $display("PCI bus error handling testing failed! Erroneous data register has incorrect value! Time %t ", $time) ;
        $display("Expected value %h, actualy read value %h !", wmem_data[110], temp_val1) ;
        test_fail("data provided in WB Erroneous Data register was wrong after Master Abort") ;
        ok = 0 ;
    end

    // check PCI status register
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[29] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Master Abort bit was not set when write was terminated with Master Abort!") ;
        test_fail("Received Master Abort bit in PCI Device Status register was not set when write was terminated with Master Abort") ;
        ok = 0 ;
    end

    if ( temp_val1[28] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Target Abort bit was set for no reason!") ;
        test_fail("Received Target Abort bit was set for no reason") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    // clear error status bit
    config_write( err_cs_offset, 32'h0000_0100, 4'h2, ok ) ;

    test_name = "CHECKING INTERRUPT REQUESTS AFTER MASTER ABORT ERROR" ;

    ok = 1 ;

    `ifdef HOST
        repeat(4)
        @(posedge wb_clock) ;
        if ( INT_O !== 1 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("WISHBONE error interrupt enable is set, error was signalled, but interrupt request was not presented on WISHBONE bus!") ;
            test_fail("WISHBONE error interrupt enable was set, error was signalled, but interrupt request was not presented on WISHBONE bus") ;
            ok = 0 ;
        end
    `else
    `ifdef GUEST
        repeat(4)
        @(posedge pci_clock) ;
        if ( INTA !== 0 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("WISHBONE error interrupt enable is set, error was signaled, but interrupt request was not presented on PCI bus!") ;
            test_fail("WISHBONE error interrupt enable was set, error was signalled, but interrupt request was not presented on PCI bus") ;
            ok = 0 ;
        end
    `endif
    `endif

    // read interrupt status register
    config_read( {4'h1, `ISR_ADDR, 2'b00}, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== 32'h0000_0002 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("WISHBONE error interrupt enable is set, error was signaled, but interrupt status register has wrong value!") ;
        $display("Expected ISR = %h, actual ISR = %h ", 32'h0000_0002, temp_val1) ;
        test_fail("Interrupt Status register returned wrong value") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;
    // clear interrupt status bits
    config_write( {4'h1, `ISR_ADDR, 2'b00}, temp_val1, 4'hF, ok ) ;

    ok = 1 ;
    test_name = "CHECKING INTERRUPT REQUESTS AFTER CLEARING THEM" ;
    // wait for two clock cycles before checking interrupt request deassertion
    `ifdef HOST
        repeat (4)
            @(posedge wb_clock) ;

        if ( INT_O !== 0 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("WISHBONE interrupt request still presented, even when interrupt statuses were cleared!") ;
            test_fail("WISHBONE interrupt request was still asserted, even when interrupt statuses were cleared!") ;
            ok = 0 ;
        end
    `else
    `ifdef GUEST
        repeat (4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("PCI interrupt request still presented, even when interrupt statuses were cleared!") ;
            test_fail("PCI interrupt request was still asserted, even when interrupt statuses were cleared!") ;
            ok = 0 ;
        end
    `endif
    `endif

    if ( ok )
        test_ok ;

    test_name = "CHECK NORMAL WRITING/READING FROM WISHBONE TO PCI AFTER ERRORS WERE PRESENTED" ;
    ok = 1 ;
    // enable target
    configuration_cycle_write(0,             // bus number
                              1,             // device number
                              0,             // function number
                              1,             // register number
                              0,             // type of configuration cycle
                              4'b0001,       // byte enables
                              32'h0000_0007  // data
                             ) ;
    // prepare data for ok write
    for ( i = 0 ; i < 3 ; i = i + 1 )
    begin
        write_data`WRITE_ADDRESS = target_address + 4*i ;
        write_data`WRITE_DATA    = wmem_data[113 + i] ;
        write_data`WRITE_SEL     = 4'hF ;
        wishbone_master.blk_write_data[i] = write_data ;
    end

    wishbone_master.wb_block_write(write_flags, write_status) ;

    if ( write_status`CYC_ACTUAL_TRANSFER !== 3 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Complete burst write through WB slave didn't succeed!") ;
        test_fail("WB Slave state machine failed to post CAB write") ;
        disable main ;
    end

    // do a read
    for ( i = 0 ; i < 3 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + 4*i ;
        read_data`READ_SEL     = 4'hF ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_SIZE   = 3 ;
    write_flags`WB_TRANSFER_CAB    = 1 ;

    wishbone_master.wb_block_read( write_flags, read_status ) ;

    if ( read_status`CYC_ACTUAL_TRANSFER !== 3 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Complete burst read through WB slave didn't succeed!") ;
        test_fail("Delayed CAB write was not processed as expected") ;
        disable main ;
    end

    for ( i = 0 ; i < 3 ; i = i + 1 )
    begin
        read_status = wishbone_master.blk_read_data_out[i] ;
        if ( read_status`READ_DATA !== wmem_data[113 + i] )
        begin
            display_warning( target_address + 4*i, wmem_data[113 + i], read_status`READ_DATA ) ;
            test_fail ( "data value provided by PCI bridge for normal read was not as expected") ;
        end
    end

    $display("Introducing master abort error to single read!") ;
    // disable target
    configuration_cycle_write(0,             // bus number
                              1,             // device number
                              0,             // function number
                              1,             // register number
                              0,             // type of configuration cycle
                              4'b0001,       // byte enables
                              32'h0000_0000  // data
                             ) ;
    // set read data
    read_data`READ_ADDRESS = target_address ;
    read_data`READ_SEL     = 4'hF ;

    // enable automatic retry handling
    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_CAB    = 0 ;

    test_name = "MASTER ABORT ERROR HANDLING FOR WB TO PCI READS" ;
    fork
    begin
        wishbone_master.wb_single_read(read_data, write_flags, read_status) ;
    end
    begin
        musnt_respond(ok) ;
        if ( ok !== 1 )
        begin
            $display("PCI bus error handling testing failed! Test of master abort handling got one target to respond! Time %t ", $time) ;
            $display("Testbench is configured wrong!") ;
            test_fail("transaction wasn't initiated by PCI Master state machine or Target responded and Master Abort didn't occur");
        end
    end
    join

    if ( (read_status`CYC_ACTUAL_TRANSFER !== 0) || (read_status`CYC_ERR !== 1) )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Read terminated with master abort should return zero data and terminate WISHBONE cycle with error!") ;
        $display("Actuals: Data transfered: %d, slave response: ACK = %b, RTY = %b, ERR = %b ", read_status`CYC_ACTUAL_TRANSFER, read_status`CYC_ACK, read_status`CYC_RTY, read_status`CYC_ERR) ;
        test_fail("read didn't finish on WB bus as expected") ;
        disable main ;
    end

    test_ok ;


    // now check for error statuses - because reads are delayed, nothing of a kind should happen on error
    test_name = "CHECKING ERROR STATUS AFTER MASTER ABORT ON READ" ;
    config_read( err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error reporting mechanism reports errors on read terminated with master abort! This shouldn't happen!") ;
        test_fail("error status bit should not be set after error occures on Delayed Read Transaction") ;
    end
    else
        test_ok ;

    // now check normal read operation
    configuration_cycle_write(0,             // bus number
                              1,             // device number
                              0,             // function number
                              1,             // register number
                              0,             // type of configuration cycle
                              4'b0001,       // byte enables
                              32'h0000_0007  // data
                             ) ;

    test_name = "CHECK NORMAL READ AFTER ERROR TERMINATED READ" ;
    read_data`READ_ADDRESS = target_address ;
    read_data`READ_SEL     = 4'hF ;

    wishbone_master.wb_single_read(read_data, write_flags, read_status) ;
    if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("WB slave failed to process single read!") ;
        $display("Slave response: ACK = %b, RTY = %b, ERR = %b ", read_status`CYC_ACTUAL_TRANSFER, read_status`CYC_ACK, read_status`CYC_RTY, read_status`CYC_ERR) ;
        test_fail("PCI Bridge didn't process single Delayed Read as expected") ;
        disable main ;
    end

    if ( read_status`READ_DATA !== wmem_data[113] )
    begin
        display_warning( target_address, wmem_data[113 + i], read_status`READ_DATA ) ;
        test_fail("when read finished on WB bus, wrong data was provided") ;
    end
    else
        test_ok ;

    // check PCI status register
    test_name = "CHECK PCI DEVICE STATUS REGISTER VALUE AFTER TARGET ABORT ON DELAYED READ" ;
    ok = 1 ;

    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[29] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Master Abort bit was not set when read was terminated with Master Abort!") ;
        test_fail("Received Master Abort bit was not set when read was terminated with Master Abort") ;
        ok = 0 ;
    end

    if ( temp_val1[28] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Target Abort bit was set for no reason!") ;
        test_fail("Received Target Abort bit was set for no reason") ;
        ok = 0 ;
    end
    if ( ok )
        test_ok ;

    config_write( pci_ctrl_offset, temp_val1, 4'b1100, ok ) ;

    $display("Introducing master abort error to CAB read!") ;
    test_name = "MASTER ABORT ERROR DURING CAB READ FROM WB TO PCI" ;

    configuration_cycle_write(0,             // bus number
                              1,             // device number
                              0,             // function number
                              1,             // register number
                              0,             // type of configuration cycle
                              4'b0001,       // byte enables
                              32'h0000_0000  // data
                             ) ;

    for ( i = 0 ; i < 3 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + 4*i ;
        read_data`READ_SEL     = 4'hF ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_SIZE   = 3 ;
    write_flags`WB_TRANSFER_CAB    = 1 ;

    fork
    begin
        wishbone_master.wb_block_read( write_flags, read_status ) ;
    end
    begin
        musnt_respond(ok) ;
        if ( ok !== 1 )
        begin
            $display("PCI bus error handling testing failed! Test of master abort handling got one target to respond! Time %t ", $time) ;
            $display("Testbench is configured wrong!") ;
            test_fail("transaction wasn't initiated by PCI Master state machine or Target responded and Master Abort didn't occur");
        end
    end
    join

    if ( (read_status`CYC_ACTUAL_TRANSFER !== 0) || (read_status`CYC_ERR !== 1) )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Read terminated with master abort should return zero data and terminate WISHBONE cycle with error!") ;
        $display("Actuals: Data transfered: %d, slave response: ACK = %b, RTY = %b, ERR = %b ", read_status`CYC_ACTUAL_TRANSFER, read_status`CYC_ACK, read_status`CYC_RTY, read_status`CYC_ERR) ;
        test_fail("Read terminated with Master Abort didn't return zero data or terminate WISHBONE cycle with error") ;
        disable main ;
    end
    else
        test_ok ;

    test_name = "CHECK PCI DEVICE STATUS REGISTER AFTER READ TERMINATED WITH MASTER ABORT" ;
    ok = 1 ;
    // check PCI status register
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[29] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Master Abort bit was not set when read was terminated with Master Abort!") ;
        test_fail("Received Master Abort bit was not set when read was terminated with Master Abort") ;
        ok = 0 ;
    end

    if ( temp_val1[28] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Target Abort bit was set for no reason!") ;
        test_fail("Received Target Abort bit was set for no reason") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    config_write( pci_ctrl_offset, temp_val1, 4'b1100, ok ) ;

    $display("Introducing target abort termination to single write!") ;

    // disable error reporting and interrupts
    test_name = "SETUP BRIDGE FOR TARGET ABORT HANDLING TESTS" ;

    configuration_cycle_write(0,             // bus number
                              1,             // device number
                              0,             // function number
                              1,             // register number
                              0,             // type of configuration cycle
                              4'b0001,       // byte enables
                              32'h0000_0007  // data
                             ) ;

    config_write( err_cs_offset, 32'h0000_0000, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("PCI bus error handling testing failed! Failed to write W_ERR_CS register! Time %t ", $time) ;
        test_fail("WB Error Control and Status register couldn't be written to") ;
        disable main ;
    end

    config_write( {4'h1, `ICR_ADDR, 2'b00}, 32'h0000_0000, 4'h1, ok) ;
    if ( ok !== 1 )
    begin
        $display("PCI bus error handling testing failed! Failed to write ICR register! Time %t ", $time) ;
        test_fail("Interrupt Control register couldn't be written to") ;
        disable main ;
    end

    test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Abort ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 1 ;

    write_data`WRITE_ADDRESS = target_address ;
    write_data`WRITE_DATA    = wmem_data[0] ;
    write_data`WRITE_SEL     = 4'hF ;

    wishbone_master.blk_write_data[0] = write_data ;

    write_data`WRITE_ADDRESS = target_address + 4;
    write_data`WRITE_DATA    = wmem_data[1] ;
    write_data`WRITE_SEL     = 4'hF ;

    wishbone_master.blk_write_data[1] = write_data ;

    write_flags`WB_TRANSFER_SIZE = 2 ;

    // don't handle retries
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
    write_flags`WB_TRANSFER_CAB    = 0 ;

    test_name = "TARGET ABORT ERROR ON SINGLE WRITE" ;
    fork
    begin
        wishbone_master.wb_block_write(write_flags, write_status) ;

        if ( write_status`CYC_ACTUAL_TRANSFER !== 2 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("Image writes were not accepted as expected!") ;
            $display("Slave response: ACK = %b, RTY = %b, ERR = %b ", write_status`CYC_ACTUAL_TRANSFER, write_status`CYC_ACK, write_status`CYC_RTY, write_status`CYC_ERR) ;
            test_fail("WB Slave state machine failed to post two single memory writes")  ;
            disable main ;
        end

        // read data back to see, if it was written OK
        read_data`READ_ADDRESS         = target_address + 4;
        read_data`READ_SEL             = 4'hF ;
        write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
        begin
            test_fail("unexpected transaction or no response detected on PCI bus, when Target Abort during Memory Write was expected") ;
        end
        else
            test_ok ;

        test_name = "NORMAL SINGLE MEMORY WRITE IMMEDIATELY AFTER ONE TERMINATED WITH TARGET ABORT" ;

        // when first transaction finishes - enable normal target response!
        test_target_response[`TARGET_ENCODED_TERMINATION] = `Test_Target_Normal_Completion ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 0 ;

        pci_transaction_progress_monitor( target_address + 4, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
        begin
            test_fail("unexpected transaction or no response detected on PCI bus, when single Memory Write was expected") ;
        end
        else
            test_ok ;

        test_name = "NORMAL SINGLE MEMORY READ AFTER WRITE TERMINATED WITH TARGET ABORT" ;
        pci_transaction_progress_monitor( target_address + 4, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
        begin
            test_fail("unexpected transaction or no response detected on PCI bus, when single Memory Read was expected") ;
        end
    end
    join

    if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Bridge failed to process single read after target abort terminated write!") ;
        test_fail("bridge failed to process single delayed read after target abort terminated write") ;
        disable main ;
    end

    if ( read_status`READ_DATA !== wmem_data[1] )
    begin
        display_warning( target_address + 4, wmem_data[1], read_status`READ_DATA ) ;
        test_fail("bridge returned unexpected data on read following Target Abort Terminated write") ;
    end
    else
        test_ok ;

    // check interrupt and error statuses!
    test_name = "WB ERROR CONTROL AND STATUS REGISTER VALUE CHECK AFTER WRITE TARGET ABORT" ;
    config_read( err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error bit in error status register was set even though error reporting was disabled!") ;
        test_fail("Error bit in error status register was set even though error reporting was disabled") ;
    end
    else
        test_ok ;

    test_name = "INTERRUPT STATUS REGISTER VALUE CHECK AFTER WRITE TARGET ABORT" ;
    config_read( {4'h1, `ISR_ADDR, 2'b00}, 4'hF, temp_val1) ;
    if ( temp_val1[1] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error interrupt request bit was set after PCI error termination, even though interrupts were disabled!") ;
        test_fail("Error interrupt request bit was set after PCI Target Abort termination, even though interrupts were disabled") ;
    end
    else
        test_ok ;

    // check PCI status register
    test_name = "PCI DEVICE STATUS REGISTER VALUE CHECK AFTER WRITE TARGET ABORT" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[29] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Master Abort bit was set with no reason!") ;
        test_fail("Received Master Abort bit was set with no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[28] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Target Abort bit was not set when write transaction was terminated with target abort!") ;
        test_fail("Received Target Abort bit was not set when write transaction was terminated with target abort") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    config_write( pci_ctrl_offset, temp_val1, 4'b1100, ok ) ;

    test_name = "TARGET ABORT ERROR ON CAB MEMORY WRITE" ;

    $display("Introducing target abort termination to CAB write!") ;
    // enable error reporting mechanism

    config_write( err_cs_offset, 32'h0000_0001, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("PCI bus error handling testing failed! Failed to write W_ERR_CS register! Time %t ", $time) ;
        test_fail("WB Error Control and Status register could not be written to") ;
        disable main ;
    end

    for ( i = 0 ; i < 3 ; i = i + 1 )
    begin
        write_data`WRITE_ADDRESS = target_address + 8 + 4*i ;
        write_data`WRITE_DATA    = wmem_data[120 + i] ;
        write_data`WRITE_SEL     = 4'b1010 ;
        wishbone_master.blk_write_data[i] = write_data ;
    end

    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
    write_flags`WB_TRANSFER_CAB    = 1 ;
    write_flags`WB_TRANSFER_SIZE   = 3 ;

    test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Abort ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 1 ;

    fork
    begin
        wishbone_master.wb_block_write(write_flags, write_status) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 3 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("Bridge failed to process complete CAB write!") ;
            test_fail("bridge failed to post CAB Memory Write") ;
            disable main ;
        end
    end
    begin
        pci_transaction_progress_monitor(target_address + 8, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
            test_fail("unexpected transaction or no response detected on PCI bus, when Target Aborted Memory Write was expected") ;
        else
            test_ok ;
    end
    join

    // check statuses and data from error
    test_name = "WB ERROR CONTROL AND STATUS REGISTER VALUE CHECK AFTER WRITE TARGET ABORT" ;
    ok = 1 ;
    config_read( err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error bit was not set even though write was terminated with Target Abort and error reporting was enabled!") ;
        test_fail("Error Signaled bit was not set even though write was terminated with Target Abort and error reporting was enabled") ;
        ok = 0 ;
    end

    if ( temp_val1[9] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error source bit was not 0 to indicate target signalled the error!") ;
        test_fail("Error source bit was not 0 to indicate target signalled the error") ;
        ok = 0 ;
    end

    if ( temp_val1[31:24] !== {4'b0101, `BC_MEM_WRITE} )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Value in W_ERR_CS register was wrong!") ;
        $display("Expected BE = %b, BC = %b ; actuals: BE = %b, BC = %b ", 4'b0101, `BC_MEM_WRITE, temp_val1[31:28], temp_val1[27:24]) ;
        test_fail("BE Field didn't provided expected value") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    test_name = "WB ERRONEOUS ADDRESS AND DATA REGISTERS' VALUES CHECK AFTER WRITE TARGET ABORT" ;
    ok = 1 ;
    // check erroneous address and data
    config_read( { 4'h1, `W_ERR_ADDR_ADDR, 2'b00}, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== (target_address + 8) )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Value in W_ERR_ADDR register was wrong!") ;
        $display("Expected value = %h, actual value = %h " , target_address + 8, temp_val1 ) ;
        test_fail("Value in WB Erroneous Address register was wrong") ;
        ok = 0 ;
    end

    config_read( { 4'h1, `W_ERR_DATA_ADDR, 2'b00}, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== wmem_data[120] )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Value in W_ERR_DATA register was wrong!") ;
        $display("Expected value = %h, actual value = %h " , wmem_data[120], temp_val1 ) ;
        test_fail("Value in WB Erroneous Data register was wrong") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    test_name = "PCI DEVICE STATUS VALUE CHECK AFTER WRITE TARGET ABORT" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[29] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Master Abort bit was set with no reason!") ;
        test_fail("Received Master Abort bit was set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[28] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Target Abort bit was not set when write transaction was terminated with target abort!") ;
        test_fail("Received Target Abort bit was not set when write transaction was terminated with target abort") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    config_write( pci_ctrl_offset, temp_val1, 4'b1100, ok ) ;

    // clear error status bit and enable error interrupts
    config_write( err_cs_offset, 32'h0000_0101, 4'b0011, ok ) ;
    config_write( {4'h1, `ICR_ADDR, 2'b00}, 32'h0000_0002, 4'h1, ok) ;

    // check if error bit was cleared
    test_name = "WB ERROR CONTROL AND STATUS REGISTER VALUE CHECK AFTER CLEARING ERROR STATUS" ;
    config_read( err_cs_offset, 4'b0011, temp_val1 ) ;
    if ( temp_val1[8] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error bit was not cleared even though one was written to its location!") ;
        test_fail("Error bit was not cleared even though one was written to its location") ;
    end

    // repeat same write with different target configuration
    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Abort ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 2 ;

    test_name = "TARGET ABORT TERMINATION ON SECOND DATA PHASE OF BURST WRITE" ;
    fork
    begin
        write_flags`WB_TRANSFER_SIZE = 2 ;
        wishbone_master.wb_block_write(write_flags, write_status) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 2 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("Bridge failed to process complete CAB write!") ;
            test_fail("bridge failed to post CAB Memory Write") ;
            disable main ;
        end

        write_flags`WB_TRANSFER_SIZE = 3 ;
        wishbone_master.wb_block_write(write_flags, write_status) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 3 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("Bridge failed to process complete CAB write!") ;
            test_fail("bridge failed to post CAB Memory Write") ;
            disable main ;
        end
    end
    begin
        pci_transaction_progress_monitor(target_address + 8, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
            test_fail("unexpected transaction or no response detected on PCI bus, when Target Aborted Memory Write was expected") ;
        else
        begin
            test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Normal_Completion ;
            test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 0 ;
            pci_transaction_progress_monitor(target_address + 8, `BC_MEM_WRITE, 3, 0, 1'b1, 1'b0, 0, ok) ;
            if ( ok !== 1 )
                test_fail("unexpected transaction or no response detected on PCI bus, when Normal Memory Write was posted immediately after Target Aborted Memory write") ;
            else
                test_ok ;
        end
    end
    join

    test_name = "WB ERROR CONTROL AND STATUS REGISTER VALUE CHECK AFTER WRITE TARGET ABORT" ;
    ok = 1 ;
    // check statuses and data from error
    config_read( err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error bit was not set even though write was terminated with Target Abort and error reporting was enabled!") ;
        test_fail("Error bit was not set even though write was terminated with Target Abort and error reporting was enabled") ;
        ok = 0 ;
    end

    if ( temp_val1[9] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error source bit was not 0 to indicate target signalled the error!") ;
        test_fail("Error source bit was not 0 to indicate target signalled the error") ;
        ok = 0 ;
    end

    if ( temp_val1[31:24] !== {4'b0101, `BC_MEM_WRITE} )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Value in W_ERR_CS register was wrong!") ;
        $display("Expected BE = %b, BC = %b ; actuals: BE = %b, BC = %b ", 4'b1010, `BC_MEM_WRITE, temp_val1[31:28], temp_val1[27:24]) ;
        test_fail("BE or bus command field(s) didn't provide expected value") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // check erroneous address and data
    config_read( { 4'h1, `W_ERR_ADDR_ADDR, 2'b00}, 4'hF, temp_val1 ) ;
    test_name = "WB ERRONEOUS ADDRESS AND DATA REGISTERS' VALUES CHECK AFTER WRITE TARGET ABORT" ;
    ok = 1 ;
    if ( temp_val1 !== (target_address + 8 + 4) )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Value in W_ERR_ADDR register was wrong!") ;
        $display("Expected value = %h, actual value = %h " , target_address + 8 + 4, temp_val1 ) ;
        test_fail("Value in WB Erroneous Address register was wrong") ;
        ok = 0 ;

    end

    config_read( { 4'h1, `W_ERR_DATA_ADDR, 2'b00}, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== wmem_data[121] )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Value in W_ERR_DATA register was wrong!") ;
        $display("Expected value = %h, actual value = %h " , wmem_data[121], temp_val1 ) ;
        test_fail("Value in WB Erroneous Data register was wrong") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    test_name = "INTERRUPT REQUEST ASSERTION AFTER ERROR REPORTING TRIGGER" ;
    `ifdef HOST
        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O !== 1 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("WISHBONE error interrupt enable is set, error was signalled, but interrupt request was not presented on WISHBONE bus!") ;
            test_fail("interrupt request was not presented on WISHBONE bus") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge pci_clock) ;
        if ( INTA !== 0 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("WISHBONE error interrupt enable is set, error was signaled, but interrupt request was not presented on PCI bus!") ;
            test_fail("interrupt request was not presented on PCI bus") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // read interrupt status register
    test_name = "INTERRUPT STATUS REGISTER VALUE CHECK AFTER TARGET ABORTED MEMORY WRITE" ;
    config_read( {4'h1, `ISR_ADDR, 2'b00}, 4'hF, temp_val1) ;
    if ( temp_val1[1] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("WISHBONE error interrupt enable is set, error was signalled, but corresponding interrupt status bit was not set in ISR!") ;
        test_fail("Expected Interrupt status bit wasn't set") ;
    end

    config_write( {4'h1, `ISR_ADDR, 2'b00}, temp_val1, 4'hF, ok ) ;

    test_name = "PCI DEVICE STATUS REGISTER VALUE CHECK AFTER TARGET ABORTED MEMORY WRITE" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[29] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Master Abort bit was set with no reason!") ;
        test_fail("Received Master Abort bit was set with no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[28] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Target Abort bit was not set when write transaction was terminated with target abort!") ;
        test_fail("Received Target Abort bit was not set when write transaction was terminated with target abort") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    config_write( pci_ctrl_offset, temp_val1, 4'b1100, ok ) ;

    // clear interrupts and errors
    config_write( err_cs_offset, 32'h0000_0101, 4'b0011, ok ) ;
    repeat( 3 )
        @(posedge pci_clock) ;

    repeat( 2 )
        @(posedge wb_clock) ;

    test_name = "INTERRUPT STATUS REGISTER VALUE CHECK AFTER CLEARING STATUS BITS" ;
    config_read( {4'h1, `ISR_ADDR, 2'b00}, 4'hF, temp_val1) ;
    if ( temp_val1[1] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("WISHBONE error interrupt status remains set in ISR after writing one to its location!") ;
        test_fail("WISHBONE error interrupt status remains set in Interrupt Status register after writing one to its location") ;
    end
    else
        test_ok ;

    test_name = "WB ERROR STATUS REGISTER VALUE CHECK AFTER CLEARING STATUS BIT" ;
    config_read( err_cs_offset, 4'b0011, temp_val1 ) ;
    if ( temp_val1[8] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("WISHBONE error status remains set in W_ERR_CS after writing one to its location!") ;
        test_fail("WISHBONE error status remains set in WB Error Status register after writing one to its location") ;
    end


    $display("Introducing Target Abort error to single read!") ;
    // set read data
    read_data`READ_ADDRESS = target_address + 8 ;
    read_data`READ_SEL     = 4'hF ;

    // enable automatic retry handling
    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_CAB    = 0 ;

    test_name = "TARGET ABORT DURING SINGLE MEMORY READ" ;

    test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Abort ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 1 ;

    wishbone_master.wb_single_read(read_data, write_flags, read_status) ;

    if ( (read_status`CYC_ACTUAL_TRANSFER !== 0) || (read_status`CYC_ERR !== 1) )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Read terminated with Target Abort should return zero data and terminate WISHBONE cycle with error!") ;
        $display("Actuals: Data transfered: %d, slave response: ACK = %b, RTY = %b, ERR = %b ", read_status`CYC_ACTUAL_TRANSFER, read_status`CYC_ACK, read_status`CYC_RTY, read_status`CYC_ERR) ;
        test_fail("single read terminated with Target Abort shouldn't return any data and should terminate WISHBONE cycle with error") ;
        disable main ;
    end
    else
        test_ok ;

    // now check for interrupts or error statuses - because reads are delayed, nothing of a kind should happen on error
    test_name = "WB ERROR STATUS REGISTER VALUE CHECK AFTER READ TERMINATED WITH TARGET ABORT" ;
    config_read( err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error reporting mechanism reports errors on read terminated with Target Abort! This shouldn't happen!") ;
        test_fail("Error reporting mechanism shouldn't report errors on reads terminated with Target Abort") ;
    end
    else
        test_ok ;

    test_name = "PCI DEVICE STATUS REGISTER VALUE CHECK AFTER READ TERMINATED WITH TARGET ABORT" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[29] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Master Abort bit was set with no reason!") ;
        test_fail("Received Master Abort bit was set with no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[28] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Target Abort bit was not set when read transaction was terminated with target abort!") ;
        test_fail("Received Target Abort bit was not set when read transaction was terminated with target abort") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    config_write( pci_ctrl_offset, temp_val1, 4'b1100, ok ) ;

    test_name = "INTERRUPT STATUS REGISTER VALUE CHECK AFTER READ TERMINATED WITH TARGET ABORT" ;
    config_read( {4'h1, `ISR_ADDR, 2'b00}, 4'hF, temp_val1) ;
    if ( temp_val1[1] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("WISHBONE error interrupt status set after read was terminated with an error - this shouldn't happen!") ;
        test_fail("WISHBONE Error Interrupt status shouldn't be set after read terminated with Target Abort") ;
    end
    else
        test_ok ;

    $display("Introducing Target Abort error to CAB read!") ;
    test_name = "TARGET ABORT ERROR DURING SECOND DATAPHASE OF BURST READ" ;

    for ( i = 0 ; i < 4 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + 8 + 4*i ;
        read_data`READ_SEL     = 4'b1010 ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_SIZE   = 2 ;
    write_flags`WB_TRANSFER_CAB    = 1 ;

    test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Abort ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 2 ;

    wishbone_master.wb_block_read( write_flags, read_status ) ;

    if ( (read_status`CYC_ACTUAL_TRANSFER !== 1) || (read_status`CYC_ERR !== 1) )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Read terminated with Target Abort on second phase should return one data and terminate WISHBONE cycle with error on second transfer!") ;
        $display("Actuals: Data transfered: %d, slave response: ACK = %b, RTY = %b, ERR = %b ", read_status`CYC_ACTUAL_TRANSFER, read_status`CYC_ACK, read_status`CYC_RTY, read_status`CYC_ERR) ;
        test_fail("Read terminated with Target Abort on second phase should return one data and terminate WISHBONE cycle with error on second transfer") ;
        disable main ;
    end

    read_status = wishbone_master.blk_read_data_out[0] ;
    temp_val1 = read_status`READ_DATA ;
    temp_val2 = wmem_data[120] ;

    // last write to this address was with only two byte enables - check only those
    if ( (temp_val1[31:24] !== temp_val2[31:24]) || (temp_val1[15:8] !== temp_val2[15:8]) )
    begin
        display_warning( target_address + 8, wmem_data[120], read_status`READ_DATA ) ;
        test_fail("data provided during normaly completed first dataphase didn't have expected value");
    end
    else
        test_ok ;

    test_name = "PCI DEVICE STATUS REGISTER VALUE CHECK AFTER READ TERMINATED WITH TARGET ABORT" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[29] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Master Abort bit was set with no reason!") ;
        test_fail("Received Master Abort bit was set with no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[28] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Target Abort bit was not set when read transaction was terminated with target abort!") ;
        test_fail("Received Target Abort bit was not set when read transaction was terminated with target abort") ;
        ok = 0 ;
    end

    if ( ok )
       test_ok ;

    config_write( pci_ctrl_offset, temp_val1, 4'b1100, ok ) ;

    test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Normal_Completion ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 0 ;

    test_name = "CHECK NORMAL BURST READ AFTER TARGET ABORT TERMINATED BURST READ" ;
    for ( i = 0 ; i < 3 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + 4*i ;
        read_data`READ_SEL     = 4'b1111 ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    write_flags`WB_TRANSFER_SIZE   = 3 ;

    wishbone_master.wb_block_read( write_flags, read_status ) ;

    if ( read_status`CYC_ACTUAL_TRANSFER !== 3 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Complete burst read through WB slave didn't succeed!") ;
        test_fail("bridge didn't process Burst Read in an expected way") ;
        disable main ;
    end
    else
        test_ok ;

    test_name = "TARGET ABORT ERROR DURING LAST DATAPHASE OF BURST READ" ;

    for ( i = 0 ; i < 3 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + 8 + 4*i ;
        read_data`READ_SEL     = 4'b1111 ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_SIZE   = 4 ;
    write_flags`WB_TRANSFER_CAB    = 1 ;

    test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Abort ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 4 ;

    wishbone_master.wb_block_read( write_flags, read_status ) ;

    if ( (read_status`CYC_ACTUAL_TRANSFER !== 3) || (read_status`CYC_ERR !== 1) )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Read terminated with Target Abort on last dataphase should return 3 words and terminate WISHBONE cycle with error on fourth transfer!") ;
        $display("Actuals: Data transfered: %d, slave response: ACK = %b, RTY = %b, ERR = %b ", read_status`CYC_ACTUAL_TRANSFER, read_status`CYC_ACK, read_status`CYC_RTY, read_status`CYC_ERR) ;
        test_fail("Read terminated with Target Abort on last dataphase should return three words and terminate WISHBONE cycle with error on fourth transfer") ;
        disable main ;
    end

    for ( i = 0 ; i < 3 ; i = i + 1 )
    begin
        ok = 1 ;
        read_status = wishbone_master.blk_read_data_out[i] ;
        temp_val1 = read_status`READ_DATA ;
        temp_val2 = wmem_data[120 + i] ;

        // last write to this address was with only two byte enables - check only those
        if ( (temp_val1[31:24] !== temp_val2[31:24]) || (temp_val1[15:8] !== temp_val2[15:8]) )
        begin
            display_warning( target_address + 8 + 4*i, wmem_data[120 + i], read_status`READ_DATA ) ;
            test_fail("data provided during normaly completed first dataphase didn't have expected value");
            ok = 0 ;
        end
    end

    if ( ok )
        test_ok ;

    test_name = "PCI DEVICE STATUS REGISTER VALUE CHECK AFTER READ TERMINATED WITH TARGET ABORT" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[29] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Master Abort bit was set with no reason!") ;
        test_fail("Received Master Abort bit was set with no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[28] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Target Abort bit was not set when read transaction was terminated with target abort!") ;
        test_fail("Received Target Abort bit was not set when read transaction was terminated with target abort") ;
        ok = 0 ;
    end

    if ( ok )
       test_ok ;

    config_write( pci_ctrl_offset, temp_val1, 4'b1100, ok ) ;

    test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Normal_Completion ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 0 ;

    test_name = "CHECK NORMAL BURST READ AFTER TARGET ABORT TERMINATED BURST READ" ;
    for ( i = 0 ; i < 3 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + 4*i ;
        read_data`READ_SEL     = 4'b1111 ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    write_flags`WB_TRANSFER_SIZE   = 3 ;

    wishbone_master.wb_block_read( write_flags, read_status ) ;

    if ( read_status`CYC_ACTUAL_TRANSFER !== 3 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Complete burst read through WB slave didn't succeed!") ;
        test_fail("bridge didn't process Burst Read in an expected way") ;
        disable main ;
    end
    else
        test_ok ;

    // test error on IO write
    // change base address
    config_write( ba_offset, image_base + 1, 4'hF, ok ) ;
    write_data`WRITE_SEL     = 4'b0101 ;
    write_data`WRITE_ADDRESS = target_address ;
    write_data`WRITE_DATA    = 32'hAAAA_AAAA ;

    write_flags`WB_TRANSFER_CAB    = 0 ;
    write_flags`WB_TRANSFER_SIZE   = 1 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
    test_name = "ERROR REPORTING FUNCTIONALITY FOR I/O WRITE" ;
    fork
    begin
        wishbone_master.wb_single_write ( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("PCI bus error handling testing failed! Time %t ", $time) ;
            $display("WB slave failed to accept IO write!") ;
            test_fail("WB Slave state machine didn't post I/O write as expected") ;
            disable main ;
        end
    end
    begin
        musnt_respond(ok) ;
        if ( ok !== 1 )
        begin
            $display("PCI bus error handling testing failed! Test of master abort handling got one target to respond! Time %t ", $time) ;
            $display("Testbench is configured wrong!") ;
            test_fail("I/O write didn't start a transaction on PCI or target responded when not expected") ;
        end
        else
            test_ok ;
    end
    join

    // check statuses and everything else
    test_name = "WB ERROR STATUS REGISTER VALUE AFTER MASTER ABORTED I/O WRITE" ;
    ok = 1 ;
    config_read( err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error bit was not set even though write was terminated with Master Abort and error reporting was enabled!") ;
        test_fail("WB Error bit was not set even though write was terminated with Master Abort and error reporting was enabled") ;
        ok = 0 ;
    end

    if ( temp_val1[9] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Error source bit was not 1 to indicate Master signalled the error!") ;
        test_fail("Error source bit was not 1 to indicate Master signalled the error") ;
        ok = 0 ;
    end

    if ( temp_val1[31:24] !== {4'b1010, `BC_IO_WRITE} )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Value in W_ERR_CS register was wrong!") ;
        $display("Expected BE = %b, BC = %b ; actuals: BE = %b, BC = %b ", 4'b1010, `BC_IO_WRITE, temp_val1[31:28], temp_val1[27:24]) ;
        test_fail("values in BE or BC field in WB Error Status register was/were wrong") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // check erroneous address and data
    test_name = "WB ERRONEOUS ADDRESS AND DATA REGISTERS' VALUES AFTER MASTER ABORTED I/O WRITE" ;
    ok = 1 ;
    config_read( { 4'h1, `W_ERR_ADDR_ADDR, 2'b00}, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== target_address )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Value in W_ERR_ADDR register was wrong!") ;
        $display("Expected value = %h, actual value = %h " , target_address, temp_val1 ) ;
        test_fail("WB Erroneous Address register didn't provide right value") ;
        ok = 0 ;
    end

    config_read( { 4'h1, `W_ERR_DATA_ADDR, 2'b00}, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== 32'hAAAA_AAAA )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Value in W_ERR_DATA register was wrong!") ;
        $display("Expected value = %h, actual value = %h " , 32'hAAAA_AAAA, temp_val1 ) ;
        test_fail("WB Erroneous Data register didn't provide right value") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER MASTER ABORTED I/O WRITE" ;
    config_read( {4'h1, `ISR_ADDR, 2'b00}, 4'hF, temp_val1) ;
    if ( temp_val1[1] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("WISHBONE error interrupt enable is set, error was signalled, but corresponding interrupt status bit was not set in ISR!") ;
        test_fail("expected interrupt status bit was not set") ;
    end
    else
        test_ok ;

    // clear interrupts and errors
    config_write( {4'h1, `ISR_ADDR, 2'b00}, temp_val1, 4'hF, ok ) ;
    config_write( err_cs_offset, 32'h0000_0101, 4'b0011, ok ) ;

    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER MASTER ABORTED I/O WRITE" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[29] !== 1 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Master Abort bit was not set when IO write transaction finished with Master Abort!") ;
        test_fail("Received Master Abort bit was not set when IO write transaction finished with Master Abort") ;
        ok = 0 ;
    end

    if ( temp_val1[28] !== 0 )
    begin
        $display("PCI bus error handling testing failed! Time %t ", $time) ;
        $display("Received Target Abort bit was set for no reason!") ;
        test_fail("Received Target Abort bit was set for no reason") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    config_write( pci_ctrl_offset, temp_val1, 4'b1100, ok ) ;

    // disable image
    config_write( am_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("PCI bus error handling testing failed! Failed to write W_AM1 register! Time %t ", $time) ;
        test_fail("WB Image Address Mask register couldn't be written") ;
        disable main ;
    end
    $display("************************ DONE testing handling of PCI bus errors ****************************************") ;

end
endtask

task parity_checking ;
    reg   [11:0] ctrl_offset ;
    reg   [11:0] ba_offset ;
    reg   [11:0] am_offset ;
    reg   [11:0] ta_offset ;
    reg `WRITE_STIM_TYPE write_data ;
    reg `READ_STIM_TYPE  read_data ;
    reg `READ_RETURN_TYPE read_status ;

    reg `WRITE_RETURN_TYPE write_status ;
    reg `WB_TRANSFER_FLAGS write_flags ;
    reg [31:0] temp_val1 ;
    reg [31:0] temp_val2 ;
    reg        ok   ;
    reg [11:0] pci_ctrl_offset ;
    reg [31:0] image_base ;
    reg [31:0] target_address ;
    reg [11:0] icr_offset ;
    reg [11:0] isr_offset ;
    reg [11:0] p_ba_offset ;
    reg [11:0] p_am_offset ;
    reg [11:0] p_ctrl_offset ;
    integer    i ;
    reg        perr_asserted ;
begin:main
    $display("******************************* Testing Parity Checker functions ********************************") ;
    $display("Testing Parity Errors during Master Transactions!") ;
    $display("Introducing Parity Erros to Master Writes!") ;
    $fdisplay(pci_mon_log_file_desc,
    "******************************************** Monitor will complain in following section for a few times - testbench is intentionally causing parity errors *********************************************") ;

    // image 1 is used for error testing, since it is always implemented
    pci_ctrl_offset = 12'h004 ;
    ctrl_offset     = {4'h1, `W_IMG_CTRL1_ADDR, 2'b00} ;
    ba_offset       = {4'h1, `W_BA1_ADDR, 2'b00} ;
    am_offset       = {4'h1, `W_AM1_ADDR, 2'b00} ;
    ta_offset       = {4'h1, `W_TA1_ADDR, 2'b00} ;
    isr_offset      = {4'h1, `ISR_ADDR, 2'b00} ;
    icr_offset      = {4'h1, `ICR_ADDR, 2'b00} ;

    // image 1 for PCI target
    p_ba_offset   = {4'h1, `P_BA1_ADDR, 2'b00} ;
    p_am_offset   = {4'h1, `P_AM1_ADDR, 2'b00} ;
    p_ctrl_offset = {4'h1, `P_IMG_CTRL1_ADDR, 2'b00} ;

    target_address  = `BEH_TAR1_MEM_START ;
    image_base      = 0 ;
    image_base[`PCI_BASE_ADDR0_MATCH_RANGE] = target_address[`PCI_BASE_ADDR0_MATCH_RANGE] ;

    target_address[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] = image_base[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] ;
    target_address[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0]  = image_base[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] ;
    write_flags                    = 0 ;
    write_flags`INIT_WAITS         = tb_init_waits ;
    write_flags`SUBSEQ_WAITS       = tb_subseq_waits ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    // enable master & target operation and disable parity functions
    test_name = "CONFIGURE BRIDGE FOR PARITY CHECKER FUNCTIONS TESTING" ;
    config_write( pci_ctrl_offset, 32'h0000_0007, 4'h3, ok) ;
    if ( ok !== 1 )
    begin
        $display("Parity checker testing failed! Failed to write PCI Device Control register! Time %t ", $time) ;
        test_fail("PCI Device Control register could not be written to") ;
        disable main ;
    end

    // prepare image control register
    config_write( ctrl_offset, 32'h0000_0000, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("Parity checker testing failed! Failed to write W_IMG_CTRL1 register! Time %t ", $time) ;
        test_fail("WB Image Control register could not be written to") ;
        disable main ;
    end

    // prepare base address register
    config_write( ba_offset, image_base, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Parity checker testing failed! Failed to write W_BA1 register! Time %t ", $time) ;
        test_fail("WB Image Base Address register could not be written to") ;
        disable main ;
    end

    // write address mask register
    config_write( am_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Parity checker testing failed! Failed to write W_AM1 register! Time %t ", $time) ;
        test_fail("WB Image Address Mask register could not be written to") ;
        disable main ;
    end

    // disable parity interrupts
    config_write( icr_offset, 0, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Parity checker testing failed! Failed to write ICR! Time %t ", $time) ;
        test_fail("Interrupt Control register could not be written to") ;
        disable main ;
    end

    write_data`WRITE_ADDRESS = target_address ;
    write_data`WRITE_DATA    = wmem_data[0] ;
    write_data`WRITE_SEL     = 4'b1111 ;

    // enable target's 1 response to parity errors
    configuration_cycle_write(0,             // bus number
                              1,             // device number
                              0,             // function number
                              1,             // register number
                              0,             // type of configuration cycle
                              4'b0001,       // byte enables
                              32'h0000_0047  // data
                             ) ;

    // disable target's 2 response to parity errors
    configuration_cycle_write(0,             // bus number
                              2,             // device number
                              0,             // function number
                              1,             // register number
                              0,             // type of configuration cycle
                              4'b0001,       // byte enables
                              32'h0000_0007  // data
                             ) ;

    test_target_response[`TARGET_ENCODED_DATA_PAR_ERR] = 1 ;

    test_name = "RESPONSE TO TARGET ASSERTING PERR DURING MASTER WRITE" ;
    fork
    begin
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Bridge failed to process single memory write!") ;
            test_fail("bridge failed to post single WB memory write") ;
            disable main ;
        end
    end
    begin:wait_perr1
        perr_asserted = 0 ;
        @(posedge pci_clock) ;

        while ( PERR !== 0 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;

    end
    begin
        pci_transaction_progress_monitor(target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok) ;

        if ( ok !== 1 )
            test_fail("bridge failed to process single memory write correctly, or target didn't respond to it") ;

        repeat(2)
            @(posedge pci_clock) ;

        #1 ;
        if ( !perr_asserted )
            disable wait_perr1 ;
    end
    join

    if ( perr_asserted && ok )
    begin
        test_ok ;
    end
    else
    if ( ~perr_asserted )
    begin
        test_fail("PCI behavioral target failed to assert invalid PERR for testing") ;
        disable main ;
    end

    // check all the statuses - if HOST is defined, wait for them to be synced
    `ifdef HOST
    repeat(4)
        @(posedge wb_clock) ;
    `endif

    test_name = "CHECK PCI DEVICE STATUS REGISTER VALUE AFTER PARITY ERROR DURING MASTER WRITE" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set after parity error on PCI bus!") ;
        test_fail("Detected Parity Error bit was not set after Write Master Data Parity Error") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set for no reason!") ;
        test_fail("Signalled System Error bit was set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit set even though Parity Error Response bit was not set!") ;
        test_fail("Master Data Parity Error bit was set even though Parity Error Response was not enabled") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    test_name = "CLEARING PARITY ERROR STATUSES" ;
    // clear parity bits and enable parity response
    config_write( pci_ctrl_offset, temp_val1 | CONFIG_CMD_PAR_ERR_EN, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Parity checker testing failed! Failed to write PCI control and status reg! Time %t ", $time) ;
        test_fail("write to PCI Status Register failed") ;
        disable main ;
    end

    test_name = "RESPONSE TO TARGET ASSERTING PERR DURING MASTER WRITE WITH PARITY ERROR RESPONSE ENABLED" ;
    fork
    begin
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Bridge failed to process single memory write!") ;
            test_fail("bridge failed to post single memory write") ;
            disable main ;
        end
    end
    begin:wait_perr2
        perr_asserted = 0 ;
        @(posedge pci_clock) ;

        while ( PERR !== 0 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;

    end
    begin
        pci_transaction_progress_monitor(target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok) ;

        if ( ok !== 1 )
            test_fail("bridge failed to process single memory write correctly, or target didn't respond to it") ;

        repeat(2)
            @(posedge pci_clock) ;

        #1 ;
        if (!perr_asserted)
            disable wait_perr2 ;
    end
    join

    if ( perr_asserted && ok )
    begin
        test_ok ;
    end
    else
    if ( ~perr_asserted )
    begin
        test_fail("PCI behavioral target failed to assert invalid PERR for testing") ;
        disable main ;
    end

    // check all the statuses - if HOST is defined, wait for them to be synced
    `ifdef HOST
    repeat(4)
        @(posedge wb_clock) ;
    `endif

    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER PARITY ERROR DURING MASTER WRITE - PAR. ERR. RESPONSE ENABLED" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set after parity error on PCI bus!") ;
        test_fail("Detected Parity Error bit was not set after parity error on PCI bus") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set for no reason!") ;
        test_fail("Signalled System Error bit was set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit wasn't set even though Parity Error Response bit was set!") ;
        test_fail("Master Data Parity Error bit wasn't set after Parity Error on PCI bus, even though Parity Error Response bit was set") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear status bits and disable parity error response
    config_write( pci_ctrl_offset, temp_val1 & ~(CONFIG_CMD_PAR_ERR_EN), 4'hF, ok ) ;

    test_name = "MASTER WRITE WITH NO PARITY ERRORS" ;

    // disable perr generation and perform a write - no bits should be set
    test_target_response[`TARGET_ENCODED_DATA_PAR_ERR] = 0 ;
    fork
    begin
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Bridge failed to process single memory write!") ;
            test_fail("bridge failed to post single memory write") ;
            disable main ;
        end
    end
    begin
        pci_transaction_progress_monitor(target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok) ;

        if ( ok !== 1 )
            test_fail("bridge failed to start posted memory write transaction on PCI bus correctly") ;
        else
            test_ok ;

        repeat(3)
            @(posedge pci_clock) ;
    end
    join

    `ifdef HOST
    repeat(4)
        @(posedge wb_clock) ;
    `endif

    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER NORMAL MEMORY WRITE" ;
    ok = 1 ;

    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was set for no reason!") ;
        test_fail("Detected Parity Error bit was set even though no parity errors happened on PCI") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set for no reason!") ;
        test_fail("Signalled System Error bit was set even though no parity errors happened on PCI") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set for no reason!") ;
        test_fail("Master Data Parity Error bit was set even though no parity errors happened on PCI") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    $display(" Introducing Parity Errors to Master reads ! " ) ;

    read_data = 0 ;
    read_data`READ_ADDRESS  = target_address ;
    read_data`READ_SEL      = 4'hF ;
    read_data`READ_TAG_STIM = 0 ;

    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;

    // enable parity and system error interrupts
    config_write ( icr_offset, 32'h0000_0018, 4'h1, ok ) ;

    // enable parity error response
    config_write ( pci_ctrl_offset, (temp_val1 | CONFIG_CMD_PAR_ERR_EN), 4'hF, ok ) ;

    test_target_response[`TARGET_ENCODED_DATA_PAR_ERR] = 1 ;

    test_name = "BRIDGE'S RESPONSE TO PARITY ERRORS DURING MASTER READS" ;
    fork
    begin
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
    end
    begin:wait_perr4
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while ( PERR !== 0 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;

    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 0, ok ) ;

        if ( ok !== 1 )
            test_fail("bridge failed to process single memory read correctly, or target didn't respond to it") ;

        repeat(2)
            @(posedge pci_clock) ;

        #1 ;
        if ( !perr_asserted )
            disable wait_perr4 ;
    end
    join

    if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Bridge failed to process single memory read!") ;
        test_fail("bridge didn't process single memory read correctly") ;
        ok = 0 ;
    end

    if ( perr_asserted && ok )
    begin
        test_ok ;
    end
    else
    if ( ~perr_asserted )
    begin
        test_fail("PCI bridge failed to assert invalid PERR on Master Read reference") ;
        disable main ;
    end

    test_name = "INTERRUPT REQUEST ASSERTION AFTER PARITY ERROR" ;
    // interrupt should also be present
    `ifdef HOST
        repeat(4)
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;

        if ( INT_O !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Parity Error was presented on Master reference, parity error int. en. was set, but INT REQ was not signalled on WB bus!") ;
            test_fail("HOST bridge didn't assert INT_O line, after Parity Error was presented during read reference and Par. Err. interrupts were enabled") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat(4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Parity Error caused interrupt request on PCI bus! PCI bus has other means for signaling parity errors!") ;
            test_fail("GUEST bridge shouldn't assert interrupt requests on Parity Errors. Other means are provided for signaling Parity errors") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // check statuses!

    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER MASTER READ PARITY ERROR" ;
    ok = 1 ;

    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when parity error was presented on Read transaction!") ;
        test_fail("Detected Parity Error bit was not set when parity error was presented on Read transaction") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set for no reason!") ;
        test_fail("Signalled System Error bit was set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was not set when parity error was presented during read transaction!") ;
        test_fail("Master Data Parity Error bit was not set when parity error was presented during read transaction and Parity Error Response was enabled") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses and disable parity error response
    config_write( pci_ctrl_offset, (temp_val1 & ~(CONFIG_CMD_PAR_ERR_EN)), 4'b1111, ok ) ;

    test_name = "INTERRUPT STATUS REGISTER AFTER MASTER READ PARITY ERROR" ;
    ok = 1 ;
    config_read( isr_offset, 4'hF, temp_val1 ) ;

    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set for no reason!") ;
        test_fail("System error interrupt status bit set for no reason") ;
        ok = 0 ;
    end

    `ifdef HOST
    if ( temp_val1[3] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was not set when Parity Error occured and PERR INT was enabled!") ;
        test_fail("Parity Error interrupt status bit in HOST bridge was not set when Parity Error occured and PERR INT was enabled") ;
        ok = 0 ;
    end
    `else
    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was set in guest implementation of the bridge!") ;
        test_fail("Parity Error interrupt status bit was set in GUEST implementation of the bridge") ;
        ok = 0 ;
    end
    `endif

    if ( ok )
        test_ok ;

    // clear int statuses
    test_name = "CLEARANCE OF PARITY INTERRUPT STATUSES" ;

    config_write( isr_offset, temp_val1, 4'hF, ok ) ;

    `ifdef HOST
        repeat(4)
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;

        if ( INT_O !== 0 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Interrupt request was not cleared when status bits were cleared!") ;
            test_fail("Interrupt request was not cleared when interrupt status bits were cleared") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat(4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Interrupt request was not cleared when status bits were cleared!") ;
            test_fail("Interrupt request was not cleared when interrupt status bits were cleared") ;
        end
        else
            test_ok ;
    `endif
    `endif

    test_name = "NO PERR ASSERTION ON MASTER READ WITH PAR. ERR. RESPONSE DISABLED" ;

    // repeat the read - because Parity error response is not enabled, PERR should not be asserted
    fork
    begin
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
    end
    begin:wait_perr5
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while ( PERR === 1 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Bridge asserted PERR during read transaction when Parity Error response was disabled!") ;
        test_fail("Bridge asserted PERR during read transaction when Parity Error response was disabled") ;
    end
    begin
        pci_transaction_progress_monitor(target_address, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
            test_fail("bridge failed to engage expected read transaction on PCI bus") ;

        // perr can be asserted on idle or next PCI address phase
        repeat(2)
            @(posedge pci_clock) ;

        #1 ;
        if ( !perr_asserted )
            disable wait_perr5 ;
    end
    join

    if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Bridge failed to process single memory read!") ;
        test_fail("bridge failed to process single memory read correctly") ;
        ok = 0 ;
    end

    if ( ok && !perr_asserted)
        test_ok ;

    test_name = "INTERRUPT REQUEST CHECK AFTER READ PARITY ERROR WITH PARITY ERROR RESPONSE DISABLED" ;

    // interrupts should not be present
    `ifdef HOST
        repeat( 4 )
            @(posedge pci_clock) ;
        repeat( 4 )
            @(posedge wb_clock) ;
        if ( INT_O !== 0 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Parity Error response was disabled, but bridge asserted interrupt because of parity error!") ;
            test_fail("Parity Error response was disabled, but bridge asserted interrupt") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat( 4 )
            @(posedge wb_clock) ;
        repeat( 4 )
            @(posedge pci_clock) ;
        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Parity Error response was disabled, but bridge asserted interrupt because of parity error!") ;
            test_fail("Parity Error response was disabled, but bridge asserted interrupt") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER MASTER READ PARITY ERROR" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when parity error was presented on Read transaction!") ;
        test_fail("Detected Parity Error bit was not set when parity error was presented on PCI Master Read transaction") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set for no reason!") ;
        test_fail("Signalled System Error bit was set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when parity error was presented during read transaction, but Parity Response was disabled!") ;
        test_fail("Master Data Parity Error bit was set, but Parity Response was disabled");
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1100, ok ) ;

    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER MASTER READ PARITY ERROR WITH PAR. ERR. RESPONSE DISABLED" ;
    ok = 1 ;
    config_read( isr_offset, 4'hF, temp_val1 ) ;

    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set for no reason!") ;
        test_fail("System error interrupt status bit set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was set when Parity Error occured and Parity Error response was disabled!") ;
        test_fail("Parity Error interrupt status bit was set when Parity Error response was disabled!") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // enable all responses to parity errors!
    test_name = "MASTER READ TRANSACTION WITH NO PARITY ERRORS" ;

    config_write( pci_ctrl_offset, 32'h0000_0147, 4'hF, ok ) ;
    config_write ( icr_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;

    test_target_response[`TARGET_ENCODED_DATA_PAR_ERR] = 0 ;

    // repeat a read
    fork
    begin
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
    end
    begin:wait_perr6
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while ( PERR === 1 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Bridge asserted PERR during read transaction when Parity Error response was disabled!") ;
        test_fail("Bridge asserted PERR during read transaction when no parity error occurred") ;
    end
    begin
        pci_transaction_progress_monitor(target_address, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
            test_fail("bridge failed to start expected read transaction on PCI bus") ;

        repeat(2)
            @(posedge pci_clock) ;

        #1 ;
        if ( !perr_asserted )
            disable wait_perr6 ;
    end
    join

    if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Bridge failed to process single memory read!") ;
        test_fail("bridge didn't process single memory read as expected") ;
        ok = 0 ;
    end

    if ( ok && !perr_asserted)
        test_ok ;

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER NORMAL READ" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was set for no reason!") ;
        test_fail("Detected Parity Error bit was set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set for no reason!") ;
        test_fail("Signalled System Error bit was set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set for no reason!") ;
        test_fail("Master Data Parity Error bit was set for no reason") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER NORMAL READ" ;
    ok = 1 ;
    config_read( isr_offset, 4'hF, temp_val1 ) ;

    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set for no reason!") ;
        test_fail("System error interrupt status bit set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity error interrupt status bit set for no reason!") ;
        test_fail("Parity error interrupt status bit set for no reason") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    $display("Presenting address parity error on PCI bus!") ;
    // enable parity errors - this should not affect system errors
    config_write( pci_ctrl_offset, 32'h0000_0047, 4'hF, ok ) ;
    config_write ( icr_offset, 32'h0000_0018, 4'hF, ok ) ;

    // perform PCI write
    // check transaction progress
    test_name = "NO SERR ASSERTION AFTER ADDRESS PARITY ERROR, SERR DISABLED AND PAR. ERR. RESPONSE ENABLED" ;
    fork
    begin
        PCIU_MEM_WRITE_MAKE_SERR ("MEM_WRITE ", `Test_Master_2,
               target_address, 32'h1234_5678,
               1, 8'h0_0, `Test_One_Zero_Target_WS,
               `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        do_pause( 1 ) ;
    end
    begin:wait_serr7
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR === 1 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("SERR asserted on address parity error when System Error response was disabled!") ;
        test_fail("bridge shouldn't assert SERR when SERR is disabled") ;
    end
    begin
        pci_transaction_progress_monitor(target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
            test_fail("behavioral master failed to start expected transaction or behavioral target didn't respond") ;

        if ( !perr_asserted )
            disable wait_serr7 ;
    end
    join

    if ( ok && !perr_asserted)
        test_ok ;

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR AND SERR DISABLED" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when address parity error was presented on PCI!") ;
        test_fail("Detected Parity Error bit was not set when address parity error was presented on PCI") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set on address parity error, when SERR enable bit was disabled!") ;
        test_fail("Signalled System Error bit was set on address parity error, when SERR enable bit was not set") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;
    test_name = "ADDRESS PARITY ERROR ON FIRST DATA PHASE OF DUAL ADDRESS CYCLE - SERR DISABLED, PAR. RESP. ENABLED" ;
    fork
    begin
        ipci_unsupported_commands_master.master_reference
        (
            32'hAAAA_AAAA,      // first part of address in dual address cycle
            32'h5555_5555,      // second part of address in dual address cycle
            `BC_DUAL_ADDR_CYC,  // dual address cycle command
            `BC_MEM_WRITE,      // normal command
            4'h0,               // byte enables
            32'h1234_5678,      // data
            1'b1,               // make address parity error on first phase of dual address
            1'b0,               // make address parity error on second phase of dual address
            ok                  // result of operation
        ) ;
        if ( !perr_asserted )
            disable wait_serr8 ;
    end
    begin:wait_serr8
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR === 1 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("SERR asserted on address parity error when System Error response was disabled!") ;
        test_fail("bridge shouldn't assert SERR when SERR is disabled") ;
    end
    join

    if ( ok && !perr_asserted)
        test_ok ;

    test_name = "ADDRESS PARITY ERROR ON SECOND DATA PHASE OF DUAL ADDRESS CYCLE - SERR DISABLED, PAR. RESP. ENABLED" ;
    fork
    begin
        ipci_unsupported_commands_master.master_reference
        (
            32'hAAAA_AAAA,      // first part of address in dual address cycle
            32'h5555_5555,      // second part of address in dual address cycle
            `BC_DUAL_ADDR_CYC,  // dual address cycle command
            `BC_MEM_WRITE,      // normal command
            4'h0,               // byte enables
            32'h1234_5678,      // data
            1'b0,               // make address parity error on first phase of dual address
            1'b1,               // make address parity error on second phase of dual address
            ok                  // result of operation
        ) ;
        if ( !perr_asserted )
            disable wait_serr9 ;
    end
    begin:wait_serr9
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR === 1 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("SERR asserted on address parity error when System Error response was disabled!") ;
        test_fail("bridge shouldn't assert SERR when SERR is disabled") ;
    end
    join

    if ( ok && !perr_asserted)
        test_ok ;

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR AND SERR DISABLED" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when address parity error was presented on PCI!") ;
        test_fail("Detected Parity Error bit was not set when address parity error was presented on PCI") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set on address parity error, when SERR enable bit was disabled!") ;
        test_fail("Signalled System Error bit was set on address parity error, when SERR enable bit was not set") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    test_name = "ADDRESS PARITY ERROR ON BOTH DATA PHASES OF DUAL ADDRESS CYCLE - SERR DISABLED, PAR. RESP. ENABLED" ;
    fork
    begin
        ipci_unsupported_commands_master.master_reference
        (
            32'hAAAA_AAAA,      // first part of address in dual address cycle
            32'h5555_5555,      // second part of address in dual address cycle
            `BC_DUAL_ADDR_CYC,  // dual address cycle command
            `BC_MEM_WRITE,      // normal command
            4'h0,               // byte enables
            32'h1234_5678,      // data
            1'b1,               // make address parity error on first phase of dual address
            1'b1,               // make address parity error on second phase of dual address
            ok                  // result of operation
        ) ;
        if ( !perr_asserted )
            disable wait_serr10 ;
    end
    begin:wait_serr10
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR === 1 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("SERR asserted on address parity error when System Error response was disabled!") ;
        test_fail("bridge shouldn't assert SERR when SERR is disabled") ;
    end
    join

    if ( ok && !perr_asserted)
        test_ok ;

    test_name = "INTERRUPT REQUEST AFTER ADDRESS PARITY ERROR" ;

    `ifdef HOST
        repeat(4)
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O !== 0 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity Error was presented on PCI, SERR int. en. was not set, but INT REQ was signalled on WB bus!") ;
            test_fail("Address Parity Error was presented on PCI, SERR was not enabled, but INT REQ was signalled on WB bus") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat(4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity Error was presented on PCI, SERR int. en. was not set, but INT REQ was signalled on PCI bus!") ;
            test_fail("GUEST bridge asserted Interrupt Request after Address Parity Error, even though SERR was disabled and PCI has other means for signaling parity errors") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR AND SERR DISABLED" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when address parity error was presented on PCI!") ;
        test_fail("Detected Parity Error bit was not set when address parity error was presented on PCI") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set on address parity error, when SERR enable bit was disabled!") ;
        test_fail("Signalled System Error bit was set on address parity error, when SERR enable bit was not set") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR AND SERR DISABLED" ;
    ok = 1 ;
    config_read( isr_offset, 4'hF, temp_val1 ) ;

    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set when SERR signaling was disabled!") ;
        test_fail("System error interrupt status bit set when SERR signaling was disabled") ;
        ok = 0 ;
    end

    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was set when  Bridge's master didn't perform references!") ;
        test_fail("Parity Error interrupt status bit was set when  Bridge's master didn't perform reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // now enable system error signaling and test response
    test_name = "ADDRESS PARITY ERROR RESPONSE WITH SERR AND PARITY ERROR RESPONSE ENABLED" ;
    config_write( pci_ctrl_offset, 32'h0000_0147, 4'hF, ok ) ;

    fork
    begin
        PCIU_MEM_WRITE_MAKE_SERR ("MEM_WRITE ", `Test_Master_2,
               target_address, 32'h1234_5678,
               1, 8'h7_0, `Test_One_Zero_Target_WS,
               `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        do_pause( 1 ) ;
    end
    begin:wait_serr11
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR !== 0 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
    end
    begin
        pci_transaction_progress_monitor(target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
            test_fail("behavioral PCI Master failed to start expected transaction or behavioral PCI target failed to respond to it") ;

        @(posedge pci_clock) ;
        #1 ;
        if ( !perr_asserted )
            disable wait_serr11 ;
    end
    join

    if ( ok && perr_asserted)
        test_ok ;
    else
    if ( !perr_asserted )
        test_fail("PCI bridge failed to assert SERR when Address Parity Error was presented on PCI bus") ;

    test_name = "INTERRUPT REQUEST AFTER ADDRESS PARITY ERROR WAS PRESENTED ON PCI" ;

    `ifdef HOST
        repeat(4)
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity error just presented on PCI should trigger an interrupt request, but no request detected!") ;
            test_fail("Interrupt Request was not triggered as expected") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat(4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity error just presented on PCI triggered an interrupt request on PCI!") ;
            test_fail("GUEST bridge isn't supposed to trigger interrupts because of parity errors") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR ON PCI" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when address parity error was presented on PCI!") ;
        test_fail("Detected Parity Error bit was not set when address parity error was presented on PCI");
        ok = 0 ;
    end

    if ( temp_val1[30] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was not set on address parity error when expected!") ;
        test_fail("Signalled System Error bit was not set on address parity error as expected") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    config_read( isr_offset, 4'hF, temp_val1 ) ;

    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR ON PCI" ;
    ok = 1 ;

    `ifdef HOST
    if ( temp_val1[4] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit not set when expected!") ;
        test_fail("System error interrupt status bit not set when expected") ;
        ok = 0 ;
    end
    `else
    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set in GUEST implementation of the bridge!") ;
        test_fail("System error interrupt status bit set in GUEST implementation of the bridge") ;
        ok = 0 ;
    end
    `endif

    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was set when  Bridge's master didn't perform references!") ;
        test_fail("Parity Error interrupt status bit was set when  Bridge's master didn't perform references") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( isr_offset, temp_val1, 4'b1111, ok ) ;

    test_name = "ADDRESS PARITY ERROR ON FIRST DATA PHASE OF DUAL ADDRESS CYCLE - SERR ENABLED, PAR. RESP. ENABLED" ;
    fork
    begin
        ipci_unsupported_commands_master.master_reference
        (
            32'hAAAA_AAAA,      // first part of address in dual address cycle
            32'h5555_5555,      // second part of address in dual address cycle
            `BC_DUAL_ADDR_CYC,  // dual address cycle command
            `BC_MEM_WRITE,      // normal command
            4'h0,               // byte enables
            32'h1234_5678,      // data
            1'b1,               // make address parity error on first phase of dual address
            1'b0,               // make address parity error on second phase of dual address
            ok                  // result of operation
        ) ;
        if ( !perr_asserted )
            disable wait_serr14 ;
    end
    begin:wait_serr14
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR !== 0 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
    end
    join

    if ( ok && perr_asserted)
        test_ok ;
    else
    if ( !perr_asserted )
        test_fail("PCI bridge failed to assert SERR when Address Parity Error was presented on PCI bus") ;

    test_name = "INTERRUPT REQUEST AFTER ADDRESS PARITY ERROR WAS PRESENTED ON PCI" ;

    `ifdef HOST
        repeat(4)
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity error just presented on PCI should trigger an interrupt request, but no request detected!") ;
            test_fail("Interrupt Request was not triggered as expected") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat(4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity error just presented on PCI triggered an interrupt request on PCI!") ;
            test_fail("GUEST bridge isn't supposed to trigger interrupts because of parity errors") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR ON PCI" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when address parity error was presented on PCI!") ;
        test_fail("Detected Parity Error bit was not set when address parity error was presented on PCI");
        ok = 0 ;
    end

    if ( temp_val1[30] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was not set on address parity error when expected!") ;
        test_fail("Signalled System Error bit was not set on address parity error as expected") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    config_read( isr_offset, 4'hF, temp_val1 ) ;

    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR ON PCI" ;
    ok = 1 ;

    `ifdef HOST
    if ( temp_val1[4] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit not set when expected!") ;
        test_fail("System error interrupt status bit not set when expected") ;
        ok = 0 ;
    end
    `else
    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set in GUEST implementation of the bridge!") ;
        test_fail("System error interrupt status bit set in GUEST implementation of the bridge") ;
        ok = 0 ;
    end
    `endif

    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was set when  Bridge's master didn't perform references!") ;
        test_fail("Parity Error interrupt status bit was set when  Bridge's master didn't perform references") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( isr_offset, temp_val1, 4'b1111, ok ) ;

    test_name = "ADDRESS PARITY ERROR ON SECOND DATA PHASE OF DUAL ADDRESS CYCLE - SERR ENABLED, PAR. RESP. ENABLED" ;
    fork
    begin
        ipci_unsupported_commands_master.master_reference
        (
            32'hAAAA_AAAA,      // first part of address in dual address cycle
            32'h5555_5555,      // second part of address in dual address cycle
            `BC_DUAL_ADDR_CYC,  // dual address cycle command
            `BC_MEM_WRITE,      // normal command
            4'h0,               // byte enables
            32'h1234_5678,      // data
            1'b0,               // make address parity error on first phase of dual address
            1'b1,               // make address parity error on second phase of dual address
            ok                  // result of operation
        ) ;
        if ( !perr_asserted )
            disable wait_serr15 ;
    end
    begin:wait_serr15
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR !== 0 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
    end
    join

    if ( ok && perr_asserted)
        test_ok ;
    else
    if ( !perr_asserted )
        test_fail("PCI bridge failed to assert SERR when Address Parity Error was presented on PCI bus") ;

    test_name = "INTERRUPT REQUEST AFTER ADDRESS PARITY ERROR WAS PRESENTED ON PCI" ;

    `ifdef HOST
        repeat(4)
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity error just presented on PCI should trigger an interrupt request, but no request detected!") ;
            test_fail("Interrupt Request was not triggered as expected") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat(4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity error just presented on PCI triggered an interrupt request on PCI!") ;
            test_fail("GUEST bridge isn't supposed to trigger interrupts because of parity errors") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR ON PCI" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when address parity error was presented on PCI!") ;
        test_fail("Detected Parity Error bit was not set when address parity error was presented on PCI");
        ok = 0 ;
    end

    if ( temp_val1[30] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was not set on address parity error when expected!") ;
        test_fail("Signalled System Error bit was not set on address parity error as expected") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    config_read( isr_offset, 4'hF, temp_val1 ) ;

    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR ON PCI" ;
    ok = 1 ;

    `ifdef HOST
    if ( temp_val1[4] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit not set when expected!") ;
        test_fail("System error interrupt status bit not set when expected") ;
        ok = 0 ;
    end
    `else
    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set in GUEST implementation of the bridge!") ;
        test_fail("System error interrupt status bit set in GUEST implementation of the bridge") ;
        ok = 0 ;
    end
    `endif

    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was set when  Bridge's master didn't perform references!") ;
        test_fail("Parity Error interrupt status bit was set when  Bridge's master didn't perform references") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, (32'h0000_0147 | temp_val1), 4'b1111, ok ) ;

    test_name = "ADDRESS PARITY ERROR ON BOTH DATA PHASES OF DUAL ADDRESS CYCLE - SERR ENABLED, PAR. RESP. ENABLED" ;
    fork
    begin
        ipci_unsupported_commands_master.master_reference
        (
            32'hAAAA_AAAA,      // first part of address in dual address cycle
            32'h5555_5555,      // second part of address in dual address cycle
            `BC_DUAL_ADDR_CYC,  // dual address cycle command
            `BC_MEM_WRITE,      // normal command
            4'h0,               // byte enables
            32'h1234_5678,      // data
            1'b1,               // make address parity error on first phase of dual address
            1'b1,               // make address parity error on second phase of dual address
            ok                  // result of operation
        ) ;
        if ( !perr_asserted )
            disable wait_serr16 ;
    end
    begin:wait_serr16
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR !== 0 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
    end
    join

    if ( ok && perr_asserted)
        test_ok ;
    else
    if ( !perr_asserted )
        test_fail("PCI bridge failed to assert SERR when Address Parity Error was presented on PCI bus") ;

    test_name = "INTERRUPT REQUEST AFTER ADDRESS PARITY ERROR WAS PRESENTED ON PCI" ;

    `ifdef HOST
        repeat(4)
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity error just presented on PCI should trigger an interrupt request, but no request detected!") ;
            test_fail("Interrupt Request was not triggered as expected") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat(4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity error just presented on PCI triggered an interrupt request on PCI!") ;
            test_fail("GUEST bridge isn't supposed to trigger interrupts because of parity errors") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR ON PCI" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when address parity error was presented on PCI!") ;
        test_fail("Detected Parity Error bit was not set when address parity error was presented on PCI");
        ok = 0 ;
    end

    if ( temp_val1[30] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was not set on address parity error when expected!") ;
        test_fail("Signalled System Error bit was not set on address parity error as expected") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    config_read( isr_offset, 4'hF, temp_val1 ) ;

    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR ON PCI" ;
    ok = 1 ;

    `ifdef HOST
    if ( temp_val1[4] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit not set when expected!") ;
        test_fail("System error interrupt status bit not set when expected") ;
        ok = 0 ;
    end
    `else
    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set in GUEST implementation of the bridge!") ;
        test_fail("System error interrupt status bit set in GUEST implementation of the bridge") ;
        ok = 0 ;
    end
    `endif

    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was set when  Bridge's master didn't perform references!") ;
        test_fail("Parity Error interrupt status bit was set when  Bridge's master didn't perform references") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( isr_offset, temp_val1, 4'b1111, ok ) ;

    // now just disable Parity Error response - on Address par errors nothing should happen
    config_write( pci_ctrl_offset, 32'h0000_0107, 4'b0011, ok ) ;

    test_name = "NO SERR ASSERTION ON ADDRESS PARITY ERROR WITH SERR ENABLED AND PAR. ERR. RESPONSE DISABLED" ;
    fork
    begin
        PCIU_MEM_WRITE_MAKE_SERR ("MEM_WRITE ", `Test_Master_2,
               target_address, 32'h1234_5678,
               1, 8'h2_0, `Test_One_Zero_Target_WS,
               `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        do_pause( 1 ) ;
    end
    begin:wait_serr12
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR === 1 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("SERR asserted on address parity error when System Error response was disabled!") ;
        test_fail("SERR asserted when parity error response was disabled") ;
    end
    begin
        pci_transaction_progress_monitor(target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
            test_fail("behavioral PCI Master failed to start expected transaction or behavioral PCI target failed to respond to it") ;

        @(posedge pci_clock) ;
        #1 ;
        if ( !perr_asserted )
            disable wait_serr12 ;
    end
    join

    if ( ok && !perr_asserted )
        test_ok ;

    test_name = "INTERRUPT REQUEST AFTER ADDR. PARITY ERROR WITH PERR RESPONSE DISABLED" ;
    `ifdef HOST
        repeat (4)
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O !== 0 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity error just presented on PCI shouldn't trigger an interrupt request!") ;
            test_fail("Interrupt Request should not be asserted when parity error response is disabled") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat (4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity error just presented on PCI shouldn't trigger an interrupt request!") ;
            test_fail("Parity Errors shouldn't trigger interrupts on PCI bus") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER ADDR PERR WITH PERR RESPONSE DISABLED" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when address parity error was presented on PCI!") ;
        test_fail("Detected Parity Error bit was not set when address parity error was presented on PCI") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set on address parity error when not expected!") ;
        test_fail("Signalled System Error bit was set on address parity error when not expected") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    config_read( isr_offset, 4'hF, temp_val1 ) ;
    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER ADDR PERR WITH PERR RESPONSE DISABLED" ;
    ok = 1 ;
    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set when not expected!") ;
        test_fail("System error interrupt status bit set when not expected") ;
        ok = 0 ;
    end

    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was set when  Bridge's master didn't perform references!") ;
        test_fail("Parity Error interrupt status bit was set when  Bridge's master didn't perform references") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( isr_offset, temp_val1, 4'b1111, ok ) ;

    test_name = "ADDRESS PARITY ERROR ON FIRST DATA PHASE OF DUAL ADDRESS CYCLE - SERR ENABLED, PAR. RESP. DISABLED" ;
    fork
    begin
        ipci_unsupported_commands_master.master_reference
        (
            32'hAAAA_AAAA,      // first part of address in dual address cycle
            32'h5555_5555,      // second part of address in dual address cycle
            `BC_DUAL_ADDR_CYC,  // dual address cycle command
            `BC_MEM_WRITE,      // normal command
            4'h0,               // byte enables
            32'h1234_5678,      // data
            1'b1,               // make address parity error on first phase of dual address
            1'b0,               // make address parity error on second phase of dual address
            ok                  // result of operation
        ) ;
        if ( !perr_asserted )
            disable wait_serr17 ;
    end
    begin:wait_serr17
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR === 1 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("SERR asserted on address parity error when Parity Error response was disabled!") ;
        test_fail("bridge shouldn't assert SERR when PERR is disabled") ;
    end
    join

    if ( ok && !perr_asserted)
        test_ok ;

    test_name = "ADDRESS PARITY ERROR ON SECOND DATA PHASE OF DUAL ADDRESS CYCLE - SERR ENABLED, PAR. RESP. DISABLED" ;
    fork
    begin
        ipci_unsupported_commands_master.master_reference
        (
            32'hAAAA_AAAA,      // first part of address in dual address cycle
            32'h5555_5555,      // second part of address in dual address cycle
            `BC_DUAL_ADDR_CYC,  // dual address cycle command
            `BC_MEM_WRITE,      // normal command
            4'h0,               // byte enables
            32'h1234_5678,      // data
            1'b0,               // make address parity error on first phase of dual address
            1'b1,               // make address parity error on second phase of dual address
            ok                  // result of operation
        ) ;
        if ( !perr_asserted )
            disable wait_serr18 ;
    end
    begin:wait_serr18
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR === 1 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("SERR asserted on address parity error when Parity Error response was disabled!") ;
        test_fail("bridge shouldn't assert SERR when PERR is disabled") ;
    end
    join

    if ( ok && !perr_asserted)
        test_ok ;

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR AND PERR DISABLED" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when address parity error was presented on PCI!") ;
        test_fail("Detected Parity Error bit was not set when address parity error was presented on PCI") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set on address parity error, when SERR enable bit was disabled!") ;
        test_fail("Signalled System Error bit was set on address parity error, when SERR enable bit was not set") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    test_name = "ADDRESS PARITY ERROR ON BOTH DATA PHASES OF DUAL ADDRESS CYCLE - SERR ENABLED, PAR. RESP. DISABLED" ;
    fork
    begin
        ipci_unsupported_commands_master.master_reference
        (
            32'hAAAA_AAAA,      // first part of address in dual address cycle
            32'h5555_5555,      // second part of address in dual address cycle
            `BC_DUAL_ADDR_CYC,  // dual address cycle command
            `BC_MEM_WRITE,      // normal command
            4'h0,               // byte enables
            32'h1234_5678,      // data
            1'b1,               // make address parity error on first phase of dual address
            1'b1,               // make address parity error on second phase of dual address
            ok                  // result of operation
        ) ;
        if ( !perr_asserted )
            disable wait_serr19 ;
    end
    begin:wait_serr19
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR === 1 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("SERR asserted on address parity error when Patity Error response was disabled!") ;
        test_fail("bridge shouldn't assert SERR when PERR is disabled") ;
    end
    join

    if ( ok && !perr_asserted)
        test_ok ;

    test_name = "INTERRUPT REQUEST AFTER ADDRESS PARITY ERROR" ;

    `ifdef HOST
        repeat(4)
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O !== 0 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity Error was presented on PCI, SERR int. en. was not set, but INT REQ was signalled on WB bus!") ;
            test_fail("Address Parity Error was presented on PCI, SERR was not enabled, but INT REQ was signalled on WB bus") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat(4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Address Parity Error was presented on PCI, SERR int. en. was not set, but INT REQ was signalled on PCI bus!") ;
            test_fail("GUEST bridge asserted Interrupt Request after Address Parity Error, even though SERR was disabled and PCI has other means for signaling parity errors") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER ADDRESS PARITY ERROR AND PERR DISABLED" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set when address parity error was presented on PCI!") ;
        test_fail("Detected Parity Error bit was not set when address parity error was presented on PCI") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set on address parity error, when SERR enable bit was disabled!") ;
        test_fail("Signalled System Error bit was set on address parity error, when SERR enable bit was not set") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    test_name = "EXTERNAL WRITE WITH NO PARITY ERRORS" ;

    // do normal write
    fork
    begin
        PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_2,
               target_address, 32'h1234_5678, `Test_All_Bytes,
               1, 8'h3_0, `Test_One_Zero_Target_WS,
               `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        do_pause( 1 ) ;
    end
    begin:wait_serr13
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while( SERR === 1 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("SERR asserted for no reason!") ;
        test_fail("SERR was asserted for no reason") ;
    end
    begin
        pci_transaction_progress_monitor(target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
            test_fail("behavioral PCI Master failed to start expected transaction or behavioral PCI target failed to respond to it") ;

        @(posedge pci_clock) ;
        #1 ;
        if ( !perr_asserted )
            disable wait_serr13 ;
    end
    join

    if ( ok && !perr_asserted )
        test_ok ;

    test_name = "INTERRUPT REQUESTS AFTER NORMAL EXTERNAL MASTER WRITE" ;
    `ifdef HOST
        repeat( 4 )
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O !== 0 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Interrupt request asserted for no reason!") ;
            test_fail("Interrupt request was asserted for no reason") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat(4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Interrupt request asserted for no reason!") ;
            test_fail("Interrupt request was asserted for no reason") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER NORMAL EXTERNAL MASTER WRITE" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was set for no reason!") ;
        test_fail("Detected Parity Error bit was set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set for no reason!") ;
        test_fail("Signalled System Error bit was set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    config_read( isr_offset, 4'hF, temp_val1 ) ;
    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER NORMAL EXTERNAL MASTER WRITE" ;
    ok = 1 ;

    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set when not expected!") ;
        test_fail("System error interrupt status bit set when not expected") ;
        ok = 0 ;
    end

    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was set when  Bridge's master didn't perform references!") ;
        test_fail("Parity Error interrupt status bit was set when  Bridge's master didn't perform references") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( isr_offset, temp_val1, 4'b1111, ok ) ;

    $display("Introducing Data Parity Errors on Bridge's Target references!") ;

    $display("Introducing Data Parity Error on Write reference to Bridge's Target!") ;

    // set response of WB SLAVE - ACK,    WAIT cycles,        RETRY cycles
    wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

    // setup target's image!
    target_address  = Target_Base_Addr_R[1] ;

    // base address
    config_write( p_ba_offset, target_address, 4'b1111, ok ) ;

    // address mask
    config_write( p_am_offset, 32'hFFFF_FFFF, 4'b1111, ok ) ;

    // image control
    config_write( p_ctrl_offset, 32'h0000_0000, 4'hF, ok ) ;

    // enable everything possible for parity checking
    config_write( pci_ctrl_offset, 32'h0000_0147, 4'b1111, ok ) ;
    config_write( icr_offset, 32'h0000_0018, 4'b0001, ok ) ;

    test_name = "INVALID PAR ON WRITE REFERENCE THROUGH BRIDGE'S TARGET - PERR RESPONSE ENABLED" ;

    fork
    begin
        if ( target_mem_image === 1 )
            PCIU_MEM_WRITE_MAKE_PERR ("MEM_WRITE ", `Test_Master_1,
                   target_address, 32'h1234_5678,
                   1, 8'h1_0, `Test_One_Zero_Target_WS,
                   `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
            PCIU_IO_WRITE_MAKE_PERR (
                                    `Test_Master_1,
                                    target_address,
                                    32'h1234_5678,
                                    4'h0,
                                    1,
                                    `Test_Target_Normal_Completion
                                    );

        do_pause( 1 ) ;
    end
    begin:wait_perr11
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while ( PERR !== 0 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;

    end
    begin
        pci_transaction_progress_monitor(target_address, ((target_mem_image === 1) ? `BC_MEM_WRITE : `BC_IO_WRITE) , 1, 0, 1'b1, 1'b0, 0, ok) ;

        if ( ok !== 1 )
            test_fail("behavioral PCI Master failed to start expected transaction or behavioral PCI target failed to respond to it") ;

        repeat(2)
            @(posedge pci_clock) ;

        #1 ;
        if ( !perr_asserted )
            disable wait_perr11 ;
    end
    join

    if ( ok && perr_asserted )
        test_ok ;
    else
    if ( !perr_asserted )
        test_fail("Bridge failed to assert PERR on write reference to bridge's target") ;

    test_name = "INTERRUPT REQUESTS AFTER TARGET WRITE REFERENCE PARITY ERROR" ;
    `ifdef HOST
        repeat (4)
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O !== 0 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Interrupt request asserted for no reason!") ;
            test_fail("Parity Errors musn't cause interrupt requests on Target references") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat (4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Interrupt request asserted for no reason!") ;
            test_fail("Parity Errors musn't cause interrupt requests on Target references") ;
        end
        else
            test_ok ;

    `endif
    `endif

    // check statuses!
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER PARITY ERROR ON TARGET REFERENCE" ;
    ok = 1 ;
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set after Target detected parity error!") ;
        test_fail("Detected Parity Error bit was not set after Target detected parity error") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set for no reason!") ;
        test_fail("Signalled System Error bit was set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    config_read( isr_offset, 4'hF, temp_val1 ) ;

    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER PARITY ERROR ON TARGET REFERENCE" ;
    ok = 1 ;
    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set when not expected!") ;
        test_fail("System error interrupt status bit set when not expected") ;
        ok = 0 ;
    end

    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was set when  Bridge's master didn't perform references!") ;
        test_fail("Parity Error interrupt status bit was set when  Bridge's master didn't perform references") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( isr_offset, temp_val1, 4'b1111, ok ) ;

    $display("Introducing Data Parity Error on Read reference to Bridge's Target!") ;

    test_name = "PARITY ERROR HANDLING ON TARGET READ REFERENCE" ;
    fork
    begin
        if ( target_mem_image === 1 )
            PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                          target_address, 32'h1234_5678,
                          1, 8'h7_0, `Test_One_Zero_Target_WS,
                          `Test_Devsel_Medium, `Test_Target_Retry_On);
        else
            PCIU_IO_READ( `Test_Master_1, target_address, 32'h1234_5678, 4'h0, 1, `Test_Target_Retry_On ) ;

        do_pause( 1 ) ;
    end
    begin:wait_perr12
        perr_asserted = 0 ;
        @(posedge pci_clock) ;
        while ( PERR !== 0 )
            @(posedge pci_clock) ;

        perr_asserted = 1 ;
    end
    begin

        wb_transaction_progress_monitor(target_address, 0, 1, 1'b1, ok) ;
        if ( ok !== 1 )
        begin
            test_fail("Bridge failed to process Target Memory read correctly") ;
            disable main ;
        end

        repeat(3)
            @(posedge pci_clock) ;

        if ( target_mem_image === 1 )
            PCIU_MEM_READ_MAKE_PERR("MEM_READ  ", `Test_Master_1,
                    target_address, 32'h1234_5678,
                    1, 8'h3_0, `Test_One_Zero_Target_WS,
                    `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
            PCIU_IO_READ_MAKE_PERR( `Test_Master_1, target_address, 32'h1234_5678, 4'h0, 1, `Test_Target_Normal_Completion ) ;

        do_pause( 1 ) ;

    end
    begin
        pci_transaction_progress_monitor(target_address, ((target_mem_image === 1) ? `BC_MEM_READ : `BC_IO_READ), 0, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
            test_fail("behavioral PCI master failed to start expected transaction or Bridge's Target failed to respond on it") ;
        else
        begin
            pci_transaction_progress_monitor(target_address, ((target_mem_image === 1) ? `BC_MEM_READ : `BC_IO_READ), 1, 0, 1'b1, 1'b0, 0, ok) ;
            if ( ok !== 1 )
                test_fail("behavioral PCI master failed to start expected transaction or Bridge's Target failed to respond on it") ;
        end

        repeat(2)
            @(posedge pci_clock) ;

        #1 ;
        if ( !perr_asserted )
            disable wait_perr12 ;
    end
    join

    if ( ok && perr_asserted )
        test_ok ;
    else
    if ( !perr_asserted )
        test_fail("behavioral master failed to assert invalid read PERR for testing") ;


    test_name = "INTERRUPT REQUESTS AFTER PERR ON READ REFERENCE THROUGH BRIDGE'S TARGET" ;
    `ifdef HOST
        repeat(4)
            @(posedge pci_clock) ;
        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O !== 0 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Interrupt request asserted for no reason!") ;
            test_fail("Parity Interrupt request should not be asserted because parity errors on Target references") ;
        end
        else
            test_ok ;
    `else
    `ifdef GUEST
        repeat(4)
            @(posedge wb_clock) ;
        repeat(4)
            @(posedge pci_clock) ;

        if ( INTA !== 1 )
        begin
            $display("Parity checker testing failed! Time %t ", $time) ;
            $display("Interrupt request asserted for no reason!") ;
            test_fail("Parity Interrupt request should not be asserted because parity errors on Target references") ;
        end
        else
            test_ok ;
    `endif
    `endif

    // check statuses!
    config_read( pci_ctrl_offset, 4'hF, temp_val1 ) ;
    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER PARITY ERROR ON TARGET READ REFERENCE" ;
    ok = 1 ;
    if ( temp_val1[31] !== 1 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Detected Parity Error bit was not set after Target receive PERR asserted!") ;
        test_fail("Detected Parity Error bit was not set after Target received PERR asserted on read reference") ;
        ok = 0 ;
    end

    if ( temp_val1[30] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Signalled System Error bit was set for no reason!") ;
        test_fail("Signalled System Error bit was set for no reason") ;
        ok = 0 ;
    end

    if ( temp_val1[24] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Master Data Parity Error bit was set when Bridge's master was not even making a reference!") ;
        test_fail("Master Data Parity Error bit was set when Bridge's master was not even making a reference") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    // clear statuses
    config_write( pci_ctrl_offset, temp_val1, 4'b1111, ok ) ;

    config_read( isr_offset, 4'hF, temp_val1 ) ;
    test_name = "INTERRUPT STATUS REGISTER VALUE AFTER PARITY ERROR ON TARGET READ REFERENCE" ;
    ok = 1 ;
    if ( temp_val1[4] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("System error interrupt status bit set when not expected!") ;
        test_fail("System error interrupt status bit set when not expected") ;
        ok = 0 ;
    end

    if ( temp_val1[3] !== 0 )
    begin
        $display("Parity checker testing failed! Time %t ", $time) ;
        $display("Parity Error interrupt status bit was set when  Bridge's master didn't perform references!") ;
        test_fail("Parity Errors on Target references should not cause interrupt statuses to be set") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;
    // clear statuses
    config_write( isr_offset, temp_val1, 4'b1111, ok ) ;

    $fdisplay(pci_mon_log_file_desc,
    "********************************************************************************** End of Monitor complaining section **********************************************************************************") ;
    test_name = "DISABLE USED IMAGES" ;
    config_write( am_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Parity checker testing failed! Failed to write W_AM1 register! Time %t ", $time) ;
        test_fail("WB Image Address Mask register could not be written to") ;
        disable main ;
    end

    config_write( p_am_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Parity checker testing failed! Failed to write W_AM1 register! Time %t ", $time) ;
        test_fail("PCI Image Address Mask register could not be written to") ;
        disable main ;
    end

    // disable target's 1 response to parity errors
    configuration_cycle_write(0,             // bus number
                              1,             // device number
                              0,             // function number
                              1,             // register number
                              0,             // type of configuration cycle
                              4'b0001,       // byte enables
                              32'h0000_0007  // data
                             ) ;

    $display("**************************** DONE testing Parity Checker functions ******************************") ;
end
endtask // parity_checking

task wb_to_pci_transactions ;
    reg   [11:0] ctrl_offset ;
    reg   [11:0] ba_offset ;
    reg   [11:0] am_offset ;
    reg   [11:0] pci_ctrl_offset ;
    reg   [11:0] err_cs_offset ;
    reg   [11:0] icr_offset ;
    reg   [11:0] isr_offset ;
    reg   [11:0] lat_tim_cls_offset ;

    reg `WRITE_STIM_TYPE  write_data ;
    reg `READ_STIM_TYPE   read_data ;
    reg `READ_RETURN_TYPE read_status ;

    reg `WRITE_RETURN_TYPE write_status ;
    reg `WB_TRANSFER_FLAGS write_flags ;
    reg [31:0] temp_val1 ;
    reg [31:0] temp_val2 ;
    reg        ok   ;

    reg [31:0] image_base ;
    reg [31:0] target_address ;
    integer i ;
    integer required_reads ;
    integer writes_left ;

begin:main
    ctrl_offset        = {4'h1, `W_IMG_CTRL1_ADDR, 2'b00} ;
    ba_offset          = {4'h1, `W_BA1_ADDR, 2'b00} ;
    am_offset          = {4'h1, `W_AM1_ADDR, 2'b00} ;
    pci_ctrl_offset    = 12'h4 ;
    err_cs_offset      = {4'h1, `W_ERR_CS_ADDR, 2'b00} ;
    icr_offset         = {4'h1, `ICR_ADDR, 2'b00} ;
    isr_offset         = {4'h1, `ISR_ADDR, 2'b00} ;
    lat_tim_cls_offset = 12'hC ;

    $display("Checking WB to PCI transaction lengths!") ;
    target_address  = `BEH_TAR1_MEM_START ;
    image_base      = 0 ;
    image_base[`PCI_BASE_ADDR0_MATCH_RANGE] = target_address[`PCI_BASE_ADDR0_MATCH_RANGE] ;

    target_address[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] = image_base[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] ;
    target_address[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0]  = image_base[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] ;
    write_flags                    = 0 ;
    write_flags`INIT_WAITS         = tb_init_waits ;
    write_flags`SUBSEQ_WAITS       = tb_subseq_waits ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    // enable master & target operation
    test_name = "BRIDGE CONFIGURATION FOR TRANSACTION TESTING" ;
    config_write( pci_ctrl_offset, 32'h0000_0147, 4'h3, ok) ;
    if ( ok !== 1 )
    begin
        $display("WB to PCI transacton progress testing failed! Failed to write PCI Device Control register! Time %t ", $time) ;
        test_fail("write to PCI Device Control register failed") ;
        disable main ;
    end

    // prepare image control register
    config_write( ctrl_offset, 32'h0000_0001, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("WB to PCI transacton progress testing failed! Failed to write W_IMG_CTRL1 register! Time %t ", $time) ;
        test_fail("write to WB Image Control register failed") ;
        disable main ;
    end

    // prepare base address register
    config_write( ba_offset, image_base, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("WB to PCI transacton progress testing failed! Failed to write W_BA1 register! Time %t ", $time) ;
        test_fail("write to WB Base Address register failed") ;
        disable main ;
    end

    // write address mask register
    config_write( am_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("WB to PCI transacton progress testing failed! Failed to write W_AM1 register! Time %t ", $time) ;
        test_fail("write to WB Address Mask register failed") ;
        disable main ;
    end

    // enable all status and error reporting features - this tests should proceede normaly and cause no statuses to be set
    config_write( err_cs_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("WB to PCI transacton progress testing failed! Failed to write W_ERR_CS register! Time %t ", $time) ;
        test_fail("write to WB Error Control and Status register failed") ;
        disable main ;
    end

    // carefull - first value here is 7, because bit 31 of ICR is software reset!
    config_write( icr_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("WB to PCI transacton progress testing failed! Failed to write IC register! Time %t ", $time) ;
        test_fail("write to Interrupt Control register failed") ;
        disable main ;
    end

    // set latency timer and cache line size to 4 - this way even the smallest fifo sizes can be tested
    config_write( lat_tim_cls_offset, 32'hFFFF_0404, 4'h3, ok ) ;
    if ( ok !== 1 )
    begin
        $display("WB to PCI transacton progress testing failed! Failed to write latency timer and cache line size values! Time %t ", $time) ;
        test_fail("write to Latency Timer and Cache Line Size registers failed") ;
        disable main ;
    end

    $display("Testing single write transaction progress from WB to PCI!") ;
    write_data`WRITE_ADDRESS = target_address ;
    write_data`WRITE_DATA    = wmem_data[0] ;
    write_data`WRITE_SEL     = 4'hF ;

    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Retry_Before ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

    test_name = "SINGLE POSTED WRITE TRANSACTION PROCESSING ON PCI" ;
    fork
    begin
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge failed to process single memory write!") ;
            test_fail("bridge failed to post single memory write") ;
            disable main ;
        end
        test_name = "SINGLE POSTED WRITE FROM WISHBONE TO PCI RETRIED FIRST TIME" ;
    end
    begin
        // wait two retries, then enable target response
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        test_name = "SINGLE POSTED WRITE FROM WISHBONE TO PCI RETRIED SECOND TIME" ;
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        test_name = "SINGLE POSTED WRITE FROM WISHBONE TO PCI DISCONNECTED" ;
        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;
    end
    join

    $display("Testing burst write transaction progress from WB to PCI!") ;
    write_data`WRITE_ADDRESS = target_address ;
    write_data`WRITE_DATA    = wmem_data[0] ;
    write_data`WRITE_SEL     = 4'hF ;

    wishbone_master.blk_write_data[0] = write_data ;

    write_data`WRITE_ADDRESS = target_address + 4 ;
    write_data`WRITE_DATA    = wmem_data[1] ;
    write_data`WRITE_SEL     = 4'hF ;

    wishbone_master.blk_write_data[1] = write_data ;

    write_flags`WB_TRANSFER_SIZE = 2 ;
    write_flags`WB_TRANSFER_CAB  = 1 ;

    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Retry_Before ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

    test_name = "BURST LENGTH 2 POSTED WRITE TRANSACTION PROCESSING ON PCI" ;

    fork
    begin
        wishbone_master.wb_block_write( write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 2 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge failed to process whole CAB memory write!") ;
            test_fail("bridge failed to post whole CAB memory write") ;
            disable main ;
        end
        test_name = "BURST LENGTH 2 POSTED WRITE STARTS WITH RETRY" ;
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        test_name = "BURST LENGTH 2 POSTED WRITE RETRIED SECOND TIME DISCONNECTED ON FIRST DATAPHASE" ;
        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        test_name = "BURST LENGTH 2 POSTED WRITE NORMAL COMPLETION AFTER DISCONNECT " ;
        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Normal_Completion ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 0 ;

        pci_transaction_progress_monitor( target_address + 4, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;
    end
    join

    test_target_response[`TARGET_ENCODED_TERMINATION] = `Test_Target_Retry_Before ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 2 ;

    test_name = "BURST LENGTH 2 POSTED WRITE TRANSACTION PROCESSING ON PCI" ;
    // try same write with other terminations
    fork
    begin
        wishbone_master.wb_block_write( write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 2 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge failed to process whole CAB memory write!") ;
            test_fail("bridge failed to post whole CAB memory write") ;
            disable main ;
        end
        test_name = "BURST LENGTH 2 POSTED WRITE DISCONNECTED AFTER FIRST DATAPHASE FIRST TIME" ;
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        test_name = "BURST LENGTH 2 POSTED WRITE DISCONNECTED WITH SECOND DATAPHASE SECOND TIME" ;
        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

        pci_transaction_progress_monitor( target_address + 4, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;
    end
    join

    // repeat the write with normal completion
    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Normal_Completion ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 0 ;

    test_name = "BURST LENGTH 2 POSTED WRITE TRANSACTION PROCESSING ON PCI WITH NORMAL COMPLETION" ;
    fork
    begin
        wishbone_master.wb_block_write( write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 2 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge failed to process whole CAB memory write!") ;
            test_fail("bridge failed to post whole CAB memory write") ;
            disable main ;
        end
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 2, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;
    end
    join

    // do the same thing with burst length of 3
    write_data`WRITE_ADDRESS = target_address + 8 ;
    write_data`WRITE_DATA    = wmem_data[2] ;
    write_data`WRITE_SEL     = 4'hF ;

    wishbone_master.blk_write_data[2] = write_data ;

    write_flags`WB_TRANSFER_SIZE = 3 ;
    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

    test_name = "BURST LENGTH 3 POSTED WRITE TRANSACTION PROCESSING ON PCI" ;

    fork
    begin
        wishbone_master.wb_block_write( write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 3 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge failed to process whole CAB memory write!") ;
            test_fail("bridge failed to post whole CAB memory write") ;
            disable main ;
        end
        test_name = "BURST LENGTH 3 POSTED WRITE TRANSACTION DISCONNECT ON FIRST DATAPHASE FIRST TIME" ;
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        test_name = "BURST LENGTH 3 POSTED WRITE TRANSACTION DISCONNECT ON SECOND DATAPHASE SECOND TIME" ;
        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 2 ;

        pci_transaction_progress_monitor( target_address + 4, `BC_MEM_WRITE, 2, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;
    end
    join

    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 2 ;

    test_name = "BURST LENGTH 3 POSTED WRITE TRANSACTION PROCESSING ON PCI" ;
    fork
    begin
        wishbone_master.wb_block_write( write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 3 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge failed to process whole CAB memory write!") ;
            test_fail("bridge failed to post whole CAB memory write") ;
            disable main ;
        end
        test_name = "BURST LENGTH 3 POSTED WRITE DISCONNECTED ON SECOND FIRST TIME" ;
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 2, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        test_name = "BURST LENGTH 3 POSTED WRITE DISCONNECTED ON FIRST SECOND TIME" ;
        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

        pci_transaction_progress_monitor( target_address + 8, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

    end
    join

    // repeat with normal completion
    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Normal_Completion ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 0 ;

    test_name = "BURST LENGTH 3 POSTED WRITE TRANSACTION WITH NORMAL COMPLETION" ;
    fork
    begin
        wishbone_master.wb_block_write( write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 3 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge failed to process whole CAB memory write!") ;
            test_fail("bridge failed to post whole CAB memory write") ;
            disable main ;
        end
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 3, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

    end
    join

    // prepare data to fill whole write FIFO + 1 - in parallel prepare read data!
    for ( i = 0 ; i < `WBW_DEPTH - 1 ; i = i + 1 )
    begin
        write_data`WRITE_ADDRESS = target_address + i*4 ;
        write_data`WRITE_DATA    = wmem_data[i] ;
        write_data`WRITE_SEL     = 4'hF ;

        read_data`READ_ADDRESS   = write_data`WRITE_ADDRESS ;
        read_data`READ_SEL       = write_data`WRITE_SEL ;

        wishbone_master.blk_write_data[i]   = write_data ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    write_flags`WB_TRANSFER_CAB      = 1 ;
    write_flags`WB_TRANSFER_SIZE     = `WBW_DEPTH - 1 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Retry_Before ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

    test_name = "BURST LENGTH OF WISHBONE FIFO DEPTH POSTED MEMORY WRITE" ;
    fork
    begin
        wishbone_master.wb_block_write( write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== `WBW_DEPTH - 2 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge failed to process right number of databeats in CAB write!") ;
            $display("WBW_FIFO can accomodate %d entries, WB_SLAVE accepted %d writes!", `WBW_DEPTH - 2, write_status`CYC_ACTUAL_TRANSFER) ;
            test_fail("bridge failed to post whole CAB memory write") ;
            disable main ;
        end

        test_name = "FULL WRITE FIFO BURST RETRIED FIRST TIME" ;

        // read here just checks if data was transfered OK
        write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
        write_flags`WB_TRANSFER_SIZE     = `WBW_DEPTH - 2 ;

        wishbone_master.wb_block_read( write_flags, read_status ) ;

        if ( read_status`CYC_ACTUAL_TRANSFER !== `WBW_DEPTH - 2 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge processed CAB read wrong!") ;
            test_fail("bridge didn't process read OK, to check data written by a burst") ;
        end

    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        test_name = "FULL WRITE FIFO BURST DISCONNECT WITH FIRST SECOND TIME" ;
        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        test_name = "FULL WRITE FIFO BURST DISCONNECT AFTER FIRST THIRD TIME" ;
        test_target_response[`TARGET_ENCODED_TERMINATION] = `Test_Target_Retry_Before ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 2 ;

        pci_transaction_progress_monitor( target_address + 4, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        test_name = "FULL WRITE FIFO BURST DISCONNECT WITH SECOND FOURTH TIME" ;
        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 2 ;

        pci_transaction_progress_monitor( target_address + 8, `BC_MEM_WRITE, 2, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        test_name = "REMAINDER OF FULL WRITE FIFO BURST NORMAL COMPLETION FIFTH TIME" ;
        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Normal_Completion ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 0 ;

        pci_transaction_progress_monitor( target_address + 16, `BC_MEM_WRITE, `WBW_DEPTH - 2 - 4, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        else
            test_ok ;

        // calculate how many read transactions must be done to read all written data back from target - bursts were set to length of 4
        required_reads = (((`WBW_DEPTH - 2) % 4) > 0) ? ((`WBW_DEPTH - 2) / 4) + 1 : ((`WBW_DEPTH - 2) / 4) ;
        test_name = "READ DATA BURSTED TO TARGET BACK AND CHECK VALUES" ;
        for ( i = 0 ; i < required_reads ; i = i + 1 )
        begin
            pci_transaction_progress_monitor( target_address + 16*i, `BC_MEM_READ_LN, 4, 0, 1'b1, 1'b0, 0, ok ) ;
            if ( ok !== 1 )
                test_fail("bridge started invalid memory write transaction or none at all or behavioral target didn't respond as expected") ;
        end
    end
    join

    for ( i = 0 ; i < (`WBW_DEPTH - 2) ; i = i + 1 )
    begin
        read_status = wishbone_master.blk_read_data_out[i] ;
        if (read_status`READ_DATA !== wmem_data[i])
        begin
            display_warning(target_address + 4 * i, wmem_data[i], read_status`READ_DATA) ;
            test_fail("data read from target wasn't the same as data written to it") ;
            ok = 0 ;
        end
    end

    if ( ok )
        test_ok ;

    $display("Testing single read transaction progress from WB to PCI!") ;
    read_data`READ_ADDRESS = target_address + 8 ;
    read_data`READ_SEL     = 4'hF ;

    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;

    test_name = "SINGLE READ TRANSACTION PROCESSING ON PCI" ;
    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Retry_Before ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

    fork
    begin
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge processed single read wrong!") ;
            test_fail("bridge processed single read wrong") ;
            disable main ;
        end

        if (read_status`READ_DATA !== wmem_data[2])
        begin
            display_warning(target_address + 8, wmem_data[2], read_status`READ_DATA) ;
            test_fail("data returned from single read was not as expected") ;
        end
        else
        if ( ok )
            test_ok ;
    end
    begin
        test_name = "SINGLE MEMORY READ RETRIED FIRST TIME" ;
        pci_transaction_progress_monitor( target_address + 8, `BC_MEM_READ, 0, 0, 1'b1, 1'b0, 0, ok ) ;

        if ( ok !== 1 )
            test_fail("bridge started invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;

        test_name = "SINGLE MEMORY READ DISCONNECTED WITH FIRST SECOND TIME" ;
        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

        pci_transaction_progress_monitor( target_address + 8, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    $display("Testing CAB read transaction progress from WB to PCI!") ;

    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Normal_Completion ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 0 ;

    test_name = "FILL TARGET MEMORY WITH DATA" ;
    // first fill target's memory with enough data to fill WBR_FIFO
    for ( i = 0 ; i < `WBR_DEPTH ; i = i + 1 )
    begin
        write_data`WRITE_ADDRESS = target_address + i*4 ;
        write_data`WRITE_DATA    = wmem_data[i] ;
        write_data`WRITE_SEL     = 4'hF ;

        wishbone_master.blk_write_data[i] = write_data ;
    end

    write_flags`WB_TRANSFER_CAB = 1 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_SIZE = `WBR_DEPTH ;

    wishbone_master.wb_block_write( write_flags, write_status ) ;

    if ( write_status`CYC_ACTUAL_TRANSFER !== (`WBR_DEPTH) )
    begin
        $display("Transaction progress testing failed! Time %t ", $time) ;
        $display("Bridge processed CAB write wrong!") ;
        test_fail("bridge didn't process all the writes as it was supposed too") ;
        disable main ;
    end

    test_name = "SINGLE READ TO PUSH WRITE DATA FROM FIFO" ;
    // perform single read to force write data to pci
    read_data`READ_ADDRESS = target_address + 8;
    read_data`READ_SEL     = 4'hF ;

    wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;

    if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Transaction progress testing failed! Time %t ", $time) ;
        $display("Bridge processed single read wrong!") ;
        test_fail("bridge didn't process single memory read as expected") ;
        disable main ;
    end

    wishbone_master.blk_read_data_in[0] = read_data ;

    read_data`READ_ADDRESS = target_address + 12 ;
    read_data`READ_SEL     = 4'hF ;

    wishbone_master.blk_read_data_in[1] = read_data ;

    read_data`READ_ADDRESS = target_address + 16 ;
    read_data`READ_SEL     = 4'hF ;

    wishbone_master.blk_read_data_in[2] = read_data ;

    write_flags`WB_TRANSFER_CAB  = 1 ;
    write_flags`WB_TRANSFER_SIZE = 2 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
    read_status = 0 ;

    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 1 ;

    test_name = "BURST READ WITH DISCONNECT ON FIRST" ;

    ok = 1 ;
    fork
    begin
        while ( read_status`CYC_ACTUAL_TRANSFER === 0 )
            wishbone_master.wb_block_read( write_flags, read_status ) ;

        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge processed CAB read wrong!") ;
            test_fail("bridge didn't process disconnected burst read as expected") ;
        end
        else
        begin

            read_status = wishbone_master.blk_read_data_out[0] ;

            if (read_status`READ_DATA !== wmem_data[2])
            begin
                display_warning(target_address + 8, wmem_data[2], read_status`READ_DATA) ;
                test_fail("bridge provided wrong read data on disconnected burst read") ;
            end
            else
                test_ok ;
        end

        test_name = "BURST READ WITH DISCONNECT AFTER FIRST" ;
        wishbone_master.blk_read_data_in[0] = wishbone_master.blk_read_data_in[1] ;
        wishbone_master.blk_read_data_in[1] = wishbone_master.blk_read_data_in[2] ;

        read_status = 0 ;

        while ( read_status`CYC_ACTUAL_TRANSFER === 0 )
            wishbone_master.wb_block_read( write_flags, read_status ) ;

        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge processed CAB read wrong!") ;
            test_fail("bridge didn't process disconnected burst read as expected") ;
        end
        else
        begin

            read_status = wishbone_master.blk_read_data_out[0] ;

            if (read_status`READ_DATA !== wmem_data[3])
            begin
                display_warning(target_address + 12, wmem_data[3], read_status`READ_DATA) ;
                test_fail("bridge provided wrong read data on disconnected burst read") ;
            end
            else
                test_ok ;
        end

        test_name = "BURST READ WITH DISCONNECT ON SECOND - TAKE OUT ONLY ONE" ;
        // complete delayed read which was requested
        read_data = wishbone_master.blk_read_data_in[2] ;
        write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;

        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge processed single out of burst read wrong!") ;
            test_fail("bridge didn't process disconnected burst converted to single read as expected") ;
        end
        else
        begin

            if (read_status`READ_DATA !== wmem_data[4])
            begin
                display_warning(target_address + 16, wmem_data[4], read_status`READ_DATA) ;
                test_fail("bridge provided wrong read data on disconnected burst read") ;
            end
            else
                test_ok ;
        end

    end
    begin
        pci_transaction_progress_monitor( target_address + 8, `BC_MEM_READ_LN, 1, 0, 1'b1, 1'b0, 0, ok ) ;

        if ( ok !== 1 )
            test_fail("bridge started invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;

        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Retry_Before ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 2 ;

        pci_transaction_progress_monitor( target_address + 12, `BC_MEM_READ_LN, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory read transaction or none at all behavioral target didn't respond as expected") ;

        test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Disc_With ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 2 ;

        pci_transaction_progress_monitor( target_address + 16, `BC_MEM_READ_LN, 2, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    // now try burst read with normal termination
    read_data`READ_ADDRESS = target_address + 12 ;
    read_data`READ_SEL     = 4'hF ;

    wishbone_master.blk_read_data_in[0] = read_data ;

    read_data`READ_ADDRESS = target_address + 16 ;
    read_data`READ_SEL     = 4'hF ;

    wishbone_master.blk_read_data_in[1] = read_data ;

    write_flags`WB_TRANSFER_SIZE = 2 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_CAB = 1 ;

    test_name = "BURST READ WITH NORMAL TERMINATION" ;
    test_target_response[`TARGET_ENCODED_TERMINATION]  = `Test_Target_Normal_Completion ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON] = 0 ;

    fork
    begin
        wishbone_master.wb_block_read( write_flags, read_status ) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 2 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge processed CAB read wrong!") ;
            test_fail("bridge didn't process burst read as expected") ;
            ok = 0 ;
        end
    end
    begin
        pci_transaction_progress_monitor( target_address + 12, `BC_MEM_READ_LN, 4, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    if ( ok )
    begin
        read_status = wishbone_master.blk_read_data_out[0] ;
        if ( read_status`READ_DATA !== wmem_data[3] )
        begin
            display_warning(target_address + 12, wmem_data[3], read_status`READ_DATA) ;
            test_fail("data provided from normaly terminated read was wrong") ;
            ok = 0 ;
        end

        read_status = wishbone_master.blk_read_data_out[1] ;
        if ( read_status`READ_DATA !== wmem_data[4] )
        begin
            display_warning(target_address + 16, wmem_data[4], read_status`READ_DATA) ;
            test_fail("data provided from normaly terminated read was wrong") ;
            ok = 0 ;
        end
    end

    if ( ok )
        test_ok ;

    // disable memory read line command and enable prefetch
    // prepare image control register
    test_name = "RECONFIGURE PCI MASTER/WISHBONE SLAVE" ;
    config_write( ctrl_offset, 32'h0000_0002, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("WB to PCI transacton progress testing failed! Failed to write W_IMG_CTRL1 register! Time %t ", $time) ;
        test_fail("WB Image Control register couldn't be written to") ;
        disable main ;
    end

    write_flags`WB_TRANSFER_SIZE = 4 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_CAB = 1 ;

    test_name = "NORMAL BURST READ WITH NORMAL COMPLETION, MEMORY READ LINE DISABLED, PREFETCH ENABLED, BURST SIZE 4" ;

    for ( i = 0 ; i < 4 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + i*4 ;
        read_data`READ_SEL     = 4'b1010 ;

        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    fork
    begin
        wishbone_master.wb_block_read( write_flags, read_status ) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 4 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge processed CAB read wrong!") ;
            test_fail("bridge didn't process prefetched burst read as expected") ;
            ok = 0 ;
        end
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_READ, 4, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    if ( ok )
    begin
        for ( i = 0 ; i < 4 ; i = i + 1 )
        begin
            read_status = wishbone_master.blk_read_data_out[i] ;
            if ( read_status`READ_DATA !== wmem_data[i] )
            begin
                display_warning(target_address + i*4, wmem_data[i], read_status`READ_DATA) ;
                test_fail("burst read returned unexpected data") ;
                ok = 0 ;
            end
        end
    end

    if ( ok )
        test_ok ;

    // do one single read with different byte enables
    read_data`READ_ADDRESS = target_address + 4 ;
    read_data`READ_SEL     = 4'b1010 ;

    test_name = "SINGLE READ WITH FUNNY BYTE ENABLE COMBINATION" ;
    fork
    begin
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge processed single read wrong!") ;
            test_fail("bridge didn't process single memory read as expected") ;
            ok = 0 ;
        end
    end
    begin
        pci_transaction_progress_monitor( target_address + 4, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    // check read data
    if ( ok )
    begin
        if ( read_status`READ_DATA !== (wmem_data[1] & 32'hFF_00_FF_00) )
        begin
            display_warning(target_address + 4, (wmem_data[1] & 32'hFF_00_FF_00), read_status`READ_DATA) ;
            $display("WB to PCI Transaction progress testing failed! Time %t ", $time) ;
            $display("Possibility of wrong read byte enable passing to PCI is possible!") ;
            ok = 0 ;
            test_fail("unexpected data received from single read") ;
        end
    end

    if ( ok )
         test_ok ;

    // enable prefetch and mrl - now memory read multiple should be used for reads and whole WBR_FIFO should be filled
    test_name = "RECONFIGURE PCI MASTER/WISHBONE SLAVE" ;

    config_write( ctrl_offset, 32'h0000_0003, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("WB to PCI transacton progress testing failed! Failed to write W_IMG_CTRL1 register! Time %t ", $time) ;
        test_fail("WB Image Control register could not be written") ;
        disable main ;
    end

    test_name = "BURST READ WITH NORMAL COMPLETION FILLING FULL FIFO - MRL AND PREFETCH BOTH ENABLED" ;
    for ( i = 0 ; i < `WBR_DEPTH ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + i*4 ;
        read_data`READ_SEL     = 4'b1111 ;

        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    write_flags`WB_TRANSFER_SIZE = `WBR_DEPTH ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
    write_flags`WB_TRANSFER_CAB = 1 ;

    fork
    begin
        read_status         = 0 ;
        read_status`CYC_RTY = 1 ;
        while ( (read_status`CYC_ACTUAL_TRANSFER === 0) && (read_status`CYC_RTY === 1) )
            wishbone_master.wb_block_read( write_flags, read_status ) ;

        if ( read_status`CYC_ACTUAL_TRANSFER !== (`WBR_DEPTH - 1) )
        begin
            $display(" WB to PCI transacton progress testing failed! Time %t ", $time) ;
            $display(" WBR_FIFO can accomodate reads of max size %d.", `WBR_DEPTH - 1 ) ;
            $display(" Max size read test failed. Size of read was %d.", read_status`CYC_ACTUAL_TRANSFER) ;
            test_fail("read performed was not as long as it was expected - full WB Read Fifo - 1") ;
            ok = 0 ;
        end
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_READ_MUL, `WBR_DEPTH - 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge started invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    // now repeat single read to flush redundant read initiated
    write_flags`WB_TRANSFER_SIZE = 1 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_CAB = 1 ;

    read_data`READ_ADDRESS = target_address + (`WBR_DEPTH - 1) * 4 ;
    read_data`READ_SEL     = 4'hF ;

    wishbone_master.blk_read_data_in[0] = read_data ;

    test_name = "SINGLE CAB READ FOR FLUSHING STALE READ DATA FROM FIFO" ;
    wishbone_master.wb_block_read( write_flags, read_status ) ;

    if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display(" WB to PCI transacton progress testing failed! Time %t ", $time) ;
        $display(" PCI bridge failed to process single CAB read!") ;
        test_fail("single CAB write was not processed as expected") ;
    end

    // because last read could be very long on PCI - delete target abort status
    config_write( pci_ctrl_offset, 32'h1000_0000, 4'b1000, ok ) ;

    // write unsupported value to cache line size register
    config_write( lat_tim_cls_offset, 32'h0000_04_05, 4'b0011, ok ) ;

    read_data`READ_ADDRESS = target_address ;
    read_data`READ_SEL     = 4'hF ;
    wishbone_master.blk_read_data_in[0] = read_data ;

    test_name = "WB BURST READ WHEN CACHE LINE SIZE VALUE IS INVALID" ;
    // perform a read
    fork
    begin
        wishbone_master.wb_block_read( write_flags, read_status ) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display(" WB to PCI transacton progress testing failed! Time %t ", $time) ;
            $display(" PCI bridge failed to process single CAB read!") ;
            test_fail("burst read was not processed as expected") ;
            ok = 0 ;
        end
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge did invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    if ( ok )
        test_ok ;

    // write 2 to cache line size register
    config_write( lat_tim_cls_offset, 32'h0000_04_02, 4'b0011, ok ) ;

    // perform a read
    fork
    begin
        wishbone_master.wb_block_read( write_flags, read_status ) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display(" WB to PCI transacton progress testing failed! Time %t ", $time) ;
            $display(" PCI bridge failed to process single CAB read!") ;
            test_fail("burst read was not processed as expected") ;
            ok = 0 ;
        end
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge did invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    if ( ok )
        test_ok ;

    // write 0 to cache line size
    config_write( lat_tim_cls_offset, 32'h0000_04_00, 4'b0011, ok ) ;
    test_name = "WB BURST READ WHEN CACHE LINE SIZE VALUE IS ZERO" ;

    // perform a read
    fork
    begin
        wishbone_master.wb_block_read( write_flags, read_status ) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display(" WB to PCI transacton progress testing failed! Time %t ", $time) ;
            $display(" PCI bridge failed to process single CAB read!") ;
            test_fail("burst read was not processed as expected") ;
            ok = 0 ;
        end
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge did invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    if ( ok )
        test_ok ;

    // write normal value to cls register
    config_write( lat_tim_cls_offset, 32'h0000_02_04, 4'b0011, ok ) ;

    $display("Testing Master's latency timer operation!") ;
    $display("Testing Latency timer during Master Writes!") ;

    for ( i = 0 ; i < 6 ; i = i + 1 )
    begin
        write_data`WRITE_ADDRESS = target_address + i*4 ;
        write_data`WRITE_SEL     = 4'b1111 ;
        write_data`WRITE_DATA    = wmem_data[1023 - i] ;

        wishbone_master.blk_write_data[i] = write_data ;
    end

    write_flags`WB_TRANSFER_SIZE = 6 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
    write_flags`WB_TRANSFER_CAB = 1 ;

    // start wb write, pci write and monitor in parallel
    test_name = "LATENCY TIMER OPERATION ON PCI MASTER WRITE" ;
    fork
    begin
        wishbone_master.wb_block_write( write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 6 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge failed to process CAB write!") ;
            test_fail("bridge didn't post whole burst memory write") ;
            disable main ;
        end
    end
    begin
        // wait for bridge's master to start transaction
        @(posedge pci_clock) ;
        while ( FRAME === 1 )
            @(posedge pci_clock) ;

        // start behavioral master request
        PCIU_MEM_WRITE("MEM_WRITE  ", `Test_Master_1,
               target_address, wmem_data[1023], `Test_All_Bytes,
               1, 8'h2_0, `Test_One_Zero_Target_WS,
               `Test_Devsel_Medium, `Test_Target_Normal_Completion);

        do_pause ( 1 ) ;
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_WRITE, 0, 2, 1'b0, 1'b1, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge didn't finish the burst write transaction after latency timer has expired") ;
        else
            test_ok ;
    end
    join

    // perform a read to check data
    for ( i = 0 ; i < 6 ; i = i + 1 )
    begin
        read_data`READ_ADDRESS = target_address + i*4 ;
        read_data`READ_SEL     = 4'b1111 ;

        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    write_flags`WB_TRANSFER_SIZE = 6 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;
    write_flags`WB_TRANSFER_CAB = 1 ;

    test_name = "BURST WRITE DATA DISCONNECTED BY LATENCY TIMEOUT" ;
    wishbone_master.wb_block_read( write_flags, read_status ) ;

    if ( read_status`CYC_ACTUAL_TRANSFER !== 6 )
    begin
        $display("Transaction progress testing failed! Time %t ", $time) ;
        $display("Bridge failed to process CAB read!") ;
        test_fail("whole burst data not read by bridge - CAB read processed wrong") ;
        disable main ;
    end

    ok = 1 ;
    for ( i = 0 ; i < 6 ; i = i + 1 )
    begin
        read_status = wishbone_master.blk_read_data_out[i] ;

        if ( read_status`READ_DATA !== wmem_data[1023 - i] )
        begin
            $display("Latency timer operation testing failed! Time %t ", $time) ;
            display_warning(target_address + i*4, wmem_data[1023 - i], read_status`READ_DATA) ;
            test_fail("unexpected data read back from PCI") ;
            ok = 0 ;
        end
    end

    if ( ok )
        test_ok ;

    $display("Testing Latency timer during Master Reads!") ;

    // at least 2 words are transfered during Master Reads terminated with timeout
    write_flags`WB_TRANSFER_SIZE = 2 ;
    test_name = "LATENCY TIMER OPERATION DURING MASTER READ" ;
    fork
    begin
        wishbone_master.wb_block_read( write_flags, read_status ) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 2 )
        begin
            $display("Transaction progress testing failed! Time %t ", $time) ;
            $display("Bridge failed to process CAB read!") ;
            test_fail("bridge didn't process burst read as expected") ;
            ok = 0 ;
        end
    end
    begin
        // wait for bridge's master to start transaction
        @(posedge pci_clock) ;
        while ( FRAME === 1 )
            @(posedge pci_clock) ;

        // start behavioral master request
        PCIU_MEM_WRITE("MEM_WRITE  ", `Test_Master_1,
               target_address, wmem_data[0], `Test_All_Bytes,
               1, 8'h3_0, `Test_One_Zero_Target_WS,
               `Test_Devsel_Medium, `Test_Target_Normal_Completion);

        do_pause ( 1 ) ;
    end
    begin
        pci_transaction_progress_monitor( target_address, `BC_MEM_READ_MUL, 0, 2, 1'b0, 1'b1, 0, ok ) ;
        if ( ok !== 1 )
            test_fail("bridge did invalid memory read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    // check data provided by target
    if ( ok )
    begin
        for ( i = 0 ; i < 2 ; i = i + 1 )
        begin
            read_status = wishbone_master.blk_read_data_out[i] ;

            if ( read_status`READ_DATA !== wmem_data[1023 - i] )
            begin
                $display("Latency timer operation testing failed! Time %t ", $time) ;
                display_warning(target_address + i*4, wmem_data[1023 - i], read_status`READ_DATA) ;
                test_fail("burst read interrupted by Latency Timeout didn't return expected data") ;
                ok = 0 ;
            end
        end
    end
    if ( ok )
        test_ok ;

    test_name = "DISABLE_IMAGE" ;
    config_write( am_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("WB to PCI transacton progress testing failed! Failed to write W_AM1 register! Time %t ", $time) ;
        test_fail("write to WB Address Mask register failed") ;
        disable main ;
    end

end
endtask //wb_to_pci_transactions

task iack_cycle ;
    reg `READ_STIM_TYPE   read_data ;
    reg `READ_RETURN_TYPE read_status ;
    reg `WB_TRANSFER_FLAGS flags ;

    reg [31:0] temp_var ;
    reg ok ;
begin

    $display(" Testing Interrupt Acknowledge cycle generation!") ;

    temp_var                                     = { `WB_CONFIGURATION_BASE, 12'h000 } ;
    temp_var[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] = 0 ;

    read_data`READ_ADDRESS = temp_var + { 4'h1, `INT_ACK_ADDR, 2'b00 } ;
    read_data`READ_SEL     = 4'hF ;

    flags = 0 ;

    flags`WB_TRANSFER_AUTO_RTY = 1 ;

    irq_respond = 0 ;
    irq_vector  = 32'hAAAA_AAAA ;
    test_name = "INTERRUPT ACKNOWLEDGE CYCLE GENERATION WITH MASTER ABORT" ;

    fork
    begin
        wishbone_master.wb_single_read( read_data, flags, read_status ) ;
    end
    begin
        pci_transaction_progress_monitor( 32'h0000_0000, `BC_IACK, 0, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
           test_fail("bridge did invalid Interrupt Acknowledge read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    if ( read_status`CYC_ACTUAL_TRANSFER !== 0 || read_status`CYC_ERR !== 1 )
    begin
        $display(" Interrupt acknowledge cycle generation failed! Time %t ", $time ) ;
        $display(" It should be terminated by master abort on PCI and therefore by error on WISHBONE bus!") ;
        test_fail("bridge didn't handle Interrupt Acknowledge cycle as expected") ;
    end
    else
    if ( ok )
        test_ok ;
	// clearing the status bits
	config_write(12'h4, 32'hFFFF_0000, 4'hC, ok);

    irq_respond = 1 ;
    irq_vector  = 32'h5555_5555 ;

    test_name = "INTERRUPT ACKNOWLEDGE CYCLE GENERATION WITH NORMAL COMPLETION" ;
    fork
    begin
        wishbone_master.wb_single_read( read_data, flags, read_status ) ;
    end
    begin
        pci_transaction_progress_monitor( 32'h0000_0000, `BC_IACK, 1, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
           test_fail("bridge did invalid Interrupt Acknowledge read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display(" Interrupt acknowledge cycle generation failed! Time %t ", $time ) ;
        $display(" Bridge failed to process Interrupt Acknowledge cycle!") ;
        test_fail("bridge didn't handle Interrupt Acknowledge cycle as expected") ;
        ok = 0 ;
    end

    if ( read_status`READ_DATA !== irq_vector )
    begin
        $display(" Time %t ", $time ) ;
        $display(" Expected interrupt acknowledge vector was %h, actualy read value was %h ! ", irq_vector, read_status`READ_DATA ) ;
        test_fail("Interrupt Acknowledge returned unexpected data") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;

    read_data`READ_SEL = 4'b0101 ;
    irq_vector  = 32'hAAAA_AAAA ;
    test_name = "INTERRUPT ACKNOWLEDGE CYCLE GENERATION WITH NORMAL COMPLETION AND FUNNY BYTE ENABLES" ;
    fork
    begin
        wishbone_master.wb_single_read( read_data, flags, read_status ) ;
    end
    begin
        pci_transaction_progress_monitor( 32'h0000_0000, `BC_IACK, 1, 0, 1'b1, 1'b0, 0, ok) ;
        if ( ok !== 1 )
           test_fail("bridge did invalid Interrupt Acknowledge read transaction or none at all or behavioral target didn't respond as expected") ;
    end
    join

    if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display(" Interrupt acknowledge cycle generation failed! Time %t ", $time ) ;
        $display(" Bridge failed to process Interrupt Acknowledge cycle!") ;
        test_fail("bridge didn't handle Interrupt Acknowledge cycle as expected") ;
        ok = 0 ;
    end

    if ( read_status`READ_DATA !== 32'h00AA_00AA )
    begin
        $display(" Time %t ", $time ) ;
        $display(" Expected interrupt acknowledge vector was %h, actualy read value was %h ! ", 32'h00AA_00AA, read_status`READ_DATA ) ;
        test_fail("Interrupt Acknowledge returned unexpected data") ;
        ok = 0 ;
    end

    if ( ok )
        test_ok ;


end
endtask //iack_cycle

task transaction_ordering ;
    reg   [11:0] wb_ctrl_offset ;
    reg   [11:0] wb_ba_offset ;
    reg   [11:0] wb_am_offset ;
    reg   [11:0] pci_ctrl_offset ;
    reg   [11:0] pci_ba_offset ;
    reg   [11:0] pci_am_offset ;
    reg   [11:0] pci_device_ctrl_offset ;
    reg   [11:0] wb_err_cs_offset ;
    reg   [11:0] pci_err_cs_offset ;
    reg   [11:0] icr_offset ;
    reg   [11:0] isr_offset ;
    reg   [11:0] lat_tim_cls_offset ;

    reg `WRITE_STIM_TYPE  write_data ;
    reg `READ_STIM_TYPE   read_data ;
    reg `READ_RETURN_TYPE read_status ;

    reg `WRITE_RETURN_TYPE write_status ;
    reg `WB_TRANSFER_FLAGS write_flags ;
    reg [31:0] temp_val1 ;
    reg [31:0] temp_val2 ;
    reg        ok   ;

    reg [31:0] wb_image_base ;
    reg [31:0] wb_target_address ;
    reg [31:0] pci_image_base ;
    integer i ;

    reg     error_monitor_done ;
begin:main
    write_flags`INIT_WAITS = tb_init_waits ;
    write_flags`SUBSEQ_WAITS = tb_subseq_waits ;

    wb_ctrl_offset        = {4'h1, `W_IMG_CTRL1_ADDR, 2'b00} ;
    wb_ba_offset          = {4'h1, `W_BA1_ADDR, 2'b00} ;
    wb_am_offset          = {4'h1, `W_AM1_ADDR, 2'b00} ;
    wb_err_cs_offset      = {4'h1, `W_ERR_CS_ADDR, 2'b00} ;

    pci_ctrl_offset        = {4'h1, `P_IMG_CTRL1_ADDR, 2'b00} ;
    pci_ba_offset          = {4'h1, `P_BA1_ADDR, 2'b00} ;
    pci_am_offset          = {4'h1, `P_AM1_ADDR, 2'b00} ;
    pci_err_cs_offset      = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;

    icr_offset         = {4'h1, `ICR_ADDR, 2'b00} ;
    isr_offset         = {4'h1, `ISR_ADDR, 2'b00} ;
    lat_tim_cls_offset = 12'hC ;
    pci_device_ctrl_offset    = 12'h4 ;

    wb_target_address  = `BEH_TAR1_MEM_START ;
    wb_image_base      = 0 ;
    wb_image_base[`PCI_BASE_ADDR0_MATCH_RANGE] = wb_target_address[`PCI_BASE_ADDR0_MATCH_RANGE] ;

    wb_target_address[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] = wb_image_base[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)] ;
    wb_target_address[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0]  = wb_image_base[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] ;
    write_flags                    = 0 ;
    write_flags`INIT_WAITS         = 0 ;
    write_flags`SUBSEQ_WAITS       = 0 ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    pci_image_base = Target_Base_Addr_R[1] ;

    // enable master & target operation
    test_name = "BRIDGE CONFIGURATION FOR TRANSACTION ORDERING TESTS" ;
    config_write( pci_device_ctrl_offset, 32'h0000_0147, 4'h3, ok) ;
    if ( ok !== 1 )
    begin
        $display("Transacton ordering testing failed! Failed to write PCI Device Control register! Time %t ", $time) ;
        test_fail("write to PCI Device Control register failed") ;
        disable main ;
    end

    // prepare image control register
    config_write( wb_ctrl_offset, 32'h0000_0001, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("Transacton ordering testing failed! Failed to write W_IMG_CTRL1 register! Time %t ", $time) ;
        test_fail("write to WB Image Control register failed") ;
        disable main ;
    end

    // prepare base address register
    config_write( wb_ba_offset, wb_image_base, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Transacton ordering testing failed! Failed to write W_BA1 register! Time %t ", $time) ;
        test_fail("write to WB Base Address register failed") ;
        disable main ;
    end

    // write address mask register
    config_write( wb_am_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Transacton ordering testing failed! Failed to write W_AM1 register! Time %t ", $time) ;
        test_fail("write to WB Address Mask register failed") ;
        disable main ;
    end

    // enable all status and error reporting features - this tests should proceede normaly and cause no statuses to be set
    config_write( wb_err_cs_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Transacton ordering testing failed! Failed to write W_ERR_CS register! Time %t ", $time) ;
        test_fail("write to WB Error Control and Status register failed") ;
        disable main ;
    end

    // prepare image control register
    config_write( pci_ctrl_offset, 32'h0000_0001, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("Transacton ordering testing failed! Failed to write P_IMG_CTRL1 register! Time %t ", $time) ;
        test_fail("write to PCI Image Control register failed") ;
        disable main ;
    end

    // prepare base address register
    config_write( pci_ba_offset, pci_image_base, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Transacton ordering testing failed! Failed to write P_BA1 register! Time %t ", $time) ;
        test_fail("write to PCI Base Address register failed") ;
        disable main ;
    end

    // write address mask register
    config_write( pci_am_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Transacton ordering testing failed! Failed to write P_AM1 register! Time %t ", $time) ;
        test_fail("write to PCI Address Mask register failed") ;
        disable main ;
    end

    // enable all status and error reporting features - this tests should proceede normaly and cause no statuses to be set
    config_write( pci_err_cs_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Transacton ordering testing failed! Failed to write P_ERR_CS register! Time %t ", $time) ;
        test_fail("write to PCI Error Control and Status register failed") ;
        disable main ;
    end

    // carefull - first value here is 7, because bit 31 of ICR is software reset!
    config_write( icr_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Transacton ordering testing failed! Failed to write IC register! Time %t ", $time) ;
        test_fail("write to Interrupt Control register failed") ;
        disable main ;
    end

    // set latency timer and cache line size to 4 - this way even the smallest fifo sizes can be tested
    config_write( lat_tim_cls_offset, 32'hFFFF_0404, 4'h3, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Transacton ordering testing failed! Failed to write latency timer and cache line size values! Time %t ", $time) ;
        test_fail("write to Latency Timer and Cache Line Size registers failed") ;
        disable main ;
    end

    test_name = "SIMULTANEOUS WRITE REFERENCE TO WB SLAVE AND PCI TARGET" ;

    // prepare wb_master write and read data
    for ( i = 0 ; i < 4 ; i = i + 1 )
    begin
        write_data`WRITE_ADDRESS = wb_target_address + i*4 ;
        write_data`WRITE_DATA    = wmem_data[500 + i] ;
        write_data`WRITE_SEL     = 4'hF ;

        read_data`READ_ADDRESS   = write_data`WRITE_ADDRESS ;
        read_data`READ_SEL       = write_data`WRITE_SEL ;

        wishbone_master.blk_write_data[i]   = write_data ;
        wishbone_master.blk_read_data_in[i] = read_data ;
    end

    // put wishbone slave in acknowledge and pci target in retry mode
    test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Retry_Before ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 1 ;

    wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

    fork
    begin
        write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction ordering test failed! Bridge failed to post single memory write! Time %t ", $time) ;
            test_fail("Bridge didn't post single memory write as expected") ;
        end

        pci_transaction_progress_monitor( wb_target_address + 12, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! PCI Master started invalid transaction or none at all on PCI bus after single memory write was posted! Time %t ", $time) ;
            test_fail("PCI Master failed to start valid transaction on PCI bus after single memory write was posted") ;
            ok = 0 ;
        end

        // now post single write to target - normal progress
        if ( target_mem_image == 1 )
            PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                        pci_image_base + 12, 32'h5555_5555, 4'h0,
                        1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
            PCIU_IO_WRITE( `Test_Master_1, pci_image_base + 12, 32'h5555_5555, 4'h0, 1, `Test_Target_Normal_Completion) ;

        do_pause( 1 ) ;

    end
    begin:error_monitor_1
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        wb_transaction_progress_monitor( pci_image_base + 12, 1'b1, 1, 1'b1, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! WB Master didn't start expected transaction on WB bus! Time %t ", $time) ;
            test_fail("WB Master didn't start expected transaction on WB bus") ;
        end
        else
        begin
            pci_transaction_progress_monitor( wb_target_address + 12, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 1'b0, ok ) ;
//            while ( FRAME === 0 || IRDY === 0 )
//                @(posedge pci_clock) ;

            // enable response in PCI target
            test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Normal_Completion ;
            test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 1 ;

            pci_transaction_progress_monitor( wb_target_address + 12, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 1'b0, ok ) ;
            if ( ok !== 1 )
            begin
                $display("Transaction ordering test failed! PCI Master didn't start expected transaction on PCI bus! Time %t ", $time) ;
                test_fail("PCI Master didn't perform expected transaction on PCI bus") ;
            end
        end

        #1 ;
        if ( !error_monitor_done )
            disable error_monitor_1 ;
    end
    join

    if ( ok )
        test_ok ;

    test_name = "SIMULTANEOUS WRITE REFERENCE TO PCI TARGET AND WB SLAVE" ;

    // put WISHBONE slave in retry mode
    wishbone_slave.cycle_response(3'b001, tb_subseq_waits, 8'hFF);

    fork
    begin
        // now post single write to target - normal progress
        if ( target_mem_image == 1 )
            PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                        pci_image_base + 12, 32'h5555_5555, 4'h0,
                        1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
            PCIU_IO_WRITE( `Test_Master_1, pci_image_base + 12, 32'h5555_5555, 4'h0, 1, `Test_Target_Normal_Completion) ;

        do_pause( 1 ) ;

        wb_transaction_progress_monitor( pci_image_base + 12, 1'b1, 0, 1'b1, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! WB Master didn't perform expected transaction on WB bus! Time %t ", $time) ;
            test_fail("WB Master didn't perform expected transaction on WB bus") ;
        end

        write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction ordering test failed! Bridge failed to post single memory write! Time %t ", $time) ;
            test_fail("Bridge didn't post single memory write as expected") ;
        end

        pci_transaction_progress_monitor( wb_target_address + 12, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! PCI Master started invalid transaction or none at all on PCI bus after single memory write was posted! Time %t ", $time) ;
            test_fail("PCI Master failed to start valid transaction on PCI bus after single memory write was posted") ;
            ok = 0 ;
        end

        wait ( CYC_O === 0 ) ;
        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

        wb_transaction_progress_monitor( pci_image_base + 12, 1'b1, 1, 1'b1, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! WB Master didn't start expected transaction on WB bus! Time %t ", $time) ;
            test_fail("WB Master didn't start expected transaction on WB bus") ;
        end

        #1 ;
        if ( !error_monitor_done )
            disable error_monitor_2 ;
    end
    begin:error_monitor_2
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    join

    test_name = "SIMULTANEOUS MULTI BEAT WRITES THROUGH WB SLAVE AND PCI TARGET" ;

    // put wishbone slave in acknowledge and pci target in retry mode
    test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Retry_Before ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 1 ;

    wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

    fork
    begin
        write_flags`WB_TRANSFER_SIZE = 3 ;
        write_flags`WB_TRANSFER_CAB  = 1 ;
        write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
        wishbone_master.wb_block_write( write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 3 )
        begin
            $display("Transaction ordering test failed! Bridge failed to post burst memory write! Time %t ", $time) ;
            test_fail("Bridge didn't post burst memory write as expected") ;
        end

        pci_transaction_progress_monitor( wb_target_address, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! PCI Master started invalid transaction or none at all on PCI bus after single memory write was posted! Time %t ", $time) ;
            test_fail("PCI Master failed to start valid transaction on PCI bus after single memory write was posted") ;
            ok = 0 ;
        end

        // now post single write to target - normal progress
        if ( target_mem_image == 1 )
            PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                        pci_image_base, 32'h5555_5555, 4'h0,
                        3, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
        begin
            PCIU_IO_WRITE( `Test_Master_1, pci_image_base, 32'h5555_5555, 4'h0, 1, `Test_Target_Normal_Completion) ;
            do_pause( 1 ) ;
            PCIU_IO_WRITE( `Test_Master_1, pci_image_base + 4, 32'hAAAA_AAAA, 4'h0, 1, `Test_Target_Normal_Completion) ;
        end

        do_pause( 1 ) ;

    end
    begin:error_monitor_3
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        if ( target_mem_image == 1 )
            wb_transaction_progress_monitor( pci_image_base, 1'b1, 3, 1'b1, ok ) ;
        else
        begin
            wb_transaction_progress_monitor( pci_image_base, 1'b1, 1, 1'b1, ok ) ;
            if ( ok )
                wb_transaction_progress_monitor( pci_image_base + 4, 1'b1, 1, 1'b1, ok ) ;
        end

        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! WB Master didn't start expected transaction on WB bus! Time %t ", $time) ;
            test_fail("WB Master didn't start expected transaction on WB bus") ;
        end
        else
        begin
        	pci_transaction_progress_monitor( wb_target_address, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 1'b0, ok ) ;
//            while ( FRAME === 0 || IRDY === 0 )
//                @(posedge pci_clock) ;

            // enable response in PCI target
            test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Normal_Completion ;
            test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 1 ;

            pci_transaction_progress_monitor( wb_target_address, `BC_MEM_WRITE, 3, 0, 1'b1, 1'b0, 1'b0, ok ) ;
            if ( ok !== 1 )
            begin
                $display("Transaction ordering test failed! PCI Master didn't start expected transaction on PCI bus! Time %t ", $time) ;
                test_fail("PCI Master didn't perform expected transaction on PCI bus") ;
            end
        end

        #1 ;
        if ( !error_monitor_done )
            disable error_monitor_3 ;
    end
    join

    if ( ok )
        test_ok ;

    test_name = "SIMULTANEOUS MULTI BEAT WRITE REFERENCE TO PCI TARGET AND WB SLAVE" ;

    // put WISHBONE slave in retry mode
    wishbone_slave.cycle_response(3'b001, tb_subseq_waits, 8'hFF);

    fork
    begin
        // now post single write to target - normal progress
        if ( target_mem_image == 1 )
            PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                        pci_image_base, 32'h5555_5555, 4'h0,
                        3, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
        begin
            PCIU_IO_WRITE( `Test_Master_1, pci_image_base, 32'h5555_5555, 4'h0, 1, `Test_Target_Normal_Completion) ;
            do_pause( 1 ) ;
            PCIU_IO_WRITE( `Test_Master_1, pci_image_base + 4, 32'h6666_6666, 4'h0, 1, `Test_Target_Normal_Completion) ;
        end

        do_pause( 1 ) ;

        wb_transaction_progress_monitor( pci_image_base, 1'b1, 0, 1'b1, ok ) ;

        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! WB Master didn't perform expected transaction on WB bus! Time %t ", $time) ;
            test_fail("WB Master didn't perform expected transaction on WB bus") ;
        end

        write_flags`WB_TRANSFER_AUTO_RTY = 0 ;
        write_flags`WB_TRANSFER_SIZE     = 4 ;
        write_flags`WB_TRANSFER_CAB      = 1 ;

        wishbone_master.wb_block_write( write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 4 )
        begin
            $display("Transaction ordering test failed! Bridge failed to post burst memory write! Time %t ", $time) ;
            test_fail("Bridge didn't post burst memory write as expected") ;
        end

        pci_transaction_progress_monitor( wb_target_address, `BC_MEM_WRITE, 4, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! PCI Master started invalid transaction or none at all on PCI bus after burst memory write was posted! Time %t ", $time) ;
            test_fail("PCI Master failed to start valid transaction on PCI bus after burst memory write was posted") ;
            ok = 0 ;
        end

        @(posedge wb_clock) ;
        while ( CYC_O === 1 )
            @(posedge wb_clock) ;

        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

        if ( target_mem_image == 1 )
            wb_transaction_progress_monitor( pci_image_base, 1'b1, 3, 1'b1, ok ) ;
        else
        begin
            wb_transaction_progress_monitor( pci_image_base, 1'b1, 1, 1'b1, ok ) ;
            if ( ok )
                wb_transaction_progress_monitor( pci_image_base + 4, 1'b1, 1, 1'b1, ok ) ;
        end

        #1 ;
        if ( !error_monitor_done )
            disable error_monitor_4 ;
    end
    begin:error_monitor_4
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    join

    if ( ok )
        test_ok ;

    test_name = "ORDERING OF TRANSACTIONS MOVING IN SAME DIRECTION - WRITE THROUGH WB SLAVE, READ THROUGH PCI TARGET" ;

    // put wishbone slave in acknowledge and pci target in retry mode
    test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Retry_Before ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 1 ;

    wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

    master1_check_received_data = 1 ;

    error_monitor_done = 0 ;
    fork
    begin:error_monitor_5
        @(error_event_int or error_monitor_done) ;
        if ( !error_monitor_done )
        begin
            test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
            ok = 0 ;
        end
    end
    begin

        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction ordering test failed! Bridge failed to post single memory write! Time %t ", $time) ;
            test_fail("Bridge didn't post single memory write as expected") ;
        end

        pci_transaction_progress_monitor( wb_target_address + 12, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! PCI Master started invalid transaction or none at all on PCI bus after single memory write was posted! Time %t ", $time) ;
            test_fail("PCI Master failed to start valid transaction on PCI bus after single memory write was posted") ;
            ok = 0 ;
        end

        // start Read Through pci target
        if ( target_mem_image == 1 )
            PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                          pci_image_base, 32'h5555_5555,
                          1, 8'h7_0, `Test_One_Zero_Target_WS,
                          `Test_Devsel_Medium, `Test_Target_Retry_On);
        else
            PCIU_IO_READ
             (
                `Test_Master_1,
                pci_image_base,
                32'h5555_5555,
                4'h0,
                1,
                `Test_Target_Retry_On
             );

         do_pause( 1 ) ;

         wb_transaction_progress_monitor( pci_image_base, 1'b0, 1, 1'b1, ok ) ;
         if ( ok !== 1 )
         begin
            $display("Transaction ordering test failed! WB Master started invalid transaction or none at all on WB bus after single delayed read was requested! Time %t ", $time) ;
            test_fail("WB Master failed to start valid transaction on WB bus after single delayed read was requested") ;
         end

         // repeat the read 4 times - it should be retried all the time by pci target
        for ( i = 0 ; i < 4 ; i = i + 1 )
        begin
            if ( target_mem_image == 1 )
                PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                            pci_image_base, 32'h5555_5555,
                            1, 8'h7_0, `Test_One_Zero_Target_WS,
                            `Test_Devsel_Medium, `Test_Target_Retry_On);
            else
                PCIU_IO_READ
                (
                    `Test_Master_1,
                    pci_image_base,
                    32'h5555_5555,
                    4'h0,
                    1,
                    `Test_Target_Retry_On
                );

            do_pause( 1 ) ;
        end

        // now do posted write through target - it must go through OK
        if ( target_mem_image == 1 )
            PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                        pci_image_base, 32'hAAAA_AAAA, 4'h0,
                        1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
            PCIU_IO_WRITE( `Test_Master_1, pci_image_base, 32'hAAAA_AAAA, 4'h0, 1, `Test_Target_Normal_Completion) ;

        do_pause( 1 ) ;

        wb_transaction_progress_monitor( pci_image_base, 1'b1, 1, 1'b1, ok ) ;
        if ( ok !== 1 )
        begin
           $display("Transaction ordering test failed! WB Master started invalid transaction or none at all on WB bus after single write was posted! Time %t ", $time) ;
           test_fail("WB Master failed to start valid transaction on WB bus after single write was posted") ;
        end

        // start a read through wb_slave
        wishbone_master.wb_single_read(read_data, write_flags, read_status) ;
        if ( (read_status`CYC_ACTUAL_TRANSFER !== 0 ) || (read_status`CYC_RTY !== 1))
        begin
            $display("Transaction ordering test failed! WB Slave didn't respond with retry on delayed read request! Time %t ", $time) ;
            test_fail("WB Slave didn't respond as expected to delayed read request") ;
            ok = 0 ;
        end

        pci_transaction_progress_monitor( write_data`WRITE_ADDRESS, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 1'b1, ok ) ;
//        while ( FRAME === 0 || IRDY === 0 )
//            @(posedge pci_clock) ;

        // set the target to normal completion
        test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Normal_Completion ;
        test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 1 ;

        pci_transaction_progress_monitor( write_data`WRITE_ADDRESS, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 1'b1, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! PCI Master started invalid transaction or none at all on PCI bus after single memory write was posted! Time %t ", $time) ;
            test_fail("PCI Master failed to start valid transaction on PCI bus after single memory write was posted") ;
            ok = 0 ;
        end

        // now wait for delayed read to finish
        pci_transaction_progress_monitor( read_data`READ_ADDRESS, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! PCI Master started invalid transaction or none at all on PCI bus after single memory read was requested! Time %t ", $time) ;
            test_fail("PCI Master failed to start valid transaction on PCI bus after single memory read was requested") ;
            ok = 0 ;
        end

        // start a write through target - it shouldn't go through since delayed read completion for WB is already present in a bridge
        fork
        begin
        	if ( target_mem_image == 1 )
            	PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                	        pci_image_base, 32'h5555_5555, 4'h0,
                    	    1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        	`Test_Devsel_Medium, `Test_Target_Retry_On);
        	else
            	PCIU_IO_WRITE( `Test_Master_1, pci_image_base, 32'h5555_5555, 4'h0, 1, `Test_Target_Retry_On) ;

        	do_pause( 1 ) ;
		end
		begin
            pci_transaction_progress_monitor( pci_image_base, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 1'b0, ok ) ;
		end
		join

        // try posting a write through wb_slave - it musn't go through since delayed read completion is present in PCI target unit
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_RTY !== 1) )
        begin
            $display("Transaction ordering test failed! WB Slave should reject posted memory write when delayed read completion is present in PCI Target Unit! Time %t ", $time) ;
            test_fail("WB Slave didn't reject posted memory write when delayed read completion was present in PCI Target Unit") ;
            ok = 0 ;
        end

        fork
        begin
        // now complete a read from PCI Target
            if ( target_mem_image == 1 )
                PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                            pci_image_base, 32'h5555_5555,
                            1, 8'h7_0, `Test_One_Zero_Target_WS,
                            `Test_Devsel_Medium, `Test_Target_Normal_Completion);
            else
                PCIU_IO_READ
                (
                    `Test_Master_1,
                    pci_image_base,
                    32'h5555_5555,
                    4'h0,
                    1,
                    `Test_Target_Normal_Completion
                );

            do_pause( 1 ) ;
        end
        begin
            if ( target_mem_image == 1 )
                pci_transaction_progress_monitor( pci_image_base, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 1'b0, ok ) ;
            else
                pci_transaction_progress_monitor( pci_image_base, `BC_IO_READ, 1, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        end
        join

        @(posedge pci_clock) ;
        repeat( 4 )
            @(posedge wb_clock) ;

        // except read completion in WB slave unit, everything is done - post anoher write - it must be accepted
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction ordering test failed! WB Slave should accept posted memory write when delayed read completion is present in WB Slave Unit! Time %t ", $time) ;
            test_fail("WB Slave didn't accept posted memory write when delayed read completion was present in WB Slave Unit") ;
            ok = 0 ;
        end

        pci_transaction_progress_monitor( write_data`WRITE_ADDRESS, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! PCI Master started invalid transaction or none at all on PCI bus after single memory write was posted! Time %t ", $time) ;
            test_fail("PCI Master failed to start valid transaction on PCI bus after single memory write was posted") ;
            ok = 0 ;
        end

        // finish a read on WISHBONE also
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction ordering test failed! Single delayed read was not processed as expected by WB Slave unit! Time %t ", $time) ;
            test_fail("WB Slave didn't process single delayed read as expected") ;
        end

        if ( read_status`READ_DATA !== wmem_data[500 + 3] )
        begin
            test_fail("delayed read data provided by WB Slave was not as expected") ;
            ok = 0 ;
        end


        error_monitor_done = 1 ;
    end
    join

    if ( ok )
        test_ok ;

    test_name = "ORDERING OF TRANSACTIONS MOVING IN SAME DIRECTION - WRITE THROUGH PCI TARGET, READ THROUGH WB SLAVE" ;

    // put wishbone slave in retry and pci target in completion mode
    test_target_response[`TARGET_ENCODED_TERMINATION]       = `Test_Target_Normal_Completion ;
    test_target_response[`TARGET_ENCODED_TERMINATE_ON]      = 1 ;

    wishbone_slave.cycle_response(3'b001, tb_subseq_waits, 8'hFF);

    master1_check_received_data = 1 ;

    error_monitor_done = 0 ;
    fork
    begin:error_monitor_6
        @(error_event_int or error_monitor_done) ;
        if ( !error_monitor_done )
        begin
            test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
            ok = 0 ;
        end
    end
    begin

        // do a write through Target
        fork
        begin
            if ( target_mem_image == 1 )
                PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                            pci_image_base + 12, 32'hDEAD_BEAF, 4'h0,
                            1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                            `Test_Devsel_Medium, `Test_Target_Normal_Completion);
            else
                PCIU_IO_WRITE( `Test_Master_1, pci_image_base + 12, 32'hDEAD_BEAF, 4'h0, 1, `Test_Target_Normal_Completion) ;

            do_pause( 1 ) ;
        end
        begin
            if ( target_mem_image == 1 )
                pci_transaction_progress_monitor( pci_image_base + 12, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 1'b0, ok ) ;
            else
                pci_transaction_progress_monitor( pci_image_base + 12, `BC_IO_WRITE, 1, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        end
        join

        // start a read through WB slave
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
        if ( (read_status`CYC_ACTUAL_TRANSFER !== 0) || (read_status`CYC_RTY !== 1) )
        begin
            $display("Transaction ordering test failed! Single delayed read request was not processed as expected by WB Slave unit! Time %t ", $time) ;
            test_fail("Single delayed read request was not processed as expected by WB Slave unit") ;
            ok = 0 ;
        end

        // now wait for this read to finish on pci
        pci_transaction_progress_monitor( read_data`READ_ADDRESS, `BC_MEM_READ, 1, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! PCI Master started invalid transaction or none at all on PCI bus after single delayed read was requested! Time %t ", $time) ;
            test_fail("PCI Master failed to start valid transaction on PCI bus after single delayed read was requested") ;
            ok = 0 ;
        end

        // repeat the read four times - it should be retried
        for ( i = 0 ; i < 4 ; i = i + 1 )
        begin
           wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
            if ( (read_status`CYC_ACTUAL_TRANSFER !== 0) || (read_status`CYC_RTY !== 1) )
            begin
                $display("Transaction ordering test failed! Single delayed read was not processed as expected by WB Slave unit! Time %t ", $time) ;
                test_fail("Single delayed read was not processed as expected by WB Slave unit") ;
                ok = 0 ;
            end
        end

        // posted write through WB Slave - must go through
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction ordering test failed! Bridge failed to post single memory write! Time %t ", $time) ;
            test_fail("Bridge didn't post single memory write as expected on WB bus") ;
            ok = 0 ;
        end

        // write must come through
        pci_transaction_progress_monitor( write_data`WRITE_ADDRESS, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! PCI Master started invalid transaction or none at all on PCI bus after single memory write was posted! Time %t ", $time) ;
            test_fail("PCI Master failed to start valid transaction on PCI bus after single memory write was posted") ;
            ok = 0 ;
        end

        // do a read through pci target
        if ( target_mem_image == 1 )
            PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                          pci_image_base + 12, 32'hDEAD_BEAF,
                          1, 8'h7_0, `Test_One_Zero_Target_WS,
                          `Test_Devsel_Medium, `Test_Target_Retry_On);
        else
            PCIU_IO_READ
             (
                `Test_Master_1,
                pci_image_base + 12,
                32'hDEAD_BEAF,
                4'h0,
                1,
                `Test_Target_Retry_On
             );

         do_pause( 1 ) ;

        // wait for current cycle to finish on WB
        wb_transaction_progress_monitor( pci_image_base + 12, 1'b1, 0, 1'b1, ok ) ;
//        @(posedge wb_clock) ;
//        while( CYC_O === 1 )
//            @(posedge wb_clock) ;

        // set slave response to acknowledge
        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

        wb_transaction_progress_monitor( pci_image_base + 12, 1'b1, 1, 1'b1, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! WB Master started invalid transaction or none at all on WB bus after single write was posted! Time %t ", $time) ;
            test_fail("WB Master failed to start valid transaction on WB bus after single write was posted") ;
        end

        // check the read to finish on wb
        wb_transaction_progress_monitor( pci_image_base + 12, 1'b0, 1, 1'b1, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Transaction ordering test failed! WB Master started invalid transaction or none at all on WB bus after single delayed read was requested! Time %t ", $time) ;
            test_fail("WB Master failed to start valid transaction on WB bus after single delayed read was requested") ;
        end

        // do a write to wb_slave - musn't go through, since delayed read completion is present in PCI Target unit
        wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
        if ( (write_status`CYC_ACTUAL_TRANSFER !== 0) || (write_status`CYC_RTY !== 1) )
        begin
            $display("Transaction ordering test failed! WB Slave didn't reject single posted write when delayed read completion was present in PCI Target unit! Time %t ", $time) ;
            test_fail("WB Slave didn't reject single posted write when delayed read completion was present in PCI Target unit") ;
            ok = 0 ;
        end

        // try a write through PCI Target Unit - musn't go through since delayed read completion is present in WB Slave Unit
        fork
        begin
            if ( target_mem_image == 1 )
                PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                            pci_image_base + 12, 32'h1234_5678, 4'h0,
                            1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                            `Test_Devsel_Medium, `Test_Target_Retry_On);
            else
                PCIU_IO_WRITE( `Test_Master_1, pci_image_base + 12, 32'h1234_5678, 4'h0, 1, `Test_Target_Retry_On) ;
        end
        begin
            if ( target_mem_image == 1 )
                pci_transaction_progress_monitor( pci_image_base + 12, `BC_MEM_WRITE, 0, 0, 1'b1, 1'b0, 1'b0, ok ) ;
            else
                pci_transaction_progress_monitor( pci_image_base + 12, `BC_IO_WRITE, 0, 0, 1'b1, 1'b0, 1'b0, ok ) ;
        end
        join

        do_pause( 1 ) ;

        // complete a read in WB Slave Unit
        wishbone_master.wb_single_read( read_data, write_flags, read_status ) ;
        if ( read_status`CYC_ACTUAL_TRANSFER !== 1 )
        begin
            $display("Transaction ordering test failed! Single delayed read request was not processed as expected by WB Slave unit! Time %t ", $time) ;
            test_fail("Single delayed read request was not processed as expected by WB Slave unit") ;
            ok = 0 ;
        end

        if ( read_status`READ_DATA !== write_data`WRITE_DATA )
        begin
            $display("Transaction ordering test failed! Single delayed read through WB Slave didn't return expected data! Time %t ", $time) ;
            test_fail("Single delayed read through WB Slave didn't return expected data") ;
            ok = 0 ;
        end

        // wait for statuses to be propagated from one side of bridge to another
        repeat( 4 )
            @(posedge pci_clock) ;

        // post a write through pci_target_unit - must go through since only delayed read completion in pci target unit is present
        fork
        begin
            if ( target_mem_image == 1 )
                PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                            pci_image_base + 12, 32'hAAAA_AAAA, 4'h0,
                            1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                            `Test_Devsel_Medium, `Test_Target_Normal_Completion);
            else
                PCIU_IO_WRITE( `Test_Master_1, pci_image_base + 12, 32'hAAAA_AAAA, 4'h0, 1, `Test_Target_Normal_Completion) ;

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( pci_image_base + 12, 1'b1, 1, 1'b1, ok ) ;
            if ( ok !== 1 )
            begin
                $display("Transaction ordering test failed! WB Master started invalid transaction or none at all on WB bus after single write was posted! Time %t ", $time) ;
                test_fail("WB Master failed to start valid transaction on WB bus after single write was posted") ;
            end
        end
        join

        // finish the last read in PCI Target Unit
        if ( target_mem_image == 1 )
            PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                          pci_image_base + 12, 32'hDEAD_BEAF,
                          1, 8'h7_0, `Test_One_Zero_Target_WS,
                          `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
            PCIU_IO_READ
             (
                `Test_Master_1,
                pci_image_base + 12,
                32'hDEAD_BEAF,
                4'h0,
                1,
                `Test_Target_Normal_Completion
             );

         do_pause( 1 ) ;

         error_monitor_done = 1 ;
    end
    join

    if ( ok )
        test_ok ;

end
endtask // transaction_ordering

task pci_transaction_progress_monitor ;
    input [31:0] address ;
    input [3:0]  bus_command ;
    input [31:0] num_of_transfers ;
    input [31:0] num_of_cycles ;
    input check_transfers ;
    input check_cycles ;
    input doing_fast_back_to_back ;
    output ok ;
    reg in_use ;
    integer deadlock_counter ;
    integer transfer_counter ;
    integer cycle_counter ;
    integer deadlock_max_val ;
begin:main

    if ( in_use === 1 )
    begin
        $display("pci_transaction_progress_monitor task re-entered! Time %t ", $time) ;
        ok = 0 ;
        disable main ;
    end

    // approximate number of cycles on WB bus for maximum transaction length
    deadlock_max_val = tb_init_waits + 100 +
                       `WBW_DEPTH *
                       (tb_subseq_waits + 1 +
                       `ifdef REGISTER_WBS_OUTPUTS
                       1) ;
                       `else
                       0) ;
                       `endif

    // time used for maximum transaction length on WB
    deadlock_max_val = deadlock_max_val * ( 1/`WB_FREQ ) ;

    // maximum pci clock cycles
    `ifdef PCI33
        deadlock_max_val = deadlock_max_val / 30 + 100 ;
    `else
        deadlock_max_val = deadlock_max_val / 15 + 100 ;
    `endif

    in_use = 1 ;
    ok     = 1 ;

    fork
    begin:wait_start

        deadlock_counter = 0 ;

        @(posedge pci_clock) ;

        if ( doing_fast_back_to_back !== 1 )
        begin
            while ( (FRAME !== 1) && (deadlock_counter < deadlock_max_val) )
            begin
                if ( (IRDY == 0) && ((TRDY == 0) || (STOP == 0)) )
                    deadlock_counter = 0 ;
                else
                    deadlock_counter = deadlock_counter + 1 ;
                @(posedge pci_clock) ;
            end
            if ( FRAME !== 1 )
            begin
                $display("pci_transaction_progress_monitor task waited for 1000 cycles for previous transaction to complete! Time %t ", $time) ;
                in_use = 0 ;
                ok     = 0 ;
                disable main ;
            end
        end

        deadlock_counter = 0 ;
        while ( (FRAME !== 0) && (deadlock_counter < deadlock_max_val) )
        begin
            deadlock_counter = deadlock_counter + 1 ;
            @(posedge pci_clock) ;
        end

        if ( FRAME !== 0 )
        begin
            $display("pci_transaction_progress_monitor task waited for 1000 cycles for transaction to start! Time %t ", $time) ;
            in_use = 0 ;
            ok     = 0 ;
            disable main ;
        end
    end //wait_start

    begin:addr_bc_monitor

        @(posedge pci_clock) ;

        if ( doing_fast_back_to_back !== 1 )
        begin
            while ( FRAME !== 1 )
                @(posedge pci_clock) ;
        end

        while( FRAME !== 0 )
            @(posedge pci_clock) ;

        // Address during Interrupt Acknowledge cycle has don't care value - don't check it
        if ( bus_command !== `BC_IACK )
        begin
            if ( AD !== address )
            begin
                $display("pci_transaction_progress_monitor detected unexpected address on PCI! Time %t ", $time) ;
                $display("Expected address = %h, detected address = %h ", address, AD) ;
                ok = 0 ;
            end
        end

        if ( CBE !== bus_command )
        begin
            $display("pci_transaction_progress_monitor detected unexpected bus command on PCI! Time %t ", $time) ;
            $display("Expected bus command = %b, detected bus command = %b", bus_command, CBE) ;
            ok = 0 ;
        end
    end //addr_bc_monitor

    begin:transfer_checker
        transfer_counter = 0 ;

        @(posedge pci_clock) ;

        if ( doing_fast_back_to_back !== 1 )
        begin
            while ( FRAME !== 1 )
                @(posedge pci_clock) ;
        end

        while( FRAME !== 0 )
            @(posedge pci_clock) ;

        while( FRAME === 0 )
        begin
            if ( (IRDY === 0) && (TRDY === 0) && (DEVSEL === 0) )
                transfer_counter = transfer_counter + 1 ;
            @(posedge pci_clock) ;
        end

        while( (IRDY === 0) && (TRDY === 1) && (STOP === 1) )
        begin
            @(posedge pci_clock) ;
        end

        if ( (TRDY === 0) && (DEVSEL === 0) )
                transfer_counter = transfer_counter + 1 ;

        if ( check_transfers === 1 )
        begin
            if ( transfer_counter !== num_of_transfers )
            begin
                $display("pci_transaction_progress_monitor detected unexpected transaction! Time %t ", $time) ;
                $display("Expected transfers in transaction = %d, actual transfers = %d ", num_of_transfers, transfer_counter) ;
                ok = 0 ;
            end
        end
    end //transfer_checker
    begin:cycle_checker
        if ( check_cycles )
        begin
            cycle_counter = 0 ;
            @(posedge pci_clock) ;

            if ( doing_fast_back_to_back !== 1)
            begin
                while ( FRAME !== 1 )
                    @(posedge pci_clock) ;
            end

            while( FRAME !== 0 )
                @(posedge pci_clock) ;

            while ( (FRAME !== 1) && (cycle_counter < num_of_cycles) )
            begin
                cycle_counter = cycle_counter + 1 ;
                @(posedge pci_clock) ;
            end

            if ( FRAME !== 1 )
            begin
                while ((FRAME === 0) && (MAS0_GNT === 0))
                    @(posedge pci_clock) ;

                if ( FRAME !== 1 )
                begin
                    while( (IRDY === 1) || ((TRDY === 1) && (STOP === 1)) )
                        @(posedge pci_clock) ;

                    @(posedge pci_clock) ;

                    if ( FRAME !== 1 )
                    begin
                        $display("pci_transaction_progress_monitor detected invalid transaction length! Time %t ", $time) ;
                        $display("Possibility of wrong operation in latency timer logic exists!") ;
                        ok = 0 ;
                    end
                end
            end
        end
    end // cycle_checker
    join

    in_use = 0 ;
end
endtask //pci_transaction_progress_monitor

reg CYC_O_previous ;
always@(posedge wb_clock or posedge reset)
begin
    if ( reset )
        CYC_O_previous <= #1 1'b0 ;
    else
        CYC_O_previous <= #1 CYC_O ;
end

task wb_transaction_progress_monitor ;
    input [31:0] address ;
    input        write ;
    input [31:0] num_of_transfers ;
    input check_transfers ;
    output ok ;
    reg in_use ;
    integer deadlock_counter ;
    integer transfer_counter ;
    integer deadlock_max_val ;
begin:main
    if ( in_use === 1 )
    begin
        $display("wb_transaction_progress_monitor task re-entered! Time %t ", $time) ;
        ok = 0 ;
        disable main ;
    end

    // number of cycles on WB bus for maximum transaction length
    deadlock_max_val = 4 - tb_init_waits + 100 +
                       `PCIW_DEPTH *
                       (4 - tb_subseq_waits + 1) ;

    // time used for maximum transaction length on PCI
    `ifdef PCI33
    deadlock_max_val = deadlock_max_val * ( 30 ) + 100 ;
    `else
    deadlock_max_val = deadlock_max_val * ( 15 ) + 100 ;
    `endif

    // maximum wb clock cycles
    deadlock_max_val = deadlock_max_val / (1/`WB_FREQ) ;

    in_use = 1 ;
    ok     = 1 ;

    fork
    begin:wait_start
        deadlock_counter = 0 ;
        @(posedge wb_clock) ;
        while ( (CYC_O !== 0 && CYC_O_previous !== 0) && (deadlock_counter < deadlock_max_val) )
        begin
        	if ((!STB_O) || (!ACK_I && !RTY_I && !ERR_I))
            	deadlock_counter = deadlock_counter + 1 ;
            else
            	deadlock_counter = 0;
            @(posedge wb_clock) ;
        end
        if ( CYC_O !== 0 && CYC_O_previous !== 0)
        begin
            $display("wb_transaction_progress_monitor task waited for 1000 cycles for previous transaction to complete! Time %t ", $time) ;
            in_use = 0 ;
            ok     = 0 ;
            disable main ;
        end

        deadlock_counter = 0 ;
        while ( (CYC_O !== 1) && (deadlock_counter < deadlock_max_val) )
        begin
            deadlock_counter = deadlock_counter + 1 ;
            @(posedge wb_clock) ;
        end

        if ( CYC_O !== 1 )
        begin
            $display("wb_transaction_progress_monitor task waited for 1000 cycles for transaction to start! Time %t ", $time) ;
            in_use = 0 ;
            ok     = 0 ;
            disable main ;
        end
    end //wait_start
    begin:addr_monitor
        @(posedge wb_clock) ;
        while ( CYC_O !== 0 && CYC_O_previous !== 0)
            @(posedge wb_clock) ;

        while( CYC_O !== 1 )
            @(posedge wb_clock) ;

        while (STB_O !== 1 )
            @(posedge wb_clock) ;

        if ( WE_O !== write )
        begin
            $display("wb_transaction_progress_monitor detected unexpected transaction on WB bus! Time %t ", $time) ;
            if ( write !== 1 )
                $display("Expected read transaction, WE_O signal value %b ", WE_O) ;
            else
                $display("Expected write transaction, WE_O signal value %b ", WE_O) ;
        end

        if ( ADR_O !== address )
        begin
            $display("wb_transaction_progress_monitor detected unexpected address on WB bus! Time %t ", $time) ;
            $display("Expected address = %h, detected address = %h ", address, ADR_O) ;
            ok = 0 ;
        end
    end
    begin:transfer_checker
        transfer_counter = 0 ;
        @(posedge wb_clock) ;
        while ( CYC_O !== 0 && CYC_O_previous !== 0)
            @(posedge wb_clock) ;

        while( CYC_O !== 1 )
            @(posedge wb_clock) ;

        while( CYC_O === 1 )
        begin
            if ( (STB_O === 1) && (ACK_I === 1) )
                transfer_counter = transfer_counter + 1 ;
            @(posedge wb_clock) ;
        end

        if ( check_transfers === 1 )
        begin
            if ( transfer_counter !== num_of_transfers )
            begin
                $display("wb_transaction_progress_monitor detected unexpected transaction! Time %t ", $time) ;
                $display("Expected transfers in transaction = %d, actual transfers = %d ", num_of_transfers, transfer_counter) ;
                ok = 0 ;
            end
        end
    end //transfer_checker
    join

    in_use = 0 ;
end
endtask // wb_transaction_progress_monitor

// this task is the same as wb_transaction_progress_monitor. It is used, when two tasks must run in parallel,
// so they are not re-entered
task wb_transaction_progress_monitor_backup ;
    input [31:0] address ;
    input        write ;
    input [31:0] num_of_transfers ;
    input check_transfers ;
    output ok ;
    reg in_use ;
    integer deadlock_counter ;
    integer transfer_counter ;
    integer deadlock_max_val ;
begin:main
    if ( in_use === 1 )
    begin
        $display("wb_transaction_progress_monitor task re-entered! Time %t ", $time) ;
        ok = 0 ;
        disable main ;
    end

    // number of cycles on WB bus for maximum transaction length
    deadlock_max_val = 4 - tb_init_waits + 100 +
                       `PCIW_DEPTH *
                       (4 - tb_subseq_waits + 1) ;

    // time used for maximum transaction length on PCI
    `ifdef PCI33
    deadlock_max_val = deadlock_max_val * ( 30 ) + 100 ;
    `else
    deadlock_max_val = deadlock_max_val * ( 15 ) + 100 ;
    `endif

    // maximum wb clock cycles
    deadlock_max_val = deadlock_max_val / (1/`WB_FREQ) ;

    in_use = 1 ;
    ok     = 1 ;

    fork
    begin:wait_start
        deadlock_counter = 0 ;
        @(posedge wb_clock) ;
        while ( (CYC_O !== 0) && (deadlock_counter < deadlock_max_val) )
        begin
        	if ((!STB_O) || (!ACK_I && !RTY_I && !ERR_I))
            	deadlock_counter = deadlock_counter + 1 ;
            else
            	deadlock_counter = 0;
            @(posedge wb_clock) ;
        end
        if ( CYC_O !== 0 )
        begin
            $display("wb_transaction_progress_monitor task waited for 1000 cycles for previous transaction to complete! Time %t ", $time) ;
            in_use = 0 ;
            ok     = 0 ;
            disable main ;
        end

        deadlock_counter = 0 ;
        while ( (CYC_O !== 1) && (deadlock_counter < deadlock_max_val) )
        begin
            deadlock_counter = deadlock_counter + 1 ;
            @(posedge wb_clock) ;
        end

        if ( CYC_O !== 1 )
        begin
            $display("wb_transaction_progress_monitor task waited for 1000 cycles for transaction to start! Time %t ", $time) ;
            in_use = 0 ;
            ok     = 0 ;
            disable main ;
        end
    end //wait_start
    begin:addr_monitor
        @(posedge wb_clock) ;
        while ( CYC_O !== 0 )
            @(posedge wb_clock) ;

        while( CYC_O !== 1 )
            @(posedge wb_clock) ;

        while (STB_O !== 1 )
            @(posedge wb_clock) ;

        if ( WE_O !== write )
        begin
            $display("wb_transaction_progress_monitor detected unexpected transaction on WB bus! Time %t ", $time) ;
            if ( write !== 1 )
                $display("Expected read transaction, WE_O signal value %b ", WE_O) ;
            else
                $display("Expected write transaction, WE_O signal value %b ", WE_O) ;
        end

        if ( ADR_O !== address )
        begin
            $display("wb_transaction_progress_monitor detected unexpected address on WB bus! Time %t ", $time) ;
            $display("Expected address = %h, detected address = %h ", address, ADR_O) ;
            ok = 0 ;
        end
    end
    begin:transfer_checker
        transfer_counter = 0 ;
        @(posedge wb_clock) ;
        while ( CYC_O !== 0 )
            @(posedge wb_clock) ;

        while( CYC_O !== 1 )
            @(posedge wb_clock) ;

        while( CYC_O === 1 )
        begin
            if ( (STB_O === 1) && (ACK_I === 1) )
                transfer_counter = transfer_counter + 1 ;
            @(posedge wb_clock) ;
        end

        if ( check_transfers === 1 )
        begin
            if ( transfer_counter !== num_of_transfers )
            begin
                $display("wb_transaction_progress_monitor detected unexpected transaction! Time %t ", $time) ;
                $display("Expected transfers in transaction = %d, actual transfers = %d ", num_of_transfers, transfer_counter) ;
                ok = 0 ;
            end
        end
    end //transfer_checker
    join

    in_use = 0 ;
end
endtask // wb_transaction_progress_monitor_backup

task wb_transaction_stop ;
    input [31:0] num_of_transfers ;
    integer transfer_counter ;
begin:main
    begin:transfer_checker
        transfer_counter = 0 ;
        @(posedge wb_clock) ;
        while ( CYC_O !== 0 )
            @(posedge wb_clock) ;

        while( CYC_O !== 1 )
            @(posedge wb_clock) ;

        if ( (STB_O === 1) && (ACK_I === 1) )
            transfer_counter = transfer_counter + 1 ;

        while( (transfer_counter < num_of_transfers) && (CYC_O === 1) )
        begin
            @(posedge wb_clock) ;
            if ( (STB_O === 1) && (ACK_I === 1) )
                transfer_counter = transfer_counter + 1 ;
        end
    end //transfer_checker
end
endtask // wb_transaction_stop

task musnt_respond ;
    output ok ;
    reg in_use ;
    integer i ;
begin:main
    if ( in_use === 1 )
    begin
        $display("Testbench error! Task musnt_repond re-entered! Time %t ", $time) ;
        #20 $stop ;
        ok = 0 ;
        disable main ;
    end

    in_use = 1 ;
    ok = 1 ;

    fork
    begin:wait_start
        @(negedge FRAME) ;
        disable count ;
    end
    begin:count
        i = 0 ;
        while ( i < 1000 )
        begin
            @(posedge pci_clock) ;
            i = i + 1 ;
        end
        $display("Error! Something is wrong! Task musnt_respond waited for 1000 cycles for transaction to start!") ;
        ok = 0 ;
        disable wait_start ;
    end
    join

    @(posedge pci_clock) ;
    while ( FRAME === 0 && ok )
    begin
        if ( DEVSEL !== 1 )
        begin
            ok = 0 ;
        end
        @(posedge pci_clock) ;
    end

    while ( IRDY === 0 && ok )
    begin
        if ( DEVSEL !== 1 )
        begin
            ok = 0 ;
        end
        @(posedge pci_clock) ;
    end
    in_use = 0 ;
end
endtask

function [31:0] wb_to_pci_addr_convert ;
    input [31:0] wb_address ;
    input [31:0] translation_address ;
    input [31:0] translate ;

    reg   [31:0] temp_address ;
begin
    if ( translate !== 1 )
    begin
        temp_address = wb_address & {{(`WB_NUM_OF_DEC_ADDR_LINES){1'b1}}, {(32 - `WB_NUM_OF_DEC_ADDR_LINES){1'b0}}} ;
    end
    else
    begin
        temp_address = {translation_address[31:(32 - `WB_NUM_OF_DEC_ADDR_LINES)], {(32 - `WB_NUM_OF_DEC_ADDR_LINES){1'b0}}} ;
    end

    temp_address = temp_address | (wb_address & {{(`WB_NUM_OF_DEC_ADDR_LINES){1'b0}}, {(32 - `WB_NUM_OF_DEC_ADDR_LINES){1'b1}}}) ;
    wb_to_pci_addr_convert = temp_address ;
end
endfunction //wb_to_pci_addr_convert

task find_pci_devices ;
    integer device_num ;
    reg     found ;
    reg [11:0] pci_ctrl_offset ;
    reg ok ;
    reg [31:0] data ;
begin:main
    pci_ctrl_offset = 12'h004 ;

    // enable master & target operation
    config_write( pci_ctrl_offset, 32'h0000_0007, 4'h1, ok) ;

    if ( ok !== 1 )
    begin
        $display("Couldn't enable master! PCI device search cannot proceede! Time %t ", $time) ;
        $stop ;
        disable main ;
    end
    // find all possible devices on pci bus by performing configuration cycles
    for ( device_num = 0 ; device_num <= 20 ; device_num = device_num + 1 )
    begin
        find_device ( device_num, found ) ;

        // check pci status register - if device is not present, Received Master Abort bit must be set
        config_read( pci_ctrl_offset, 4'hF, data ) ;

        if ( (data[29] !== 0) && (found !== 0) )
    begin
            $display( "Time %t ", $time ) ;
            $display( "Target responded to Configuration cycle, but Received Master Abort bit was set!") ;
            $display( "Value read from device status register: %h ", data[31:16] ) ;
            #20 $stop ;
        end

        if ( (data[29] !== 1) && (found !== 1) )
        begin
            $display( "Time %t ", $time ) ;
            $display( "Target didn't respond to Configuration cycle, but Received Master Abort bit was not set!") ;
            $display( "Value read from device status register: %h ", data[31:16] ) ;
            #20 $stop ;
        end

        // clear Master Abort status if set
        if ( data[29] !== 0 )
        begin
            config_write( pci_ctrl_offset, 32'hFFFF_0000, 4'b1000, ok) ;
        end
    end
end //main
endtask //find_pci_devices

task find_device ;
    input [31:0] device_num ;
    output  found ;

    reg [31:0] read_data ;
begin
    found = 1'b0 ;

    configuration_cycle_read ( 8'h00, device_num[4:0], 3'h0, 6'h00, 2'h0, 4'hF, read_data) ;
    if ( read_data == 32'hFFFF_FFFF)
        $display("Device %d not present on PCI bus!", device_num) ;
    else
    begin
        $display("Device %d with device id 0x%h and vendor id 0x%h found on PCI bus!", device_num, read_data[31:16], read_data[15:0]) ;
        found = 1'b1 ;
    end
end
endtask //find_device

/*task set_bridge_parameters ;
    reg [11:0] current_offset ;
    reg [2:0] result ;
    reg [31:0] write_data ;
begin
    // set burst size
    // set latency timer
    current_offset = 12'h00C ;
    // set burst size to 16 and latency timer to 8
    write_data     = {24'h0000_08, system_burst_size} ;
    config_write(current_offset, write_data, 4'b1111) ;

    // set io image
    current_offset = {4'b0001, `W_IMG_CTRL1_ADDR, 2'b00};
    write_data = 32'h0000_000_3 ;
    config_write(current_offset, write_data, 4'b1111) ;


    current_offset = { 4'b0001, `W_BA1_ADDR, 2'b00 } ;
    write_data = 32'h0001_000_1 ;
    config_write(current_offset, write_data, 4'b1111) ;

    current_offset = { 4'b0001, `W_AM1_ADDR, 2'b00 } ;
    write_data = 32'hFFFF_0000 ;
    config_write(current_offset, write_data, 4'b1111) ;

    // set memory image
    current_offset = { 4'b0001, `W_IMG_CTRL2_ADDR, 2'b00 } ;
    write_data = 32'h0000_000_7 ;
    config_write(current_offset, write_data, 4'b1111) ;

    current_offset = { 4'b0001, `W_BA2_ADDR, 2'b00 } ;
    write_data = 32'h0002_000_0 ;
    config_write(current_offset, write_data, 4'b1111) ;

    current_offset = { 4'b0001, `W_TA2_ADDR, 2'b00 } ;
    write_data = 32'h0001_0000 ;
    config_write(current_offset, write_data, 4'b1111) ;

    current_offset = { 4'b0001, `W_AM2_ADDR, 2'b00 } ;
    write_data = 32'hFFFF_0000 ;
    config_write(current_offset, write_data, 4'b1111) ;

    // set parameters for bridge's target unit
    // image control 0
    current_offset = { 4'b0001, `P_IMG_CTRL0_ADDR, 2'b00 } ;
    write_data     = 32'h0000_0002 ;
    config_write(current_offset, write_data, 4'b0001) ;

    // base_address 0
    current_offset = { 4'b0001, `P_BA0_ADDR, 2'b00 } ;
    write_data      = 32'h2000_0000 ;
    config_write(current_offset, write_data, 4'b1111) ;

    // address mask 0
    current_offset = { 4'b0001, `P_AM0_ADDR, 2'b00 } ;
    write_data     = 32'hFFFF_F000 ;
    config_write(current_offset, write_data, 4'b1111) ;

    // command register - enable response to io and mem space and PCI master
    current_offset = 12'h004 ;
    write_data     = 32'h0000_0007 ; // enable device's memory and io access response !
    config_write(current_offset, write_data, 4'b1111) ;
end
endtask // set_bridge_parameters
*/

task configuration_cycle_write ;
    input [7:0]  bus_num ;
    input [4:0]  device_num ;
    input [2:0]  func_num ;
    input [5:0]  reg_num ;
    input [1:0]  type ;
    input [3:0]  byte_enables ;
    input [31:0] data ;

    `ifdef HOST
    reg `WRITE_STIM_TYPE write_data ;
    reg `WB_TRANSFER_FLAGS write_flags ;
    reg `WRITE_RETURN_TYPE write_status ;
    `endif

    reg [31:0] write_address ;
    reg [31:0] temp_var ;
    reg in_use ;
    reg ok ;
begin:main

    if ( in_use === 1 )
    begin
        $display(" Task conf_write re-entered! Time %t ", $time ) ;
        disable main ;
    end

    if ( device_num > 20 )
    begin
        $display("Configuration cycle generation only supports access to 21 devices!") ;
        disable main ;
    end

    in_use = 1 ;

    if ( type )
        write_address = { 8'h00, bus_num, device_num, func_num, reg_num, type } ;
    else
    begin
        write_address = 0 ;
        write_address[10:0] = { func_num, reg_num, type } ;
        write_address[11 + device_num] = 1'b1 ;
    end

fork
begin
    `ifdef HOST
    // setup write flags
    write_flags                    = 0 ;
    write_flags`INIT_WAITS         = tb_init_waits ;
    write_flags`SUBSEQ_WAITS       = tb_subseq_waits ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    temp_var                                     = { `WB_CONFIGURATION_BASE, 12'h000 } ;
    temp_var[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] = 0 ;

    write_data`WRITE_ADDRESS  = temp_var + { 4'h1, `CNF_ADDR_ADDR, 2'b00 } ;
    write_data`WRITE_DATA     = { 8'h00, bus_num, device_num, func_num, reg_num, type } ;
    write_data`WRITE_SEL      = 4'hF ;
    write_data`WRITE_TAG_STIM = 0 ;

    wishbone_master.wb_single_write(write_data, write_flags, write_status) ;

    // check if write succeeded
    if (write_status`CYC_ACTUAL_TRANSFER !== 1)
    begin
        $display("Configuration cycle generation failed! Couldn't write to configuration address register! Time %t ", $time) ;
        $stop ;
    end

    // write to configuration data register
    write_flags`WB_TRANSFER_AUTO_RTY = 1 ;

    write_data`WRITE_ADDRESS = temp_var + {4'b0001, `CNF_DATA_ADDR, 2'b00} ;
    write_data`WRITE_DATA    = data ;
    write_data`WRITE_SEL     = byte_enables ;

    wishbone_master.wb_single_write(write_data, write_flags, write_status) ;

    if (write_status`CYC_ACTUAL_TRANSFER !== 1)
    begin
        $display("Configuration cycle generation failed! Time %t ", $time) ;
        $stop ;
    end

    `else
    `ifdef GUEST

     if ( type )
         write_address = { 8'h00, bus_num, device_num, func_num, reg_num, type } ;
     else
     begin
         write_address = 0 ;
         write_address[10:0] = { func_num, reg_num, type } ;
         write_address[11 + device_num] = 1'b1 ;
     end
     PCIU_CONFIG_WRITE ("CFG_WRITE ", `Test_Master_2,
                 write_address,
                 data, ~byte_enables,
                 1, `Test_No_Master_WS, `Test_No_Target_WS,
                 `Test_Devsel_Medium, `Test_Target_Normal_Completion);
     do_pause(1) ;
    `endif
    `endif
end
begin
    pci_transaction_progress_monitor( write_address, `BC_CONF_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
end
join

    in_use = 0 ;
end
endtask // configuration_cycle_write

task configuration_cycle_read ;
    input [7:0]  bus_num ;
    input [4:0]  device_num ;
    input [2:0]  func_num ;
    input [5:0]  reg_num ;
    input [1:0]  type ;
    input [3:0]  byte_enables ;
    output [31:0] data ;

    reg `READ_STIM_TYPE read_data ;
    reg `WB_TRANSFER_FLAGS  flags ;
    reg `READ_RETURN_TYPE   read_status ;

    reg `WRITE_STIM_TYPE   write_data ;
    reg `WRITE_RETURN_TYPE write_status ;

    reg [31:0] read_address ;
    reg in_use ;

    reg [31:0] temp_var ;
    reg master_check_data_prev ;
begin:main

    if ( in_use === 1 )
    begin
        $display("configuration_cycle_read task re-entered! Time %t ", $time) ;
        data = 32'hxxxx_xxxx ;
        disable main ;
    end

    if ( device_num > 20 )
    begin
        $display("Configuration cycle generation only supports access to 20 devices!") ;
        data = 32'hxxxx_xxxx ;
        disable main ;
    end

    in_use = 1 ;

    `ifdef HOST
    // setup flags
    flags = 0 ;

    temp_var                                     = { `WB_CONFIGURATION_BASE, 12'h000 } ;
    temp_var[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] = 0 ;

    write_data`WRITE_ADDRESS  = temp_var + { 4'h1, `CNF_ADDR_ADDR, 2'b00 } ;
    write_data`WRITE_DATA     = { 8'h00, bus_num, device_num, func_num, reg_num, type } ;
    write_data`WRITE_SEL      = 4'hF ;
    write_data`WRITE_TAG_STIM = 0 ;

    wishbone_master.wb_single_write(write_data, flags, write_status) ;

    // check if write succeeded
    if (write_status`CYC_ACTUAL_TRANSFER !== 1)
    begin
        $display("Configuration cycle generation failed! Couldn't write to configuration address register! Time %t ", $time) ;
        $stop ;
        data = 32'hFFFF_FFFF ;
        disable main ;
    end

    // read from configuration data register
    // setup flags for wb master to handle retries
    flags`WB_TRANSFER_AUTO_RTY = 1 ;

    read_data`READ_ADDRESS  = temp_var + {4'b0001, `CNF_DATA_ADDR, 2'b00} ;
    read_data`READ_SEL      = 4'hF ;
    read_data`READ_TAG_STIM = 0 ;

    wishbone_master.wb_single_read(read_data, flags, read_status) ;

    // check if read succeeded
    if (read_status`CYC_ACTUAL_TRANSFER !== 1)
    begin
        $display("Configuration cycle generation failed! Configuration read not processed correctly by the bridge! Time %t ", $time) ;
        $stop ;
        data = 32'hFFFF_FFFF ;
        disable main ;
    end


    data = read_status`READ_DATA ;
    `else
    `ifdef GUEST
     master_check_data_prev = master1_check_received_data ;
     if ( type )
         read_address = { 8'h00, bus_num, device_num, func_num, reg_num, type } ;
     else
     begin
         read_address = 0 ;
         read_address[10:0] = { func_num, reg_num, type } ;
         read_address[11 + device_num] = 1'b1 ;
     end

     fork
     begin
         PCIU_CONFIG_READ ("CFG_READ  ", `Test_Master_1,
                 read_address,
                 data, ~byte_enables,
                 1, `Test_No_Master_WS, `Test_No_Target_WS,
                 `Test_Devsel_Medium, `Test_Target_Normal_Completion);
         do_pause(1) ;
     end
     begin
         @(master1_received_data_valid) ;
         data = master1_received_data ;
     end
     join

    master1_check_received_data = master_check_data_prev ;
    `endif
    `endif

    in_use = 0 ;

end //main
endtask // configuration_cycle_read

task display_warning;
    input [31:0] error_address ;
    input [31:0] expected_data ;
    input [31:0] actual ;
begin
    $display("Read from address %h produced wrong result! \nExpected value was %h\tread value was %h!", error_address, expected_data, actual) ;
end
endtask // display warning

/*############################################################################
PCI TARGET UNIT tasks (some general tasks from WB SLAVE UNIT also used)
=====================
############################################################################*/

// Task reslease the PCI bus for 'delay' clocks
task do_pause;
  input  [15:0] delay;
  reg    [15:0] cnt;
  begin
    test_start <= 1'b0;  // no device is allowed to take this
    for (cnt = 16'h0000; cnt[15:0] < delay[15:0]; cnt[15:0] = cnt[15:0] + 16'h0001)
    begin
      if (~pci_reset_comb)
      begin
           @ (negedge pci_ext_clk or posedge pci_reset_comb) ;
      end
      `NO_ELSE;
    end
  end
endtask // do_pause

// Reference task for using pci_behavioral_master! (from Blue Beaver)
task DO_REF;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [3:0] command;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [PCI_BUS_CBE_RANGE:0] byte_enables_l;
  input  [9:0] size;
  input   make_addr_par_error, make_data_par_error;
  input  [7:0] master_wait_states;
  input  [7:0] target_wait_states;
  input  [1:0] target_devsel_speed;
  input   fast_back_to_back;
  input  [2:0] target_termination;
  input   expect_master_abort;
  reg     waiting;
  begin
// Cautiously wait for previous command to be done
    for (waiting = test_accepted_l_int; waiting != 1'b0; waiting = waiting)
    begin
      if (~pci_reset_comb && (test_accepted_l_int == 1'b0))
      begin
        if (~pci_reset_comb)
        begin
             @ (negedge pci_ext_clk or posedge pci_reset_comb) ;
        end
        `NO_ELSE;
      end
      else
      begin
        waiting = 1'b0;  // ready to do next command
      end
    end
    next_test_name[79:0] <= name[79:0];
    test_master_number <= master_number[2:0];
    test_address[PCI_BUS_DATA_RANGE:0] <= address[PCI_BUS_DATA_RANGE:0];
    test_command[3:0] <= command[3:0] ;
    test_data[PCI_BUS_DATA_RANGE:0] <= data[PCI_BUS_DATA_RANGE:0];
    test_byte_enables_l[PCI_BUS_CBE_RANGE:0] <= byte_enables_l[PCI_BUS_CBE_RANGE:0];
    test_size <= size;
    test_make_addr_par_error <= make_addr_par_error;
    test_make_data_par_error <= make_data_par_error;
    test_master_initial_wait_states <= 4 - tb_init_waits ;
    test_master_subsequent_wait_states <= 4 - tb_subseq_waits ;
    test_target_initial_wait_states <= target_wait_states[7:4];
    test_target_subsequent_wait_states <= target_wait_states[3:0];
    test_target_devsel_speed <= target_devsel_speed[1:0];
    test_fast_back_to_back <= fast_back_to_back;
    test_target_termination <= target_termination[2:0];
    test_expect_master_abort <= expect_master_abort;
    test_start <= 1'b1;
    if (~pci_reset_comb)
    begin
      @ (negedge pci_ext_clk or posedge pci_reset_comb) ;
    end
    `NO_ELSE;
// wait for new command to start
    for (waiting = 1'b1; waiting != 1'b0; waiting = waiting)
    begin
      if (~pci_reset_comb && (test_accepted_l_int == 1'b1))
      begin
        if (~pci_reset_comb) @ (negedge pci_ext_clk or posedge pci_reset_comb) ;
      end
      else
      begin
        waiting = 1'b0;  // ready to do next command
      end
    end
  end
endtask // DO_REF

// Use Macros defined in pci_defines.vh as paramaters

// DO_REF (name[79:0], master_number[2:0],
//          address[PCI_FIFO_DATA_RANGE:0], command[3:0],
//          data[PCI_FIFO_DATA_RANGE:0], byte_enables_l[PCI_FIFO_CBE_RANGE:0], size[3:0],
//          make_addr_par_error, make_data_par_error,
//          master_wait_states[8:0], target_wait_states[8:0],
//          target_devsel_speed[1:0], fast_back_to_back,
//          target_termination[2:0],
//          expect_master_abort);
//
// Example:
//      DO_REF ("CFG_R_MA_0", `Test_Master_1, 32'h12345678, `Config_Read,
//                32'h76543210, `Test_All_Bytes, `Test_No_Data,
//                `Test_No_Addr_Perr, `Test_No_Data_Perr, `Test_One_Zero_Master_WS,
//                `Test_One_Zero_Target_WS, `Test_Devsel_Medium, `Test_Fast_B2B,
//                `Test_Target_Normal_Completion, `Test_Master_Abort);

// Access a location with no high-order bits set, assuring that no device responds
task PCIU_CONFIG_READ_MASTER_ABORT;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [9:0] size;
  begin
    DO_REF (name[79:0], master_number[2:0], `NO_DEVICE_IDSEL_ADDR,
               PCI_COMMAND_CONFIG_READ, 32'h76543210, `Test_All_Bytes, size[9:0],
              `Test_Addr_Perr, `Test_Data_Perr, `Test_One_Zero_Master_WS,
              `Test_One_Zero_Target_WS, `Test_Devsel_Medium, `Test_No_Fast_B2B,
              `Test_Target_Normal_Completion, `Test_Expect_Master_Abort);
  end
endtask // PCIU_CONFIG_READ_MASTER_ABORT

// Access a location with no high-order bits set, assuring that no device responds
task PCIU_CONFIG_WRITE_MASTER_ABORT;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [9:0] size;
  begin
    DO_REF (name[79:0], master_number[2:0], `NO_DEVICE_IDSEL_ADDR,
               PCI_COMMAND_CONFIG_WRITE, 32'h76543210, `Test_All_Bytes, size[9:0],
              `Test_Addr_Perr, `Test_Data_Perr, `Test_One_Zero_Master_WS,
              `Test_One_Zero_Target_WS, `Test_Devsel_Medium, `Test_No_Fast_B2B,
              `Test_Target_Normal_Completion, `Test_Expect_Master_Abort);
  end
endtask // PCIU_CONFIG_WRITE_MASTER_ABORT

// Access a location with no high-order bits set, assuring that no device responds
task PCIU_MEM_READ_MASTER_ABORT;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [9:0] size;
  begin
    DO_REF (name[79:0], master_number[2:0], `NO_DEVICE_IDSEL_ADDR,
               PCI_COMMAND_MEMORY_READ, 32'h76543210, `Test_All_Bytes, size[9:0],
              `Test_Addr_Perr, `Test_Data_Perr, 8'h0_0,
              `Test_One_Zero_Target_WS, `Test_Devsel_Medium, `Test_No_Fast_B2B,
              `Test_Target_Normal_Completion, `Test_Expect_Master_Abort);
  end
endtask // PCIU_MEM_READ_MASTER_ABORT

// Access a location with no high-order bits set, assuring that no device responds
task PCIU_MEM_WRITE_MASTER_ABORT;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [9:0] size;
  begin
    DO_REF (name[79:0], master_number[2:0], `NO_DEVICE_IDSEL_ADDR,
               PCI_COMMAND_MEMORY_WRITE, 32'h76543210, `Test_All_Bytes, size[9:0],
              `Test_Addr_Perr, `Test_Data_Perr, 8'h0_0,
              `Test_One_Zero_Target_WS, `Test_Devsel_Medium, `Test_No_Fast_B2B,
              `Test_Target_Normal_Completion, `Test_Expect_Master_Abort);
  end
endtask // PCIU_MEM_WRITE_MASTER_ABORT

// Do variable length transfers with various paramaters
task PCIU_CONFIG_READ;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [3:0] be ;
  input  [9:0] size;
  input  [7:0] master_wait_states;
  input  [7:0] target_wait_states;
  input  [1:0] target_devsel_speed;
  input  [2:0] target_termination;
  begin
    DO_REF (name[79:0], master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_CONFIG_READ, data[PCI_BUS_DATA_RANGE:0], ~be,
              size[9:0], `Test_No_Addr_Perr, `Test_No_Data_Perr,
              8'h0_0, target_wait_states[7:0],
              target_devsel_speed[1:0], `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_CONFIG_READ

task PCIU_CONFIG_WRITE;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [3:0] be ;
  input  [9:0] size;
  input  [7:0] master_wait_states;
  input  [7:0] target_wait_states;
  input  [1:0] target_devsel_speed;
  input  [2:0] target_termination;
  begin
    DO_REF (name[79:0], master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_CONFIG_WRITE, data[PCI_BUS_DATA_RANGE:0], be,
              size[9:0], `Test_No_Addr_Perr, `Test_No_Data_Perr,
              8'h0_0, target_wait_states[7:0],
              target_devsel_speed[1:0], `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_CONFIG_WRITE

task PCIU_READ;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [3:0] command;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [3:0] byte_en;
  input  [9:0] size;
  input  [7:0] master_wait_states;
  input  [7:0] target_wait_states;
  input  [1:0] target_devsel_speed;
  input  [2:0] target_termination;
  reg    [79:0] name;
  begin
    if (command == `BC_MEM_READ)
        name = "MEM_READ  " ;
    else if (command == `BC_MEM_READ_LN)
        name = "MEM_RD_LN " ;
    else if (command == `BC_MEM_READ_MUL )
        name = "MEM_RD_MUL" ;
    else
        name = "WRONG_READ" ;
    DO_REF (name[79:0], master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              command[3:0], data[PCI_BUS_DATA_RANGE:0], byte_en[3:0],
              size[9:0], `Test_No_Addr_Perr, `Test_No_Data_Perr,
              8'h0_0, target_wait_states[7:0],
              target_devsel_speed[1:0], `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_READ

task PCIU_MEM_READ;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [9:0] size;
  input  [7:0] master_wait_states;
  input  [7:0] target_wait_states;
  input  [1:0] target_devsel_speed;
  input  [2:0] target_termination;
  begin

    DO_REF (name[79:0], master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_MEMORY_READ, data[PCI_BUS_DATA_RANGE:0], `Test_All_Bytes,
              size[9:0], `Test_No_Addr_Perr, `Test_No_Data_Perr,
              8'h0_0, target_wait_states[7:0],
              target_devsel_speed[1:0], `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_MEM_READ

task PCIU_IO_READ;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [3:0] byte_en ;
  input  [9:0] size;
  input  [2:0] target_termination ;
  begin

    DO_REF ("IO_READ   ", master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_IO_READ, data[PCI_BUS_DATA_RANGE:0], byte_en,
              size[9:0], `Test_No_Addr_Perr, `Test_No_Data_Perr,
              8'h0_0, `Test_One_Zero_Target_WS,
              `Test_Devsel_Medium, `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_IO_READ

task PCIU_IO_READ_MAKE_PERR;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [3:0] byte_en ;
  input  [9:0] size;
  input  [2:0] target_termination ;
  begin

    DO_REF ("IO_READ   ", master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_IO_READ, data[PCI_BUS_DATA_RANGE:0], byte_en,
              size[9:0], `Test_No_Addr_Perr, `Test_Data_Perr,
              8'h0_0, `Test_One_Zero_Target_WS,
              `Test_Devsel_Medium, `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_IO_READ_MAKE_PERR

task PCIU_MEM_READ_LN;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [9:0] size;
  input  [7:0] master_wait_states;
  input  [7:0] target_wait_states;
  input  [1:0] target_devsel_speed;
  input  [2:0] target_termination;
  begin
    DO_REF (name[79:0], master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_MEMORY_READ_LINE, data[PCI_BUS_DATA_RANGE:0], `Test_All_Bytes,
              size[9:0], `Test_No_Addr_Perr, `Test_No_Data_Perr,
              8'h0_0, target_wait_states[7:0],
              target_devsel_speed[1:0], `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_MEM_READ_LN

task PCIU_MEM_READ_MUL;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [9:0] size;
  input  [7:0] master_wait_states;
  input  [7:0] target_wait_states;
  input  [1:0] target_devsel_speed;
  input  [2:0] target_termination;
  begin
    DO_REF (name[79:0], master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_MEMORY_READ_MULTIPLE, data[PCI_BUS_DATA_RANGE:0], `Test_All_Bytes,
              size[9:0], `Test_No_Addr_Perr, `Test_No_Data_Perr,
              8'h0_0, target_wait_states[7:0],
              target_devsel_speed[1:0], `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_MEM_READ_MUL

task PCIU_MEM_READ_MAKE_PERR;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [9:0] size;
  input  [7:0] master_wait_states;
  input  [7:0] target_wait_states;
  input  [1:0] target_devsel_speed;
  input  [2:0] target_termination;
  begin
    DO_REF (name[79:0], master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_MEMORY_READ, data[PCI_BUS_DATA_RANGE:0], `Test_All_Bytes,
              size[9:0], `Test_No_Addr_Perr, `Test_Data_Perr,
              8'h0_0, target_wait_states[7:0],
              target_devsel_speed[1:0], `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_MEM_READ_MAKE_PERR

task PCIU_MEM_WRITE;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [3:0] byte_en;
  input  [9:0] size;
  input  [7:0] master_wait_states;
  input  [7:0] target_wait_states;
  input  [1:0] target_devsel_speed;
  input  [2:0] target_termination;
  begin
    DO_REF (name[79:0], master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_MEMORY_WRITE, data[PCI_BUS_DATA_RANGE:0], byte_en[3:0],
              size[9:0], `Test_No_Addr_Perr, `Test_No_Data_Perr,
              8'h0_0, target_wait_states[7:0],
              target_devsel_speed[1:0], `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_MEM_WRITE

task PCIU_IO_WRITE;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [3:0] byte_en;
  input  [9:0] size;
  input  [2:0] target_termination ;
  begin

    DO_REF ("IO_WRITE  ", master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_IO_WRITE, data[PCI_BUS_DATA_RANGE:0], byte_en[3:0],
              size[9:0], `Test_No_Addr_Perr, `Test_No_Data_Perr,
              8'h0_0, `Test_One_Zero_Target_WS,
              `Test_Devsel_Medium, `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_IO_WRITE

task PCIU_IO_WRITE_MAKE_PERR ;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [3:0] byte_en;
  input  [9:0] size;
  input  [2:0] target_termination ;
  begin

    DO_REF ("IO_WRITE  ", master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_IO_WRITE, data[PCI_BUS_DATA_RANGE:0], byte_en[3:0],
              size[9:0], `Test_No_Addr_Perr, `Test_Data_Perr,
              8'h0_0, `Test_One_Zero_Target_WS,
              `Test_Devsel_Medium, `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_IO_WRITE

task PCIU_MEM_WRITE_MAKE_SERR;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [9:0] size;
  input  [7:0] master_wait_states;
  input  [7:0] target_wait_states;
  input  [1:0] target_devsel_speed;
  input  [2:0] target_termination;
  begin
    DO_REF (name[79:0], master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_MEMORY_WRITE, data[PCI_BUS_DATA_RANGE:0], `Test_All_Bytes,
              size[9:0], `Test_Addr_Perr, `Test_No_Data_Perr,
              8'h0_0, target_wait_states[7:0],
              target_devsel_speed[1:0], `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_MEM_WRITE_MAKE_SERR

task PCIU_MEM_WRITE_MAKE_PERR;
  input  [79:0] name;
  input  [2:0] master_number;
  input  [PCI_BUS_DATA_RANGE:0] address;
  input  [PCI_BUS_DATA_RANGE:0] data;
  input  [9:0] size;
  input  [7:0] master_wait_states;
  input  [7:0] target_wait_states;
  input  [1:0] target_devsel_speed;
  input  [2:0] target_termination;
  begin
    DO_REF (name[79:0], master_number[2:0], address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_MEMORY_WRITE, data[PCI_BUS_DATA_RANGE:0], `Test_All_Bytes,
              size[9:0], `Test_No_Addr_Perr, `Test_Data_Perr,
              8'h0_0, target_wait_states[7:0],
              target_devsel_speed[1:0], `Test_No_Fast_B2B,
              target_termination[2:0], `Test_Expect_No_Master_Abort);
  end
endtask // PCIU_MEM_WRITE

/*--------------------------------------------------------------------------
Initialization CASES
--------------------------------------------------------------------------*/

// Initialize the basic Config Registers of the PCI bridge target device
task configure_bridge_target;
    reg [11:0] offset ;
    reg [31:0] data ;
    `ifdef HOST
    reg `WRITE_STIM_TYPE   write_data ;
    reg `WB_TRANSFER_FLAGS write_flags ;
    reg `WRITE_RETURN_TYPE write_status ;
    `else
    reg [PCI_BUS_DATA_RANGE:0] pci_conf_addr;
    reg [PCI_BUS_CBE_RANGE:0]  byte_enables;
    `endif

    reg [31:0] temp_var ;
begin
`ifdef HOST //  set Header
    offset  = 12'h4 ; // PCI Header Command register
    data    = 32'h0000_0007 ; // enable master & target operation

    write_flags                      = 0 ;
    write_flags`INIT_WAITS           = tb_init_waits ;
    write_flags`SUBSEQ_WAITS         = tb_subseq_waits ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    write_data`WRITE_ADDRESS  = { `WB_CONFIGURATION_BASE, offset } ;
    write_data`WRITE_SEL      = 4'h1 ;
    write_data`WRITE_TAG_STIM = 0 ;
    write_data`WRITE_DATA     = data ;

    next_test_name[79:0] <= "Init_Tar_R";

    $display(" bridge target - Enabling master and target operation!");
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
    end

    temp_var                                     = { `WB_CONFIGURATION_BASE, 12'h000 } ;
    temp_var[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] = 0 ;

    offset  = {4'h1, `P_BA0_ADDR, 2'b00} ; // PCI Base Address 0
    data    = Target_Base_Addr_R[0] ; // `TAR0_BASE_ADDR_0 = 32'h1000_0000

    write_data`WRITE_ADDRESS  = temp_var + offset ;
    write_data`WRITE_SEL      = 4'hf ;
    write_data`WRITE_TAG_STIM = 0 ;
    write_data`WRITE_DATA     = data ;

 `ifdef  NO_CNF_IMAGE
  `ifdef PCI_IMAGE0 //      set P_BA0

    $display(" bridge target - Setting base address P_BA0 to    32'h %h !", data);
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
    end
  `endif
 `else //      set P_BA0

    $display(" bridge target - Setting base address P_BA0 to    32'h %h !", data);
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
    end
 `endif

`else // GUEST, set Header, set P_BA0
    data            = 32'h0000_0007 ; // enable master & target operation
    byte_enables    = 4'hF ;
    $display(" bridge target - Enabling master and target operation!");
    configuration_cycle_write(0,             // bus number
                              0,             // device number
                              0,             // function number
                              1,             // register number
                              0,             // type of configuration cycle
                              byte_enables,  // byte enables
                              data           // data
                             ) ;

    data = Target_Base_Addr_R[0] ; // `TAR0_BASE_ADDR_0 = 32'h1000_0000
    $display(" bridge target - Setting base address P_BA0 to    32'h %h !", data);
    byte_enables = 4'hf ;
    configuration_cycle_write(0,             // bus number
                              0,             // device number
                              0,             // function number
                              4,             // register number
                              0,             // type of configuration cycle
                              byte_enables,  // byte enables
                              data           // data
                             ) ;

`endif
end
endtask // configure_bridge_target

// Initialize the basic Config Registers of the PCI bridge target device
task configure_bridge_target_base_addresses;
    reg [11:0] offset ;
    reg [31:0] data ;
    `ifdef HOST
    reg `WRITE_STIM_TYPE   write_data ;
    reg `WB_TRANSFER_FLAGS write_flags ;
    reg `WRITE_RETURN_TYPE write_status ;
    `else
    reg [PCI_BUS_DATA_RANGE:0] pci_conf_addr;
    reg [PCI_BUS_CBE_RANGE:0]  byte_enables;
    `endif

    reg [31:0] temp_var ;
begin
`ifdef HOST //  set Header
    offset  = 12'h4 ; // PCI Header Command register
    data    = 32'h0000_0007 ; // enable master & target operation

    write_flags                    = 0 ;
    write_flags`INIT_WAITS         = tb_init_waits ;
    write_flags`SUBSEQ_WAITS       = tb_subseq_waits ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    temp_var                                   = { `WB_CONFIGURATION_BASE, 12'h000 } ;
    temp_var[(31-`WB_NUM_OF_DEC_ADDR_LINES):0] = 0 ;

    write_data`WRITE_ADDRESS  = temp_var + offset ;
    write_data`WRITE_SEL      = 4'h1 ;
    write_data`WRITE_TAG_STIM = 0 ;
    write_data`WRITE_DATA     = data ;

    next_test_name[79:0] <= "Init_Tar_R";

    $display(" bridge target - Enabling master and target operation!");
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
    end

    offset  = {4'h1, `P_BA0_ADDR, 2'b00} ; // PCI Base Address 0
    data    = Target_Base_Addr_R[0] ; // `TAR0_BASE_ADDR_0 = 32'h1000_0000

    write_data`WRITE_ADDRESS  = temp_var + offset ;
    write_data`WRITE_SEL      = 4'hf ;
    write_data`WRITE_TAG_STIM = 0 ;
    write_data`WRITE_DATA     = data ;

 `ifdef  NO_CNF_IMAGE
  `ifdef PCI_IMAGE0 //      set P_BA0

    $display(" bridge target - Setting base address P_BA0 to    32'h %h !", data);
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
    end
  `endif
 `else //      set P_BA0

    $display(" bridge target - Setting base address P_BA0 to    32'h %h !", data);
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
    end
 `endif
    offset  = {4'h1, `P_BA1_ADDR, 2'b00} ; // PCI Base Address 1
    data    = Target_Base_Addr_R[1] ; // `TAR0_BASE_ADDR_1 = 32'h2000_0000

    write_data`WRITE_ADDRESS  = temp_var + offset ;
    write_data`WRITE_SEL      = 4'hf ;
    write_data`WRITE_TAG_STIM = 0 ;
    write_data`WRITE_DATA     = data ;

    $display(" bridge target - Setting base address P_BA1 to    32'h %h !", data);
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
    end
 `ifdef PCI_IMAGE2

    offset  = {4'h1, `P_BA2_ADDR, 2'b00} ; // PCI Base Address 2
    data    = Target_Base_Addr_R[2] ; // `TAR0_BASE_ADDR_2 = 32'h3000_0000

    write_data`WRITE_ADDRESS  = temp_var + offset ;
    write_data`WRITE_SEL      = 4'hf ;
    write_data`WRITE_TAG_STIM = 0 ;
    write_data`WRITE_DATA     = data ;

    $display(" bridge target - Setting base address P_BA2 to    32'h %h !", data);
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
    end
 `endif
 `ifdef PCI_IMAGE3
    offset  = {4'h1, `P_BA3_ADDR, 2'b00} ; // PCI Base Address 3
    data    = Target_Base_Addr_R[3] ; // `TAR0_BASE_ADDR_3 = 32'h4000_0000

    write_data`WRITE_ADDRESS  = temp_var + offset ;
    write_data`WRITE_SEL      = 4'hf ;
    write_data`WRITE_TAG_STIM = 0 ;
    write_data`WRITE_DATA     = data ;

    $display(" bridge target - Setting base address P_BA3 to    32'h %h !", data);
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
    end
 `endif
 `ifdef PCI_IMAGE4
    offset  = {4'h1, `P_BA4_ADDR, 2'b00} ; // PCI Base Address 4
    data    = Target_Base_Addr_R[4] ; // `TAR0_BASE_ADDR_4 = 32'h5000_0000

    write_data`WRITE_ADDRESS  = temp_var + offset ;
    write_data`WRITE_SEL      = 4'hf ;
    write_data`WRITE_TAG_STIM = 0 ;
    write_data`WRITE_DATA     = data ;

    $display(" bridge target - Setting base address P_BA4 to    32'h %h !", data);
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
    end
 `endif
 `ifdef PCI_IMAGE5
    offset  = {4'h1, `P_BA5_ADDR, 2'b00} ; // PCI Base Address 5
    data    = Target_Base_Addr_R[5] ; // `TAR0_BASE_ADDR_5 = 32'h6000_0000

    write_data`WRITE_ADDRESS  = temp_var + offset ;
    write_data`WRITE_SEL      = 4'hf ;
    write_data`WRITE_TAG_STIM = 0 ;
    write_data`WRITE_DATA     = data ;

    $display(" bridge target - Setting base address P_BA5 to    32'h %h !", data);
    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
    end
 `endif

`else // GUEST, set Header, set P_BA0
    data            = 32'h0000_0007 ; // enable master & target operation
    byte_enables    = 4'hF ;
    $display(" bridge target - Enabling master and target operation!");
    configuration_cycle_write(0,             // bus number
                              0,             // device number
                              0,             // function number
                              1,             // register number
                              0,             // type of configuration cycle
                              byte_enables,  // byte enables
                              data           // data
                             ) ;

    data = Target_Base_Addr_R[0] ; // `TAR0_BASE_ADDR_0 = 32'h1000_0000
    $display(" bridge target - Setting base address P_BA0 to    32'h %h !", data);
    byte_enables = 4'hf ;
    configuration_cycle_write(0,             // bus number
                              0,             // device number
                              0,             // function number
                              4,             // register number
                              0,             // type of configuration cycle
                              byte_enables,  // byte enables
                              data           // data
                             ) ;

    data = Target_Base_Addr_R[1] ; // `TAR0_BASE_ADDR_1 = 32'h2000_0000
    $display(" bridge target - Setting base address P_BA1 to    32'h %h !", data);
    byte_enables = 4'hf ;
    configuration_cycle_write(0,             // bus number
                              0,             // device number
                              0,             // function number
                              5,             // register number
                              0,             // type of configuration cycle
                              byte_enables,  // byte enables
                              data           // data
                             ) ;
 `ifdef PCI_IMAGE2
    data = Target_Base_Addr_R[2] ; // `TAR0_BASE_ADDR_2 = 32'h3000_0000
    $display(" bridge target - Setting base address P_BA2 to    32'h %h !", data);
    byte_enables = 4'hf ;
    configuration_cycle_write(0,             // bus number
                              0,             // device number
                              0,             // function number
                              6,             // register number
                              0,             // type of configuration cycle
                              byte_enables,  // byte enables
                              data           // data
                             ) ;
 `endif
 `ifdef PCI_IMAGE3
    data = Target_Base_Addr_R[3] ; // `TAR0_BASE_ADDR_3 = 32'h4000_0000
    $display(" bridge target - Setting base address P_BA3 to    32'h %h !", data);
    byte_enables = 4'hf ;
    configuration_cycle_write(0,             // bus number
                              0,             // device number
                              0,             // function number
                              7,             // register number
                              0,             // type of configuration cycle
                              byte_enables,  // byte enables
                              data           // data
                             ) ;
 `endif
 `ifdef PCI_IMAGE4
    data = Target_Base_Addr_R[4] ; // `TAR0_BASE_ADDR_4 = 32'h5000_0000
    $display(" bridge target - Setting base address P_BA4 to    32'h %h !", data);
    byte_enables = 4'hf ;
    configuration_cycle_write(0,             // bus number
                              0,             // device number
                              0,             // function number
                              8,             // register number
                              0,             // type of configuration cycle
                              byte_enables,  // byte enables
                              data           // data
                             ) ;
 `endif
 `ifdef PCI_IMAGE5
    data = Target_Base_Addr_R[5] ; // `TAR0_BASE_ADDR_5 = 32'h6000_0000
    $display(" bridge target - Setting base address P_BA5 to    32'h %h !", data);
    byte_enables = 4'hf ;
    configuration_cycle_write(0,             // bus number
                              0,             // device number
                              0,             // function number
                              9,             // register number
                              0,             // type of configuration cycle
                              byte_enables,  // byte enables
                              data           // data
                             ) ;
 `endif
`endif
end
endtask // configure_bridge_target_base_addresses

/*--------------------------------------------------------------------------
Test CASES
--------------------------------------------------------------------------*/

// function converts PCI address to WB with the same data as the pci_decoder does
function [31:0] pci_to_wb_addr_convert ;

    input [31:0] pci_address ;
    input [31:0] translation_address ;
    input [31:0] translate ;

    reg   [31:0] temp_address ;
begin
    if ( translate !== 1 )
    begin
        temp_address = pci_address & {{(`PCI_NUM_OF_DEC_ADDR_LINES){1'b1}}, {(32 - `PCI_NUM_OF_DEC_ADDR_LINES){1'b0}}} ;
    end
    else
    begin
        temp_address = {translation_address[31:(32 - `PCI_NUM_OF_DEC_ADDR_LINES)], {(32 - `PCI_NUM_OF_DEC_ADDR_LINES){1'b0}}} ;
    end

    temp_address = temp_address | (pci_address & {{(`PCI_NUM_OF_DEC_ADDR_LINES){1'b0}}, {(32 - `PCI_NUM_OF_DEC_ADDR_LINES){1'b1}}}) ;
    pci_to_wb_addr_convert = temp_address ;
end
endfunction // pci_to_wb_addr_convert

// Test normal write and read to WB slave
task test_normal_wr_rd;
  input  [2:0]  Master_ID;
  input  [PCI_BUS_DATA_RANGE:0] Address;
  input  [PCI_BUS_DATA_RANGE:0] Data;
  input  [3:0]  Be;
  input  [2:0]  Image_num;
  input  [9:0]  Set_size;
  input         Set_addr_translation;
  input         Set_prefetch_enable;
  input  [7:0]  Cache_lsize;
  input         Set_wb_wait_states;
  input         MemRdLn_or_MemRd_when_cache_lsize_read;

  reg    [31:0] rd_address;
  reg    [31:0] rd_data;
  reg    [3:0]  rd_be;
  reg    [11:0] addr_offset;
  reg    [31:0] read_data;
  reg           continue ;
  reg           ok   ;
  reg    [31:0] expect_address ;
  reg    [31:0] expect_rd_address ;
  reg           expect_we ;
  reg    [9:0]  expect_length_wr ;
  reg    [9:0]  expect_length_rd ;
  reg    [9:0]  expect_length_rd1 ;
  reg    [9:0]  expect_length_rd2 ;
  reg    [3:0]  use_rd_cmd ;
  integer       i ;
  reg           error_monitor_done ;
begin:main

    // enable ERROR reporting, because error must NOT be reported and address translation if required!
    $display("Setting the ERR_EN bit of PCI Error Control and Status register P_ERR_CS.");
    $display(" - errors will be reported, but they should not occur!");
    test_name = "CONFIGURE BRIDGE FOR NORMAL TARGET READ/WRITE" ;
    addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;
    config_write( addr_offset, 32'h0000_0001, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write P_ERR_CS register! Time %t ", $time);
        test_fail("write to P_ERR_CS register didn't succeede") ;
        disable main;
    end

    `ifdef  ADDR_TRAN_IMPL

    // set or clear address translation
    if (Set_addr_translation)
    begin
        $display("Setting the AT_EN bit of PCI Image Control register P_IMG_CTRL%d.", Image_num);
        $display(" - address translation will be performed!");
    end
    else
    begin
        $display("Clearing the AT_EN bit of PCI Image Control register P_IMG_CTRL%d.", Image_num);
        $display(" - address translation will not be performed!");
    end
    // set or clear pre-fetch enable
    if (Set_prefetch_enable)
    begin
        $display("Setting the PRF_EN bit of PCI Image Control register P_IMG_CTRL%d.", Image_num);
        $display(" - bursts can be performed!");
    end
    else
    begin
        $display("Clearing the PRF_EN bit of PCI Image Control register P_IMG_CTRL%d.", Image_num);
        $display(" - bursts can not be performed!");
    end

    addr_offset = {4'h1, `P_IMG_CTRL0_ADDR, 2'b00} + (12'h10 * Image_num);
    config_write( addr_offset, {29'h0, Set_addr_translation, Set_prefetch_enable, 1'b0}, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write P_IMG_CTRL%d register! Time %t ", Image_num, $time);
        test_fail("write to P_IMG_CTRL didn't succeede") ;
        disable main;
    end

    // predict the address and control signals on WB bus
    expect_address = pci_to_wb_addr_convert( Address, Target_Tran_Addr_R[Image_num], Set_addr_translation ) ;
    expect_we      = 1'b1 ; // WRITE

    `else

    // address translation is not implemented
    $display("Address translation is NOT implemented for PCI images!");
    // set or clear pre-fetch enable
    if (Set_prefetch_enable)
    begin
        $display("Setting the PRF_EN bit of PCI Image Control register P_IMG_CTRL%d.", Image_num);
        $display(" - bursts can be performed!");
    end
    else
    begin
        $display("Clearing the PRF_EN bit of PCI Image Control register P_IMG_CTRL%d.", Image_num);
        $display(" - bursts can not be performed!");
    end

    addr_offset = {4'h1, `P_IMG_CTRL0_ADDR, 2'b00} + (12'h10 * Image_num);
    config_write( addr_offset, {29'h0, 1'b0, Set_prefetch_enable, 1'b0}, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write P_IMG_CTRL%d register! Time %t ", Image_num, $time);
        test_fail("write to P_IMG_CTRL didn't succeede") ;
        disable main;
    end

    // predict the address and control signals on WB bus
    expect_address = Address ;
    expect_we      = 1'b1 ; // WRITE

    `endif

    // set WB SLAVE parameters
    if (Set_wb_wait_states)
        $display("Setting behavioral WB slave parameters: ACK termination and 3 WAIT states!");
    else
        $display("Setting behavioral WB slave parameters: ACK termination and 0 WAIT states!");
    // set response of WB SLAVE - ACK,    WAIT cycles,        RETRY cycles
    wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

    if ( Set_size > (`PCIW_DEPTH - 2) )
    begin
        expect_length_wr = `PCIW_DEPTH - 2 ;
    end
    else
    begin
        expect_length_wr = Set_size ;
    end
    // write through the PCI bridge to WB slave
    test_name = "NORMAL POSTED WRITE THROUGH PCI TARGET UNIT" ;
    $display("Memory write (%d words) through PCI bridge to WB slave!", expect_length_wr);

    fork
    begin
        PCIU_MEM_WRITE ("MEM_WRITE ", Master_ID[2:0],
                   Address[PCI_BUS_DATA_RANGE:0], Data[PCI_BUS_DATA_RANGE:0], Be[3:0],
                   expect_length_wr, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                   `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        do_pause( 1 ) ;
    end
    begin
       wb_transaction_progress_monitor( expect_address, expect_we, expect_length_wr, 1'b1, ok ) ;
       if ( ok !== 1 )
           test_fail("WB Master started invalid transaction or none at all after Target write was posted") ;
       else
           test_ok ;
    end
    join

    // predict the address and control signals on WB bus
    expect_we      = 1'b0 ; // READ

    // read from the PCI bridge (from WB slave) - PCI master behavioral checks if the same data was written
    $display("Memory read through PCI bridge to WB slave!");

    if ( expect_length_wr == 1 )
    begin
    	if (Set_prefetch_enable)
        begin
            expect_length_rd1 = Cache_lsize ;
            expect_length_rd2 = 0 ;
    		// If PCI behavioral master must check received DATA
    		master2_check_received_data = 0 ;
		    master1_check_received_data = 0 ;
    	end
    	else
        begin
            expect_length_rd1 = 1 ;
            expect_length_rd2 = 0 ;
    		// If PCI behavioral master must check received DATA
    		master2_check_received_data = 1 ;
		    master1_check_received_data = 1 ;
        end
        use_rd_cmd = PCI_COMMAND_MEMORY_READ ;
        expect_length_rd  = expect_length_rd1 ;
    end
    else if ( expect_length_wr == (`PCIR_DEPTH - 1) )
    begin
        expect_length_rd1 = `PCIR_DEPTH - 1 ;
        expect_length_rd2 = 0 ;
        use_rd_cmd = PCI_COMMAND_MEMORY_READ_MULTIPLE ;
        expect_length_rd  = expect_length_rd1 ;
		// If PCI behavioral master must check received DATA
    	master2_check_received_data = 1 ;
	    master1_check_received_data = 1 ;
    end
    else if ( expect_length_wr > (`PCIR_DEPTH - 1) )
    begin
        expect_length_rd1 = `PCIR_DEPTH - 1 ;
        expect_length_rd2 = expect_length_wr - expect_length_rd1 ;
        use_rd_cmd = PCI_COMMAND_MEMORY_READ_MULTIPLE ;
        expect_length_rd  = expect_length_rd1 ;
   		// If PCI behavioral master must check received DATA
   		master2_check_received_data = 1 ;
	    master1_check_received_data = 1 ;
    end
    else
    begin
        if ( Cache_lsize >= (`PCIR_DEPTH - 1) )
        begin
            expect_length_rd1 = `PCIR_DEPTH - 1 ;
            expect_length_rd2 = 0 ;
            use_rd_cmd = PCI_COMMAND_MEMORY_READ_MULTIPLE ;
            expect_length_rd  = expect_length_rd1 ;
    		// If PCI behavioral master must check received DATA
    		master2_check_received_data = 0 ;
		    master1_check_received_data = 0 ;
        end
        else
        begin
            if ( expect_length_wr > Cache_lsize )
            begin
                expect_length_rd1 = Cache_lsize ;
                expect_length_rd2 = expect_length_wr - Cache_lsize ;
                if ( MemRdLn_or_MemRd_when_cache_lsize_read )
                    use_rd_cmd = PCI_COMMAND_MEMORY_READ_LINE ;
                else
                    use_rd_cmd = PCI_COMMAND_MEMORY_READ ;
                expect_length_rd  = expect_length_rd1 ;
    			// If PCI behavioral master must check received DATA
	    		master2_check_received_data = 1 ;
			    master1_check_received_data = 1 ;
            end
            else
            begin
                expect_length_rd1 = Cache_lsize ;
                expect_length_rd2 = 0 ;
                if ( MemRdLn_or_MemRd_when_cache_lsize_read )
                    use_rd_cmd = PCI_COMMAND_MEMORY_READ_LINE ;
                else
                    use_rd_cmd = PCI_COMMAND_MEMORY_READ ;
                expect_length_rd  = expect_length_wr ;
				if ( expect_length_wr == Cache_lsize )
	    		begin
	    			// If PCI behavioral master must check received DATA
    				master2_check_received_data = 1 ;
				    master1_check_received_data = 1 ;
				end
				else
				begin
	    			// If PCI behavioral master must check received DATA
    				master2_check_received_data = 0 ;
				    master1_check_received_data = 0 ;
            	end
            end
        end
    end

    rd_address = Address[PCI_BUS_DATA_RANGE:0];
    expect_rd_address = expect_address ;
    rd_data[31:0] = Data[31:0];
    rd_be[3:0] = Be[3:0];

    test_name = "NORMAL MEMORY READ THROUGH PCI TARGET UNIT" ;
    while (expect_length_rd2 > 0)
    begin
        // do read
        $display("Read %d words!", expect_length_rd);

        fork
        begin
            PCIU_READ (Master_ID[2:0], rd_address[PCI_BUS_DATA_RANGE:0],
                        use_rd_cmd, rd_data[PCI_BUS_DATA_RANGE:0], rd_be[3:0],
                        expect_length_rd1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Start_Delayed_Read);

            wb_transaction_stop( expect_length_rd - 1) ;

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( expect_rd_address, expect_we, expect_length_rd1, 1'b1, ok ) ;
            if ( ok !== 1 )
                test_fail("WB Master started invalid transaction or none at all after Target delayed read was requested") ;

            repeat( 3 )
                @(posedge pci_clock) ;

            PCIU_READ (Master_ID[2:0], rd_address[PCI_BUS_DATA_RANGE:0],
                        use_rd_cmd, rd_data[PCI_BUS_DATA_RANGE:0], rd_be[3:0],
                        expect_length_rd, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Normal_Completion);

            do_pause( 1 ) ;
            while ( FRAME === 0 )
                @(posedge pci_clock) ;

            while ( IRDY === 0 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event1 ;
        end
        begin:monitor_error_event1
            error_monitor_done = 0 ;
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        // increasing the starting address for PCI master and for WB transaction monitor
        rd_address = rd_address + (4 * expect_length_rd) ;
        expect_rd_address = expect_rd_address + (4 * expect_length_rd) ;
        // change the expected read data and byte enables as they are changed in the PCI behavioral master
        rd_data[31:24] = Data[31:24] + expect_length_rd;
        rd_data[23:16] = Data[23:16] + expect_length_rd;
        rd_data[15: 8] = Data[15: 8] + expect_length_rd;
        rd_data[ 7: 0] = Data[ 7: 0] + expect_length_rd;
        for (i=0; i<expect_length_rd; i=i+1)
            rd_be[3:0] = {Be[2:0], Be[3]};

        // set parameters for next read
        if ( expect_length_rd2 == 1 )
        begin
    		if (Set_prefetch_enable)
	        begin
	            expect_length_rd1 = Cache_lsize ;
	            expect_length_rd2 = 0 ;
    			// If PCI behavioral master must check received DATA
    			master2_check_received_data = 0 ;
			    master1_check_received_data = 0 ;
	    	end
	    	else
	        begin
	            expect_length_rd1 = 1 ;
	            expect_length_rd2 = 0 ;
    			// If PCI behavioral master must check received DATA
    			master2_check_received_data = 1 ;
			    master1_check_received_data = 1 ;
	        end
            use_rd_cmd = PCI_COMMAND_MEMORY_READ ;
            expect_length_rd  = expect_length_rd1 ;
        end
        else if ( expect_length_rd2 == (`PCIR_DEPTH - 1) )
        begin
            expect_length_rd1 = `PCIR_DEPTH - 1 ;
            expect_length_rd2 = 0 ;
            use_rd_cmd = PCI_COMMAND_MEMORY_READ_MULTIPLE ;
            expect_length_rd  = expect_length_rd1 ;
   			// If PCI behavioral master must check received DATA
   			master2_check_received_data = 1 ;
		    master1_check_received_data = 1 ;
        end
        else if ( expect_length_rd2 > (`PCIR_DEPTH - 1) )
        begin
            expect_length_rd1 = `PCIR_DEPTH - 1 ;
            expect_length_rd2 = expect_length_rd2 - expect_length_rd1 ;
            use_rd_cmd = PCI_COMMAND_MEMORY_READ_MULTIPLE ;
            expect_length_rd  = expect_length_rd1 ;
   			// If PCI behavioral master must check received DATA
   			master2_check_received_data = 1 ;
		    master1_check_received_data = 1 ;
        end
        else
        begin
            if ( Cache_lsize >= (`PCIR_DEPTH - 1) )
            begin
                expect_length_rd1 = `PCIR_DEPTH - 1 ;
                expect_length_rd2 = 0 ;
                use_rd_cmd = PCI_COMMAND_MEMORY_READ_MULTIPLE ;
                expect_length_rd  = expect_length_rd1 ;
    			// If PCI behavioral master must check received DATA
    			master2_check_received_data = 0 ;
			    master1_check_received_data = 0 ;
            end
            else
            begin
                if ( expect_length_rd2 > Cache_lsize )
                begin
                    expect_length_rd1 = Cache_lsize ;
                    expect_length_rd2 = expect_length_rd2 - Cache_lsize ;
                    if ( MemRdLn_or_MemRd_when_cache_lsize_read )
                        use_rd_cmd = PCI_COMMAND_MEMORY_READ_LINE ;
                    else
                        use_rd_cmd = PCI_COMMAND_MEMORY_READ ;
                    expect_length_rd  = expect_length_rd1 ;
    				// If PCI behavioral master must check received DATA
    				master2_check_received_data = 1 ;
				    master1_check_received_data = 1 ;
                end
                else
                begin
                    expect_length_rd  = expect_length_rd2 ;
                    expect_length_rd1 = Cache_lsize ;
                    expect_length_rd2 = 0 ;
                    if ( MemRdLn_or_MemRd_when_cache_lsize_read )
                        use_rd_cmd = PCI_COMMAND_MEMORY_READ_LINE ;
                    else
                        use_rd_cmd = PCI_COMMAND_MEMORY_READ ;
					if ( expect_length_rd2 == Cache_lsize )
	    			begin
	    				// If PCI behavioral master must check received DATA
    					master2_check_received_data = 1 ;
					    master1_check_received_data = 1 ;
					end
					else
					begin
	    				// If PCI behavioral master must check received DATA
    					master2_check_received_data = 0 ;
					    master1_check_received_data = 0 ;
	            	end
                end
            end
        end
    end
    // do last read
    $display("Read %d words!", expect_length_rd);

    fork
    begin
        PCIU_READ (Master_ID[2:0], rd_address[PCI_BUS_DATA_RANGE:0],
                    use_rd_cmd, rd_data[PCI_BUS_DATA_RANGE:0], rd_be[3:0],
                    expect_length_rd1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                    `Test_Devsel_Medium, `Test_Target_Start_Delayed_Read);

        wb_transaction_stop(expect_length_rd - 1) ;
        do_pause( 1 ) ;
    end
    begin
        wb_transaction_progress_monitor( expect_rd_address, expect_we, expect_length_rd1, 1'b1, ok ) ;

        do_pause(3) ;
        PCIU_READ (Master_ID[2:0], rd_address[PCI_BUS_DATA_RANGE:0],
                    use_rd_cmd, rd_data[PCI_BUS_DATA_RANGE:0], rd_be[3:0],
                    expect_length_rd, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                    `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        do_pause(1) ;

        while ( FRAME === 0 )
            @(posedge pci_clock) ;

        while ( IRDY === 0 )
            @(posedge pci_clock) ;

        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event2 ;
    end
    begin:monitor_error_event2
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    join

    if ( ok )
        test_ok ;

    // Check that no ERRORs were reported
    test_name = "PCI ERROR STATUS AFTER NORMAL WRITE/READ" ;
    $display("Reading the ERR_SIG bit of PCI Error Control and Status register P_ERR_CS.");
    addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;
    config_read( addr_offset, 4'hF, read_data ) ;
    if ( read_data[8] !== 0 )
    begin
        $display("Image testing failed! Failed because ERROR was signaled, Time %t ", $time);
        test_fail("error status was set even though no errors occured on WB bus") ;
    end
    else
    begin
        $display("No error was signaled, as expected!");
        test_ok ;
    end

end // main
endtask // test_normal_wr_rd

// Test erroneous write to WB slave
task test_wb_error_wr;
  input  [2:0]  Master_ID;
  input  [PCI_BUS_DATA_RANGE:0] Address;
  input  [PCI_BUS_DATA_RANGE:0] Data;
  input  [3:0]  Be;
  input  [2:0]  Image_num;
  input  [9:0]  Set_size;
  input         Set_err_and_int_report;
  input         Set_wb_wait_states;
  input  [1:0]  Imm_BefLast_Last_error;

  reg    [11:0] addr_offset;
  reg    [31:0] read_data;
  reg           continue ;
  reg           ok   ;
  reg    [9:0]  expect_length ;
  reg    [31:0] expect_address ;
  reg    [0:0]  expect_we ;
  reg    [31:0] rd_address;
  reg    [31:0] rd_data;
  reg    [3:0]  rd_be;
  integer       i ;
begin:main
    if (Set_err_and_int_report)
    begin
        // enable ERROR reporting, because error must be reported and interrupt if required!
        $display("Setting the ERR_EN bit of PCI Error Control and Status register P_ERR_CS.");
        $display(" - errors will be reported when they will occur!");
        // enable INTERRUPT reporting, because error must be reported and interrupt if required!
        $display("Setting the PCI_EINT_EN and WB_EINT_EN bit of Interrupt Control register ICR.");
        $display(" - interrupt will be reported when error will occur!");
    end
    else
    begin
        // disable ERROR reporting, because error and interrupt must not be reported!
        $display("Clearing the ERR_EN bit of PCI Error Control and Status register P_ERR_CS.");
        $display(" - errors will NOT be reported when they will occur!");
        // disable INTERRUPT reporting, because error and interrupt must not be reported!
        $display("Clearing the PCI_EINT_EN and WB_EINT_EN bit of Interrupt Control register ICR.");
        $display(" - interrupt will NOT be reported when error will occur!");
    end
    // enable/disable ERROR reporting
    test_name = "CONFIGURE BRIDGE FOR ERROR TERMINATED WRITES THROUGH PCI TARGET UNIT" ;

    addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;
    config_write( addr_offset, {31'h0, Set_err_and_int_report}, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write P_ERR_CS register! Time %t ", $time);
        test_fail("PCI Error Control and Status register could not be written") ;
        disable main;
    end
    // enable/disable INTERRUPT reporting
    addr_offset = {4'h1, `ICR_ADDR, 2'b00} ;
    config_write( addr_offset, {29'h0, Set_err_and_int_report, Set_err_and_int_report, 1'b0}, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write ICR register! Time %t ", $time);
        test_fail("Interrupt Control register could not be written") ;
        disable main;
    end

    `ifdef  ADDR_TRAN_IMPL

    $display("Reading the AT_EN bit of Image Control register P_IMG_CTRL%d, for WB address prediction", Image_num);
    addr_offset = {4'h1, `P_IMG_CTRL0_ADDR, 2'b00} + (12'h10 * Image_num);
    config_read( addr_offset, 4'hF, read_data ) ;
    if ( read_data[2] !== 0 )
    begin
        $display("Address translation is set for PCI image%d!", Image_num);
        // predict the address and control signals on WB bus
        expect_address = pci_to_wb_addr_convert( Address, Target_Tran_Addr_R[Image_num], 1'b1 ) ;
        expect_we      = 1'b1 ; // WRITE
    end
    else
    begin
        $display("Address translation is NOT set for PCI image%d!", Image_num);
        // predict the address and control signals on WB bus
        expect_address = Address ;
        expect_we      = 1'b1 ; // WRITE
    end

    `else

    // address translation is not implemented
    $display("Address translation is NOT implemented for PCI images!");
    // predict the address and control signals on WB bus
    expect_address = Address ;
    expect_we      = 1'b1 ; // WRITE

    `endif

    if ( Set_size > (`PCIW_DEPTH - 2) )
    begin
        expect_length = `PCIW_DEPTH - 2 ;
    end
    else
    begin
        expect_length = Set_size ;
    end

    if ((Imm_BefLast_Last_error == 0) || (expect_length <= 2))
    begin
        $display("ERR termination with first data!");
        test_name = "POSTED WRITE THROUGH PCI TARGET ERROR TERMINATION ON WB ON FIRST TRANSFER" ;
    end
    else if (Imm_BefLast_Last_error == 1)
    begin
        $display("ERR termination before last data!");
        test_name = "POSTED WRITE THROUGH PCI TARGET ERROR TERMINATION ON WB ONE BEFORE LAST TRANSFER" ;
    end
    else
    begin
        $display("ERR termination with last data!");
        test_name = "POSTED WRITE THROUGH PCI TARGET ERROR TERMINATION ON WB ON LAST TRANSFER" ;
    end

    // write through the PCI bridge to WB slave
    $display("Memory write (%d words) through PCI bridge to WB slave!", expect_length);
    fork
    begin
        PCIU_MEM_WRITE ("MEM_WRITE ", Master_ID[2:0],
                   Address[PCI_BUS_DATA_RANGE:0], Data[PCI_BUS_DATA_RANGE:0], Be[3:0],
                   expect_length, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                   `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        do_pause( 1 ) ;
    end
    begin
        if ((Imm_BefLast_Last_error == 0) || (expect_length <= 2))
        begin
            wb_transaction_progress_monitor( expect_address, expect_we, 4'h0, 1'b1, ok ) ;
            if ( ok !== 1 )
                test_fail("WB Master started invalid transaction or none at all after Target write was posted") ;
        end
        else if (Imm_BefLast_Last_error == 1)
        begin
            wb_transaction_progress_monitor( expect_address, expect_we, (expect_length-2), 1'b1, ok ) ;
            if ( ok !== 1 )
                test_fail("WB Master started invalid transaction or none at all after Target write was posted") ;
        end
        else
        begin
            wb_transaction_progress_monitor( expect_address, expect_we, (expect_length-1), 1'b1, ok ) ;
            if ( ok !== 1 )
                test_fail("WB Master started invalid transaction or none at all after Target write was posted") ;
        end
    end
    begin
        if ((Imm_BefLast_Last_error == 0) || (expect_length <= 2))
            // set response of WB SLAVE - ERR,    WAIT cycles,        RETRY cycles
            wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);
        else if (Imm_BefLast_Last_error == 1)
        begin
            // set response of WB SLAVE - ACK,    WAIT cycles,        RETRY cycles
            wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);
            wb_transaction_stop(expect_length-2) ;
            // set response of WB SLAVE - ERR,    WAIT cycles,        RETRY cycles
            wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);
        end
        else
        begin
            // set response of WB SLAVE - ACK,    WAIT cycles,        RETRY cycles
            wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);
            wb_transaction_stop(expect_length-1) ;
            // set response of WB SLAVE - ERR,    WAIT cycles,        RETRY cycles
            wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);
        end
    end
    join

    if ( ok )
        test_ok ;

    if ((Imm_BefLast_Last_error == 0) || (expect_length <= 2))
    begin
        rd_data[31:0] = Data[31:0];
        rd_address[31:0] = expect_address;
        rd_be[3:0] = Be[3:0];
    end
    else if (Imm_BefLast_Last_error == 1)
    begin
        rd_data[31:24] = Data[31:24] + expect_length - 2;
        rd_data[23:16] = Data[23:16] + expect_length - 2;
        rd_data[15: 8] = Data[15: 8] + expect_length - 2;
        rd_data[ 7: 0] = Data[ 7: 0] + expect_length - 2;
        rd_address[31:0] = expect_address + ((expect_length - 2) * 4);
        rd_be[3:0] = Be[3:0];
        for (i=0; i<(expect_length-2); i=i+1)
            rd_be[3:0] = {rd_be[2:0], rd_be[3]};
    end
    else
    begin
        rd_data[31:24] = Data[31:24] + expect_length - 1;
        rd_data[23:16] = Data[23:16] + expect_length - 1;
        rd_data[15: 8] = Data[15: 8] + expect_length - 1;
        rd_data[ 7: 0] = Data[ 7: 0] + expect_length - 1;
        rd_address[31:0] = expect_address + ((expect_length - 1) * 4);
        rd_be[3:0] = Be[3:0];
        for (i=0; i<(expect_length-1); i=i+1)
            rd_be[3:0] = {rd_be[2:0], rd_be[3]};
    end

    master2_check_received_data = 0 ;
    master1_check_received_data = 0 ;

    // Check if ERRORs were reported
    $display("Reading the PCI Error Control and Status register P_ERR_CS.");
    addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;
    test_name = "PCI ERROR CONTROL AND STATUS REGISTER VALUE AFTER WRITE TERMINATED WITH ERROR" ;

    ok = 1 ;
    config_read( addr_offset, 4'hF, read_data ) ;
    if (( read_data[10:8] === 3'b001 ) && ( Set_err_and_int_report === 1'b1 ))
    begin
        $display("Error was signaled and reported, as expected!");
        if (read_data[31:28] === rd_be)
            $display("Byte enables written into P_ERR_CS register are as expected!");
        else
        begin
            $display("Byte enables written into P_ERR_CS register are NOT as expected!");
            test_fail("Byte enable information in PCI Error Control and Status register was wrong") ;
            ok = 0 ;
        end
        if (read_data[27:24] === PCI_COMMAND_MEMORY_WRITE)
            $display("Bus command written into P_ERR_CS register is as expected!");
        else
        begin
            $display("Bus command written into P_ERR_CS register is NOT as expected!");
            test_fail("Bus command information in PCI Error Control and Status register was wrong") ;
            ok = 0 ;
        end

        if ( ok )
            test_ok ;

        $display("Reading the PCI Error Data register P_ERR_DATA.");

        test_name = "PCI ERRONEOUS DATA REGISTER VALUE AFTER WRITE TERMINATED WITH ERROR ON WB" ;
        addr_offset = {4'h1, `P_ERR_DATA_ADDR, 2'b00} ;
        config_read( addr_offset, 4'hF, read_data ) ;
        if (read_data === rd_data)
        begin
            $display("Data written into P_ERR_DATA register is as expected!");
            test_ok ;
        end
        else
        begin
            $display("Data written into P_ERR_DATA register is NOT as expected!");
            test_fail("PCI Erroneous Data register value was wrong") ;
        end

        $display("Reading the PCI Error Address register P_ERR_ADDR.");

        test_name = "PCI ERRONEOUS ADDRESS REGISTER VALUE AFTER WRITE TERMINATED WITH ERROR ON WB" ;

        addr_offset = {4'h1, `P_ERR_ADDR_ADDR, 2'b00} ;
        config_read( addr_offset, 4'hF, read_data ) ;
        if (read_data === rd_address)
        begin
            $display("Address written into P_ERR_ADDR register is as expected!");
            test_ok ;
        end
        else
        begin
            $display("Address written into P_ERR_ADDR register is NOT as expected!");
            test_fail("PCI Erroneous Address register value was wrong") ;
        end
    end
    else if (( read_data[8] === 1'b0 ) && ( Set_err_and_int_report === 1'b0 ))
    begin
        $display("Error was signaled and not reported, as expected!");
        test_ok ;
    end
    else
    begin
        $display("Error was signaled and reported, as NOT expected!");
        test_fail("Error status bit was set event though error reporting was disabled") ;
    end

    // Check if Interrupts were reported
    $display("Reading the PCI_EINT and WB_EINT bit of Interrupt Status register ISR.");

    test_name = "INTERRUPT ASSERTION AND STATUS AFTER WRITE TERMINATED WITH ERROR ON WB" ;
    ok = 1 ;
    addr_offset = {4'h1, `ISR_ADDR, 2'b00} ;
    config_read( addr_offset, 4'hF, read_data ) ;
    if (( read_data[2:1] === 2'b10 ) && ( Set_err_and_int_report === 1'b1 ))
    begin
        $display("Interrupts was signaled and reported, as expected!");
    end
    else if (( read_data[2:1] === 2'b00 ) && ( Set_err_and_int_report === 1'b0 ))
    begin
        $display("Interrupts was signaled and not reported, as expected!");
    end
    else
    begin
        $display("Interrupt was signaled and reported, as NOT expected!");
        test_fail("PCI Error Interrupt status was set when not expected") ;
        ok = 0 ;
    end

    `ifdef HOST
    repeat( 4 )
        @(posedge wb_clock) ;

    if ( INT_O === Set_err_and_int_report )
        $display("Interrupt pin INT_O was correctly set to logic '%d'!", Set_err_and_int_report);
    else
    begin
        $display("Interrupt pin INT_O was NOT correctly set to logic '%d'!", Set_err_and_int_report);
        test_fail("Interrupt request didn't have expected value") ;
        ok = 0 ;
    end

    `else // GUEST
    repeat( 4 )
        @(posedge pci_clock) ;

    if ( INTA === !Set_err_and_int_report )
        $display("Interrupt pin INTA was correctly set to logic '%d'!", !Set_err_and_int_report);
    else
    begin
        $display("Interrupt pin INTA was NOT correctly set to logic '%d'!", !Set_err_and_int_report);
        test_fail("Interrupt request didn't have expected value") ;
        ok = 0 ;
    end

    `endif

    if ( ok )
        test_ok ;

    if (Set_err_and_int_report)
    begin
        test_name = "CLEARING STATUS BITS AFTER WRITE TERMINATED WITH ERROR ON WB" ;
        $display("Error and Interrupt must be cleared!");
        // clear  ERROR reporting bit
        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;
        config_write( addr_offset, 32'h0000_0100, 4'hF, ok) ;
        if ( ok !== 1 )
        begin
            $display("Image testing failed! Failed to write P_ERR_CS register! Time %t ", $time);
            test_fail("PCI Error Control and Status register could not be written to") ;
            disable main;
        end

        // clear INTERRUPT reporting bit
        addr_offset = {4'h1, `ISR_ADDR, 2'b00} ;
        config_write( addr_offset, 32'h0000_0004, 4'hF, ok) ;
        if ( ok !== 1 )
        begin
            $display("Image testing failed! Failed to write ISR register! Time %t ", $time);
            test_fail("Interrupt Status register could not be written to") ;
            disable main;
        end

        test_ok ;
        test_name = "INTERRUPT REQUEST FINISHED AFTER PCI ERROR INTERRUPT STATUS IS CLEARED" ;
        `ifdef HOST

        repeat(4)
            @(posedge wb_clock) ;
        if ( INT_O === 1'b0 )
        begin
            $display("Interrupt pin INT_O was correctly cleared!");
            test_ok ;
        end
        else
        begin
            $display("Interrupt pin INT_O was NOT correctly cleared!");
            test_fail("Interrupt pin was still asserted after Interrupt status was cleared") ;
            disable main;
        end

        `else // GUEST

        repeat(4)
            @(posedge pci_clock) ;
        if ( INTA === 1'b1 )
        begin
            $display("Interrupt pin INTA was correctly cleared!");
            test_ok ;
        end
        else
        begin
            $display("Interrupt pin INTA was NOT correctly cleared!");
            test_fail("Interrupt pin was still asserted after Interrupt status was cleared") ;
            disable main;
        end

        `endif

    end
    else
    begin
        $display("Error and Interrupt don't need to be cleared!");
    end
end // main
endtask // test_wb_error_wr

task test_wb_error_rd;
    reg    [11:0] addr_offset ;
    reg    [11:0] ctrl_offset ;
    reg    [11:0] ba_offset ;
    reg    [11:0] am_offset ;
    reg    [11:0] ta_offset ;
    reg    [31:0] read_data;
    reg           ok   ;
    reg    [9:0]  expect_length ;
    reg    [31:0] expect_address ;
    reg    [0:0]  expect_we ;
    reg    [31:0] rd_address;
    reg    [31:0] rd_data;
    reg    [3:0]  rd_be;
    integer       i ;
    reg           do_mem_aborts ;
    reg           do_io_aborts ;
    reg           error_monitor_done ;
begin:main
    // enable all error reporting mechanisms - during erroneous reads, errors should not be reported

    if ( target_mem_image !== -1 )
    begin
        do_mem_aborts = 1 ;

        if (target_mem_image === 1)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL1_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA1_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM1_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA1_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 2)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL2_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA2_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM2_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA2_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 3)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL3_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA3_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM3_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA3_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 4)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL4_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA4_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM4_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA4_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 5)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL5_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA5_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM5_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA5_ADDR, 2'b00} ;
        end
    end
    else
        do_mem_aborts = 0 ;

    if ( do_mem_aborts )
    begin
        test_name = "CONFIGURE BRIDGE FOR ERROR TERMINATED READS THROUGH PCI TARGET UNIT" ;

        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;
        config_write( addr_offset, 32'hFFFF_FFFF, 4'hF, ok) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write P_ERR_CS register! Time %t ", $time);
            test_fail("PCI Error Control and Status register could not be written") ;
            disable main;
        end

        // enable INTERRUPT reporting
        addr_offset = {4'h1, `ICR_ADDR, 2'b00} ;
        config_write( addr_offset, 32'h7FFF_FFFF, 4'hF, ok) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write ICR register! Time %t ", $time);
            test_fail("Interrupt Control register could not be written") ;
            disable main;
        end

        addr_offset = 12'h010 + (4*target_mem_image) ;

        config_write( addr_offset, Target_Base_Addr_R[target_mem_image], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write PCI Base Address register! Time %t ", $time);
            test_fail("PCI Base Address register could not be written") ;
            disable main;
        end

        // disable address translation and enable prefetch so read bursts can be performed
        config_write( ctrl_offset, 32'h0000_0002, 4'h1, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write PCI Image Control register! Time %t ", $time);
            test_fail("PCI Image Control register could not be written") ;
            disable main;
        end

        config_write( am_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write PCI Address Mask register! Time %t ", $time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main;
        end

        addr_offset = 12'h00C ;

        config_write( addr_offset, 32'h0000_04_04, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write PCI Latency Timer/Cache line size register! Time %t ", $time);
            test_fail("PCI Latency Timer/Cache line size register could not be written") ;
            disable main;
        end

        // disable PCI master data checking
        master1_check_received_data = 0 ;

        // set response of WB SLAVE - ERR,    WAIT cycles,        RETRY cycles
        wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);


        // do a single read error terminated on WB bus
        test_name = "SINGLE READ THROUGH PCI TARGET TERMINATED WITH ERROR ON WISHBONE" ;

        fork
        begin
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ, 32'h1234_5678, 4'hF,
                        1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Retry_On);

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( Target_Base_Addr_R[target_mem_image], 1'b0, 0, 1'b1, ok ) ;

            if ( ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus") ;

            do_pause(3) ;
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ, 32'h1234_5678, 4'hF,
                        1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Abort_On);
            do_pause(1) ;

            while ( FRAME === 0 )
                @(posedge pci_clock) ;

            while ( IRDY === 0 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event1 ;
        end
        begin:monitor_error_event1
            error_monitor_done = 0 ;
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( ok )
            test_ok ;
            
        @(posedge pci_clock) ;
        @(posedge pci_clock) ;
        @(posedge wb_clock) ;
        @(posedge wb_clock) ;

        test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER TARGET ABORT" ;
        addr_offset = 12'h004 ;
        config_read(addr_offset, 4'hF, read_data) ;
        ok = 1 ;
        if ( read_data[27] !== 1 )
        begin
            $display("Signaled Target Abort bit not set in PCI Device Status register after read terminated with Target Abort") ;
            test_fail("Signaled Target Abort bit was not set in PCI Device Status register after read terminated with Target Abort") ;
            ok = 0 ;
        end
        if ( read_data[28] !== 0 )
        begin
            $display("Received Target Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Target Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end
        if ( read_data[29] !== 0 )
        begin
            $display("Received Master Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Master Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end

        // clear statuses
        config_write(addr_offset, {2'b00, read_data[29:27], 27'h0}, 4'b11_00, ok) ;
        if ( !ok )
        begin
            test_fail("write to PCI Device Status register failed") ;
            $display("Couldn't write PCI Device Status register") ;
            disable main ;
        end

        if ( ok )
            test_ok ;

        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;

        ok = 1 ;
        test_name = "PCI ERROR CONTROL AND STATUS REGISTER VALUE AFTER READ THROUGH TARGET TERMINATED WITH TARGET ABORT" ;

        config_read(addr_offset, 4'hF, read_data) ;
        if (read_data[8] !== 0)
        begin
            $display("Error signaled bit set after Target Read Terminated with Error on WISHBONE. Error reporting should be done only for posted writes") ;
            test_fail("Error signaled bit set after Target Read Terminated with Error on WISHBONE") ;
            ok = 0 ;
        end
        else
            test_ok ;

        if ( ok !== 1 )
        begin
            config_write(addr_offset, read_data, 4'hF, ok) ;
            if ( !ok )
            begin
                test_fail("PCI Error Control and Status register could not be written") ;
                disable main ;
            end
        end

        wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);
        fork
        begin
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ, 32'h1234_5678, 4'hF,
                        2, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Retry_Before);

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( Target_Base_Addr_R[target_mem_image], 1'b0, 0, 1'b1, ok ) ;

            if ( ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus") ;

            do_pause(3) ;
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ, 32'h1234_5678, 4'hF,
                        2, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Abort_Before);
            do_pause(1) ;

            while ( FRAME === 0 )
                @(posedge pci_clock) ;

            while ( IRDY === 0 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event2 ;
        end
        begin:monitor_error_event2
            error_monitor_done = 0 ;
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( ok )
            test_ok ;

        @(posedge pci_clock) ;
        @(posedge pci_clock) ;
        @(posedge wb_clock) ;
        @(posedge wb_clock) ;

        test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER TARGET ABORT" ;
        addr_offset = 12'h004 ;
        config_read(addr_offset, 4'hF, read_data) ;
        ok = 1 ;
        if ( read_data[27] !== 1 )
        begin
            $display("Signaled Target Abort bit not set in PCI Device Status register after read terminated with Target Abort") ;
            test_fail("Signaled Target Abort bit was not set in PCI Device Status register after read terminated with Target Abort") ;
            ok = 0 ;
        end
        if ( read_data[28] !== 0 )
        begin
            $display("Received Target Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Target Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end
        if ( read_data[29] !== 0 )
        begin
            $display("Received Master Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Master Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end

        // clear statuses
        config_write(addr_offset, {2'b00, read_data[29:27], 27'h0}, 4'b11_00, ok) ;
        if ( !ok )
        begin
            test_fail("write to PCI Device Status register failed") ;
            $display("Couldn't write PCI Device Status register") ;
            disable main ;
        end

        if ( ok )
            test_ok ;

        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;

        ok = 1 ;
        test_name = "PCI ERROR CONTROL AND STATUS REGISTER VALUE AFTER READ THROUGH TARGET TERMINATED WITH TARGET ABORT" ;

        config_read(addr_offset, 4'hF, read_data) ;
        if (read_data[8] !== 0)
        begin
            $display("Error signaled bit set after Target Read Terminated with Error on WISHBONE. Error reporting should be done only for posted writes") ;
            test_fail("Error signaled bit set after Target Read Terminated with Error on WISHBONE") ;
            ok = 0 ;
        end
        else
            test_ok ;

        if ( ok !== 1 )
        begin
            config_write(addr_offset, read_data, 4'hF, ok) ;
            if ( !ok )
            begin
                test_fail("PCI Error Control and Status register could not be written") ;
                disable main ;
            end
        end

        // do a single read error terminated on WB bus
        test_name = "BURST READ THROUGH PCI TARGET TERMINATED WITH ERROR ON WISHBONE ON FIRST DATAPHASE" ;

        wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);

        fork
        begin
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                        2, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Retry_Before);

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( Target_Base_Addr_R[target_mem_image], 1'b0, 0, 1'b1, ok ) ;

            if ( ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus") ;

            do_pause(3) ;
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                        2, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Abort_Before);
            do_pause(1) ;

            while ( FRAME === 0 )
                @(posedge pci_clock) ;

            while ( IRDY === 0 )
                @(posedge pci_clock) ;

            if ( !error_monitor_done )
                disable monitor_error_event3 ;
        end
        begin:monitor_error_event3
            error_monitor_done = 0 ;
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( ok )
            test_ok ;

        @(posedge pci_clock) ;
        @(posedge pci_clock) ;
        @(posedge wb_clock) ;
        @(posedge wb_clock) ;

        test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER TARGET ABORT" ;
        addr_offset = 12'h004 ;
        config_read(addr_offset, 4'hF, read_data) ;
        ok = 1 ;
        if ( read_data[27] !== 1 )
        begin
            $display("Signaled Target Abort bit not set in PCI Device Status register after read terminated with Target Abort") ;
            test_fail("Signaled Target Abort bit was not set in PCI Device Status register after read terminated with Target Abort") ;
            ok = 0 ;
        end
        if ( read_data[28] !== 0 )
        begin
            $display("Received Target Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Target Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end
        if ( read_data[29] !== 0 )
        begin
            $display("Received Master Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Master Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end

        // clear statuses
        config_write(addr_offset, {2'b00, read_data[29:27], 27'h0}, 4'b11_00, ok) ;
        if ( !ok )
        begin
            test_fail("write to PCI Device Status register failed") ;
            $display("Couldn't write PCI Device Status register") ;
            disable main ;
        end

        if ( ok )
            test_ok ;

        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;

        ok = 1 ;
        test_name = "PCI ERROR CONTROL AND STATUS REGISTER VALUE AFTER READ THROUGH TARGET TERMINATED WITH TARGET ABORT" ;

        config_read(addr_offset, 4'hF, read_data) ;
        if (read_data[8] !== 0)
        begin
            $display("Error signaled bit set after Target Read Terminated with Error on WISHBONE. Error reporting should be done only for posted writes") ;
            test_fail("Error signaled bit set after Target Read Terminated with Error on WISHBONE") ;
            ok = 0 ;
        end
        else
            test_ok ;

        if ( ok !== 1 )
        begin
            config_write(addr_offset, read_data, 4'hF, ok) ;
            if ( !ok )
            begin
                test_fail("PCI Error Control and Status register could not be written") ;
                disable main ;
            end
        end

        test_name = "BURST READ THROUGH PCI TARGET TERMINATED WITH ERROR ON WISHBONE ON SECOND DATAPHASE" ;

        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

        fork
        begin
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                        2, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Retry_Before);

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( Target_Base_Addr_R[target_mem_image], 1'b0, 1, 1'b1, ok ) ;

            if ( ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus") ;

            do_pause(3) ;
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                        3, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Abort_Before);
            do_pause(1) ;

            while ( FRAME === 0 )
                @(posedge pci_clock) ;

            while ( IRDY === 0 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event4 ;
        end
        begin:monitor_error_event4
            error_monitor_done = 0 ;
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_stop( 1 ) ;
            wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);
        end
        join

        if ( ok )
            test_ok ;

        @(posedge pci_clock) ;
        @(posedge pci_clock) ;
        @(posedge wb_clock) ;
        @(posedge wb_clock) ;

        test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER TARGET ABORT" ;
        addr_offset = 12'h004 ;
        config_read(addr_offset, 4'hF, read_data) ;
        ok = 1 ;
        if ( read_data[27] !== 1 )
        begin
            $display("Signaled Target Abort bit not set in PCI Device Status register after read terminated with Target Abort") ;
            test_fail("Signaled Target Abort bit was not set in PCI Device Status register after read terminated with Target Abort") ;
            ok = 0 ;
        end
        if ( read_data[28] !== 0 )
        begin
            $display("Received Target Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Target Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end
        if ( read_data[29] !== 0 )
        begin
            $display("Received Master Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Master Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end

        // clear statuses
        config_write(addr_offset, {2'b00, read_data[29:27], 27'h0}, 4'b11_00, ok) ;
        if ( !ok )
        begin
            test_fail("write to PCI Device Status register failed") ;
            $display("Couldn't write PCI Device Status register") ;
            disable main ;
        end

        if ( ok )
            test_ok ;

        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;

        ok = 1 ;
        test_name = "PCI ERROR CONTROL AND STATUS REGISTER VALUE AFTER READ THROUGH TARGET TERMINATED WITH TARGET ABORT" ;

        config_read(addr_offset, 4'hF, read_data) ;
        if (read_data[8] !== 0)
        begin
            $display("Error signaled bit set after Target Read Terminated with Error on WISHBONE. Error reporting should be done only for posted writes") ;
            test_fail("Error signaled bit set after Target Read Terminated with Error on WISHBONE") ;
            ok = 0 ;
        end
        else
            test_ok ;

        if ( ok !== 1 )
        begin
            config_write(addr_offset, read_data, 4'hF, ok) ;
            if ( !ok )
            begin
                test_fail("PCI Error Control and Status register could not be written") ;
                disable main ;
            end
        end

        test_name = "BURST READ THROUGH PCI TARGET TERMINATED WITH ERROR ON WISHBONE ON LAST DATAPHASE" ;

        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

        fork
        begin
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                        2, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Retry_Before);

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( Target_Base_Addr_R[target_mem_image], 1'b0, 3, 1'b1, ok ) ;

            if ( ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus") ;

            do_pause(3) ;
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                        4, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Abort_On);
            do_pause(1) ;

            while ( FRAME === 0 )
                @(posedge pci_clock) ;

            while ( IRDY === 0 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event5 ;
        end
        begin:monitor_error_event5
            error_monitor_done = 0 ;
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_stop( 3 ) ;
            wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);
        end
        join

        if ( ok )
            test_ok ;

        @(posedge pci_clock) ;
        @(posedge pci_clock) ;
        @(posedge wb_clock) ;
        @(posedge wb_clock) ;

        test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER TARGET ABORT" ;
        addr_offset = 12'h004 ;
        config_read(addr_offset, 4'hF, read_data) ;
        ok = 1 ;
        if ( read_data[27] !== 1 )
        begin
            $display("Signaled Target Abort bit not set in PCI Device Status register after read terminated with Target Abort") ;
            test_fail("Signaled Target Abort bit was not set in PCI Device Status register after read terminated with Target Abort") ;
            ok = 0 ;
        end
        if ( read_data[28] !== 0 )
        begin
            $display("Received Target Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Target Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end
        if ( read_data[29] !== 0 )
        begin
            $display("Received Master Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Master Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end

        // clear statuses
        config_write(addr_offset, {2'b00, read_data[29:27], 27'h0}, 4'b11_00, ok) ;
        if ( !ok )
        begin
            test_fail("write to PCI Device Status register failed") ;
            $display("Couldn't write PCI Device Status register") ;
            disable main ;
        end

        if ( ok )
            test_ok ;

        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;

        ok = 1 ;
        test_name = "PCI ERROR CONTROL AND STATUS REGISTER VALUE AFTER READ THROUGH TARGET TERMINATED WITH TARGET ABORT" ;

        config_read(addr_offset, 4'hF, read_data) ;
        if (read_data[8] !== 0)
        begin
            $display("Error signaled bit set after Target Read Terminated with Error on WISHBONE. Error reporting should be done only for posted writes") ;
            test_fail("Error signaled bit set after Target Read Terminated with Error on WISHBONE") ;
            ok = 0 ;
        end
        else
            test_ok ;

        if ( ok !== 1 )
        begin
            config_write(addr_offset, read_data, 4'hF, ok) ;
            if ( !ok )
            begin
                test_fail("PCI Error Control and Status register could not be written") ;
                disable main ;
            end
        end

        test_name = "BURST READ THROUGH PCI TARGET TERMINATED WITH ERROR ON WISHBONE ON ONE BEFORE LAST DATAPHASE" ;
        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);
        fork
        begin
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                        2, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Retry_Before);

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( Target_Base_Addr_R[target_mem_image], 1'b0, 3, 1'b1, ok ) ;

            if ( ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus") ;

            do_pause(3) ;
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                        5, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Abort_Before);
            do_pause(1) ;

            while ( FRAME === 0 )
                @(posedge pci_clock) ;

            while ( IRDY === 0 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done ) 
                disable monitor_error_event6 ;
        end
        begin:monitor_error_event6
            error_monitor_done = 0 ;
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_stop( 3 ) ;
            wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);
        end
        join

        if ( ok )
            test_ok ;

        @(posedge pci_clock) ;
        @(posedge pci_clock) ;
        @(posedge wb_clock) ;
        @(posedge wb_clock) ;

        test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER TARGET ABORT" ;
        addr_offset = 12'h004 ;
        config_read(addr_offset, 4'hF, read_data) ;
        ok = 1 ;
        if ( read_data[27] !== 1 )
        begin
            $display("Signaled Target Abort bit not set in PCI Device Status register after read terminated with Target Abort") ;
            test_fail("Signaled Target Abort bit was not set in PCI Device Status register after read terminated with Target Abort") ;
            ok = 0 ;
        end
        if ( read_data[28] !== 0 )
        begin
            $display("Received Target Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Target Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end
        if ( read_data[29] !== 0 )
        begin
            $display("Received Master Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Master Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end

        // clear statuses
        config_write(addr_offset, {2'b00, read_data[29:27], 27'h0}, 4'b11_00, ok) ;
        if ( !ok )
        begin
            test_fail("write to PCI Device Status register failed") ;
            $display("Couldn't write PCI Device Status register") ;
            disable main ;
        end

        if ( ok )
            test_ok ;

        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;

        ok = 1 ;
        test_name = "PCI ERROR CONTROL AND STATUS REGISTER VALUE AFTER READ THROUGH TARGET TERMINATED WITH TARGET ABORT" ;

        config_read(addr_offset, 4'hF, read_data) ;
        if (read_data[8] !== 0)
        begin
            $display("Error signaled bit set after Target Read Terminated with Error on WISHBONE. Error reporting should be done only for posted writes") ;
            test_fail("Error signaled bit set after Target Read Terminated with Error on WISHBONE") ;
            ok = 0 ;
        end
        else
            test_ok ;

        if ( ok !== 1 )
        begin
            config_write(addr_offset, read_data, 4'hF, ok) ;
            if ( !ok )
            begin
                test_fail("PCI Error Control and Status register could not be written") ;
                disable main ;
            end
        end

        test_name = "FULL FIFO BURST READ THROUGH PCI TARGET TERMINATED WITH ERROR ON WISHBONE ON LAST DATAPHASE" ;
        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);
        fork
        begin
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_MUL, 32'h1234_5678, 4'hF,
                        2, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Retry_Before);

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( Target_Base_Addr_R[target_mem_image], 1'b0, `PCIR_DEPTH - 2, 1'b1, ok ) ;

            if ( ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus") ;

            do_pause(3) ;
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_MUL, 32'h1234_5678, 4'hF,
                        `PCIR_DEPTH - 1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Abort_On);
            do_pause(1) ;

            while ( FRAME === 0 )
                @(posedge pci_clock) ;

            while ( IRDY === 0 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event7 ;
        end
        begin:monitor_error_event7
            error_monitor_done = 0 ;
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_stop( `PCIR_DEPTH - 2 ) ;
            wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);
        end
        join

        if ( ok )
            test_ok ;

        @(posedge pci_clock) ;
        @(posedge pci_clock) ;
        @(posedge wb_clock) ;
        @(posedge wb_clock) ;

        test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER TARGET ABORT" ;
        addr_offset = 12'h004 ;
        config_read(addr_offset, 4'hF, read_data) ;
        ok = 1 ;
        if ( read_data[27] !== 1 )
        begin
            $display("Signaled Target Abort bit not set in PCI Device Status register after read terminated with Target Abort") ;
            test_fail("Signaled Target Abort bit was not set in PCI Device Status register after read terminated with Target Abort") ;
            ok = 0 ;
        end
        if ( read_data[28] !== 0 )
        begin
            $display("Received Target Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Target Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end
        if ( read_data[29] !== 0 )
        begin
            $display("Received Master Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Master Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end

        // clear statuses
        config_write(addr_offset, {2'b00, read_data[29:27], 27'h0}, 4'b11_00, ok) ;
        if ( !ok )
        begin
            test_fail("write to PCI Device Status register failed") ;
            $display("Couldn't write PCI Device Status register") ;
            disable main ;
        end

        if ( ok )
            test_ok ;

        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;

        ok = 1 ;
        test_name = "PCI ERROR CONTROL AND STATUS REGISTER VALUE AFTER READ THROUGH TARGET TERMINATED WITH TARGET ABORT" ;

        config_read(addr_offset, 4'hF, read_data) ;
        if (read_data[8] !== 0)
        begin
            $display("Error signaled bit set after Target Read Terminated with Error on WISHBONE. Error reporting should be done only for posted writes") ;
            test_fail("Error signaled bit set after Target Read Terminated with Error on WISHBONE") ;
            ok = 0 ;
        end
        else
            test_ok ;

        if ( ok !== 1 )
        begin
            config_write(addr_offset, read_data, 4'hF, ok) ;
            if ( !ok )
            begin
                test_fail("PCI Error Control and Status register could not be written") ;
                disable main ;
            end
        end

        test_name = "FULL FIFO BURST READ THROUGH PCI TARGET TERMINATED WITH ERROR ON WISHBONE BEFORE LAST DATAPHASE" ;
        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);
        fork
        begin
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_MUL, 32'h1234_5678, 4'hF,
                        2, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Retry_Before);

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( Target_Base_Addr_R[target_mem_image], 1'b0, `PCIR_DEPTH - 2, 1'b1, ok ) ;

            if ( ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus") ;

            do_pause(3) ;
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_MUL, 32'h1234_5678, 4'hF,
                        `PCIR_DEPTH, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Abort_Before);
            do_pause(1) ;

            while ( FRAME === 0 )
                @(posedge pci_clock) ;

            while ( IRDY === 0 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event8 ;
        end
        begin:monitor_error_event8
            error_monitor_done = 0 ;
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_stop( `PCIR_DEPTH - 2 ) ;
            wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);
        end
        join

        if ( ok )
            test_ok ;

        @(posedge pci_clock) ;
        @(posedge pci_clock) ;
        @(posedge wb_clock) ;
        @(posedge wb_clock) ;

        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

        test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER TARGET ABORT" ;
        addr_offset = 12'h004 ;
        config_read(addr_offset, 4'hF, read_data) ;
        ok = 1 ;
        if ( read_data[27] !== 1 )
        begin
            $display("Signaled Target Abort bit not set in PCI Device Status register after read terminated with Target Abort") ;
            test_fail("Signaled Target Abort bit was not set in PCI Device Status register after read terminated with Target Abort") ;
            ok = 0 ;
        end
        if ( read_data[28] !== 0 )
        begin
            $display("Received Target Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Target Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end
        if ( read_data[29] !== 0 )
        begin
            $display("Received Master Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Master Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end

        // clear statuses
        config_write(addr_offset, {2'b00, read_data[29:27], 27'h0}, 4'b11_00, ok) ;
        if ( !ok )
        begin
            test_fail("write to PCI Device Status register failed") ;
            $display("Couldn't write PCI Device Status register") ;
            disable main ;
        end

        if ( ok )
            test_ok ;

        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;

        ok = 1 ;
        test_name = "PCI ERROR CONTROL AND STATUS REGISTER VALUE AFTER READ THROUGH TARGET TERMINATED WITH TARGET ABORT" ;

        config_read(addr_offset, 4'hF, read_data) ;
        if (read_data[8] !== 0)
        begin
            $display("Error signaled bit set after Target Read Terminated with Error on WISHBONE. Error reporting should be done only for posted writes") ;
            test_fail("Error signaled bit set after Target Read Terminated with Error on WISHBONE") ;
            ok = 0 ;
        end
        else
            test_ok ;

        if ( ok !== 1 )
        begin
            config_write(addr_offset, read_data, 4'hF, ok) ;
            if ( !ok )
            begin
                test_fail("PCI Error Control and Status register could not be written") ;
                disable main ;
            end
        end

        test_name = "BURST READ THROUGH PCI TARGET TERMINATED WITH ERROR ON WISHBONE, ERROR NOT PULLED OUT ON PCI" ;
        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);
        fork
        begin
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                        2, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Retry_Before);

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( Target_Base_Addr_R[target_mem_image], 1'b0, 3, 1'b1, ok ) ;

            if ( ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus") ;

            do_pause(3) ;
            PCIU_READ (1, Target_Base_Addr_R[target_mem_image],
                        `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                        3, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Normal_Completion);
            do_pause(1) ;

            while ( FRAME === 0 )
                @(posedge pci_clock) ;

            while ( IRDY === 0 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event9 ;
        end
        begin:monitor_error_event9
            error_monitor_done = 0 ; 
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_stop( 3 ) ;
            wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);
        end
        join

        if ( ok )
            test_ok ;

        // now check all other statuses too
        test_name = "ALL PCI DEVICE STATUSES AFTER ERRONEOUS TARGET READS" ;
        ok = 1 ;

        addr_offset = 12'h004 ;
        config_read(addr_offset, 4'hF, read_data) ;
        if (read_data[31] !== 0)
        begin
            $display("Detected Parity Error bit set for no reason") ;
            test_fail("Detected Parity Error bit was set for no reason") ;
            ok = 0 ;
        end

        if (read_data[30] !== 0)
        begin
            $display("Signaled System Error bit set for no reason") ;
            test_fail("Signaled System Error bit was set for no reason") ;
            ok = 0 ;
        end

        if (read_data[29] !== 0)
        begin
            $display("Received Master Abort bit set for no reason") ;
            test_fail("Received Master Abort bit was set for no reason") ;
            ok = 0 ;
        end

        if (read_data[28] !== 0)
        begin
            $display("Received Target Abort bit set for no reason");
            test_fail("Received Target Abort bit was set for no reason") ;
            ok = 0 ;
        end

        if (read_data[27] !== 0)
        begin
            $display("Signaled Target Abort bit set after it was cleared by writing one to its location") ;
            test_fail("Signaled Target Abort bit was set after it was cleared by writing one to its location") ;
            ok = 0 ;
        end

        if (read_data[24] !== 0)
        begin
            $display("Master Data Parity Error bit set for no reason") ;
            test_fail("Master Data Parity Error bit was set for no reason") ;
            ok = 0 ;
        end

        if ( ok )
            test_ok ;

        test_name = "DISABLE IMAGE" ;
        config_write( am_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write PCI Address Mask register! Time %t ", $time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main;
        end
    end

    if ( target_io_image !== -1 )
    begin
        do_io_aborts = 1 ;

        if (target_io_image === 1)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL1_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA1_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM1_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA1_ADDR, 2'b00} ;
        end
        else if (target_io_image === 2)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL2_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA2_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM2_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA2_ADDR, 2'b00} ;
        end
        else if (target_io_image === 3)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL3_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA3_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM3_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA3_ADDR, 2'b00} ;
        end
        else if (target_io_image === 4)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL4_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA4_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM4_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA4_ADDR, 2'b00} ;
        end
        else if (target_io_image === 5)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL5_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA5_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM5_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA5_ADDR, 2'b00} ;
        end
    end
    else
        do_io_aborts = 0 ;

    if ( do_io_aborts )
    begin

        test_name = "CONFIGURE BRIDGE FOR ERROR TERMINATED READS THROUGH PCI TARGET UNIT" ;

        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;
        config_write( addr_offset, 32'hFFFF_FFFF, 4'hF, ok) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write P_ERR_CS register! Time %t ", $time);
            test_fail("PCI Error Control and Status register could not be written") ;
            disable main;
        end

        // enable INTERRUPT reporting
        addr_offset = {4'h1, `ICR_ADDR, 2'b00} ;
        config_write( addr_offset, 32'h7FFF_FFFF, 4'hF, ok) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write ICR register! Time %t ", $time);
            test_fail("Interrupt Control register could not be written") ;
            disable main;
        end

        addr_offset = 12'h010 + (4*target_io_image) ;

        config_write( addr_offset, Target_Base_Addr_R[target_io_image] | 32'h0000_0001, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write PCI Base Address register! Time %t ", $time);
            test_fail("PCI Base Address register could not be written") ;
            disable main;
        end

        // disable address translation and enable prefetch so read bursts can be performed
        config_write( ctrl_offset, 32'h0000_0002, 4'h1, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write PCI Image Control register! Time %t ", $time);
            test_fail("PCI Image Control register could not be written") ;
            disable main;
        end

        config_write( am_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write PCI Address Mask register! Time %t ", $time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main;
        end

        addr_offset = 12'h00C ;

        config_write( addr_offset, 32'h0000_04_04, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write PCI Latency Timer/Cache line size register! Time %t ", $time);
            test_fail("PCI Latency Timer/Cache line size register could not be written") ;
            disable main;
        end

        // set response of WB SLAVE - ERR,    WAIT cycles,        RETRY cycles
        wishbone_slave.cycle_response(3'b010, tb_subseq_waits, 8'h0);

        // do a single read error terminated on WB bus
        test_name = "SINGLE I/O READ THROUGH PCI TARGET TERMINATED WITH ERROR ON WISHBONE" ;

        fork
        begin
            PCIU_IO_READ
             (
                `Test_Master_1,
                Target_Base_Addr_R[target_io_image],
                32'hAAAA_5555,
                4'h0,
                1,
                `Test_Target_Retry_On
             );

            do_pause( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( Target_Base_Addr_R[target_io_image], 1'b0, 0, 1'b1, ok ) ;

            if ( ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus") ;

            do_pause(3) ;

            PCIU_IO_READ
             (
                `Test_Master_1,
                Target_Base_Addr_R[target_io_image],
                32'hAAAA_5555,
                4'h0,
                1,
                `Test_Target_Abort_On
             );

            do_pause( 1 ) ;

            while ( FRAME === 0 )
                @(posedge pci_clock) ;

            while ( IRDY === 0 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event10 ;
        end
        begin:monitor_error_event10
            error_monitor_done = 0 ;
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while reading through PCI Target Unit") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( ok )
            test_ok ;

        @(posedge pci_clock) ;
        @(posedge pci_clock) ;
        @(posedge wb_clock) ;
        @(posedge wb_clock) ;

        test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER TARGET ABORT" ;
        addr_offset = 12'h004 ;
        config_read(addr_offset, 4'hF, read_data) ;
        ok = 1 ;
        if ( read_data[27] !== 1 )
        begin
            $display("Signaled Target Abort bit not set in PCI Device Status register after read terminated with Target Abort") ;
            test_fail("Signaled Target Abort bit was not set in PCI Device Status register after read terminated with Target Abort") ;
            ok = 0 ;
        end
        if ( read_data[28] !== 0 )
        begin
            $display("Received Target Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Target Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end
        if ( read_data[29] !== 0 )
        begin
            $display("Received Master Abort bit set in PCI Device Status register after Target read terminated with Target Abort. Master was not working") ;
            test_fail("Received Master Abort bit was set in PCI Device Status register after Target read was terminated with Target Abort and Master wasn't doing references") ;
            ok = 0 ;
        end

        // clear statuses
        config_write(addr_offset, {2'b00, read_data[29:27], 27'h0}, 4'b11_00, ok) ;
        if ( !ok )
        begin
            test_fail("write to PCI Device Status register failed") ;
            $display("Couldn't write PCI Device Status register") ;
            disable main ;
        end

        if ( ok )
            test_ok ;

        addr_offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;

        ok = 1 ;
        test_name = "PCI ERROR CONTROL AND STATUS REGISTER VALUE AFTER READ THROUGH TARGET TERMINATED WITH TARGET ABORT" ;

        config_read(addr_offset, 4'hF, read_data) ;
        if (read_data[8] !== 0)
        begin
            $display("Error signaled bit set after Target Read Terminated with Error on WISHBONE. Error reporting should be done only for posted writes") ;
            test_fail("Error signaled bit set after Target Read Terminated with Error on WISHBONE") ;
            ok = 0 ;
        end
        else
            test_ok ;

        if ( ok !== 1 )
        begin
            config_write(addr_offset, read_data, 4'hF, ok) ;
            if ( !ok )
            begin
                test_fail("PCI Error Control and Status register could not be written") ;
                disable main ;
            end
        end

        test_name = "DISABLE IMAGE" ;
        config_write( am_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Erroneous PCI Target read testing failed! Failed to write PCI Address Mask register! Time %t ", $time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main;
        end

        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

    end
end // main
endtask // test_wb_error_rd

task test_target_abort ;
    input [2:0]  image_num ;
    reg   [11:0] pci_ctrl_offset ;
    reg   [11:0] ctrl_offset ;
    reg   [11:0] ba_offset ;
    reg   [11:0] am_offset ;
    reg   [11:0] ta_offset ;
    reg   [31:0] pci_address ;
    reg   [3:0]  byte_enables ;
    reg          ok ;
    reg          error_monitor_done ;
begin:main
    pci_ctrl_offset = 12'h4 ;
    if (image_num === 0)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL0_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA0_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM0_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA0_ADDR, 2'b00} ;
    end
    else if (image_num === 1)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL1_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA1_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM1_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA1_ADDR, 2'b00} ;
    end
    else if (image_num === 2)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL2_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA2_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM2_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA2_ADDR, 2'b00} ;
    end
    else if (image_num === 3)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL3_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA3_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM3_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA3_ADDR, 2'b00} ;
    end
    else if (image_num === 4)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL4_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA4_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM4_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA4_ADDR, 2'b00} ;
    end
    else if (image_num === 5)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL5_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA5_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM5_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA5_ADDR, 2'b00} ;
    end

    test_name = "CONFIGURE TARGET FOR TARGET ABORT TESTING" ;

    config_write( ba_offset, Target_Base_Addr_R[image_num] | 32'h0000_0001, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Target Abort testing failed! Failed to write P_BA%d register! Time %t ", 1 ,$time);
        test_fail("PCI Base Address register could not be written") ;
        disable main ;
    end

    // Set Address Mask of IMAGE
    config_write( am_offset, Target_Addr_Mask_R[image_num], 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Target Abort testing failed! Failed to write P_AM%d register! Time %t ",1 ,$time);
        test_fail("PCI Address Mask register could not be written") ;
        disable main ;
    end

    // Set Translation Address of IMAGE
    config_write( ta_offset, Target_Tran_Addr_R[image_num], 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Target Abort testing failed! Failed to write P_TA%d register! Time %t ",1 ,$time);
        test_fail("PCI Translation Address Register could not be written") ;
        disable main ;
    end

    config_write( ctrl_offset, 32'h0000_0002, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Target Abort testing failed! Failed to write P_IMG_CTRL%d register! Time %t ",1 ,$time);
        test_fail("PCI Image Control register could not be written") ;
        disable main ;
    end

    wishbone_slave.cycle_response( 3'b010, tb_subseq_waits, 0 ) ;

    test_name = "TARGET ABORT SIGNALING ON I/O ACCESSES WITH INVALID ADDRESS/BYTE ENABLE COMBINATION" ;

    pci_address  = Target_Base_Addr_R[image_num] ;
    byte_enables = 4'b0001 ;

    fork
    begin
        PCIU_IO_WRITE( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Abort_On) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event1
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event1 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;

    fork
    begin
        PCIU_IO_READ( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Abort_On) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event2
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_READ, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event2 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;

    pci_address  = Target_Base_Addr_R[image_num] + 1 ;
    byte_enables = 4'b0011 ;

    fork
    begin
        PCIU_IO_WRITE( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 2, `Test_Target_Abort_Before) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event3
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        if ( !error_monitor_done )
            disable monitor_error_event3 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;

    byte_enables = 4'b0000 ;

    fork
    begin
        PCIU_IO_READ( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 2, `Test_Target_Abort_Before) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event4
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_READ, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event4 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;

    pci_address  = Target_Base_Addr_R[image_num] + 2 ;
    byte_enables = 4'b0111 ;

    fork
    begin
        PCIU_IO_WRITE( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Abort_On) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event5
        error_monitor_done = 0 ; 
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event5 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;

    byte_enables = 4'b0010 ;

    fork
    begin
        PCIU_IO_READ( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 2, `Test_Target_Abort_Before) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event6
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_READ, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event6 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;
    byte_enables = 4'b0001 ;

    fork
    begin
        PCIU_IO_READ( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 2, `Test_Target_Abort_Before) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event7
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_READ, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event7 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;
    byte_enables = 4'b0000 ;

    fork
    begin
        PCIU_IO_WRITE( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Abort_On) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event8
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event8 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;

    pci_address  = Target_Base_Addr_R[image_num] + 3 ;
    byte_enables = 4'b0110 ;

    fork
    begin
        PCIU_IO_WRITE( `Test_Master_1, pci_address, 32'hA1A2_A3A4, byte_enables, 1, `Test_Target_Abort_On) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event9
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event9 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;
    fork
    begin
        PCIU_IO_READ( `Test_Master_1, pci_address, 32'hA1A2_A3A4, byte_enables, 1, `Test_Target_Abort_On) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event10
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_READ, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;

        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event10 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;

    byte_enables = 4'b0001 ;

    fork
    begin
        PCIU_IO_WRITE( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 2, `Test_Target_Abort_Before) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event11
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event11 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;

    byte_enables = 4'b0101 ;

    fork
    begin
        PCIU_IO_WRITE( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 2, `Test_Target_Abort_Before) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event12
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event12 ;
    end
    join

    if ( ok )
        test_ok ;

    ok = 1 ;

    byte_enables = 4'b0011 ;

    fork
    begin
        PCIU_IO_WRITE( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 2, `Test_Target_Abort_Before) ;
        do_pause ( 1 ) ;
    end
    begin:monitor_error_event13
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI Master or Monitor detected invalid operation on PCI bus") ;
        ok = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_IO_WRITE, 0, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
        #1 ;
        if ( !error_monitor_done )
            disable monitor_error_event13 ;
    end
    join

    if ( ok )
        test_ok ;

    test_name = "PCI DEVICE STATUS REGISTER VALUE AFTER TARGET ABORT" ;
    config_read(pci_ctrl_offset, 4'hF, pci_address) ;
    ok = 1 ;
    if ( pci_address[27] !== 1 )
    begin
        $display("Signaled Target Abort bit not set in PCI Device Status register after a few Target Aborts were signaled") ;
        test_fail("Signaled Target Abort bit was not set in PCI Device Status register after target terminated with Target Abort") ;
        ok = 0 ;
    end
    if ( pci_address[28] !== 0 )
    begin
        $display("Received Target Abort bit set in PCI Device Status register after Target terminated with Target Abort. Master was not working") ;
        test_fail("Received Target Abort bit was set in PCI Device Status register after Target terminated with Target Abort and Master wasn't doing references") ;
        ok = 0 ;
    end
    if ( pci_address[29] !== 0 )
    begin
        $display("Received Master Abort bit set in PCI Device Status register after Target terminated with Target Abort. Master was not working") ;
        test_fail("Received Master Abort bit was set in PCI Device Status register after Target terminated with Target Abort and Master wasn't doing references") ;
        ok = 0 ;
    end

    // clear statuses
    config_write(pci_ctrl_offset, {2'b00, pci_address[29:27], 27'h0}, 4'b11_00, ok) ;
    if ( !ok )
    begin
        test_fail("write to PCI Device Status register failed") ;
        $display("Couldn't write PCI Device Status register") ;
        disable main ;
    end

    if ( ok )
        test_ok ;

    test_name = "ERROR CONTROL AND STATUS REGISTER VALUE CHECK AFTER TARGET ABORTS" ;
    config_read({4'h1, `P_ERR_CS_ADDR, 2'b00}, 4'hF, pci_address) ;
    if ( pci_address[8] !== 0 )
    begin
        test_fail("writes and reads terminated with Target Aborts on PCI side should not proceede to WISHBONE bus") ;
    end
    else
        test_ok ;

    wishbone_slave.cycle_response( 3'b100, tb_subseq_waits, 0 ) ;

    test_name = "DISABLE IMAGE" ;

    config_write( am_offset, Target_Addr_Mask_R[image_num] & 32'h7FFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Target Abort testing failed! Failed to write P_AM%d register! Time %t ",1 ,$time);
        test_fail("PCI Address Mask register could not be written") ;
        disable main ;
    end
end
endtask // test_target_abort

task test_target_io_wr_rd ;
    input [2:0]  image_num ;
    input        translate_address ;
    input [11:0] img_ctrl_offset ;
    reg   [31:0] expect_address ;
    reg   [31:0] pci_address ;
    reg          translation ;
    reg   [31:0] read_data ;
    reg   [3:0]  byte_enables ;
    reg          ok ;
    reg          pci_ok ;
    reg          wb_ok ;
    integer      i ;
    reg          error_monitor_done ;
begin:main
    `ifdef ADDR_TRAN_IMPL
        translation = translate_address ;
    `else
        translation = 0 ;
    `endif

    wishbone_slave.cycle_response( 3'b100, tb_subseq_waits, 0 ) ;

    test_name = "ENABLE/DISABLE ADDRESS TRANSLATION" ;
    config_read( img_ctrl_offset, 4'hF, read_data ) ;
    if ( translation )
        config_write( img_ctrl_offset, read_data | 32'h0000_0004, 4'hF, ok ) ;
    else
        config_write( img_ctrl_offset, read_data & 32'hFFFF_FFFB, 4'hF, ok ) ;

    if ( !ok )
    begin
        $display("Failed to write PCI Image Control register, time %t ", $time ) ;
        test_fail("PCI Image Control register could not be written") ;
    end

    test_name = "BYTE ADDRESSABLE WRITES THROUGH TARGET IO IMAGE" ;
    pci_address  = Target_Base_Addr_R[image_num] ;
    byte_enables = 4'b0000 ;
    expect_address = pci_to_wb_addr_convert( pci_address, Target_Tran_Addr_R[image_num], translation ) ;

    fork
    begin
        PCIU_IO_WRITE( `Test_Master_2, pci_address, 32'h5555_5555, byte_enables, 1, `Test_Target_Normal_Completion) ;
        do_pause ( 1 ) ;
    end
    begin
        wb_transaction_progress_monitor( expect_address, 1'b1, 1, 1'b1, wb_ok ) ;
        if ( wb_ok !== 1 )
            test_fail("WISHBONE Master started invalid transaction or none at all on WB bus after single I/O write was posted on PCI") ;

        #1 ;
        if ( !error_monitor_done )
            disable monitor_pci_error_1 ;
    end
    begin:monitor_pci_error_1
        error_monitor_done = 0 ;
        pci_ok = 1 ;
        @(error_event_int) ;
        pci_ok = 0 ;
        test_fail ( "PCI Monitor or PCI Master detected an error on PCI bus while doing IO refernce to Target" ) ;
        error_monitor_done = 1 ;
    end
    join

    byte_enables = 4'b1111 ;
    for ( i = 0 ; i < 4 ; i = i + 1 )
    begin:loop_1
        byte_enables[i] = 0 ;
        if ( i > 0 )
            byte_enables[i - 1] = 1 ;
        fork
        begin
            PCIU_IO_WRITE( `Test_Master_2, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Normal_Completion) ;
            do_pause ( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( expect_address, 1'b1, 1, 1'b1, wb_ok ) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus after single I/O write was posted on PCI") ;

            #1 ;
            if ( !error_monitor_done ) 
                disable monitor_pci_error_2 ;
        end
        begin:monitor_pci_error_2
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            pci_ok = 0 ;
            test_fail ( "PCI Monitor or PCI Master detected an error on PCI bus while doing IO reference to Target" ) ;
            error_monitor_done = 1 ;
        end
        join

        if ( !pci_ok || !wb_ok )
            disable loop_1 ;

        pci_address = pci_address + 1 ;
        expect_address = expect_address + 1 ;
    end

    if ( pci_ok && wb_ok )
        test_ok ;

    test_name = "READ BY WORDS IO DATA PREVIOUSLY WRITTEN BY BYTES" ;
    pci_address  = Target_Base_Addr_R[image_num] ;
    byte_enables = 4'b1100 ;
    expect_address = pci_to_wb_addr_convert( pci_address, Target_Tran_Addr_R[image_num], translation ) ;

    master1_check_received_data = 1 ;
    fork
    begin
        PCIU_IO_READ( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Retry_On) ;
        do_pause( 1 ) ;
    end
    begin
        wb_transaction_progress_monitor( expect_address, 1'b0, 1, 1'b1, wb_ok ) ;
        if ( wb_ok !== 1 )
            test_fail("WISHBONE Master started invalid transaction or none at all on WB bus after single I/O read requested on PCI") ;

        do_pause ( 2 ) ;
        PCIU_IO_READ( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Normal_Completion) ;
        do_pause ( 16 ) ;

        #1 ;
        if ( !error_monitor_done )
            disable monitor_pci_error_3 ;
    end
    begin:monitor_pci_error_3
        error_monitor_done = 0 ;
        pci_ok = 1 ;
        @(error_event_int) ;
        pci_ok = 0 ;
        test_fail ( "PCI Monitor or PCI Master detected an error on PCI bus while doing IO read reference to Target" ) ;
        error_monitor_done = 1 ;
    end
    join

    if ( !pci_ok || !wb_ok )
    begin
        disable main ;
    end

    pci_address  = Target_Base_Addr_R[image_num] + 2;
    byte_enables = 4'b0011 ;
    expect_address = pci_to_wb_addr_convert( pci_address, Target_Tran_Addr_R[image_num], translation ) ;

    master1_check_received_data = 1 ;
    fork
    begin
        PCIU_IO_READ( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Retry_On) ;
        do_pause( 1 ) ;
    end
    begin
        wb_transaction_progress_monitor( expect_address, 1'b0, 1, 1'b1, wb_ok ) ;
        if ( wb_ok !== 1 )
            test_fail("WISHBONE Master started invalid transaction or none at all on WB bus after single I/O read requested on PCI") ;

        do_pause ( 2 ) ;
        PCIU_IO_READ( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Normal_Completion) ;
        do_pause ( 16 ) ;

        #1 ;
        if ( !error_monitor_done )
            disable monitor_pci_error_4 ;
    end
    begin:monitor_pci_error_4
        error_monitor_done = 0 ;
        pci_ok = 1 ;
        @(error_event_int) ;
        pci_ok = 0 ;
        test_fail ( "PCI Monitor or PCI Master detected an error on PCI bus while doing IO read reference to Target" ) ;
        error_monitor_done = 1 ;
    end
    join

    if ( !pci_ok || !wb_ok )
    begin
        disable main ;
    end

    pci_address  = Target_Base_Addr_R[image_num] ;
    byte_enables = 4'b0000 ;
    expect_address = pci_to_wb_addr_convert( pci_address, Target_Tran_Addr_R[image_num], translation ) ;

    master1_check_received_data = 1 ;
    fork
    begin
        PCIU_IO_READ( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Retry_On) ;
        do_pause( 1 ) ;
    end
    begin
        wb_transaction_progress_monitor( expect_address, 1'b0, 1, 1'b1, wb_ok ) ;
        if ( wb_ok !== 1 )
            test_fail("WISHBONE Master started invalid transaction or none at all on WB bus after single I/O read requested on PCI") ;

        do_pause ( 2 ) ;
        PCIU_IO_READ( `Test_Master_1, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Normal_Completion) ;
        do_pause ( 16 ) ;

        #1 ;
        if ( !error_monitor_done )
            disable monitor_pci_error_5 ;
    end
    begin:monitor_pci_error_5
        error_monitor_done = 0 ;
        pci_ok = 1 ;
        @(error_event_int) ;
        pci_ok = 0 ;
        test_fail ( "PCI Monitor or PCI Master detected an error on PCI bus while doing IO read reference to Target" ) ;
        error_monitor_done = 1 ;
    end
    join

    if ( pci_ok && wb_ok )
        test_ok ;
end
endtask // test_target_io_wr_rd

task test_target_io_err_wr ;
    input [2:0]  image_num ;
    input        translate_address ;
    input [11:0] img_ctrl_offset ;
    input        enable_error_report ;
    input        enable_error_interrupt ;

    reg   [31:0] expect_address ;
    reg   [31:0] pci_address ;
    reg          translation ;
    reg   [31:0] read_data ;
    reg   [3:0]  byte_enables ;
    reg          ok ;
    reg          pci_ok ;
    reg          wb_ok ;
    integer      i ;
    reg   [11:0] offset ;
    reg          error_monitor_done ;
begin:main
    `ifdef ADDR_TRAN_IMPL
        translation = translate_address ;
    `else
        translation = 0 ;
    `endif

    wishbone_slave.cycle_response( 3'b010, tb_subseq_waits, 0 ) ;

    test_name = "ENABLE/DISABLE ADDRESS TRANSLATION" ;
    config_read( img_ctrl_offset, 4'hF, read_data ) ;
    if ( translation )
        config_write( img_ctrl_offset, read_data | 32'h0000_0004, 4'hF, ok ) ;
    else
        config_write( img_ctrl_offset, read_data & 32'hFFFF_FFFB, 4'hF, ok ) ;

    if ( !ok )
    begin
        $display("Failed to write PCI Image Control register, time %t ", $time ) ;
        test_fail("PCI Image Control register could not be written") ;
    end

    test_name = "ENABLE/DISABLE ERROR REPORTING" ;
    offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;
    if ( enable_error_report )
    begin
        config_write(offset, 32'h0000_0001, 4'b0001, ok) ;
        if ( !ok )
        begin
            test_fail("PCI Error Control and Status register could not be written") ;
            disable main ;
        end
    end
    else
    begin
        config_write(offset, 32'h0000_0000, 4'b0001, ok) ;
        if ( !ok )
        begin
            test_fail("PCI Error Control and Status register could not be written") ;
            disable main ;
        end
    end

    test_name = "ENABLE/DISABLE PCI ERROR INTERRUPTS" ;
    offset    = {4'h1, `ICR_ADDR, 2'b00} ;
    if ( enable_error_interrupt )
    begin
        config_write(offset, 32'h0000_0004, 4'b0001, ok) ;
        if ( !ok )
        begin
            test_fail("Interrupt Control register could not be written") ;
            disable main ;
        end
    end
    else
    begin
        config_write(offset, 32'h0000_0000, 4'b0001, ok) ;
        if ( !ok )
        begin
            test_fail("Interrupt Control register could not be written") ;
            disable main ;
        end
    end

    pci_address  = Target_Base_Addr_R[image_num] ;
    expect_address = pci_to_wb_addr_convert( pci_address, Target_Tran_Addr_R[image_num], translation ) ;

    byte_enables = 4'b1111 ;

    for ( i = 0 ; i < 4 ; i = i + 1 )
    begin:loop_1
        test_name = "POST IO WRITE THAT WILL BE TERMINATED WITH ERROR ON WISHBONE" ;
        byte_enables[i] = 0 ;
        if ( i > 0 )
            byte_enables[i - 1] = 1 ;

        fork
        begin
            PCIU_IO_WRITE( `Test_Master_2, pci_address, 32'hAAAA_AAAA, byte_enables, 1, `Test_Target_Normal_Completion) ;
            do_pause ( 1 ) ;
        end
        begin
            wb_transaction_progress_monitor( expect_address, 1'b1, 0, 1'b1, wb_ok ) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WB bus after single I/O write was posted on PCI") ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_pci_error_2 ;
        end
        begin:monitor_pci_error_2
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            pci_ok = 0 ;
            test_fail ( "PCI Monitor or PCI Master detected an error on PCI bus while doing IO reference to Target" ) ;
            error_monitor_done = 1 ;
        end
        join

         test_name = "INTERRUPT REQUEST ASSERTION AFTER ERROR TERMINATED WRITE ON WISHBONE" ;
        `ifdef HOST

            repeat ( 4 )
                @( posedge wb_clock ) ;

            if ( enable_error_interrupt && enable_error_report )
            begin
                if ( INT_O !== 1 )
                begin
                    test_fail("bridge didn't assert interrupt request on WISHBONE after error terminated posted write with error reporting and interrupts enabled") ;
                end
            end
            else
            begin
                if ( INT_O !== 0 )
                begin
                    test_fail("bridge asserted interrupt request on WISHBONE after error terminated posted write with error reporting or interrupts disabled") ;
                end
            end
        `else
            repeat ( 4 )
                @( posedge pci_clock ) ;

            if ( enable_error_interrupt && enable_error_report )
            begin
                if ( INTA !== 0 )
                begin
                    test_fail("bridge didn't assert interrupt request on PCI after error terminated posted write with error reporting and interrupts enabled") ;
                end
            end
            else
            begin
                if ( INTA !== 1 )
                begin
                    test_fail("bridge asserted interrupt request on PCI after error terminated posted write with error reporting or interrupts disabled") ;
                end
            end
        `endif

        test_name = "ERROR STATUS REGISTER VALUE CHECK AFTER ERROR TERMINATED POSTED IO WRITE ON WISHBONE" ;
        offset = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;
        config_read( offset, 4'hF, read_data ) ;
        ok = 1 ;
        if ( enable_error_report )
        begin
            if ( read_data[8] !== 1 )
            begin
                test_fail("error terminated write on WISHBONE didn't set Error bit when error reporting was enabled" ) ;
                ok = 0 ;
            end

            if ( read_data[9] !== 0 )
            begin
                test_fail("WISHBONE slave was a cause of error, but error source bit was not 0" ) ;
                ok = 0 ;
            end

            if ( read_data[31:28] !== byte_enables )
            begin
                test_fail("byte enable field in PCI Error Control and Status register was wrong" ) ;
                ok = 0 ;
            end

            if ( read_data[27:24] !== `BC_IO_WRITE )
            begin
                test_fail("bus command field in PCI Error Control and Status register was wrong" ) ;
                ok = 0 ;
            end

            if ( ok )
                test_ok ;

            test_name = "CLEAR ERROR STATUS" ;
            config_write( offset, read_data, 4'hF, ok ) ;
            if ( !ok )
                test_fail("PCI Error Control and Status register could not be written") ;

            test_name = "ERRONEOUS ADDRESS AND DATA REGISTERS' VALUES CHECK AFTER WRITE TERMINATED WITH ERROR ON WISHBONE" ;
            offset = {4'h1, `P_ERR_ADDR_ADDR, 2'b00} ;
            config_read ( offset, 4'hf, read_data ) ;

            if ( read_data !== expect_address )
            begin
                test_fail("value in Erroneous Address register was incorrect") ;
                ok = 0 ;
            end

            offset = {4'h1, `P_ERR_DATA_ADDR, 2'b00} ;
            config_read ( offset, 4'hf, read_data ) ;

            if ( read_data !== 32'hAAAA_AAAA )
            begin
                test_fail("value in Erroneous Data register was incorrect") ;
                ok = 0 ;
            end

            if ( ok )
                test_ok ;

        end
        else
        begin
            if ( read_data[8] !== 0 )
            begin
                test_fail("error terminated write on WISHBONE set Error bit when error reporting was disabled" ) ;
                ok = 0 ;
            end
            else
                test_ok ;
        end

        test_name = "INTERRUPT STATUS REGISTER VALUE AFTER ERROR TERMINATED WRITE ON WISHBONE" ;
        offset = {4'h1, `ISR_ADDR, 2'b00} ;
        ok = 1 ;

        config_read ( offset, 4'hF, read_data ) ;
        if ( enable_error_report && enable_error_interrupt )
        begin
            if ( read_data[2] !== 1 )
            begin
                test_fail("PCI Error Interrupt Status bit was not set when expected") ;
                ok = 0 ;
            end

            test_name = "CLEARING INTERRUPT STATUS" ;
            config_write( offset, read_data, 4'hF, ok ) ;
            if ( !ok )
                test_fail("Interrupt Status register could not be written") ;
        end
        else
        begin
            if ( read_data[2] !== 0 )
            begin
                test_fail("PCI Error Interrupt Status bit was set when Error Interrupts were disabled") ;
                ok = 0 ;
            end
        end

        if ( ok )
            test_ok ;

        test_name = "INTERRUPT REQUEST DEASSERTION AFTER CLEARING INTERRUPT STATUS" ;
        `ifdef HOST

            repeat ( 4 )
                @( posedge wb_clock ) ;

            if ( INT_O !== 0 )
            begin
                test_fail("bridge asserted interrupt request on WISHBONE for no apparent reason") ;
            end
            else
                test_ok ;

        `else
            repeat ( 4 )
                @( posedge pci_clock ) ;

            if ( INTA !== 1 )
            begin
                test_fail("bridge asserted interrupt request on PCI for no apparent reason") ;
            end
            else
                test_ok ;

        `endif

        pci_address = pci_address + 1 ;
        expect_address = expect_address + 1 ;
    end

end
endtask // test_target_io_err_wr

task test_pci_image ;
    input [2:0]  image_num ;
    reg   [11:0] pci_ctrl_offset ;
    reg   [11:0] ctrl_offset ;
    reg   [11:0] ba_offset ;
    reg   [11:0] am_offset ;
    reg   [11:0] ta_offset ;
    reg   [7:0]  cache_lsize ;
    reg          ok ;
    reg          test_io ;
    reg          test_mem ;
begin
    pci_ctrl_offset = 12'h4 ;
    if (image_num === 0)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL0_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA0_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM0_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA0_ADDR, 2'b00} ;
    end
    else if (image_num === 1)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL1_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA1_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM1_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA1_ADDR, 2'b00} ;
    end
    else if (image_num === 2)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL2_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA2_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM2_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA2_ADDR, 2'b00} ;
    end
    else if (image_num === 3)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL3_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA3_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM3_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA3_ADDR, 2'b00} ;
    end
    else if (image_num === 4)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL4_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA4_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM4_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA4_ADDR, 2'b00} ;
    end
    else if (image_num === 5)
    begin
        ctrl_offset = {4'h1, `P_IMG_CTRL5_ADDR, 2'b00} ;
        ba_offset   = {4'h1, `P_BA5_ADDR, 2'b00} ;
        am_offset   = {4'h1, `P_AM5_ADDR, 2'b00} ;
        ta_offset   = {4'h1, `P_TA5_ADDR, 2'b00} ;
    end

    `ifdef HOST
        test_io  = 1 ;
        test_mem = 1 ;
    `else
        if (image_num == 1)
            test_io = `PCI_BA1_MEM_IO ;
        else if ( image_num == 2 )
            test_io = `PCI_BA2_MEM_IO ;
        else if ( image_num == 3 )
            test_io = `PCI_BA3_MEM_IO ;
        else if ( image_num == 4 )
            test_io = `PCI_BA4_MEM_IO ;
        else if ( image_num == 5 )
            test_io = `PCI_BA5_MEM_IO ;

        test_mem = !test_io ;
    `endif

    $display(" ");
    $display("########################################################################") ;
    $display("Setting the IMAGE %d configuration registers (P_BA, P_AM, P_TA)",image_num);
    test_name = "PCI IMAGE SETTINGS" ;

    // Set Base Address of IMAGE
    config_write( ba_offset, Target_Base_Addr_R[image_num], 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write P_BA%d register! Time %t ",image_num ,$time);
        test_fail("PCI Base Address register could not be written") ;
    end

    // Set Address Mask of IMAGE
    config_write( am_offset, Target_Addr_Mask_R[image_num], 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write P_AM%d register! Time %t ",image_num ,$time);
        test_fail("PCI Address Mask register could not be written") ;
    end

    // Set Translation Address of IMAGE
    config_write( ta_offset, Target_Tran_Addr_R[image_num], 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write P_TA%d register! Time %t ",image_num ,$time);
        test_fail("PCI Translation Address register could not be written") ;
    end

// Following are defines for byte enable signals !
//      Byte Masks
//      `Test_Byte_0                            (4'b1110)
//      `Test_Byte_1                            (4'b1101)
//      `Test_Byte_2                            (4'b1011)
//      `Test_Byte_3                            (4'b0111)
//      `Test_Half_0                            (4'b1100)
//      `Test_Half_1                            (4'b0011)
//      `Test_All_Bytes                         (4'b0000)

// "TEST NORMAL SINGLE WRITE READ THROUGH IMAGE WITHOUT ADDRESS TRANSLATION AND WITHOUT PREFETCABLE IMAGES"
    // Set Cache Line Size
    cache_lsize = 8'h4 ;

    $display(" ");
    $display("Setting the Cache Line Size register to %d word(s)", cache_lsize);
    config_write( pci_ctrl_offset + 12'h8, {24'h0000_00, cache_lsize}, 4'h1, ok) ;
    if ( ok !== 1 )
    begin
        $display("Image testing failed! Failed to write Cache Line Size register! Time %t ",$time);
        test_fail("PCI Device Control and Status register could not be written") ;
    end

    if (test_mem)
    begin
        $display("Do single WR / RD test through the IMAGE %d !",image_num);
        // Task test_normal_wr_rd has the following parameters:
        //  [2:0]Master_ID, [31:0]Address, [31:0]Data, [3:0]Be, [2:0]Image_num,
        //  [3:0]Set_size, Set_addr_translation, Set_prefetch_enable, [7:0]Cache_lsize, Set_wb_wait_states,
        //  MemRdLn_or_MemRd_when_cache_lsize_read.
        test_normal_wr_rd( `Test_Master_2, Target_Base_Addr_R[image_num], 32'h1234_5678, `Test_All_Bytes, image_num,
                            `Test_One_Word, 1'b0, 1'b1, cache_lsize, 1'b0, 1'b0 );

    // TEST NORMAL BURST WRITE READ THROUGH IMAGE WITHOUT ADDRESS TRANSLATION AND WITH PREFETCABLE IMAGES
        // Set Cache Line Size
        cache_lsize = 8'h4 ;

        $display(" ");
        $display("Setting the Cache Line Size register to %d word(s)", cache_lsize);
        config_write( pci_ctrl_offset + 12'h8, {24'h0000_00, cache_lsize}, 4'h1, ok) ;
        if ( ok !== 1 )
        begin
            $display("Image testing failed! Failed to write Cache Line Size register! Time %t ",$time);
            test_fail("Cache Line Size register could not be written" ) ;
        end

        $display("Do burst (2 words) WR / RD test through the IMAGE %d !",image_num);
        // Task test_normal_wr_rd has the following parameters:
        //  [2:0]Master_ID, [31:0]Address, [31:0]Data, [3:0]Be, [2:0]Image_num,
        //  [3:0]Set_size, Set_addr_translation, Set_prefetch_enable, [7:0]Cache_lsize, Set_wb_wait_states,
        //  MemRdLn_or_MemRd_when_cache_lsize_read.
        test_normal_wr_rd( `Test_Master_2, Target_Base_Addr_R[image_num] + 32'h4, 32'h5a5a_5a5a, `Test_Half_0, image_num,
                            `Test_Two_Words, 1'b0, 1'b1, cache_lsize, 1'b1, 1'b0 );

    // TEST NORMAL BURST WRITE READ THROUGH IMAGE WITH ADDRESS TRANSLATION AND WITH PREFETCABLE IMAGES
        // Set Cache Line Size
        cache_lsize = 8'h8 ;

        $display(" ");
        $display("Setting the Cache Line Size register to %d word(s)", cache_lsize);
        config_write( pci_ctrl_offset + 12'h8, {24'h0000_00, cache_lsize}, 4'h1, ok) ;
        if ( ok !== 1 )
        begin
            $display("Image testing failed! Failed to write Cache Line Size register! Time %t ",$time);
            test_fail("Cache Line Size register could not be written" ) ;
        end

        $display("Do burst (3 words) WR / RD test through the IMAGE %d !",image_num);
        // Task test_normal_wr_rd has the following parameters:
        //  [2:0]Master_ID, [31:0]Address, [31:0]Data, [3:0]Be, [2:0]Image_num,
        //  [3:0]Set_size, Set_addr_translation, Set_prefetch_enable, [7:0]Cache_lsize, Set_wb_wait_states,
        //  MemRdLn_or_MemRd_when_cache_lsize_read.
        test_normal_wr_rd( `Test_Master_2, Target_Base_Addr_R[image_num] + 32'h10, 32'h3c3c_3c3c, `Test_Half_1, image_num,
                            `Test_Three_Words, 1'b1, 1'b1, cache_lsize, 1'b1, 1'b1 );

    // TEST NORMAL BURST WRITE READ THROUGH IMAGE WITH ADDRESS TRANSLATION AND WITH PREFETCABLE IMAGES
        // Set Cache Line Size
        cache_lsize = 8'h4 ;

        $display(" ");
        $display("Setting the Cache Line Size register to %d word(s)", cache_lsize);
        config_write( pci_ctrl_offset + 12'h8, {24'h0000_00, cache_lsize}, 4'h1, ok) ;
        if ( ok !== 1 )
        begin
            $display("Image testing failed! Failed to write Cache Line Size register! Time %t ",$time);
            test_fail("Cache Line Size register could not be written" ) ;
        end

        $display("Do burst (full fifo depth words) WR / RD test through the IMAGE %d !",image_num);
        // Task test_normal_wr_rd has the following parameters:
        //  [2:0]Master_ID, [31:0]Address, [31:0]Data, [3:0]Be, [2:0]Image_num,
        //  [3:0]Set_size, Set_addr_translation, Set_prefetch_enable, [7:0]Cache_lsize, Set_wb_wait_states,
        //  MemRdLn_or_MemRd_when_cache_lsize_read.
        test_normal_wr_rd( `Test_Master_2, Target_Base_Addr_R[image_num] + 32'h20, 32'h7147_a5c3, `Test_Byte_2, image_num,
                            `PCIW_DEPTH - 2, 1'b1, 1'b1, cache_lsize, 1'b1, 1'b1 );

    // TEST ERRONEOUS SINGLE WRITE THROUGH IMAGE WITHOUT ERROR AND INTERRUPT REPORTING
        $display(" ");
        $display("Do single erroneous WR test through the IMAGE %d !",image_num);
        // Task test_wb_error_wr has the following parameters: [2:0]Master_ID, [31:0]Address, [31:0]Data, [3:0]Be,
        //  [2:0]Image_num, [3:0]Set_size, Set_err_and_int_report, Set_wb_wait_states, [1:0]Imm_BefLast_Last_error.
        test_wb_error_wr( `Test_Master_2, Target_Base_Addr_R[image_num], 32'h1234_5678, `Test_All_Bytes,
                            image_num, `Test_One_Word, 1'b0, 1'b0, 2'h0 );

    // TEST ERRONEOUS BURST WRITE THROUGH IMAGE WITH ERROR AND INTERRUPT REPORTING
        $display(" ");
        $display("Do burst (2 words) erroneous WR test through the IMAGE %d !",image_num);
        // Task test_wb_error_wr has the following parameters: [2:0]Master_ID, [31:0]Address, [31:0]Data, [3:0]Be,
        //  [2:0]Image_num, [3:0]Set_size, Set_err_and_int_report, Set_wb_wait_states, [1:0]Imm_BefLast_Last_error.
        test_wb_error_wr( `Test_Master_2, Target_Base_Addr_R[image_num] + 32'h30, 32'h1234_5678, `Test_Half_0,
                            image_num, `Test_Two_Words, 1'b1, 1'b0, 2'h0 );

    // TEST ERRONEOUS BURST WRITE THROUGH IMAGE WITHOUT ERROR AND INTERRUPT REPORTING
        $display(" ");
        $display("Do burst (3 words) erroneous WR test through the IMAGE %d !",image_num);
        // Task test_wb_error_wr has the following parameters: [2:0]Master_ID, [31:0]Address, [31:0]Data, [3:0]Be,
        //  [2:0]Image_num, [3:0]Set_size, Set_err_and_int_report, Set_wb_wait_states, [1:0]Imm_BefLast_Last_error.
        test_wb_error_wr( `Test_Master_2, Target_Base_Addr_R[image_num] + 32'h40, 32'h1234_5678, `Test_Half_1,
                            image_num, `Test_Three_Words, 1'b0, 1'b1, 2'h2 );

    // TEST ERRONEOUS BURST WRITE THROUGH IMAGE WITH ERROR AND INTERRUPT REPORTING
        $display(" ");
        $display("Do burst (8 words) erroneous WR test through the IMAGE %d !",image_num);
        // Task test_wb_error_wr has the following parameters: [2:0]Master_ID, [31:0]Address, [31:0]Data, [3:0]Be,
        //  [2:0]Image_num, [3:0]Set_size, Set_err_and_int_report, Set_wb_wait_states, [1:0]Imm_BefLast_Last_error.
        test_wb_error_wr( `Test_Master_2, Target_Base_Addr_R[image_num] + 32'h50, 32'h1234_5678, `Test_Byte_1,
                            image_num, `Test_Eight_Words, 1'b1, 1'b1, 2'h1 );
    end

    if ( test_io )
    begin
        test_name = "PCI IMAGE SETTINGS" ;

        // Set Base Address of IMAGE
        config_write( ba_offset, Target_Base_Addr_R[image_num] | 1, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Image testing failed! Failed to write P_BA%d register! Time %t ",image_num ,$time);
            test_fail("PCI Base Address register could not be written") ;
        end

        // Set Address Mask of IMAGE
        config_write( am_offset, Target_Addr_Mask_R[image_num], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Image testing failed! Failed to write P_AM%d register! Time %t ",image_num ,$time);
            test_fail("PCI Address Mask register could not be written") ;
        end

        // Set Translation Address of IMAGE
        config_write( ta_offset, Target_Tran_Addr_R[image_num], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Image testing failed! Failed to write P_TA%d register! Time %t ",image_num ,$time);
            test_fail("PCI Translation Address register could not be written") ;
        end

        // Set Cache Line Size
        cache_lsize = 8'h4 ;

        $display(" ");
        $display("Setting the Cache Line Size register to %d word(s)", cache_lsize);
        config_write( pci_ctrl_offset + 12'h8, {24'h0000_00, cache_lsize}, 4'h1, ok) ;
        if ( ok !== 1 )
        begin
            $display("Image testing failed! Failed to write Cache Line Size register! Time %t ",$time);
            test_fail("Cache Line Size register could not be written" ) ;
        end

        test_target_io_wr_rd
        (
            image_num,    // image number
            0,            // test with address translation
            ctrl_offset   // image control register offset
        ) ;

        test_target_io_wr_rd
        (
            image_num,    // image number
            1,            // test with address translation
            ctrl_offset   // image control register offset
        ) ;

        test_target_io_err_wr
        (
            image_num,      // image number
            0,              // address translation on/off
            ctrl_offset,    // image control register offset
            0,              // enable error reporting
            0               // enable error interrupts
        ) ;

        test_target_io_err_wr
        (
            image_num,      // image number
            1,              // address translation on/off
            ctrl_offset,    // image control register offset
            0,              // enable error reporting
            1               // enable error interrupts
        ) ;

        test_target_io_err_wr
        (
            image_num,      // image number
            0,              // address translation on/off
            ctrl_offset,    // image control register offset
            1,              // enable error reporting
            0               // enable error interrupts
        ) ;

        test_target_io_err_wr
        (
            image_num,      // image number
            1,              // address translation on/off
            ctrl_offset,    // image control register offset
            1,              // enable error reporting
            1               // enable error interrupts
        ) ;
    end

    // Test master abort with NON supported commands
    target_unsupported_cmds( Target_Base_Addr_R[image_num], image_num ) ;

    // disable the image
    config_write( am_offset, Target_Addr_Mask_R[image_num] & 32'h7FFF_FFFF, 4'hF, ok ) ;
end
endtask //test_pci_image

task target_fast_back_to_back ;
    reg   [11:0] pci_ctrl_offset ;
    reg   [11:0] ctrl_offset ;
    reg   [11:0] ba_offset ;
    reg   [11:0] am_offset ;
    reg   [11:0] ta_offset ;
    reg   [11:0] cls_offset ;
    reg          do_mem_fb2b ;
    reg          do_io_fb2b ;
    reg          ok ;
begin:main

    if ( target_mem_image !== -1 )
    begin
        do_mem_fb2b = 1 ;

        if (target_mem_image === 1)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL1_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA1_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM1_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA1_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 2)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL2_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA2_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM2_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA2_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 3)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL3_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA3_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM3_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA3_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 4)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL4_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA4_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM4_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA4_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 5)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL5_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA5_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM5_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA5_ADDR, 2'b00} ;
        end
    end
    else
        do_mem_fb2b = 0 ;

    pci_ctrl_offset = 12'h4 ;
    cls_offset      = 12'h00C ;

    if ( do_mem_fb2b )
    begin
        test_name = "CONFIGURE TARGET FOR FAST B2B TESTING" ;
        config_write( ba_offset, Target_Base_Addr_R[target_mem_image], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write P_BA%d register! Time %t ", 1 ,$time);
            test_fail("PCI Base Address register could not be written") ;
            disable main ;
        end

        // Set Address Mask of IMAGE
        config_write( am_offset, Target_Addr_Mask_R[target_mem_image], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write P_AM%d register! Time %t ",1 ,$time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main ;
        end

        // Set Translation Address of IMAGE
        config_write( ta_offset, Target_Tran_Addr_R[target_mem_image], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write P_TA%d register! Time %t ",1 ,$time);
            test_fail("PCI Translation Address Register could not be written") ;
            disable main ;
        end

        config_write( ctrl_offset, 32'h0000_0002, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write P_IMG_CTRL%d register! Time %t ",1 ,$time);
            test_fail("PCI Image Control register could not be written") ;
            disable main ;
        end

        config_write( cls_offset, 32'h0000_00_04, 4'b0001, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write Cache Line Size register! Time %t ", $time);
            test_fail("Cache Line Size register could not be written") ;
            disable main ;
        end

        // enable master 1 fast_b2b
        configuration_cycle_write(0,             // bus number
                                  1,             // device number
                                  0,             // function number
                                  1,             // register number
                                  0,             // type of configuration cycle
                                  4'b1111,       // byte enables
                                  32'hFFFF_FFFF  // data
                                 ) ;

        wishbone_slave.cycle_response(3'b001, tb_subseq_waits, 0) ;

        test_name = "FAST BACK TO BACK THROUGH TARGET - FILL WRITE FIFO, CHECK RETRY ON FAST B2B WRITE" ;
        fork
        begin
            DO_REF ("MEM_WRITE ", 1, Target_Base_Addr_R[target_mem_image],
                  PCI_COMMAND_MEMORY_WRITE, 32'h1234_5678, 4'hF,
                  `PCIW_DEPTH - 2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                  0, `Test_One_Zero_Target_WS,
                  `Test_Devsel_Medium, `Test_Fast_B2B,
                  `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);

            DO_REF ("MEM_WRITE ", 1, Target_Base_Addr_R[target_mem_image],
                  PCI_COMMAND_MEMORY_WRITE, 32'h1234_5678, 4'hF,
                  1, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                  0, `Test_One_Zero_Target_WS,
                  `Test_Devsel_Medium, `Test_Fast_B2B,
                  `Test_Target_Retry_On, `Test_Expect_No_Master_Abort);
            do_pause(5) ;

            wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 0) ;
        end
        begin:wb_monitor1
            wb_transaction_progress_monitor(Target_Base_Addr_R[target_mem_image], 1'b1, `PCIW_DEPTH - 2, 1'b1, ok) ;
            if ( ok !== 1 )
                test_fail("WISHBONE master did invalid transaction or none at all on WISHBONE bus") ;

            disable monitor_error_event1 ;
        end
        begin:monitor_error_event1
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while testing Fast Back to Back through PCI Target Unit") ;
            ok = 0 ;
            disable wb_monitor1 ;
        end
        join

        if ( ok )
            test_ok ;

        test_name = "FAST BACK TO BACK THROUGH TARGET - BOTH WRITES SMALL ENOUGH TO PROCEEDE THROUGH FIFO" ;
        wishbone_slave.cycle_response(3'b001, tb_subseq_waits, 0) ;
        fork
        begin
            DO_REF ("MEM_WRITE ", 1, Target_Base_Addr_R[target_mem_image],
                  PCI_COMMAND_MEMORY_WRITE, 32'h1234_5678, 4'hF,
                  `PCIW_DEPTH - 6, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                  0, `Test_One_Zero_Target_WS,
                  `Test_Devsel_Medium, `Test_Fast_B2B,
                  `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);

            DO_REF ("MEM_WRITE ", 1, Target_Base_Addr_R[target_mem_image] + ((`PCIW_DEPTH - 6) * 4),
                  PCI_COMMAND_MEMORY_WRITE, 32'h1234_5678, 4'hF,
                  2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                  0, `Test_One_Zero_Target_WS,
                  `Test_Devsel_Medium, `Test_Fast_B2B,
                  `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);
            do_pause(5) ;

            wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 0) ;

        end
        begin:wb_monitor2
            wb_transaction_progress_monitor(Target_Base_Addr_R[target_mem_image], 1'b1, `PCIW_DEPTH - 6, 1'b1, ok) ;
            if ( ok !== 1 )
                test_fail("WISHBONE master did invalid transaction or none at all on WISHBONE bus") ;
            else
            begin
                wb_transaction_progress_monitor(Target_Base_Addr_R[target_mem_image] + ((`PCIW_DEPTH - 6) * 4), 1'b1, 2, 1'b1, ok) ;
                if ( ok !== 1 )
                    test_fail("WISHBONE master did invalid second transaction or none at all on WISHBONE bus when it was requested as fast back to back") ;
            end

            disable monitor_error_event2 ;
        end
        begin:monitor_error_event2
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while testing Fast Back to Back through PCI Target Unit") ;
            ok = 0 ;
            disable wb_monitor2 ;
        end
        join

        if ( ok )
            test_ok ;

        test_name = "FAST BACK TO BACK THROUGH TARGET - FIRST WRITE FULL FIFO, THEN READ BACK" ;
        wishbone_slave.cycle_response(3'b001, tb_subseq_waits, 0) ;
        fork
        begin
            DO_REF ("MEM_WRITE ", 1, Target_Base_Addr_R[target_mem_image],
                  PCI_COMMAND_MEMORY_WRITE, 32'h1234_5678, 4'hF,
                  `PCIW_DEPTH - 2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                  0, `Test_One_Zero_Target_WS,
                  `Test_Devsel_Medium, `Test_Fast_B2B,
                  `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);

            DO_REF ("MEM_READ  ", 1, Target_Base_Addr_R[target_mem_image],
                  `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                  2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                  0, `Test_One_Zero_Target_WS,
                  `Test_Devsel_Medium, `Test_Fast_B2B,
                  `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause(5) ;
            wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 0) ;

        end
        begin:wb_monitor3
            fork
            begin
                wb_transaction_progress_monitor(Target_Base_Addr_R[target_mem_image], 1'b1, `PCIW_DEPTH - 2, 1'b1, ok) ;
                if ( ok !== 1 )
                    test_fail("WISHBONE master did invalid transaction or none at all on WISHBONE bus") ;
            end
            begin
                wb_transaction_stop(`PCIW_DEPTH - 2) ;
                wb_transaction_progress_monitor_backup(Target_Base_Addr_R[target_mem_image], 1'b0, 4, 1'b1, ok) ;
                if ( ok !== 1 )
                    test_fail("WISHBONE master did invalid second transaction or none at all on WISHBONE bus when it was requested as fast back to back") ;
            end
            join

            if ( ok )
            begin
                fork
                begin
                    do_pause(3) ;

                    DO_REF ("MEM_WRITE ", 1, Target_Base_Addr_R[target_mem_image],
                        PCI_COMMAND_MEMORY_WRITE, 32'h8765_4321, 4'hF,
                        `PCIW_DEPTH - 2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Fast_B2B,
                        `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);

                    DO_REF ("MEM_READ  ", 1, Target_Base_Addr_R[target_mem_image],
                            `BC_MEM_READ_LN, 32'h1234_5678, 4'hF,
                            4, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                            0, `Test_One_Zero_Target_WS,
                            `Test_Devsel_Medium, `Test_Fast_B2B,
                            `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);

                    do_pause(1) ;
                end
                begin
                    pci_transaction_progress_monitor(Target_Base_Addr_R[target_mem_image], `BC_MEM_WRITE, `PCIW_DEPTH - 2, 0, 1'b1, 1'b0, 0, ok) ;
                    if ( ok !== 1 )
                    begin
                        test_fail("unexpected transaction was detected on PCI when FastB2B read was repeated") ;
                        disable monitor_error_event3 ;
                    end
                    else
                    begin
                        pci_transaction_progress_monitor(Target_Base_Addr_R[target_mem_image], `BC_MEM_READ_LN, 4, 0, 1'b1, 1'b0, 1, ok) ;
                        if ( ok !== 1 )
                            test_fail("unexpected transaction was detected on PCI when FastB2B read was repeated") ;
                    end
                end
                begin
           		    wb_transaction_progress_monitor(Target_Base_Addr_R[target_mem_image], 1'b1, `PCIW_DEPTH - 2, 1'b1, ok) ;
       		        if ( ok !== 1 )
   		                test_fail("WISHBONE master did invalid transaction or none at all on WISHBONE bus") ;
                end
                join
            end
            disable monitor_error_event3 ;
        end
        begin:monitor_error_event3
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while testing Fast Back to Back through PCI Target Unit") ;
            ok = 0 ;
            disable wb_monitor3 ;
        end
        join

        if ( ok )
            test_ok ;

        test_name = "DISABLING MEM IMAGE" ;
        config_write( am_offset, Target_Addr_Mask_R[target_mem_image] & 32'h7FFF_FFFF, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write P_AM%d register! Time %t ",1 ,$time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main ;
        end
    end

    if ( target_io_image !== -1 )
    begin
        do_io_fb2b = 1 ;

        if (target_io_image === 1)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL1_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA1_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM1_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA1_ADDR, 2'b00} ;
        end
        else if (target_io_image === 2)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL2_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA2_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM2_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA2_ADDR, 2'b00} ;
        end
        else if (target_io_image === 3)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL3_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA3_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM3_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA3_ADDR, 2'b00} ;
        end
        else if (target_io_image === 4)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL4_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA4_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM4_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA4_ADDR, 2'b00} ;
        end
        else if (target_io_image === 5)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL5_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA5_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM5_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA5_ADDR, 2'b00} ;
        end
    end
    else
        do_io_fb2b = 0 ;

    if ( do_io_fb2b )
    begin

        test_name = "CONFIGURE TARGET FOR FAST B2B TESTING" ;
        config_write( ba_offset, Target_Base_Addr_R[target_io_image] | 32'h0000_0001, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write P_BA%d register! Time %t ", 1 ,$time);
            test_fail("PCI Base Address register could not be written") ;
            disable main ;
        end

        // Set Address Mask of IMAGE
        config_write( am_offset, Target_Addr_Mask_R[target_io_image], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write P_AM%d register! Time %t ",1 ,$time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main ;
        end

        // Set Translation Address of IMAGE
        config_write( ta_offset, Target_Tran_Addr_R[target_io_image], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write P_TA%d register! Time %t ",1 ,$time);
            test_fail("PCI Translation Address Register could not be written") ;
            disable main ;
        end

        config_write( ctrl_offset, 32'h0000_0002, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write P_IMG_CTRL%d register! Time %t ",1 ,$time);
            test_fail("PCI Image Control register could not be written") ;
            disable main ;
        end

        config_write( cls_offset, 32'h0000_00_04, 4'b0001, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write Cache Line Size register! Time %t ", $time);
            test_fail("Cache Line Size register could not be written") ;
            disable main ;
        end

        // enable master 1 fast_b2b
        configuration_cycle_write(0,             // bus number
                                  1,             // device number
                                  0,             // function number
                                  1,             // register number
                                  0,             // type of configuration cycle
                                  4'b1111,       // byte enables
                                  32'hFFFF_FFFF  // data
                                 ) ;

        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 0) ;

        test_name = "FAST BACK TO BACK THROUGH TARGET - TWO SINGLE IO WRITES" ;
        fork
        begin
            DO_REF ("IO_WRITE  ", 1, Target_Base_Addr_R[target_io_image] + 100,
                  `BC_IO_WRITE, 32'hA5A5_A5A5, 4'hF,
                  1, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                  0, `Test_One_Zero_Target_WS,
                  `Test_Devsel_Medium, `Test_Fast_B2B,
                  `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);

            DO_REF ("IO_WRITE  ", 1, Target_Base_Addr_R[target_io_image] + 104,
                  `BC_IO_WRITE, 32'h5A5A_5A5A, 4'hF,
                  1, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                  0, `Test_One_Zero_Target_WS,
                  `Test_Devsel_Medium, `Test_Fast_B2B,
                  `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);
            do_pause(5) ;

        end
        begin:wb_monitor4
            wb_transaction_progress_monitor(Target_Base_Addr_R[target_io_image] + 100, 1'b1, 1, 1'b1, ok) ;
            if ( ok !== 1 )
                test_fail("WISHBONE master did invalid transaction or none at all on WISHBONE bus") ;

            if ( ok )
            begin
                wb_transaction_progress_monitor(Target_Base_Addr_R[target_io_image] + 104, 1'b1, 1, 1'b1, ok) ;
                if ( ok !== 1 )
                    test_fail("WISHBONE master did invalid transaction or none at all on WISHBONE bus") ;
            end

            disable monitor_error_event4 ;
        end
        begin:monitor_error_event4
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while testing Fast Back to Back through PCI Target Unit") ;
            ok = 0 ;
            disable wb_monitor4 ;
        end
        join

        if ( ok )
            test_ok ;

        test_name = "FAST BACK TO BACK THROUGH TARGET - FIRST I/O WRITE, THEN READ BACK" ;
        wishbone_slave.cycle_response(3'b001, tb_subseq_waits, 0) ;
        fork
        begin
            DO_REF ("IO_WRITE  ", 1, Target_Base_Addr_R[target_io_image] + 40,
                  `BC_IO_WRITE, 32'hAAAA_AAAA, 4'hF,
                  1, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                  0, `Test_One_Zero_Target_WS,
                  `Test_Devsel_Medium, `Test_Fast_B2B,
                  `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);

            DO_REF ("IO_READ   ", 1, Target_Base_Addr_R[target_io_image] + 40,
                  `BC_IO_READ, 32'hAAAA_AAAA, 4'hF,
                  1, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                  0, `Test_One_Zero_Target_WS,
                  `Test_Devsel_Medium, `Test_Fast_B2B,
                  `Test_Target_Retry_On, `Test_Expect_No_Master_Abort);
            do_pause(5) ;
            wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 0) ;
        end
        begin:wb_monitor5
            fork
            begin
                wb_transaction_progress_monitor(Target_Base_Addr_R[target_io_image] + 40, 1'b1, 1, 1'b1, ok) ;
                if ( ok !== 1 )
                    test_fail("WISHBONE master did invalid transaction or none at all on WISHBONE bus") ;
            end
            begin
                wb_transaction_stop( 1 ) ;
                wb_transaction_progress_monitor_backup(Target_Base_Addr_R[target_io_image] + 40, 1'b0, 1, 1'b1, ok) ;
                if ( ok !== 1 )
                    test_fail("WISHBONE master did invalid second transaction or none at all on WISHBONE bus when it was requested as fast back to back") ;
            end
            join

            if ( ok )
            begin
                fork
                begin
                    do_pause(3) ;

                    DO_REF ("IO_WRITE  ", 1, Target_Base_Addr_R[target_io_image] + 40,
                            `BC_IO_WRITE, 32'h5555_5555, 4'hF,
                            1, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                            0, `Test_One_Zero_Target_WS,
                            `Test_Devsel_Medium, `Test_Fast_B2B,
                            `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);

                    DO_REF ("IO_READ   ", 1, Target_Base_Addr_R[target_io_image] + 40,
                            `BC_IO_READ, 32'hAAAA_AAAA, 4'hF,
                            1, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                            0, `Test_One_Zero_Target_WS,
                            `Test_Devsel_Medium, `Test_Fast_B2B,
                            `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);

                    do_pause(1) ;
                end
                begin
                    pci_transaction_progress_monitor(Target_Base_Addr_R[target_io_image] + 40, `BC_IO_WRITE, 1, 0, 1'b1, 1'b0, 0, ok) ;
                    if ( ok !== 1 )
                    begin
                        test_fail("unexpected transaction was detected on PCI when FastB2B read was repeated") ;
                        disable monitor_error_event5 ;
                    end
                    else
                    begin
                        pci_transaction_progress_monitor(Target_Base_Addr_R[target_io_image] + 40, `BC_IO_READ, 1, 0, 1'b1, 1'b0, 1, ok) ;
                        if ( ok !== 1 )
                            test_fail("unexpected transaction was detected on PCI when FastB2B read was repeated") ;
                    end
                end
                join
            end
            disable monitor_error_event5 ;
        end
        begin:monitor_error_event5
            @(error_event_int) ;
            test_fail("either PCI Monitor or PCI Master detected an error while testing Fast Back to Back through PCI Target Unit") ;
            ok = 0 ;
            disable wb_monitor5 ;
        end
        join

        if ( ok )
            test_ok ;

        test_name = "DISABLING IO IMAGE" ;
        config_write( am_offset, Target_Addr_Mask_R[target_io_image] & 32'h7FFF_FFFF, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Fast B2B testing failed! Failed to write P_AM%d register! Time %t ",1 ,$time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main ;
        end
    end

end
endtask //target_fast_back_to_back

task target_disconnects ;
    reg   [11:0] pci_ctrl_offset ;
    reg   [11:0] ctrl_offset ;
    reg   [11:0] ba_offset ;
    reg   [11:0] am_offset ;
    reg   [11:0] ta_offset ;
    reg   [11:0] cls_offset ;
    reg          pci_ok ;
    reg          wb_ok ;
    reg          ok ;
    reg   [31:0] pci_address ;
    reg   [31:0] data ;
    reg   [3:0]  byte_enables ;
    reg   [9:0]  expect_length ;

    reg          do_mem_disconnects ;
    reg          do_io_disconnects ;
    reg          error_monitor_done ;
begin:main
    if ( target_mem_image !== -1 )
    begin
        do_mem_disconnects = 1 ;

        if (target_mem_image === 1)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL1_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA1_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM1_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA1_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 2)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL2_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA2_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM2_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA2_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 3)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL3_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA3_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM3_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA3_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 4)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL4_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA4_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM4_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA4_ADDR, 2'b00} ;
        end
        else if (target_mem_image === 5)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL5_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA5_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM5_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA5_ADDR, 2'b00} ;
        end
    end
    else
        do_mem_disconnects = 0 ;

    pci_ctrl_offset = 12'h4 ;
    cls_offset = 12'h00C ;

    master1_check_received_data = 0 ;
    master2_check_received_data = 0 ;

    `ifdef HOST
        `ifdef NO_CNF_IMAGE
        `else
            `define TEST_BURST_CONFIG_READ
        `endif
    `else
        `define TEST_BURST_CONFIG_READ
        `define TEST_BURST_CONFIG_WRITE
    `endif

    `ifdef TEST_BURST_CONFIG_WRITE
        pci_address = Target_Base_Addr_R[0] + 12'h00C ;

        data = 32'h0000_08_08 ;

        test_name = "TARGET DISCONNECT ON BURST WRITE TO CONFIGURATION SPACE" ;
        byte_enables = 4'b0000 ;

        fork
        begin
            DO_REF ("MEM_W_CONF", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                    PCI_COMMAND_MEMORY_WRITE, data[PCI_BUS_DATA_RANGE:0],
                    byte_enables,
                    2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                    8'h0_0, `Test_One_Zero_Target_WS,
                    `Test_Devsel_Medium, `Test_Fast_B2B,
                    `Test_Target_Retry_On, `Test_Expect_No_Master_Abort);

            data = 32'h0000_04_04 ;
            DO_REF ("MEM_W_CONF", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                    PCI_COMMAND_MEMORY_WRITE, data[PCI_BUS_DATA_RANGE:0],
                    byte_enables,
                    3, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                    8'h0_0, `Test_One_Zero_Target_WS,
                    `Test_Devsel_Medium, `Test_Fast_B2B,
                    `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 1 ) ;
            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event1 ;
        end
        begin:monitor_error_event1
            error_monitor_done = 0 ;
            ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on bursts to configuration space") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        config_read(pci_address, 4'hF, data) ;
        if ( data [15:0] !== 16'h04_04 )
        begin
            test_fail("only the first data in a burst to configuration space should actually be written to it") ;
        end
        else if ( ok )
            test_ok ;

        pci_address  = {20'h0000_0, 1'b1, 11'h00C} ;
        data         = 32'h0000_0808 ;
        byte_enables = 4'h0 ;
        fork
        begin
            DO_REF ("CFG_WRITE ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                    `BC_CONF_WRITE, data[PCI_BUS_DATA_RANGE:0],
                    byte_enables,
                    2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                    8'h0_0, `Test_One_Zero_Target_WS,
                    `Test_Devsel_Medium, `Test_Fast_B2B,
                    `Test_Target_Retry_On, `Test_Expect_No_Master_Abort);

            data = 32'h0000_04_04 ;
            DO_REF ("CFG_WRITE ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                    `BC_CONF_WRITE, data[PCI_BUS_DATA_RANGE:0],
                    byte_enables,
                    3, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                    8'h0_0, `Test_One_Zero_Target_WS,
                    `Test_Devsel_Medium, `Test_Fast_B2B,
                    `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);

            do_pause( 1 ) ;
            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event2 ;
        end
        begin:monitor_error_event2
            error_monitor_done = 0 ;
            ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on bursts to configuration space") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        config_read(pci_address, 4'hF, data) ;
        if ( data [15:0] !== 16'h04_04 )
        begin
            test_fail("only the first data in a burst to configuration space should actually be written to it") ;
        end
        else if ( ok )
            test_ok ;
    `endif

    `ifdef TEST_BURST_CONFIG_READ
        pci_address = Target_Base_Addr_R[0] + 12'h00C ;

        data = 32'h0000_04_04 ;

        test_name = "TARGET DISCONNECT ON BURST READ FROM CONFIGURATION SPACE" ;
        byte_enables = 4'b0000 ;

        fork
        begin
            DO_REF ("MEM_R_CONF", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                    PCI_COMMAND_MEMORY_READ, data[PCI_BUS_DATA_RANGE:0],
                    byte_enables,
                    2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                    8'h0_0, `Test_One_Zero_Target_WS,
                    `Test_Devsel_Medium, `Test_Fast_B2B,
                    `Test_Target_Retry_On, `Test_Expect_No_Master_Abort);
            do_pause( 1 ) ;

            DO_REF ("MEM_R_CONF", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                    PCI_COMMAND_MEMORY_READ, data[PCI_BUS_DATA_RANGE:0],
                    byte_enables,
                    3, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                    8'h0_0, `Test_One_Zero_Target_WS,
                    `Test_Devsel_Medium, `Test_No_Fast_B2B,
                    `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 1 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            if ( !error_monitor_done )
                disable monitor_error_event3 ;
        end
        begin:monitor_error_event3
            error_monitor_done = 0 ;
            ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on bursts to configuration space") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( ok )
            test_ok ;

        pci_address  = {20'h0000_0, 1'b1, 11'h00C} ;
        fork
        begin
            DO_REF ("CFG_READ  ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                    `BC_CONF_READ, data[PCI_BUS_DATA_RANGE:0],
                    byte_enables,
                    2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                    8'h0_0, `Test_One_Zero_Target_WS,
                    `Test_Devsel_Medium, `Test_No_Fast_B2B,
                    `Test_Target_Retry_On, `Test_Expect_No_Master_Abort);
            do_pause( 1 ) ;

            DO_REF ("CFG_READ  ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                    `BC_CONF_READ, data[PCI_BUS_DATA_RANGE:0],
                    byte_enables,
                    3, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                    8'h0_0, `Test_One_Zero_Target_WS,
                    `Test_Devsel_Medium, `Test_No_Fast_B2B,
                    `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 1 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event4 ;
        end
        begin:monitor_error_event4
            error_monitor_done = 0 ;
            ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on bursts to configuration space") ;
            ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( ok )
            test_ok ;
    `endif

    `ifdef TEST_BURST_CONFIG_READ
        `undef TEST_BURST_CONFIG_READ
    `endif

    `ifdef TEST_BURST_CONFIG_WRITE
        `undef TEST_BURST_CONFIG_WRITE
    `endif

    master1_check_received_data = 1 ;
    master2_check_received_data = 1 ;

    if ( do_mem_disconnects )
    begin
        test_name = "CONFIGURE TARGET FOR DISCONNECT TESTING" ;
        config_write( ba_offset, Target_Base_Addr_R[target_mem_image], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write P_BA%d register! Time %t ", 1 ,$time);
            test_fail("PCI Base Address register could not be written") ;
            disable main ;
        end

        // Set Address Mask of IMAGE
        config_write( am_offset, Target_Addr_Mask_R[target_mem_image], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write P_AM%d register! Time %t ",1 ,$time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main ;
        end

        // Set Translation Address of IMAGE
        config_write( ta_offset, Target_Tran_Addr_R[target_mem_image], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write P_TA%d register! Time %t ",1 ,$time);
            test_fail("PCI Translation Address Register could not be written") ;
            disable main ;
        end

        config_write( ctrl_offset, 32'h0000_0002, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write P_IMG_CTRL%d register! Time %t ",1 ,$time);
            test_fail("PCI Image Control register could not be written") ;
            disable main ;
        end

        config_write( cls_offset, 32'h0000_04_04, 4'b0011, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write Cache Line Size register! Time %t ", $time);
            test_fail("Cache Line Size register could not be written") ;
            disable main ;
        end

        test_name = "TARGET DISCONNECT WHEN WRITE FIFO FILLED DURING BURST WRITE" ;
        pci_address = Target_Base_Addr_R[target_mem_image] ;
        data = 32'hAAAA_AAAA ;
        byte_enables = 4'h0 ;
        expect_length = `PCIW_DEPTH - 2 ;

        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);
        fork
        begin
            DO_REF ("MEM_WRITE ", `Test_Master_1, pci_address[PCI_BUS_DATA_RANGE:0],
                        PCI_COMMAND_MEMORY_WRITE, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        (tb_subseq_waits != 4) ? expect_length : (expect_length + 1), `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        (tb_subseq_waits != 4) ? `Test_Target_Disc_On : `Test_Target_Retry_On, `Test_Expect_No_Master_Abort);
            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event5 ;
        end
        begin:monitor_error_event5
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target write burst to WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_progress_monitor(pci_address, 1'b1, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        test_name = "TARGET DISCONNECT WHEN WRITE FIFO FILLED DURING BURST WRITE" ;
        pci_address = Target_Base_Addr_R[target_mem_image] ;
        data = 32'hAAAA_AAAA ;
        byte_enables = 4'h0 ;
        expect_length = `PCIW_DEPTH - 2 ;

        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);
        fork
        begin
            DO_REF ("MEM_WRITE ", `Test_Master_1, pci_address[PCI_BUS_DATA_RANGE:0],
                        PCI_COMMAND_MEMORY_WRITE, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        (tb_subseq_waits != 4) ? (expect_length + 1) : (expect_length + 2) , `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        (tb_subseq_waits != 4) ? `Test_Target_Disc_Before : `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event6 ;
        end
        begin:monitor_error_event6
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target write burst to WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_progress_monitor(pci_address, 1'b1, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        master1_check_received_data = 1 ;
//        master2_check_received_data = 0 ;
        test_name = "TARGET DISCONNECT WHEN READ FIFO IS EMPTIED DURING BURST READ" ;
        pci_address = Target_Base_Addr_R[target_mem_image] ;
        data = 32'hAAAA_AAAA ;
        byte_enables = 4'h0 ;
        expect_length = 4 ;

        fork
        begin
            DO_REF ("MEM_READ ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                        `BC_MEM_READ_LN, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 1 ) ;

            wb_transaction_progress_monitor(pci_address, 1'b0, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;

            do_pause(2) ;

            DO_REF ("MEM_READ ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                        `BC_MEM_READ_LN, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        expect_length + 1, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        (tb_subseq_waits == 4) ? `Test_Target_Retry_On : `Test_Target_Disc_Before, `Test_Expect_No_Master_Abort);
            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event7 ;
        end
        begin:monitor_error_event7
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target read burst from WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        master1_check_received_data = 1 ;
        test_name = "TARGET DISCONNECT WHEN READ FIFO IS EMPTIED DURING BURST READ" ;
        pci_address = Target_Base_Addr_R[target_mem_image] ;
        data = 32'hAAAA_AAAA ;
        byte_enables = 4'h0 ;
        expect_length = 4 ;

        fork
        begin
            DO_REF ("MEM_READ ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                        `BC_MEM_READ_LN, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 1 ) ;

            wb_transaction_progress_monitor(pci_address, 1'b0, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;

            do_pause(2) ;

            DO_REF ("MEM_READ ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
                        `BC_MEM_READ_LN, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        expect_length, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Normal_Completion/*Test_Target_Disc_On*/, `Test_Expect_No_Master_Abort);
            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event8 ;
        end
        begin:monitor_error_event8
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target read burst from WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        test_name = "TARGET DISCONNECT ON WRITES WITH UNSUPPORTED WRAPING MODES" ;
        pci_address = Target_Base_Addr_R[target_mem_image] + 100 ;
        data = 32'hDEAF_BEAF ;
        byte_enables = 4'h0 ;
        expect_length = 1 ;

        fork
        begin
            DO_REF ("MEM_WRITE ", `Test_Master_1, pci_address[PCI_BUS_DATA_RANGE:0] + 1,
                        PCI_COMMAND_MEMORY_WRITE, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        expect_length + 1, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Retry_On, `Test_Expect_No_Master_Abort);
            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event9 ;
        end
        begin:monitor_error_event9
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target write burst to WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_progress_monitor(pci_address, 1'b1, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        pci_address = Target_Base_Addr_R[target_mem_image] + 104 ;
        data = 32'hDEAD_BEAF ;
        byte_enables = 4'h0 ;
        expect_length = 1 ;

        fork
        begin
            DO_REF ("MEM_WRITE ", `Test_Master_1, pci_address[PCI_BUS_DATA_RANGE:0] + 2,
                        PCI_COMMAND_MEMORY_WRITE, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        expect_length + 2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event10 ;
        end
        begin:monitor_error_event10
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target write burst to WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_progress_monitor(pci_address, 1'b1, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        pci_address = Target_Base_Addr_R[target_mem_image] + 108 ;
        data = 32'hAAAA_AAAA ;
        byte_enables = 4'h0 ;
        expect_length = 1 ;

        fork
        begin
            DO_REF ("MEM_WRITE ", `Test_Master_1, pci_address[PCI_BUS_DATA_RANGE:0] + 3,
                        PCI_COMMAND_MEMORY_WRITE, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        expect_length + 2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event11 ;
        end
        begin:monitor_error_event11
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target write burst to WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_progress_monitor(pci_address, 1'b1, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        master1_check_received_data = 1 ;

        test_name = "TARGET DISCONNECT ON READS WITH UNSUPPORTED WRAPING MODES" ;
        pci_address = Target_Base_Addr_R[target_mem_image] + 100 ;
        data = 32'hDEAF_BEAF ;
        byte_enables = 4'h0 ;
        expect_length = 1 ;

        fork
        begin
            DO_REF ("MEM_READ ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0] + 1,
                        `BC_MEM_READ_LN, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 1 ) ;

            wb_transaction_progress_monitor(pci_address, 1'b0, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;

            do_pause(3) ;

            DO_REF ("MEM_READ ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0] + 1,
                        `BC_MEM_READ_LN, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Retry_On, `Test_Expect_No_Master_Abort);
            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event12 ;
        end
        begin:monitor_error_event12
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target read burst from WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        pci_address = Target_Base_Addr_R[target_mem_image] + 104 ;
        data = 32'hDEAD_BEAF ;
        byte_enables = 4'h0 ;
        expect_length = 1 ;

        fork
        begin
            DO_REF ("MEM_READ ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0] + 2,
                        `BC_MEM_READ_LN, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 1 ) ;

            wb_transaction_progress_monitor(pci_address, 1'b0, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;

            do_pause(3) ;

            DO_REF ("MEM_READ ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0] + 2,
                        `BC_MEM_READ_LN, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        3, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event13 ;
        end
        begin:monitor_error_event13
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target read burst from WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        pci_address = Target_Base_Addr_R[target_mem_image] + 108 ;
        data = 32'hAAAA_AAAA ;
        byte_enables = 4'h0 ;
        expect_length = 1 ;

        fork
        begin
            DO_REF ("MEM_READ ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0] + 3,
                        `BC_MEM_READ_MUL, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        2, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 1 ) ;

            wb_transaction_progress_monitor(pci_address, 1'b0, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;

            do_pause(3) ;

            DO_REF ("MEM_READ ", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0] + 3,
                        `BC_MEM_READ_MUL, data[PCI_BUS_DATA_RANGE:0],
                        byte_enables,
                        3, `Test_No_Addr_Perr, `Test_No_Data_Perr,
                        8'h0_0, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_No_Fast_B2B,
                        `Test_Target_Retry_Before, `Test_Expect_No_Master_Abort);
            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event14 ;
        end
        begin:monitor_error_event14
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target read burst from WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        // disable the image
        test_name = "DISABLING MEMORY IMAGE" ;
        config_write( am_offset, Target_Addr_Mask_R[target_mem_image] & 32'h7FFF_FFFF, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write P_AM%d register! Time %t ",1 ,$time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main ;
        end
    end
//*
    if ( target_io_image !== -1 )
    begin
        do_io_disconnects = 1 ;

        if (target_io_image === 1)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL1_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA1_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM1_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA1_ADDR, 2'b00} ;
        end
        else if (target_io_image === 2)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL2_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA2_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM2_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA2_ADDR, 2'b00} ;
        end
        else if (target_io_image === 3)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL3_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA3_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM3_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA3_ADDR, 2'b00} ;
        end
        else if (target_io_image === 4)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL4_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA4_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM4_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA4_ADDR, 2'b00} ;
        end
        else if (target_io_image === 5)
        begin
            ctrl_offset = {4'h1, `P_IMG_CTRL5_ADDR, 2'b00} ;
            ba_offset   = {4'h1, `P_BA5_ADDR, 2'b00} ;
            am_offset   = {4'h1, `P_AM5_ADDR, 2'b00} ;
            ta_offset   = {4'h1, `P_TA5_ADDR, 2'b00} ;
        end
    end
    else
        do_io_disconnects = 0 ;

    if ( do_io_disconnects )
    begin
        test_name = "CONFIGURE TARGET FOR DISCONNECT TESTING" ;
        config_write( ba_offset, Target_Base_Addr_R[target_io_image] | 32'h0000_0001, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write P_BA%d register! Time %t ", 1 ,$time);
            test_fail("PCI Base Address register could not be written") ;
            disable main ;
        end

        // Set Address Mask of IMAGE
        config_write( am_offset, Target_Addr_Mask_R[target_io_image], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write P_AM%d register! Time %t ",1 ,$time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main ;
        end

        // Set Translation Address of IMAGE
        config_write( ta_offset, Target_Tran_Addr_R[target_io_image], 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write P_TA%d register! Time %t ",1 ,$time);
            test_fail("PCI Translation Address Register could not be written") ;
            disable main ;
        end

        config_write( ctrl_offset, 32'h0000_0002, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write P_IMG_CTRL%d register! Time %t ",1 ,$time);
            test_fail("PCI Image Control register could not be written") ;
            disable main ;
        end

        config_write( cls_offset, 32'h0000_04_04, 4'b0011, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write Cache Line Size register! Time %t ", $time);
            test_fail("Cache Line Size register could not be written") ;
            disable main ;
        end

        test_name = "TARGET DISCONNECT ON BURST WRITE TO IO SPACE" ;
        pci_address = Target_Base_Addr_R[target_io_image] + 200 ;
        data = 32'h5555_5555 ;
        byte_enables = 4'h0 ;
        expect_length = 1 ;

        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

        fork
        begin
            PCIU_IO_WRITE
            (
                `Test_Master_1,             // which master
                pci_address,                // to what address
                data,                       // data
                byte_enables,               // byte enable
                expect_length + 1,          // length to request
                `Test_Target_Retry_On       // expected target termination
            ) ;

            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event15 ;
        end
        begin:monitor_error_event15
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target IO burst write") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_progress_monitor(pci_address, 1'b1, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        data = 32'hAAAA_AAAA ;
        fork
        begin
            PCIU_IO_WRITE
            (
                `Test_Master_1,             // which master
                pci_address,                // to what address
                data,                       // data
                byte_enables,               // byte enable
                expect_length + 2,          // length to request
                `Test_Target_Retry_Before   // expected target termination
            ) ;

            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event16 ;
        end
        begin:monitor_error_event16
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target IO burst write") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        begin
            wb_transaction_progress_monitor(pci_address, 1'b1, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        master2_check_received_data = 1 ;

        test_name = "TARGET DISCONNECT ON BURST READ TO IO SPACE" ;

        fork
        begin

             PCIU_IO_READ
             (
                `Test_Master_2,
                pci_address[PCI_BUS_DATA_RANGE:0],
                data,
                byte_enables,
                2,
                `Test_Target_Retry_Before
             );

            do_pause( 1 ) ;

            wb_transaction_progress_monitor(pci_address, 1'b0, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;

            do_pause(3) ;

            PCIU_IO_READ
             (
                `Test_Master_2,
                pci_address[PCI_BUS_DATA_RANGE:0],
                data,
                byte_enables,
                expect_length + 1,
                `Test_Target_Retry_On
             );

            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event17 ;
        end
        begin:monitor_error_event17
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target read burst from WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        fork
        begin

             PCIU_IO_READ
             (
                `Test_Master_2,
                pci_address[PCI_BUS_DATA_RANGE:0],
                data,
                byte_enables,
                2,
                `Test_Target_Retry_Before
             );

            do_pause( 1 ) ;

            wb_transaction_progress_monitor(pci_address, 1'b0, expect_length, 1'b1, wb_ok) ;
            if ( wb_ok !== 1 )
                test_fail("WISHBONE Master started invalid transaction or none at all on WISHBONE bus") ;

            do_pause(3) ;

            PCIU_IO_READ
             (
                `Test_Master_2,
                pci_address[PCI_BUS_DATA_RANGE:0],
                data,
                byte_enables,
                expect_length + 2,
                `Test_Target_Retry_Before
             );

            do_pause( 3 ) ;

            while ( FRAME !== 1 || IRDY !== 1 )
                @(posedge pci_clock) ;

            #1 ;
            if ( !error_monitor_done )
                disable monitor_error_event18 ;
        end
        begin:monitor_error_event18
            error_monitor_done = 0 ;
            pci_ok = 1 ;
            @(error_event_int) ;
            test_fail("PCI Master or Monitor signaled an error while testing disconnect on Target read burst from WISHBONE") ;
            pci_ok = 0 ;
            error_monitor_done = 1 ;
        end
        join

        if ( wb_ok && pci_ok )
            test_ok ;

        test_name = "DISABLING IO IMAGE" ;
        config_write( am_offset, Target_Addr_Mask_R[target_io_image] & 32'h7FFF_FFFF, 4'hF, ok ) ;
        if ( ok !== 1 )
        begin
            $display("Target Disconnect testing failed! Failed to write P_AM%d register! Time %t ",1 ,$time);
            test_fail("PCI Address Mask register could not be written") ;
            disable main ;
        end
    end
//*/
end
endtask // target_disconnects

task target_unsupported_cmds ;
	input [31:0] Address;
	input [2:0]  image_num ;
    reg          ok ;
begin:main
	// PCI IACK behavioral Target must NOT respond!!!
    irq_respond = 0 ;

    $display("  ") ;
    $display("  Master abort testing with unsuported bus command to image %d (BC is IACK)!", image_num) ;
    test_name = "MASTER ABORT WHEN ACCESSING TARGET WITH UNSUPPORTED BUS COMMAND - IACK" ;
    ipci_unsupported_commands_master.master_reference
    (
        Address,      		// first part of address in dual address cycle
        Address,      		// second part of address in dual address cycle
        `BC_IACK,  			// dual address cycle command
        `BC_IACK,      		// normal command
        4'h0,               // byte enables
        32'h1234_5678,      // data
        1'b0,               // make address parity error on first phase of dual address
        1'b0,               // make address parity error on second phase of dual address
        ok                  // result of operation
    ) ;
    if ( ok )
        test_ok ;
    else
    begin
        $display("* Master abort testing failed, PCI Target responded on unsuported bus command!	Time %t ", $time) ;
        test_fail("PCI Target shouldn't responded on unsuported bus command IACK") ;
    end

    $display("  Master abort testing with unsuported bus command to image %d (BC is SPECIAL)!", image_num) ;
    test_name = "MASTER ABORT WHEN ACCESSING TARGET WITH UNSUPPORTED BUS COMMAND - SPECIAL" ;
    ipci_unsupported_commands_master.master_reference
    (
        Address,      		// first part of address in dual address cycle
        Address,      		// second part of address in dual address cycle
        `BC_SPECIAL,  		// dual address cycle command
        `BC_SPECIAL,      	// normal command
        4'h0,               // byte enables
        32'h1234_5678,      // data
        1'b0,               // make address parity error on first phase of dual address
        1'b0,               // make address parity error on second phase of dual address
        ok                  // result of operation
    ) ;
    if ( ok )
        test_ok ;
    else
    begin
        $display("* Master abort testing failed, PCI Target responded on unsuported bus command!	Time %t ", $time) ;
        test_fail("PCI Target shouldn't responded on unsuported bus command SPECIAL") ;
    end

    $display("  Master abort testing with unsuported bus command to image %d (BC is RESERVED0)!", image_num) ;
    test_name = "MASTER ABORT WHEN ACCESSING TARGET WITH UNSUPPORTED BUS COMMAND - RESERVED0" ;
    ipci_unsupported_commands_master.master_reference
    (
        Address,      		// first part of address in dual address cycle
        Address,      		// second part of address in dual address cycle
        `BC_RESERVED0,  	// dual address cycle command
        `BC_RESERVED0,      // normal command
        4'h0,               // byte enables
        32'h1234_5678,      // data
        1'b0,               // make address parity error on first phase of dual address
        1'b0,               // make address parity error on second phase of dual address
        ok                  // result of operation
    ) ;
    if ( ok )
        test_ok ;
    else
    begin
        $display("* Master abort testing failed, PCI Target responded on unsuported bus command!	Time %t ", $time) ;
        test_fail("PCI Target shouldn't responded on unsuported bus command RESERVED0") ;
    end

    $display("  Master abort testing with unsuported bus command to image %d (BC is RESERVED1)", image_num) ;
    test_name = "MASTER ABORT WHEN ACCESSING TARGET WITH UNSUPPORTED BUS COMMAND - RESERVED1" ;
    ipci_unsupported_commands_master.master_reference
    (
        Address,      		// first part of address in dual address cycle
        Address,      		// second part of address in dual address cycle
        `BC_RESERVED1,  	// dual address cycle command
        `BC_RESERVED1,      // normal command
        4'h0,               // byte enables
        32'h1234_5678,      // data
        1'b0,               // make address parity error on first phase of dual address
        1'b0,               // make address parity error on second phase of dual address
        ok                  // result of operation
    ) ;
    if ( ok )
        test_ok ;
    else
    begin
        $display("* Master abort testing failed, PCI Target responded on unsuported bus command!	Time %t ", $time) ;
        test_fail("PCI Target shouldn't responded on unsuported bus command RESERVED1") ;
    end

    $display("  Master abort testing with unsuported bus command to image %d (BC is RESERVED2)!", image_num) ;
    test_name = "MASTER ABORT WHEN ACCESSING TARGET WITH UNSUPPORTED BUS COMMAND - RESERVED2" ;
    ipci_unsupported_commands_master.master_reference
    (
        Address,      		// first part of address in dual address cycle
        Address,      		// second part of address in dual address cycle
        `BC_RESERVED2,  	// dual address cycle command
        `BC_RESERVED2,      // normal command
        4'h0,               // byte enables
        32'h1234_5678,      // data
        1'b0,               // make address parity error on first phase of dual address
        1'b0,               // make address parity error on second phase of dual address
        ok                  // result of operation
    ) ;
    if ( ok )
        test_ok ;
    else
    begin
        $display("* Master abort testing failed, PCI Target responded on unsuported bus command!	Time %t ", $time) ;
        test_fail("PCI Target shouldn't responded on unsuported bus command RESERVED2") ;
    end

    $display("  Master abort testing with unsuported bus command to image %d (BC is RESERVED3)!", image_num) ;
    test_name = "MASTER ABORT WHEN ACCESSING TARGET WITH UNSUPPORTED BUS COMMAND - RESERVED3" ;
    ipci_unsupported_commands_master.master_reference
    (
        Address,      		// first part of address in dual address cycle
        Address,      		// second part of address in dual address cycle
        `BC_RESERVED3,  	// dual address cycle command
        `BC_RESERVED3,      // normal command
        4'h0,               // byte enables
        32'h1234_5678,      // data
        1'b0,               // make address parity error on first phase of dual address
        1'b0,               // make address parity error on second phase of dual address
        ok                  // result of operation
    ) ;
    if ( ok )
        test_ok ;
    else
    begin
        $display("* Master abort testing failed, PCI Target responded on unsuported bus command!	Time %t ", $time) ;
        test_fail("PCI Target shouldn't responded on unsuported bus command RESERVED3") ;
    end

    $display("PCI monitor will complain twice with 'CBE Bus Changed'; second bus command (MEM_WRITE) and ") ;
    $display("    byte enables are different than first bus command (DUAL_ADDR_CYC)!") ;
    $display("  Master abort testing with unsuported bus command to image %d (BC is DUAL_ADDR_CYC)!", image_num) ;
    test_name = "MASTER ABORT WHEN ACCESSING TARGET WITH UNSUPPORTED BUS COMMAND - DUAL_ADDR_CYC" ;
    ipci_unsupported_commands_master.master_reference
    (
        Address,      		// first part of address in dual address cycle
        Address,      		// second part of address in dual address cycle
        `BC_DUAL_ADDR_CYC,  // dual address cycle command
        `BC_MEM_WRITE,      // normal command
        4'h0,               // byte enables;
        32'h1234_5678,      // data
        1'b0,               // make address parity error on first phase of dual address
        1'b0,               // make address parity error on second phase of dual address
        ok                  // result of operation
    ) ;
    if ( ok )
        test_ok ;
    else
    begin
        $display("* Master abort testing failed, PCI Target responded on unsuported bus command!	Time %t ", $time) ;
        test_fail("PCI Target shouldn't responded on unsuported bus command DUAL_ADDR_CYC") ;
    end

    irq_respond = 1 ;

end
endtask // target_unsupported_cmds

task target_completion_expiration ;
    reg   [11:0] pci_ctrl_offset ;
    reg   [11:0] pci_ba_offset ;
    reg   [11:0] pci_am_offset ;
    reg   [11:0] pci_device_ctrl_offset ;
    reg   [11:0] pci_err_cs_offset ;
    reg   [11:0] icr_offset ;
    reg   [11:0] isr_offset ;
    reg   [11:0] lat_tim_cls_offset ;

    reg [31:0] temp_val1 ;
    reg [31:0] temp_val2 ;
    reg        ok   ;
    reg        ok_wb ;
    reg        ok_pci ;

    reg [31:0] pci_image_base ;
    integer i ;
    integer clocks_after_completion ;
    reg     error_monitor_done ;
    reg     test_mem ;

begin:main
    pci_ctrl_offset        = {4'h1, `P_IMG_CTRL1_ADDR, 2'b00} ;
    pci_ba_offset          = {4'h1, `P_BA1_ADDR, 2'b00} ;
    pci_am_offset          = {4'h1, `P_AM1_ADDR, 2'b00} ;
    pci_err_cs_offset      = {4'h1, `P_ERR_CS_ADDR, 2'b00} ;

    icr_offset         = {4'h1, `ICR_ADDR, 2'b00} ;
    isr_offset         = {4'h1, `ISR_ADDR, 2'b00} ;
    lat_tim_cls_offset = 12'hC ;
    pci_device_ctrl_offset    = 12'h4 ;

    `ifdef HOST
        test_mem = 1'b1 ;
        pci_image_base = Target_Base_Addr_R[1] & 32'hFFFF_FFFE ;
    `else
        test_mem = !`PCI_BA1_MEM_IO ;
        pci_image_base = Target_Base_Addr_R[1] ;
    `endif

    // enable master & target operation
    test_name = "BRIDGE CONFIGURATION FOR DELAYED COMPLETION EXPIRATION TEST" ;
    config_write( pci_device_ctrl_offset, 32'h0000_0147, 4'h3, ok) ;
    if ( ok !== 1 )
    begin
        $display("Target completion expiration testing failed! Failed to write PCI Device Control register! Time %t ", $time) ;
        test_fail("write to PCI Device Control register failed") ;
        disable main ;
    end

    // prepare image control register
    config_write( pci_ctrl_offset, 32'h0000_0002, 4'hF, ok) ;
    if ( ok !== 1 )
    begin
        $display("Target completion expiration testing failed! Failed to write P_IMG_CTRL1 register! Time %t ", $time) ;
        test_fail("write to PCI Image Control register failed") ;
        disable main ;
    end

    // prepare base address register
    config_write( pci_ba_offset, pci_image_base, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Target completion expiration testing failed! Failed to write P_BA1 register! Time %t ", $time) ;
        test_fail("write to PCI Base Address register failed") ;
        disable main ;
    end

    // write address mask register
    config_write( pci_am_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Target completion expiration testing failed! Failed to write P_AM1 register! Time %t ", $time) ;
        test_fail("write to PCI Address Mask register failed") ;
        disable main ;
    end

    // enable all status and error reporting features - this tests should proceede normaly and cause no statuses to be set
    config_write( pci_err_cs_offset, 32'hFFFF_FFFF, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Target completion expiration testing failed! Failed to write P_ERR_CS register! Time %t ", $time) ;
        test_fail("write to PCI Error Control and Status register failed") ;
        disable main ;
    end

    config_write( icr_offset, 32'h0000_0000, 4'hF, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Target completion expiration testing failed! Failed to write IC register! Time %t ", $time) ;
        test_fail("write to Interrupt Control register failed") ;
        disable main ;
    end

    // set latency timer and cache line size to 4 - this way even the smallest fifo sizes can be tested
    config_write( lat_tim_cls_offset, 32'hFFFF_0404, 4'h3, ok ) ;
    if ( ok !== 1 )
    begin
        $display("Target completion expiration testing failed! Failed to write latency timer and cache line size values! Time %t ", $time) ;
        test_fail("write to Latency Timer and Cache Line Size registers failed") ;
        disable main ;
    end

    pci_image_base[(31-`PCI_NUM_OF_DEC_ADDR_LINES):0] = 0 ;

    wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);
    test_name = "FLUSH OF DELAYED READ UNCOMPLETED IN 2^^16 CYCLES FROM PCI TARGET UNIT" ;
    master1_check_received_data = 0 ;

    ok_pci = 1 ;
    // start a delayed read request
    fork
    begin
        if ( test_mem )
        
            PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                          pci_image_base, 32'h1234_5678,
                          1, 8'h7_0, `Test_One_Zero_Target_WS,
                          `Test_Devsel_Medium, `Test_Target_Retry_On);
        else
            PCIU_IO_READ( `Test_Master_1, pci_image_base, 32'h1234_5678, 4'h0, 1, `Test_Target_Retry_On ) ;
  
        do_pause( 1 ) ;
    end
    begin:error_monitor1
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
        ok_pci = 0 ; 
        error_monitor_done = 1 ;
    end
    begin
        if ( test_mem )
            wb_transaction_progress_monitor( pci_image_base, 1'b0, 4, 1'b1, ok_wb ) ;
        else
            wb_transaction_progress_monitor( pci_image_base, 1'b0, 1, 1'b1, ok_wb ) ;

        if ( ok_wb !== 1 )
        begin
            test_fail("Bridge failed to process Target Memory read correctly") ;
            disable main ;
        end 
        
        #1 ;
        if ( !error_monitor_done )
            disable error_monitor1 ;
    end
    join

    clocks_after_completion = 0 ;
    // now do another - different transaction
    fork
    begin
        if ( test_mem )
            PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                          pci_image_base + 4, 32'h1234_5678,
                          1, 8'h7_0, `Test_One_Zero_Target_WS,
                         `Test_Devsel_Medium, `Test_Target_Retry_On);
        else
            PCIU_IO_READ( `Test_Master_1, pci_image_base + 4, 32'h1234_5678, 4'h0, 1, `Test_Target_Retry_On ) ;

        while ( clocks_after_completion < 32'h0000_FFF0 )
        begin
            @(posedge pci_clock) ;
            clocks_after_completion = clocks_after_completion + 1 ;
        end

        do_pause('hFF) ;

        if ( test_mem )
            PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                          pci_image_base + 4, 32'h1234_5678,
                          1, 8'h7_0, `Test_One_Zero_Target_WS,
                         `Test_Devsel_Medium, `Test_Target_Retry_On);
        else
            PCIU_IO_READ( `Test_Master_1, pci_image_base + 4, 32'h1234_5678, 4'h0, 1, `Test_Target_Retry_On ) ;

        do_pause( 1 ) ;
    end
    begin:error_monitor2
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
        ok_pci = 0 ;
        error_monitor_done = 1 ;
    end
    begin
        wait ( clocks_after_completion === 32'h0000_FFF0 ) ;
        repeat( 'hFF )
            @(posedge pci_clock) ;

        if ( test_mem )
            wb_transaction_progress_monitor( pci_image_base + 4, 1'b0, 4, 1'b1, ok_wb ) ;
        else
            wb_transaction_progress_monitor( pci_image_base + 4, 1'b0, 1, 1'b1, ok_wb ) ;

        if ( ok_wb !== 1 )
        begin
            test_fail("Bridge failed to process Target Memory read correctly") ;
            disable main ;
        end

        repeat(4)
            @(posedge pci_clock) ;

        fork
        begin
            if ( test_mem )
                PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                              pci_image_base + 4, 32'h1234_5678,
                              4, 8'h7_0, `Test_One_Zero_Target_WS,
                             `Test_Devsel_Medium, `Test_Target_Normal_Completion);
            else
                PCIU_IO_READ( `Test_Master_1, pci_image_base + 4, 32'h1234_5678, 4'h0, 1, `Test_Target_Normal_Completion ) ;

            do_pause(1) ;
        end
        begin
           pci_transaction_progress_monitor( pci_image_base + 4, test_mem ? `BC_MEM_READ : `BC_IO_READ, test_mem ? 4 : 1, 0, 1'b1, 1'b0, 0, ok ) ; 
           #1 ;
           if ( !error_monitor_done )
               disable error_monitor2 ;
        end
        join
    end
    join

    if ( ok && ok_pci && ok_wb )
        test_ok ;

    if ( ok !== 1 )
    begin
        $display("Target completion expiration testing failed! Failed to write P_AM1 register! Time %t ", $time) ;
        test_fail("write to PCI Address Mask register failed") ;
        disable main ;
    end

    // check statuses after this situation - none should be set
    test_name = "PCI DEVICE ERROR STATUS BITS' STATES AFTER COMPLETION EXPIRED IN PCI TARGET UNIT" ;
    config_read( pci_device_ctrl_offset, 4'hC, temp_val1 ) ;
    if ( ok !== 1 )
    begin
        $display("Target completion expiration testing failed! Failed to read pci device status register! Time %t ", $time) ;
        test_fail("read from pci device status register failed") ;
        disable main ;
    end

    if ( temp_val1[31] )
    begin
        $display("Target completion expiration testing failed! Detected parity error bit set for no reason! Time %t ", $time) ;
        test_fail("detected parity error bit was set for no reason") ;
    end

    if ( temp_val1[30] )
    begin
        $display("Target completion expiration testing failed! Signaled system error bit set for no reason! Time %t ", $time) ;
        test_fail("signaled system error bit was set for no reason") ;
    end

    if ( temp_val1[29] )
    begin
        $display("Target completion expiration testing failed! Received master abort bit set for no reason! Time %t ", $time) ;
        test_fail("received master abort bit was set for no reason") ;
    end

    if ( temp_val1[28] )
    begin
        $display("Target completion expiration testing failed! Received Target abort bit set for no reason! Time %t ", $time) ;
        test_fail("received target abort bit was set for no reason") ;
    end

    if ( temp_val1[27] )
    begin
        $display("Target completion expiration testing failed! Signaled Target abort bit set for no reason! Time %t ", $time) ;
        test_fail("signaled target abort bit was set for no reason") ;
    end

    if ( temp_val1[24] )
    begin
        $display("Target completion expiration testing failed! Master Data parity error bit set for no reason! Time %t ", $time) ;
        test_fail("Master Data parity error bit was set for no reason") ;
    end

    test_name = "PCI TARGET UNIT ERROR REPORTING REGISTER VALUE AFTER COMPLETION EXPIRED" ;
    config_read( pci_err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 0 )
    begin
        $display("Target completion expiration testing failed! Error status bit in PCI error reporting register set for no reason! Time %t ", $time) ;
        test_fail("Error status bit in PCI error reporting register was set for no reason") ;
    end
    // test target retry counter expiration
    // set wb slave to retry response
    wishbone_slave.cycle_response(3'b001, tb_subseq_waits, 8'd255);
    test_name = "RETRY COUNTER EXPIRATION DURING WRITE THROUGH PCI TARGET UNIT" ;
    ok_pci = 1 ;

    fork
    begin
        if ( test_mem == 1 )
            PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                        pci_image_base, 32'hDEAD_BEAF, 4'hA,
                        1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
            PCIU_IO_WRITE( `Test_Master_1, pci_image_base, 32'hDEAD_BEAF, 4'hA, 1, `Test_Target_Normal_Completion) ;

        do_pause(1) ;

        // do another write with same address and different data
        if ( test_mem == 1 )
            PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                        pci_image_base, 32'h8765_4321, 4'h0,
                        1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
            PCIU_IO_WRITE( `Test_Master_1, pci_image_base, 32'h8765_4321, 4'h0, 1, `Test_Target_Normal_Completion) ;
 
        do_pause(1) ;
    end
    begin
        for ( i = 0 ; i < `WB_RTY_CNT_MAX ; i = i + 1 )
        begin
            wb_transaction_progress_monitor( pci_image_base, 1'b1, 0, 1'b1, ok_wb ) ;
            if ( ok_wb !== 1 )
            begin
                $display("WISHBONE Master Retry Counter expiration test failed! WB transaction progress monitor detected invalid transaction or none at all on WB bus! Time %t", $time) ;
                test_fail("WB transaction progress monitor detected invalid transaction or none at all on WB bus") ;
                disable main ;
            end
        end

        // set WB slave to normal completion
        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);
        
        wb_transaction_progress_monitor( pci_image_base, 1'b1, 1, 1'b1, ok_wb ) ; 
        if ( ok_wb !== 1 )
        begin
            $display("WISHBONE Master Retry Counter expiration test failed! WB transaction progress monitor detected invalid transaction or none at all on WB bus! Time %t", $time) ;
            test_fail("WB transaction progress monitor detected invalid transaction or none at all on WB bus") ;
            disable main ;
        end
 
        #1 ;
        if ( !error_monitor_done )
            disable error_monitor3 ;
    end
    begin:error_monitor3
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
        ok_pci = 0 ;
        error_monitor_done = 1 ;
    end
    join

    if ( ok_wb && ok_pci )
    begin
        test_ok ;
    end

    test_name = "ERROR STATUS REGISTER VALUE CHECK AFTER RETRY COUNTER EXPIRED" ;
    config_read( pci_err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 1'b1 )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! WB Master should signal an error when retry counter expires during a write! Time %t", $time) ;
        test_fail("error wasn't reported, when retry counter expired during posted write through PCI Target unit") ;
    end    

    if ( temp_val1[9] !== 1 )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! WB Master should set Error Source bit when retry counter expires during a write! Time %t", $time) ;
        test_fail("error source bit wasn't set, when retry counter expired during posted write through PCI Target unit") ;
    end

    if ( temp_val1[10] !== 1 )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! WB Master should set Retry Expired bit when retry counter expires during a write! Time %t", $time) ;
        test_fail("retry expired bit wasn't set, when retry counter expired during posted write through PCI Target unit") ;
    end

    if ( temp_val1[27:24] !== (test_mem ? `BC_MEM_WRITE : `BC_IO_WRITE) )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! Invalid value in bus command field in error control and status register when retry counter expired during a write! Time %t", $time) ;
        test_fail("bus command field in error control and status register was wrong when retry counter expired during posted write through PCI Target unit") ;
    end
    
    if ( temp_val1[31:28] !== 4'hA )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! Invalid value in byte enable field in error control and status register when retry counter expired during a write! Time %t", $time) ;
        test_fail("byte enable field in error control and status register was wrong when retry counter expired during posted write through PCI Target unit") ;
    end

    // clear error status register
    config_write( pci_err_cs_offset, temp_val1, 4'h2, ok ) ;
    
    test_name = "ERROR ADDRESS REGISTER VALUE CHECK AFTER RETRY COUNTER EXPIRED" ;
    config_read( pci_err_cs_offset + 4, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== pci_image_base )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! Invalid value in error address register when retry counter expired during a write! Time %t", $time) ;
        test_fail("value in error address register was wrong when retry counter expired during posted write through PCI Target unit") ;
    end

    test_name = "ERROR DATA REGISTER VALUE CHECK AFTER RETRY COUNTER EXPIRED" ;
    config_read( pci_err_cs_offset + 8, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== 32'hDEAD_BEAF )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! Invalid value in error data register when retry counter expired during a write! Time %t", $time) ;
        test_fail("value in error data register was wrong when retry counter expired during posted write through PCI Target unit") ;
    end

    test_name = "RETRY COUNTER EXPIRATION DURING READ THROUGH PCI TARGET UNIT" ;
    ok_pci = 1 ;
    wishbone_slave.cycle_response(3'b001, tb_subseq_waits, 8'd255);
    
    i = 0 ;
    fork
    begin
        if ( test_mem )
            PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                          pci_image_base + 4, 32'h1234_5678,
                          2, 8'h7_0, `Test_One_Zero_Target_WS,
                         `Test_Devsel_Medium, `Test_Target_Retry_Before);
        else
            PCIU_IO_READ( `Test_Master_1, pci_image_base + 4, 32'h1234_5678, 4'h0, 2, `Test_Target_Retry_Before ) ;
 
        do_pause( 1 ) ;

    end
    begin
        for ( i = 0 ; i < `WB_RTY_CNT_MAX ; i = i + 1 )
        begin
            wb_transaction_progress_monitor( pci_image_base + 4, 1'b0, 0, 1'b1, ok_wb ) ;
            if ( ok_wb !== 1 )
            begin
                $display("WISHBONE Master Retry Counter expiration test failed! WB transaction progress monitor detected invalid transaction or none at all on WB bus! Time %t", $time) ;
                test_fail("WB transaction progress monitor detected invalid transaction or none at all on WB bus") ;
                disable main ;
            end
        end
 
        // set WB slave to normal completion
        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'h0);

        fork
        begin
            repeat(4)
                @(posedge pci_clock) ;
 
            if ( test_mem )
                PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                              pci_image_base, 32'h8765_4321,
                              1, 8'h7_0, `Test_One_Zero_Target_WS,
                             `Test_Devsel_Medium, `Test_Target_Retry_On);
            else
                PCIU_IO_READ( `Test_Master_1, pci_image_base, 32'h8765_4321, 4'h0, 1, `Test_Target_Retry_On ) ;

            do_pause(1) ;
        end
        begin

            wb_transaction_progress_monitor( pci_image_base, 1'b0, test_mem ? 4 : 1, 1'b1, ok_wb ) ;
            if ( ok_wb !== 1 )
            begin
                $display("WISHBONE Master Retry Counter expiration test failed! WB transaction progress monitor detected invalid transaction or none at all on WB bus! Time %t", $time) ;
                test_fail("WB transaction progress monitor detected invalid transaction or none at all on WB bus") ;
                disable main ;
            end
        end
        join

        repeat( 4 )
            @(posedge pci_clock) ;

        if ( test_mem )
            PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                          pci_image_base, 32'h8765_4321,
                          1, 8'h7_0, `Test_One_Zero_Target_WS,
                         `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
            PCIU_IO_READ( `Test_Master_1, pci_image_base, 32'h8765_4321, 4'h0, 1, `Test_Target_Normal_Completion ) ;
 
        do_pause(1) ;

        #1 ;
        if ( !error_monitor_done )
            disable error_monitor4 ;
    end
    begin:error_monitor4
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
        ok_pci = 0 ;
        error_monitor_done = 1 ;
    end
    join

    if ( ok_wb && ok_pci )
        test_ok ;

    test_name = "ERROR STATUS REGISTER VALUE CHECK AFTER RETRY COUNTER EXPIRED DURING READ" ;
    config_read( pci_err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 1'b0 )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! WB Master shouldn't signal an error when retry counter expires during a read! Time %t", $time) ;
        test_fail("error shouldn't be reported, when retry counter expires during read through PCI Target unit") ;
    end
    
    test_name = "NO RESPONSE COUNTER EXPIRATION DURING READ THROUGH PCI TARGET UNIT" ;
    ok_pci = 1 ;
    wishbone_slave.cycle_response(3'b000, tb_subseq_waits, 8'd255);
 
    fork
    begin
        if ( test_mem )
            PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                          pci_image_base + 4, 32'h1234_5678,
                          2, 8'h7_0, `Test_One_Zero_Target_WS,
                         `Test_Devsel_Medium, `Test_Target_Retry_Before);
        else
            PCIU_IO_READ( `Test_Master_1, pci_image_base + 4, 32'h1234_5678, 4'h0, 2, `Test_Target_Retry_Before ) ;
 
        do_pause( 1 ) ;
 
    end
    begin
        wb_transaction_progress_monitor( pci_image_base + 4, 1'b0, 0, 1'b1, ok_wb ) ;
        if ( ok_wb !== 1 )
        begin
            $display("WISHBONE Master Retry Counter expiration test failed! WB transaction progress monitor detected invalid transaction or none at all on WB bus! Time %t", $time) ;
            test_fail("WB transaction progress monitor detected invalid transaction or none at all on WB bus") ;
            disable main ;
        end
 
        repeat(4)
            @(posedge pci_clock) ;
        
        fork
        begin
 
            if ( test_mem )
                PCIU_MEM_READ("MEM_READ  ", `Test_Master_1,
                              pci_image_base + 4, 32'h8765_4321,
                              1, 8'h7_0, `Test_One_Zero_Target_WS,
                             `Test_Devsel_Medium, `Test_Target_Abort_On);
            else
                PCIU_IO_READ( `Test_Master_1, pci_image_base + 4, 32'h8765_4321, 4'h0, 1, `Test_Target_Abort_On ) ;

            do_pause(1) ;
 
        end
        begin
           
            pci_transaction_progress_monitor( pci_image_base + 4, test_mem ? `BC_MEM_READ : `BC_IO_READ, 0, 0, 1'b1, 1'b0, 1'b0, ok ) ;
            if ( ok !== 1 )
            begin
                $display("WISHBONE Master Retry Counter expiration test failed! PCI transaction progress monitor detected invalid transaction or none at all on PCI bus! Time %t", $time) ;
                test_fail("PCI transaction progress monitor detected invalid transaction or none at all on PCI bus") ;
                disable main ;
            end
        end
        join
 
        #1 ;
        if ( !error_monitor_done )
            disable error_monitor5 ;
    end
    begin:error_monitor5
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
        ok_pci = 0 ;
        error_monitor_done = 1 ;
    end
    join
 
    if ( ok_wb && ok_pci )
        test_ok ;

    test_name = "ERROR STATUS REGISTER VALUE CHECK AFTER NO RESPONSE COUNTER EXPIRED DURING READ" ;
    config_read( pci_err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 1'b0 )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! WB Master shouldn't signal an error when retry counter expires during a read! Time %t", $time) ;
        test_fail("error shouldn't be reported, when retry counter expires during read through PCI Target unit") ;
    end 

    test_name = "PCI DEVICE STATUS REGISTER VALUE CHECK AFTER NO RESPONSE COUNTER EXPIRED DURING READ" ;
    config_read( pci_device_ctrl_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[25] !== 1'b1 )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! Signaled Target Abort bit not set when PCI Target terminated with target abort! Time %t", $time) ;
        test_fail("Signaled Target Abort bit was not set when PCI Target terminated with target abort") ;
    end
 
    config_write( pci_device_ctrl_offset, temp_val1, 4'hF, ok ) ;

    test_name = "NO RESPONSE COUNTER EXPIRATION DURING WRITE THROUGH PCI TARGET UNIT" ;
    ok_pci = 1 ;
    wishbone_slave.cycle_response(3'b000, tb_subseq_waits, 8'd255);

    fork
    begin
        if ( test_mem == 1 )
            PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                        pci_image_base, 32'hBEAF_DEAD, 4'h0,
                        1, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
            PCIU_IO_WRITE( `Test_Master_1, pci_image_base, 32'hBEAF_DEAD, 4'h0, 1, `Test_Target_Normal_Completion) ;
 
        do_pause(1) ;
 
        // do another write with same address and different data
        if ( test_mem == 1 )
            PCIU_MEM_WRITE ("MEM_WRITE ", `Test_Master_1,
                        pci_image_base, 32'h8765_6789, 4'h0,
                        3, `Test_One_Zero_Master_WS, `Test_One_Zero_Target_WS,
                        `Test_Devsel_Medium, `Test_Target_Normal_Completion);
        else
            PCIU_IO_WRITE( `Test_Master_1, pci_image_base, 32'h8765_6789, 4'h0, 1, `Test_Target_Normal_Completion) ;

        do_pause(1) ;
        
        $display("PCIU monitor (WB bus) will complain in following section for a few times - no WB response test!") ;
        $fdisplay(pciu_mon_log_file_desc,
        "********************************************  Monitor will complain in following section for a few times - testbench is intentionally causing no response  *********************************************") ;
    end 
    begin
        wb_transaction_progress_monitor( pci_image_base, 1'b1, 0, 1'b1, ok_wb ) ;
        if ( ok_wb !== 1 )
        begin
            $display("WISHBONE Master Retry Counter expiration test failed! WB transaction progress monitor detected invalid transaction or none at all on WB bus! Time %t", $time) ;
            test_fail("WB transaction progress monitor detected invalid transaction or none at all on WB bus") ;
            disable main ;
        end    

        wishbone_slave.cycle_response(3'b100, tb_subseq_waits, 8'd255);
        
        if ( test_mem )
            wb_transaction_progress_monitor( pci_image_base, 1'b1, 3, 1'b1, ok_wb ) ;
        else
            wb_transaction_progress_monitor( pci_image_base, 1'b1, 1, 1'b1, ok_wb ) ;

        if ( ok_wb !== 1 )
        begin
            $display("WISHBONE Master Retry Counter expiration test failed! WB transaction progress monitor detected invalid transaction or none at all on WB bus! Time %t", $time) ;
            test_fail("WB transaction progress monitor detected invalid transaction or none at all on WB bus") ;
            disable main ;
        end

        #1 ;
        if ( !error_monitor_done )
            disable error_monitor6 ;
    end
    begin:error_monitor6
        error_monitor_done = 0 ;
        @(error_event_int) ;
        test_fail("PCI behavioral master or PCI monitor detected error on PCI bus") ;
        ok_pci = 0 ;
        error_monitor_done = 1 ;
    end 
    join

    $display("PCIU monitor (WB bus) should NOT complain any more!") ;
    $fdisplay(pciu_mon_log_file_desc,
    "********************************************  Monitor should NOT complain any more  ********************************************************************************************************************") ;

    test_name = "ERROR STATUS REGISTER VALUE CHECK AFTER NO RESPONSE COUNTER EXPIRED DURING TARGET WRITE" ;
    config_read( pci_err_cs_offset, 4'hF, temp_val1 ) ;
    if ( temp_val1[8] !== 1'b1 )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! WB Master should signal an error when no response counter expires during a write! Time %t", $time) ;
        test_fail("error wasn't reported, when no response counter expired during posted write through PCI Target unit") ;
    end
 
    if ( temp_val1[9] !== 0 )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! WB Master should not set Error Source bit when no response counter expires during a write! Time %t", $time) ;
        test_fail("error source bit was set, when no response counter expired during posted write through PCI Target unit") ;
    end
 
    if ( temp_val1[10] !== 1 )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! WB Master should set Retry Expired bit when no response counter expires during a write! Time %t", $time) ;
        test_fail("retry expired bit wasn't set, when no response counter expired during posted write through PCI Target unit") ;
    end
 
    if ( temp_val1[27:24] !== (test_mem ? `BC_MEM_WRITE : `BC_IO_WRITE) )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! Invalid value in bus command field in error control and status register when no response counter expired during a write! Time %t", $time) ;
        test_fail("bus command field in error control and status register was wrong when no response counter expired during posted write through PCI Target unit") ;
    end
 
    if ( temp_val1[31:28] !== 4'h0 )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! Invalid value in byte enable field in error control and status register when no response counter expired during a write! Time %t", $time) ;
        test_fail("byte enable field in error control and status register was wrong when no response counter expired during posted write through PCI Target unit") ;
    end
 
    // clear error status register
    config_write( pci_err_cs_offset, temp_val1, 4'h2, ok ) ; 

    test_name = "ERROR ADDRESS REGISTER VALUE CHECK AFTER NO RESPONSE COUNTER EXPIRED" ;
    config_read( pci_err_cs_offset + 4, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== pci_image_base )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! Invalid value in error address register when no response counter expired during a write! Time %t", $time) ;
        test_fail("value in error address register was wrong when no response counter expired during posted write through PCI Target unit") ;
    end
 
    test_name = "ERROR DATA REGISTER VALUE CHECK AFTER NO RESPONSE COUNTER EXPIRED" ;
    config_read( pci_err_cs_offset + 8, 4'hF, temp_val1 ) ;
    if ( temp_val1 !== 32'hBEAF_DEAD )
    begin
        $display("WISHBONE Master Retry Counter expiration test failed! Invalid value in error data register when no response counter expired during a write! Time %t", $time) ;
        test_fail("value in error data register was wrong when no response counter expired during posted write through PCI Target unit") ;
    end
    
    // disable current image - write address mask register
    config_write( pci_am_offset, 32'h7FFF_FFFF, 4'hF, ok ) ;
end
endtask // target_completion_expired

task config_write ;
    input [11:0] offset ;
    input [31:0] data ;
    input [3:0]  byte_enable ;
    output       ok ;
    `ifdef HOST
    reg   `WRITE_STIM_TYPE   write_data ;
    reg   `WB_TRANSFER_FLAGS write_flags ;
    reg   `WRITE_RETURN_TYPE write_status ;
    `else
    reg   [PCI_BUS_DATA_RANGE:0] pci_address;
    reg   [PCI_BUS_CBE_RANGE:0]  byte_enables_l; // active LOW
    `endif
    reg in_use ;
    reg [31:0] temp_var ;
begin
    if ( in_use === 1 )
    begin
        $display("config_read task re-entered! Time %t ", $time) ;
        ok = 0 ;
        #20 $stop ;
    end
    else
    begin
        ok = 1 ;
        in_use = 1 ;
    end
    `ifdef HOST
    write_flags                    = 0 ;
    write_flags`INIT_WAITS         = tb_init_waits ;
    write_flags`SUBSEQ_WAITS       = tb_subseq_waits ;
    write_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    temp_var                                     = { `WB_CONFIGURATION_BASE, 12'h000 } ;
    temp_var[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] = 0 ;
    write_data`WRITE_ADDRESS                     = temp_var + offset ;
    write_data`WRITE_SEL                         = byte_enable ;
    write_data`WRITE_TAG_STIM                    = 0 ;
    write_data`WRITE_DATA                        = data ;

    wishbone_master.wb_single_write( write_data, write_flags, write_status ) ;
    if ( write_status`CYC_ACTUAL_TRANSFER !== 1 )
    begin
        $display("Write to configuration space failed! Time %t ", $time) ;
        ok = 0 ;
    end

    @(posedge wb_clock) ;
    // normal thing to do for software would be to disable master and target operation before doing a configuration write
    // here we just wait for two guest cycles for conf space bits to synchronize
    repeat( 2 )
        @(posedge pci_clock) ;

    `else // GUEST
    byte_enables_l = ~byte_enable ;
    pci_address    = Target_Base_Addr_R[0] | { 20'h0, offset } ;

    fork
    begin
        DO_REF ("MEM_W_CONF", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
              PCI_COMMAND_MEMORY_WRITE, data[PCI_BUS_DATA_RANGE:0],
              byte_enables_l[PCI_BUS_CBE_RANGE:0],
              `Test_One_Word, `Test_No_Addr_Perr, `Test_No_Data_Perr,
              8'h0_0, `Test_One_Zero_Target_WS,
              `Test_Devsel_Medium, `Test_No_Fast_B2B,
              `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);
    do_pause( 1 ) ;
    end
    begin
        pci_transaction_progress_monitor( pci_address, `BC_MEM_WRITE, 1, 0, 1'b1, 1'b0, 0, ok ) ;
        @(posedge pci_clock) ;
    end
    join

     repeat( 2 )
         @(posedge wb_clock) ;

    `endif
    in_use = 0 ;
end
endtask // config_write

task config_read ;
    input [11:0] offset ;
    input [3:0]  byte_enable ;
    output [31:0] data ;

    reg `READ_STIM_TYPE    read_data ;
    reg `WB_TRANSFER_FLAGS read_flags ;
    reg `READ_RETURN_TYPE  read_status ;

    reg [31:0] pci_address ;
    reg [3:0] byte_enables_l ;

    reg master_check_data_prev ;
    reg in_use ;
    reg [31:0] temp_var ;
begin:main
    if ( in_use === 1 )
    begin
        $display("config_read task re-entered! Time %t ", $time) ;
        data = 32'hxxxx_xxxx ;
        disable main ;
    end

    in_use = 1 ;

`ifdef HOST
    repeat(4)
        @(posedge pci_clock) ;
    repeat(4)
        @(posedge wb_clock) ;
    read_flags                    = 0 ;
    read_flags`INIT_WAITS         = tb_init_waits ;
    read_flags`SUBSEQ_WAITS       = tb_subseq_waits ;
    read_flags`WB_TRANSFER_AUTO_RTY = 0 ;

    temp_var                                     = { `WB_CONFIGURATION_BASE, 12'h000 } ;
    temp_var[(31 - `WB_NUM_OF_DEC_ADDR_LINES):0] = 0 ;

    read_data`READ_ADDRESS  = temp_var + offset ;
    read_data`READ_SEL      = byte_enable ;
    read_data`READ_TAG_STIM = 0 ;

    wishbone_master.wb_single_read( read_data, read_flags, read_status ) ;
    if (read_status`CYC_ACTUAL_TRANSFER !== 1)
    begin
        $display("Configuration read failed! Bridge failed to process single memory read! Time %t ", $time) ;
        #20 $stop ;
    end
    data = read_status`READ_DATA ;
`else
  `ifdef GUEST
    repeat(4)
        @(posedge wb_clock) ;
    repeat(4)
        @(posedge pci_clock) ;
    master_check_data_prev = master2_check_received_data ;
    master2_check_received_data = 0 ;

    byte_enables_l = ~byte_enable ;
    pci_address    = Target_Base_Addr_R[0] | { 20'h0, offset } ;

    DO_REF ("MEM_R_CONF", `Test_Master_2, pci_address[PCI_BUS_DATA_RANGE:0],
             PCI_COMMAND_MEMORY_READ, data[PCI_BUS_DATA_RANGE:0],
             byte_enables_l[PCI_BUS_CBE_RANGE:0],
             `Test_One_Word, `Test_No_Addr_Perr, `Test_No_Data_Perr,
             8'h4_0, `Test_One_Zero_Target_WS,
             `Test_Devsel_Medium, `Test_No_Fast_B2B,
             `Test_Target_Normal_Completion, `Test_Expect_No_Master_Abort);
    do_pause( 1 ) ;

    @(master2_received_data_valid) ;
    data = master2_received_data ;

    master2_check_received_data = master_check_data_prev ;
  `endif
`endif
    in_use = 0 ;
end
endtask //config_read

task test_fail ;
    input [7999:0] failure_reason ;
    reg   [8007:0] display_failure ;
    reg   [799:0] display_test ;
begin
    tests_failed = tests_failed + 1 ;

    display_failure = {failure_reason, "!"} ;
    while ( display_failure[7999:7992] == 0 )
        display_failure = display_failure << 8 ;

    display_test = test_name ;
    while ( display_test[799:792] == 0 )
       display_test = display_test << 8 ;

    $fdisplay( tb_log_file, "*****************************************************************************************" ) ;
    $fdisplay( tb_log_file, " At time %t ", $time ) ;
    $fdisplay( tb_log_file, " Test %s", display_test ) ;
    $fdisplay( tb_log_file, " *FAILED* because") ;
    $fdisplay( tb_log_file, " %s", display_failure ) ;
    $fdisplay( tb_log_file, "*****************************************************************************************" ) ;
    $fdisplay( tb_log_file, " " ) ;

    `ifdef STOP_ON_FAILURE
    #20 $stop ;
    `endif
end
endtask // test_fail

task test_ok ;
    reg [799:0] display_test ;
begin
   tests_successfull = tests_successfull + 1 ;

   display_test = test_name ;
   while ( display_test[799:792] == 0 )
       display_test = display_test << 8 ;

   $fdisplay( tb_log_file, "*****************************************************************************************" ) ;
   $fdisplay( tb_log_file, " At time %t ", $time ) ;
   $fdisplay( tb_log_file, " Test %s", display_test ) ;
   $fdisplay( tb_log_file, " reported *SUCCESSFULL*! ") ;
   $fdisplay( tb_log_file, "*****************************************************************************************" ) ;
   $fdisplay( tb_log_file, " " ) ;
end
endtask // test_ok

task test_summary;
begin
    $fdisplay(tb_log_file, "******************************* PCI Testcase summary *******************************") ;
    $fdisplay(tb_log_file, "Tests performed:   %d", tests_successfull + tests_failed) ;
    $fdisplay(tb_log_file, "Failed tests   :   %d", tests_failed) ;
    $fdisplay(tb_log_file, "Successfull tests: %d", tests_successfull) ;
    $fdisplay(tb_log_file, "******************************* PCI Testcase summary *******************************") ;
    $fclose(tb_log_file) ;
end
endtask

endmodule
