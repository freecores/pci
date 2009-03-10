`include "pci_stat.v"

module pci_master32_sm
(
    // system inputs
    clk_in,
    reset_in,
    // arbitration
    pci_req_out,
    pci_req_en_out,
    pci_gnt_in,
    // master in/outs
    pci_frame_in,
    pci_frame_out,
    pci_frame_en_out,
    pci_irdy_in,
    pci_irdy_out,
    pci_irdy_en_out,
    // target response inputs
    pci_trdy_in,
    pci_stop_in,
    pci_devsel_in,
    // address, data, bus command, byte enable in/outs
    pci_ad_in,
    pci_ad_out,
    pci_ad_en_out,
    pci_cbe_out,
    pci_cbe_en_out,

    // other side of state machine
    address_i,
    bc_i,
    data_i,
    data_o,
    be_i,
    req_i,
    rdy_i,
    status_o,
    last_i,
    latency_tim_val_i
) ;

// system inputs
input   clk_in,
        reset_in ;

/*==================================================================================================================
PCI interface signals - bidirectional signals are divided to inputs and outputs in I/O cells instantiation
module. Enables are separate signals.
==================================================================================================================*/
// arbitration
output  pci_req_out,
        pci_req_en_out ;

input   pci_gnt_in ;

// master in/outs
input    pci_frame_in ;

output   pci_frame_out,
         pci_frame_en_out ;

input    pci_irdy_in ;
output   pci_irdy_out,
         pci_irdy_en_out,
    
// target response inputs
input   pci_trdy_in,
        pci_stop_in,
        pci_devsel_in ;
    
// address, data, bus command, byte enable in/outs
input   [31:0]  pci_ad_in ;
output  [31:0]  pci_ad_out ;
output          pci_ad_en_out ;
output  [3:0]   pci_cbe_out ;
output          pci_cbe_en_out ;

/*==================================================================================================================
Other side of master state machine - the one that issues requests.
==================================================================================================================*/
/*==================================================================================================================
address input - address always qualifies current data present at data_i input or data that needs to be read from
                data_o output.
==================================================================================================================*/
input   [31:0]  address_i ; // current request address input

/*==================================================================================================================
bus command input - bus command is always valid vhen request is asserted and identifies what state machine should
                    do.
==================================================================================================================*/
input   [3:0]   bc_i ;      // current request bus command input

/*==================================================================================================================
data input - for PCI write bus commands - this bus provides valid write data during write commands when rdy_i is
             asserted. If rdy_i is deasserted then state machine inserts wait states ( if between burst )
==================================================================================================================*/
input   [31:0]  data_i ;    // current dataphase data input

/*==================================================================================================================
data output - for PCI read bus commands - data on this bus is provided from master state machine and is qualified
              with `TRANSFERED status. When `TRANSFERED status is present on status output, backend is responsible
              for storing this data. Backend signals if it is ready to accept data with rdy_i input.
==================================================================================================================*/
output  [31:0]  data_o ;    // for read operations - current request data output

/*==================================================================================================================
byte enable input - active low byte enables for driving BE# pins on pci during transfers - they are valid always
                    when request is asserted by the backend
==================================================================================================================*/
input   [3:0]   be_i ;      // current dataphase byte enable inputs

/*==================================================================================================================
request and ready inputs from backend - req_i input means that backend has some transaction request for PCI bus.
During write requests it must provide address, bus command, byte enables and data. Write data is qualified with
rdy_i signal input. State machine signals status back to the backend. Statuses are described later.
During read requests backend must provide address, bus command and byte enables to the state machine. When it is ready
to receive data, it asserts rdy_i. Status of current request is signaled to back end all the time.
==================================================================================================================*/
input           req_i ;     // initiator cycle is requested
input           rdy_i ;     // requestor indicates that data is ready to be sent for write transaction and ready to
                            // be received on read transaction
/*==================================================================================================================
Last input - this input in conjuction with req_i and rdy_i qualifies last data beat in current request.
             for single transfers all three signals are asserted in parallel, for burst transfers last_i is inactive
             until last intended data is present at data_i or requested from data_o. After last is sampled asserted,
             state machine must go to idle state and monitor for new requests. 
==================================================================================================================*/
input           last_i ;    // last dataphase in current transaction indicator

/*==================================================================================================================
Status output - Status is always provided to the backend depending on PCI transaction progress. Status descriptions:
`WAIT - default status - means that current request is not processed in any way yet - this status is propagated
        to backend when Master state machine is requesting for the bus, latency timer has expired on current phase,
        target is inserting wait states or rdy_i input is down.
`TRANSFERED - status means, that current data phase is completing succesfully on PCI bus. During writes, this status
              signals to the backend that it should provide new data imediately after rising clock edge or deassert
              rdy_i if it is not ready to do so. During reads, this status signals to the backend that on the rising
              edge of clock data must be latched from data_o bus. After this, backend must deassert rdy_i if it is not
              ready to accept more data. 
`DISCONNECT_WO_DATA - Disconnect Without Data can be treated by the backend the same as `WAIT status. Separate status 
                      is provided only becuse some backends might use it for different purposes.
`DISCONNECT_W_DATA  - Dissconnect With Data can be treated the same as `TRANSFERED status. Separate status is 
                      provided because some backends might use it for different purposes
`TABORT - Target Abort status - current transfer resulted in target abort termination. Backend must not repeat this
                                transaction anymore. State machine is not responsible for error handling
`MABORT - State Machine signals this status to backend when no Target claims the transaction. Backend must decide if
          this is an error or not. State machine isn't responsible for error handling
`RETRY  - State Machine signals this status to backend when Target Retry occurs on current dataphase. State Machine
          is responsible for releasing a bus, by PCI protocol, while transaction repeating is up to backend.
==================================================================================================================*/
output  [3:0]   status_o ;  // current dataphase completion status

// latency timer value input - state machine starts latency timer whenever it starts a transaction and last is not
// asserted ( meaning burst transfer ). 
input [7:0] latency_tim_val_i ;

endmodule