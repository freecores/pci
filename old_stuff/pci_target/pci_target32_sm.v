//==========================//
// PCI Target state machine //
//==========================//

`include "pci_stat.v"

module pci_target32_sm
(
    // system inputs
    clk_in,
    reset_in,
    // master inputs
    pci_frame_in,
    pci_irdy_in,
    // target response outputs
    pci_trdy_out,
    pci_stop_out,
    pci_devsel_out,
    // address, data, bus command, byte enable in/outs
    pci_ad_in,
    pci_ad_out,
    pci_ad_en_out,
    pci_cbe_in,

    // other side of state machine
    address_o,
    addr_claim_i,
    bc_o,
    data_o,
    data_i,
    be_o,
    req_o,
    rdy_o,
    status_i,
    last_o
) ;

// system inputs
input   clk_in,
        reset_in ;

/*==================================================================================================================
PCI interface signals - bidirectional signals are divided to inputs and outputs in I/O cells instantiation
module. Enables are separate signals.
==================================================================================================================*/
// master inputs
input    pci_frame_in ;
input    pci_irdy_in ;
    
// target response outputs
output  pci_trdy_out,
        pci_stop_out,
        pci_devsel_out ;
    
// address, data, bus command, byte enable in/outs
input   [31:0]  pci_ad_in ;
output  [31:0]  pci_ad_out ;
output          pci_ad_en_out ;
input   [3:0]   pci_cbe_in ;

/*==================================================================================================================
Other side of master state machine - the one that issues requests.
==================================================================================================================*/
/*==================================================================================================================
address output - address always qualifies current data present at data_i input or data_o output and because of that
				 it must be written to counter at address phase. After all dta phases address counter must be 
				 incremented and the address claim input signal tells if the address falls into valid address space.
				 If address (address from PCI bus at address phase) is not claimed, then PCI Target must not respond
				 in any way, but if address from burst transfere crosses valid address space boundaries after first
				 data phase, then PCI Target respond with Target Abort.
==================================================================================================================*/
output  [31:0]  address_o ; // current request address output

/*==================================================================================================================
address claim input - address claim input always confirms if current address present at address_o output falls 
                      into valid address space (for address phase) or is still in the valid address space (for all
                      data phases after first one). Address decoders with address translation are implemented at
                      the backend. 
==================================================================================================================*/
input           addr_claim_i ; // current request address claim input

/*==================================================================================================================
bus command output - bus command output must be valid through all transfere to identifie what backend should do (it
                     is PCI bus command). Because of that it must be latched at address phase when req_o is 
                     asserted.
==================================================================================================================*/
output  [3:0]   bc_o ;      // current request bus command output

/*==================================================================================================================
data input - for PCI read bus commands - this bus provides valid read data from backend during read commands when 
             req_o and rdy_o are asserted (rdy_o - PCI Target state machine is ready to receive data) and backend
             signals `TRANSFERED status on status_i bus. On the rising edge of clock data must be latched from 
             data_i bus. After this, PCI Target state machine must deasert rdy_o if it is not ready to accept more 
             data.
==================================================================================================================*/
input   [31:0]  data_i ;    // for read operations - current dataphase data input

/*==================================================================================================================
data output - for PCI write bus commands - data on this bus is provided from PCI Target state machine during write
              commands and is qualified with rdy_o asserted. Backend is responsible for storing the data. Backend
              signals if it is ready to store data with `TRANSFERED status on status_i bus. When that happens, PCI 
              Target state machine should provide new data imediately after rising clock edge or deassert rdy_o if 
              it is not ready to do so.
==================================================================================================================*/
output  [31:0]  data_o ;    // for write operations - current request data output

/*==================================================================================================================
byte enable output - active low byte enables drived from BE# pins on pci during transfers - they are valid at every
                     data phase (rdy_o asserted). No matter which byte enables are active still all data must be 
                     privided to backend or to PCI (write or read). 
==================================================================================================================*/
output   [3:0]  be_o ;      // current dataphase byte enable outputs

/*==================================================================================================================
request and ready outputs from PCI Target - req_o output means that PCI target has some transaction request for
backend (address and all data phases following).
During write requests PCI Target must provide address and bus command at address phase (and all data phases) and 
byte enables and data during all data phases. During whole request req_o must be aserted, meanwhile rdy_o is 
asserted with valid byte enables and data. Transaction and data phases are completed regarding status_i bus from 
backend described later.
During read requests PCI Target must provide address and bus command at address phase (and all data phases) and
byte enables during all data phases. During whole request req_o must be aserted, meanwhile rdy_o is asserted when
PCI Target state machine is ready (PCI Master is pending for data) for data until backend signal status on status_i
bus that data are ready (`TRANSFERED) or to terminate with reatry (`RETRY - because all read transactions are
delayed read transactions). Other posibilitys on status_i bus are described later. When status `TRANSFERED is 
signaled by backend, PCI Target must release rdy_o signal if it is not capable to receive more data. When PCI 
Target state machine terminates with retry, PCI Master will release the PCI bus and PCI Target must also deasert 
req_o signal. 
==================================================================================================================*/
output          req_o ;     // PCI Target cycle is requested to backend
output          rdy_o ;     // requestor indicates that data is ready to be sent for write transaction and ready to
                            // be received on read transaction
/*==================================================================================================================
Last output - this output in conjuction with req_o and rdy_o qualifies last data beat in current request.
              for single transfers all three signals are asserted in parallel, for burst transfers last_o is inactive
              until last intended data is present at data_o ( assign last_o = pci_frame_in && ~pci_irdy_in ; boath
              PCI signals are active low ).
==================================================================================================================*/
output          last_o ;    // last dataphase in current transaction indicator

/*==================================================================================================================
Status input - Status is always provided from the backend depending on FIFO progress. Status descriptions:
`WAIT - Default status - Means that current request is not processed in any way yet - this status is propagated
        to Target state machine when backend is waiting FIFO to be ready while Target state machine wants to read/
        write from/to FIFO (PCI Target then inserts wait states), or rdy_o output is down (nothing is to be done).
`TRANSFERED - Status means, that current data phase is completing succesfully on FIFO. During writes, this status 
              signals to the PCI Target state machine that it should provide new data imediately after rising clock 
              edge or deassert rdy_o if it is not ready to do so. During reads, this status signals to the PCI Target 
              state machine that on the rising edge of clock data must be latched from data_i bus. After this, PCI 
              Target state machine must deasert rdy_o if it is not ready to accept more data.
`DISCONNECT_WO_DATA - Backend signals this status to the PCI Target state machine when e.g. during write transaction,
                      in the middle of the burst, FIFO becommes full and no data can be written into it anymore.
                      Disconnect without data is normal termination and PCI Target state machine must also emediately
                      deaserts rdy_o and req_o.
`DISCONNECT_W_DATA  - Backend signals this status to the PCI Target state machine when e.g. during write transaction,
                      in the middle of the burst, FIFO is almost full and only this data can be written into it or
                      if burst is attempt to I/O space, to which only single transactions are allowed, this way
                      burst is broken into single transfers.
                      Disconnect with data is normal termination and PCI Target state machine must also emediately
                      deaserts rdy_o and req_o.
`TABORT - Target Abort status - Backend signals this status to PCI Target state machine when there e.g. data parrity 
                                error occures (depending of the backend design) in the middle of the transfere or 
                                e.g. address parrity error occurs at the beginning of the transfere or if address
                                in the middle of the burst transfere crosses the valid address space boundaries.
                                PCI Target state machine must terminate the transaction with Target Abort if it 
                                receives this status and it must deaserts rdy_o and req_o.
`RETRY  - Backend signals this status to PCI Target state machine whenever there is a read transaction and backend
          cannot start providing data because it does not have any data ready yet (e.g. FIFO still empty since all
          reads are delayed reads) or when there is a write transaction and backend cannot start receiving data 
          because FIFO is still full. 
          PCI Target state machine is responsible to terminate transaxction with retry and emediately deaserts
          rdy_o and req_o.
==================================================================================================================*/
input   [3:0]   status_i ;  // current dataphase completion status


endmodule