test_bench.v file
Instantiates all currenlty available modules developed for
pci bridge design. PCI side of the bridge is only
simulated at this time - stubs are fed with some 
test vectors, to simulate PCI interface operation.
Test bench is intended only for HOST implementation, since
configuration space accesses, configuration and interrupt acknowledge
generation are HOST implementation features.