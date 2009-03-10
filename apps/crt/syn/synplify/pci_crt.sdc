# Synplicity, Inc. constraint file
# /shared/projects/pci/mihad/pci/apps/crt/syn/synplify/pci_crt.sdc
# Written on Mon Mar 10 13:33:22 2003
# by Synplify Pro, 7.2         Scope Editor

#
# Clocks
#
define_clock            -name {CLK}  -period 30.000 -clockgroup pci_clkgrp
define_clock            -name {CRT_CLK}  -period 44.000 -clockgroup crt_clkgrp

#
# Inputs/Outputs
#
define_input_delay               {DEVSEL}  23.00 -ref CLK:r
define_input_delay               {TRDY}  23.00 -ref CLK:r
define_input_delay               {STOP}  23.00 -ref CLK:r
define_input_delay               {IDSEL}  23.00 -ref CLK:r
define_input_delay               {FRAME}  23.00 -ref CLK:r
define_input_delay               {IRDY}  23.00 -ref CLK:r
define_input_delay               {GNT}  20.00 -ref CLK:r
define_input_delay               {PAR}  23.00 -ref CLK:r
define_input_delay               {PERR}  23.00 -ref CLK:r
define_input_delay               {AD0}  23.00 -ref CLK:r
define_input_delay               {AD1}  23.00 -ref CLK:r
define_input_delay               {AD2}  23.00 -ref CLK:r
define_input_delay               {AD3}  23.00 -ref CLK:r
define_input_delay               {AD4}  23.00 -ref CLK:r
define_input_delay               {AD5}  23.00 -ref CLK:r
define_input_delay               {AD6}  23.00 -ref CLK:r
define_input_delay               {AD7}  23.00 -ref CLK:r
define_input_delay               {AD8}  23.00 -ref CLK:r
define_input_delay               {AD9}  23.00 -ref CLK:r
define_input_delay               {AD10}  23.00 -ref CLK:r
define_input_delay               {AD11}  23.00 -ref CLK:r
define_input_delay               {AD12}  23.00 -ref CLK:r
define_input_delay               {AD13}  23.00 -ref CLK:r
define_input_delay               {AD14}  23.00 -ref CLK:r
define_input_delay               {AD15}  23.00 -ref CLK:r
define_input_delay               {AD16}  23.00 -ref CLK:r
define_input_delay               {AD17}  23.00 -ref CLK:r
define_input_delay               {AD18}  23.00 -ref CLK:r
define_input_delay               {AD19}  23.00 -ref CLK:r
define_input_delay               {AD20}  23.00 -ref CLK:r
define_input_delay               {AD21}  23.00 -ref CLK:r
define_input_delay               {AD22}  23.00 -ref CLK:r
define_input_delay               {AD23}  23.00 -ref CLK:r
define_input_delay               {AD24}  23.00 -ref CLK:r
define_input_delay               {AD25}  23.00 -ref CLK:r
define_input_delay               {AD26}  23.00 -ref CLK:r
define_input_delay               {AD27}  23.00 -ref CLK:r
define_input_delay               {AD28}  23.00 -ref CLK:r
define_input_delay               {AD29}  23.00 -ref CLK:r
define_input_delay               {AD30}  23.00 -ref CLK:r
define_input_delay               {AD31}  23.00 -ref CLK:r
define_input_delay               {CBE0}  23.00 -ref CLK:r
define_input_delay               {CBE1}  23.00 -ref CLK:r
define_input_delay               {CBE2}  23.00 -ref CLK:r
define_input_delay               {CBE3}  23.00 -ref CLK:r
define_output_delay              {AD0}  19.00 -ref CLK:r
define_output_delay              {AD1}  19.00 -ref CLK:r
define_output_delay              {AD2}  19.00 -ref CLK:r
define_output_delay              {AD3}  19.00 -ref CLK:r
define_output_delay              {AD4}  19.00 -ref CLK:r
define_output_delay              {AD5}  19.00 -ref CLK:r
define_output_delay              {AD6}  19.00 -ref CLK:r
define_output_delay              {AD7}  19.00 -ref CLK:r
define_output_delay              {AD8}  19.00 -ref CLK:r
define_output_delay              {AD9}  19.00 -ref CLK:r
define_output_delay              {AD10}  19.00 -ref CLK:r
define_output_delay              {AD11}  19.00 -ref CLK:r
define_output_delay              {AD12}  19.00 -ref CLK:r
define_output_delay              {AD13}  19.00 -ref CLK:r
define_output_delay              {AD14}  19.00 -ref CLK:r
define_output_delay              {AD15}  19.00 -ref CLK:r
define_output_delay              {AD16}  19.00 -ref CLK:r
define_output_delay              {AD17}  19.00 -ref CLK:r
define_output_delay              {AD18}  19.00 -ref CLK:r
define_output_delay              {AD19}  19.00 -ref CLK:r
define_output_delay              {AD20}  19.00 -ref CLK:r
define_output_delay              {AD21}  19.00 -ref CLK:r
define_output_delay              {AD22}  19.00 -ref CLK:r
define_output_delay              {AD23}  19.00 -ref CLK:r
define_output_delay              {AD24}  19.00 -ref CLK:r
define_output_delay              {AD25}  19.00 -ref CLK:r
define_output_delay              {AD26}  19.00 -ref CLK:r
define_output_delay              {AD27}  19.00 -ref CLK:r
define_output_delay              {AD28}  19.00 -ref CLK:r
define_output_delay              {AD29}  19.00 -ref CLK:r
define_output_delay              {AD30}  19.00 -ref CLK:r
define_output_delay              {AD31}  19.00 -ref CLK:r
define_output_delay              {CBE0}  19.00 -ref CLK:r
define_output_delay              {CBE1}  19.00 -ref CLK:r
define_output_delay              {CBE2}  19.00 -ref CLK:r
define_output_delay              {CBE3}  19.00 -ref CLK:r
define_output_delay              {DEVSEL}  19.00 -ref CLK:r
define_output_delay              {TRDY}  19.00 -ref CLK:r
define_output_delay              {STOP}  19.00 -ref CLK:r
define_output_delay              {FRAME}  19.00 -ref CLK:r
define_output_delay              {IRDY}  19.00 -ref CLK:r
define_output_delay              {REQ}  18.00 -ref CLK:r
define_output_delay              {PAR}  19.00 -ref CLK:r
define_output_delay              {PERR}  19.00 -ref CLK:r
define_output_delay              {SERR}  19.00 -ref CLK:r
define_input_delay               -default  10.00 -ref CRT_CLK:r
define_output_delay              -default  10.00 -ref CRT_CLK:r

#
# Registers
#
define_reg_output_delay          {*sync_data_out*} -route 20.00
define_reg_output_delay          {*meta_q_o*} -route 20.00

#
# Multicycle Path
#

#
# False Path
#

#
# Attributes
#
define_attribute          {CLK} xc_loc {P185}
define_attribute          {INTA} xc_loc {P195}
define_attribute          {RST} xc_loc {P199}
define_attribute          {GNT} xc_loc {P200}
define_attribute          {REQ} xc_loc {P201}
define_attribute          {AD31} xc_loc {P203}
define_attribute          {AD30} xc_loc {P204}
define_attribute          {AD29} xc_loc {P205}
define_attribute          {AD28} xc_loc {P206}
define_attribute          {AD27} xc_loc {P3}
define_attribute          {AD26} xc_loc {P4}
define_attribute          {AD25} xc_loc {P5}
define_attribute          {AD24} xc_loc {P6}
define_attribute          {CBE3} xc_loc {P8}
define_attribute          {IDSEL} xc_loc {P9}
define_attribute          {AD23} xc_loc {P10}
define_attribute          {AD22} xc_loc {P14}
define_attribute          {AD21} xc_loc {P15}
define_attribute          {AD20} xc_loc {P16}
define_attribute          {AD19} xc_loc {P17}
define_attribute          {AD18} xc_loc {P18}
define_attribute          {AD17} xc_loc {P20}
define_attribute          {AD16} xc_loc {P21}
define_attribute          {CBE2} xc_loc {P22}
define_attribute          {FRAME} xc_loc {P23}
define_attribute          {IRDY} xc_loc {P24}
define_attribute          {TRDY} xc_loc {P27}
define_attribute          {DEVSEL} xc_loc {P29}
define_attribute          {STOP} xc_loc {P30}
define_attribute          {PERR} xc_loc {P31}
define_attribute          {SERR} xc_loc {P33}
define_attribute          {PAR} xc_loc {P34}
define_attribute          {CBE1} xc_loc {P35}
define_attribute          {AD15} xc_loc {P36}
define_attribute          {AD14} xc_loc {P37}
define_attribute          {AD13} xc_loc {P41}
define_attribute          {AD12} xc_loc {P42}
define_attribute          {AD11} xc_loc {P43}
define_attribute          {AD10} xc_loc {P45}
define_attribute          {AD9} xc_loc {P46}
define_attribute          {AD8} xc_loc {P47}
define_attribute          {CBE0} xc_loc {P48}
define_attribute          {AD7} xc_loc {P49}
define_attribute          {AD6} xc_loc {P57}
define_attribute          {AD5} xc_loc {P58}
define_attribute          {AD4} xc_loc {P59}
define_attribute          {AD3} xc_loc {P61}
define_attribute          {AD2} xc_loc {P62}
define_attribute          {AD1} xc_loc {P63}
define_attribute          {AD0} xc_loc {P67}
define_attribute          {CRT_CLK} xc_loc {P182}
define_attribute          {HSYNC} xc_loc {P83}
define_attribute          {VSYNC} xc_loc {P84}
define_attribute          {RGB4} xc_loc {P166}
define_attribute          {RGB5} xc_loc {P167}
define_attribute          {RGB6} xc_loc {P168}
define_attribute          {RGB7} xc_loc {P172}
define_attribute          {RGB8} xc_loc {P173}
define_attribute          {RGB9} xc_loc {P174}
define_attribute          {RGB10} xc_loc {P175}
define_attribute          {RGB11} xc_loc {P176}
define_attribute          {RGB12} xc_loc {P178}
define_attribute          {RGB13} xc_loc {P179}
define_attribute          {RGB14} xc_loc {P180}
define_attribute          {RGB15} xc_loc {P181}
define_attribute          {LED} xc_loc {P202}
define_global_attribute          syn_useioff {1}
define_attribute          {v:work.pci_cbe_en_crit} syn_hier {hard}
define_attribute          {v:work.pci_frame_crit} syn_hier {hard}
define_attribute          {v:work.pci_frame_en_crit} syn_hier {hard}
define_attribute          {v:work.pci_frame_load_crit} syn_hier {hard}
define_attribute          {v:work.pci_irdy_out_crit} syn_hier {hard}
define_attribute          {v:work.pci_mas_ad_en_crit} syn_hier {hard}
define_attribute          {v:work.pci_mas_ad_load_crit} syn_hier {hard}
define_attribute          {v:work.pci_mas_ch_state_crit} syn_hier {hard}
define_attribute          {v:work.pci_par_crit} syn_hier {hard}
define_attribute          {v:work.pci_io_mux_ad_en_crit} syn_hier {hard}
define_attribute          {v:work.pci_io_mux_ad_load_crit} syn_hier {hard}
define_attribute          {v:work.pci_target32_clk_en} syn_hier {hard}
define_attribute          {v:work.pci_target32_devs_crit} syn_hier {hard}
define_attribute          {v:work.pci_target32_stop_crit} syn_hier {hard}
define_attribute          {v:work.pci_target32_trdy_crit} syn_hier {hard}
define_attribute          {v:work.pci_perr_crit} syn_hier {hard}
define_attribute          {v:work.pci_perr_en_crit} syn_hier {hard}
define_attribute          {v:work.pci_serr_crit} syn_hier {hard}
define_attribute          {v:work.pci_serr_en_crit} syn_hier {hard}

#
# Compile Points
#

#
# Other Constraints
#
