# Synplicity, Inc. constraint file
# /shared/projects/pci/mihad/pci/apps/test/syn/synplify/pci_test_top.sdc
# Written on Thu Apr 17 16:11:16 2003
# by Amplify, Amplify 3.1          Scope Editor

#
# Clocks
#
define_clock            -name {pci_clk_pad_i}  -period 30.000 -clockgroup pci_clkgrp

#
# Inputs/Outputs
#
define_input_delay               {pci_devsel_pad_io}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_trdy_pad_io}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_stop_pad_io}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_idsel_pad_i}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_frame_pad_io}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_irdy_pad_io}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_gnt_pad_i}  20.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_par_pad_io}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_perr_pad_io}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad0_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad1_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad2_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad3_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad4_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad5_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad6_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad7_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad8_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad9_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad10_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad11_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad12_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad13_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad14_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad15_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad16_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad17_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad18_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad19_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad20_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad21_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad22_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad23_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad24_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad25_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad26_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad27_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad28_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad29_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad30_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_ad31_pad_io}  23.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_cbe0_pad_io}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_cbe1_pad_io}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_cbe2_pad_io}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_input_delay               {pci_cbe3_pad_io}  23.00 -route 2.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad0_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad1_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad2_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad3_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad4_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad5_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad6_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad7_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad8_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad9_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad10_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad11_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad12_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad13_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad14_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad15_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad16_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad17_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad18_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad19_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad20_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad21_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad22_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad23_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad24_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad25_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad26_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad27_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad28_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad29_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad30_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_ad31_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_cbe0_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_cbe1_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_cbe2_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_cbe3_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_devsel_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_trdy_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_stop_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_frame_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_irdy_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_req_pad_o}  18.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_par_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_perr_pad_io}  19.00 -ref pci_clk_pad_i:r
define_output_delay              {pci_serr_pad_o}  19.00 -ref pci_clk_pad_i:r

#
# Registers
#
define_reg_output_delay          {*sync_data_out*} -route 10.00

#
# Multicycle Path
#

#
# False Path
#

#
# Attributes
#
define_attribute          {pci_clk_pad_i} xc_loc {P185}
define_attribute          {pci_rst_pad_i} xc_loc {P199}
define_attribute          {pci_gnt_pad_i} xc_loc {P200}
define_attribute          {pci_req_pad_o} xc_loc {P201}
define_attribute          {pci_ad31_pad_io} xc_loc {P203}
define_attribute          {pci_ad30_pad_io} xc_loc {P204}
define_attribute          {pci_ad29_pad_io} xc_loc {P205}
define_attribute          {pci_ad28_pad_io} xc_loc {P206}
define_attribute          {pci_ad27_pad_io} xc_loc {P3}
define_attribute          {pci_ad26_pad_io} xc_loc {P4}
define_attribute          {pci_ad25_pad_io} xc_loc {P5}
define_attribute          {pci_ad24_pad_io} xc_loc {P6}
define_attribute          {pci_cbe3_pad_io} xc_loc {P8}
define_attribute          {pci_idsel_pad_i} xc_loc {P9}
define_attribute          {pci_ad23_pad_io} xc_loc {P10}
define_attribute          {pci_ad22_pad_io} xc_loc {P14}
define_attribute          {pci_ad21_pad_io} xc_loc {P15}
define_attribute          {pci_ad20_pad_io} xc_loc {P16}
define_attribute          {pci_ad19_pad_io} xc_loc {P17}
define_attribute          {pci_ad18_pad_io} xc_loc {P18}
define_attribute          {pci_ad17_pad_io} xc_loc {P20}
define_attribute          {pci_ad16_pad_io} xc_loc {P21}
define_attribute          {pci_cbe2_pad_io} xc_loc {P22}
define_attribute          {pci_frame_pad_io} xc_loc {P23}
define_attribute          {pci_irdy_pad_io} xc_loc {P24}
define_attribute          {pci_trdy_pad_io} xc_loc {P27}
define_attribute          {pci_devsel_pad_io} xc_loc {P29}
define_attribute          {pci_stop_pad_io} xc_loc {P30}
define_attribute          {pci_perr_pad_io} xc_loc {P31}
define_attribute          {pci_serr_pad_o} xc_loc {P33}
define_attribute          {pci_par_pad_io} xc_loc {P34}
define_attribute          {pci_cbe1_pad_io} xc_loc {P35}
define_attribute          {pci_ad15_pad_io} xc_loc {P36}
define_attribute          {pci_ad14_pad_io} xc_loc {P37}
define_attribute          {pci_ad13_pad_io} xc_loc {P41}
define_attribute          {pci_ad12_pad_io} xc_loc {P42}
define_attribute          {pci_ad11_pad_io} xc_loc {P43}
define_attribute          {pci_ad10_pad_io} xc_loc {P45}
define_attribute          {pci_ad9_pad_io} xc_loc {P46}
define_attribute          {pci_ad8_pad_io} xc_loc {P47}
define_attribute          {pci_cbe0_pad_io} xc_loc {P48}
define_attribute          {pci_ad7_pad_io} xc_loc {P49}
define_attribute          {pci_ad6_pad_io} xc_loc {P57}
define_attribute          {pci_ad5_pad_io} xc_loc {P58}
define_attribute          {pci_ad4_pad_io} xc_loc {P59}
define_attribute          {pci_ad3_pad_io} xc_loc {P61}
define_attribute          {pci_ad2_pad_io} xc_loc {P62}
define_attribute          {pci_ad1_pad_io} xc_loc {P63}
define_attribute          {pci_ad0_pad_io} xc_loc {P67}
define_attribute          {clk_pad_i} xc_loc {P182}
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
define_attribute          {pci_gnt_pad_i} xc_padtype {IBUF_PCI33_5}
define_attribute          {pci_req_pad_o} xc_padtype {OBUFT_PCI33_5}
define_attribute          {pci_ad31_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad30_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad29_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad28_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad27_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad26_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad25_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad24_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_cbe3_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_idsel_pad_i} xc_padtype {IBUF_PCI33_5}
define_attribute          {pci_ad23_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad22_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad21_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad20_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad19_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad18_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad17_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad16_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_cbe2_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_frame_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_irdy_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_trdy_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_devsel_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_stop_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_perr_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_serr_pad_o} xc_padtype {OBUFT_PCI33_5}
define_attribute          {pci_par_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_cbe1_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad15_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad14_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad13_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad12_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad11_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad10_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad9_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad8_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_cbe0_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad7_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad6_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad5_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad4_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad3_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad2_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad1_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {pci_ad0_pad_io} xc_padtype {IOBUF_PCI33_5}
define_attribute          {v:work.synchronizer_flop_1_0} syn_hier {hard}
define_attribute          {v:work.synchronizer_flop_3_0} syn_hier {hard}
define_attribute          {v:work.synchronizer_flop_4_0} syn_hier {hard}
define_attribute          {v:work.synchronizer_flop_4_1} syn_hier {hard}
define_attribute          {v:work.synchronizer_flop_4_3} syn_hier {hard}
define_attribute          {v:work.synchronizer_flop_6_0} syn_hier {hard}
define_attribute          {v:work.synchronizer_flop_7_0} syn_hier {hard}
define_attribute          {v:work.synchronizer_flop_7_3} syn_hier {hard}

#
# Other Constraints
#

#
#  Order of waveforms 
#
