create_clock -name sys50 -period 20.000 -waveform {0 10.000} [get_ports {clk50m}]
# Conservative 100MHz budget; measure actual PCLK and DVP setup/hold on board.
create_clock -name camera_pclk -period 10.000 -waveform {0 5.000} [get_ports {cam1_pclk}]
# PLL clocks are derived by Gowin from the instantiated PLL IP.
# Only the FIRST stage D pins of asynchronous toggle synchronizers are excepted.
# Stage 1 -> stage 2 and bundled row metadata paths remain timed.
set_false_path -to [get_pins {video/rows/ack1*/D}]
set_false_path -to [get_pins {video/rows/pub1*/D}]
# Diagnostic Gray counters and the publish toggle use explicit synchronizers.
# Except FIRST stages only; the synchronizer propagation stages remain timed.
set_false_path -to [get_pins {video/flow_monitor/sync1*/D}]
set_false_path -to [get_pins {diag_sync_0*/D}]
set_false_path -to [get_pins {video/flow_monitor/req_sync_0*/D}]
set_false_path -to [get_pins {video/flow_monitor/ack_sync_0*/D}]
# Snapshot is held until the next 100ms request; acknowledge crosses two stages.
set_max_delay -from [get_pins {video/flow_monitor/line_snapshot*/Q}] -to [get_pins {video/flow_monitor/line_received*/D}] 20.000
# Row/frame metadata is held until the acknowledgement returns. The receiver
# captures it only after the publish toggle crosses two synchronizer stages
# (at least 16ns at 125MHz). Bound these bundled-data paths to 8ns, rather than
# assuming a fixed 2ns phase relationship between independent camera/TX clocks.
set_max_delay -from [get_pins {video/rows/frame0*/Q}] -to [get_pins {video/rows/send_frame*/D}] 8.000
set_max_delay -from [get_pins {video/rows/frame1*/Q}] -to [get_pins {video/rows/send_frame*/D}] 8.000
set_max_delay -from [get_pins {video/rows/row0*/Q}] -to [get_pins {video/rows/send_row*/D}] 8.000
set_max_delay -from [get_pins {video/rows/row1*/Q}] -to [get_pins {video/rows/send_row*/D}] 8.000
# Raw reset assertion/release is asynchronous at the reset synchronizer only.
# Its three-clock deassertion chain and downstream reset paths remain timed.
set_false_path -to [get_pins {video/r50/pipe*/CLEAR}]
set_false_path -to [get_pins {video/r125/pipe*/CLEAR}]
set_false_path -to [get_pins {video/rp/pipe*/CLEAR}]
# Async UART/encoder inputs use explicit synchronizers. External RGMII I/O timing
# and camera input delays require the board-specific measurement before signoff.
# Diagnostic reports only; clock periods and timing exceptions stay unchanged.
report_timing -setup -max_paths 300 -max_common_paths 1
