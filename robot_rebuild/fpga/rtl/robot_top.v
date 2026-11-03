`timescale 1ns/1ps
module robot_top #(parameter CLK_HZ=50000000,ENC_L_INV=0,ENC_R_INV=1,
 parameter MOT_L_INV=0,MOT_R_INV=0,USE_EXT_ESTOP=0,
 // Receive/diagnose by default; enable control only after mounting/scan checks.
 parameter LIDAR_CONTROL=0,LIDAR_FRONT_ZERO_Q6=0,LIDAR_USB_FORWARD=1,
 // New geometry remains observational until the complete model is measured.
 parameter LIDAR_PLANNER_ENABLE=1,LIDAR_ADVANCED_CONTROL=0,LIDAR_MODEL_VALIDATED=0,
 parameter LIDAR_BODY_RADIUS_MM=120,LIDAR_DECEL_MM_S2=200,LIDAR_INPUT_BUDGET_MS=300,
 // Zero keeps the main/manual heading on IMU without global fusion.
 parameter MOTION_TRACK_MM=0,MOTION_COUNTS_PER_M=7640,MOTION_YAW_SIGN=1,
 // Navigation uses wheel heading consistently for pose and straight control.
 parameter NAV_USE_ENCODER=1,
 // User measured the drive-wheel centre spacing as 10cm on 2026-10-29.
 // Local avoidance geometry and over-travel guard; global fusion stays opt-in.
 parameter AVOID_TRACK_MM=100,
 parameter AVOID_ANGLE_TOLERANCE_CD=500,AVOID_FINAL_TOLERANCE_CD=1000,
 parameter AVOID_MIN_PIVOT_COUNTS=4,AVOID_SETTLE_JITTER_COUNTS=1,
 parameter USE_PREDICTIVE=1,PRED_ALPHA_L_SHIFT=2,PRED_ALPHA_R_SHIFT=2,
 parameter PRED_YAW_GAIN_Q8=0,PRED_MODEL_VALIDATED=0,PRED_SLEW_COUNTS=2,
 parameter LIDAR_MAP_DISPLAY=0)(
 input clk50m,estop_n,uart_rx,imu_rx,ultra_l_rx,ultra_r_rx,ir_rx,lidar_rx,
 input enc_l_a,enc_l_b,enc_r_a,enc_r_b,
 output uart_tx,motor_stby,motor_l_pwm,motor_l_in1,motor_l_in2,
 output motor_r_pwm,motor_r_in1,motor_r_in2,
 input cam1_pclk,cam1_href,cam1_vsync,input [7:0] cam1_data,
 output cam1_xclk,cam1_pwdn,cam1_rst_n,cam1_scl,inout cam1_sda,
 output rgmii_tx_clk,rgmii_tx_ctl,output [3:0] rgmii_txd
);
 reg [20:0] startup=0;
 always @(posedge clk50m) if(!startup[20]) startup<=startup+1;
 wire rst_n=startup[20];
 reg [2:0] estop_sync;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n) estop_sync<=0;else estop_sync<={estop_sync[1:0],estop_n};
 // Optional physical NC switch. Default build uses remote STOP/deadman.
 wire stop_permission=(USE_EXT_ESTOP!=0)?estop_sync[2]:1'b1;
 wire [7:0] rx_data,speed;wire rx_valid,armed,command_update,link_ok;
 wire [2:0] command;
 wire [7:0] host_rx_data,lidar_stream_data;wire host_rx_valid,lidar_stream_valid,lidar_clear;
 rb_uart_rx #(.CLK_HZ(CLK_HZ)) receiver(clk50m,rst_n,uart_rx,host_rx_data,host_rx_valid);
 rb_lidar_bridge_rx #(.CLK_HZ(CLK_HZ)) lidar_bridge(
  clk50m,rst_n,host_rx_data,host_rx_valid,rx_data,rx_valid,
  lidar_stream_data,lidar_stream_valid,lidar_clear);
 wire uart_armed,uart_update,uart_fresh,ir_valid,ir_repeat,ir_selected,uart_nav,uart_follow;
 wire [2:0] uart_command;
 wire [7:0] uart_speed,ir_key;
 rb_command_rx #(.CLK_HZ(CLK_HZ)) commands(clk50m,rst_n,rx_data,rx_valid,
  uart_armed,uart_command,uart_speed,uart_update,uart_fresh,uart_nav,uart_follow);
 rb_ir_nec #(.CLK_HZ(CLK_HZ)) infrared(clk50m,rst_n,ir_rx,ir_key,ir_valid,ir_repeat);
 // A volume click during visual following adjusts its preset, not ownership.
 // Key5 and all existing manual takeover/stop behavior still reach remote.
 wire follow_volume_key=uart_follow && !ir_selected &&
                        (ir_key==8'h15 || ir_key==8'h07);
 rb_remote_control #(.CLK_HZ(CLK_HZ)) remote(
  clk50m,rst_n,ir_key,ir_valid && !follow_volume_key,ir_repeat,uart_armed,uart_command,uart_speed,
  uart_update,uart_fresh,armed,command,speed,command_update,link_ok,ir_selected);
 wire [15:0] dl,dr;wire lu,ru,lok,rok;
 rb_ultrasonic #(.CLK_HZ(CLK_HZ)) left_sensor(clk50m,rst_n,ultra_l_rx,dl,lu,lok);
 rb_ultrasonic #(.CLK_HZ(CLK_HZ)) right_sensor(clk50m,rst_n,ultra_r_rx,dr,ru,rok);
 wire lidar_update,lidar_fresh;wire [2:0] lidar_valid;
 wire [15:0] lidar_front,lidar_left,lidar_right,lidar_scan_id,lidar_packets,lidar_errors;
 wire lidar_map_update,lidar_map_fresh;
 wire [575:0] lidar_map_ranges;wire [35:0] lidar_map_known;
 wire [15:0] lidar_map_scan_id,lidar_map_age,lidar_map_span;
 generate if(LIDAR_USB_FORWARD!=0)begin:usb_lidar
  rb_lidar_scan_map #(.CLK_HZ(CLK_HZ),.FRONT_ZERO_Q6(LIDAR_FRONT_ZERO_Q6)) lidar(
   .clk(clk50m),.rst_n(rst_n),.data(lidar_stream_data),.valid(lidar_stream_valid),.clear(lidar_clear),
   .update(lidar_update),.fresh(lidar_fresh),.sector_valid(lidar_valid),
   .front_mm(lidar_front),.left_mm(lidar_left),.right_mm(lidar_right),.scan_id(lidar_scan_id),
   .good_packets(lidar_packets),.bad_packets(lidar_errors),.map_update(lidar_map_update),
   .map_fresh(lidar_map_fresh),.map_ranges(lidar_map_ranges),.map_known(lidar_map_known),
   .map_scan_id(lidar_map_scan_id),.map_age_ms(lidar_map_age),.map_span_ms(lidar_map_span));
 end else begin:serial_lidar
  wire [7:0] serial_data;wire serial_valid;
  rb_uart_rx #(.CLK_HZ(CLK_HZ)) lidar_uart(clk50m,rst_n,lidar_rx,serial_data,serial_valid);
  rb_lidar_scan_map #(.CLK_HZ(CLK_HZ),.FRONT_ZERO_Q6(LIDAR_FRONT_ZERO_Q6),.PACKET_GAP_MS(5)) lidar(
   .clk(clk50m),.rst_n(rst_n),.data(serial_data),.valid(serial_valid),.clear(1'b0),
   .update(lidar_update),.fresh(lidar_fresh),.sector_valid(lidar_valid),
   .front_mm(lidar_front),.left_mm(lidar_left),.right_mm(lidar_right),.scan_id(lidar_scan_id),
   .good_packets(lidar_packets),.bad_packets(lidar_errors),.map_update(lidar_map_update),
   .map_fresh(lidar_map_fresh),.map_ranges(lidar_map_ranges),.map_known(lidar_map_known),
   .map_scan_id(lidar_map_scan_id),.map_age_ms(lidar_map_age),.map_span_ms(lidar_map_span));
 end endgenerate
 wire [15:0] roll,pitch,yaw;wire imu_update,imu_ok;
 imu_parse #(.CLK_FREQ(CLK_HZ)) imu(.clk(clk50m),.rst_n(rst_n),.uart_rx(imu_rx),
  .yaw(yaw),.roll(roll),.pitch(pitch),.data_valid(imu_update),.frame_error(),
  .frame_ok_cnt(),.debug_last_byte(),.o_rx_byte_cnt(),.o_gyro_z(),.o_gyro_z_valid());
 rb_fresh #(.CLK_HZ(CLK_HZ)) imu_alive(clk50m,rst_n,imu_update,imu_ok);
 wire signed [31:0] raw_l,raw_r;
 encoder_qdec le(.clk(clk50m),.rst_n(rst_n),.a(enc_l_a),.b(enc_l_b),.pos(raw_l),.dir());
 encoder_qdec re(.clk(clk50m),.rst_n(rst_n),.a(enc_r_a),.b(enc_r_b),.pos(raw_r),.dir());
 // Separate encoder sign correction from downstream subtract/compare chains.
 // Every consumer uses the same one-clock (20ns) delayed positions.
 reg signed [31:0] pos_l,pos_r;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)begin pos_l<=0;pos_r<=0;end
  else begin pos_l<=ENC_L_INV?-raw_l:raw_l;pos_r<=ENC_R_INV?-raw_r:raw_r;end
 wire signed [15:0] target_l,target_r,vl,vr;wire brake,lfault,rfault;
 wire clear_fault=command_update && !armed;
 wire motion_update,motion_ready,motion_fault;
 wire signed [15:0] motion_heading,motion_encoder_heading,motion_innovation,motion_yaw_rate;
 wire signed [15:0] motion_vl,motion_vr;
 wire signed [31:0] motion_pos_l,motion_pos_r;wire signed [15:0] motion_imu_yaw;
 wire [31:0] motion_stamp;wire [15:0] motion_flags;
 // Always derive an independent wheel heading for local detour turns. Global
 // IMU/encoder fusion remains opt-in through the separately calibrated track.
 localparam ENCODER_TRACK_MM=(MOTION_TRACK_MM!=0)?MOTION_TRACK_MM:AVOID_TRACK_MM;
 rb_motion_feedback #(.CLK_HZ(CLK_HZ),.TRACK_MM(ENCODER_TRACK_MM),
  .FUSE_ENCODER(MOTION_TRACK_MM!=0),.COUNTS_PER_M(MOTION_COUNTS_PER_M),.YAW_SIGN(MOTION_YAW_SIGN)) motion_feedback(
  .clk(clk50m),.rst_n(rst_n),.clear_fault(clear_fault),.imu_ok(imu_ok),.imu_update(imu_update),
  .pos_l(pos_l),.pos_r(pos_r),.imu_yaw($signed(yaw)),.update(motion_update),
  .ready(motion_ready),.fault(motion_fault),.heading(motion_heading),
  .encoder_heading(motion_encoder_heading),.innovation(motion_innovation),.yaw_rate(motion_yaw_rate),
  .velocity_l(motion_vl),.velocity_r(motion_vr),.sample_pos_l(motion_pos_l),.sample_pos_r(motion_pos_r),
  .sample_yaw(motion_imu_yaw),.timestamp_ms(motion_stamp),.flags(motion_flags));
 wire motion_ok=imu_ok && motion_ready && !motion_fault;
 wire [3:0] state;wire [15:0] fault;
 wire ir_stop_now=ir_valid && ir_key==8'h1c;
 // Key 0 requests RK visual following, never grants motor permission locally.
 // Saturating generation: repeats do not queue starts, reset is not a wrap event.
 reg [15:0] ir_follow_count;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)ir_follow_count<=0;
  else if(ir_valid && !ir_repeat && ir_key==8'h16 && ir_follow_count!=16'hffff)
   ir_follow_count<=ir_follow_count+1'b1;
 // Key 7 requests waypoint mode/start on RK; never ARM motors locally.
 reg [15:0] ir_nav_mode_count;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)ir_nav_mode_count<=0;
  else if(ir_valid && !ir_repeat && ir_key==8'h42 && ir_nav_mode_count!=16'hffff)
   ir_nav_mode_count<=ir_nav_mode_count+1'b1;
 // Key9 toggles RK video recording only; never changes motor permission.
 reg [15:0] ir_record_count;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)ir_record_count<=0;
  else if(ir_valid && !ir_repeat && ir_key==8'h4a && ir_record_count!=16'hffff)
   ir_record_count<=ir_record_count+1'b1;
 // Persistent STOP event generation lets the RK mission scheduler observe a
 // physical STOP even when a later DISARM releases IR ownership. UART traffic
 // must never clear this counter. Four bits wrap modulo 16 on repeated STOPs.
 reg [3:0] ir_stop_epoch;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n) ir_stop_epoch<=0;
  else if(ir_stop_now) ir_stop_epoch<=ir_stop_epoch+1'b1;
 wire nav_selected=uart_nav && !ir_selected && !ir_stop_now;
 wire follow_selected=uart_follow && !ir_selected && !ir_stop_now;
 // The FPGA owns the persistent preset. RK echoes this fresh, CRC-protected
 // setting in visual commands; changing it never ARM/enables motion locally.
 reg [7:0] ir_follow_speed;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)ir_follow_speed<=8;
  else if(follow_selected && ir_valid && !ir_repeat && !(uart_update && !uart_armed))begin
   if(ir_key==8'h15 && ir_follow_speed<12)ir_follow_speed<=ir_follow_speed+8'd2;
   else if(ir_key==8'h07 && ir_follow_speed>4)ir_follow_speed<=ir_follow_speed-8'd2;
  end
 // Missing sonar is observational only. Do not reuse a stale near distance.
 // Visual follow performs its valid-distance hold on RK, without FPGA detours.
 wire [15:0] control_dl,control_dr;
 wire control_lok,control_rok,control_lu,control_ru,lidar_follow_hold;
 rb_lidar_ranges #(.ENABLE_CONTROL(LIDAR_CONTROL)) lidar_ranges(
  follow_selected,dl,dr,lok,rok,lu,ru,lidar_update,lidar_valid,lidar_front,
  control_dl,control_dr,control_lok,control_rok,control_lu,control_ru,lidar_follow_hold);
 wire nav_run=nav_selected && armed && command==1 && speed!=0 && link_ok;
 wire cfg_write,cfg_commit,cfg_reset,cfg_clear,cfg_ready;
 wire [2:0] cfg_index,nav_index;
 wire signed [15:0] cfg_x,cfg_y,nav_heading;
 wire [3:0] cfg_count,nav_state;
 wire [7:0] cfg_ack_seq,cfg_ack_status,nav_speed;
 wire [2:0] nav_command;
 wire [2:0] nav_intent_command;wire [7:0] nav_intent_speed;wire nav_output_permitted;
 wire signed [31:0] nav_x,nav_y;
 wire nav_active,nav_done,nav_failed,nav_ready,nav_pose_ready;
 wire signed [15:0] nav_turn_error,safety_turn_error;
 wire signed [15:0] navigation_heading=(NAV_USE_ENCODER!=0)?motion_encoder_heading:motion_heading;
 wire nav_turn_valid,safety_turn_valid;
 wire signed [15:0] nav_goal_error;wire nav_goal_valid;
 localparam LIDAR_LOCAL_GEOMETRY_OK=MOTION_COUNTS_PER_M==7640 &&
   ENCODER_TRACK_MM==100 && LIDAR_BODY_RADIUS_MM==120 && LIDAR_DECEL_MM_S2==200 &&
   LIDAR_INPUT_BUDGET_MS==300;
 wire lidar_local_navigation=LIDAR_PLANNER_ENABLE!=0 && LIDAR_CONTROL!=0 &&
   LIDAR_ADVANCED_CONTROL!=0 && LIDAR_MODEL_VALIDATED!=0 && LIDAR_LOCAL_GEOMETRY_OK && nav_selected;
 wire predictive_busy,predictive_fault,predictive_valid;
 // Configuration changes require an actual stopped chassis, not just a
 // commanded brake. The fixed +/-1-count band tolerates observed stationary
 // jitter; cumulative creep still invalidates the full 200ms window.
 reg [31:0] cfg_div;
 reg [7:0] cfg_still;
 reg signed [31:0] cfg_prev_l,cfg_prev_r;
 reg [31:0] cfg_l_minus,cfg_l_plus,cfg_r_minus,cfg_r_plus;
 wire cfg_tick=cfg_div==CLK_HZ/1000-1;
 wire cfg_stationary=(cfg_prev_l==pos_l || cfg_l_minus==pos_l || cfg_l_plus==pos_l) &&
                     (cfg_prev_r==pos_r || cfg_r_minus==pos_r || cfg_r_plus==pos_r);
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n) begin
   cfg_div<=0;cfg_still<=0;cfg_prev_l<=0;cfg_prev_r<=0;
   cfg_l_minus<=32'hffffffff;cfg_l_plus<=1;cfg_r_minus<=32'hffffffff;cfg_r_plus<=1;
  end
  else begin
   cfg_div<=cfg_tick?0:cfg_div+1;
   if(!cfg_stationary)begin
    cfg_prev_l<=pos_l;cfg_prev_r<=pos_r;cfg_still<=0;
    cfg_l_minus<=pos_l-32'd1;cfg_l_plus<=pos_l+32'd1;
    cfg_r_minus<=pos_r-32'd1;cfg_r_plus<=pos_r+32'd1;
   end
   else if(cfg_tick && cfg_still<200) cfg_still<=cfg_still+1;
  end
 wire cfg_allowed=!armed && !nav_run && cfg_stationary && cfg_still>=200;
 rb_nav_config_rx #(.CLK_HZ(CLK_HZ)) nav_config(
  .clk(clk50m),.rst_n(rst_n),.data(rx_data),.valid(rx_valid),.cfg_allowed(cfg_allowed),
  .route_write(cfg_write),.route_index(cfg_index),.route_x(cfg_x),.route_y(cfg_y),
  .route_commit(cfg_commit),.route_count(cfg_count),.pose_reset(cfg_reset),
  .route_clear(cfg_clear),.route_ready(cfg_ready),.last_ack_seq(cfg_ack_seq),.last_ack_status(cfg_ack_status));
 rb_navigation #(.CLK_HZ(CLK_HZ),.COUNTS_PER_M(MOTION_COUNTS_PER_M),.YAW_SIGN(MOTION_YAW_SIGN),.LOCAL_STEERING_ENABLE(1)) navigation(
  .clk(clk50m),.rst_n(rst_n),.run(nav_run),
  .healthy(motion_ok && (NAV_USE_ENCODER==0 || motion_flags[10]) && stop_permission && !lfault && !rfault && !predictive_fault && fault==0 && (!nav_selected || link_ok)),
  .safety_state(state),.pos_l(pos_l),.pos_r(pos_r),.yaw(navigation_heading),
  .route_write(cfg_write),.route_index(cfg_index),.way_x(cfg_x),.way_y(cfg_y),
  .route_commit(cfg_commit),.route_count(cfg_count),.pose_reset(cfg_reset),.route_clear(cfg_clear),
  .command(nav_command),.speed(nav_speed),.active(nav_active),.done(nav_done),.failed(nav_failed),
  .state(nav_state),.index(nav_index),.x(nav_x),.y(nav_y),.heading(nav_heading),
  .route_ready(nav_ready),.pose_ready(nav_pose_ready),.turn_error(nav_turn_error),.turn_error_valid(nav_turn_valid),
  .local_steering(lidar_local_navigation),.goal_error(nav_goal_error),.goal_error_valid(nav_goal_valid),
  .intent_command(nav_intent_command),.intent_speed(nav_intent_speed),.output_permitted(nav_output_permitted));
 wire nav_forward=(nav_command==1 || nav_command==3 || nav_command==4);
 // Threshold distributes over the valid-source minimum. Avoid forming a
 // 16-bit fused minimum on the immediate command/goal/pause protection path.
 wire navigation_near=!follow_selected && ((lok && dl<200) || (rok && dr<200) ||
  (LIDAR_CONTROL!=0 && lidar_valid[0] && lidar_front<200));
 wire [2:0] navigation_request=(nav_forward && navigation_near)?3'd1:nav_command;
 wire [2:0] effective_command=ir_stop_now?3'd0:(nav_selected?navigation_request:command);
 wire [7:0] effective_speed=nav_selected?nav_speed:speed;
 wire lidar_plan_scope,lidar_plan_takeover,lidar_plan_hold,lidar_plan_current;
 wire lidar_dynamic_gate,lidar_dynamic_hold,lidar_follow_limit;
 wire signed [15:0] lidar_plan_l,lidar_plan_r,lidar_follow_l,lidar_follow_r;
 wire [3:0] lidar_plan_selected;wire [319:0] lidar_planning_diag;
 wire signed [15:0] lidar_goal_cd=nav_selected?nav_goal_error:
          ir_stop_now?16'sd0:command==3?16'sd9000:command==4?-16'sd9000:16'sd0;
 // Isolate remote/navigation decoding from planner revocation and arithmetic.
 // The raw bundle mismatch below still brakes before this sampling edge;
 // raw STOP/DISARM/health gates retain their independent wheel bypasses.
 reg [2:0] lidar_request_cmd;
 reg [7:0] lidar_request_speed;
 reg lidar_request_armed,lidar_request_follow,lidar_request_nav,lidar_request_goal_valid;
 reg lidar_request_stop,lidar_request_healthy,lidar_feedback_update,lidar_snapshot_update;
 reg signed [15:0] lidar_request_goal;
 wire lidar_goal_valid=!nav_selected || nav_goal_valid;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)begin
   lidar_request_cmd<=0;lidar_request_speed<=0;lidar_request_armed<=0;
   lidar_request_follow<=0;lidar_request_nav<=0;lidar_request_goal_valid<=0;lidar_request_goal<=0;
   lidar_request_stop<=1;lidar_request_healthy<=0;lidar_feedback_update<=0;lidar_snapshot_update<=0;
  end else begin
   lidar_request_cmd<=effective_command;lidar_request_speed<=effective_speed;lidar_request_armed<=armed;
   lidar_request_follow<=follow_selected;lidar_request_nav<=nav_selected;
   lidar_request_goal_valid<=lidar_goal_valid;lidar_request_goal<=lidar_goal_cd;
   lidar_request_stop<=ir_stop_now;
   lidar_request_healthy<=motion_ok && link_ok && stop_permission && !lfault && !rfault && !predictive_fault;
   lidar_feedback_update<=motion_update;lidar_snapshot_update<=lidar_map_update;
  end
 // Compare branch data before the mode selection: only one-bit mismatch
 // decisions depend on the live mode/STOP path. No guard is delayed.
 wire nav_intent_forward=nav_intent_command==1 || nav_intent_command==3 || nav_intent_command==4;
 wire nav_intent_cmd_pending=(nav_intent_forward && navigation_near)?
   (lidar_request_cmd!=3'd1):(lidar_request_cmd!=nav_intent_command);
 wire nav_cmd_pending=nav_output_permitted?nav_intent_cmd_pending:(lidar_request_cmd!=3'd0);
 wire nav_speed_pending=nav_output_permitted?(lidar_request_speed!=nav_intent_speed):(lidar_request_speed!=8'd0);
 wire lidar_cmd_pending=ir_stop_now?(lidar_request_cmd!=3'd0):
   nav_selected?nav_cmd_pending:(lidar_request_cmd!=command);
 wire lidar_speed_pending=nav_selected?nav_speed_pending:(lidar_request_speed!=speed);
 // Goal selection intentionally keeps navigation ahead of STOP, matching
 // lidar_goal_cd; command selection above intentionally keeps STOP first.
 wire lidar_goal_pending=nav_selected?(lidar_request_goal!=nav_goal_error):
   ir_stop_now?(lidar_request_goal!=16'sd0):
   command==3?(lidar_request_goal!=16'sd9000):
   command==4?(lidar_request_goal!=-16'sd9000):(lidar_request_goal!=16'sd0);
 wire lidar_context_pending=lidar_cmd_pending || lidar_speed_pending ||
   lidar_request_armed!=armed || lidar_request_follow!=follow_selected || lidar_request_nav!=nav_selected ||
   lidar_goal_pending || lidar_request_goal_valid!=lidar_goal_valid;
 wire lidar_context_guard=LIDAR_PLANNER_ENABLE!=0 && LIDAR_CONTROL!=0 && LIDAR_ADVANCED_CONTROL!=0 &&
   LIDAR_MODEL_VALIDATED!=0 && LIDAR_LOCAL_GEOMETRY_OK && lidar_context_pending;
 rb_lidar_controller #(.CLK_HZ(CLK_HZ),.COUNTS_PER_M(MOTION_COUNTS_PER_M),
  .WHEELBASE_MM(ENCODER_TRACK_MM),.BODY_RADIUS_MM(LIDAR_BODY_RADIUS_MM),
  .DECEL_MM_S2(LIDAR_DECEL_MM_S2),.INPUT_BUDGET_MS(LIDAR_INPUT_BUDGET_MS)) lidar_control(
  .clk(clk50m),.rst_n(rst_n),.enabled(LIDAR_PLANNER_ENABLE!=0),
  .control_enabled(LIDAR_CONTROL!=0 && LIDAR_ADVANCED_CONTROL!=0),
  .original_control_enabled(LIDAR_CONTROL!=0),
  .model_validated(LIDAR_MODEL_VALIDATED!=0),.follow_mode(lidar_request_follow),.nav_mode(lidar_request_nav),
  .armed(lidar_request_armed),.stop_now(lidar_request_stop),.healthy(lidar_request_healthy),
  .request_cmd(lidar_request_cmd),.request_speed(lidar_request_speed),.safety_state(state),
  .feedback_update(lidar_feedback_update),.velocity_l(motion_vl),.velocity_r(motion_vr),.goal_cd(lidar_request_goal),
  .goal_valid(lidar_request_goal_valid),
  .sonar_l_ok(lok),.sonar_r_ok(rok),.sonar_l(dl),.sonar_r(dr),
  .laser_front_ok(lidar_fresh && lidar_valid[0]),.laser_front(lidar_front),
  .map_update(lidar_snapshot_update),.map_fresh(lidar_map_fresh),.map_ranges(lidar_map_ranges),.map_known(lidar_map_known),
  .map_scan_id(lidar_map_scan_id),.map_age_ms(lidar_map_age),
  .scope(lidar_plan_scope),.takeover(lidar_plan_takeover),.hold(lidar_plan_hold),
  .follow_limit_valid(lidar_follow_limit),.dynamic_gate(lidar_dynamic_gate),.dynamic_hold(lidar_dynamic_hold),
  .target_l(lidar_plan_l),.target_r(lidar_plan_r),.follow_l(lidar_follow_l),.follow_r(lidar_follow_r),
  .current(lidar_plan_current),.selected(lidar_plan_selected),.diagnostic(lidar_planning_diag));
 // Capture the arbitrated request as one coherent bundle before the scan FSM.
 // Raw STOP/DISARM/link/IMU gates below still brake the bridges immediately;
 // only ordinary state-machine command processing gains one 20ns clock.
 reg safety_armed_in,safety_update_in;
 reg [2:0] safety_command_in;
 reg [7:0] safety_speed_in;
 reg [15:0] safety_dl_in,safety_dr_in;
 reg safety_lok_in,safety_rok_in,safety_lu_in,safety_ru_in;
 reg safety_follow_in;
 wire entering_follow=follow_selected && !safety_follow_in;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)begin
   safety_armed_in<=0;safety_update_in<=0;safety_command_in<=0;safety_speed_in<=0;
   safety_dl_in<=16'hfffe;safety_dr_in<=16'hfffe;
   safety_lok_in<=0;safety_rok_in<=0;safety_lu_in<=0;safety_ru_in<=0;
   safety_follow_in<=0;
  end else begin
   safety_armed_in<=armed && !ir_stop_now;
   safety_update_in<=command_update || ir_stop_now || entering_follow;
   // FAR ranges alone cannot cancel a scan already pending from manual mode.
   // One STOP bundle on entering FOLLOW cancels that old detour first.
   safety_command_in<=entering_follow?3'd0:effective_command;
   safety_speed_in<=effective_speed;safety_follow_in<=follow_selected;
   // Keep the follow-mode filter, range values, validity and update pulses
   // coherent with the request. Raw STOP still brakes the bridges directly.
   // In calibrated advanced DRIVE the new controller owns front clearance.
   // Its independent hold includes sonar evidence and unknown-map handling;
   // keep the old survey FSM dormant instead of competing with local arcs.
   safety_dl_in<=lidar_plan_scope?16'hfffe:control_dl;
   safety_dr_in<=lidar_plan_scope?16'hfffe:control_dr;
   safety_lok_in<=control_lok;safety_rok_in<=control_rok;safety_lu_in<=control_lu;safety_ru_in<=control_ru;
  end
 rb_safety_control #(.CLK_HZ(CLK_HZ),.YAW_SIGN(MOTION_YAW_SIGN),
  .TURN_TRACK_MM(ENCODER_TRACK_MM),.TURN_COUNTS_PER_M(MOTION_COUNTS_PER_M),
  .ANGLE_TOLERANCE_CD(AVOID_ANGLE_TOLERANCE_CD),.FINAL_ANGLE_TOLERANCE_CD(AVOID_FINAL_TOLERANCE_CD),
  .RELAX_AFTER_CORRECTIONS(1),.MIN_PIVOT_COUNTS(AVOID_MIN_PIVOT_COUNTS),
  .SETTLE_JITTER_COUNTS(AVOID_SETTLE_JITTER_COUNTS),.USE_LIDAR(LIDAR_CONTROL)) safety(
  .clk(clk50m),.rst_n(rst_n),.link_ok(link_ok),.imu_ok(motion_ok),.left_ok(safety_lok_in),.right_ok(safety_rok_in),
  .estop_ok(stop_permission),.armed(safety_armed_in),.cmd_update(safety_update_in),.command(safety_command_in),.speed(safety_speed_in),
  .dl(safety_dl_in),.dr(safety_dr_in),.left_update(safety_lu_in),.right_update(safety_ru_in),.pos_l(pos_l),.pos_r(pos_r),
  .yaw(motion_encoder_heading),.yaw_update(motion_update),
  .motor_fault(lfault||rfault||predictive_fault),.target_l(target_l),.target_r(target_r),.brake(brake),.state(state),.fault(fault),
  .turn_error(safety_turn_error),.turn_error_valid(safety_turn_valid),
  .lidar_update(lidar_update),.lidar_sides_valid(lidar_valid[1] && lidar_valid[2]),
  .lidar_left(lidar_left),.lidar_right(lidar_right),.lidar_front_valid(lidar_valid[0]));
 wire signed [15:0] heading_target_l,heading_target_r,heading_goal,heading_error,heading_trim;
 wire heading_holding;
 reg heading_nav_selected;
 reg signed [15:0] heading_selected_sample;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)heading_nav_selected<=0;else heading_nav_selected<=nav_selected;
 // The motion producer publishes heading in phase4, then update in phase5.
 // Register source selection continuously before the outer-loop arithmetic;
 // its coherent new heading is therefore ready before feedback_update is used.
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)heading_selected_sample<=0;
  else heading_selected_sample<=nav_selected?navigation_heading:motion_heading;
 wire heading_source_changed=(NAV_USE_ENCODER!=0) && (nav_selected!=heading_nav_selected);
 // Break safety/navigation arbitration before heading arithmetic. Sensor/STOP
 // gates below still reach PWM directly, independent of these two registers.
 reg safety_brake_stage,safety_hard_brake_stage,safety_lidar_owner;reg signed [15:0] safety_target_l,safety_target_r;
 // Carry the producer identity with its target, including follow <-> lattice
 // transitions for which the old one-bit lidar ownership stays asserted.
 wire [1:0] actuator_owner=lidar_plan_scope?2'd2:lidar_follow_limit?2'd1:2'd0;
 reg [1:0] safety_owner, wheel_owner;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)begin safety_owner<=0;wheel_owner<=0;end
  else begin safety_owner<=actuator_owner;wheel_owner<=safety_owner;end
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)begin safety_brake_stage<=1;safety_hard_brake_stage<=1;safety_lidar_owner<=0;safety_target_l<=0;safety_target_r<=0;end
  else begin
   safety_brake_stage<=brake || lidar_follow_hold || lidar_plan_hold;
   safety_hard_brake_stage<=brake || lidar_follow_hold;
   safety_lidar_owner<=lidar_dynamic_gate;
   safety_target_l<=lidar_plan_scope?lidar_plan_l:lidar_follow_limit?lidar_follow_l:target_l;
   safety_target_r<=lidar_plan_scope?lidar_plan_r:lidar_follow_limit?lidar_follow_r:target_r;
  end
 rb_heading_control #(.YAW_SIGN(MOTION_YAW_SIGN)) heading_control(
  .clk(clk50m),.rst_n(rst_n),.brake(safety_brake_stage || !armed || ir_stop_now || heading_source_changed),.healthy(motion_ok),
  .feedback_update(motion_update),.heading(heading_selected_sample),.target_l(safety_target_l),.target_r(safety_target_r),
  .corrected_l(heading_target_l),.corrected_r(heading_target_r),.holding(heading_holding),
  .goal(heading_goal),.error(heading_error),.trim(heading_trim));
 wire signed [15:0] predictive_l,predictive_r,predictive_end_error;
 wire [15:0] predictive_generation,predictive_cycles,predictive_flags;
 wire [3:0] predictive_selected;wire [31:0] predictive_cost;
 wire [31:0] predictive_time;wire signed [15:0] predictive_ref_l,predictive_ref_r;
 reg predictive_sample,predictive_sample_hold,predictive_brake_stage,predictive_healthy_stage;
 reg signed [15:0] predictive_ref_stage_l,predictive_ref_stage_r,predictive_error_stage;
 reg predictive_heading_stage;
 // Let the heading outer loop consume the same coherent snapshot first.
 wire predictive_heading_valid=safety_turn_valid || (nav_selected && nav_turn_valid) || heading_holding;
 wire signed [15:0] predictive_heading_error=safety_turn_valid?safety_turn_error:
                  (nav_selected && nav_turn_valid)?nav_turn_error:heading_error;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)begin
   predictive_sample<=0;predictive_sample_hold<=0;predictive_brake_stage<=1;predictive_healthy_stage<=0;
   predictive_ref_stage_l<=0;predictive_ref_stage_r<=0;predictive_error_stage<=0;predictive_heading_stage<=0;
  end else begin
   predictive_sample_hold<=motion_update;predictive_sample<=predictive_sample_hold;
   // The calibrated lidar controller owns speed while active. Keep the old
   // supervisor cancelled so no pre-takeover trajectory survives its return.
   predictive_brake_stage<=safety_brake_stage || lidar_dynamic_gate || safety_lidar_owner || !armed || ir_stop_now;
   predictive_healthy_stage<=motion_ok && link_ok && stop_permission && !lfault && !rfault;
   predictive_ref_stage_l<=heading_target_l;predictive_ref_stage_r<=heading_target_r;
   predictive_error_stage<=predictive_heading_error;predictive_heading_stage<=predictive_heading_valid;
  end
 rb_predictive_motion #(.ALPHA_L_SHIFT(PRED_ALPHA_L_SHIFT),.ALPHA_R_SHIFT(PRED_ALPHA_R_SHIFT),
  .YAW_GAIN_Q8(PRED_YAW_GAIN_Q8),.MODEL_VALIDATED(PRED_MODEL_VALIDATED),.SLEW_COUNTS(PRED_SLEW_COUNTS)) predictive_control(
  .clk(clk50m),.rst_n(rst_n),.enabled(USE_PREDICTIVE!=0),
  .brake(predictive_brake_stage),.healthy(predictive_healthy_stage),
  .clear_fault(clear_fault),.sample(predictive_sample),.heading_valid(predictive_heading_stage),
  .heading_error(predictive_error_stage),.reference_l(predictive_ref_stage_l),.reference_r(predictive_ref_stage_r),
  .velocity_l(motion_vl),.velocity_r(motion_vr),.sample_time(motion_stamp),.output_l(predictive_l),.output_r(predictive_r),
  .busy(predictive_busy),.fault(predictive_fault),.result_valid(predictive_valid),
  .generation(predictive_generation),.solve_cycles(predictive_cycles),.selected(predictive_selected),
  .cost(predictive_cost),.predicted_error(predictive_end_error),.flags(predictive_flags),
  .result_time(predictive_time),.sampled_l(predictive_ref_l),.sampled_r(predictive_ref_r));
 // Pipeline the nontrivial safety/target decode before the PI datapaths.
 // Raw STOP, DISARM and sensor/link/fault gates still brake without waiting
 // for these registers. Normal targets have two clocks (40ns) of staging.
 reg wheel_brake_hold,wheel_hard_brake_hold,wheel_lidar_owner;reg signed [15:0] wheel_target_l,wheel_target_r;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)begin wheel_brake_hold<=1;wheel_hard_brake_hold<=1;wheel_lidar_owner<=0;wheel_target_l<=0;wheel_target_r<=0;end
  else begin
   wheel_brake_hold<=safety_brake_stage;wheel_lidar_owner<=safety_lidar_owner;
   wheel_hard_brake_hold<=safety_hard_brake_stage;
   // Lidar candidates/caps already bound both wheels. Heading trim and the
   // old predictive supervisor must not change that checked geometry/speed.
   // Carry ownership with those staged targets into the existing wheel PI.
   wheel_target_l<=safety_lidar_owner?safety_target_l:predictive_l;
   wheel_target_r<=safety_lidar_owner?safety_target_r:predictive_r;
  end
 // Brake until both target stages have adopted the current owner, including
 // exit to a legacy reverse/turn request before its next predictive sample.
 wire wheel_owner_change=(lidar_dynamic_gate!=safety_lidar_owner) ||
                         (lidar_dynamic_gate!=wheel_lidar_owner) ||
                         (LIDAR_CONTROL!=0 && LIDAR_ADVANCED_CONTROL!=0 && LIDAR_MODEL_VALIDATED!=0 &&
                          ((actuator_owner!=safety_owner) || (actuator_owner!=wheel_owner)));
 wire wheel_hard_brake=wheel_hard_brake_hold || !armed || ir_stop_now || !link_ok || !motion_ok ||
                  !stop_permission || lfault || rfault || predictive_fault || lidar_follow_hold || wheel_owner_change ||
                  (lidar_dynamic_gate && lidar_dynamic_hold) || (lidar_context_guard && !lidar_dynamic_gate);
 // Include both old hold stages in the same bounded pause. Otherwise its
 // trailing register cycle would erase the PI immediately before resume.
 wire wheel_planner_pause=lidar_dynamic_gate && !wheel_hard_brake &&
                         (lidar_context_guard || lidar_plan_hold || safety_brake_stage || wheel_brake_hold);
 wire wheel_brake=wheel_hard_brake || wheel_planner_pause;
 localparam LIDAR_PI_PAUSE=(LIDAR_CONTROL!=0 && LIDAR_ADVANCED_CONTROL!=0 && LIDAR_MODEL_VALIDATED!=0);
 wire wheel_l_pwm,wheel_l_in1,wheel_l_in2,wheel_r_pwm,wheel_r_in1,wheel_r_in2;
 wire wheel_l_pending,wheel_r_pending;
 rb_wheel_control #(.CLK_HZ(CLK_HZ),.MOT_INV(MOT_L_INV),.PAUSE_ENABLE(LIDAR_PI_PAUSE),
  .ACTUATOR_PIPELINE(LIDAR_PI_PAUSE)) left_wheel(
  clk50m,rst_n,wheel_hard_brake,clear_fault,wheel_target_l,pos_l,vl,lfault,wheel_l_pwm,wheel_l_in1,wheel_l_in2,wheel_planner_pause,wheel_l_pending);
 rb_wheel_control #(.CLK_HZ(CLK_HZ),.MOT_INV(MOT_R_INV),.PAUSE_ENABLE(LIDAR_PI_PAUSE),
  .ACTUATOR_PIPELINE(LIDAR_PI_PAUSE)) right_wheel(
  clk50m,rst_n,wheel_hard_brake,clear_fault,wheel_target_r,pos_r,vr,rfault,wheel_r_pwm,wheel_r_in1,wheel_r_in2,wheel_planner_pause,wheel_r_pending);
 // Physical veto is deliberately outside the registered actuator state.
 // The wheel also holds a queued veto until a replacement duty commits.
 // Both wheels must have committed their packet before either drives. A
 // candidate can change just one target, or one wheel can take longer to
 // consume a reversal; publishing half of that pair is not acceptance.
 wire queued_pair_veto=wheel_l_pending || wheel_r_pending;
 wire bridge_veto=LIDAR_PI_PAUSE && (wheel_brake || queued_pair_veto);
 assign motor_l_pwm=rst_n && (bridge_veto || wheel_l_pwm);
 assign motor_l_in1=rst_n && (bridge_veto || wheel_l_in1);
 assign motor_l_in2=rst_n && (bridge_veto || wheel_l_in2);
 assign motor_r_pwm=rst_n && (bridge_veto || wheel_r_pwm);
 assign motor_r_in1=rst_n && (bridge_veto || wheel_r_in1);
 assign motor_r_in2=rst_n && (bridge_veto || wheel_r_in2);
 // Mirror the accepted packet for telemetry, rather than reporting the
 // upstream target while the local actuator still owns the previous one.
 reg signed [15:0] actuator_target_l,actuator_target_r;
 reg [1:0] accepted_owner;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)begin actuator_target_l<=0;actuator_target_r<=0;accepted_owner<=0;end
  else begin actuator_target_l<=wheel_target_l;actuator_target_r<=wheel_target_r;accepted_owner<=wheel_owner;end
 wire actual_bridge_braked=(motor_l_in1 && motor_l_in2) && (motor_r_in1 && motor_r_in2);
 // Keep the bridge enabled for short braking on remote STOP or timeout.
 // Optional physical switch additionally disables STBY without clock latency.
 assign motor_stby=rst_n && ((USE_EXT_ESTOP!=0)?estop_n:1'b1);
 wire camera_ready;
 wire [239:0] video_diag;wire video_diag_toggle;
 rb_video_top video(clk50m,rst_n,cam1_pclk,cam1_href,cam1_vsync,cam1_data,
  cam1_xclk,cam1_pwdn,cam1_rst_n,cam1_scl,cam1_sda,rgmii_tx_clk,rgmii_tx_ctl,rgmii_txd,camera_ready,
  video_diag,video_diag_toggle);
 // Bundled-data CDC: source publishes a stable bus with a toggle every 100ms.
 reg [2:0] diag_sync;reg [239:0] camera_diag;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)diag_sync<=0;
  else diag_sync<={diag_sync[1:0],video_diag_toggle};
 // Capture the long-held bundle on the opposite edge, away from the PLL's
 // source publish edge. The following UART snapshot has a half cycle to settle.
 always @(negedge clk50m or negedge rst_n)
  if(!rst_n)camera_diag<=0;
  else if(diag_sync[2]!=diag_sync[1])camera_diag<=video_diag;
 reg [1:0] cam_ready_sync;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n) cam_ready_sync<=0;else cam_ready_sync<={cam_ready_sync[0],camera_ready};
 wire [7:0] health={cam_ready_sync[1],armed,(brake || lidar_follow_hold || lidar_plan_hold),stop_permission,rok,lok,motion_ok,link_ok};
 wire [7:0] nav_flags={1'b1,1'b1,nav_pose_ready,nav_selected,nav_ready,nav_failed,nav_done,nav_active};
 // Capture the entire diagnostic bundle together after the heading loop has
 // consumed this sample. Never mix a new heading with old goals/positions.
 reg [1:0] motion_publish;reg [319:0] motion_diag;
 reg signed [31:0] sample_l,sample_r;
 reg signed [15:0] sample_target_l,sample_target_r;
 reg sample_braked;
 always @(posedge clk50m or negedge rst_n)
  if(!rst_n)begin motion_publish<=0;motion_diag<=0;sample_l<=0;sample_r<=0;sample_target_l<=0;sample_target_r<=0;sample_braked<=1;end
  else begin
   motion_publish<={motion_publish[0],motion_update};
   if(motion_update)begin
    sample_l<=motion_pos_l;sample_r<=motion_pos_r;
    sample_target_l<=LIDAR_PI_PAUSE?actuator_target_l:wheel_target_l;
    sample_target_r<=LIDAR_PI_PAUSE?actuator_target_r:wheel_target_r;
    sample_braked<=LIDAR_PI_PAUSE?actual_bridge_braked:wheel_brake;
   end
   if(motion_publish[1])motion_diag<={sample_r,sample_l,motion_yaw_rate,heading_trim,
    sample_target_r,sample_target_l,motion_vr,motion_vl,heading_error,heading_goal,
    motion_innovation,motion_encoder_heading,motion_heading,motion_imu_yaw,motion_stamp,
    (motion_flags | (heading_holding?16'h0040:16'd0) | (sample_braked?16'h0200:16'd0)),16'd0};
  end
 wire [15:0] predictive_gain_cfg=PRED_YAW_GAIN_Q8;
 wire [7:0] predictive_alpha_cfg=(PRED_ALPHA_R_SHIFT<<4)|PRED_ALPHA_L_SHIFT;
 wire [7:0] predictive_slew_cfg=PRED_SLEW_COUNTS;
 wire [255:0] predictive_diag={predictive_slew_cfg,predictive_alpha_cfg,predictive_gain_cfg,predictive_end_error,predictive_cost,
  predictive_r,predictive_l,predictive_ref_r,predictive_ref_l,8'd8,{4'd0,predictive_selected},
  predictive_cycles,predictive_generation,predictive_time,predictive_flags,16'd0};
 // Preserve the 26-byte v1 wire layout; former rear bit/slot are reserved.
 wire [15:0] lidar_flags={8'd0,(LIDAR_USB_FORWARD!=0),1'b1,(LIDAR_CONTROL!=0),lidar_fresh,1'b0,lidar_valid};
 wire [15:0] lidar_front_zero=LIDAR_FRONT_ZERO_Q6;
 wire [159:0] lidar_diag={lidar_front_zero,lidar_errors,lidar_packets,16'hffff,lidar_right,
                        lidar_left,lidar_front,lidar_scan_id,lidar_flags,16'd0};
 // This display switch only enables the C9 transmitter. Motor gates are independent.
 localparam LIDAR_MAP_TELEMETRY=(LIDAR_MAP_DISPLAY!=0) ||
        (LIDAR_PLANNER_ENABLE!=0 && LIDAR_CONTROL!=0 &&
         LIDAR_ADVANCED_CONTROL!=0 && LIDAR_MODEL_VALIDATED!=0);
 wire [655:0] lidar_map_diag={lidar_map_ranges,4'b0000,lidar_map_known,lidar_map_age,
                             lidar_map_scan_id,7'b0000000,lidar_map_fresh};
 rb_telemetry #(.CLK_HZ(CLK_HZ),.REMOTE_FOLLOW(1),.REMOTE_NAV(1),.REMOTE_SPEED(1),.REMOTE_RECORD(1),.ENABLE_CAMERA_DIAG(1),.CAMERA_DIAG_V2(1),.CAMERA_DIAG_V3(1),.CAMERA_DIAG_V4(1),.ENABLE_MOTION_DIAG(1),.MOTION_HZ(50),.ENABLE_PREDICTIVE_DIAG(1),.ENABLE_LIDAR_DIAG(1),.ENABLE_LIDAR_PLANNING_DIAG(1),.ENABLE_LIDAR_MAP_DIAG(LIDAR_MAP_TELEMETRY)) telemetry(
  .clk(clk50m),.rst_n(rst_n),.state({4'd0,state}),.health(health),.command({5'd0,effective_command}),.speed(effective_speed),
  .dl(lok?dl:16'hffff),.dr(rok?dr:16'hffff),.vl(vl),.vr(vr),.roll(roll),.pitch(pitch),.yaw(yaw),.fault(fault),.tx(uart_tx),
  .nav_state({ir_stop_epoch,nav_state}),.nav_index(nav_index),.nav_count(cfg_count),.nav_flags(nav_flags),
  .nav_x(nav_x),.nav_y(nav_y),.nav_heading(nav_heading),.nav_ack_seq(cfg_ack_seq),.nav_ack_status(cfg_ack_status),.follow_speed(ir_follow_speed),.record_count(ir_record_count),
  .camera_diag(camera_diag),.follow_count(ir_follow_count),.nav_mode_count(ir_nav_mode_count),
  // Health is current even if acquisition stopped. Kinematic fields and their
  // timestamp remain the last coherent snapshot; host rejects a frozen clock.
  .motion_diag({motion_diag[319:32],(motion_flags | (heading_holding?16'h0040:16'd0) | (sample_braked?16'h0200:16'd0)),16'd0}),
  .predictive_diag(predictive_diag),.lidar_diag(lidar_diag),.lidar_planning_diag(lidar_planning_diag),
  .lidar_map_diag(lidar_map_diag));
endmodule
