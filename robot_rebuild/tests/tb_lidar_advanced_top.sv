`timescale 1ns/1ps
`define ROBOT_PINS .clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),.lidar_rx(1'b1),.ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),.enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),.cam1_pclk(1'b0),.cam1_href(1'b0),.cam1_vsync(1'b0),.cam1_data(8'd0),.uart_tx(),.motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),.motor_r_pwm(),.motor_r_in1(),.motor_r_in2(),.cam1_xclk(),.cam1_pwdn(),.cam1_rst_n(),.cam1_scl(),.cam1_sda(),.rgmii_tx_clk(),.rgmii_tx_ctl(),.rgmii_txd()
module tb_lidar_advanced_top;
 reg clk=0;always #5 clk=~clk;
 reg arm=0;reg [2:0] cmd=0;reg [7:0] req=12;reg changed=1,follow=0,stop_now=0;
 reg map_live=1;reg [15:0] map_id=1;reg [575:0] ranges={36{16'd8000}};
 reg sonar_near=0;reg sample=0;integer i,diagnostic_file;
 robot_top #(.CLK_HZ(1000000),.USE_PREDICTIVE(0),.LIDAR_PLANNER_ENABLE(0)) baseline(`ROBOT_PINS);
 robot_top #(.CLK_HZ(1000000),.USE_PREDICTIVE(0)) shadow(`ROBOT_PINS);
 robot_top #(.CLK_HZ(1000000),.USE_PREDICTIVE(0),.LIDAR_CONTROL(1),
  .LIDAR_ADVANCED_CONTROL(1),.LIDAR_MODEL_VALIDATED(1)) active(`ROBOT_PINS);
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task common;
 begin
  force baseline.rst_n=0;force shadow.rst_n=0;force active.rst_n=0;ticks(5);
  force baseline.rst_n=1;force shadow.rst_n=1;force active.rst_n=1;
  force baseline.armed=arm;force shadow.armed=arm;force active.armed=arm;
  force baseline.command=cmd;force shadow.command=cmd;force active.command=cmd;
  force baseline.speed=req;force shadow.speed=req;force active.speed=req;
  force baseline.command_update=changed;force shadow.command_update=changed;force active.command_update=changed;
  force baseline.link_ok=1;force shadow.link_ok=1;force active.link_ok=1;
  force baseline.motion_ok=1;force shadow.motion_ok=1;force active.motion_ok=1;
  force baseline.motion_encoder_heading=0;force shadow.motion_encoder_heading=0;force active.motion_encoder_heading=0;
  force baseline.motion_heading=0;force shadow.motion_heading=0;force active.motion_heading=0;
  force baseline.motion_vl=0;force shadow.motion_vl=0;force active.motion_vl=0;
  force baseline.motion_vr=0;force shadow.motion_vr=0;force active.motion_vr=0;
  force baseline.motion_update=sample;force shadow.motion_update=sample;force active.motion_update=sample;
  force baseline.nav_selected=0;force shadow.nav_selected=0;force active.nav_selected=0;
  force baseline.follow_selected=follow;force shadow.follow_selected=follow;force active.follow_selected=follow;
  force baseline.ir_stop_now=stop_now;force shadow.ir_stop_now=stop_now;force active.ir_stop_now=stop_now;
  force baseline.lfault=0;force shadow.lfault=0;force active.lfault=0;
  force baseline.rfault=0;force shadow.rfault=0;force active.rfault=0;
  force baseline.predictive_fault=0;force shadow.predictive_fault=0;force active.predictive_fault=0;
  force baseline.lok=sonar_near;force shadow.lok=sonar_near;force active.lok=sonar_near;
  force baseline.rok=0;force shadow.rok=0;force active.rok=0;
  force baseline.dl=150;force shadow.dl=150;force active.dl=150;
  force baseline.dr=16'hfffe;force shadow.dr=16'hfffe;force active.dr=16'hfffe;
  force baseline.lidar_fresh=map_live;force shadow.lidar_fresh=map_live;force active.lidar_fresh=map_live;
  force baseline.lidar_valid=3'b111;force shadow.lidar_valid=3'b111;force active.lidar_valid=3'b111;
  force baseline.lidar_front=1000;force shadow.lidar_front=1000;force active.lidar_front=1000;
  force baseline.lidar_map_fresh=map_live;force shadow.lidar_map_fresh=map_live;force active.lidar_map_fresh=map_live;
  force baseline.lidar_map_ranges=ranges;force shadow.lidar_map_ranges=ranges;force active.lidar_map_ranges=ranges;
  force baseline.lidar_map_known={36{1'b1}};force shadow.lidar_map_known={36{1'b1}};force active.lidar_map_known={36{1'b1}};
  force baseline.lidar_map_scan_id=map_id;force shadow.lidar_map_scan_id=map_id;force active.lidar_map_scan_id=map_id;
  force baseline.lidar_map_age=90;force shadow.lidar_map_age=90;force active.lidar_map_age=90;
 end endtask
 task calculate;begin sample=1;ticks(1);sample=0;ticks(1100);end endtask
 always @(posedge clk)if(baseline.rst_n && shadow.rst_n)begin
  if(baseline.wheel_brake!==shadow.wheel_brake || baseline.wheel_target_l!==shadow.wheel_target_l ||
     baseline.wheel_target_r!==shadow.wheel_target_r || baseline.state!==shadow.state)
   $fatal(1,"shadow changed legacy actuator behavior");
 end
 initial begin
  diagnostic_file=$fopen("planning_diagnostics.hex","w");
  common();ticks(12);changed=0;arm=1;cmd=1;changed=1;ticks(4);changed=0;
  calculate();
  if(!active.lidar_plan_current || active.lidar_plan_selected!=7 || !active.lidar_plan_takeover)
   $fatal(1,"calibrated clear-space plan not selected");
  if(active.wheel_brake || active.wheel_target_l!=12 || active.wheel_target_r!=12)
   $fatal(1,"planner not in actual target path");
  if(shadow.lidar_plan_takeover || shadow.lidar_dynamic_gate)$fatal(1,"shadow claimed takeover");
  $fwrite(diagnostic_file,"drive %080h\n",active.lidar_planning_diag);
  $fwrite(diagnostic_file,"observe %080h\n",shadow.lidar_planning_diag);
  // Missing map in explicitly enabled advanced mode holds, without a latched fault.
  map_live=0;#1;if(!active.wheel_brake)$fatal(1,"expired map did not brake immediately");
  ticks(5);if(active.fault!=0)$fatal(1,"temporary map loss latched vehicle fault");
  map_live=1;map_id=2;calculate();
  if(active.wheel_brake || !active.lidar_plan_current)$fatal(1,"map recovery failed");
  // Sonar near evidence cannot be hidden by clear laser geometry.
  sonar_near=1;calculate();
  if(!active.wheel_brake || !active.lidar_dynamic_hold)$fatal(1,"dynamic fusion lost sonar near stop");
  $fwrite(diagnostic_file,"hold %080h\n",active.lidar_planning_diag);
  sonar_near=0;
  // Physical remote STOP bypasses all planning/predictive arithmetic.
  stop_now=1;#1;
  if(!active.wheel_brake || !shadow.wheel_brake || !baseline.wheel_brake)$fatal(1,"raw STOP lost priority");
  ticks(2);stop_now=0;follow=1;calculate();
  if(active.lidar_plan_takeover || active.lidar_plan_scope)$fatal(1,"follow entered autonomous detour");
  $fwrite(diagnostic_file,"follow %080h\n",active.lidar_planning_diag);
  arm=0;cmd=0;changed=1;ticks(5);
  if(!active.wheel_brake || active.lidar_plan_takeover)$fatal(1,"DISARM left planner driving");
  $fclose(diagnostic_file);
  $display("PASS advanced lidar top: shadow equivalent, calibrated targets, missing-map hold/recovery, sonar near, STOP and follow exclusion");$finish;
 end
 initial begin #3000000;$fatal(1,"advanced lidar top timeout");end
endmodule
`undef ROBOT_PINS
