`timescale 1ns/1ps
`define ROBOT_PINS .clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),.lidar_rx(1'b1),.ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),.enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),.cam1_pclk(1'b0),.cam1_href(1'b0),.cam1_vsync(1'b0),.cam1_data(8'd0),.uart_tx(),.motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),.motor_r_pwm(),.motor_r_in1(),.motor_r_in2(),.cam1_xclk(),.cam1_pwdn(),.cam1_rst_n(),.cam1_scl(),.cam1_sda(),.rgmii_tx_clk(),.rgmii_tx_ctl(),.rgmii_txd()
module tb_lidar_predictive_top;
 // Real 1 MHz test clock; feedback pulses are exactly 20 ms apart.
 reg clk=0;always #500 clk=~clk;
 reg arm=0;reg [2:0] cmd=0;reg [7:0] req=12;
 reg check_exit=0;
 reg changed=1,follow=0,stop_now=0,sample=0,map_live=1;
 reg [15:0] map_id=1,front=1000;reg [35:0] known={36{1'b1}};
 reg [575:0] ranges={36{16'd8000}};integer i;
 // USE_PREDICTIVE intentionally retains its production default (1).
 robot_top #(.CLK_HZ(1000000),.LIDAR_CONTROL(1),
  .LIDAR_ADVANCED_CONTROL(1),.LIDAR_MODEL_VALIDATED(1)) dut(`ROBOT_PINS);
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task feedback_period(input integer expected_l,input integer expected_r,input integer stopped);
 begin
  @(negedge clk);sample=1;ticks(1);@(negedge clk);sample=0;ticks(1000);
  $display("PERIOD cmd=%0d follow=%0d plan=%0d gate=%0d brake=%0d wheels=%0d,%0d predictive=%0d,%0d valid=%0d",
   cmd,follow,dut.lidar_plan_current,dut.lidar_dynamic_gate,dut.wheel_brake,
   dut.wheel_target_l,dut.wheel_target_r,dut.predictive_l,dut.predictive_r,dut.predictive_valid);
  $display("STATE safety=%0d fault=%0d pred_fault=%0d armed=%0d raw_brake=%0d env=%0d hold=%0d",dut.state,dut.fault,dut.predictive_fault,dut.armed,dut.brake,dut.lidar_control.env_valid,dut.lidar_plan_hold);
  if(stopped)begin
   if(!dut.wheel_brake)$fatal(1,"protection failed during repeated feedback");
  end else begin
   if(dut.wheel_brake || dut.wheel_target_l!=expected_l || dut.wheel_target_r!=expected_r)
    $fatal(1,"default predictive path blocked or changed advanced wheel targets");
  end
  if(dut.predictive_fault)$fatal(1,"unexpected real predictive fault");
  ticks(18999);
 end endtask
 task command_change(input [2:0] next_cmd);
 begin @(negedge clk);cmd=next_cmd;changed=1;ticks(1);@(negedge clk);changed=0;ticks(10);end endtask
 always @(posedge clk)begin
  #1;
  if(check_exit && !dut.wheel_brake && (dut.wheel_target_l>0 || dut.wheel_target_r>0))
   $fatal(1,"old positive target survived ownership transition");
 end
 initial begin
  force dut.rst_n=0;ticks(5);force dut.rst_n=1;
  force dut.armed=arm;force dut.command=cmd;force dut.speed=req;force dut.command_update=changed;
  force dut.link_ok=1;force dut.motion_ok=1;
  force dut.motion_encoder_heading=0;force dut.motion_heading=0;
  force dut.motion_vl=0;force dut.motion_vr=0;force dut.motion_update=sample;
  force dut.nav_selected=0;force dut.follow_selected=follow;force dut.ir_stop_now=stop_now;
  force dut.lfault=0;force dut.rfault=0;
  // No predictive outputs, validity, state or faults are forced.
  force dut.lok=0;force dut.rok=0;force dut.dl=16'hfffe;force dut.dr=16'hfffe;
  force dut.lidar_fresh=map_live;force dut.lidar_valid=3'b111;force dut.lidar_front=front;
  force dut.lidar_map_fresh=map_live;force dut.lidar_map_update=0;
  force dut.lidar_map_ranges=ranges;force dut.lidar_map_known=known;
  force dut.lidar_map_scan_id=map_id;force dut.lidar_map_age=90;
  ticks(10);arm=1;command_change(1);
  if(!$test$plusargs("FOLLOW_ONLY"))begin
   for(i=0;i<4;i=i+1)feedback_period(12,12,0);
   // Planner command context is sampled as a bundle, but its raw change
   // already short-brakes the wheel path before that sampling edge.
   @(negedge clk);req=6;#1;
   if(dut.lidar_control.request_speed!=12 || !dut.wheel_brake)
    $fatal(1,"planner command lacks registered boundary or immediate stale-target brake");
   ticks(1);if(dut.lidar_control.request_speed!=6)$fatal(1,"planner request sample missing");
   @(negedge clk);req=12;feedback_period(12,12,0);
   // Inject an existing heading correction at the ownership boundary; the
   // checked geometric candidate must remain 12/12, never become 9/15.
   force dut.heading_control.holding=1;force dut.heading_control.trim=3;ticks(5);
   if(dut.heading_target_l==12 && dut.heading_target_r==12)$fatal(1,"heading correction stimulus ineffective");
   if(dut.wheel_brake || dut.wheel_target_l!=12 || dut.wheel_target_r!=12)
    $fatal(1,"heading trim changed an already checked lattice candidate");
   release dut.heading_control.holding;release dut.heading_control.trim;
   // A fresh snapshot with unknown sectors must never authorize motion.
   known=0;map_id=2;for(i=0;i<2;i=i+1)feedback_period(0,0,1);
   known={36{1'b1}};map_id=3;feedback_period(12,12,0);
   front=150;#1;if(!dut.wheel_brake)$fatal(1,"near evidence did not brake immediately");
   feedback_period(0,0,1);front=1000;feedback_period(12,12,0);
   stop_now=1;#1;if(!dut.wheel_brake)$fatal(1,"raw STOP did not brake immediately");
   ticks(10);stop_now=0;command_change(1);feedback_period(12,12,0);
  end
  // FOLLOW has a 600 mm floor: 700 mm yields half of the 12-count request.
  follow=1;front=700;ticks(10);command_change(1);
  for(i=0;i<4;i=i+1)feedback_period(6,6,0);
  force dut.heading_control.holding=1;force dut.heading_control.trim=1;ticks(5);
  if(dut.heading_target_l==6 && dut.heading_target_r==6)$fatal(1,"follow heading stimulus ineffective");
  if(dut.wheel_brake || dut.wheel_target_l!=6 || dut.wheel_target_r!=6)
   $fatal(1,"heading trim exceeded the checked FOLLOW cap");
  release dut.heading_control.holding;release dut.heading_control.trim;
  front=600;#1;if(!dut.wheel_brake)$fatal(1,"follow near did not brake immediately");
  feedback_period(0,0,1);front=700;feedback_period(6,6,0);
  // Leaving ownership for reverse must not reuse advanced positive torque.
  follow=0;front=1000;check_exit=1;command_change(2);
  for(i=0;i<20;i=i+1)begin ticks(1);
   if(!dut.wheel_brake && (dut.wheel_target_l>0 || dut.wheel_target_r>0))
    $fatal(1,"old positive target survived ownership exit");
  end
  @(negedge clk);sample=1;ticks(1);@(negedge clk);sample=0;ticks(1000);
  if(dut.lidar_dynamic_gate || dut.wheel_brake || !dut.predictive_valid ||
     dut.wheel_target_l>=0 || dut.wheel_target_r>=0 ||
     dut.wheel_target_l!=dut.predictive_l || dut.wheel_target_r!=dut.predictive_r)
   $fatal(1,"legacy real predictive path did not recover after ownership exit");
  // Reentry must discard the old reverse trajectory before forward release.
  check_exit=0;command_change(1);feedback_period(12,12,0);
  arm=0;#1;if(!dut.wheel_brake)$fatal(1,"DISARM did not brake immediately");
  $display("PASS default predictive top: repeated 20ms planner/FOLLOW targets, unknown/near/STOP/DISARM, ownership exit/reentry and real legacy prediction");$finish;
 end
 initial begin #500000000;$fatal(1,"predictive top timeout");end
endmodule
`undef ROBOT_PINS





