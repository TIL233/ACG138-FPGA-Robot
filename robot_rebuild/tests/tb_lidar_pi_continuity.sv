`timescale 1ns/1ps
module tb_lidar_pi_continuity;
 reg clk=0;always #500 clk=~clk;
 reg arm=0,changed=1,follow=0,stop_now=0;
 reg [2:0] cmd=0;reg [15:0] front=1000;
 reg signed [31:0] encoder_position=0;
 reg signed [15:0] navigation_goal=0;
 reg [35:0] map_known={36{1'b1}};reg [15:0] map_id=1;
 integer i,j,directed;
 reg check_continuity=0;
 integer saved_integral_l,saved_integral_r,minimum_duty_l,minimum_duty_r;
 robot_top #(.CLK_HZ(1000000),.LIDAR_CONTROL(1),
  .LIDAR_ADVANCED_CONTROL(1),.LIDAR_MODEL_VALIDATED(1)) dut(
  .clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),.lidar_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),
  .cam1_pclk(1'b0),.cam1_href(1'b0),.cam1_vsync(1'b0),.cam1_data(8'd0),
  .uart_tx(),.motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),
  .motor_r_pwm(),.motor_r_in1(),.motor_r_in2(),.cam1_xclk(),.cam1_pwdn(),
  .cam1_rst_n(),.cam1_scl(),.cam1_sda(),.rgmii_tx_clk(),.rgmii_tx_ctl(),.rgmii_txd());
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task real_sample;begin @(posedge dut.motion_update);#1;end endtask
 task arm_continuity(input integer next_l,input integer next_r,input integer maximum_feedback);
  integer minimum_target_l,minimum_target_r,floor_l,floor_r;
 begin
  saved_integral_l=dut.left_wheel.integral;saved_integral_r=dut.right_wheel.integral;
  minimum_target_l=dut.wheel_target_l<next_l?dut.wheel_target_l:next_l;
  minimum_target_r=dut.wheel_target_r<next_r?dut.wheel_target_r:next_r;
  if(minimum_target_l<maximum_feedback || minimum_target_r<maximum_feedback ||
     saved_integral_l<0 || saved_integral_r<0)
   $fatal(1,"continuity fixture requires nonnegative old/new PI errors");
  // All old/new errors in these scenarios are nonnegative. Thus a regular
  // update cannot decrease I, and pause/resume must retain it. FF + KP*error
  // + saved I bounds the raw duty below. SLEW moves duty toward that value,
  // never below min(previous duty, raw lower bound), regardless of whether
  // one normal update and/or a resume update occurs in this interval.
  floor_l=minimum_target_l*dut.left_wheel.FF+
          (minimum_target_l-maximum_feedback)*dut.left_wheel.KP+saved_integral_l;
  floor_r=minimum_target_r*dut.right_wheel.FF+
          (minimum_target_r-maximum_feedback)*dut.right_wheel.KP+saved_integral_r;
  if(floor_l>dut.left_wheel.MAX_DUTY)floor_l=dut.left_wheel.MAX_DUTY;
  if(floor_r>dut.right_wheel.MAX_DUTY)floor_r=dut.right_wheel.MAX_DUTY;
  minimum_duty_l=dut.left_wheel.duty<floor_l?dut.left_wheel.duty:floor_l;
  minimum_duty_r=dut.right_wheel.duty<floor_r?dut.right_wheel.duty:floor_r;
  check_continuity=1;
 end endtask
 task inspect_drive(input integer requested_l,input integer requested_r);
 begin
  ticks(1000);
  $display("PI cmd=%0d follow=%0d wheel=%0d,%0d duty=%0d,%0d brake=%0d timers=%0d,%0d",
   cmd,follow,dut.wheel_target_l,dut.wheel_target_r,dut.left_wheel.duty,dut.right_wheel.duty,
   dut.wheel_brake,dut.motion_feedback.timer,dut.left_wheel.timer);
  if(dut.wheel_brake || dut.wheel_target_l!=requested_l || dut.wheel_target_r!=requested_r)
   $fatal(1,"valid planning target did not reach wheel input");
  if(dut.left_wheel.target_hold!=requested_l || dut.right_wheel.target_hold!=requested_r)
   $fatal(1,"PI retained a superseded target after planning hold release");
  if(dut.left_wheel.duty<=0 || dut.right_wheel.duty<=0)
   $fatal(1,"periodic envelope recompute cleared real PI drive after hold release");
  if(check_continuity)begin
   $display("CONTINUITY minimum=%0d,%0d duty=%0d,%0d previous_I=%0d,%0d current_I=%0d,%0d SLEW=%0d",
    minimum_duty_l,minimum_duty_r,dut.left_wheel.duty,dut.right_wheel.duty,
    saved_integral_l,saved_integral_r,dut.left_wheel.integral,dut.right_wheel.integral,dut.left_wheel.SLEW);
   if(dut.left_wheel.integral<saved_integral_l || dut.right_wheel.integral<saved_integral_r ||
      dut.left_wheel.duty<minimum_duty_l || dut.right_wheel.duty<minimum_duty_r)
    $fatal(1,"bounded planning update reset accumulated PI state or violated the slew lower bound");
   check_continuity=0;
  end
  directed=0;
  for(j=0;j<5000;j=j+1)begin
   ticks(1);
   if(dut.motor_l_pwm && dut.motor_l_in1 && !dut.motor_l_in2 &&
      dut.motor_r_pwm && dut.motor_r_in1 && !dut.motor_r_in2)directed=directed+1;
  end
  if(directed<100)$fatal(1,"nonzero target lacked sustained directional PWM");
 end endtask
 task check_period(input integer requested);
 begin real_sample();inspect_drive(requested,requested);end endtask
 initial begin
  force dut.rst_n=0;ticks(5);force dut.rst_n=1;
  force dut.armed=arm;force dut.command=cmd;force dut.speed=12;force dut.command_update=changed;
  force dut.link_ok=1;force dut.nav_selected=0;force dut.follow_selected=follow;
  force dut.ir_stop_now=stop_now;
  // Feed legal stationary IMU data. Motion producer, wheel PI, their timers,
  // feedback pulses, predictive solver and wheel faults all run unforced.
  force dut.imu_ok=1;force dut.imu_update=1;force dut.yaw=0;
  // Inject the encoder-count boundary, retaining the real shared position
  // staging and velocity calculation; right encoder polarity is inverted.
  force dut.raw_l=encoder_position;force dut.raw_r=-encoder_position;
  force dut.lok=0;force dut.rok=0;force dut.dl=16'hfffe;force dut.dr=16'hfffe;
  force dut.lidar_fresh=1;force dut.lidar_valid=3'b111;force dut.lidar_front=front;
  force dut.lidar_map_fresh=1;force dut.lidar_map_update=0;
  force dut.lidar_map_ranges={36{16'd8000}};force dut.lidar_map_known=map_known;
  force dut.lidar_map_scan_id=map_id;force dut.lidar_map_age=90;
  ticks(5);changed=0;wait(dut.motion_ok);ticks(5);
  @(negedge clk);arm=1;cmd=1;changed=1;ticks(2);changed=0;
  real_sample();
  for(i=0;i<4;i=i+1)check_period(12);
  // Changing measured counts invalidates lattice context even though the
  // envelope maximum remains bounded by the 12-count request.
  for(i=0;i<4;i=i+1)begin
   arm_continuity(12,12,4);
   encoder_position=encoder_position+((i%2)==0?2:4);
   check_period(12);
   if(dut.motion_vl!=((i%2)==0?2:4) || dut.motion_vr!=dut.motion_vl)
    $fatal(1,"moving encoder stimulus failed to reach real motion feedback");
  end
  // Fresh clear-range changes also require a bounded recheck, not a hard
  // reset of PI state. Both ranges are beyond the computed slow threshold.
  real_sample();
  for(i=0;i<4;i=i+1)begin
   real_sample();arm_continuity(12,12,4);
   front=(i%2)==0?1001:1000;
   inspect_drive(12,12);
  end
  // Inject stored navigation intent/readiness, keeping its real run, health,
  // reset and safety permission gates live. Waypoint math is tested separately.
  force dut.nav_selected=1;
  force dut.navigation.desired_command=1;force dut.navigation.desired_speed=12;
  force dut.navigation.active=1;force dut.navigation.failed=0;
  force dut.navigation.pose_ready=1;force dut.navigation.route_ready=1;
  force dut.nav_goal_valid=1;force dut.nav_goal_error=navigation_goal;
  real_sample();
  for(i=0;i<4;i=i+1)begin
   real_sample();
   arm_continuity((i%2)==0?6:12,(i%2)==0?12:6,4);
   navigation_goal=(i%2)==0?16'sd9000:-16'sd9000;
   inspect_drive((i%2)==0?6:12,(i%2)==0?12:6);
   if(dut.lidar_control.goal_cd!=navigation_goal)
    $fatal(1,"navigation goal stimulus did not reach local planner");
  end
  force dut.nav_selected=0;
  release dut.navigation.desired_command;release dut.navigation.desired_speed;
  release dut.navigation.active;release dut.navigation.failed;
  release dut.navigation.pose_ready;release dut.navigation.route_ready;
  release dut.nav_goal_valid;release dut.nav_goal_error;
  map_known=0;map_id=2;#1;
  if(!dut.wheel_brake)$fatal(1,"unknown map did not brake immediately");
  ticks(1100);
  if(!dut.wheel_brake || dut.left_wheel.duty!=0 || dut.right_wheel.duty!=0)
   $fatal(1,"long unknown-map hold retained old PI drive state");
  map_known={36{1'b1}};map_id=3;real_sample();check_period(12);
  // Same real feedback phase must also preserve FOLLOW's checked speed cap.
  follow=1;front=700;real_sample();
  for(i=0;i<4;i=i+1)check_period(6);
  front=600;#1;
  if(!dut.wheel_brake || !dut.motor_l_in1 || !dut.motor_l_in2 || !dut.motor_l_pwm)
   $fatal(1,"valid near range lost immediate short brake");
  ticks(1100);
  if(dut.left_wheel.duty!=0 || dut.right_wheel.duty!=0)
   $fatal(1,"long near-range hold retained old PI drive state");
  front=700;real_sample();check_period(6);
  stop_now=1;#1;if(!dut.wheel_brake)$fatal(1,"STOP lost immediate priority");
  ticks(5);
  if(dut.left_wheel.duty!=0 || dut.right_wheel.duty!=0)
   $fatal(1,"STOP did not clear PI drive state immediately");
  stop_now=0;changed=1;ticks(2);changed=0;real_sample();check_period(6);
  // Encoders remain stationary deliberately: repeated unchanged range samples
  // must not erase the original 50-window stall protection.
  repeat(55)real_sample();ticks(10);
  if(!dut.lfault || !dut.rfault || !dut.wheel_brake)
   $fatal(1,"periodic planning reset suppressed original wheel stall protection");
  $display("PASS lidar real PI: native timer phase, changing measured speed/navigation goals, sustained planner/FOLLOW PWM, near/STOP and persistent stall protection");
  $finish;
 end
 initial begin #2000000000;$fatal(1,"real PI continuity timeout");end
endmodule
