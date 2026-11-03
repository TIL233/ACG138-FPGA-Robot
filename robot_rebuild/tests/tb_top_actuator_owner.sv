`timescale 1ns/1ps
module tb_top_actuator_owner;
 reg clk=0;always #5 clk=~clk;
 reg scoped=1,follow=0,dynamic=1;
 robot_top #(.CLK_HZ(20000),.USE_PREDICTIVE(0),.LIDAR_CONTROL(1),
  .LIDAR_ADVANCED_CONTROL(1),.LIDAR_MODEL_VALIDATED(1)) dut(
  .clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),.lidar_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0));
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task bridge_stopped;begin #1;
  if(!(dut.motor_l_pwm && dut.motor_l_in1 && dut.motor_l_in2 &&
       dut.motor_r_pwm && dut.motor_r_in1 && dut.motor_r_in2))$fatal(1,"owner/raw veto leaked drive");
 end endtask
 initial begin
  force dut.rst_n=0;ticks(3);force dut.rst_n=1;
  force dut.armed=1;force dut.ir_stop_now=0;force dut.link_ok=1;force dut.motion_ok=1;
  force dut.stop_permission=1;force dut.predictive_fault=0;force dut.lfault=0;force dut.rfault=0;
  force dut.brake=0;force dut.lidar_follow_hold=0;force dut.lidar_plan_hold=0;
  force dut.lidar_dynamic_hold=0;force dut.lidar_context_guard=0;
  force dut.lidar_plan_scope=scoped;force dut.lidar_follow_limit=follow;
  force dut.lidar_dynamic_gate=dynamic;
  force dut.lidar_plan_l=12;force dut.lidar_plan_r=12;
  force dut.lidar_follow_l=6;force dut.lidar_follow_r=6;
  force dut.predictive_l=8;force dut.predictive_r=8;
  ticks(420);if(dut.motor_l_in1 && dut.motor_l_in2)$fatal(1,"initial packet never drove");
  // A changed packet may alter only one wheel. Neither bridge may resume
  // while the other wheel still holds its replacement calculation queued.
  @(negedge clk);force dut.lidar_plan_l=10;ticks(2);bridge_stopped();
  ticks(12);force dut.lidar_plan_l=12;ticks(16);
  // Both modes have dynamic_gate=1. A one-bit lidar owner cannot detect this.
  @(negedge clk);scoped=0;follow=1;#1;
  if(!dut.wheel_brake)$fatal(1,"follow/lattice owner switch was not revoked immediately");
  bridge_stopped();ticks(2);bridge_stopped();
  ticks(12);if(dut.left_wheel.target_hold!=6 || dut.right_wheel.target_hold!=6)
   $fatal(1,"follow target was not accepted coherently");
  // Owner returns before the first target has left all stages.
  @(negedge clk);scoped=1;follow=0;ticks(1);
  @(negedge clk);scoped=0;follow=1;bridge_stopped();ticks(3);bridge_stopped();
  ticks(12);if(dut.left_wheel.target_hold!=6)$fatal(1,"owner bounce accepted stale target");
  @(negedge clk);dynamic=0;follow=0;bridge_stopped();ticks(3);bridge_stopped();
  ticks(12);if(dut.left_wheel.target_hold!=8)$fatal(1,"legacy owner return missed predictive target");
  @(negedge clk);force dut.ir_stop_now=1;bridge_stopped();ticks(1);
  @(negedge clk);force dut.ir_stop_now=0;bridge_stopped();ticks(2);bridge_stopped();
  ticks(12);if((dut.motor_l_in1 && dut.motor_l_in2) || (dut.motor_r_in1 && dut.motor_r_in2))
   $fatal(1,"STOP return never acknowledged replacement duty");
  // Actual FOLLOW cmd3/speed4/front650 can legally produce (0,1). A stable
  // zero wheel short-brakes, but is an accepted packet, not a pending one.
  @(negedge clk);dynamic=1;follow=1;force dut.lidar_follow_l=0;force dut.lidar_follow_r=1;
  ticks(420);
  if(!(dut.motor_l_in1 && dut.motor_l_in2) || (dut.motor_r_in1 && dut.motor_r_in2))
   $fatal(1,"legal stable (0,1) packet permanently brakes moving wheel");
  @(negedge clk);force dut.lidar_follow_l=1;force dut.lidar_follow_r=0;ticks(2);bridge_stopped();
  ticks(420);
  if((dut.motor_l_in1 && dut.motor_l_in2) || !(dut.motor_r_in1 && dut.motor_r_in2))
   $fatal(1,"legal stable (1,0) packet permanently brakes moving wheel");
  @(negedge clk);force dut.lidar_follow_l=0;ticks(12);bridge_stopped();
  @(negedge clk);force dut.lidar_follow_l=6;force dut.lidar_follow_r=6;ticks(2);bridge_stopped();
  ticks(12);if(dut.motor_l_in1 && dut.motor_l_in2 || dut.motor_r_in1 && dut.motor_r_in2)
   $fatal(1,"zero pair could not accept a replacement nonzero packet");
  $display("PASS actuator TOP: owner packet, follow/lattice/legacy transitions, bounce, raw+queued STOP");$finish;
 end
 initial begin #200000;$fatal(1,"owner timeout");end
endmodule
