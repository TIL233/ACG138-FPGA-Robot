`timescale 1ns/1ps
module tb_top_brake_pipeline;
 reg clk=0;always #5 clk=~clk;
 robot_top dut(.clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0));
 task braked;
 begin #1;
  if(!dut.motor_l_pwm || !dut.motor_l_in1 || !dut.motor_l_in2 ||
     !dut.motor_r_pwm || !dut.motor_r_in1 || !dut.motor_r_in2)
    $fatal(1,"raw stop/fault must short-brake bridges before next clock");
 end endtask
 initial begin
  repeat(3)@(negedge clk);
  force dut.rst_n=1;force dut.armed=1;force dut.link_ok=1;
  force dut.imu_ok=1;force dut.motion_ready=1;force dut.motion_fault=0;force dut.lok=1;force dut.rok=1;
  force dut.lfault=0;force dut.rfault=0;force dut.ir_valid=0;
  // Isolate the raw STOP/fault bypass from both staged brake causes.
  force dut.wheel_brake_hold=0;force dut.wheel_hard_brake_hold=0;
  force dut.wheel_target_l=8;force dut.wheel_target_r=8;
  force dut.left_wheel.duty=100;force dut.right_wheel.duty=100;
  force dut.left_wheel.previous_target=8;force dut.right_wheel.previous_target=8;
  #1;if(dut.motor_l_in2 || dut.motor_r_in2)$fatal(1,"baseline should not be braking");
  @(negedge clk);force dut.effective_command=5;force dut.effective_speed=8;force dut.command_update=1;
  force dut.control_dl=321;force dut.control_dr=654;
  force dut.lu=1;force dut.ru=0;
  @(posedge clk);#1;
  if(!dut.safety_armed_in || !dut.safety_update_in || dut.safety_command_in!=5 || dut.safety_speed_in!=8)
   $fatal(1,"arbitrated ARM/update/command/speed must enter safety as one coherent clock bundle");
  if(dut.safety_dl_in!=321 || dut.safety_dr_in!=654 ||
     !dut.safety_lok_in || !dut.safety_rok_in || !dut.safety_lu_in || dut.safety_ru_in)
   $fatal(1,"range values, validity and update pulses must enter with the same request bundle");
  @(negedge clk);release dut.control_dl;release dut.control_dr;release dut.lu;release dut.ru;
  @(negedge clk);release dut.effective_command;release dut.effective_speed;release dut.command_update;
  @(negedge clk);force dut.imu_ok=0;braked();force dut.imu_ok=1;
  @(negedge clk);force dut.motion_ready=0;braked();force dut.motion_ready=1;
  @(negedge clk);force dut.motion_fault=1;braked();force dut.motion_fault=0;
  @(negedge clk);force dut.link_ok=0;braked();force dut.link_ok=1;
  @(negedge clk);force dut.lok=0;#1;
  if(dut.wheel_brake)$fatal(1,"left sonar missing must not impose a raw motor brake");force dut.lok=1;
  @(negedge clk);force dut.rok=0;#1;
  if(dut.wheel_brake)$fatal(1,"right sonar missing must not impose a raw motor brake");force dut.rok=1;
  @(negedge clk);force dut.lfault=1;braked();force dut.lfault=0;
  @(negedge clk);force dut.predictive_fault=1;braked();force dut.predictive_fault=0;
  @(negedge clk);force dut.armed=0;braked();force dut.armed=1;
  @(negedge clk);force dut.ir_valid=1;force dut.ir_key=8'h1c;braked();
  @(posedge clk);#1;
  if(dut.safety_armed_in || dut.safety_command_in!=0 || !dut.safety_update_in)
   $fatal(1,"raw STOP brakes immediately and reaches scan FSM coherently on the next edge");
  @(negedge clk);force dut.ir_valid=0;
  @(negedge clk);force dut.raw_l=32'sd10;force dut.raw_r=-32'sd10;
  @(posedge clk);#1;if(dut.pos_l!=10 || dut.pos_r!=10)$fatal(1,"encoder sign pipeline");
  $display("PASS top brake pipeline: coherent command bundle, raw STOP/DISARM/IMU/link/fault brake, missing sonar allowed, encoder sign stage");$finish;
 end
 initial begin #100000;$fatal(1,"brake pipeline timeout");end
endmodule
