`timescale 1ns/1ps
module tb_navigation_heading_source_switch;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,nav=0,update=0,stop=0;
 reg signed [15:0] imu=1000,encoder=9000;
 robot_top dut(.clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),
  .uart_tx(),.motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),
  .motor_r_pwm(),.motor_r_in1(),.motor_r_in2(),
  .cam1_pclk(1'b0),.cam1_href(1'b0),.cam1_vsync(1'b0),.cam1_data(8'd0),
  .cam1_xclk(),.cam1_pwdn(),.cam1_rst_n(),.cam1_scl(),.cam1_sda(),
  .rgmii_tx_clk(),.rgmii_tx_ctl(),.rgmii_txd());
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit v,input string m);if(!v)$fatal(1,"%s goal=%0d err=%0d trim=%0d",m,dut.heading_goal,dut.heading_error,dut.heading_trim);endtask
 // Match the real producer: heading is published in phase4, update in phase5.
 task sample;begin cycles(1);@(negedge clk);update=1;cycles(1);@(negedge clk);update=0;cycles(1);end endtask
 initial begin
  force dut.rst_n=rst_n;force dut.nav_selected=nav;force dut.nav_run=0;
  force dut.armed=1;force dut.ir_stop_now=stop;
  force dut.motion_ok=1;force dut.motion_update=update;
  force dut.motion_heading=imu;force dut.motion_encoder_heading=encoder;
  force dut.safety_brake_stage=0;
  force dut.safety_target_l=16'sd8;force dut.safety_target_r=16'sd8;
  cycles(3);rst_n=1;cycles(2);sample;
  check(dut.heading_holding && dut.heading_goal==1000,"manual mode anchors IMU");
  @(negedge clk);nav=1;update=1;cycles(1);update=0;
  check(!dut.heading_holding && dut.heading_trim==0,"coincident mode switch and feedback clear old source anchor");
  sample;check(dut.heading_goal==9000 && dut.heading_holding,"navigation reacquires encoder anchor");
  encoder=9300;sample;
  check(dut.heading_error==-300 && dut.heading_trim==-1,"navigation straight correction follows encoder changes");
  @(negedge clk);nav=0;cycles(1);
  check(!dut.heading_holding && dut.heading_trim==0,"manual takeover clears encoder anchor without waiting for sample");
  sample;check(dut.heading_goal==1000 && dut.heading_trim==0,"manual takeover reacquires IMU anchor without inherited trim");
  imu=400;sample;check(dut.heading_trim==2,"manual IMU straight correction still works");
  @(negedge clk);nav=1;stop=1;#1;
  check(dut.heading_target_l==8 && dut.heading_target_r==8,"STOP immediately bypasses outer-loop correction");
  cycles(1);check(!dut.heading_holding,"STOP takes priority over navigation source switch");
  $display("PASS navigation heading-source switching: manual/nav/manual/coincident sample/reanchor/STOP");$finish;
 end
 initial begin #100000;$fatal(1,"heading source switch timeout");end
endmodule
