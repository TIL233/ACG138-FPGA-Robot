`timescale 1ns/1ps
module tb_wheel;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,brake=1,clear=0;reg signed [15:0] target=0;
 reg signed [31:0] position=0;wire signed [15:0] velocity;wire fault,pwm,in1,in2,actuator_pending;
 rb_wheel_control #(.CLK_HZ(20000),.MAX_DUTY(1),.SLEW(1)) dut(
 clk,rst_n,brake,clear,target,position,velocity,fault,pwm,in1,in2,1'b0,actuator_pending);
 task cycles(input integer n);repeat(n) begin @(posedge clk);#1;end endtask
 task check(input bit yes,input string msg);if(!yes) $fatal(1,"%s",msg);endtask
 initial begin
  cycles(3);rst_n=1;cycles(3);check(in1&&in2&&pwm,"reset release brakes");
  brake=0;target=16;cycles(410);check(in1&&!in2,"positive target direction");
  target=-16;#1;check(in1&&in2&&pwm,"target reversal brakes before next PI sample");
  cycles(410);check(dut.duty==0,"reversal must cross zero");
  cycles(400);check(!in1&&in2,"negative target direction");
  brake=1;#1;check(in1&&in2&&pwm,"brake overrides previous negative drive immediately");
  cycles(2);check(dut.integral==0 && dut.duty==0,"stop clears PI and ramp");
  brake=0;target=16;cycles(21000);check(fault && in1 && in2,"no encoder movement latches fault");
  clear=1;brake=1;cycles(2);clear=0;cycles(2);check(!fault,"explicit clear recovers");
  // Incorrect encoder polarity must not look like healthy moving feedback:
  // negative counts during forward drive otherwise increase PI torque forever.
  brake=0;target=16;
  repeat(20) begin position=position-3;cycles(400);end
  check(fault && in1 && in2,"persistent reverse encoder feedback stops drive");
  clear=1;brake=1;cycles(2);clear=0;cycles(2);check(!fault,"reverse feedback clear");
  // A few reverse counts immediately after direction reversal can be inertia.
  // They must not fault before the initial direction guard has expired.
  target=-16;brake=0;
  repeat(3) begin position=position+3;cycles(400);end
  check(!fault,"brief old-direction coast is tolerated");
  repeat(12) begin position=position-3;cycles(400);end
  check(!fault && !in1 && in2,"correct direction feedback remains healthy");
  $display("PASS wheel: direction/reversal/braking/stall/clear");$finish;
 end
 initial begin #1000000;$fatal(1,"test hung");end
endmodule
