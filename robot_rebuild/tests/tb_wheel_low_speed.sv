`timescale 1ns/1ps
// Legal remote/follow speeds4/6 retain the same bounded zero-feedback stall
// protection as speed8, including either requested direction.
module tb_wheel_low_speed;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,brake=1,clear_fault=0;
 reg signed [15:0] target=0;
 reg signed [31:0] position=0;
 wire fault,pwm,in1,in2;wire signed [15:0] velocity;
 rb_wheel_control #(.CLK_HZ(20000),.MAX_DUTY(1),.SLEW(1)) dut(
  .clk(clk),.rst_n(rst_n),.brake(brake),.clear_fault(clear_fault),
  .target(target),.position(position),.velocity(velocity),.fault(fault),
  .pwm(pwm),.in1(in1),.in2(in2),.planner_pause(1'b0),.actuator_pending());
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit good,input string message);
  if(!good)$fatal(1,"%s target=%0d position=%0d velocity=%0d fault=%b stall=%0d",message,
   target,position,velocity,fault,dut.stall);
 endtask
 task start(input integer requested);
  begin
   @(negedge clk);rst_n=0;brake=1;clear_fault=0;target=0;position=0;cycles(3);
   @(negedge clk);rst_n=1;cycles(3);
   @(negedge clk);brake=0;target=requested;
  end
 endtask
 task stalled(input integer requested);
  begin
   start(requested);cycles(400*49);
   check(!fault,"low-speed stall retains the original one-second allowance");
   cycles(400);
   check(fault && pwm && in1 && in2,"zero feedback at legal low speed faults and brakes within one second");
   cycles(800);check(fault,"stall fault remains latched");
   @(negedge clk);target=0;brake=1;clear_fault=1;cycles(2);
   @(negedge clk);clear_fault=0;cycles(800);
   check(!fault && dut.duty==0 && pwm && in1 && in2,
    "explicit stopped clear recovers without restarting the wheel");
  end
 endtask
 task moving(input integer requested);
  integer direction;
  begin
   start(requested);direction=requested>0?1:-1;
   for(integer k=0;k<100;k=k+1)begin
    @(negedge clk);position=position+direction;cycles(400);
    check(!fault,"healthy low-speed counts remain fault-free beyond two stall windows");
   end
   @(negedge clk);brake=1;target=0;#1;
   check(pwm && in1 && in2,"STOP brakes healthy low-speed motion immediately");
   cycles(800);check(!fault && dut.duty==0,"STOP clears drive and does not restart");
  end
 endtask
 initial begin
  stalled(4);stalled(-4);stalled(6);stalled(-6);
  moving(4);moving(-4);moving(6);moving(-6);
  start(4);
  for(integer k=0;k<60;k=k+1)begin
   @(negedge clk);position=(k%2)?0:-1;cycles(400);
   check(!fault,"one-count feedback jitter retains its existing non-direction-fault behavior");
  end
  // Opposite counts retain the existing200ms start guard and100ms persistence.
  start(4);
  repeat(3)begin @(negedge clk);position=position-2;cycles(400);end
  check(!fault,"brief opposite startup feedback remains tolerated at low speed");
  repeat(13)begin @(negedge clk);position=position-2;cycles(400);end
  check(fault && in1 && in2,"persistent opposite low-speed feedback retains its original fault");
  $display("PASS low-speed wheel protection: +/-4/+/-6 stall timeout, healthy counts, STOP, explicit clear and direction guard");$finish;
 end
 initial begin #4000000;$fatal(1,"low-speed wheel protection test hung");end
endmodule
