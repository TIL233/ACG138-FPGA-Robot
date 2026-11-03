`timescale 1ns/1ps
module tb_wheel_pipeline;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,brake=1;reg signed [15:0] target=0;
 reg signed [31:0] position=0;
 wire signed [15:0] velocity;wire fault,pwm,in1,in2,actuator_pending;
 rb_wheel_control #(.CLK_HZ(1000000),.MAX_DUTY(40),.SLEW(4)) dut(
  clk,rst_n,brake,1'b0,target,position,velocity,fault,pwm,in1,in2,1'b0,actuator_pending);
 task cycles(input integer n);repeat(n) begin @(posedge clk);#1;end endtask
 task check(input bit yes,input string msg);if(!yes) $fatal(1,"%s",msg);endtask
 integer n,p,feedback,ref_i=0,ref_d=0,prior=0,err,sum,raw,want;
 initial begin
  cycles(3);@(negedge clk);rst_n=1;brake=0;
  for(n=0;n<18;n=n+1) begin
   @(negedge clk);target=(n>=6 && n<12)?-16:16;
   feedback=(n%3==0)?0:((n%3==1)?4:32);
   if(target<0) feedback=-feedback;
   while(dut.timer!=19999) @(negedge clk);
   position=position+feedback;
   if(prior!=0 && ((prior<0)!=(target<0))) begin ref_i=0;ref_d=0;end
   else begin
    err=target-feedback;sum=ref_i+2*err;
    ref_i=(sum>1000)?1000:((sum< -1000)?-1000:sum);
    raw=16*target+8*err+ref_i;
    if(target>0) want=(raw<0)?0:((raw>40)?40:raw);
    else want=(raw>0)?0:((raw< -40)?-40:raw);
    ref_d=(want>ref_d+4)?ref_d+4:((want<ref_d-4)?ref_d-4:want);
   end
   prior=target;
   cycles(5);
   check(dut.duty==ref_d && dut.integral==ref_i,"staged PI differs from sampled control equation");
   check(velocity==feedback && !fault,"sampled encoder velocity");
  end
  for(p=1;p<=4;p=p+1) begin
   @(negedge clk);brake=1;cycles(2);
   @(negedge clk);target=16;brake=0;
   while(dut.phase!=p) @(negedge clk);
   brake=1;#1;check(pwm && in1 && in2,"stop must bypass every pipeline stage");
   cycles(6);check(dut.duty==0 && dut.integral==0 && dut.phase==0,"stop cancels pending result");
   @(negedge clk);brake=0;target=16;
   while(dut.phase!=p) @(negedge clk);
   target=24;cycles(6);
   check(dut.duty==0 && dut.integral==0 && dut.phase==0,"target change cancels stale PI result");
  end
  $display("PASS wheel pipeline: sampled PI equivalence and stop/target cancellation in all stages");
  $finish;
 end
 initial begin #10000000;$fatal(1,"test hung");end
endmodule
