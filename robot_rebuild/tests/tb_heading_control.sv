`timescale 1ns/1ps
module tb_heading_control;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,brake=1,healthy=1,u=0;
 reg signed [15:0] h=0,tl=0,tr=0;
 wire signed [15:0] l,r,goal,err,trim;wire hold;
 rb_heading_control dut(clk,rst_n,brake,healthy,u,h,tl,tr,l,r,hold,goal,err,trim);
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task tick;begin @(negedge clk);u=1;cycles(1);@(negedge clk);u=0;end endtask
 task check(input bit good,input string msg);if(!good)$fatal(1,"%s l=%d r=%d error=%d",msg,l,r,err);endtask
 initial begin
  cycles(3);rst_n=1;brake=0;tl=12;tr=12;tick;
  check(hold && goal==0 && l==12 && r==12,"straight start captures heading");
  h=-600;tick;check(err==600 && trim==3 && l==9 && r==15,"right drift gets left correction");
  h=600;tick;check(err==-600 && l==15 && r==9,"left drift gets right correction");
  tl=3;tr=3;#1;check(l>=2 && r>=2 && l+r==6,"trim re-clamps immediately on slower speed");
  tl=4;tr=12;#1;check(l==4 && r==12,"explicit arc bypasses correction before clock");
  cycles(1);check(!hold,"curve clears old straight anchor");
  tl=12;tr=12;h=17900;tick;h=-17900;tick;
  check(err==-200 && l>r,"shortest angular error across wrap");
  tl=-12;tr=-12;#1;check(l==-12 && r==-12,"reversal drops stale trim immediately");
  cycles(1);tick;h=-17300;tick;
  check(err==-600 && l==-9 && r==-15,"reverse heading correction keeps both requested signs");
  brake=1;#1;check(l==tl && r==tr,"raw brake bypasses held correction");cycles(1);check(!hold,"STOP clears anchor");
  brake=0;tick;healthy=0;cycles(1);check(!hold && trim==0,"stale feedback clears controller");
  healthy=1;tl=-8;tr=8;tick;check(!hold && l==-8 && r==8,"explicit pivot is preserved");
  $display("PASS heading control: forward/reverse correction/wrap/limit/curves/pivots/STOP/stale");$finish;
 end
 initial begin #10000;$fatal(1,"heading control timeout");end
endmodule
