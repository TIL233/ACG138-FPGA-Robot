`timescale 1ns/1ps
module tb_motion_feedback;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,clear=0,ok=1,iu=0;
 reg signed [31:0] pl=0,pr=0;
 reg signed [15:0] yaw=0;
 wire u,ready,fault;wire signed [15:0] h,eh,inn,rate,vl,vr;
 wire [31:0] stamp;wire [15:0] flags;
 wire signed [31:0] sl,sr;wire signed [15:0] sy;
 rb_motion_feedback #(.CLK_HZ(100000),.COUNTS_PER_M(10000),.TRACK_MM(200),
  .DISAGREE_WINDOWS(3),.DISAGREE_CD(40)) dut(
  .clk(clk),.rst_n(rst_n),.clear_fault(clear),.imu_ok(ok),.imu_update(iu),
  .pos_l(pl),.pos_r(pr),.imu_yaw(yaw),.update(u),.ready(ready),.fault(fault),
  .heading(h),.encoder_heading(eh),.innovation(inn),.yaw_rate(rate),
  .velocity_l(vl),.velocity_r(vr),.sample_pos_l(sl),.sample_pos_r(sr),.sample_yaw(sy),
  .timestamp_ms(stamp),.flags(flags));
 wire bypass_u,bypass_ready;wire signed [15:0] bypass_h;
 rb_motion_feedback #(.CLK_HZ(100000),.TRACK_MM(0)) bypass(
  .clk(clk),.rst_n(rst_n),.clear_fault(clear),.imu_ok(ok),.imu_update(iu),
  .pos_l(pl),.pos_r(pr),.imu_yaw(yaw),.update(bypass_u),.ready(bypass_ready),.heading(bypass_h));
 wire signed [15:0] mirrored_yaw=-yaw;
 wire signed [15:0] mirrored_heading;wire mirror_ready,mirror_fault;
 rb_motion_feedback #(.CLK_HZ(100000),.COUNTS_PER_M(10000),.TRACK_MM(200),.YAW_SIGN(-1)) mirror(
  .clk(clk),.rst_n(rst_n),.clear_fault(clear),.imu_ok(ok),.imu_update(iu),
  .pos_l(pl),.pos_r(pr),.imu_yaw(mirrored_yaw),.heading(mirrored_heading),
  .ready(mirror_ready),.fault(mirror_fault));
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit good,input string msg);if(!good)$fatal(1,"%s h=%d enc=%d flags=%h",msg,h,eh,flags);endtask
 task frame(input integer angle);begin
  @(negedge clk);yaw=angle;iu=1;cycles(1);@(negedge clk);iu=0;
 end endtask
 task snapshot;begin @(posedge u);#1;@(negedge clk);end endtask
 task recover(input integer angle);begin
  @(negedge clk);clear=1;cycles(1);@(negedge clk);clear=0;
  frame(angle);snapshot;
 end endtask
 integer i;reg [31:0] last_stamp;reg signed [15:0] held;
 initial begin
  cycles(4);rst_n=1;frame(0);snapshot;
  check(ready && !fault && flags[2] && stamp>=19,"initialized calibrated FPGA snapshot");
  check(bypass_ready && bypass_h==0,"uncalibrated IMU path initializes");
  for(i=1;i<=10;i=i+1)begin
   pl=pl-10;pr=pr+10;frame(i*57);snapshot;
   check(vl==-10 && vr==10 && sl==pl && sr==pr && sy==yaw,"coherent position/IMU/count snapshot");
  end
  check(h>=560 && h<=580 && eh>=570 && eh<=574,"encoder prediction and IMU agreement");
  check(!flags[5] && bypass_h==yaw,"no disagreement on calibrated movement");
  // Q16 flooring and the 1/4 integer correction can differ by up to 0.04deg
  // between signs; this is not a 180deg direction reversal.
  check(mirror_ready && !mirror_fault && mirrored_heading>=-h-4 && mirrored_heading<=-h+4,
        "negative IMU convention uses the same physical encoder prediction");
  clear=1;cycles(1);clear=0;cycles(1);
  check(ready && !fault,"ordinary DISARM preserves a healthy estimator for immediate pose reset");
  last_stamp=stamp;pl=pl+10;pr=pr+10;frame(yaw);snapshot;
  check(stamp>=last_stamp+19 && h>=560 && h<=580,"straight movement does not create turn");
  // One dropped IMU frame: wheel prediction continues, no repeated correction
  // towards an older angle. The control timeout remains independently bounded.
  held=h;pl=pl-10;pr=pr+10;snapshot;
  check(h>=held+56 && h<=held+58,"encoder prediction between asynchronous IMU frames");
  frame(yaw+114);pl=pl-10;pr=pr+10;snapshot;
  check(!flags[5],"IMU comparison accounts for encoder motion over missing frame");
  // Encoder-only apparent turning while the IMU remains fixed is diagnostic,
  // not proof of a particular slip mode and not an automatic motor ARM.
  for(i=0;i<4;i=i+1)begin pl=pl-10;pr=pr+10;frame(yaw);snapshot;end
  check(flags[5] && !fault,"persistent sensor disagreement reported without fabricated wiring fault");
  // Encoder discontinuity must not be integrated; explicit reset is required.
  pl=pl+1000;frame(yaw);cycles(2100);
  check(fault && !ready && flags[7],"encoder spike latches invalid feedback and a specific reason");
  recover(17900);check(ready && !fault,"STOP clear and valid input recover");
  pl=pl-35;pr=pr+35;frame(-17900);snapshot;
  check(h< -17890 && h> -17910,"heading wrap crosses +180/-180 by small positive step");
  check(rate>0 && rate<15000,"wrap-safe bounded angular rate");
  frame(0);cycles(3);check(fault && !ready && flags[8],"single IMU jump invalidates immediately with a reason");
  recover(0);last_stamp=stamp;cycles(21000);
  check(!ready && !flags[3] && stamp<last_stamp+210,"stale IMU stops publication, encoder cannot hide loss");
  frame(0);snapshot;check(ready,"fresh IMU reacquires a zero-delta baseline");
  ok=0;cycles(1);check(!ready,"raw sensor loss bypasses sample wait");
  $display("PASS motion feedback: coherent samples/fusion/asynchronous IMU/wrap/disagreement/fault/recovery/stale");$finish;
 end
 initial begin #3000000;$fatal(1,"motion feedback timeout");end
endmodule
