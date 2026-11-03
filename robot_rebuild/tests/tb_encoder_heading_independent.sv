`timescale 1ns/1ps
module tb_encoder_heading_independent;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,clear=0,ok=1,iu=0;
 reg signed [31:0] pl=0,pr=0;
 reg signed [15:0] yaw=0;
 wire update,ready,fault;wire signed [15:0] heading,encoder_heading,innovation;
 wire signed [15:0] velocity_l,velocity_r,sample_yaw;wire [15:0] flags;
 rb_motion_feedback #(.CLK_HZ(100000),.COUNTS_PER_M(7640),.TRACK_MM(100),
                      .FUSE_ENCODER(0),.DISAGREE_WINDOWS(3)) dut(
  .clk(clk),.rst_n(rst_n),.clear_fault(clear),.imu_ok(ok),.imu_update(iu),
  .pos_l(pl),.pos_r(pr),.imu_yaw(yaw),.update(update),.ready(ready),.fault(fault),
  .heading(heading),.encoder_heading(encoder_heading),.innovation(innovation),
  .velocity_l(velocity_l),.velocity_r(velocity_r),.sample_yaw(sample_yaw),.flags(flags),
  .yaw_rate(),.sample_pos_l(),.sample_pos_r(),.timestamp_ms());
 wire signed [15:0] mirror_heading,mirror_encoder;
 wire mirror_ready,mirror_fault;wire [15:0] mirror_flags;
 wire signed [15:0] mirror_yaw=-yaw;
 rb_motion_feedback #(.CLK_HZ(100000),.COUNTS_PER_M(7640),.TRACK_MM(100),
                      .FUSE_ENCODER(0),.YAW_SIGN(-1)) mirror(
  .clk(clk),.rst_n(rst_n),.clear_fault(clear),.imu_ok(ok),.imu_update(iu),
  .pos_l(pl),.pos_r(pr),.imu_yaw(mirror_yaw),.heading(mirror_heading),
  .encoder_heading(mirror_encoder),.ready(mirror_ready),.fault(mirror_fault),.flags(mirror_flags),
  .update(),.innovation(),.yaw_rate(),.velocity_l(),.velocity_r(),
  .sample_pos_l(),.sample_pos_r(),.sample_yaw(),.timestamp_ms());
 wire bypass_ready,bypass_fault;wire signed [15:0] bypass_heading,bypass_encoder;
 wire [15:0] bypass_flags;
 rb_motion_feedback #(.CLK_HZ(100000),.TRACK_MM(0)) unmeasured(
  .clk(clk),.rst_n(rst_n),.clear_fault(clear),.imu_ok(ok),.imu_update(iu),
  .pos_l(pl),.pos_r(pr),.imu_yaw(yaw),.ready(bypass_ready),.fault(bypass_fault),
  .heading(bypass_heading),.encoder_heading(bypass_encoder),.flags(bypass_flags),
  .update(),.innovation(),.yaw_rate(),.velocity_l(),.velocity_r(),
  .sample_pos_l(),.sample_pos_r(),.sample_yaw(),.timestamp_ms());

 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit good,input string message);
  if(!good)$fatal(1,"%s main=%0d encoder=%0d flags=%h ready=%b fault=%b",
                  message,heading,encoder_heading,flags,ready,fault);
 endtask
 task frame(input integer angle);begin
  @(negedge clk);yaw=angle;iu=1;cycles(1);@(negedge clk);iu=0;
 end endtask
 task snapshot;begin @(posedge update);#1;@(negedge clk);end endtask
 task reset_at(input integer angle);begin
  @(negedge clk);rst_n=0;clear=0;ok=1;iu=0;pl=0;pr=0;yaw=angle;cycles(4);
  @(negedge clk);rst_n=1;frame(angle);snapshot;
  check(ready && !fault && flags[0] && flags[1] && flags[3],"fresh IMU initializes readiness");
  check(flags[10] && !flags[2] && !flags[6],"independent encoder availability has bit10 without enabling fusion or holding");
  check(heading==angle && encoder_heading==angle,"encoder and IMU share only their initial angular origin");
  check(bypass_ready && !bypass_fault && !bypass_flags[10] && !bypass_flags[2],
        "default zero-track behavior remains unconfigured and unfused");
 end endtask
 task move(input integer dl,input integer dr,input integer angle);begin
  @(negedge clk);pl=pl+dl;pr=pr+dr;frame(angle);snapshot;
 end endtask
 integer i;reg signed [15:0] held_encoder;
 initial begin
  reset_at(0);
  // This is a geometric encoder reference (600 counts per wheel is about90
  // degrees for these parameters), not a physical ground-angle calibration.
  for(i=1;i<=10;i=i+1)begin
   move(-60,60,i*225);
   check(heading==i*225 && sample_yaw==i*225 && innovation==0,
         "main heading remains the quarter-scale IMU without correction");
   check(velocity_l==-60 && velocity_r==60,"encoder motion samples retain direction");
  end
  check(encoder_heading>=8998 && encoder_heading<=9001 && heading==2250,
        "encoder90 stays independent when IMU reports only22.5degrees");
  check(flags[5] && !fault,"persistent disagreement remains diagnostic and cannot disable turning by itself");
  check(mirror_ready && !mirror_fault && mirror_heading==-2250 &&
        mirror_encoder>=-encoder_heading-1 && mirror_encoder<=-encoder_heading+1,
        "negative IMU convention mirrors the independent encoder angle");
  check(bypass_heading==2250 && bypass_encoder==2250,"zero-track encoder output retains its legacy IMU fallback");

  held_encoder=encoder_heading;
  for(i=1;i<=8;i=i+1)begin
   move(0,0,2250+i*100);
   check(encoder_heading==held_encoder && heading==2250+i*100,
         "stationary IMU drift cannot pull the independent encoder angle");
  end
  @(negedge clk);clear=1;cycles(1);@(negedge clk);clear=0;cycles(2);
  check(ready && !fault && encoder_heading==held_encoder,"ordinary fault-clear preserves a healthy encoder origin");

  reset_at(0);
  for(i=1;i<=10;i=i+1)move(-60,60,0);
  check(heading==0 && encoder_heading>=8998 && encoder_heading<=9001 && !fault,
        "repeated fresh but frozen IMU reports do not freeze the encoder angle");
  for(i=1;i<=10;i=i+1)move(60,-60,0);
  check(encoder_heading>=-1 && encoder_heading<=1 && !fault,"opposite wheel rotation unwinds the independent encoder angle");

  reset_at(17900);move(-20,20,17900);
  check(heading==17900 && encoder_heading>=-17802 && encoder_heading<=-17799,
        "independent positive encoder turn crosses+180 while the IMU remains fixed");
  reset_at(-17900);move(20,-20,-17900);
  check(heading==-17900 && encoder_heading>=17799 && encoder_heading<=17802,
        "independent negative encoder turn crosses-180 while the IMU remains fixed");

  // Positions also wrap modulo2^32. No motion is fabricated when signed
  // encoder positions cross their representational boundary.
  reset_at(0);
  @(negedge clk);rst_n=0;pl=32'h80000010;pr=32'h7ffffff0;cycles(3);
  @(negedge clk);rst_n=1;frame(0);snapshot;
  move(-40,40,0);
  check(!fault && encoder_heading>=599 && encoder_heading<=601,"32bit position wrap preserves short signed wheel increments");

  reset_at(0);@(negedge clk);pl=pl+513;frame(0);cycles(2100);
  check(fault && !ready && flags[7],"encoder discontinuity still latches failure with fusion disabled");
  reset_at(0);@(negedge clk);pl=pl-250;pr=pr+250;frame(0);cycles(2100);
  check(fault && !ready && flags[7],"implausible derived encoder angular step still faults when each wheel delta is in range");
  reset_at(0);frame(4000);cycles(3);
  check(fault && !ready && flags[8],"implausible IMU jumps remain protected with an independent encoder output");
  reset_at(0);cycles(21000);
  check(!ready && !flags[3],"missing IMU frames still invalidate readiness");
  frame(500);snapshot;
  check(ready && !fault && heading==500 && encoder_heading==500,"fresh reinitialization establishes a coherent new angular origin");
  ok=0;cycles(1);check(!ready,"raw IMU loss still invalidates readiness immediately");
  $display("PASS independent encoder heading: quarter/frozen/drifting IMU, no global fusion, signs, wrap, initialization, encoder/IMU faults and stale readiness");
  $finish;
 end
 initial begin #5000000;$fatal(1,"independent encoder heading test hung");end
endmodule
