`timescale 1ns/1ps
module tb_imu_angle_boundary;
 reg clk=0;always #250 clk=~clk;
 reg rst=0,rx=1;wire[15:0] roll,pitch,yaw;wire valid,err;wire[31:0] good;
 wire fault,ready;wire[15:0] flags;wire[31:0] stamp;
 imu_parse #(.CLK_FREQ(2000000),.SCAN_MS(50),.RELOCK_MS(50)) dut(
  .clk(clk),.rst_n(rst),.uart_rx(rx),.roll(roll),.pitch(pitch),.yaw(yaw),
  .data_valid(valid),.frame_error(err),.frame_ok_cnt(good),.o_rx_byte_cnt(),
  .debug_last_byte(),.o_gyro_z(),.o_gyro_z_valid());
 rb_motion_feedback #(.CLK_HZ(2000000),.TRACK_MM(100),.FUSE_ENCODER(0)) motion(
  .clk(clk),.rst_n(rst),.clear_fault(1'b0),.imu_ok(1'b1),.imu_update(valid),
  .pos_l(32'sd0),.pos_r(32'sd0),.imu_yaw($signed(yaw)),
  .fault(fault),.ready(ready),.flags(flags),.timestamp_ms(stamp));
 wire signed[15:0] qr,qp,qy;wire qdone;
 quat2euler converter(.clk(clk),.rst_n(rst),.start(1'b0),.q0(16'sd0),.q1(16'sd0),
  .q2(16'sd0),.q3(16'sd0),.roll_o(qr),.pitch_o(qp),.yaw_o(qy),.done(qdone),.busy());
 task send(input[7:0]b);begin rx=0;#8680;
  for(integer i=0;i<8;i=i+1)begin rx=b[i];#8680;end rx=1;#8680;end endtask
 task word(input[31:0]w);for(integer i=0;i<4;i=i+1)send(w>>(8*i));endtask
 task packet(input[31:0] w,input[31:0] x,input[31:0] y,input[31:0] z);
 reg[31:0] before_good;begin
  before_good=good;
  send('h7e);send('h23);send(21);send('h16);word(w);word(x);word(y);word(z);send(0);#200000;
  if(good!=before_good+1)$fatal(1,"UART boundary packet was not accepted exactly once");
  if(err || $signed(yaw)>18000 || $signed(yaw)<-18000 ||
     $signed(roll)>18000 || $signed(roll)<-18000)
   $fatal(1,"legal UART quaternion produced angle outside +/-18000: roll=%0d yaw=%0d",$signed(roll),$signed(yaw));
 end endtask
 integer a,expected;reg signed[18:0] angle,negative_angle;reg[31:0] prior_stamp;
 initial begin
  repeat(4)@(negedge clk);rst=1;
  // Actual float32 UART input, no forced parser/converter output.
  packet(0,0,0,32'h3f800000);
  if(good!=1 || !dut.baud_lock || $signed(yaw)!=18000)$fatal(1,"180-degree parser result");
  #22000000;
  if(fault || !ready)$fatal(1,"legal 180-degree yaw latched motion fault");
  prior_stamp=stamp;
  // Small positive/negative w around 180 degrees: both signed wrap sides.
  packet(32'h38d1b717,0,0,32'h3f800000);
  if($signed(yaw)<17990)$fatal(1,"positive180 yaw side not exercised");
  packet(32'hb8d1b717,0,0,32'h3f800000);
  if($signed(yaw)>-17990)$fatal(1,"negative180 yaw side not exercised");
  packet(0,0,0,32'hbf800000);
  #22000000;
  if(fault || !ready || stamp<=prior_stamp)$fatal(1,"boundary motion stopped updating");
  // Independent resets allow a legitimate 180-degree roll without treating
  // the unrelated yaw change from the previous pose as physical motion.
  @(negedge clk);rst=0;repeat(4)@(negedge clk);rst=1;
  packet(0,32'h3f800000,0,0);
  if($signed(roll)!=18000)$fatal(1,"positive180 roll result");
  packet(32'hb8d1b717,32'h3f800000,0,0);
  if($signed(roll)>-17990)$fatal(1,"negative180 roll side not exercised");
  // Exhaust the converter's admitted millidegree magnitude at its CORDIC
  // result boundary. Product registers, output stages and signed restoration
  // are real RTL; this is not a substitute for the full UART tests above.
  force converter.ZR0=angle;force converter.ZR1=negative_angle;force converter.ZR2=negative_angle;
  for(a=0;a<=180000;a=a+1)begin
   @(negedge clk);angle=a;negative_angle=-a;
   force converter.st=4'd9;@(posedge clk);#1;release converter.st;
   wait(qdone);#1;expected=(a+5)/10;
   if(qr!==expected || qp!==-expected || qy!==-expected)
    $fatal(1,"exact rounding mismatch magnitude=%0d got=%0d,%0d,%0d expected=%0d",a,qr,qp,qy,expected);
   @(posedge clk);#1;
  end
  $display("PASS IMU UART +/-180 yaw/roll, motion wrap continuity, exact conversion all180001 magnitudes both signs");$finish;
 end
 initial begin #1000000000;$fatal(1,"timeout");end
endmodule
