`timescale 1ns/1ps
module tb_imu;
 reg clk=0;always #250 clk=~clk;
 reg rst=0,rx=1;wire [15:0] roll,pitch,yaw;wire valid,err;
 wire [31:0] good,bytes;integer errors=0;real bit_ns=8680.0;
 imu_parse #(.CLK_FREQ(2000000),.SCAN_MS(50),.RELOCK_MS(50)) dut(
  .clk(clk),.rst_n(rst),.uart_rx(rx),.roll(roll),.pitch(pitch),.yaw(yaw),
  .data_valid(valid),.frame_error(err),.frame_ok_cnt(good),.o_rx_byte_cnt(bytes),
  .debug_last_byte(),.o_gyro_z(),.o_gyro_z_valid());
 always @(posedge clk) if(err) errors=errors+1;
 task send(input [7:0] b);
  rx=0;#(bit_ns);for(integer i=0;i<8;i=i+1) begin rx=b[i];#(bit_ns);end
  rx=1;#(bit_ns);
 endtask
 task packet(input [31:0] w,input [7:0] len=21);
  send('h7e);send('h23);send(len);send('h16);
  for(integer i=0;i<4;i=i+1) send(w>>(8*i));
  repeat(12) send(0);
  if(len==21) send(0);
  #200000;
 endtask
 initial begin
  repeat(4) @(negedge clk);rst=1;
  packet(32'h7f800000);packet(32'h7fc00000);
  if(good!=0 || errors!=2) $fatal(1,"NaN/Inf was accepted or not reported");
  packet(32'h3f800000,20);
  if(good!=0) $fatal(1,"wrong quaternion frame length accepted");
  packet(32'h3f800000);
  if(good!=1 || !dut.baud_lock) $fatal(1,"normal quaternion did not lock");
  // Sensor restart at a different rate: the original implementation stayed
  // locked forever and could not recover without resetting the FPGA.
  #55000000;
  if(dut.baud_lock) $fatal(1,"no-data timeout failed to unlock baud");
  bit_ns=104166.667;
  for(integer n=0;n<40 && good==1;n=n+1) packet(32'h3f800000);
  if(good<2 || !dut.baud_lock || dut.baud_idx!=7) $fatal(1,"9600 baud recovery failed");
  if($signed(roll)>3 || $signed(roll)<-3 || $signed(pitch)>3 || $signed(pitch)<-3 || $signed(yaw)>3 || $signed(yaw)<-3) $fatal(1,"identity attitude corrupted");
  $display("PASS IMU: nonfinite/length rejection and baud relock after sensor restart");$finish;
 end
 initial begin #1500000000;$fatal(1,"timeout");end
endmodule
