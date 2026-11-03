`timescale 1ns/1ps
module tb_motion_wire;
 reg clk=0;always #5 clk=~clk;reg rst_n=0;
 wire tx;wire [7:0] b;wire v;
 reg [319:0] bundle;
 rb_telemetry #(.CLK_HZ(1000000),.ENABLE_CAMERA_DIAG(1),.CAMERA_DIAG_V4(1),
  .REMOTE_FOLLOW(1),.REMOTE_NAV(1),.ENABLE_MOTION_DIAG(1)) tele(
  .clk(clk),.rst_n(rst_n),.state(0),.health(0),.command(0),.speed(0),
  .dl(0),.dr(0),.vl(0),.vr(0),.roll(0),.pitch(0),.yaw(0),.fault(0),.tx(tx),
  .nav_state(0),.nav_index(0),.nav_count(0),.nav_flags(0),.nav_x(0),.nav_y(0),
  .nav_heading(0),.nav_ack_seq(0),.nav_ack_status(0),.camera_diag(240'd0),
  .follow_count(0),.nav_mode_count(0),.motion_diag(bundle));
 rb_uart_rx #(.CLK_HZ(1000000)) rx(clk,rst_n,tx,b,v);
 reg [7:0] packet[0:45];integer n=0,frames=0,bytes_seen=0,legacy=0,nav=0,cam=0;
 reg [7:0] prev=0;reg collecting=0;reg [15:0] crc;
 function [15:0] cb(input [15:0] a,input [7:0] x);
  reg [15:0] c;integer i;begin c=a^{x,8'd0};for(i=0;i<8;i=i+1)c=c[15]?(c<<1)^16'h1021:c<<1;cb=c;end
 endfunction
 integer j;
 always @(posedge clk)if(v)begin
  bytes_seen=bytes_seen+1;
  if(collecting)begin packet[n]=b;n=n+1;if(n==46)begin
   collecting=0;frames=frames+1;
   if(packet[2]!=1 || packet[44]!=13 || packet[45]!=10)$fatal(1,"motion framing");
   crc=16'hffff;for(j=2;j<42;j=j+1)crc=cb(crc,packet[j]);
   if({packet[43],packet[42]}!=crc)$fatal(1,"motion CRC");
   if({packet[5],packet[4]}!=16'h004f || {packet[9],packet[8],packet[7],packet[6]}!=32'h12345678)
    $fatal(1,"flags and FPGA sample clock");
   if({packet[11],packet[10]}!=-16'sd17900 || {packet[13],packet[12]}!=16'sd17900)
    $fatal(1,"signed headings");
   if({packet[41],packet[40],packet[39],packet[38]}!=-32'sd2000)$fatal(1,"signed position end offset");
  end end else if(prev==8'h5a)begin
   if(b==8'ha5)legacy=legacy+1;
   if(b==8'hc3)nav=nav+1;
   if(b==8'hd4)cam=cam+1;
   if(b==8'he5)begin collecting=1;n=2;packet[0]=prev;packet[1]=b;end
  end
  prev=b;
 end
 initial begin
  bundle={-32'sd2000,32'sd1000,-16'sd900,16'sd2,16'sd12,16'sd8,
   16'sd11,16'sd9,-16'sd200,16'sd17700,16'sd100,16'sd17800,16'sd17900,-16'sd17900,
   32'h12345678,16'h004f,16'd0};
  repeat(4)@(negedge clk);rst_n=1;
  wait(frames==2);#1;
  if(legacy<2 || nav<2 || cam<2)$fatal(1,"motion telemetry must not starve existing streams");
  $display("PASS motion wire: CRC/signed offsets/sample timestamp/coexistence with legacy/nav/camera");$finish;
 end
 initial begin #3000000;$fatal(1,"motion wire timeout");end
endmodule
