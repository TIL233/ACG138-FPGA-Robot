`timescale 1ns/1ps
module tb_camera_flow_wire;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;wire tx;wire [7:0]data;wire valid;
 rb_telemetry #(.CLK_HZ(1000000),.ENABLE_CAMERA_DIAG(1),.CAMERA_DIAG_V2(1)) tele(
  .clk(clk),.rst_n(rst),.state(8'd0),.health(8'h3e),.command(8'd0),.speed(8'd0),
  .dl(16'd1000),.dr(16'd1000),.vl(16'd0),.vr(16'd0),.roll(16'd0),.pitch(16'd0),.yaw(16'd0),.fault(16'd0),.tx(tx),
  .nav_state(8'h32),.nav_index(3'd0),.nav_count(4'd1),.nav_flags(8'hc0),
  .nav_x(32'd0),.nav_y(32'd0),.nav_heading(16'd0),.nav_ack_seq(8'd0),.nav_ack_status(8'hff),
  .camera_diag({16'd2,16'd119,16'd120,16'd3,16'd1440,16'd9000,8'hff,8'h0f,16'd3,8'h40,8'h56,8'd12,8'd6,16'd0}));
 rb_uart_rx #(.CLK_HZ(1000000)) rx(clk,rst,tx,data,valid);
 reg [7:0] bytes[0:79];integer n=0,j;reg [15:0]crc;
 function [15:0] crcbyte(input [15:0] a,input [7:0] b);
  reg [15:0]c;integer k;begin c=a^{b,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;crcbyte=c;end
 endfunction
 always @(posedge clk)if(valid && n<80)begin bytes[n]<=data;n<=n+1;end
 initial begin
  repeat(4)@(negedge clk);rst=1;
  wait(n==80);@(negedge clk);
  if(bytes[0]!=8'h5a || bytes[1]!=8'ha5 || bytes[28]!=8'h5a || bytes[29]!=8'hc3 || bytes[32]!=8'h32)$fatal(1,"legacy/nav preserved");
  if(bytes[52]!=8'h5a || bytes[53]!=8'hd4 || bytes[54]!=2 || bytes[56]!=6 || bytes[57]!=12)$fatal(1,"diagnostic header/state");
  if(bytes[58]!=8'h56 || bytes[59]!=8'h40 || bytes[60]!=3 || bytes[61]!=0 || bytes[62]!=8'h0f || bytes[63]!=8'hff)$fatal(1,"diagnostic snapshot endian");
  if({bytes[65],bytes[64]}!=9000 || {bytes[67],bytes[66]}!=1440 || {bytes[69],bytes[68]}!=3 || {bytes[71],bytes[70]}!=120 || {bytes[73],bytes[72]}!=119 || {bytes[75],bytes[74]}!=2)$fatal(1,"flow snapshot endian");
  crc=16'hffff;for(j=54;j<76;j=j+1)crc=crcbyte(crc,bytes[j]);
  if(bytes[76]!=crc[7:0] || bytes[77]!=crc[15:8] || bytes[78]!=13 || bytes[79]!=10)$fatal(1,"diagnostic wire CRC");
  $display("PASS camera flow wire v2: legacy/nav preserved, ID and six counters, real UART CRC");$finish;
 end
 initial begin #3000000;$fatal(1,"diagnostic stream timeout");end
endmodule
