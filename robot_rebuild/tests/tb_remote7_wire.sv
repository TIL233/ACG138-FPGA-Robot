`timescale 1ns/1ps
module tb_remote7_wire;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;wire tx;wire [7:0] data;wire valid;
 rb_telemetry #(.CLK_HZ(1000000),.REMOTE_FOLLOW(1),.REMOTE_NAV(1)) tele(
  .clk(clk),.rst_n(rst),.state(8'd0),.health(8'h3e),.command(8'd0),.speed(8'd0),
  .dl(16'd1000),.dr(16'd1000),.vl(16'd0),.vr(16'd0),.roll(16'd0),.pitch(16'd0),.yaw(16'd0),.fault(16'd0),.tx(tx),
  .nav_state(8'h32),.nav_index(3'd1),.nav_count(4'd2),.nav_flags(8'hc0),
  .nav_x(-32'sd100),.nav_y(32'sd200),.nav_heading(-16'sd300),.nav_ack_seq(8'd9),.nav_ack_status(8'd0),.follow_count(16'h1234),.nav_mode_count(16'h5678));
 rb_uart_rx #(.CLK_HZ(1000000)) rx(clk,rst,tx,data,valid);
 reg [7:0] bytes[0:83];integer n=0,j;reg [15:0] crc;
 function [15:0] crcbyte(input [15:0] a,input [7:0] b);
  reg [15:0] c;integer k;begin c=a^{b,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;crcbyte=c;end
 endfunction
 always @(posedge clk)if(valid && n<84)begin bytes[n]<=data;n<=n+1;end
 initial begin
  repeat(4)@(negedge clk);rst=1;wait(n==84);@(negedge clk);
  if(bytes[0]!=8'h5a || bytes[1]!=8'ha5 || bytes[28]!=8'h5a || bytes[29]!=8'hc3 || bytes[30]!=3)$fatal(1,"legacy plus nav v3 header");
  if(bytes[32]!=8'h32 || bytes[48]!=8'h34 || bytes[49]!=8'h12 || bytes[50]!=8'h78 || bytes[51]!=8'h56)$fatal(1,"STOP, key0 and key7 snapshot endian");
  crc=16'hffff;for(j=30;j<52;j=j+1)crc=crcbyte(crc,bytes[j]);
  if(bytes[52]!=crc[7:0] || bytes[53]!=crc[15:8] || bytes[54]!=13 || bytes[55]!=10)$fatal(1,"nav v3 CRC/trailer");
  if(bytes[56]!=8'h5a || bytes[57]!=8'ha5 || bytes[58]!=1)$fatal(1,"ordinary telemetry must follow navigation");
  crc=16'hffff;for(j=58;j<80;j=j+1)crc=crcbyte(crc,bytes[j]);
  if(bytes[80]!=crc[7:0] || bytes[81]!=crc[15:8] || bytes[82]!=13 || bytes[83]!=10)$fatal(1,"ordinary CRC after navigation");
  $display("PASS remote7 wire: ordinary/navigation alternate with valid CRC, STOP and remote generations");$finish;
 end
 initial begin #3000000;$fatal(1,"remote7 wire timeout");end
endmodule
