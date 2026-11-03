`timescale 1ns/1ps
module tb_lidar_wire;
 reg clk=0;always #5 clk=~clk;reg rst_n=0;
 wire tx;wire [7:0] b;wire v;reg [255:0] bundle;
 rb_telemetry #(.CLK_HZ(1000000),.ENABLE_CAMERA_DIAG(1),.CAMERA_DIAG_V4(1),
  .REMOTE_FOLLOW(1),.REMOTE_NAV(1),.ENABLE_MOTION_DIAG(1),.MOTION_HZ(50),.ENABLE_PREDICTIVE_DIAG(1),.ENABLE_LIDAR_DIAG(1)) tele(
  .clk(clk),.rst_n(rst_n),.state(8'd0),.health(8'd0),.command(8'd0),.speed(8'd0),
  .dl(16'd0),.dr(16'd0),.vl(16'd0),.vr(16'd0),.roll(16'd0),.pitch(16'd0),.yaw(16'd0),.fault(16'd0),.tx(tx),
  .nav_state(8'd0),.nav_index(3'd0),.nav_count(4'd0),.nav_flags(8'd0),.nav_x(32'd0),.nav_y(32'd0),
  .nav_heading(16'd0),.nav_ack_seq(8'd0),.nav_ack_status(8'd0),.camera_diag(240'd0),
  .follow_count(16'd0),.nav_mode_count(16'd0),.motion_diag(320'd0),.predictive_diag(bundle),.lidar_diag({16'd640,16'd2,16'd123,16'hffff,16'd900,16'd1200,16'd500,16'd7,16'h37,16'd0}));
 rb_uart_rx #(.CLK_HZ(1000000)) rx(clk,rst_n,tx,b,v);
 reg [7:0] p[0:25];integer n=0,frames=0,legacy=0,nav=0,cam=0,motion=0;
 reg [7:0] prev=0;reg collecting=0;reg [15:0] crc;integer j;
 function [15:0] cb(input [15:0] a,input [7:0] x);
  reg [15:0] c;integer i;begin c=a^{x,8'd0};for(i=0;i<8;i=i+1)c=c[15]?(c<<1)^16'h1021:c<<1;cb=c;end
 endfunction
 always @(posedge clk)if(v)begin
  if(collecting)begin p[n]=b;n=n+1;if(n==26)begin
   collecting=0;frames=frames+1;
   crc=16'hffff;for(j=2;j<22;j=j+1)crc=cb(crc,p[j]);
   if(p[2]!=1 || p[24]!=13 || p[25]!=10 || {p[23],p[22]}!=crc)$fatal(1,"lidar framing/CRC");
   if({p[5],p[4]}!=55 || {p[7],p[6]}!=7 || {p[9],p[8]}!=500 || {p[11],p[10]}!=1200)$fatal(1,"lidar flags/id/front/left");
   if({p[13],p[12]}!=900 || {p[15],p[14]}!=16'hffff || {p[17],p[16]}!=123 || {p[19],p[18]}!=2 || {p[21],p[20]}!=640)$fatal(1,"lidar right/reserved/counters/zero");
  end end else if(prev==8'h5a)begin
   case(b)
    8'ha5:legacy=legacy+1;8'hc3:nav=nav+1;8'hd4:cam=cam+1;8'he5:motion=motion+1;
    8'hb7:begin collecting=1;n=2;p[0]=prev;p[1]=b;end
   endcase
  end
  prev=b;
 end
 initial begin
  bundle={8'd2,8'h22,16'd1024,-16'sd90,32'h12345678,16'sd8,-16'sd8,16'sd12,-16'sd12,
   8'd8,8'd6,16'd181,16'd42,32'd500,16'h73,16'd0};
  repeat(4)@(negedge clk);rst_n=1;
  repeat(400000)@(posedge clk);#1;
  if(frames<3 || legacy<3 || nav<3 || cam<3 || motion<15)$fatal(1,"mixed stream starvation %d/%d/%d/%d/%d",frames,legacy,nav,cam,motion);
  $display("PASS lidar wire: 26-byte CRC/ranges/counters and mixed telemetry fairness");$finish;
 end
 initial begin #5000000;$fatal(1,"predictive wire timeout");end
endmodule
