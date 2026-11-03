`timescale 1ns/1ps
module tb_predictive_wire;
 reg clk=0;always #5 clk=~clk;reg rst_n=0;
 wire tx;wire [7:0] b;wire v;reg [255:0] bundle;
 rb_telemetry #(.CLK_HZ(1000000),.ENABLE_CAMERA_DIAG(1),.CAMERA_DIAG_V4(1),
  .REMOTE_FOLLOW(1),.REMOTE_NAV(1),.ENABLE_MOTION_DIAG(1),.MOTION_HZ(50),.ENABLE_PREDICTIVE_DIAG(1)) tele(
  .clk(clk),.rst_n(rst_n),.state(8'd0),.health(8'd0),.command(8'd0),.speed(8'd0),
  .dl(16'd0),.dr(16'd0),.vl(16'd0),.vr(16'd0),.roll(16'd0),.pitch(16'd0),.yaw(16'd0),.fault(16'd0),.tx(tx),
  .nav_state(8'd0),.nav_index(3'd0),.nav_count(4'd0),.nav_flags(8'd0),.nav_x(32'd0),.nav_y(32'd0),
  .nav_heading(16'd0),.nav_ack_seq(8'd0),.nav_ack_status(8'd0),.camera_diag(240'd0),
  .follow_count(16'd0),.nav_mode_count(16'd0),.motion_diag(320'd0),.predictive_diag(bundle));
 rb_uart_rx #(.CLK_HZ(1000000)) rx(clk,rst_n,tx,b,v);
 reg [7:0] p[0:37];integer n=0,frames=0,legacy=0,nav=0,cam=0,motion=0;
 reg [7:0] prev=0;reg collecting=0;reg [15:0] crc;integer j;
 function [15:0] cb(input [15:0] a,input [7:0] x);
  reg [15:0] c;integer i;begin c=a^{x,8'd0};for(i=0;i<8;i=i+1)c=c[15]?(c<<1)^16'h1021:c<<1;cb=c;end
 endfunction
 always @(posedge clk)if(v)begin
  if(collecting)begin p[n]=b;n=n+1;if(n==38)begin
   collecting=0;frames=frames+1;
   crc=16'hffff;for(j=2;j<34;j=j+1)crc=cb(crc,p[j]);
   if(p[2]!=1 || p[36]!=13 || p[37]!=10 || {p[35],p[34]}!=crc)$fatal(1,"predictive framing/CRC");
   if({p[5],p[4]}!=16'h73 || {p[11],p[10]}!=42 || {p[13],p[12]}!=181)$fatal(1,"predictive flags/solve tag");
   if(p[14]!=6 || p[15]!=8 || {p[17],p[16]}!=-16'sd12 || {p[21],p[20]}!=-16'sd8)$fatal(1,"predictive candidate/signed targets");
   if({p[29],p[28]}!=-16'sd90 || {p[31],p[30]}!=1024 || p[32]!=8'h22 || p[33]!=2)$fatal(1,"predictive model fields");
  end end else if(prev==8'h5a)begin
   case(b)
    8'ha5:legacy=legacy+1;8'hc3:nav=nav+1;8'hd4:cam=cam+1;8'he5:motion=motion+1;
    8'hf6:begin collecting=1;n=2;p[0]=prev;p[1]=b;end
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
  $display("PASS predictive wire: 38-byte CRC/model fields/50Hz motion plus legacy/navigation/camera fairness");$finish;
 end
 initial begin #5000000;$fatal(1,"predictive wire timeout");end
endmodule
