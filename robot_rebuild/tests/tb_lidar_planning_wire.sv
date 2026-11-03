`timescale 1ns/1ps
module tb_lidar_planning_wire;
 reg clk=0;always #5 clk=~clk;reg rst_n=0;
 wire tx;wire [7:0] b;wire v;
 reg [319:0] planning;
 rb_telemetry #(.CLK_HZ(1000000),.ENABLE_CAMERA_DIAG(1),.CAMERA_DIAG_V4(1),
  .REMOTE_FOLLOW(1),.REMOTE_NAV(1),.REMOTE_SPEED(1),.REMOTE_RECORD(1),
  .ENABLE_MOTION_DIAG(1),.MOTION_HZ(50),.ENABLE_PREDICTIVE_DIAG(1),
  .ENABLE_LIDAR_DIAG(1),.ENABLE_LIDAR_PLANNING_DIAG(1)) dut(
  .clk(clk),.rst_n(rst_n),.state(8'd0),.health(8'd0),.command(8'd0),.speed(8'd0),
  .dl(16'd0),.dr(16'd0),.vl(16'd0),.vr(16'd0),.roll(16'd0),.pitch(16'd0),.yaw(16'd0),.fault(16'd0),.tx(tx),
  .nav_state(8'd0),.nav_index(3'd0),.nav_count(4'd0),.nav_flags(8'd0),.nav_x(32'd0),.nav_y(32'd0),
  .nav_heading(16'd0),.nav_ack_seq(8'd0),.nav_ack_status(8'd0),.camera_diag(240'd0),
  .follow_count(16'd0),.nav_mode_count(16'd0),.motion_diag(320'd0),.predictive_diag(256'd0),
  .follow_speed(8'd8),.record_count(16'd0),.lidar_diag(160'd0),.lidar_planning_diag(planning));
 rb_uart_rx #(.CLK_HZ(1000000)) rx(clk,rst_n,tx,b,v);
 reg [7:0] p[0:45];integer n=0,frames=0,legacy=0,nav=0,cam=0,motion=0,lidar=0,predictive=0;
 reg [7:0] prev=0;reg collecting=0;reg [15:0] crc;integer j;
 function [15:0] cb(input [15:0] a,input [7:0] x);
  reg [15:0] c;integer i;begin c=a^{x,8'd0};for(i=0;i<8;i=i+1)c=c[15]?(c<<1)^16'h1021:c<<1;cb=c;end
 endfunction
 always @(posedge clk)if(v)begin
  if(collecting)begin p[n]=b;n=n+1;if(n==46)begin
   collecting=0;frames=frames+1;crc=16'hffff;
   for(j=2;j<42;j=j+1)crc=cb(crc,p[j]);
   if(p[2]!=1 || p[44]!=13 || p[45]!=10 || {p[43],p[42]}!=crc)$fatal(1,"planning CRC/layout");
   if({p[5],p[4]}!=16'h001b || {p[9],p[8],p[7],p[6]}!=32'h12345678)$fatal(1,"flags/time offset");
   if({p[11],p[10]}!=7 || {p[13],p[12]}!=9 || {p[15],p[14]}!=90)$fatal(1,"scan/generation/age");
   if({p[17],p[16]}!=250 || {p[19],p[18]}!=450 || {p[21],p[20]}!=333)$fatal(1,"threshold/clearance");
   if({p[23],p[22]}!=52 || {p[25],p[24]}!=120 || p[26]!=1 || p[27]!=9)$fatal(1,"speed/cycles/candidate");
   if({p[29],p[28]}!=6 || {p[31],p[30]}!=6 || {p[33],p[32]}!=2 || {p[35],p[34]}!=0)$fatal(1,"targets/masks");
   if($signed({p[37],p[36]})!=-1500 || {p[39],p[38]}!=21 || {p[41],p[40]}!=410)$fatal(1,"goal/epoch/budget");
  end end else if(prev==8'h5a)begin
   case(b)
    8'ha5:legacy=legacy+1;8'hc3:nav=nav+1;8'hd4:cam=cam+1;
    8'he5:motion=motion+1;8'hb7:lidar=lidar+1;8'hf6:predictive=predictive+1;
    8'hc8:begin collecting=1;n=2;p[0]=prev;p[1]=b;end
   endcase
  end
  prev=b;
 end
 initial begin
  planning={16'd410,16'd21,-16'sd1500,16'd0,16'd2,16'sd6,16'sd6,8'd9,8'd1,
   16'd120,16'd52,16'd333,16'd450,16'd250,16'd90,16'd9,16'd7,32'h12345678,16'h001b,16'd0};
  repeat(4)@(negedge clk);rst_n=1;
  repeat(450000)@(posedge clk);#1;
  if(frames<3 || legacy<3 || nav<3 || cam<3 || motion<15 || lidar<3 || predictive<3)
   $fatal(1,"planning stream fairness %d/%d/%d/%d/%d/%d/%d",frames,legacy,nav,cam,motion,lidar,predictive);
  $display("PASS planning wire: C8 46-byte CRC/signed fields and all legacy streams fair");$finish;
 end
 initial begin #6000000;$fatal(1,"planning wire timeout");end
endmodule
