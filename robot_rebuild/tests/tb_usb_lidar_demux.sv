`timescale 1ns/1ps
// The PRE_BRIDGE run characterizes the unsafe old shared UART path: an
// otherwise valid ARM hidden in lidar bytes reaches the command receiver.
module tb_usb_lidar_demux;
 reg clk=0;always #5 clk=~clk;reg rst_n=0,valid=0;reg [7:0] data=0;
 wire [7:0] cd,ld;wire cv,lv,clear;
`ifdef PRE_BRIDGE
 assign cd=data;assign cv=valid;assign ld=0;assign lv=0;assign clear=0;
`else
 rb_lidar_bridge_rx #(.CLK_HZ(1000000)) dut(clk,rst_n,data,valid,cd,cv,ld,lv,clear);
`endif
 wire armed,updated;wire [2:0] command;
 rb_command_rx #(.CLK_HZ(1000000)) commands(.clk(clk),.rst_n(rst_n),.data(cd),.valid(cv),
  .armed(armed),.command(command),.update(updated),.speed(),.fresh(),.mode_nav(),.mode_follow());
 wire route_write;wire [7:0] ack;
 rb_nav_config_rx #(.CLK_HZ(1000000)) nav(.clk(clk),.rst_n(rst_n),.data(cd),.valid(cv),
  .cfg_allowed(1'b1),.route_write(route_write),.last_ack_seq(ack),.route_index(),.route_x(),.route_y(),
  .route_commit(),.route_count(),.pose_reset(),.route_clear(),.route_ready(),.last_ack_status());
 reg [7:0] payload[0:127],control[0:4095],lidar[0:4095];
 integer nc=0,nl=0,nclear=0,narm=0,nroute=0,i,base,before_clear;
 always @(posedge clk)begin
  if(cv)begin control[nc]=cd;nc=nc+1;end
  if(lv)begin lidar[nl]=ld;nl=nl+1;end
  if(clear)nclear=nclear+1;
  if(updated && armed)narm=narm+1;
  if(route_write)nroute=nroute+1;
 end
 function [15:0] cb(input [15:0] crc,input [7:0] b);
  reg [15:0] c;integer j;begin c=crc^{b,8'd0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;cb=c;end
 endfunction
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task send(input [7:0] b);begin @(negedge clk);data=b;valid=1;@(negedge clk);valid=0;end endtask
 task escaped(input [7:0] b);begin
  if(b==8'ha5 || b==8'h7d || b==8'h7e)begin send(8'h7d);send(b^8'h20);end else send(b);
 end endtask
 task frame(input [7:0] seq,input [7:0] flags,input integer n,input bit corrupt);
  reg [15:0] c;integer j;begin
   c=cb(cb(cb(cb(16'hffff,1),seq),flags),n);
   send(8'ha5);send(8'hb7);escaped(1);escaped(seq);escaped(flags);escaped(n);
   for(j=0;j<n;j=j+1)begin escaped(payload[j]);c=cb(c,payload[j]);end
   escaped(c[7:0]^corrupt);escaped(c[15:8]);send(8'h7e);ticks(8);
  end
 endtask
 task arm_payload(input [7:0] seq,input bit arm);
  reg [15:0] c;integer j;begin
   payload[0]=8'ha5;payload[1]=8'h5a;payload[2]=1;payload[3]=seq;
   payload[4]=arm;payload[5]=arm;payload[6]=arm?8:0;payload[7]=0;
   c=16'hffff;for(j=2;j<8;j=j+1)c=cb(c,payload[j]);
   payload[8]=c[7:0];payload[9]=c[15:8];payload[10]=13;payload[11]=10;
  end
 endtask
 task route_payload;
  reg [15:0] c;integer j;begin
   payload[0]=8'ha5;payload[1]=8'hc3;payload[2]=1;payload[3]=7;payload[4]=1;payload[5]=0;
   payload[6]=8'ha5;payload[7]=8'h07;payload[8]=8'hb7;payload[9]=8'h07;
   for(j=10;j<16;j=j+1)payload[j]=0;
   c=16'hffff;for(j=2;j<16;j=j+1)c=cb(c,payload[j]);
   payload[16]=c[7:0];payload[17]=c[15:8];payload[18]=13;payload[19]=10;
  end
 endtask
 initial begin
  ticks(5);rst_n=1;ticks(5);
  arm_payload(1,1);frame(0,0,12,1);ticks(2000);
  if(nc || armed || narm)$fatal(1,"corrupt tunnel leaked ARM to legacy receiver nc=%0d armed=%0d",nc,armed);
  frame(0,0,12,0);ticks(2000);
  if(nc || armed || narm || nl!=12 || lidar[0]!=8'ha5 || lidar[1]!=8'h5a)$fatal(1,"validated tunnel routing/escaping");
  route_payload();frame(1,0,20,0);ticks(2500);
  if(nroute || ack!=0 || nc || nl!=32)$fatal(1,"tunneled route reached config receiver");
  // Legacy frame contents must be opaque, even an embedded tunnel header.
  base=nc;arm_payload(8'ha5,1);payload[4]=8'hb7;
  for(i=0;i<12;i=i+1)send(payload[i]);ticks(20);
  if(nc-base!=12)$fatal(1,"legacy command truncated at embedded A5B7");
  for(i=0;i<12;i=i+1)if(control[base+i]!==payload[i])$fatal(1,"legacy command byte changed %0d",i);
  base=nc;route_payload();payload[6]=8'ha5;payload[7]=8'hb7;
  for(i=0;i<20;i=i+1)send(payload[i]);ticks(20);
  if(nc-base!=20)$fatal(1,"legacy nav truncated");
  for(i=0;i<20;i=i+1)if(control[base+i]!==payload[i])$fatal(1,"legacy nav byte changed %0d",i);
  // A real ARM and STOP still reach the command receiver between tunnels.
  arm_payload(9,1);for(i=0;i<12;i=i+1)send(payload[i]);ticks(20);
  if(!armed || narm!=1)$fatal(1,"ordinary ARM lost");
  frame(2,0,12,0);ticks(1300);
  arm_payload(10,0);for(i=0;i<12;i=i+1)send(payload[i]);ticks(20);
  if(armed || command!=0)$fatal(1,"interleaved STOP lost");
  // Unescaped A5 aborts a truncated tunnel and starts the STOP header.
  send(8'ha5);send(8'hb7);send(1);send(3);send(0);send(32);send(9);
  arm_payload(11,0);base=nc;for(i=0;i<12;i=i+1)send(payload[i]);ticks(30);
  if(nc-base!=12 || armed || control[base]!=8'ha5 || control[base+1]!=8'h5a)$fatal(1,"truncated tunnel swallowed STOP");
  // Wrong escape/version/length/trailer are all consumed, never forwarded.
  base=nc;before_clear=nclear;
  send(8'ha5);send(8'hb7);send(2);send(8'h7d);send(8'h85);send(8'h5a);send(8'h7e);
  send(8'ha5);send(8'hb7);send(1);send(0);send(0);send(0);send(8'h7e);
  send(8'ha5);send(8'hb7);send(1);send(0);send(0);send(1);send(8'h7d);send(0);send(8'h7e);ticks(20);
  if(nc!=base || nclear-before_clear!=3)$fatal(1,"malformed tunnel not quarantined/cleared");
  // Sequence gaps drop their payload, clear queued bytes, then resume at +1.
  payload[0]=8'h41;payload[1]=8'h7d;payload[2]=8'h7e;payload[3]=8'ha5;
  frame(20,0,4,0);ticks(500);base=nl;before_clear=nclear;
  frame(22,0,4,0);ticks(500);
  if(nl!=base || nclear!=before_clear+1)$fatal(1,"sequence gap was emitted");
  frame(23,0,4,0);ticks(500);
  if(nl!=base+4 || lidar[base+1]!=8'h7d || lidar[base+2]!=8'h7e || lidar[base+3]!=8'ha5)$fatal(1,"post-gap escaping/recovery");
  // RESET flushes a frame already draining, not merely the receive buffer.
  for(i=0;i<128;i=i+1)payload[i]=i;
  frame(24,0,128,0);ticks(100);frame(99,1,0,0);base=nl;ticks(12000);
  if(nl!=base)$fatal(1,"pending bytes emitted after RESET");
  payload[0]=8'hcc;frame(0,0,1,0);ticks(200);
  if(nl!=base+1 || lidar[base]!=8'hcc)$fatal(1,"RESET failed to reset sequence baseline");
  // Sequence arithmetic wraps at 255 and a duplicate is a discontinuity.
  frame(90,1,0,0);payload[0]=8'hdd;frame(254,0,1,0);ticks(100);
  before_clear=nclear;base=nl;frame(255,0,1,0);ticks(100);frame(0,0,1,0);ticks(100);
  if(nclear!=before_clear || nl!=base+2)$fatal(1,"sequence rollover rejected");
  frame(0,0,1,0);ticks(100);if(nclear!=before_clear+1 || nl!=base+2)$fatal(1,"duplicate sequence was accepted");
  // Three fast full chunks exceed the bounded two-bank store. Flush both
  // banks and stop the old draining stream until a new validated frame.
  frame(90,1,0,0);for(i=0;i<128;i=i+1)payload[i]=i;
  frame(0,0,128,0);frame(1,0,128,0);before_clear=nclear;frame(2,0,128,0);base=nl;ticks(12000);
  if(nclear!=before_clear+1 || nl!=base)$fatal(1,"buffer overflow retained queued raw bytes");
  payload[0]=8'hee;frame(200,0,1,0);ticks(100);
  if(nl!=base+1 || lidar[base]!=8'hee)$fatal(1,"overflow recovery failed");
  base=nc;before_clear=nclear;send(8'ha5);send(8'hb7);send(1);ticks(5010);
  if(nclear!=before_clear+1 || nc!=base)$fatal(1,"tunnel gap did not clear");
  ticks(10010);send(8'h7d);send(8'h85);send(8'h5a);send(8'h7e);ticks(20);
  if(nc!=base || nclear!=before_clear+1)$fatal(1,"delayed timed-out tunnel remainder escaped quarantine");
  arm_payload(12,0);for(i=0;i<12;i=i+1)send(payload[i]);ticks(20);
  if(armed || narm!=1)$fatal(1,"unexpected ARM");
  // A legacy header is allowed the existing 20ms gap, not tunnel5ms.
  base=nc;arm_payload(13,0);send(payload[0]);ticks(6000);
  for(i=1;i<12;i=i+1)send(payload[i]);ticks(20);
  if(nc-base!=12 || armed)$fatal(1,"legacy header incorrectly used5ms tunnel deadline");
  $display("PASS USB lidar demux: quarantine, CRC/escape/errors, legacy headers, STOP, sequence, RESET and timeout");$finish;
 end
 initial begin #1000000;$fatal(1,"demux timeout");end
endmodule
