`timescale 1ns/1ps
module tb_nav_config;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,valid=0,allowed=1;reg [7:0] data=0;
 wire wr,commit,reset_pose,clear,ready;wire [2:0] index;
 wire signed [15:0] x,y;wire [3:0] count;wire [7:0] ack_seq,ack_status;
 rb_nav_config_rx #(.CLK_HZ(1000000)) dut(clk,rst_n,data,valid,allowed,
  wr,index,x,y,commit,count,reset_pose,clear,ready,ack_seq,ack_status);
 wire armed,update,fresh,mode_nav,mode_follow;wire [2:0] cmd;wire [7:0] speed;
 rb_command_rx #(.CLK_HZ(1000000)) control(clk,rst_n,data,valid,
  armed,cmd,speed,update,fresh,mode_nav,mode_follow);
 integer writes=0,commits=0,resets=0,clears=0,controls=0;
 always @(posedge clk)begin
  if(wr)writes<=writes+1;if(commit)commits<=commits+1;
  if(reset_pose)resets<=resets+1;if(clear)clears<=clears+1;
  if(update)controls<=controls+1;
 end
 function [15:0] crcbyte(input [15:0] a,input [7:0] b);
  reg [15:0] c;integer k;begin c=a^{b,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;crcbyte=c;end
 endfunction
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task send(input [7:0] b);begin @(negedge clk);data=b;valid=1;cycles(1);@(negedge clk);valid=0;end endtask
 task check(input bit yes,input string msg);if(!yes)$fatal(1,"%s",msg);endtask
 task cfg(input [7:0] seq,op,ix,input signed [15:0] xx,yy,input [7:0] n,res,
          input bit bad,input integer drop);
  reg [7:0] b[0:13];reg [15:0] crc;integer k;
  begin
   b[0]=1;b[1]=seq;b[2]=op;b[3]=ix;b[4]=xx[7:0];b[5]=xx[15:8];
   b[6]=yy[7:0];b[7]=yy[15:8];b[8]=n;
   b[9]=res;b[10]=0;b[11]=0;b[12]=0;b[13]=0;
   send(8'ha5);send(8'hc3);crc=16'hffff;
   for(k=0;k<14;k=k+1)begin
    if(k==drop)allowed=0;
    send(b[k]);crc=crcbyte(crc,b[k]);
    if(k==drop)begin cycles(2);allowed=1;end
   end
   send(crc[7:0]^{7'd0,bad});send(crc[15:8]);send(13);send(10);cycles(3);
  end
 endtask
 task command_frame(input [7:0] seq,arm,command,res);
  reg [7:0] b[0:5];reg [15:0] crc;integer k;
  begin
   b[0]=1;b[1]=seq;b[2]=arm;b[3]=command;b[4]=8;b[5]=res;
   send(8'ha5);send(8'h5a);crc=16'hffff;
   for(k=0;k<6;k=k+1)begin send(b[k]);crc=crcbyte(crc,b[k]);end
   send(crc[7:0]);send(crc[15:8]);send(13);send(10);cycles(3);
  end
 endtask
 reg [15:0] partial_crc;integer k;
 initial begin
  cycles(4);rst_n=1;cycles(4);
  check(!ready && ack_status==255,"reset has no false accepted ACK");
  cfg(0,4,0,0,0,0,0,0,-1);check(clears==1 && ack_seq==0 && ack_status==0,"CLEAR accepted");
  cfg(1,1,0,-10000,10000,0,0,0,-1);
  check(writes==1 && index==0 && x==-10000 && y==10000 && !ready && ack_status==0,"signed coordinate boundaries");
  cfg(2,2,0,0,0,2,0,0,-1);check(commits==0 && ack_status==3,"incomplete bitmap rejects commit");
  cfg(3,1,1,1000,-1000,0,0,0,-1);
  cfg(4,2,0,0,0,2,0,0,-1);check(ready && count==2 && commits==1 && ack_status==0,"complete commit");
  cfg(4,2,0,0,0,2,0,0,-1);check(commits==1 && ack_status==0,"duplicate accepted commit has no side effects");
  cfg(4,1,0,0,0,0,0,0,-1);check(writes==2 && ready && ack_status==2,"same sequence different body rejected");
  allowed=0;cfg(5,4,0,0,0,0,0,0,-1);
  check(clears==1 && ready && ack_status==1,"busy configuration acknowledged without changing route");
  allowed=1;cfg(5,4,0,0,0,0,0,0,-1);
  check(clears==2 && !ready && count==0 && ack_status==0,"busy request may retry after stopping");
  cfg(6,1,0,1000,1000,0,0,0,5);check(writes==2 && ack_status==1,"permission lost during frame rejects write");
  cfg(7,1,0,1000,1000,0,0,1,-1);check(writes==2 && ack_seq==6,"bad CRC cannot write or ACK");
  cfg(7,1,0,10001,0,0,0,0,-1);check(writes==2 && ack_status==2,"out-of-range coordinate rejected");
  cfg(8,1,8,0,0,0,0,0,-1);check(writes==2 && ack_status==2,"out-of-range waypoint index rejected");
  cfg(8,1,7,10000,-10000,0,0,0,-1);check(writes==3 && index==7,"eight point capacity");
  cfg(9,3,0,0,0,0,0,0,-1);cfg(9,3,0,0,0,0,0,0,-1);
  check(resets==1 && ack_status==0,"duplicate pose reset is idempotent");
  cfg(10,2,0,0,0,8,0,0,-1);check(commits==1 && ack_status==3,"sparse point 7 is not complete eight-point route");
  cfg(11,1,0,1000,0,0,0,0,-1);cfg(12,2,0,0,0,1,0,0,-1);
  check(commits==2 && ready && count==1,"one point route valid");
  cfg(13,4,0,0,0,0,1,0,-1);check(clears==2 && ready && ack_status==2,"nonzero reserved rejected");
  // Build a valid RESET, with LF arriving exactly after the timeout boundary.
  send(8'ha5);send(8'hc3);partial_crc=16'hffff;
  send(1);partial_crc=crcbyte(partial_crc,1);send(14);partial_crc=crcbyte(partial_crc,14);
  send(3);partial_crc=crcbyte(partial_crc,3);
  for(k=0;k<11;k=k+1)begin send(0);partial_crc=crcbyte(partial_crc,0);end
  send(partial_crc[7:0]);send(partial_crc[15:8]);send(13);cycles(20000);send(10);cycles(3);
  check(resets==1 && ack_seq==13,"deadline LF cannot complete expired configuration");
  cfg(14,3,0,0,0,0,0,0,-1);check(resets==2 && ack_status==0,"resync after timeout");
  check(controls==0 && !armed && !fresh,"configuration never arms or refreshes command watchdog");
  command_frame(1,1,3,1);check(controls==0,"navigation turn command illegal");
  command_frame(1,0,1,1);check(controls==0,"navigation disarmed forward illegal");
  command_frame(1,1,1,4);check(controls==0,"unknown reserved bit illegal");
  command_frame(1,1,1,1);check(controls==1 && armed && cmd==1 && mode_nav,"explicit navigation start");
  command_frame(2,0,0,1);check(controls==2 && !armed && cmd==0 && mode_nav,"explicit navigation disarm");
  command_frame(3,1,3,0);check(controls==3 && armed && cmd==3 && !mode_nav,"legacy manual mode unchanged");
  check(resets==2 && writes==4,"control packets cannot become route writes");
  $display("PASS navigation configuration: permission/CRC/ACK/bitmap/replay/timeout/mode isolation");$finish;
 end
 initial begin #5000000;$fatal(1,"navigation config hung");end
endmodule
