`timescale 1ns/1ps
module tb_lidar_packet;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,valid=0;reg [7:0] data=0;
 wire pv,start,ok,err;wire [15:0] angle,distance;wire [1:0] flags;
 rb_lidar_packet #(.CLK_HZ(1000000)) dut(clk,rst,data,valid,pv,start,angle,distance,flags,ok,err);
 integer seen=0,errors=0,packets=0;reg [15:0] angles[0:1023],ranges[0:1023];reg [1:0] observed_flags[0:1023];reg starts[0:1023];
 always @(posedge clk)begin
  if(pv)begin angles[seen]=angle;ranges[seen]=distance;observed_flags[seen]=flags;starts[seen]=start;seen=seen+1;end
  if(err)errors=errors+1;if(ok)packets=packets+1;
 end
 task tick(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task byte_in(input [7:0] b);begin @(negedge clk);data=b;valid=1;tick(1);@(negedge clk);valid=0;tick(1);end endtask
 task word_in(input [15:0] w);begin byte_in(w[7:0]);byte_in(w[15:8]);end endtask
 task packet(input integer n,input integer a,input integer b,input bit startbit,input bit corrupt,input bit badangle);
  reg [15:0] f,l,cs,w;integer j;begin
   f=(a<<1)|!badangle;l=(b<<1)|1;
   cs=16'h55aa^{n[7:0],7'd0,startbit}^f^l;
   for(j=0;j<n;j=j+1)begin w=((1000+j)<<2)|(j%4);cs=cs^w;end
   byte_in(8'haa);byte_in(8'h55);byte_in(startbit);byte_in(n);
   word_in(f);word_in(l);word_in(cs^corrupt);
   for(j=0;j<n;j=j+1)word_in(((1000+j)<<2)|(j%4));
   tick(2*n+25);
  end
 endtask
 integer base;
 initial begin
  tick(3);rst=1;tick(3);
  byte_in(8'ha5);byte_in(8'h5a);byte_in(8'h00); // startup/noise is not a scan
  packet(3,350*64,10*64,1,0,0);
  if(seen!=3 || angles[0]!=350*64 || angles[1]!=0 || angles[2]!=10*64 || !starts[0] || starts[1])$fatal(1,"wrap/start interpolation");
  if(ranges[2]!=1002 || observed_flags[2]!=2)$fatal(1,"distance/flags");
  packet(4,0,10,0,0,0);
  if(angles[4]!=3 || angles[5]!=6 || angles[6]!=10)$fatal(1,"fractional endpoint interpolation");
  base=seen;packet(3,0,128,0,1,0);
  if(seen!=base || errors!=1)$fatal(1,"bad XOR leaked points");
  packet(1,0,0,1,0,1);
  if(seen!=base || errors!=2)$fatal(1,"invalid angle check bit");
  packet(1,1234,1234,1,0,0);
  if(seen!=base+1 || angles[base]!=1234)$fatal(1,"single point");
  base=seen;packet(255,100,1000,0,0,0);
  if(seen!=base+255 || angles[base+254]!=1000)$fatal(1,"255 samples endpoint");
  byte_in(8'haa);byte_in(8'h55);byte_in(0);byte_in(0);tick(5);
  if(errors!=3)$fatal(1,"zero count");
  byte_in(8'haa);byte_in(8'h55);byte_in(0);tick(5010);
  if(errors!=4)$fatal(1,"partial frame timeout");
  base=seen;packet(2,0,180*64,0,0,0);
  if(errors!=5 || seen!=base)$fatal(1,"impossible packet span");
  packet(2,0,64,0,0,0);
  if(seen!=base+2 || packets!=5)$fatal(1,"recovery");
  $display("PASS lidar packet checksum, wrap, interpolation, flags, max count, timeout and recovery");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
