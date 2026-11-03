`timescale 1ns/1ps
// Host byte spacing equals 50MHz/115200 8N1. Worst-case escape expansion
// is not the nominal X3 workload; payload below includes every reserved byte.
module tb_usb_lidar_throughput;
 reg clk=0;always #10 clk=~clk;reg rst_n=0,valid=0;reg [7:0] data=0;
 wire [7:0] cd,ld;wire cv,lv,clear,armed,update;wire [2:0] command;
 rb_lidar_bridge_rx dut(clk,rst_n,data,valid,cd,cv,ld,lv,clear);
 rb_command_rx commands(.clk(clk),.rst_n(rst_n),.data(cd),.valid(cv),.armed(armed),.command(command),
  .speed(),.update(update),.fresh(),.mode_nav(),.mode_follow());
 reg [7:0] expected[0:2559];integer received=0,stops=0,clears=0,i,j;
 time started,ended,previous=0;real bytes_per_second;
 always @(posedge clk)begin
  if(lv)begin
   if(ld!==expected[received])$fatal(1,"raw byte ordering/corruption at%0d",received);
   if(previous && $time-previous<86800)$fatal(1,"raw byte pace too fast");
   previous=$time;received=received+1;
  end
  if(update)begin if(armed || command!=0)$fatal(1,"control corruption");stops=stops+1;end
  if(clear)clears=clears+1;
 end
 function [15:0] cb(input [15:0] crc,input [7:0] b);
  reg [15:0] c;integer t;begin c=crc^{b,8'd0};for(t=0;t<8;t=t+1)c=c[15]?(c<<1)^16'h1021:c<<1;cb=c;end
 endfunction
 task send(input [7:0] b);begin
  @(negedge clk);data=b;valid=1;@(negedge clk);valid=0;repeat(4338)@(negedge clk);
 end endtask
 task escaped(input [7:0] b);begin
  if(b==8'ha5 || b==8'h7d || b==8'h7e)begin send(8'h7d);send(b^8'h20);end else send(b);
 end endtask
 task frame(input [7:0] seq);
  reg [15:0] c;integer t;reg [7:0] b;begin
   c=cb(cb(cb(cb(16'hffff,1),seq),0),128);
   send(8'ha5);send(8'hb7);escaped(1);escaped(seq);escaped(0);escaped(128);
   for(t=0;t<128;t=t+1)begin b=t^(seq[0]?8'h80:0);escaped(b);c=cb(c,b);end
   escaped(c[7:0]);escaped(c[15:8]);send(8'h7e);
  end
 endtask
 task stop_frame(input [7:0] seq);
  reg [15:0] c;begin
   c=cb(cb(cb(cb(cb(cb(16'hffff,1),seq),0),0),0),0);
   send(8'ha5);send(8'h5a);send(1);send(seq);send(0);send(0);send(0);send(0);
   send(c[7:0]);send(c[15:8]);send(13);send(10);
  end
 endtask
 initial begin
  for(i=0;i<20;i=i+1)for(j=0;j<128;j=j+1)expected[i*128+j]=j^(i[0]?8'h80:0);
  repeat(5)@(negedge clk);rst_n=1;started=$time;
  for(i=0;i<20;i=i+1)begin frame(i);if(i[0])stop_frame(i);end
  wait(received==2560);ended=$time;repeat(100)@(negedge clk);
  bytes_per_second=2560.0*1000000000.0/(ended-started);
  if(bytes_per_second<9000.0 || stops!=10 || clears || armed)$fatal(1,"throughput/fairness: %0f B/s stops=%0d clears=%0d",bytes_per_second,stops,clears);
  $display("PASS USB lidar throughput: %0f rawB/s,2560 ordered bytes,10 interleaved STOPs,no overflow/ARM",bytes_per_second);$finish;
 end
 initial begin #500000000;$fatal(1,"throughput timeout");end
endmodule
