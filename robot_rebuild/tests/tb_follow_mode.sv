`timescale 1ns/1ps
module tb_follow_mode;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,valid=0;reg [7:0] data=0;
 wire armed,update,fresh,nav,follow;wire [2:0] command;wire [7:0] speed;
 rb_command_rx #(.CLK_HZ(1000000)) dut(clk,rst,data,valid,armed,command,speed,update,fresh,nav,follow);
 integer updates=0;always @(posedge clk)if(update)updates<=updates+1;
 function [15:0] crcbyte(input [15:0] a,input [7:0] b);
  reg [15:0] c;integer j;begin c=a^{b,8'd0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;crcbyte=c;end
 endfunction
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task send(input [7:0] b);begin @(negedge clk);data=b;valid=1;ticks(1);@(negedge clk);valid=0;end endtask
 task packet(input [7:0] seq,input bit arm,input [7:0] cmd,input [7:0] flags,input bit bad);
  reg [7:0] body[0:5];reg [15:0] crc;integer k;
  begin body[0]=1;body[1]=seq;body[2]=arm;body[3]=cmd;body[4]=8;body[5]=flags;
   crc=16'hffff;send(8'ha5);send(8'h5a);
   for(k=0;k<6;k=k+1)begin send(body[k]);crc=crcbyte(crc,body[k]);end
   send(crc[7:0]^bad);send(crc[15:8]);send(13);send(10);ticks(4);
  end
 endtask
 initial begin
  ticks(4);rst=1;ticks(4);
  packet(1,1,1,2,0);
  if(!armed || !follow || nav || command!=1 || updates!=1)$fatal(1,"follow flag decoded");
  packet(2,1,1,3,0);packet(2,1,2,2,0);packet(2,1,1,4,0);packet(2,1,1,0,1);
  if(updates!=1 || !follow || nav)$fatal(1,"conflicting modes/reverse/unknown flags/bad CRC rejected");
  packet(2,1,3,2,0);
  if(command!=3 || !follow || updates!=2)$fatal(1,"follow arc decoded");
  packet(3,0,0,2,0);
  if(armed || command!=0 || !follow)$fatal(1,"follow explicit stop decoded");
  packet(4,0,0,0,0);
  if(armed || follow || nav)$fatal(1,"legacy disarm clears mode");
  packet(5,1,1,1,0);
  if(!nav || follow || !armed)$fatal(1,"navigation remains independent");
  $display("PASS follow mode: explicit CRC-protected mode, mutual exclusion, no reverse, legacy stop, navigation");$finish;
 end
 initial begin #1000000;$fatal(1,"follow mode timeout");end
endmodule
