`timescale 1ns/1ps
module tb_camera_probe;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;wire done,scl,ha,la,timed;wire [2:0]phase;wire [15:0]chip;
 tri1 sda;
 rb_camera_probe #(.CLK_HZ(10000),.SCL_HZ(100)) dut(clk,rst,done,phase,chip,ha,la,timed,scl,sda);
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task reset;begin rst=0;ticks(3);rst=1;ticks(2);end endtask
 task response(input [7:0] value,input bit bad);
 begin
  force dut.value=value;force dut.nack=bad;force dut.finished=1;
  ticks(1);force dut.finished=0;ticks(2);
 end endtask
 initial begin
  force dut.finished=0;reset();
  wait(phase==2);ticks(2);response(8'h56,0);
  wait(phase==4);ticks(2);response(8'h40,0);
  if(!done || !ha || !la || chip!=16'h5640 || timed)$fatal(1,"ID success/byte order");
  reset();wait(phase==2);ticks(2);response(8'hff,1);
  wait(phase==4);ticks(2);response(8'hff,1);
  if(!done || ha || la || chip!=16'hffff)$fatal(1,"NACK must release initializer without claiming ACK");
  reset();ticks(500);
  if(!done || !timed || ha || la)$fatal(1,"hung transaction must timeout and release initializer");
  $display("PASS camera probe: ID/read-only ACK/NACK/bounded timeout/handoff");$finish;
 end
 initial begin #100000;$fatal(1,"test timeout");end
endmodule
