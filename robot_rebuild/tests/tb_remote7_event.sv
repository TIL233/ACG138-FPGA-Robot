`timescale 1ns/1ps
module tb_remote7_event;
 reg clk=0;always #5 clk=~clk;
 robot_top dut(.clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0));
 task key(input [7:0] k,input rep);
 begin
  @(negedge clk);force dut.ir_key=k;force dut.ir_repeat=rep;force dut.ir_valid=1;
  @(negedge clk);force dut.ir_valid=0;
 end endtask
 initial begin
  force dut.rst_n=0;force dut.ir_valid=0;
  repeat(3)@(negedge clk);force dut.rst_n=1;
  key(8'h42,0);#1;
  if(dut.ir_nav_mode_count!=1 || dut.ir_follow_count!=0 || dut.armed || dut.command!=0)
   $fatal(1,"7 must publish one independent event without granting local motion");
  key(8'h42,1);key(8'h42,1);key(8'h4a,0);#1;
  if(dut.ir_nav_mode_count!=1)$fatal(1,"repeat and unused key9 cannot queue navigation starts");
  key(8'h16,0);#1;
  if(dut.ir_nav_mode_count!=1 || dut.ir_follow_count!=1 || dut.armed)$fatal(1,"key0 remains independent");
  key(8'h1c,0);#1;
  if(dut.ir_stop_epoch!=1 || dut.armed || dut.command!=0)$fatal(1,"key5 stop remains independent");
  key(8'h42,0);#1;
  if(dut.ir_nav_mode_count!=2 || dut.armed)$fatal(1,"new key7 click after STOP");
  @(negedge clk);dut.ir_nav_mode_count=16'hffff;
  key(8'h42,0);#1;
  if(dut.ir_nav_mode_count!=16'hffff)$fatal(1,"counter must saturate, not replay on wrap");
  force dut.rst_n=0;#1;
  if(dut.ir_nav_mode_count!=0)$fatal(1,"reset clears event baseline");
  $display("PASS remote7 event: independent click, repeat/saturation/reset, no local ARM, STOP and key0 retained");$finish;
 end
 initial begin #100000;$fatal(1,"remote7 event timeout");end
endmodule
