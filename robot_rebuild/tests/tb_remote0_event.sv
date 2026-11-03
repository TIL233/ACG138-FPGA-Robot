`timescale 1ns/1ps
module tb_remote0_event;
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
  key(8'h16,0);#1;
  if(dut.ir_follow_count!=1 || dut.armed || dut.command!=0)$fatal(1,"0 must publish one event without granting motion");
  key(8'h16,1);key(8'h16,1);#1;
  if(dut.ir_follow_count!=1)$fatal(1,"hold repeat must not queue starts");
  key(8'h1c,0);#1;
  if(dut.ir_stop_epoch!=1 || dut.armed || dut.command!=0)$fatal(1,"5 stop must remain independent");
  key(8'h16,0);#1;
  if(dut.ir_follow_count!=2 || dut.armed)$fatal(1,"new click event after stop");
  force dut.rst_n=0;#1;
  if(dut.ir_follow_count!=0)$fatal(1,"reset event baseline");
  $display("PASS remote0 event: click only, repeats ignored, no local ARM, STOP independent, reset");$finish;
 end
 initial begin #100000;$fatal(1,"remote0 timeout");end
endmodule
