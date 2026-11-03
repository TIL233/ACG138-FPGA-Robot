`timescale 1ns/1ps
module tb_follow_speed;
 reg clk=0;always #5 clk=~clk;
 wire tx;wire [7:0] rxbyte;wire rxvalid;
 robot_top #(.CLK_HZ(1000000)) dut(.clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),.uart_tx(tx));
 rb_uart_rx #(.CLK_HZ(1000000)) rx(clk,dut.rst_n,tx,rxbyte,rxvalid);
 reg [7:0] bytes[0:30];integer n=0,captured=0,j;reg [15:0] crc;
 function [15:0] crcbyte(input [15:0] a,input [7:0] b);
  reg [15:0] c;integer k;begin c=a^{b,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;crcbyte=c;end
 endfunction
 always @(posedge clk)if(rxvalid)begin
  if(n==0)begin if(rxbyte==8'h5a)begin bytes[0]<=rxbyte;n<=1;end end
  else if(n==1)begin if(rxbyte==8'hc3)begin bytes[1]<=rxbyte;n<=2;end else n<=(rxbyte==8'h5a)?1:0;end
  else begin bytes[n]<=rxbyte;if(n==30)begin n<=0;captured<=captured+1;end else n<=n+1;end
 end
 task key(input [7:0] k,input rep);
 begin
  @(negedge clk);force dut.ir_key=k;force dut.ir_repeat=rep;force dut.ir_valid=1;
  @(negedge clk);force dut.ir_valid=0;#1;
 end endtask
 initial begin
  force dut.rst_n=0;force dut.ir_valid=0;force dut.uart_follow=0;
  force dut.uart_armed=0;force dut.uart_command=0;force dut.uart_speed=0;
  force dut.uart_update=0;force dut.uart_fresh=1;
  repeat(3)@(negedge clk);force dut.rst_n=1;
  if(dut.ir_follow_speed!=8)$fatal(1,"default follow preset");
  key(8'h15,0);
  if(!dut.ir_selected || dut.remote.ir_speed!=18 || dut.ir_follow_speed!=8 || dut.armed)
   $fatal(1,"manual VOL must retain stopped takeover and manual speed");
  @(negedge clk);force dut.uart_update=1;
  @(negedge clk);force dut.uart_update=0;repeat(2)@(negedge clk);
  force dut.uart_follow=1;force dut.uart_armed=1;force dut.uart_command=1;force dut.uart_speed=8;
  key(8'h15,0);
  if(dut.ir_follow_speed!=10 || dut.ir_selected || !dut.armed || dut.command!=1 || dut.speed!=8)
   $fatal(1,"follow VOL must change preset without stealing UART or inventing speed commands");
  key(8'h15,1);if(dut.ir_follow_speed!=10)$fatal(1,"repeat cannot change preset");
  key(8'h15,0);key(8'h15,0);if(dut.ir_follow_speed!=12)$fatal(1,"upper clamp");
  key(8'h07,0);if(dut.ir_follow_speed!=10)$fatal(1,"one decrement");
  key(8'h07,0);key(8'h07,0);key(8'h07,0);key(8'h07,0);
  if(dut.ir_follow_speed!=4 || dut.remote.ir_speed!=18)$fatal(1,"lower clamp and independent manual preset");
  // A distance hold is still follow mode, and VOL must not restart motion.
  force dut.uart_command=0;force dut.uart_speed=0;
  key(8'h15,0);
  if(dut.ir_follow_speed!=6 || dut.command!=0 || dut.speed!=0 || dut.ir_selected)
   $fatal(1,"hold speed adjustment cannot release stop");
  wait(captured==1);@(negedge clk);
  if(bytes[2]!=5 || bytes[24]!=6 || bytes[25]!=0 || bytes[26]!=0)$fatal(1,"real UART v5 current follow preset");
  crc=16'hffff;for(j=2;j<27;j=j+1)crc=crcbyte(crc,bytes[j]);
  if(bytes[27]!=crc[7:0] || bytes[28]!=crc[15:8] || bytes[29]!=13 || bytes[30]!=10)
   $fatal(1,"real UART v5 CRC and trailer");
  key(8'h1c,0);
  if(dut.armed || dut.command!=0 || !dut.ir_selected || dut.ir_stop_epoch!=1 || dut.ir_follow_speed!=6)
   $fatal(1,"key5 retains immediate stop and saved follow preset");
  key(8'h15,0);
  if(dut.remote.ir_speed!=20 || dut.ir_follow_speed!=6)$fatal(1,"manual VOL after STOP stays manual");
  force dut.rst_n=0;#1;if(dut.ir_follow_speed!=8)$fatal(1,"reset default");
  $display("PASS follow speed: click/repeat/clamps, ownership/STOP/hold, independent presets and real UART v5 CRC");$finish;
 end
 initial begin #6000000;$fatal(1,"follow speed timeout");end
endmodule
