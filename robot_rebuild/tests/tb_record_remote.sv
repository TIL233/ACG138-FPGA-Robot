`timescale 1ns/1ps
module tb_record_remote;
 reg clk=0;always #5 clk=~clk;
 wire tx;wire [7:0] data;wire valid;
 robot_top #(.CLK_HZ(1000000)) dut(.clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),.uart_tx(tx));
 rb_uart_rx #(.CLK_HZ(1000000)) rx(clk,dut.rst_n,tx,data,valid);
 reg [7:0] bytes[0:30];integer n=0,complete=0,j;reg [15:0] crc;
 function [15:0] crcbyte(input [15:0] a,input [7:0] b);
  reg [15:0] c;integer k;begin c=a^{b,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;crcbyte=c;end
 endfunction
 always @(posedge clk)if(valid)begin
  if(n==0)begin if(data==8'h5a)begin bytes[0]<=data;n<=1;end end
  else if(n==1)begin if(data==8'hc3)begin bytes[1]<=data;n<=2;end else n<=(data==8'h5a)?1:0;end
  else begin bytes[n]<=data;if(n==30)begin n<=0;complete<=complete+1;end else n<=n+1;end
 end
 task key(input [7:0] k,input rep);begin
  @(negedge clk);force dut.ir_key=k;force dut.ir_repeat=rep;force dut.ir_valid=1;
  @(negedge clk);force dut.ir_valid=0;#1;
 end endtask
 initial begin
  force dut.rst_n=0;force dut.ir_valid=0;
  repeat(3)@(negedge clk);force dut.rst_n=1;
  key(8'h4a,0);key(8'h4a,1);key(8'h4a,1);
  if(dut.ir_record_count!=1 || dut.armed || dut.command!=0)$fatal(1,"key9 click publishes once, no ARM");
  key(8'h1c,0);if(dut.ir_record_count!=1 || dut.ir_stop_epoch!=1)$fatal(1,"STOP does not invent a recording toggle");
  wait(complete==1);@(negedge clk);
  if(bytes[2]!=5 || bytes[24]!=8 || bytes[25]!=1 || bytes[26]!=0)$fatal(1,"record event and speed on real v5 UART");
  crc=16'hffff;for(j=2;j<27;j=j+1)crc=crcbyte(crc,bytes[j]);
  if(bytes[27]!=crc[7:0] || bytes[28]!=crc[15:8] || bytes[29]!=13 || bytes[30]!=10)$fatal(1,"record event CRC/trailer");
  @(negedge clk);dut.ir_record_count=16'hffff;
  key(8'h4a,0);if(dut.ir_record_count!=16'hffff)$fatal(1,"record count saturates without wrap/replay");
  force dut.rst_n=0;#1;if(dut.ir_record_count!=0)$fatal(1,"record reset baseline");
  $display("PASS recording remote: one click event, repeats/saturation/reset, STOP independence, no ARM and real UART v5 CRC");$finish;
 end
 initial begin #6000000;$fatal(1,"record remote timeout");end
endmodule
