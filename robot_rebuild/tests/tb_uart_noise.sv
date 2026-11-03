`timescale 1ns/1ps
module tb_uart_noise;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,rx=1;
 wire [7:0] data;wire valid;
 rb_uart_rx #(.CLK_HZ(1000000)) dut(clk,rst_n,rx,data,valid);
 integer count=0;
 reg [7:0] bytes[0:3];
 always @(posedge clk) if(valid) begin bytes[count]=data;count=count+1;end
 task ticks(input integer n);repeat(n) @(negedge clk);endtask
 task frame(input [7:0] b,input bit good_stop);
  integer i;
  begin
   rx=0;ticks(9);
   for(i=0;i<8;i=i+1) begin rx=b[i];ticks(9);end
   rx=good_stop;ticks(9);
  end
 endtask
 initial begin
  ticks(4);rst_n=1;ticks(4);
  // A short glitch is rejected by the middle-of-start-bit check.
  rx=0;ticks(2);rx=1;ticks(20);
  if(count!=0) $fatal(1,"false start emitted a byte");
  // A sustained break and an invalid stop bit cannot produce valid bytes.
  rx=0;ticks(400);rx=1;ticks(30);
  if(count!=0) $fatal(1,"break emitted bytes");
  frame(8'hff,0);rx=1;ticks(30);
  if(count!=0) $fatal(1,"framing error emitted a byte");
  // Recover after malformed traffic and receive gapless 8N1 bytes.
  frame(8'h00,1);frame(8'h55,1);frame(8'haa,1);frame(8'hff,1);
  rx=1;ticks(20);
  if(count!=4 || bytes[0]!=0 || bytes[1]!=8'h55 || bytes[2]!=8'haa || bytes[3]!=8'hff)
   $fatal(1,"UART break recovery/back-to-back bytes count=%0d",count);
  $display("PASS UART: false start/break/framing rejection and gapless recovery");
  $finish;
 end
 initial begin #100000;$fatal(1,"test hung");end
endmodule
