`timescale 1ns/1ps
module tb_camera_flow;
 reg clk=0,pclk=0,txclk=0,run=1;
 always #5 clk=~clk;
 always #2 if(run)pclk=~pclk;else pclk=0;
 always #3 txclk=~txclk;
 reg rst=0,prst=0,trst=0,vs=0,href=0,start=0,done=0;
 reg [15:0] drops=0;wire [159:0] flow;
 rb_camera_flow #(.CLK_HZ(10000)) dut(clk,rst,pclk,prst,vs,href,txclk,trst,start,done,drops,flow);
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 initial begin
  ticks(4);rst=1;prst=1;trst=1;ticks(1100);
  if(flow[15:0]==0 || flow[95:16]!=0)$fatal(1,"PCLK without frames must be observable");
  @(negedge pclk);href=1;vs=1;drops=1;
  repeat(64)@(negedge pclk);href=0;vs=0;
  @(negedge txclk);start=1;done=1;
  repeat(4)@(negedge txclk);start=0;done=0;
  ticks(1100);
  if(flow[31:16]!=1 || flow[47:32]!=1 || flow[63:48]!=1 || flow[79:64]!=1 || flow[95:80]!=1)$fatal(1,"edge counts/CDC/window deltas");
  if(flow[111:96]!=4 || flow[127:112]!=4)$fatal(1,"high level clocks /16");
  ticks(1100);
  if(flow[143:128]!=64)$fatal(1,"last completed line snapshot");
  run=0;ticks(2200);
  if(flow[127:0]!=0)$fatal(1,"stopped source must not retain old activity");
  rst=0;prst=0;trst=0;ticks(3);rst=1;prst=1;trst=1;ticks(1100);
  if(flow[127:0]!=0)$fatal(1,"reset without a pixel clock must report no activity");
  $display("PASS camera flow: independent clocks, edge counts, CDC, window deltas, missing PCLK/reset");$finish;
 end
 initial begin #1000000;$fatal(1,"flow test timeout");end
endmodule
