`timescale 1ns/1ps
module tb_lidar_uart;
 reg clk=0;always #10 clk=~clk;reg rst=0,rx=1;
 wire update,fresh;wire [2:0] mask;wire [15:0] front,left,right,id,good,bad;
 rb_lidar_x3 dut(clk,rst,rx,update,fresh,mask,front,left,right,id,good,bad);
 integer points=0;
 always @(posedge clk)if(dut.pv)begin
  if(dut.distance!=7161 || dut.angle!=0 || dut.flags!=0 || !dut.start)$fatal(1,"wire point disagrees with manual E4 6F example");
  points=points+1;
 end
 task bit_time;repeat(434)@(negedge clk);endtask
 task send(input [7:0] b);
  integer i;begin rx=0;bit_time();for(i=0;i<8;i=i+1)begin rx=b[i];bit_time();end rx=1;bit_time();end
 endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;repeat(5)@(negedge clk);
  // Literal independently calculated XOR 55AA ^ 0101 ^ 0001 ^ 0001 ^ 6FE4 = 3B4F.
  send('haa);send('h55);send('h01);send('h01);send('h01);send('h00);
  send('h01);send('h00);send('h4f);send('h3b);send('he4);send('h6f);
  repeat(1000)@(negedge clk);
  if(points!=1 || good!=1 || bad!=0 || fresh || mask!=0)$fatal(1,"UART acquisition/partial scan");
  $display("PASS lidar 50MHz/115200 physical UART, literal checksum and documented distance example");$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
