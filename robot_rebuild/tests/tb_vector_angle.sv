`timescale 1ns/1ps
module tb_vector_angle;
 reg clk=0;always #5 clk=~clk;reg rst_n=0,clear=0,start=0;
 reg signed [53:0] x=0,y=0;wire valid;wire signed [15:0] angle;
 rb_vector_angle dut(clk,rst_n,clear,start,x,y,valid,angle);
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task point(input signed [53:0] a,b,input integer expected);
  integer error;begin
   @(negedge clk);x=a;y=b;start=1;cycles(1);@(negedge clk);start=0;
   wait(valid);#1;error=$signed(angle)-expected;
   if(error>18000)error=error-36000;if(error< -18000)error=error+36000;
   if(error>4 || error< -4)$fatal(1,"atan2 %d/%d expected=%d got=%d",a,b,expected,angle);
  end
 endtask
 initial begin
  cycles(3);rst_n=1;
  point(1,0,0);point(0,1,9000);point(-1,0,18000);point(0,-1,-9000);
  point(1,1,4500);point(-1,1,13500);point(-1,-1,-13500);point(1,-1,-4500);
  point(54'sd1234567890000,54'sd1234567890000,4500);
  point(-54'sd9007199254740992,0,18000);
  @(negedge clk);x=0;y=0;start=1;cycles(1);start=0;cycles(60);
  if(valid)$fatal(1,"zero vector has no direction");
  $display("PASS vector angle: normalized fixed-point atan2 across all quadrants/scales");$finish;
 end
 initial begin #100000;$fatal(1,"vector angle timeout");end
endmodule
