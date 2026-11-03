`timescale 1ns/1ps
module tb_lidar_sectors;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,pv=0,start=0,err=0,ok=0;reg [15:0] a=0,d=0;reg [1:0] f=0;
 wire update,fresh;wire [2:0] mask;wire [15:0] front,left,right,id,good,bad;
 rb_lidar_sectors #(.CLK_HZ(1000),.TIMEOUT_MS(60),.MIN_SCAN_POINTS(8),.MIN_SECTOR_POINTS(2)) dut(
 clk,rst,pv,start,err,a,d,f,update,fresh,mask,front,left,right,id,good,bad,ok);
 task tick(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task point(input integer angle,input integer range_mm,input bit first,input [1:0] flags);
  begin @(negedge clk);pv=1;a=angle*64;d=range_mm;start=first;f=flags;tick(1);@(negedge clk);pv=0;start=0;tick(1);end
 endtask
 task rest_scan;begin
  point(5,700,0,0);point(270,1200,0,0);point(280,1300,0,0);
  point(90,800,0,0);point(100,900,0,0);point(180,2000,0,0);point(190,2100,0,0);
 end endtask
 initial begin
  tick(3);rst=1;tick(3);
  point(0,500,0,0);rest_scan();if(fresh || id!=0)$fatal(1,"partial boot revolution published");
  point(0,500,1,0);rest_scan();point(0,700,1,0);tick(2);
  if(!fresh || mask!=7 || front!=500 || left!=1200 || right!=800 || id!=1)$fatal(1,"only front/left/right sector minima required");
  @(negedge clk);err=1;tick(1);@(negedge clk);err=0;
  rest_scan();point(0,0,1,0);if(id!=1 || bad!=1)$fatal(1,"dirty scan published");
  // Invalid/no-return/interference points do not create a clear sector.
  point(1,0,0,0);point(270,1000,0,2);point(280,1000,0,3);
  point(90,119,0,0);point(100,8001,0,0);point(180,0,0,0);point(190,0,0,0);
  point(0,500,1,0);tick(2);
  if(id!=2 || mask!=0 || front!=16'hffff)$fatal(1,"invalid points became clearance");
  rest_scan();point(0,500,1,0);tick(2);if(mask!=7)$fatal(1,"recovered scan");
  tick(65);if(fresh || mask!=0)$fatal(1,"unplug must expire");
  // A boundary after a long pause cannot revive the accumulated old scan.
  point(0,500,1,0);tick(2);if(fresh)$fatal(1,"old interrupted scan revived");
  // Pure rear returns cannot populate any of the three requested sectors.
  point(180,120,1,0);repeat(7)point(190,130,0,0);point(180,120,1,0);tick(2);
  if(!fresh || mask!=0 || front!=16'hffff || left!=16'hffff || right!=16'hffff)$fatal(1,"rear-only returns populated a driving sector");
  $display("PASS lidar three sectors, rear ignored, startup, contamination, unknown, recovery, expiry");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
