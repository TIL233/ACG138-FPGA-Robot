`timescale 1ns/1ps
module tb_lidar_ranges;
 reg follow=0,lo=1,ro=1,lu=0,ru=0,update=0;reg [15:0] sl=1000,sr=900,front=150;reg [2:0] mask=1;
 wire [15:0] l,r,offl,offr;wire lok,rok,lup,rup,hold,offhold;
 rb_lidar_ranges #(.ENABLE_CONTROL(1)) dut(follow,sl,sr,lo,ro,lu,ru,update,mask,front,l,r,lok,rok,lup,rup,hold);
 rb_lidar_ranges #(.ENABLE_CONTROL(0)) off(follow,sl,sr,lo,ro,lu,ru,update,mask,front,offl,offr,,,,,offhold);
 initial begin
  #1;if(l!=150 || r!=150 || offl!=1000 || offr!=900 || offhold)$fatal(1,"gated range fusion");
  sl=130;#1;if(l!=130)$fatal(1,"lidar erased nearer sonar");
  mask=0;#1;if(l!=130 || r!=900)$fatal(1,"stale lidar reused");
  lo=0;ro=0;#1;if(l!=16'hfffe || r!=16'hfffe || lok || rok)$fatal(1,"sonar missing policy");
  mask=1;update=1;#1;if(!lok || !rok || !lup || !rup)$fatal(1,"lidar freshness propagation");
  follow=1;front=600;#1;if(!hold || l!=16'hfffe || r!=16'hfffe)$fatal(1,"follow 60cm hold/no detour");
  front=601;#1;if(hold)$fatal(1,"follow recovery");
  front=150;mask=0;#1;if(hold)$fatal(1,"expired reading must not latch hold");
  $display("PASS lidar optional fusion, sonar minimum, follow hold and recovery");$finish;
 end
endmodule
