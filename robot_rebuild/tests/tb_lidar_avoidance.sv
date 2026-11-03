`timescale 1ns/1ps
module tb_lidar_avoidance;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,armed=0,cu=0,link=1,lu=0,ru=0,update=0,sides=1;
 reg [2:0] cmd=0;reg [15:0] dl=1000,dr=1000,ll=1500,lr=800;
 reg signed [15:0] yaw=0;reg signed [31:0] pl=0,pr=0;
 wire signed [15:0] tl,tr;wire brake;wire [3:0] state;wire [15:0] fault;
 rb_safety_control #(.CLK_HZ(1000),.USE_LIDAR(1),.TURN_TRACK_MM(100),.SETTLE_MS(4),
  .SCAN_WAIT_MS(12),.STEP_TIMEOUT_MS(100),.TURN_TIMEOUT_MS(150),.ANGLE_TOLERANCE_CD(50)) dut(
  .clk(clk),.rst_n(rst),.armed(armed),.command(cmd),.cmd_update(cu),.speed(8'd16),
  .link_ok(link),.imu_ok(1'b1),.left_ok(1'b1),.right_ok(1'b1),.estop_ok(1'b1),
  .dl(dl),.dr(dr),.left_update(lu),.right_update(ru),.pos_l(pl),.pos_r(pr),.motor_fault(1'b0),
  .yaw(yaw),.yaw_update(1'b1),.target_l(tl),.target_r(tr),.brake(brake),.state(state),.fault(fault),
  .lidar_update(update),.lidar_sides_valid(sides),.lidar_left(ll),.lidar_right(lr),.lidar_front_valid(1'b1));
 task tick(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task reset_run;begin
  @(negedge clk);rst=0;armed=0;cmd=0;cu=0;dl=1000;dr=1000;yaw=0;pl=0;pr=0;link=1;update=0;lu=0;ru=0;
  tick(3);@(negedge clk);rst=1;tick(3);@(negedge clk);cu=1;tick(1);
  @(negedge clk);cu=0;armed=1;cmd=1;tick(3);
  @(negedge clk);dl=150;dr=150;tick(12);
 end endtask
 task scan;begin @(negedge clk);update=1;tick(1);@(negedge clk);update=0;tick(2);end endtask
 task turn(input integer sign);
  integer j;begin for(j=0;j<9;j=j+1)begin @(negedge clk);yaw=yaw+sign*1000;pl=pl-sign;pr=pr+sign;tick(3);end tick(2);end
 endtask
 initial begin
  sides=1;ll=1500;lr=800;reset_run();
  if(state!=1 || !brake)$fatal(1,"must wait stationary scan");
  scan();if(state!=1)$fatal(1,"moving revolution accepted");
  scan();if(state!=2 || dut.phase!=2 || tl>=0 || tr<=0 || dut.turn_goal!=9000)$fatal(1,"left direct selection");
  turn(1);if(state!=3)$fatal(1,"left settle");
  tick(6);@(negedge clk);dl=1000;dr=1000;lu=1;ru=1;tick(3);
  if(state!=3 || !brake)$fatal(1,"must not use in-turn scan for clearance");
  scan();if(state!=3)$fatal(1,"first post-turn publication still mixed");scan();
  if(state!=4 || tl<=0 || tr<=0)$fatal(1,"direct turn must advance, not right180 scan");
  @(negedge clk);armed=0;cmd=0;cu=1;tick(2);if(!brake || state!=0)$fatal(1,"STOP priority");
  ll=800;lr=1600;reset_run();scan();scan();
  if(state!=2 || dut.chosen_left || tl<=0 || tr>=0)$fatal(1,"right direct selection");
  @(negedge clk);link=0;tick(2);if(!brake || fault==0)$fatal(1,"link loss priority");
  ll=250;lr=300;reset_run();scan();scan();if(state!=7 || fault!=16'h0800 || !brake)$fatal(1,"both sides blocked");
  sides=0;reset_run();if(state!=2 || dut.phase!=0 || tl>=0)$fatal(1,"missing lidar must retain sonar scan");
  $display("PASS lidar settled direct choice, no survey sweep, forward recheck, fallback, STOP and faults");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
