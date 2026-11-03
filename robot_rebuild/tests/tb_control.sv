`timescale 1ns/1ps
module tb_control;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,link_ok=1,imu_ok=1,left_ok=1,right_ok=1,estop_ok=1;
 reg armed=0,cmd_update=0;reg [2:0] command=0;reg [7:0] speed=16;
 reg [15:0] dl=1000,dr=1000;reg lu=0,ru=0,yu=1;
 reg signed [31:0] pl=0,pr=0;reg mf=0;reg signed [15:0] yaw=0;
 wire signed [15:0] tl,tr;wire brake;wire [3:0] state;wire [15:0] fault;
 rb_safety_control #(.CLK_HZ(1000),.TURN_TRACK_MM(100),.SETTLE_MS(4),.ADV_COUNTS(10),.SCAN_WAIT_MS(12),
  .STEP_TIMEOUT_MS(60),.TURN_TIMEOUT_MS(150),.ADVANCE_TIMEOUT_MS(60),
  .DETOUR_TIMEOUT_MS(1000),.ANGLE_TOLERANCE_CD(50)) dut(
  .clk(clk),.rst_n(rst_n),.link_ok(link_ok),.imu_ok(imu_ok),.left_ok(left_ok),.right_ok(right_ok),
  .estop_ok(estop_ok),.armed(armed),.cmd_update(cmd_update),.command(command),.speed(speed),
  .dl(dl),.dr(dr),.left_update(lu),.right_update(ru),.pos_l(pl),.pos_r(pr),.motor_fault(mf),
  .yaw(yaw),.yaw_update(yu),.target_l(tl),.target_r(tr),.brake(brake),.state(state),.fault(fault));
 wire signed [15:0] mirrored_yaw=-yaw;
 wire signed [15:0] mirror_l,mirror_r;
 wire mirror_brake;wire [3:0] mirror_state;wire [15:0] mirror_fault;
 rb_safety_control #(.CLK_HZ(1000),.TURN_TRACK_MM(100),.SETTLE_MS(4),.ADV_COUNTS(10),.SCAN_WAIT_MS(12),
  .STEP_TIMEOUT_MS(60),.TURN_TIMEOUT_MS(150),.ADVANCE_TIMEOUT_MS(60),
  .DETOUR_TIMEOUT_MS(1000),.ANGLE_TOLERANCE_CD(50),.YAW_SIGN(-1)) mirror(
  .clk(clk),.rst_n(rst_n),.link_ok(link_ok),.imu_ok(imu_ok),.left_ok(left_ok),.right_ok(right_ok),
  .estop_ok(estop_ok),.armed(armed),.cmd_update(cmd_update),.command(command),.speed(speed),
  .dl(dl),.dr(dr),.left_update(lu),.right_update(ru),.pos_l(pl),.pos_r(pr),.motor_fault(mf),
  .yaw(mirrored_yaw),.yaw_update(yu),.target_l(mirror_l),.target_r(mirror_r),
  .brake(mirror_brake),.state(mirror_state),.fault(mirror_fault));
 always @(posedge clk)begin
  #1;if(rst_n && {mirror_l,mirror_r,mirror_brake,mirror_state,mirror_fault}!=={tl,tr,brake,state,fault})
   $fatal(1,"opposite IMU sign with YAW_SIGN=-1 must preserve physical scan behavior");
 end
 task step(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit yes,input string msg);
  if(!yes)$fatal(1,"%s state=%0d phase=%0d yaw=%0d progress=%0d goal=%0d valid=%b%b scan_ms=%0d fault=%h",msg,state,dut.phase,yaw,dut.turn_progress,dut.turn_goal,left_ok,right_ok,dut.scan_ms,fault);
 endtask
 task disarm;begin
  @(negedge clk);armed=0;cmd_update=1;command=0;step(1);
  @(negedge clk);cmd_update=0;step(2);
 end endtask
 task reset_at(input integer heading);begin
  @(negedge clk);rst_n=0;armed=0;cmd_update=0;command=0;speed=16;
  link_ok=1;imu_ok=1;estop_ok=1;left_ok=1;right_ok=1;dl=1000;dr=1000;
  lu=0;ru=0;yu=1;pl=0;pr=0;mf=0;yaw=heading;
  step(3);@(negedge clk);rst_n=1;step(3);disarm;
  @(negedge clk);armed=1;command=1;step(2);
 end endtask
 task begin_scan;begin
  dl=150;dr=180;step(2);check(state==1 && brake,"near must stop before scanning");
  step(10);check(state==2 && tl<0 && tr>0 && dut.turn_goal==9000,"always scan left90 first");
 end endtask
 task turn_by(input integer angle);
  integer delta,remaining,value;
  begin
   remaining=angle;
   while(remaining!=0)begin
    delta=(remaining>0)?((remaining>1000)?1000:remaining):((remaining< -1000)?-1000:remaining);
    @(negedge clk);value=$signed(yaw)+delta;
    if(value>18000)value=value-36000;
    if(value< -18000)value=value+36000;
    yaw=value;pl=pl-((delta>0)?1:-1);pr=pr+((delta>0)?1:-1);
    remaining=remaining-delta;step(3);
   end
   step(2);
  end
 endtask
 task sample(input integer l,input integer r);begin
  @(negedge clk);dl=l;dr=r;lu=1;ru=1;step(1);
  @(negedge clk);lu=0;ru=0;step(2);
 end endtask
 task scan_left(input integer l,input integer r);begin
  turn_by(9000);check(state==3 && brake,"left90 must stop before measuring");
  step(6);sample(l,r);
  check(state==2 && tl>0 && tr<0 && dut.turn_goal==18000,"after left scan, turn right180");
 end endtask
 task scan_right(input integer l,input integer r);begin
  turn_by(-18000);check(state==3 && brake,"right180 must stop at original right side");
  step(6);sample(l,r);
 end endtask
 task finish_left;begin
  check(state==2 && tl<0 && tr>0,"left chosen: rotate left180 from right scan pose");
  turn_by(18000);check(state==3 && brake,"selected left side must be rechecked at rest");
  step(6);sample(1200,1300);check(state==4 && tl>0 && tr>0,"fresh chosen-left samples allow advance");
 end endtask
 task bypass_and_return(input integer angle);begin
  pl=pl+10;pr=pr+10;step(3);check(state==6 && brake,"stop after bounded advance before restoring heading");
  step(8);check(state==5 && !brake,"restore original heading using IMU");
  turn_by(angle);check(state==6 && brake,"settle after restored heading");
  step(6);sample(1000,1000);
  if(!left_ok || !right_ok)step(16);
  check(state==0 && tl==16 && tr==16,"resume original forward command");
 end endtask

 initial begin
  reset_at(0);check(!brake && tl==16,"forward remains available");
  dl=350;step(2);check(tl==8 && tr==8,"slowdown remains below40cm");
  begin_scan();scan_left(1200,1300);
  turn_by(-9000);check(state==2 && tl>0 && tr<0,"right scan must complete180, not stop at halfway");
  turn_by(-9000);check(state==3 && brake,"right scan completes at minus90");
  step(6);sample(600,700);check(dut.left_clearance==1200 && dut.right_clearance==600,"compare minimum of both forward probes at each pose");
  finish_left();bypass_and_return(-9000);

  reset_at(0);begin_scan();scan_left(500,600);scan_right(1400,1500);
  check(state==4 && !dut.chosen_left && tl>0 && tr>0,"right wider: already facing right, go without another turn");
  bypass_and_return(9000);

  reset_at(0);begin_scan();scan_left(800,900);scan_right(800,900);
  check(state==2 && dut.chosen_left,"equal distances deterministically prefer left");
  finish_left();
  dl=150;step(2);check(state==7 && brake && fault==16'h1000,"new valid near obstacle aborts advance");

  reset_at(0);begin_scan();scan_left(399,500);scan_right(350,600);
  check(state==7 && brake && fault==16'h0800,"both valid sides below40cm must not advance");
  dl=2000;dr=2000;step(10);check(brake && fault!=0,"clearance recovery alone cannot restart failed maneuver");

  reset_at(0);begin_scan();scan_left(399,500);scan_right(400,500);
  check(state==4 && !dut.chosen_left,"exact40cm right clearance is allowed");

  reset_at(0);begin_scan();scan_left(1600,1700);scan_right(800,900);
  turn_by(18000);step(6);sample(150,200);
  check(state==7 && brake,"left becomes blocked during right scan: recheck prevents stale-left departure");

  reset_at(0);begin_scan();left_ok=0;right_ok=0;dl=100;dr=100;
  turn_by(9000);step(25);
  check(state==2 && !dut.turn_left && dut.left_clearance==16'hfffe,"no frames at left pose means FAR after finite window");
  turn_by(-18000);step(25);
  check(state==2 && dut.turn_left && dut.right_clearance==16'hfffe,"both missing means FAR, tie left");
  turn_by(18000);step(25);check(state==4 && !brake && fault==0,"missing both probes permits actual departure");
  bypass_and_return(-9000);

  reset_at(0);begin_scan();turn_by(9000);right_ok=0;step(6);sample(650,100);
  step(16);check(dut.left_clearance==650 && state==2,"one missing probe does not hide the other valid distance");
  right_ok=1;scan_right(1200,1300);check(state==4,"one-sided scan chooses measured wider right");

  reset_at(0);begin_scan();turn_by(9000);step(6);
  lu=1;ru=0;dl=100;step(1);lu=0;left_ok=0;right_ok=0;step(18);
  check(dut.left_clearance==16'hfffe && state==2,"invalidated scan sample must not retain old near value");

  reset_at(0);begin_scan();lu=1;ru=1;dl=2000;dr=2000;step(1);lu=0;ru=0;
  turn_by(9000);step(6);
  check(state==3 && dut.left_clearance==16'hfffe,"updates during rotation cannot become stopped scan");
  sample(450,460);check(dut.left_clearance==450,"fresh stopped frame replaces turn-time data");

  reset_at(0);begin_scan();turn_by(9000);step(6);
  wait(dut.scan_ms==11);
  sample(399,500);
  check(dut.left_clearance==399,"frame on final scan tick must override missing-FAR fallback");

  reset_at(0);begin_scan();turn_by(9000);step(6);
  @(negedge clk);pl=pl+1;lu=1;ru=1;dl=500;dr=500;step(1);
  check(state==3 && brake,"same-cycle encoder movement invalidates a stopped scan");
  @(negedge clk);lu=0;ru=0;step(6);sample(600,700);
  check(state==2 && dut.left_clearance==600,"fresh sample after new stationary window is required");

  reset_at(0);dl=150;dr=180;step(2);
  repeat(8)begin @(negedge clk);pl=pl+1;pr=pr+1;step(1);end
  check(state==1 && brake,"encoder activity prevents stop-to-turn race");
  step(10);check(state==2,"turn begins only after a new stationary interval");
  command=0;#1;check(brake,"operator STOP gates turn output immediately");step(2);
  check(state==0 && brake,"operator STOP cancels the whole scan");

  reset_at(17000);begin_scan();scan_left(500,600);scan_right(1600,1700);
  check(state==4 && !dut.chosen_left,"IMU +/-180 wrap preserves left90 and directed right180");
  bypass_and_return(9000);
  check($signed(yaw)==17000,"return preserves original wrapped heading");

  // Seed near the signed counter boundary before the maneuver. Teleporting
  // from zero to 2^31 during a stopped scan is an encoder fault, not wrap.
  reset_at(0);@(negedge clk);pl=32'h7fffffff;pr=32'h7fffffff;step(3);
  begin_scan();scan_left(500,600);turn_by(-18000);step(8);sample(1500,1600);
  check(state==4,"wrapped encoder start enters advance");
  bypass_and_return(9000);

  reset_at(0);begin_scan();step(152);check(state==7 && fault==16'h0400,"no angular progress times out");
  reset_at(0);begin_scan();yaw=-1000;step(4);check(state==7 && fault==16'h0400,"wrong IMU direction fails rather than turning indefinitely");
  reset_at(0);begin_scan();yaw=4000;step(4);check(state==7 && fault[1],"implausible yaw jump cancels maneuver");
  reset_at(0);begin_scan();imu_ok=0;#1;check(brake,"IMU loss gates output immediately");step(2);check(fault[1],"IMU failure latches");
  imu_ok=1;step(3);check(brake,"IMU reconnect cannot auto-resume");
  reset_at(0);begin_scan();link_ok=0;#1;check(brake,"command watchdog immediately stops scan");step(2);check(fault[0],"link failure latches");
  reset_at(0);begin_scan();estop_ok=0;step(2);check(brake && fault[4],"stop permission wins during rotation");
  reset_at(0);begin_scan();mf=1;step(2);check(brake && fault[5],"motor fault wins during rotation");
  reset_at(0);begin_scan();@(negedge clk);dut.total_ms=1000;mirror.total_ms=1000;step(2);
  check(state==7 && fault==16'h0100,"complete maneuver has finite total timeout");
  reset_at(0);left_ok=0;right_ok=0;dl=100;dr=100;step(4);
  check(state==0 && !brake && fault==0,"missing probes allow ordinary motion too");
  left_ok=1;right_ok=1;command=2;step(2);check(tl<0 && tr<0,"reverse can leave near obstacle");
  $display("PASS control: IMU left90/right180, wider/tie/blocked, fresh pose samples, missing FAR, return, wrap, STOP/fault/timeout");$finish;
 end
 initial begin #1000000;$fatal(1,"control scan test hung");end
endmodule
