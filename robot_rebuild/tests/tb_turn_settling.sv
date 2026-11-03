`timescale 1ns/1ps
// Terminal angle is verified after coast/braking, not at an ideal yaw crossing.
module tb_turn_settling;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,armed=0,cmd_update=0;
 reg [2:0] command=0;reg [7:0] speed=16;
 reg [15:0] dl=150,dr=180;reg lu=0,ru=0,yu=1;
 reg signed [31:0] pl=0,pr=0;reg signed [15:0] yaw=0;
 wire signed [15:0] tl,tr,turn_error;wire turn_valid,brake;
 wire [3:0] state;wire [15:0] fault;
 rb_safety_control #(.CLK_HZ(1000),.TURN_TRACK_MM(100),.SETTLE_MS(4),.ADV_COUNTS(10),.SCAN_WAIT_MS(12),
  .STEP_TIMEOUT_MS(100),.TURN_TIMEOUT_MS(200),.ADVANCE_TIMEOUT_MS(100),
  .DETOUR_TIMEOUT_MS(3000),.MAX_TURN_CORRECTIONS(3)) dut(
  .clk(clk),.rst_n(rst_n),.link_ok(1'b1),.imu_ok(1'b1),.left_ok(1'b1),.right_ok(1'b1),
  .estop_ok(1'b1),.armed(armed),.cmd_update(cmd_update),.command(command),.speed(speed),
  .dl(dl),.dr(dr),.left_update(lu),.right_update(ru),.pos_l(pl),.pos_r(pr),.motor_fault(1'b0),
  .yaw(yaw),.yaw_update(yu),.target_l(tl),.target_r(tr),.brake(brake),.state(state),.fault(fault),
  .turn_error(turn_error),.turn_error_valid(turn_valid));
 task step(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit yes,input string msg);
  if(!yes)$fatal(1,"%s state=%0d phase=%0d yaw=%0d target=%0d progress=%0d correction=%b/%0d fault=%h",
   msg,state,dut.phase,yaw,dut.turn_heading,dut.turn_progress,dut.correction_active,dut.correction_count,fault);
 endtask
 task reset_at(input integer heading);begin
  @(negedge clk);rst_n=0;armed=0;cmd_update=0;command=0;speed=16;
  dl=150;dr=180;lu=0;ru=0;yu=1;pl=0;pr=0;yaw=heading;step(3);
  @(negedge clk);rst_n=1;step(3);
  @(negedge clk);cmd_update=1;step(1);
  @(negedge clk);cmd_update=0;armed=1;command=1;step(12);
  check(state==2 && tl==-8 && tr==8,"first leg is directed left90 at bounded speed");
 end endtask
 task move_yaw(input integer angle);
  integer delta,remaining,value;
  begin
   remaining=angle;
   while(remaining!=0)begin
    delta=remaining>0?((remaining>700)?700:remaining):((remaining< -700)?-700:remaining);
    @(negedge clk);value=$signed(yaw)+delta;
    if(value>18000)value=value-36000;if(value< -18000)value=value+36000;
    yaw=value;pl=pl-(delta>0?1:-1);pr=pr+(delta>0?1:-1);
    remaining=remaining-delta;step(3);
   end
  end
 endtask
 task sample(input integer l,input integer r);begin
  @(negedge clk);dl=l;dr=r;lu=1;ru=1;step(1);
  @(negedge clk);lu=0;ru=0;step(3);
 end endtask
 task overshoot_left;begin
  move_yaw(9000);check(state==3 && brake,"brake after provisional left crossing");
  move_yaw(600);step(8);
  check(state==2 && dut.correction_active && !dut.turn_left && tl==2 && tr==-2,
   "settled left overshoot requires a slow right correction");
  check(dut.left_clearance==16'hfffe && !dut.seen_l && !dut.seen_r,
   "off-heading ranges cannot be consumed as a left scan");
 end endtask

 initial begin
  reset_at(0);
  move_yaw(9000);move_yaw(4500);step(8);
  check(state==7 && brake && fault==16'h0400,"135-degree settled heading is rejected rather than accepted as left90");
  check(dut.left_clearance==16'hfffe && dut.phase==0,"large error never records a left sample or starts right scan");

  reset_at(0);move_yaw(9000);step(8);sample(1000,1100);
  move_yaw(-18000);move_yaw(-4500);step(8);
  check(state==7 && brake && fault==16'h0400 && dut.right_clearance==16'hfffe,
   "overshot right sweep also rejects an off-angle settled measurement");

  reset_at(0);move_yaw(9000);
  wait(dut.scan_ms==11);
  @(negedge clk);yaw=9600;dl=1200;dr=1300;lu=1;ru=1;step(1);
  check(state==2 && dut.correction_active && !dut.turn_left && dut.left_clearance==16'hfffe,
   "new off-heading yaw on sonar deadline overrides an older valid settled pose");
  @(negedge clk);lu=0;ru=0;

  reset_at(0);move_yaw(9000);
  wait(dut.scan_ms==11);
  @(negedge clk);yaw=9100;dl=1200;dr=1300;lu=1;ru=1;step(1);
  check(state==2 && !dut.turn_left && dut.turn_goal==18100,
   "next directed leg uses same-edge fresh in-tolerance heading too");
  @(negedge clk);lu=0;ru=0;

  reset_at(0);move_yaw(9000);
  wait(dut.scan_ms==11);
  @(negedge clk);yaw=14000;dl=1200;dr=1300;lu=1;ru=1;step(1);
  check(state==3 && dut.left_clearance==16'hfffe,
   "same-edge implausible IMU jump cannot be consumed as a valid scan");
  step(2);check(state==7 && brake && fault[1],"bad fresh yaw latches IMU failure");
  @(negedge clk);lu=0;ru=0;

  reset_at(0);overshoot_left();
  // Correct to 91 degrees: a real tolerance crossing, not exact 90-degree input.
  move_yaw(-500);check(state==3 && brake,"correction brakes within absolute heading tolerance");
  @(negedge clk);yu=0;step(10);sample(2500,2600);
  check(state==3 && !dut.pose_seen && dut.left_clearance==16'hfffe,
   "old yaw and new sonar alone cannot complete a settled scan");
  @(negedge clk);yu=1;step(3);sample(1000,1100);
  check(state==2 && !dut.correction_active && !dut.turn_left && dut.turn_goal==18100,
   "right sweep retains direction and targets original right90 from actual settled left pose");
  move_yaw(-9000);check(state==2 && tl>0 && tr<0,"main right sweep never stops halfway at original forward heading");
  move_yaw(-9100);check(state==3 && brake,"right sweep brakes at original minus90");
  move_yaw(-500);step(8);
  check(state==2 && dut.correction_active && dut.turn_left && tl==-2 && tr==2,
   "settled right overshoot corrects left using shortest angle");
  move_yaw(500);step(8);sample(1800,1900);
  check(state==4 && !dut.chosen_left && !brake,"only corrected right pose can supply the right clearance and advance");

  // Return must pass the same final-heading check before resuming forward.
  @(negedge clk);pl=pl+10;pr=pr+10;step(3);
  check(state==6 && brake,"advance brakes before returning");
  step(8);check(state==5 && dut.turn_left && dut.turn_heading==0,"return retains original absolute heading");
  move_yaw(9000);check(state==6 && brake,"return crossing is provisional too");
  move_yaw(500);step(8);
  check(state==5 && dut.correction_active && !dut.turn_left && tl==2 && tr==-2,
   "return overshoot must correct before normal forward can resume");
  move_yaw(-500);step(8);sample(1000,1100);
  check(state==0 && tl==16 && tr==16,"corrected return resumes original forward command");

  reset_at(0);move_yaw(9000);step(8);sample(1000,1100);
  move_yaw(-18000);step(8);sample(1800,1900);
  @(negedge clk);pl=pl+10;pr=pr+10;step(11);
  check(state==5,"large return error scenario starts return leg");
  move_yaw(9000);move_yaw(4500);step(8);
  check(state==7 && brake && fault==16'h0400,"large return-heading error cannot resume ordinary forward motion");

  reset_at(0);move_yaw(9000);step(8);sample(2000,2100);
  move_yaw(-18000);step(8);sample(1000,1100);
  check(state==2 && dut.phase==2 && dut.turn_left,"wider left starts directed left180 alignment");
  move_yaw(18000);move_yaw(600);step(8);
  check(state==2 && dut.phase==2 && dut.correction_active && !dut.turn_left && brake==0,
   "selected-left alignment also corrects settled overshoot before departure");
  move_yaw(-600);step(8);sample(1500,1600);
  check(state==4 && dut.chosen_left,"corrected alignment requires a fresh left recheck before advancing");

  reset_at(0);move_yaw(9000);move_yaw(-500);step(8);
  check(state==2 && dut.correction_active && dut.turn_left,"settled undershoot is corrected forward toward target");
  move_yaw(500);step(8);sample(1000,1100);
  check(state==2 && !dut.turn_left && dut.phase==1,"undershoot cannot silently become the left scan pose");

  reset_at(17000);overshoot_left();
  check(dut.turn_heading==-10000,"absolute left target wraps correctly across180");
  move_yaw(-600);step(8);sample(1000,1100);
  check(state==2 && !dut.turn_left && dut.turn_goal==18000 && dut.turn_heading==8000,
   "correction across180 still permits the full directed right180 leg");

  reset_at(0);overshoot_left();
  move_yaw(-1200);step(8);check(state==2 && dut.correction_count==2 && dut.turn_left,"second correction reverses a braking overshoot");
  move_yaw(1200);step(8);check(state==2 && dut.correction_count==3 && !dut.turn_left,"third bounded correction is allowed");
  move_yaw(-1200);step(8);
  check(state==7 && brake && fault==16'h0400,"oscillation has a finite correction limit");

  reset_at(0);overshoot_left();
  @(negedge clk);command=0;#1;check(brake,"operator STOP overrides a correction immediately");
  step(2);check(state==0 && brake,"STOP cancels the complete correction maneuver");

  reset_at(0);move_yaw(7000);check(tl==-4 && tr==4,"last30degrees reduces pivot speed");
  move_yaw(1200);check(tl==-2 && tr==2,"last10degrees further reduces pivot speed");
  @(negedge clk);speed=1;#1;check(tl==-1 && tr==1,"slow pivot never exceeds requested speed cap");
  @(negedge clk);speed=16;move_yaw(620);check(state==2,"below angle threshold keeps turning");
  move_yaw(70);check(state==3 && brake,"non-exact88.9degree crossing brakes without needing an ideal90 sample");
  step(8);sample(1000,1100);
  check(state==2 && !dut.turn_left && dut.turn_goal==17890,"settled pose inside tolerance advances to directed next leg");

  $display("PASS turn settling: reject135, correct both directions/return/wrap, fresh pose/range, bounded oscillation, STOP, speed caps and non-exact crossings");
  $finish;
 end
 initial begin #2000000;$fatal(1,"turn settling test hung");end
endmodule
