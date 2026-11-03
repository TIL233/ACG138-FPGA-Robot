`timescale 1ns/1ps
// Fault injection at the safety controller's real encoder/IMU interface.
// A 100mm track and 7640 counts/m give approximately 20 total wheel counts
// per 1.5 degrees. The independent observer below counts physical wheel
// travel; it does not reuse the guard's arithmetic or internal state.
module tb_turn_guard_integration;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,armed=0,cmd_update=0;
 reg [2:0] command=0;reg [7:0] speed=16;
 reg [15:0] dl=1000,dr=1000;reg lu=0,ru=0,yu=1;
 reg signed [31:0] pl=0,pr=0;
 reg signed [15:0] yaw=0;
 wire signed [15:0] tl,tr;
 wire brake;wire [3:0] state;wire [15:0] fault;
 rb_safety_control #(.CLK_HZ(1000),.SETTLE_MS(4),.ADV_COUNTS(3056),
  .SCAN_WAIT_MS(100),.STEP_TIMEOUT_MS(1000),.TURN_TIMEOUT_MS(20000),
  .ADVANCE_TIMEOUT_MS(10000),.DETOUR_TIMEOUT_MS(200000),
  .TURN_TRACK_MM(100),.TURN_COUNTS_PER_M(7640),.TURN_TRAVEL_MARGIN_CD(3000)) dut(
  .clk(clk),.rst_n(rst_n),.link_ok(1'b1),.imu_ok(1'b1),
  .left_ok(1'b1),.right_ok(1'b1),.estop_ok(1'b1),
  .armed(armed),.cmd_update(cmd_update),.command(command),.speed(speed),
  .dl(dl),.dr(dr),.left_update(lu),.right_update(ru),
  .pos_l(pl),.pos_r(pr),.motor_fault(1'b0),.yaw(yaw),.yaw_update(yu),
  .target_l(tl),.target_r(tr),.brake(brake),.state(state),.fault(fault),
  .turn_error(),.turn_error_valid());

 // With the default zero track, ordinary driving remains available but an
 // autonomous scan may not silently rely on the known-unverified IMU alone.
 wire signed [15:0] unconfigured_l,unconfigured_r;
 wire unconfigured_brake;wire [3:0] unconfigured_state;
 wire [15:0] unconfigured_fault;
 rb_safety_control #(.CLK_HZ(1000),.SETTLE_MS(4),.ADV_COUNTS(3056),
  .SCAN_WAIT_MS(100),.STEP_TIMEOUT_MS(1000),.TURN_TIMEOUT_MS(20000),
  .ADVANCE_TIMEOUT_MS(10000),.DETOUR_TIMEOUT_MS(200000),.TURN_TRACK_MM(0)) unconfigured(
  .clk(clk),.rst_n(rst_n),.link_ok(1'b1),.imu_ok(1'b1),
  .left_ok(1'b1),.right_ok(1'b1),.estop_ok(1'b1),
  .armed(armed),.cmd_update(cmd_update),.command(command),.speed(speed),
  .dl(dl),.dr(dr),.left_update(lu),.right_update(ru),
  .pos_l(pl),.pos_r(pr),.motor_fault(1'b0),.yaw(yaw),.yaw_update(yu),
  .target_l(unconfigured_l),.target_r(unconfigured_r),.brake(unconfigured_brake),
  .state(unconfigured_state),.fault(unconfigured_fault),.turn_error(),.turn_error_valid());

 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit good,input string message);
  if(!good)$fatal(1,"%s state=%0d yaw=%0d pos=%0d/%0d target=%0d/%0d brake=%b fault=%h",
   message,state,$signed(yaw),$signed(pl),$signed(pr),$signed(tl),$signed(tr),brake,fault);
 endtask
 task set_yaw_delta(input integer delta);
  integer value;
  begin
   value=$signed(yaw)+delta;
   if(value>18000)value=value-36000;
   if(value< -18000)value=value+36000;
   yaw=value;
  end
 endtask
 task reset_drive(input integer heading);begin
  @(negedge clk);rst_n=0;armed=0;cmd_update=0;command=0;speed=16;
  dl=1000;dr=1000;lu=0;ru=0;yu=1;pl=0;pr=0;yaw=heading;cycles(3);
  @(negedge clk);rst_n=1;cycles(3);
  @(negedge clk);cmd_update=1;cycles(1);
  @(negedge clk);cmd_update=0;armed=1;command=1;cycles(3);
  check(state==0 && fault==0 && !brake && tl==16 && tr==16,
   "clear forward drive remains available with configured geometry");
 end endtask
 task begin_scan;begin
  @(negedge clk);dl=150;dr=180;cycles(2);
  check(state==1 && brake,"near obstacle brakes before the scan");
  cycles(12);
  check(state==2 && fault==0 && tl<0 && tr>0,
   "configured first scan starts a left pivot");
 end endtask
 task reset_scan(input integer heading);begin reset_drive(heading);begin_scan();end endtask

 // Coherent, correctly scaled real turns, including braking/coast steps.
 // All callers use multiples of 150 centidegrees, so one step is 10 counts
 // backwards on one wheel and 10 forwards on the other.
 task turn_real(input integer angle);
  integer remaining,direction;
  begin
   remaining=angle;
   while(remaining!=0 && fault==0)begin
    direction=remaining>0?1:-1;
    @(negedge clk);set_yaw_delta(direction*150);
    pl=pl-direction*10;pr=pr+direction*10;
    remaining=remaining-direction*150;cycles(3);
   end
  end
 endtask
 task sample(input integer l,input integer r);begin
  @(negedge clk);dl=l;dr=r;lu=1;ru=1;cycles(1);
  @(negedge clk);lu=0;ru=0;cycles(3);
 end endtask
 task finish_left_scan(input integer l,input integer r);begin
  turn_real(9000);cycles(8);
  check(state==3 && brake && fault==0,"normal left90 reaches its stopped scan pose");
  sample(l,r);
  check(state==2 && tl>0 && tr<0 && fault==0,
   "fresh left range begins a new right180 leg");
 end endtask
 task finish_right_scan(input integer l,input integer r);begin
  turn_real(-18000);cycles(8);
  check(state==3 && brake && fault==0,"normal right180 reaches its stopped scan pose");
  sample(l,r);
 end endtask
 task advance_to_return;
  integer distance,increment;
  begin
   check(state==4 && tl>0 && tr>0 && fault==0,"selected clear side permits forward travel");
   distance=0;
   // 3056/7640 metres = 40cm on each wheel. This is more wheel travel
   // than a complete100mm-track revolution and must not consume a turn budget.
   while(distance<3056 && fault==0)begin
    increment=(3056-distance)>50?50:(3056-distance);
    @(negedge clk);pl=pl+increment;pr=pr+increment;
    distance=distance+increment;cycles(3);
    if(distance<3056)check(state==4 && !brake && fault==0,
     "normal40cm ADVANCE must not accumulate the prior scan's turn budget");
   end
   check(distance==3056 && state==6 && brake && fault==0,
    "advance completes the full40cm and stops before restoring heading");
   cycles(8);
   check(state==5 && !brake && fault==0,"return gets a fresh independent turn budget");
  end
 endtask

 // New yaw frames keep arriving, but each physical 1.5-degree encoder step
 // reports only 0.37 degree, or reports no yaw change at all. Timeout is set
 // much longer than this injection: a pass requires encoder travel protection.
 task bad_heading_turn(input bit left,input bit frozen,output integer traveled);
  integer direction;
  begin
   direction=left?1:-1;traveled=0;
   while(fault==0 && traveled<4800)begin
    @(negedge clk);
    pl=pl-direction*10;pr=pr+direction*10;
    if(!frozen)set_yaw_delta(direction*37);
    traveled=traveled+20;cycles(3);
   end
   cycles(4);
   check(traveled<4800 && state==7 && fault==16'h0400 && brake && tl==0 && tr==0,
    "low/frozen IMU must fail and brake before a full physical encoder revolution");
  end
 endtask

 integer traveled;
 initial begin
  reset_scan(0);bad_heading_turn(1,0,traveled);
  check(traveled>=1200 && traveled<=1800,
   "quarter-scale first left90 uses its own roughly120-degree physical limit");

  reset_scan(0);bad_heading_turn(1,1,traveled);
  check(traveled>=1200 && traveled<=1800,
   "fresh but frozen yaw cannot prolong left90 to the fifteen-second timeout");

  reset_scan(0);finish_left_scan(1000,1100);
  bad_heading_turn(0,0,traveled);
  check(traveled>=2400 && traveled<=3000,
   "right180 resets the left budget and has a roughly210-degree physical limit");

  reset_scan(0);finish_left_scan(800,900);finish_right_scan(1800,1900);
  advance_to_return();
  check(tl<0 && tr>0,"right-side departure returns with a left90 pivot");
  bad_heading_turn(1,1,traveled);
  check(traveled>=1200 && traveled<=1800,
   "return90 also rejects frozen heading independently of earlier scan travel");

  // Correction moves share the original leg budget, including coast while
  // stopped. Resetting the guard per correction would allow this oscillation.
  reset_scan(0);turn_real(9000);turn_real(600);cycles(8);
  check(state==2 && tl>0 && tr<0 && fault==0,
   "small settled left overshoot permits one slow right correction");
  turn_real(-1800);cycles(8);
  check(state==2 && tl<0 && tr>0 && fault==0,
   "second small correction remains inside the original physical budget");
  turn_real(1800);cycles(8);
  check(state==7 && fault==16'h0400 && brake && tl==0 && tr==0,
   "correction oscillation cannot reset or escape the original90-degree budget");

  // A valid complete left-side maneuver exceeds one revolution in total
  // wheel travel, but every independent main leg remains correctly bounded.
  // Starting near180 also checks that yaw wrapping does not reset the budget.
  reset_scan(17000);finish_left_scan(2000,2100);finish_right_scan(800,900);
  check(state==2 && tl<0 && tr>0 && fault==0,
   "wider left side starts a separate left180 alignment");
  turn_real(18000);cycles(8);sample(1200,1300);
  advance_to_return();
  check(tl>0 && tr<0,"left-side departure returns with a right90 pivot");
  turn_real(-9000);cycles(8);sample(1000,1100);
  check(state==0 && fault==0 && !brake && tl==16 && tr==16 && yaw==17000,
   "normal wrapped left90/right180/left180/return90 does not share unrelated budgets");

  reset_scan(0);
  @(negedge clk);pl=pl-10;pr=pr+10;set_yaw_delta(37);cycles(3);
  @(negedge clk);command=0;#1;
  check(brake && tl==0 && tr==0,"operator STOP brakes before the next clock during guarded turn");
  cycles(3);check(state==0 && fault==0 && brake,"STOP cancels the guarded maneuver without a late turn pulse");

  reset_drive(0);
  check(unconfigured_state==0 && unconfigured_fault==0 && !unconfigured_brake &&
   unconfigured_l==16 && unconfigured_r==16,
   "unconfigured geometry does not suppress ordinary clear forward commands");
  begin_scan();
  check(unconfigured_state==7 && unconfigured_fault==16'h0400 && unconfigured_brake &&
   unconfigured_l==0 && unconfigured_r==0,
   "unconfigured autonomous scan refuses to turn blindly on IMU alone");

  $display("PASS integrated turn guard: quarter-scale/frozen IMU, left90/right180/return, cumulative corrections, normal wrapped maneuver, STOP and missing geometry");
  $finish;
 end
 initial begin #3000000;$fatal(1,"integrated turn guard test hung");end
endmodule
