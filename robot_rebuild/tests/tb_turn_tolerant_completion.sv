`timescale 1ns/1ps
// Exercise the actual independent encoder-heading producer and safety FSM.
// The IMU remains at the initial heading: successful scan rotation must come
// from wheel motion, not a testbench yaw value advanced without encoder data.
module tb_turn_tolerant_completion;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,armed=0,cmd_update=0;
 reg [2:0] command=0;reg [7:0] speed=16;
 reg signed [31:0] pl=0,pr=0;
 reg signed [15:0] imu_yaw=0;
 reg [15:0] dl=1000,dr=1000;reg lu=0,ru=0;
 wire motion_update,motion_ready,motion_fault;
 wire signed [15:0] imu_heading,encoder_heading;
 wire signed [15:0] tl,tr;
 wire brake;wire [3:0] state;wire [15:0] fault;
 wire clear_fault=cmd_update && !armed;
 reg guard_fault_seen=0;

 rb_motion_feedback #(.CLK_HZ(1000),.TRACK_MM(100),.COUNTS_PER_M(7640),.FUSE_ENCODER(0)) feedback(
  .clk(clk),.rst_n(rst_n),.clear_fault(clear_fault),.imu_ok(1'b1),.imu_update(1'b1),
  .pos_l(pl),.pos_r(pr),.imu_yaw(imu_yaw),.update(motion_update),
  .ready(motion_ready),.fault(motion_fault),.heading(imu_heading),
  .encoder_heading(encoder_heading),.innovation(),.yaw_rate(),.velocity_l(),.velocity_r(),
  .sample_pos_l(),.sample_pos_r(),.sample_yaw(),.timestamp_ms(),.flags());
 rb_safety_control #(.CLK_HZ(1000),.SETTLE_MS(4),.ADV_COUNTS(3056),
  .SCAN_WAIT_MS(1000),.STEP_TIMEOUT_MS(2000),.TURN_TIMEOUT_MS(20000),
  .ADVANCE_TIMEOUT_MS(10000),.DETOUR_TIMEOUT_MS(200000),
  .TURN_TRACK_MM(100),.TURN_COUNTS_PER_M(7640),.TURN_TRAVEL_MARGIN_CD(3000),
  .ANGLE_TOLERANCE_CD(500),.FINAL_ANGLE_TOLERANCE_CD(1000),
  .RELAX_AFTER_CORRECTIONS(1),.MAX_TURN_CORRECTIONS(4),.MIN_PIVOT_COUNTS(4)) dut(
  .clk(clk),.rst_n(rst_n),.link_ok(1'b1),.imu_ok(motion_ready && !motion_fault),
  .left_ok(1'b1),.right_ok(1'b1),.estop_ok(1'b1),
  .armed(armed),.cmd_update(cmd_update),.command(command),.speed(speed),
  .dl(dl),.dr(dr),.left_update(lu),.right_update(ru),
  .pos_l(pl),.pos_r(pr),.motor_fault(1'b0),.yaw(encoder_heading),.yaw_update(motion_update),
  .target_l(tl),.target_r(tr),.brake(brake),.state(state),.fault(fault),
  .turn_error(),.turn_error_valid());
 always @(posedge clk or negedge rst_n)
  if(!rst_n)guard_fault_seen<=0;
  else if(dut.turn_guard_fault)guard_fault_seen<=1;

 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit good,input string message);
  if(!good)$fatal(1,"%s state=%0d encoder=%0d imu=%0d positions=%0d/%0d targets=%0d/%0d corrections=%0d fault=%h motion_fault=%b",
   message,state,$signed(encoder_heading),$signed(imu_heading),$signed(pl),$signed(pr),
   $signed(tl),$signed(tr),dut.correction_count,fault,motion_fault);
 endtask
 task await_state(input integer wanted);
  integer elapsed;
  begin
   elapsed=0;
   while(state!=wanted && fault==0 && elapsed<300)begin cycles(1);elapsed=elapsed+1;end
   check(state==wanted && fault==0,"expected motion phase becomes ready");
  end
 endtask
 task reset_scan(input integer heading);begin
  @(negedge clk);rst_n=0;armed=0;cmd_update=0;command=0;speed=16;
  pl=0;pr=0;imu_yaw=heading;dl=1000;dr=1000;lu=0;ru=0;cycles(3);
  @(negedge clk);rst_n=1;cycles(60);
  check(motion_ready && !motion_fault && imu_heading==heading,
   "real feedback initializes while main heading remains the unmodified IMU");
  @(negedge clk);cmd_update=1;cycles(1);
  @(negedge clk);cmd_update=0;armed=1;command=1;cycles(3);
  check(state==0 && !brake && tl==16 && tr==16,"clear forward command remains available");
  @(negedge clk);dl=150;dr=180;cycles(2);
  check(state==1 && brake,"near obstacle first stops the chassis");
  await_state(2);cycles(6);
  check(tl<0 && tr>0 && !brake,"first guarded scan turns left");
 end endtask

 // Each20 total wheel counts is approximately1.5deg at100mm /7640counts/m.
 // Wait beyond the real20ms feedback snapshot after each physical step.
 task turn_real(input integer angle);
  integer remaining,direction;
  begin
   remaining=angle;
   while(remaining!=0 && fault==0)begin
    direction=remaining>0?1:-1;
    @(negedge clk);pl=pl-direction*10;pr=pr+direction*10;
    remaining=remaining-direction*150;cycles(30);
   end
   check(!motion_fault,"physical turn samples remain valid for the real feedback producer");
  end
 endtask
 task settle;begin cycles(60);end endtask
 task sample(input integer l,input integer r);begin
  @(negedge clk);dl=l;dr=r;lu=1;ru=1;cycles(1);
  @(negedge clk);lu=0;ru=0;cycles(8);
 end endtask
 task require_right_sweep;begin
  check(state==2 && tl>0 && tr<0 && fault==0,
   "accepted left pose starts a protected right sweep without an extra correction or guard rejection");
 end endtask
 task advance40cm;
  integer distance,increment;
  begin
   check(state==4 && !brake && fault==0,"clear chosen side starts forward bypass");
   distance=0;
   while(distance<3056 && fault==0)begin
    increment=(3056-distance)>50?50:(3056-distance);
    @(negedge clk);pl=pl+increment;pr=pr+increment;distance=distance+increment;
    if(distance<3056)cycles(30);else cycles(3);
    if(distance<3056)check(state==4 && !brake && fault==0,
     "forty-centimetre forward bypass does not consume a turning budget");
   end
   check(distance==3056 && state==6 && brake && fault==0,"full forty-centimetre bypass stops before return");
   await_state(5);cycles(6);
  end
 endtask

 integer i;
 initial begin
  // Slightly short of90deg is accepted directly under the normal5deg band.
  reset_scan(0);turn_real(8550);settle();
  check(state==3 && brake && dut.correction_count==0,"normal minus4.5deg error does not demand correction");
  sample(1000,1100);require_right_sweep();

  // Full valid maneuver with a positive4.5deg left endpoint. Its right sweep
  // exceeds181.5deg, so the guard must honor the configured normal tolerance.
  reset_scan(17000);turn_real(9450);settle();
  check(state==3 && brake && dut.correction_count==0,"normal plus4.5deg error is accepted directly");
  sample(2000,2100);require_right_sweep();
  turn_real(-18450);settle();
  check(state==3 && brake && fault==0,"normal directed right184.5deg completes with the independent encoder heading");
  sample(800,900);
  check(state==2 && tl<0 && tr>0,"wider left side begins a fresh left180deg alignment");
  turn_real(18000);settle();sample(1200,1300);advance40cm();
  check(tl>0 && tr<0,"selected-left departure returns right toward original heading");
  turn_real(-9000);settle();sample(1000,1100);
  check(state==0 && fault==0 && !brake && tl==16 && tr==16 && imu_heading==17000 &&
        encoder_heading>16980 && encoder_heading<17020,
   "encoder-controlled left/right/alignment/return completes while global IMU heading stays unchanged");

  // One bounded correction may finish within the final10deg band. All yaw
  // changes below are generated by the actual encoder position movements.
  reset_scan(0);turn_real(9000);turn_real(600);settle();
  check(state==2 && tl==4 && tr==-4 && dut.correction_count==1 && fault==0,
   "plus6deg settled error gets one slow right correction at the configured minimum");
  turn_real(-1200);settle();
  check(state==3 && brake && dut.correction_count==1 && fault==0,
   "minus6deg residual is accepted after one correction rather than oscillating or failing");
  sample(1000,1100);require_right_sweep();

  // The opposite residual sign produces a right186deg main sweep. Its goal
  // must fit180deg plus the final tolerance, not just the normal5deg band.
  reset_scan(0);turn_real(9000);turn_real(-600);settle();
  check(state==2 && tl==-4 && tr==4 && dut.correction_count==1 && fault==0,
   "minus6deg settled error gets a slow left correction");
  turn_real(1200);settle();
  check(state==3 && brake && dut.correction_count==1 && fault==0,
   "plus6deg residual is also accepted after one bounded correction");
  sample(1000,1100);require_right_sweep();
  turn_real(-18600);settle();
  check(state==3 && brake && fault==0,"relaxed positive endpoint does not make the next186deg right goal illegal");

  // More than10deg is not called complete merely because one correction ran.
  reset_scan(0);turn_real(9000);turn_real(600);settle();
  turn_real(-1800);settle();
  check(state==2 && tl<0 && tr>0 && fault==0 && dut.correction_count==2,
   "roughly minus12deg residual requires further correction and cannot use relaxed completion");
  sample(2500,2600);
  check(state==2 && tl<0 && tr>0 && fault==0,
   "range frames cannot advance an off-heading correction into the right scan");

  // Over-travel still wins. Inject rapid coasting after the provisional stop;
  // raw per-clock encoder steps stay within bounds, and the guard must act
  // before the slower encoder-heading snapshot can finish classifying coast.
  reset_scan(0);turn_real(9000);
  for(i=0;i<40 && fault==0;i=i+1)begin
   @(negedge clk);pl=pl-10;pr=pr+10;cycles(1);
  end
  cycles(8);
  check(guard_fault_seen && state==7 && fault==16'h0400 && brake && tl==0 && tr==0,
   "independent over-travel budget still latches0400 despite relaxed angular completion");

  // STOP also interrupts the corrective phase before the next clock.
  reset_scan(0);turn_real(9000);turn_real(600);settle();
  check(state==2 && dut.correction_count==1,"STOP scenario is correcting a small error");
  @(negedge clk);command=0;#1;
  check(brake && tl==0 && tr==0,"operator STOP immediately overrides tolerant correction");
  cycles(3);check(state==0 && brake && fault==0,"STOP cancels the complete tolerant maneuver");

  reset_scan(0);turn_real(8100);
  check(state==2 && tl==-4 && tr==4,"last ten degrees retain the usable four-count pivot minimum");
  @(negedge clk);speed=1;#1;
  check(tl==-1 && tr==1,"minimum pivot setting never raises a lower operator speed cap");

  $display("PASS tolerant completion: real independent encoder feedback, normal5deg/final10deg bands, one correction, residual rejection, extended directed goals, complete bypass, travel guard, STOP and pivot minimum");
  $finish;
 end
 initial begin #5000000;$fatal(1,"tolerant completion test hung");end
endmodule
