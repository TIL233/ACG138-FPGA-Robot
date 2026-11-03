`timescale 1ns/1ps
// Missing local ownership must fail the forward assertion for a 90-degree goal.
// NAV_LEGACY_RED runs that behavior against the original, unchanged interface.
module tb_navigation_local_steering;
 reg clk=0; always #5 clk=~clk;
 reg rst_n=0,run=0,healthy=1,local_steering=0;
 reg [3:0] safety_state=0;
 reg signed [31:0] pl=0,pr=0;
 reg signed [15:0] yaw=0,wx=0,wy=1000;
 reg wr=0,commit=0,reset_pose=0;
 wire [2:0] command,legacy_command,open_command;
 wire [7:0] speed,legacy_speed,open_speed;
 wire active,done,failed; wire [3:0] state;
 wire signed [15:0] goal_error,turn_error;
 wire goal_valid,turn_valid;
 rb_navigation #(.CLK_HZ(5000),.CONTROL_HZ(50),.COUNTS_PER_M(1000),
  .SETTLE_MS(60),.ARRIVE_MM(50),.SLOW_MM(300),.MAX_YAW_STEP_CD(18000),
  .TOTAL_TIMEOUT_MS(20000),.PROGRESS_TIMEOUT_MS(10000),.TURN_TIMEOUT_MS(10000)
`ifndef NAV_LEGACY_RED
  ,.LOCAL_STEERING_ENABLE(1)
`endif
 ) dut(
  .clk(clk),.rst_n(rst_n),.run(run),.healthy(healthy),.safety_state(safety_state),
  .pos_l(pl),.pos_r(pr),.yaw(yaw),.route_write(wr),.route_index(3'd0),
  .way_x(wx),.way_y(wy),.route_commit(commit),.route_count(4'd1),
  .route_clear(1'b0),.pose_reset(reset_pose),.command(command),.speed(speed),
  .active(active),.done(done),.failed(failed),.state(state),.index(),.x(),.y(),
  .heading(),.route_ready(),.pose_ready(),.turn_error(turn_error),.turn_error_valid(turn_valid)
`ifndef NAV_LEGACY_RED
  ,.local_steering(local_steering),.goal_error(goal_error),.goal_error_valid(goal_valid)
`endif
 );
 rb_navigation #(.CLK_HZ(5000),.CONTROL_HZ(50),.COUNTS_PER_M(1000),
  .SETTLE_MS(60),.ARRIVE_MM(50),.SLOW_MM(300),.MAX_YAW_STEP_CD(18000),
  .TOTAL_TIMEOUT_MS(20000),.PROGRESS_TIMEOUT_MS(10000),.TURN_TIMEOUT_MS(10000)) legacy(
  .clk(clk),.rst_n(rst_n),.run(run),.healthy(healthy),.safety_state(safety_state),
  .pos_l(pl),.pos_r(pr),.yaw(yaw),.route_write(wr),.route_index(3'd0),
  .way_x(wx),.way_y(wy),.route_commit(commit),.route_count(4'd1),
  .route_clear(1'b0),.pose_reset(reset_pose),.command(legacy_command),.speed(legacy_speed),
  .active(),.done(),.failed(),.state(),.index(),.x(),.y(),.heading(),
  .route_ready(),.pose_ready(),.turn_error(),.turn_error_valid()
`ifndef NAV_LEGACY_RED
  ,.local_steering(local_steering),.goal_error(),.goal_error_valid()
`endif
 );
 // The unconnected input is Z. Default parameter zero must mask it completely.
 rb_navigation #(.CLK_HZ(5000),.CONTROL_HZ(50),.COUNTS_PER_M(1000),
  .SETTLE_MS(60),.ARRIVE_MM(50),.SLOW_MM(300),.MAX_YAW_STEP_CD(18000),
  .TOTAL_TIMEOUT_MS(20000),.PROGRESS_TIMEOUT_MS(10000),.TURN_TIMEOUT_MS(10000)) legacy_open(
  .clk(clk),.rst_n(rst_n),.run(run),.healthy(healthy),.safety_state(safety_state),
  .pos_l(pl),.pos_r(pr),.yaw(yaw),.route_write(wr),.route_index(3'd0),
  .way_x(wx),.way_y(wy),.route_commit(commit),.route_count(4'd1),
  .route_clear(1'b0),.pose_reset(reset_pose),.command(open_command),.speed(open_speed),
  .active(),.done(),.failed(),.state(),.index(),.x(),.y(),.heading(),
  .route_ready(),.pose_ready(),.turn_error(),.turn_error_valid()
`ifndef NAV_LEGACY_RED
  ,.local_steering(),.goal_error(),.goal_error_valid()
`endif
 );
 task cycles(input integer n); repeat(n) begin @(posedge clk);#1;end endtask
 task check(input bit ok,input string msg);
  if(!ok)$fatal(1,"%s command=%0d state=%0d speed=%0d goal=%0d valid=%b",msg,command,state,speed,goal_error,goal_valid);
 endtask
 task samples(input integer n);cycles(n*100+40);endtask
 task wait_goal;
  integer timeout;
  begin
   timeout=0;
   while(!goal_valid && timeout<200)begin cycles(1);timeout=timeout+1;end
   check(goal_valid,"goal angle completes within two control samples");
  end
 endtask
 task configure(input integer px,input integer py);
  begin
   @(negedge clk);run=0;wx=px;wy=py;wr=1;cycles(1);
   @(negedge clk);wr=0;commit=1;reset_pose=1;cycles(1);
   @(negedge clk);commit=0;reset_pose=0;cycles(3);
   @(negedge clk);run=1;samples(2);
  end
 endtask
 always @(negedge clk) if(rst_n)
  check({legacy_command,legacy_speed}==={open_command,open_speed},"default parameter masks connected or Z local input");
 initial begin
  cycles(3);@(negedge clk);rst_n=1;cycles(3);
  configure(0,1000);
  check(command==5 && state==1 && speed==6,"ordinary side goal pivots left");
`ifndef NAV_LEGACY_RED
  wait_goal;
  check(goal_valid && turn_valid && goal_error>=8950 && goal_error<=9050 && goal_error==turn_error,
   "side goal exposes current 90-degree vector and legacy turn feedback");
`endif
  @(negedge clk);local_steering=1;samples(2);
  check(command==1 && state==2 && speed==12,"local steering owns sideways goal and drives at maximum speed");
`ifndef NAV_LEGACY_RED
  wait_goal;
  check(goal_valid && goal_error>=8950 && goal_error<=9050 && !turn_valid,
   "goal vector remains valid while driving; legacy turn valid stays TURNING-only");
`endif
  check(legacy_command==5,"default-disabled instance still pivots with local input high");
  @(negedge clk);local_steering=0;samples(2);
  check(command==5 && state==1,"disabling local steering restores ordinary pivot");
  @(negedge clk);local_steering=1;samples(2);
  safety_state=2;cycles(3);check(command==1 && state==4,"legacy avoidance keeps ownership");
  @(negedge clk);safety_state=0;cycles(1);check(command==0 && state==1,"avoidance exit keeps its ordinary stopped handoff");
  samples(2);check(command==1 && state==2,"local steering resumes after avoidance handoff");
  @(negedge clk);run=0;#1;check(command==0 && speed==0,"STOP immediately gates motion");
  cycles(3);check(!active && !done,"STOP pauses without completing route");
`ifndef NAV_LEGACY_RED
  check(!goal_valid,"inactive goal is invalid");
`endif
  @(negedge clk);run=1;samples(2);check(command==1,"paused route resumes local steering");
`ifndef NAV_LEGACY_RED
  wait_goal;
`endif
  @(negedge clk);healthy=0;#1;check(command==0 && speed==0,"health immediately gates motion");
`ifndef NAV_LEGACY_RED
  check(!goal_valid,"unhealthy goal is invalid immediately");
`endif
  cycles(3);healthy=1;samples(2);check(failed && !active && command==0,"health failure remains latched");
  configure(0,200);check(command==1 && state==2 && speed==6 && !failed,"local near goal keeps slow-speed policy");
  configure(-1000,0);check(command==1 && state==2 && speed==12,"local steering also owns a behind goal");
  configure(1000,0);check(command==1 && speed==12,"aligned local goal drives forward");
  @(negedge clk);pl=970;pr=970;samples(1);
  check(command==0 && state==3 && !done,"local steering cannot bypass arrival braking");
  samples(1);check(!done,"arrival still waits for settling");
  samples(4);check(done && !active && !failed && command==0,"arrival completes only after settling");
  @(negedge clk);run=0;cycles(3);run=1;samples(2);
  check(done && command==0,"completed route does not restart implicitly");
  $display("PASS navigation local steering: default-Z, side/behind, goal vector, switch, speed, avoidance, STOP, health, arrival");$finish;
 end
 initial begin #1000000;$fatal(1,"local steering test hung");end
endmodule
