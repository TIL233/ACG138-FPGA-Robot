`timescale 1ns/1ps
module tb_navigation;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,run=0,healthy=1;
 reg [3:0] safety_state=0;
 reg signed [31:0] pl=0,pr=0;
 reg signed [15:0] yaw=0;
 reg wr=0,commit=0,clear_route=0,reset_pose=0;
 reg [2:0] wi=0;
 reg signed [15:0] wx=0,wy=0;
 reg [3:0] wc=0;
 wire [2:0] cmd,index;
 wire [7:0] speed;
 wire active,done,failed,ready,pose_ready;
 wire [3:0] state;
 wire signed [31:0] x,y;
 wire signed [15:0] heading;
 rb_navigation #(.CLK_HZ(5000),.CONTROL_HZ(50),.COUNTS_PER_M(1000),
  .SETTLE_MS(60),.TOTAL_TIMEOUT_MS(5000),.PROGRESS_TIMEOUT_MS(300),
  .TURN_TIMEOUT_MS(500),.MAX_YAW_STEP_CD(10000),.ARRIVE_MM(50),.SLOW_MM(300)) dut(
  .clk(clk),.rst_n(rst_n),.run(run),.healthy(healthy),.safety_state(safety_state),
  .pos_l(pl),.pos_r(pr),.yaw(yaw),.route_write(wr),.route_index(wi),
  .way_x(wx),.way_y(wy),.route_commit(commit),.route_count(wc),
  .route_clear(clear_route),.pose_reset(reset_pose),.command(cmd),.speed(speed),
  .active(active),.done(done),.failed(failed),.state(state),.index(index),
  .x(x),.y(y),.heading(heading),.route_ready(ready),.pose_ready(pose_ready));
 task cycles(input integer n);repeat(n) begin @(posedge clk);#1;end endtask
 task check(input bit v,input string msg);
  if(!v) $fatal(1,"%s cmd=%d state=%d x=%d y=%d heading=%d idx=%d failed=%b",msg,cmd,state,x,y,heading,index,failed);
 endtask
 task samples(input integer n);cycles(n*100+40);endtask
 task route_point(input integer n,input integer px,input integer py);
  begin @(negedge clk);wi=n;wx=px;wy=py;wr=1;cycles(1);@(negedge clk);wr=0;cycles(1);end
 endtask
 task commit_route(input integer n);
  begin @(negedge clk);wc=n;commit=1;cycles(1);@(negedge clk);commit=0;cycles(3);end
 endtask
 task origin;
  begin @(negedge clk);reset_pose=1;cycles(1);@(negedge clk);reset_pose=0;cycles(3);end
 endtask
 task move(input integer d);
  begin @(negedge clk);pl=pl+d;pr=pr+d;samples(1);end
 endtask
 initial begin
  cycles(3);rst_n=1;cycles(3);
  route_point(0,1000,0);route_point(1,1000,1000);commit_route(2);
  check(ready && !pose_ready,"route is independent of origin");
  run=1;cycles(3);check(failed && cmd==0,"must explicitly set pose before movement");
  run=0;origin;check(!failed && pose_ready && x==0 && y==0,"reset clears failure and sets origin");
  run=1;samples(1);check(cmd==1 && speed==12,"aligned first point drives forward");
  move(600);check(x>=599 && x<=602 && y>=-2 && y<=2,"calibrated encoder distance projects onto X");
  move(200);check(cmd==1 && speed==6,"approach slows down");
  move(200);check(cmd==0 && state==3,"arrival first brakes");
  samples(4);check(index==1 && !done,"next waypoint starts only after settling");
  samples(1);check(cmd==5,"next point to the left requests CCW rotation");
  yaw=9000;samples(1);check(cmd==1 && heading==9000,"IMU turn aligns second waypoint");
  move(600);check(y>=598 && y<=602,"90-degree travel projects onto Y");
  move(360);samples(4);check(done && !active && cmd==0,"last point completes stopped");
  run=0;samples(1);run=1;samples(1);check(done && cmd==0,"completed route never restarts implicitly");

  // Committing a new route preserves measured pose, and paused manual motion
  // is tracked while sensors remain healthy.
  run=0;route_point(0,2000,1000);commit_route(1);
  check(!done && !failed && x>950 && y>950,"new route does not silently reset pose");
  move(40);check(y>990,"odometry continues while navigation is paused");
  run=1;samples(1);check(cmd==6,"target to the right rotates clockwise");
  yaw=0;samples(1);check(cmd==1,"right turn reaches forward heading");
  // Obstacle avoidance runs in the existing safety FSM, while pose continues.
  safety_state=2;cycles(3);check(cmd==1 && state==4,"safety detour retains forward request");
  move(100);check(x>1050,"odometry includes detour displacement");
  safety_state=0;samples(1);check(cmd==1,"detour exit recalculates current target direction");
  healthy=0;#1;check(cmd==0,"health loss gates drive immediately");cycles(3);
  healthy=1;samples(1);check(failed && cmd==0,"health recovery cannot restart motion");
  run=0;samples(1);run=1;samples(1);check(failed && cmd==0,"run toggle does not clear failure");

  // Failure can be acknowledged with an explicit valid origin reset.
  run=0;origin;check(!failed && x==0 && y==0,"origin reset recovers latched failure");
  route_point(0,2000,0);commit_route(1);run=1;samples(1);check(cmd==1,"route can be restarted after explicit reset");
  samples(17);check(failed && cmd==0,"no distance progress times out stopped");
  run=0;origin;
  // Modulo-2^32 encoder wrap must remain a short positive displacement.
  pl=32'h7ffffff0;pr=32'h7ffffff0;origin;
  pl=32'h80000010;pr=32'h80000010;samples(1);
  check(!failed && x>=31 && x<=33,"encoder wrap is not a position discontinuity");
  // A jump beyond the physical increment bound is rejected, never integrated.
  pl=pl+5000;pr=pr+5000;samples(1);
  check(failed && !pose_ready && x<100,"bad encoder jump invalidates pose without integration");
  origin;yaw=17000;origin;yaw=-17000;samples(1);
  check(!failed && heading==2000,"IMU +/-180 wrap gives short relative heading");
  run=0;origin;
  // Configuration errors cannot leave an old route silently armed.
  wc=9;commit=1;cycles(1);commit=0;cycles(3);
  check(!ready && failed,"invalid route length rejected");
  clear_route=1;cycles(1);clear_route=0;cycles(3);
  check(!ready && index==0 && !done,"clear removes route without resetting pose");
  route_point(0,1000,0);commit_route(2);
  check(!ready && failed,"unwritten waypoint cannot become a route");
  yaw=0;origin;commit_route(1);run=1;samples(1);
  move(970);check(state==3 && cmd==0,"enter arrival settling before coasting");
  move(150);samples(4);
  check(!done && state!=3,"coasting beyond tolerance cannot falsely complete a waypoint");

  run=0;yaw=4500;origin;route_point(0,-1000,-1000);commit_route(1);
  yaw=0;samples(1);yaw=-9000;samples(1);run=1;samples(1);
  check(heading==-13500 && cmd==1,"origin-relative heading handles a negative-coordinate target");
  move(500);check(x>=-356 && x<=-351 && y>=-356 && y<=-351,"negative diagonal odometry retains signed products");
  yaw=9000;samples(1);
  check(failed && !pose_ready && cmd==0,"implausible IMU jump invalidates pose and latches stop");

  run=0;yaw=0;origin;route_point(0,0,1000);commit_route(1);run=1;samples(1);
  check(cmd==5,"turn-timeout test begins left turn");
  samples(27);check(failed && cmd==0,"turn without heading progress times out");
  run=0;origin;clear_route=1;cycles(1);clear_route=0;cycles(3);
  for(integer j=0;j<8;j=j+1) route_point(j,0,0);
  commit_route(8);check(ready,"all eight waypoint slots can be committed");
  run=1;samples(24);check(done && index==7 && !failed,"eight-point route stops at the final slot without wrapping");
  run=0;origin;route_point(0,1000,0);commit_route(1);run=1;samples(1);
  safety_state=8;#1;check(cmd==0,"unknown safety state gates output immediately");cycles(3);
  check(failed,"unknown safety state latches failure");
  $display("PASS navigation: pose/route/turn/arrival/coasting/slow/avoid/pause/fault/timeout/wrap/jump/8points");$finish;
 end
 initial begin #1000000;$fatal(1,"navigation test hung");end
endmodule
