`timescale 1ns/1ps
module tb_lidar_controller;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,enabled=1,control_enabled=1,original_control_enabled=1,model_validated=1,follow_mode=0,nav_mode=0;
 reg armed=1,stop_now=0,healthy=1;reg [2:0] request_cmd=1;reg [7:0] request_speed=12;
 reg [3:0] safety_state=0;reg feedback_update=0;
 reg signed [15:0] velocity_l=0,velocity_r=0,goal_cd=0;
 reg goal_valid=1;
 reg sonar_l_ok=0,sonar_r_ok=0;reg [15:0] sonar_l=350,sonar_r=8000;
 reg laser_front_ok=1;reg [15:0] laser_front=1000;
 reg map_update=0,map_fresh=1;reg [575:0] map_ranges={36{16'd8000}};
 reg [35:0] map_known={36{1'b1}};reg [15:0] map_scan_id=1,map_age_ms=90;
 wire scope,takeover,hold,follow_limit_valid,dynamic_gate,dynamic_hold,current;
 wire signed [15:0] target_l,target_r,follow_l,follow_r;wire [3:0] selected;wire [319:0] diagnostic;
 reg [319:0] saved_diagnostic; reg [31:0] sampled_time;
 rb_lidar_controller dut(.*);
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task pulse;begin @(negedge clk);feedback_update=1;ticks(1);@(negedge clk);feedback_update=0;end endtask
 reg [15:0] expected_generation=0;
 reg delayed_completion=0;
 always @(posedge clk)begin
  if(!rst_n)begin expected_generation=0;delayed_completion=0;end
  else begin
   if(delayed_completion)expected_generation=expected_generation+1'b1;
   delayed_completion=dut.was_busy && !dut.busy && (dut.plan_current || dut.completed_no_path);
   #1;if(dut.generation!==expected_generation)$fatal(1,"diagnostic generation lacks one-clock completion boundary");
  end
 end
 always @(negedge clk)if(rst_n)begin
  if(dut.env_valid && dut.envelope_cap!==((dut.env_l>dut.env_r?dut.env_l:dut.env_r)>255?8'd255:(dut.env_l>dut.env_r?dut.env_l:dut.env_r)))
   $fatal(1,"raw cap was not atomic with valid wheel limits");
  if(scope && dut.cap_pending && (current || !hold))
   $fatal(1,"new raw cap bypassed protected adoption");
 end
 initial begin
  ticks(3);rst_n=1;ticks(4);pulse();ticks(700);
  if(hold || !current || target_l!=12)$fatal(1,"initial clear plan");
  // Invalid passthrough requests must not feed the published cap data path.
  @(negedge clk);request_speed=20;ticks(2);
  if(dut.env_valid || dut.envelope_cap!=12 || dut.cap_pending)
   $fatal(1,"invalid passthrough request changed published cap");
  request_speed=12;ticks(3);pulse();ticks(700);
  // Even a non-winning raw field must take the short pending path: no live
  // min/age fusion may be needed to withdraw permission. Adoption alone must
  // release it without a new sample, epoch, or envelope calculation.
  @(negedge clk);sonar_r=7999;#1;
  if(!dut.range_pending || current || !hold || target_l!=0 || dut.front!=1000)
   $fatal(1,"non-winning raw range did not immediately withdraw permission");
  ticks(1);
  if(!dut.range_pending || current || !hold)$fatal(1,"raw range first-slot hold gap");
  ticks(1);
  if(dut.range_pending || !current || hold || target_l!=12 || dut.envelope.running)
   $fatal(1,"non-winning raw range did not release after adoption without sample");
  // Returning before adoption cannot hide the occupied raw capture slot.
  @(negedge clk);sonar_r=7998;ticks(1);@(negedge clk);sonar_r=7999;#1;
  if(!dut.range_pending || current || !hold)$fatal(1,"non-winning raw bounce reopened permission");
  ticks(1);
  if(!dut.range_pending || current || !hold)$fatal(1,"raw bounce adoption hold gap");
  ticks(1);
  if(dut.range_pending || !current || hold)$fatal(1,"raw bounce failed to drain");
  // A one-clock excursion must drain both pipeline slots before permission
  // can return, even when the live input has already returned to the old key.
  @(negedge clk);velocity_l=13;ticks(1);@(negedge clk);velocity_l=0;#1;
  if(current || !hold || target_l!=0 || target_r!=0)
   $fatal(1,"captured speed excursion revived old permission before adoption");
  ticks(3);pulse();ticks(700);
  @(negedge clk);laser_front=900;ticks(1);@(negedge clk);laser_front=1000;#1;
  if(current || !hold || target_l!=0 || target_r!=0)
   $fatal(1,"captured range excursion revived old permission before adoption");
  ticks(3);pulse();ticks(700);
  @(negedge clk);request_speed=20;ticks(1);@(negedge clk);request_speed=12;#1;
  if(current || !hold || target_l!=0 || target_r!=0)
   $fatal(1,"captured request excursion revived old permission before adoption");
  ticks(3);pulse();ticks(700);
  @(negedge clk);velocity_l=13;#1;
  if(dut.measured!=0 || !hold || current)
   $fatal(1,"derived speed bound lacks registered boundary or immediate speed guard");
  ticks(2);if(dut.measured!=13)$fatal(1,"derived measured snapshot did not arrive");
  velocity_l=0;ticks(3);pulse();ticks(700);
  // Derived bounds must cover signed minimum and the full requested speed.
  @(negedge clk);velocity_l=-32768;#1;
  if(!hold || current)$fatal(1,"signed-min speed rise did not withdraw immediately");
  ticks(2);
  if(dut.measured!=32768 || dut.registered_bound!=32768 || !dut.bounds_ready)
   $fatal(1,"signed-min snapshot/17-bit bound validation");
  pulse();ticks(500);
  if(!hold || dut.speed_mm_s!=65535)$fatal(1,"signed-min bound reached unsafe envelope");
  // A deliberately under-reported measured bound fails the snapshot check.
  force dut.measured_capture=16'd1;ticks(1);
  if(dut.bounds_ready || dut.env_valid || !hold)$fatal(1,"under-measured snapshot admitted");
  release dut.measured_capture;velocity_l=0;ticks(3);pulse();ticks(700);
  // The sample edge changing the request must not capture a prior lower bound.
  @(negedge clk);request_speed=20;feedback_update=1;ticks(1);
  @(negedge clk);feedback_update=0;ticks(3);
  if(dut.envelope.counts_q!=20 || dut.envelope.request_l_q!=20 || !dut.bounds_ready)
   $fatal(1,"new request mixed with old combined bound");
  ticks(500);if(dut.speed_mm_s!=131)$fatal(1,"request above measured bound ignored");
  request_speed=12;ticks(3);pulse();ticks(700);
  force dut.bound_capture=16'd1;ticks(1);
  if(dut.bounds_ready || dut.env_valid || !hold)$fatal(1,"under-request snapshot admitted");
  release dut.bound_capture;ticks(3);
  if(dut.env_valid)$fatal(1,"invalid bound resurrected old publication");
  pulse();ticks(700);
  // A real near obstacle withdraws before the first sampling edge and
  // remains stopped after adoption and a completed envelope calculation.
  @(negedge clk);laser_front=150;#1;
  if(current || !hold || target_l!=0 || target_r!=0)$fatal(1,"near range lacked immediate hold");
  ticks(3);pulse();ticks(700);
  if(!hold || target_l!=0 || target_r!=0 || !dut.env_hold)
   $fatal(1,"near range escaped stop after adoption");
  laser_front=1000;ticks(3);pulse();ticks(700);
  if(!current || hold || target_l!=12)$fatal(1,"clear range failed to recover after near stop");
  if(!$test$plusargs("FOLLOW_ONLY"))begin
   @(negedge clk);sonar_l_ok=1;#1;
   if(dut.front!=1000 || dut.front_age!=90 || !hold || current)
    $fatal(1,"range fusion lacks registered bundle or immediate old-plan withdrawal");
   ticks(2);
   if(dut.front!=350 || dut.front_age!=0 || !dut.front_valid)
    $fatal(1,"fused range/valid/age did not sample together");
   pulse();wait(dut.env_valid);#1;
   if(!hold && (target_l>dut.env_l || target_r>dut.env_r))
    $fatal(1,"new dynamic cap released a stale faster plan for one clock");
   ticks(250);
   if(hold || target_l>6 || target_r>6)$fatal(1,"slower plan did not recover");
  end
  sonar_l_ok=0;follow_mode=1;velocity_l=200;pulse();ticks(500);
  if(!hold || !dynamic_hold || !follow_limit_valid)$fatal(1,"follow dynamic hold not established");
  pulse();
  if(!hold || !follow_limit_valid || follow_l!=0 || follow_r!=0)
   $fatal(1,"follow recalculation withdrew an established stop");
  ticks(500);
  laser_front_ok=0;#1;
  if(hold || follow_limit_valid)$fatal(1,"missing ranging changed legacy follow policy");
  follow_mode=0;laser_front_ok=1;velocity_l=0;original_control_enabled=0;#1;
  if(scope || takeover)$fatal(1,"original lidar control gate bypassed");
  original_control_enabled=1;nav_mode=1;goal_valid=0;#1;
  if(!scope || !hold || current)$fatal(1,"unready navigation angle left a plan live");
  @(negedge clk);saved_diagnostic=diagnostic;goal_cd=1234;map_scan_id=77;map_age_ms=123;
  #1;if(diagnostic!==saved_diagnostic)$fatal(1,"C8 bundle changed between sampling edges");
  sampled_time=dut.time_ms; ticks(1);
  if(diagnostic[287:272]!=1234 || diagnostic[79:64]!=77 || diagnostic[111:96]!=123 ||
     diagnostic[63:32]!=sampled_time || diagnostic[199:192]!=255 || diagnostic[31:16]&16'h0010)
   $fatal(1,"C8 registered bundle mixed contexts or timestamp");
  $display("PASS lidar controller: atomic cap adoption, continuous follow hold, missing-data policy and original control gate");$finish;
 end
 initial begin #130000;$fatal(1,"controller timeout");end
endmodule







