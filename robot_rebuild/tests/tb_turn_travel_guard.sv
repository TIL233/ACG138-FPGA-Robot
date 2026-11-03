`timescale 1ns/1ps
module tb_turn_travel_guard;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,enable=0,count_enable=0,new_leg=0;
 reg [15:0] goal_cd=9000;
 reg signed [31:0] pl=0,pr=0;
 wire fault,configured,unconfigured_fault,unconfigured;
 wire invalid_fault,invalid_configured;
 rb_turn_travel_guard #(.TRACK_MM(100)) dut(
  .clk(clk),.rst_n(rst_n),.enable(enable),.count_enable(count_enable),.new_leg(new_leg),
  .goal_cd(goal_cd),.pos_l(pl),.pos_r(pr),.fault(fault),.configured(configured));
 rb_turn_travel_guard unmeasured(
  .clk(clk),.rst_n(rst_n),.enable(enable),.count_enable(count_enable),.new_leg(new_leg),
  .goal_cd(goal_cd),.pos_l(pl),.pos_r(pr),.fault(unconfigured_fault),.configured(unconfigured));
 rb_turn_travel_guard #(.TRACK_MM(100),.COUNTS_PER_M(0)) invalid(
  .clk(clk),.rst_n(rst_n),.enable(enable),.count_enable(count_enable),.new_leg(new_leg),
  .goal_cd(goal_cd),.pos_l(pl),.pos_r(pr),.fault(invalid_fault),.configured(invalid_configured));
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit yes,input string msg);
  if(!yes)$fatal(1,"%s fault=%b budget=%0d travel=%0d positions=%0d/%0d",
                 msg,fault,dut.budget_counts,dut.travel_counts,pl,pr);
 endtask
 task stop_guard;begin
  @(negedge clk);enable=0;count_enable=0;new_leg=0;cycles(2);
  check(!fault && !unconfigured_fault && !invalid_fault,"disable clears all latched faults");
 end endtask
 task leg(input integer goal);begin
  @(negedge clk);enable=1;count_enable=1;goal_cd=goal;new_leg=1;cycles(1);
  @(negedge clk);new_leg=0;cycles(3);
 end endtask
 task move(input integer left_step,input integer right_step);begin
  @(negedge clk);pl=pl+left_step;pr=pr+right_step;cycles(4);
 end endtask
 task fill_to_1600(input integer direction);
  integer i;begin
   for(i=0;i<10;i=i+1)move(-80*direction,80*direction);
  end
 endtask
 integer i;
 initial begin
  cycles(3);@(negedge clk);rst_n=1;cycles(2);
  check(configured && !unconfigured && !invalid_configured,"only measured valid geometry is configured");
  leg(9000);
  check(unconfigured_fault && invalid_fault,"new leg rejects zero track or invalid count calibration");
  check(!fault && dut.budget_counts==1601,"90 degree plus30 degree margin has ceil-rounded count budget");
  fill_to_1600(1);check(!fault && dut.travel_counts==1600,"left90 remains below independent travel limit");
  @(negedge clk);pl=pl-1;pr=pr+1;cycles(2);
  check(!fault,"travel samples pass through the registered arithmetic stages");
  cycles(1);check(fault,"left over-travel faults within three sample edges without IMU input");
  leg(9000);check(fault,"new leg cannot clear a fault");
  stop_guard();leg(9000);fill_to_1600(-1);
  check(!fault && dut.travel_counts==1600,"right rotation uses identical absolute count budget");
  move(1,-1);check(fault,"right over-travel also faults");

  stop_guard();leg(18000);check(dut.budget_counts==2801,"180 degree budget differs from90 degree budget");
  for(i=0;i<14;i=i+1)move(-100,100);
  check(!fault && dut.travel_counts==2800,"left180 remains below its limit");
  move(-1,1);check(fault,"left180 over-travel fails");
  stop_guard();leg(18000);
  for(i=0;i<14;i=i+1)move(100,-100);
  check(!fault && dut.travel_counts==2800,"right180 remains below its limit");
  move(1,-1);check(fault,"right180 over-travel fails");

  stop_guard();leg(9000);move(-400,400);move(400,-400);
  check(!fault && dut.travel_counts==1600,
        "reversing correction counts total travel rather than net displacement");
  move(-1,1);check(fault,"correction cannot reset the main-leg travel budget");

  stop_guard();leg(9000);move(-400,400);leg(9000);
  check(!fault && dut.travel_counts==0,"a genuinely new main leg receives a fresh budget");
  fill_to_1600(1);check(!fault,"new leg does not inherit the old leg travel");

  stop_guard();@(negedge clk);pl=32'h7ffffff0;pr=32'h80000010;cycles(2);leg(9000);
  move(40,-40);
  check(!fault && dut.travel_counts==80,"signed32 position wrap is a short modular displacement");
  move(-40,40);check(!fault && dut.travel_counts==160,"reverse32 wrap counts absolute travel");

  stop_guard();leg(9000);@(negedge clk);pl=pl+513;cycles(1);
  check(fault,"single-wheel513 count jump faults on first edge");
  stop_guard();leg(9000);@(negedge clk);pr=pr-513;cycles(1);
  check(fault,"negative single-wheel jump also faults immediately");
  stop_guard();leg(9000);move(512,0);check(!fault,"exact512 count step remains accepted");

  // Straight detour movement updates the encoder baseline but does not spend
  // the previous turn's budget. The next turn is explicitly a new leg.
  stop_guard();leg(9000);move(-100,100);
  @(negedge clk);count_enable=0;cycles(3);
  for(i=0;i<20;i=i+1)move(200,200);
  check(!fault && dut.travel_counts==200,"straight advance is excluded from turn travel");
  leg(9000);move(-10,10);
  check(!fault && dut.travel_counts==20,"counting resumes without importing paused displacement");

  // A new leg cancels in-flight old deltas and the old budget multiplication.
  stop_guard();leg(9000);@(negedge clk);pl=pl-300;pr=pr+300;cycles(1);
  @(negedge clk);new_leg=1;goal_cd=18000;cycles(1);
  @(negedge clk);new_leg=0;cycles(5);
  check(!fault && dut.travel_counts==0 && dut.budget_counts==2801,
        "new main leg discards prior pipeline results");
  @(negedge clk);new_leg=1;goal_cd=9000;cycles(1);
  @(negedge clk);goal_cd=18000;cycles(1);
  @(negedge clk);new_leg=0;cycles(4);
  check(!fault && dut.budget_counts==2801,"consecutive new legs discard stale goal products");

  stop_guard();leg(18150);check(!fault && dut.budget_counts==2821,"directed180 plus tolerance is valid");
  stop_guard();leg(18151);check(fault,"larger than allowed directed turn is rejected");
  stop_guard();leg(0);check(!fault && dut.budget_counts==401,"zero goal permits only the configured margin");
  move(-200,200);check(!fault,"zero-goal margin is available");
  move(-1,1);check(fault,"zero-goal margin is still finite");
  stop_guard();

  // Exercise saturation without billions of simulation cycles. The arithmetic
  // must saturate and latch failure rather than wrap back into the valid range.
  leg(9000);@(negedge clk);dut.travel_counts=32'hfffffffe;dut.budget_counts=32'hffffffff;
  move(-2,2);check(fault && dut.travel_counts==32'hffffffff,"travel counter saturates on overflow");
  stop_guard();
  $display("PASS turn travel guard: bidirectional90/180, correction budget, new legs, encoder wrap, jump, configuration, pause, pipeline, latch and saturation");
  $finish;
 end
 initial begin #1000000;$fatal(1,"turn travel guard test hung");end
endmodule
