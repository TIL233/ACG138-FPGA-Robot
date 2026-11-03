`timescale 1ns/1ps
module tb_wheel_planner_pause;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,brake=1,clear_fault=0,planner_pause=0;
 reg signed[15:0]target=12;reg signed[31:0]position=0;
 wire signed[15:0]velocity;wire fault,pwm,in1,in2,actuator_pending;
 wire signed[15:0]legacy_velocity;wire legacy_fault,legacy_pwm,legacy_in1,legacy_in2;
 integer saved_duty,saved_integral,saved_anchor,sign_case;
 rb_wheel_control #(.CLK_HZ(1000000),.MAX_DUTY(2000),.PAUSE_ENABLE(1)) dut(.*);
 rb_wheel_control #(.CLK_HZ(1000000)) legacy(
  .clk(clk),.rst_n(rst_n),.brake(brake),.clear_fault(clear_fault),.target(target),.position(position),
  .velocity(legacy_velocity),.fault(legacy_fault),.pwm(legacy_pwm),.in1(legacy_in1),.in2(legacy_in2),
  .planner_pause(planner_pause),.actuator_pending());
 integer raw_pause_count=0,lagged_pause_count=0;
 always @(posedge clk)begin
  if(!rst_n)begin raw_pause_count=0;lagged_pause_count=0;end
  else begin
   lagged_pause_count=raw_pause_count;
   if(!planner_pause)raw_pause_count=0;
   else if(raw_pause_count<1024)raw_pause_count=raw_pause_count+1;
  end
  #1;
  if(dut.pause_cycles!==lagged_pause_count)$fatal(1,"PI pause counter lacks queued boundary");
  if(dut.pause_timeout!==(raw_pause_count>=1023))$fatal(1,"compensated timeout differs from raw budget");
 end
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task seed;begin brake=0;planner_pause=0;target=12;ticks(20010);
  if(dut.duty<=0)$fatal(1,"PI setup failed");end endtask
 task reset_drive;begin
  brake=1;clear_fault=1;planner_pause=0;target=12;position=0;ticks(3);clear_fault=0;seed();
 end endtask
 task threshold_window(input integer movement);begin
  wait(dut.timer==19999);@(negedge clk);
  force dut.stall=6'd49;force dut.direction_guard=8'd10;force dut.wrong_direction=8'd4;
  #1;release dut.stall;release dut.direction_guard;release dut.wrong_direction;
  position=$signed(dut.last_pos)+movement;
 end endtask
 initial begin
  ticks(3);rst_n=1;ticks(3);seed();
  // The advanced soft zero classification has one local clock boundary.
  // Its queued flag must keep the bridge stopped until old state is cleared.
  saved_duty=dut.duty;@(negedge clk);target=0;#1;
  if(!(pwm && in1 && in2))$fatal(1,"raw soft zero did not brake immediately");
  ticks(1);
  if(dut.duty!=saved_duty)$fatal(1,"soft zero PI classification lacks local register boundary");
  @(negedge clk);target=12;#1;
  if(!(pwm && in1 && in2))$fatal(1,"queued soft zero released old duty");
  ticks(1);if(dut.duty!=0 || dut.integral!=0)$fatal(1,"queued soft zero failed to clear PI");
  brake=1;ticks(3);seed();
  // One-clock pause with simultaneous target zero must neither clear PI nor
  // release a stale duty between raw and queued pause or during recompute.
  saved_duty=dut.duty;saved_integral=dut.integral;
  @(negedge clk);planner_pause=1;target=0;#1;
  if(!(pwm && in1 && in2))$fatal(1,"raw pause+zero not immediate");
  ticks(1);@(negedge clk);planner_pause=0;target=6;#1;
  if(!(pwm && in1 && in2))$fatal(1,"one-clock pause release gap");
  ticks(2);
  if(dut.duty!=saved_duty || dut.integral!=saved_integral || !(pwm && in1 && in2))
   $fatal(1,"queued pause lost state or bridge barrier");
  ticks(7);if(dut.target_hold!=6 || dut.integral!=saved_integral || (in1&&in2))
   $fatal(1,"single pulse resume not current or extra integration");
  brake=1;ticks(3);seed();
  // Repeated recalculation inside one 20ms window must not create extra
  // slew steps or integration time. Raw age/range guards may repeat quickly.
  saved_duty=dut.duty;saved_integral=dut.integral;
  repeat(3)begin
   planner_pause=1;ticks(2);planner_pause=0;ticks(8);
   if(dut.duty!=saved_duty || dut.integral!=saved_integral)
    $fatal(1,"repeated resume accelerated PI slew within one feedback window");
  end
  ticks(20000-dut.timer+6);
  if(dut.duty<=saved_duty || dut.duty>saved_duty+64 || dut.integral<=saved_integral)
   $fatal(1,"a genuine feedback window must advance the normal bounded PI step");
  // Pause across a genuine feedback boundary so resume uses new velocity.
  wait(dut.timer==19990);@(negedge clk);planner_pause=1;
  saved_duty=dut.duty;saved_integral=dut.integral;
  #1;if(!(pwm && in1 && in2))$fatal(1,"planner pause must short-brake immediately");
  if(legacy_in1 && legacy_in2)$fatal(1,"default-off pause changed legacy output");
  target=0;position=4;ticks(20);
  if(dut.duty!=saved_duty || dut.integral!=saved_integral || velocity!=4)
   $fatal(1,"bounded pause lost PI state or feedback");
  target=6;@(negedge clk);planner_pause=0;#1;
  if(!(pwm && in1 && in2))$fatal(1,"resume leaked saved torque before recomputation");
  ticks(2);target=4;ticks(8);
  if(dut.target_hold!=4 || dut.error_hold!=0 || dut.duty<=0 || (in1 && in2))
   $fatal(1,"resume did not promptly recompute latest target/feedback");
  // Raw hard stops always win, including during a planner pause.
  planner_pause=1;ticks(3);brake=1;ticks(1);
  if(dut.duty!=0 || dut.integral!=0)$fatal(1,"hard brake masked by pause");
  planner_pause=0;ticks(3);seed();
  planner_pause=1;target=0;ticks(1030);
  if(dut.duty!=0 || dut.integral!=0 || !(pwm && in1 && in2))
   $fatal(1,"long hold did not become a clearing hard brake");
  planner_pause=0;brake=1;ticks(3);seed();
  planner_pause=1;target=-12;#1;
  if(!(pwm && in1 && in2))$fatal(1,"reverse during pause did not brake");
  ticks(1);if(dut.duty!=0 || dut.integral!=0)$fatal(1,"reverse retained old PI state");
  // Nonterminal supervision must retain the old four-count intent when
  // raw pause and zero target arrive on the first, not-yet-queued WIN edge.
  reset_drive();target=4;ticks(20010);wait(dut.timer==19999);@(negedge clk);
  force dut.stall=6'd48;#1;release dut.stall;
  planner_pause=1;target=0;ticks(1);
  if(fault || dut.previous_target!=4 || dut.stall!=49)$fatal(1,"raw pause WIN lost previous intent before next stall window");
  ticks(5);target=4;planner_pause=0;ticks(8);wait(dut.timer==19999);@(negedge clk);
  planner_pause=1;target=0;ticks(1);if(!fault)$fatal(1,"repeated bounded pause suppressed accumulated stall");
  reset_drive();target=4;ticks(20010);wait(dut.timer==19999);@(negedge clk);
  force dut.direction_guard=8'd10;force dut.wrong_direction=8'd3;#1;
  release dut.direction_guard;release dut.wrong_direction;position=$signed(dut.last_pos)-4;
  planner_pause=1;target=0;ticks(1);
  if(fault || dut.previous_target!=4 || dut.wrong_direction!=4)$fatal(1,"raw pause WIN lost previous intent before next wrong-dir window");
  ticks(5);target=4;planner_pause=0;ticks(8);wait(dut.timer==19999);@(negedge clk);
  position=$signed(dut.last_pos)-4;planner_pause=1;target=0;ticks(1);
  if(!fault)$fatal(1,"repeated bounded pause suppressed accumulated wrong direction");
  // The retained intent must also catch reversal immediately after that
  // first paused WIN, clearing positive duty/anchor before any reverse drive.
  reset_drive();target=4;ticks(20010);wait(dut.timer==19999);@(negedge clk);
  planner_pause=1;target=0;ticks(1);@(negedge clk);target=-4;#1;
  if(!dut.hard_brake || !(pwm&&in1&&in2))$fatal(1,"paused first-WIN reversal escaped raw hard brake");
  ticks(1);if(dut.duty!=0 || dut.duty_anchor!=0 || dut.integral!=0)$fatal(1,"reverse retained positive duty anchor");
  planner_pause=0;ticks(10);if(in1 && !in2)$fatal(1,"old positive torque survived reverse resume");
  // Both wheel directions must preserve PI state when zero is merely a
  // raw pause placeholder on the first WIN edge before the soft queue fills.
  for(sign_case=-1;sign_case<=1;sign_case=sign_case+2)begin
   reset_drive();target=sign_case*4;ticks(40010);
   if(dut.duty*sign_case<=0 || dut.integral*sign_case<=0)$fatal(1,"signed pause fixture failed");
   wait(dut.timer==19999);@(negedge clk);
   saved_duty=dut.duty;saved_integral=dut.integral;saved_anchor=dut.duty_anchor;
   planner_pause=1;target=0;#1;if(!(pwm&&in1&&in2))$fatal(1,"signed raw pause physical brake delayed");
   ticks(1);
   // A real WIN advances the anchor to the preceding published duty, but
   // neither signed PI duty nor integral may be reset by the zero placeholder.
   if(dut.duty!=saved_duty || dut.integral!=saved_integral || dut.duty_anchor!=saved_duty || dut.previous_target!=sign_case*4)
    $fatal(1,"signed first-WIN pause zero cleared PI state sign=%0d duty=%0d integral=%0d",sign_case,dut.duty,dut.integral);
   @(negedge clk);target=-sign_case*4;#1;
   if(!dut.hard_brake || !(pwm&&in1&&in2))$fatal(1,"signed queued reversal failed immediate barrier");
   ticks(1);if(dut.duty!=0 || dut.duty_anchor!=0 || dut.integral!=0)$fatal(1,"signed queued reversal kept old PI");
   planner_pause=0;#1;if(!(pwm&&in1&&in2))$fatal(1,"signed queued reversal released saved torque");
   ticks(10);
   if((sign_case>0 && in1&&!in2) || (sign_case<0 && in2&&!in1))$fatal(1,"old signed torque survived queued reversal");
  end
  // Raw hard brake on the precise fault-supervision boundary wins over
  // both maximum stall and wrong-direction counters (near/STOP semantics).
  reset_drive();threshold_window(0);brake=1;#1;
  if(!(pwm&&in1&&in2))$fatal(1,"boundary STOP physical brake delayed");
  ticks(1);if(fault || dut.stall!=0 || dut.duty!=0)$fatal(1,"boundary STOP falsely latched stall");
  reset_drive();threshold_window(-4);planner_pause=1;target=0;brake=1;ticks(1);
  if(fault || dut.wrong_direction!=0 || dut.duty!=0)$fatal(1,"boundary near hard brake falsely latched wrong direction");
  // A pure soft pause must preserve, not erase, true raw-window protection.
  reset_drive();threshold_window(0);planner_pause=1;target=0;ticks(1);
  if(!fault)$fatal(1,"pause suppressed real stall at feedback boundary");
  reset_drive();threshold_window(-4);planner_pause=1;target=0;ticks(1);
  if(!fault)$fatal(1,"pause suppressed real wrong-direction fault");
  reset_drive();threshold_window(0);target=0;planner_pause=0;ticks(1);
  if(fault || dut.stall!=0)$fatal(1,"raw soft zero used delayed fault exclusion");
  // A captured one-clock gap must reset the raw budget even though the
  // queued counter still holds the old near-threshold value for that edge.
  reset_drive();planner_pause=1;ticks(1022);@(negedge clk);planner_pause=0;ticks(1);
  @(negedge clk);planner_pause=1;#1;if(dut.pause_timeout)$fatal(1,"captured gap kept stale near-timeout");
  ticks(1);if(dut.pause_timeout)$fatal(1,"fresh raw assertion inherited stale timeout");
  ticks(1022);if(!dut.pause_timeout)$fatal(1,"contiguous edge1023 did not assert physical timeout");
  ticks(1);if(dut.duty!=0 || dut.integral!=0)$fatal(1,"edge1024 did not clear PI");
  // An unsampled low pulse cannot renew the old raw budget.
  reset_drive();planner_pause=1;ticks(1022);@(negedge clk);planner_pause=0;#1;planner_pause=1;#1;
  if(dut.pause_timeout)$fatal(1,"unsampled gap shortened budget");
  ticks(1);if(!dut.pause_timeout)$fatal(1,"unsampled gap incorrectly reset budget");
  ticks(1);if(dut.duty!=0)$fatal(1,"unsampled-gap timeout failed to clear");
  reset_drive();saved_duty=dut.duty;saved_integral=dut.integral;
  repeat(10)begin @(negedge clk);planner_pause=1;ticks(1);@(negedge clk);planner_pause=0;ticks(1);end
  ticks(10);if(dut.duty!=saved_duty || dut.integral!=saved_integral)$fatal(1,"repeated one-clock pulses added KI/SLEW");
  // Mode exit/hard cancel must consume queued pause without saved torque.
  planner_pause=1;ticks(1);brake=1;planner_pause=0;target=-12;#1;
  if(!(pwm&&in1&&in2))$fatal(1,"mode cancel released bridge");
  ticks(1);if(dut.duty!=0 || dut.integral!=0)$fatal(1,"mode cancel retained state");
  $display("PASS wheel planner pause: immediate physical brake, state/feedback preservation, bounded timeout, atomic resume, reverse/hard-stop priority and default-off compatibility");$finish;
 end
 initial begin #15000000;$fatal(1,"wheel pause timeout");end
endmodule
