`timescale 1ns/1ps
module tb_wheel_actuator_boundary;
 reg clk=0; always #5 clk=~clk;
 reg rst_n=0,brake=1,clear_fault=0,planner_pause=0;
 reg signed[15:0] target=12;
 reg signed[31:0] position=0;
 wire signed[15:0] velocity;
 wire fault,pwm,in1,in2,actuator_pending;
 rb_wheel_control #(.CLK_HZ(20000),.PAUSE_ENABLE(1),.ACTUATOR_PIPELINE(1)) dut(.*);
 task tick(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task reset_drive;begin
  @(negedge clk);rst_n=0;brake=1;planner_pause=0;clear_fault=0;target=12;position=0;
  tick(3);@(negedge clk);rst_n=1;tick(3);@(negedge clk);brake=0;
  tick(410);if(dut.duty<=0)$fatal(1,"drive setup failed");
 end endtask
 task threshold(input integer movement);begin
  wait(dut.timer==399);@(negedge clk);
  dut.stall=49;dut.direction_guard=10;dut.wrong_direction=4;
  position=$signed(dut.last_pos)+movement;
 end endtask
 integer keep_i,keep_d,sgn,k,offset;
 initial begin
  reset_drive();threshold(0);tick(1);
  if(fault)$fatal(1,"advanced supervision must publish one clock after the captured WIN event");
  tick(1);if(!fault)$fatal(1,"captured stall fault missing at WIN+1");
  reset_drive();threshold(-2);tick(1);
  if(fault)$fatal(1,"wrong-direction fault bypassed event boundary");
  tick(1);if(!fault)$fatal(1,"captured wrong-direction fault missing");
  reset_drive();threshold(0);brake=1;#1;
  if(!(pwm&&in1&&in2))$fatal(1,"raw STOP did not immediately brake");
  tick(2);if(fault)$fatal(1,"exact-WIN STOP falsely latched stall");
  reset_drive();threshold(513);brake=1;clear_fault=1;tick(1);
  if(!fault)$fatal(1,"severe delta must win immediately over STOP and clear_fault");
  reset_drive();threshold(0);planner_pause=1;target=0;tick(1);
  if(fault)$fatal(1,"raw pause supervision published before its captured event");
  tick(1);if(!fault)$fatal(1,"raw pause erased the previous nonzero stall intent");
  reset_drive();threshold(-2);planner_pause=1;target=0;tick(2);
  if(!fault)$fatal(1,"raw pause erased the previous wrong-direction intent");
  for(sgn=-1;sgn<=1;sgn=sgn+2)begin
   reset_drive();@(negedge clk);target=sgn*12;tick(410);
   keep_i=dut.integral;keep_d=dut.duty;
   repeat(3)begin
    @(negedge clk);planner_pause=1;target=0;#1;
    if(!(pwm&&in1&&in2))$fatal(1,"raw pause failed");
    tick(1);@(negedge clk);planner_pause=0;target=sgn*12;
    tick(2);if(!(pwm&&in1&&in2))$fatal(1,"queued pause released stale duty");
    tick(8);if(dut.integral!=keep_i || dut.duty!=keep_d)$fatal(1,"repeat resume added KI or SLEW");
   end
   // A target bounces while its previous packet has been captured.
   @(negedge clk);target=sgn*6;tick(1);@(negedge clk);target=sgn*12;
   tick(2);if(!(pwm&&in1&&in2))$fatal(1,"capture bounce revived an old duty");
   tick(8);if(dut.target_hold!=sgn*12 || in1&&in2)$fatal(1,"latest target did not commit");
   // A hard event cancels every arithmetic phase; its return cannot release old duty.
   for(k=1;k<=4;k=k+1)begin
    @(negedge clk);planner_pause=1;tick(2);@(negedge clk);planner_pause=0;
    wait(dut.phase==k);@(negedge clk);brake=1;#1;
    if(!(pwm&&in1&&in2))$fatal(1,"hard event during arithmetic leaked drive");
    tick(1);@(negedge clk);brake=0;tick(2);
    if(!(pwm&&in1&&in2))$fatal(1,"hard-event queue released before replacement commit");
    tick(8);
   end
  end
  // The physical timeout retains the old raw 1024-edge budget. Internal
  // clearing follows its captured hard event by one edge, never permitting drive.
  reset_drive();keep_i=dut.integral;
  @(negedge clk);planner_pause=1;target=0;tick(1022);
  if(dut.pause_timeout)$fatal(1,"pause timeout fired before edge1023");
  @(negedge clk);planner_pause=0;target=12;tick(1);
  @(negedge clk);planner_pause=1;target=0;#1;
  if(dut.pause_timeout)$fatal(1,"sampled low gap did not reset physical pause budget");
  tick(1022);if(dut.pause_timeout)$fatal(1,"fresh pause inherited old budget");
  tick(1);if(!dut.pause_timeout || !(pwm&&in1&&in2))$fatal(1,"edge1023 physical timeout missing");
  tick(2);if(dut.integral!=0 || dut.duty!=0)$fatal(1,"queued timeout did not clear by edge1025");
  reset_drive();@(negedge clk);planner_pause=1;tick(1022);
  @(negedge clk);planner_pause=0;#1;planner_pause=1;tick(1);
  if(!dut.pause_timeout)$fatal(1,"unsampled gap incorrectly renewed budget");
  tick(2);if(dut.duty!=0)$fatal(1,"unsampled-gap timeout failed");
  for(sgn=-1;sgn<=1;sgn=sgn+2)begin
   for(offset=-1;offset<=1;offset=offset+1)begin
    reset_drive();@(negedge clk);target=sgn*4;tick(810);
    wait(dut.timer==398);@(negedge clk);tick(offset+1);@(negedge clk);
    keep_i=dut.integral;keep_d=dut.duty;planner_pause=1;target=0;
    tick(2);if(dut.integral!=keep_i || dut.duty!=keep_d)$fatal(1,"WIN +/-1 signed pause lost committed PI");
    @(negedge clk);target=-sgn*4;#1;
    if(!(pwm&&in1&&in2))$fatal(1,"signed reversal lost raw barrier");
    tick(2);if(dut.duty!=0 || dut.integral!=0 || dut.duty_anchor!=0)$fatal(1,"captured reversal kept old PI");
    @(negedge clk);planner_pause=0;tick(12);
    if((sgn>0 && in1&&!in2) || (sgn<0 && in2&&!in1))$fatal(1,"old direction survived reversal");
   end
  end
  $display("PASS actuator boundary supervision/STOP/bounce/commit");$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
