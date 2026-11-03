`timescale 1ns/1ps
// Compare zero-deadband compatibility with the bounded, fixed-anchor observer.
// Jitter changes are more frequent than SETTLE_MS; slow real movement advances
// by just one count per step and must still reset the stationary window.
module tb_settle_jitter;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,armed=0,cmd_update=0;
 reg [2:0] command=0;reg [7:0] speed=16;
 reg signed [31:0] pl=0,pr=0;
 reg signed [15:0] yaw=0;
 reg [15:0] dl=150,dr=180;reg lu=0,ru=0;
 wire signed [15:0] tl,tr,strict_l,strict_r;
 wire brake,strict_brake;wire [3:0] state,strict_state;
 wire [15:0] fault,strict_fault;
 rb_safety_control #(.CLK_HZ(1000),.SETTLE_MS(20),.STEP_TIMEOUT_MS(100),
  .SCAN_WAIT_MS(1000),.TURN_TIMEOUT_MS(1000),.DETOUR_TIMEOUT_MS(10000),
  .TURN_TRACK_MM(100),.SETTLE_JITTER_COUNTS(1)) dut(
  .clk(clk),.rst_n(rst_n),.link_ok(1'b1),.imu_ok(1'b1),
  .left_ok(1'b1),.right_ok(1'b1),.estop_ok(1'b1),
  .armed(armed),.cmd_update(cmd_update),.command(command),.speed(speed),
  .dl(dl),.dr(dr),.left_update(lu),.right_update(ru),
  .pos_l(pl),.pos_r(pr),.motor_fault(1'b0),.yaw(yaw),.yaw_update(1'b1),
  .target_l(tl),.target_r(tr),.brake(brake),.state(state),.fault(fault),
  .turn_error(),.turn_error_valid());
 rb_safety_control #(.CLK_HZ(1000),.SETTLE_MS(20),.STEP_TIMEOUT_MS(100),
  .SCAN_WAIT_MS(1000),.TURN_TIMEOUT_MS(1000),.DETOUR_TIMEOUT_MS(10000),
  .TURN_TRACK_MM(100)) strict_default(
  .clk(clk),.rst_n(rst_n),.link_ok(1'b1),.imu_ok(1'b1),
  .left_ok(1'b1),.right_ok(1'b1),.estop_ok(1'b1),
  .armed(armed),.cmd_update(cmd_update),.command(command),.speed(speed),
  .dl(dl),.dr(dr),.left_update(lu),.right_update(ru),
  .pos_l(pl),.pos_r(pr),.motor_fault(1'b0),.yaw(yaw),.yaw_update(1'b1),
  .target_l(strict_l),.target_r(strict_r),.brake(strict_brake),
  .state(strict_state),.fault(strict_fault),.turn_error(),.turn_error_valid());
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit good,input string message);
  if(!good)$fatal(1,"%s state=%0d strict=%0d pos=%0d/%0d yaw=%0d targets=%0d/%0d fault=%h strict_fault=%h",
   message,state,strict_state,$signed(pl),$signed(pr),$signed(yaw),
   $signed(tl),$signed(tr),fault,strict_fault);
 endtask
 task reset_near(input signed [31:0] left_start,input signed [31:0] right_start);begin
  @(negedge clk);rst_n=0;armed=0;cmd_update=0;command=0;speed=16;
  pl=left_start;pr=right_start;yaw=0;dl=150;dr=180;lu=0;ru=0;cycles(3);
  @(negedge clk);rst_n=1;cycles(4);
  @(negedge clk);cmd_update=1;cycles(1);
  @(negedge clk);cmd_update=0;armed=1;command=1;cycles(2);
  check(state==1 && brake && fault==0,"near obstacle starts a fresh stop window");
 end endtask
 task turn_left90;
  integer i;
  begin
   for(i=0;i<60;i=i+1)begin
    @(negedge clk);pl=pl-10;pr=pr+10;yaw=yaw+150;cycles(3);
   end
   check(state==3 && brake && fault==0,"physical left90 brakes before stopped sampling");
  end
 endtask
 task stop_command;begin
  @(negedge clk);command=0;#1;
  check(brake && tl==0 && tr==0,"STOP overrides the motion outputs immediately");
  cycles(3);check(state==0 && brake && fault==0,"STOP cancels the maneuver");
 end endtask

 integer i;
 reg signed [31:0] base_l,base_r;
 initial begin
  // Replay the observed two-position pattern: the stationary right encoder
  // toggles between zero and minus one, while every20ms yaw frame is valid.
  reset_near(0,0);
  for(i=0;i<120;i=i+1)begin
   @(negedge clk);pr=(i%10<5)?0:-1;cycles(1);
   if(i<15)check(state==1 && brake,"deadband never skips the full stationary duration");
  end
  check(state==2 && !brake && fault==0,
   "bounded one-count right jitter still completes the stop-before-turn window");
  check(strict_state==7 && strict_fault==16'h0200 && strict_brake,
   "default zero deadband preserves strict behavior and its settle timeout");

  // After an actual90deg turn, two-wheel jitter around fixed anchors must
  // allow a new stationary pose and real stopped sonar frames to be accepted.
  @(negedge clk);pr=0;cycles(3);turn_left90();base_l=pl;base_r=pr;
  for(i=0;i<35;i=i+1)begin
   @(negedge clk);pl=base_l+((i%2)?1:-1);pr=base_r+((i%2)?-1:1);cycles(1);
  end
  check(state==3 && brake && fault==0,"turn-end bounded jitter remains safely stopped during acquisition");
  @(negedge clk);dl=1000;dr=1100;lu=1;ru=1;cycles(1);
  @(negedge clk);lu=0;ru=0;cycles(5);
  check(state==2 && tl>0 && tr<0 && fault==0,
   "fresh left range is accepted after the jitter-tolerant stationary window and starts right180");
  stop_command();

  // Ignoring the size of each single step would incorrectly accept this.
  // A fixed anchor instead sees the cumulative second count and resets.
  reset_near(0,0);
  for(i=0;i<24;i=i+1)begin
   @(negedge clk);pl=pl+1;cycles(5);
   check(state!=2 && brake,"slow one-count left drift cannot be declared stopped");
  end
  check(state==7 && fault==16'h0200,"continuous slow left motion reaches bounded settle timeout");

  reset_near(0,0);
  for(i=0;i<24;i=i+1)begin
   @(negedge clk);pr=pr-1;cycles(5);
   check(state!=2 && brake,"slow negative right drift also resets the stationary window");
  end
  check(state==7 && fault==16'h0200,"right wheel is checked independently of left");

  reset_near(0,0);
  for(i=0;i<24;i=i+1)begin
   @(negedge clk);pl=pl+1;pr=pr-1;cycles(5);
   check(state!=2 && brake,"opposed wheel motion cannot cancel in the stationary test");
  end
  check(state==7 && fault==16'h0200,"two-wheel slow rotation is not hidden by a zero position sum");

  // +/-1 is modular even at the signed32 position seam. In particular,
  // 0x7fffffff ->0x80000000 is one count, not a giant displacement.
  reset_near(32'h7fffffff,32'h80000000);
  for(i=0;i<35;i=i+1)begin
   @(negedge clk);
   pl=(i%2)?32'h7fffffff:32'h80000000;
   pr=(i%2)?32'h80000000:32'h7fffffff;
   cycles(1);
  end
  check(state==2 && !brake && fault==0,"bounded two-wheel jitter across signed32 wrap may settle");
  stop_command();

  reset_near(32'h7fffffff,32'h80000000);
  for(i=0;i<24;i=i+1)begin
   @(negedge clk);pl=pl+1;pr=pr-1;cycles(5);
   check(state!=2 && brake,"cumulative motion across signed32 wrap still exceeds the fixed deadband");
  end
  check(state==7 && fault==16'h0200,"modular slow drift times out instead of passing or fabricating a jump");

  reset_near(0,0);
  for(i=0;i<10;i=i+1)begin
   @(negedge clk);pr=(i%2)?0:-1;cycles(1);
  end
  check(state==1 && brake,"STOP-before-settled scenario has not entered a turn");
  stop_command();
  @(negedge clk);command=1;cycles(2);
  for(i=0;i<15;i=i+1)begin
   @(negedge clk);pr=(i%2)?0:-1;cycles(1);
   check(state==1 && brake,"restart requires a new full settle interval despite earlier bounded jitter");
  end
  cycles(12);check(state==2 && fault==0,"restart can turn after its new stationary interval completes");
  stop_command();

  $display("PASS settle jitter: observed +/-1 pattern, default strict compatibility, post-turn sonar, left/right/opposed slow drift, signed32 wrap, STOP and fresh restart window");
  $finish;
 end
 initial begin #1000000;$fatal(1,"settle jitter test hung");end
endmodule
