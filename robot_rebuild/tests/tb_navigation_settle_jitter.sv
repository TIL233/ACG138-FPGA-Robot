`timescale 1ns/1ps
module tb_navigation_settle_jitter;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,run=0,wr=0,commit=0,reset_pose=0;
 reg signed [31:0] pl=0,pr=0;
 wire [2:0] command;wire done,failed,active;wire [3:0] state;
 rb_navigation #(.CLK_HZ(10000),.SETTLE_MS(200),.TOTAL_TIMEOUT_MS(5000)) dut(
  .clk(clk),.rst_n(rst_n),.run(run),.healthy(1'b1),.safety_state(4'd0),
  .pos_l(pl),.pos_r(pr),.yaw(16'sd0),.route_write(wr),.route_index(3'd0),
  .way_x(16'sd0),.way_y(16'sd0),.route_commit(commit),.route_count(4'd1),
  .route_clear(1'b0),.pose_reset(reset_pose),.command(command),.speed(),
  .active(active),.done(done),.failed(failed),.state(state),.index(),.x(),.y(),
  .heading(),.route_ready(),.pose_ready(),.turn_error(),.turn_error_valid());
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit yes,input string message);
  if(!yes)$fatal(1,"%s state=%0d done=%b still=%0d pos=%0d/%0d",message,state,done,dut.still_ms,pl,pr);
 endtask
 task start;
  begin
   @(negedge clk);rst_n=0;run=0;wr=0;commit=0;reset_pose=0;pl=0;pr=0;cycles(3);
   @(negedge clk);rst_n=1;wr=1;cycles(1);
   @(negedge clk);wr=0;commit=1;reset_pose=1;cycles(1);
   @(negedge clk);commit=0;reset_pose=0;cycles(3);run=1;
  end
 endtask
 initial begin
  start();
  // Every real20ms observation alternates0/-1; the old previous-sample test
  // resets its timer on every sample despite there being no net displacement.
  for(integer k=0;k<18;k=k+1)begin
   @(negedge clk);pr=(k%2)?0:-1;cycles(200);
   if(k<8)check(!done,"arrival still waits its full200ms despite bounded jitter");
  end
  check(done && !failed && !active && command==0,
   "arrival completes stopped under the observed stationary right-wheel jitter");
  start();
  for(integer k=0;k<12;k=k+1)begin
   @(negedge clk);pl=pl+1;pr=pr+1;cycles(800);
   check(!done && !failed && command==0,"slow one-count creep cannot falsely complete arrival");
  end
  cycles(2400);check(done && !failed,"arrival completes after real creep stops");
  start();cycles(450);check(state==3 && !done,"STOP scenario starts in arrival settling");
  @(negedge clk);run=0;#1;check(command==0,"STOP gates commands immediately");
  cycles(2400);check(!done && !active && command==0,"STOP prevents an inactive arrival from completing");
  $display("PASS navigation settling jitter: observed pattern, full window, fixed anchors, monotonic creep and STOP");$finish;
 end
 initial begin #500000;$fatal(1,"navigation settling jitter hung");end
endmodule
