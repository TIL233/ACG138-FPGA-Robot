`timescale 1ns/1ps
module tb_predictive_motion;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,en=1,brake=0,healthy=1,clear=0,sample=0,hv=0;
 reg signed [15:0] rl=8,rr=8,vl=0,vr=0,he=0;
 reg [31:0] stamp=0;
 wire signed [15:0] ol,orr,pe,sl,sr;wire busy,fault,valid;
 wire [15:0] gen,cycles_used,flags;wire [3:0] selected;wire [31:0] cost,rt;
 rb_predictive_motion #(.YAW_GAIN_Q8(1024)) dut(
  .clk(clk),.rst_n(rst_n),.enabled(en),.brake(brake),.healthy(healthy),.clear_fault(clear),
  .sample(sample),.heading_valid(hv),.heading_error(he),.reference_l(rl),.reference_r(rr),
  .velocity_l(vl),.velocity_r(vr),.sample_time(stamp),.output_l(ol),.output_r(orr),
  .busy(busy),.fault(fault),.result_valid(valid),.generation(gen),.solve_cycles(cycles_used),
  .selected(selected),.cost(cost),.predicted_error(pe),.flags(flags),.result_time(rt),.sampled_l(sl),.sampled_r(sr));
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit good,input string msg);if(!good)$fatal(1,"%s out=%d/%d fault=%b phase=%d",msg,ol,orr,fault,dut.phase);endtask
 task trigger;begin @(negedge clk);stamp=stamp+20;sample=1;cycles(1);@(negedge clk);sample=0;end endtask
 task evaluate;reg [15:0] old;begin old=gen;trigger();wait(gen!=old);#1;@(negedge clk);end endtask
 task stop;begin brake=1;cycles(2);brake=0;end endtask
 integer i,prev_l,prev_r;reg [15:0] before_gen;integer wvl,wvr;
 initial begin
  cycles(4);rst_n=1;cycles(2);
  check(ol==0 && orr==0,"no unsolved result may drive wheels");
  evaluate();check(ol==2 && orr==2 && valid && !fault,"bounded first forward candidate");
  check(cycles_used<220 && rt==20 && sl==8 && sr==8,"bounded solve latency and coherent input tag");
  check(flags[0] && flags[1] && flags[4] && flags[5] && flags[6] && !flags[7],"hardware lanes and nominal model are reported truthfully");
  prev_l=ol;prev_r=orr;
  for(i=0;i<12;i=i+1)begin
   vl=ol;vr=orr;evaluate();
   check(ol>=0 && ol<=8 && orr>=0 && orr<=8,"forward direction and command cap");
   check(ol<=prev_l+2 && orr<=prev_r+2,"per-window acceleration bound");
   prev_l=ol;prev_r=orr;
  end
  check(ol==8 && orr==8,"tracking reaches the requested steady speed");
  rl=3;rr=3;#1;check(ol<=3 && orr<=3,"speed reduction clamps without waiting for solve");
  rl=-8;rr=-8;#1;check(ol==0 && orr==0,"reversal cannot leak old positive command");
  evaluate();check(ol>=-2 && ol<=0 && orr>=-2 && orr<=0,"reverse starts within bounded command set");
  brake=1;#1;check(ol==0 && orr==0,"STOP bypasses pending result before clock");cycles(2);brake=0;
  // A measured yaw gain lets the score anticipate turn inertia. With wheels
  // already rotating, the 0.3deg remaining goal should select early braking.
  rl=-8;rr=8;vl=-8;vr=8;he=30;hv=1;evaluate();
  check(ol==0 && orr==0 && pe<0,"near-angle prediction chooses braking despite nonzero turn request");
  stop();he=9000;evaluate();check(ol==-2 && orr==2,"far-angle goal allows continued bounded pivot");
  // The wheel model remains convergent when the simulated plant is slower
  // than the nominal predictor. No physical motor is commanded by this test.
  stop();hv=0;rl=16;rr=16;wvl=0;wvr=0;
  for(i=0;i<100;i=i+1)begin
   vl=wvl>>>8;vr=wvr>>>8;evaluate();
   wvl=wvl+(($signed(ol)*256-wvl)>>>3);
   wvr=wvr+(($signed(orr)*256-wvr)>>>3);
   check(!fault && ol>=0 && ol<=16 && orr>=0 && orr<=16,"model mismatch stays within intent bounds");
  end
  check(vl>=15 && vr>=15,"slower plant converges without a fabricated calibrated-model claim");
  // Pending results computed for superseded references must never publish.
  before_gen=gen;trigger();cycles(15);rr=6;cycles(3);
  check(!busy && gen==before_gen && orr<=6,"command change cancels old prediction");
  evaluate();healthy=0;#1;check(ol==0 && orr==0,"feedback loss bypasses predictor immediately");cycles(2);healthy=1;
  trigger();cycles(10);trigger();cycles(1);check(fault && !valid && ol==0,"overlapping sample latches bounded-deadline fault");
  clear=1;cycles(1);clear=0;evaluate();check(!fault && valid,"explicit STOP clear recovers solver");
  stop();trigger();cycles(4);force dut.elapsed=1024;cycles(1);release dut.elapsed;
  check(fault && ol==0,"watchdog cancels a stalled prediction");
  clear=1;cycles(1);clear=0;rl=100;rr=100;cycles(1);check(fault,"out-of-range requests are rejected");
  clear=1;cycles(1);clear=0;rl=8;rr=8;en=0;#1;check(ol==8 && orr==8,"explicit disabled configuration preserves original path");
  $display("PASS predictive motion: bounded horizon/parallel candidates/slew/anticipatory turn brake/model mismatch/STOP/cancel/watchdog");$finish;
 end
 initial begin #1000000;$fatal(1,"predictive motion test timeout");end
endmodule
