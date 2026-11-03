`timescale 1ns/1ps
// Catches unsafe fixed stop distance, wraparound, request inflation and stale hold.
module tb_lidar_envelope;
 // 112 shift/add clocks + 320 restoring-division clocks + 11 registered
 // round/distance/scale-load clocks. Publication is the final clock only.
 localparam SOLVE_CYCLES=443;
 reg clk=0; always #5 clk=~clk;
 reg rst_n=0,sample=0,enabled=1,follow_mode=0,front_valid=1;
 reg [15:0] front_mm=1000,age_ms=0;
 reg signed [15:0] velocity_l=0,velocity_r=0,request_l=12,request_r=12;
 reg [15:0] registered_counts=0;reg registered_counts_valid=0;
 integer cycle,abort_clock;
 wire [7:0] published_cap;
 wire [2:0] age_cases_done;
 envelope_age_case #(0) age0(age_cases_done[0]);
 envelope_age_case #(20) age20(age_cases_done[1]);
 envelope_age_case #(65535) age65535(age_cases_done[2]);
 wire valid,hold,config_ok; wire signed [15:0] limited_l,limited_r;
 wire [15:0] stop_mm,slow_mm,speed_mm_s,latency_ms,solve_cycles;
 rb_lidar_envelope dut(.*);
 wire trusted_valid,trusted_hold;
 wire signed [15:0] trusted_l,trusted_r;
 wire [15:0] trusted_stop,trusted_cycles;
 rb_lidar_envelope #(.USE_REGISTERED_COUNTS(1)) trusted(
  .clk(clk),.rst_n(rst_n),.sample(sample),.enabled(enabled),.follow_mode(follow_mode),
  .front_valid(front_valid),.front_mm(front_mm),.age_ms(age_ms),
  .velocity_l(velocity_l),.velocity_r(velocity_r),.request_l(request_l),.request_r(request_r),
  .registered_counts(registered_counts),.registered_counts_valid(registered_counts_valid),
  .valid(trusted_valid),.hold(trusted_hold),.limited_l(trusted_l),.limited_r(trusted_r),
  .stop_mm(trusted_stop),.slow_mm(),.speed_mm_s(),.latency_ms(),.solve_cycles(trusted_cycles),.config_ok(),.published_cap());
 wire invalid_config,invalid_valid;
 rb_lidar_envelope #(.DECEL_MM_S2(0)) bad_config(
  .clk(clk),.rst_n(rst_n),.sample(sample),.enabled(enabled),.follow_mode(follow_mode),
  .front_valid(front_valid),.front_mm(front_mm),.age_ms(age_ms),
  .velocity_l(velocity_l),.velocity_r(velocity_r),.request_l(request_l),.request_r(request_r),
  .registered_counts(registered_counts),.registered_counts_valid(registered_counts_valid),
  .config_ok(invalid_config),.valid(invalid_valid),.hold(),.limited_l(),.limited_r(),
  .stop_mm(),.slow_mm(),.speed_mm_s(),.latency_ms(),.solve_cycles(),.published_cap());
 task tick(input integer n); repeat(n) begin @(posedge clk);#1;end endtask
 task run_solve;begin @(negedge clk);sample=1;tick(1);@(negedge clk);sample=0;tick(SOLVE_CYCLES);end endtask
 task abort_solve_at(input integer n);begin
  age_ms=10;@(negedge clk);sample=1;tick(1);@(negedge clk);sample=0;tick(n);
  front_mm=999;#1;if(valid)$fatal(1,"late input change did not revoke immediately");
  tick(1);front_mm=1000;tick(1);
  if(valid)$fatal(1,"cancelled late-stage result revived after restoring input");
  tick(SOLVE_CYCLES);
  if(valid)$fatal(1,"cancelled late-stage arithmetic published later");
  age_ms=0;run_solve();
  if(!valid || stop_mm!=242 || limited_l!=12 || limited_r!=12 || solve_cycles!=SOLVE_CYCLES)
   $fatal(1,"fresh solve failed after late-stage cancellation");
 end endtask
 always @(negedge clk)if(valid && published_cap!==((limited_l>limited_r?limited_l:limited_r)>255?8'd255:(limited_l>limited_r?limited_l:limited_r)))
  $fatal(1,"published cap disagrees with atomic limited wheels");
 initial begin
  tick(3);rst_n=1;tick(2);run_solve();
  // ceil(600000/7640)=79; ceil(79*.320)=26; ceil(79^2/400)=16.
  if(!valid || stop_mm!=242 || slow_mm!=442 || speed_mm_s!=79 || latency_ms!=320 || hold || limited_l!=12 || solve_cycles!=SOLVE_CYCLES || solve_cycles>512) $fatal(1,"nominal dynamic distance or fixed solve latency");
  // A still-covered publication stays continuously usable while its
  // replacement is solved; neither partial arithmetic nor the new key leaks.
  age_ms=10;@(negedge clk);sample=1;tick(1);@(negedge clk);sample=0;
  for(cycle=0;cycle<SOLVE_CYCLES-1;cycle=cycle+1)begin
   tick(1);
   if(!valid || hold || limited_l!=12 || limited_r!=12 || stop_mm!=242 ||
      slow_mm!=442 || speed_mm_s!=79 || latency_ms!=320 || solve_cycles!=SOLVE_CYCLES)
    $fatal(1,"background solve interrupted or partially replaced published envelope");
  end
  tick(1);
  if(!valid || stop_mm!=243 || slow_mm!=443 || latency_ms!=330 || solve_cycles!=SOLVE_CYCLES)
   $fatal(1,"replacement envelope was not published atomically");
  // Revoke once observed; restoring an old input must not resurrect it.
  front_mm=999;tick(1);front_mm=1000;tick(1);
  if(valid)$fatal(1,"revoked publication resurrected after input restoration");
  // Lower speed remains covered by the old upper bound, but the resulting
  // distance and both scaled wheel targets must still publish together.
  age_ms=0;front_mm=369;velocity_l=20;run_solve();
  if(limited_l!=5 || limited_r!=5 || stop_mm!=285)$fatal(1,"high-bound setup");
  velocity_l=12;@(negedge clk);sample=1;tick(1);@(negedge clk);sample=0;
  for(cycle=0;cycle<SOLVE_CYCLES-1;cycle=cycle+1)begin
   tick(1);if(!valid || limited_l!=5 || limited_r!=5 || stop_mm!=285 || speed_mm_s!=131)
    $fatal(1,"partial reduced-bound result leaked into publication");
  end
  tick(1);if(!valid || limited_l!=7 || limited_r!=7 || stop_mm!=242 || speed_mm_s!=79)
   $fatal(1,"reduced-bound publication not atomic");
  // A pending refresh cannot extend the old publication's age budget.
  front_mm=1000;velocity_l=0;age_ms=0;run_solve();
  age_ms=10;@(negedge clk);sample=1;tick(1);@(negedge clk);sample=0;tick(100);
  age_ms=21;tick(1);if(valid)$fatal(1,"working key extended old age budget");
  age_ms=20;tick(1);if(valid)$fatal(1,"expired old result revived while recomputing");
  tick(SOLVE_CYCLES-102);if(!valid || latency_ms!=330)$fatal(1,"fresh publication after old expiry failed");
  age_ms=0;run_solve();
  velocity_l=13;tick(1);if(valid)$fatal(1,"increased measured speed must revoke old bound");
  velocity_l=0;run_solve();age_ms=20;tick(1);if(!valid)$fatal(1,"control budget must tolerate age progress");
  age_ms=21;tick(1);if(valid)$fatal(1,"age beyond control budget must revoke");
  age_ms=100;run_solve();age_ms=99;tick(1);if(valid)$fatal(1,"new scan age rollback revokes");
  if(invalid_config || invalid_valid)$fatal(1,"invalid braking parameter admitted");
  age_ms=350;run_solve();if(stop_mm!=269 || latency_ms!=670)$fatal(1,"age distance");
  front_mm=269;run_solve();if(!hold || limited_l!=0)$fatal(1,"inclusive stop boundary");
  front_mm=369;run_solve();if(hold || limited_l!=6 || limited_r!=6)$fatal(1,"linear slowdown");
  front_mm=469;run_solve();if(limited_l!=12 || limited_r!=12)$fatal(1,"slow boundary preserves request");
  front_mm=369;
  request_l=1;request_r=2;velocity_l=12;run_solve();if(limited_l>1 || limited_r>2 || stop_mm!=269)$fatal(1,"measured speed and small request");
  follow_mode=1;front_mm=600;run_solve();if(stop_mm!=600 || !hold)$fatal(1,"follow 600 floor");
  @(negedge clk);sample=1;tick(1);@(negedge clk);sample=0;
  for(cycle=0;cycle<SOLVE_CYCLES;cycle=cycle+1)begin
   tick(1);if(!valid || !hold || limited_l!=0 || limited_r!=0)
    $fatal(1,"background solve withdrew an existing stop");
  end
  @(negedge clk);request_l=-2;request_r=-2;tick(1);if(valid || hold || limited_l!=-2)$fatal(1,"reverse must clear stale hold");
  request_l=12;request_r=12;follow_mode=0;velocity_l=-32768;age_ms=65535;front_mm=65535;run_solve();
  if(stop_mm!=65535 || slow_mm!=65535 || speed_mm_s!=65535 || !hold || solve_cycles!=SOLVE_CYCLES)$fatal(1,"overflow must saturate safely");
  enabled=0;tick(1);if(valid || hold || limited_l!=12)$fatal(1,"disable clears immediately");
  enabled=1;front_valid=0;run_solve();if(valid || hold)$fatal(1,"unknown is not a valid envelope");
  front_valid=1;velocity_l=0;velocity_r=0;front_mm=65535;age_ms=65535;run_solve();
  if(stop_mm!=5419 || slow_mm!=5619 || latency_ms!=65535)$fatal(1,"latency display saturation must not truncate arithmetic");
  // New registered stages must retain the same cancellation priority. Work
  // clocks 277..281 cover rounding, distance sum/floor, margin and scale load;
  // clock 442 is immediately before the single publication clock.
  front_mm=1000;age_ms=0;run_solve();
  for(abort_clock=277;abort_clock<=281;abort_clock=abort_clock+1)
   abort_solve_at(abort_clock);
  abort_solve_at(SOLVE_CYCLES-1);
  if(trusted_valid)$fatal(1,"unvalidated registered bound admitted");
  age_ms=0;front_mm=1000;registered_counts=12;registered_counts_valid=1;run_solve();
  if(!trusted_valid || trusted_hold || trusted_l!=12 || trusted_r!=12 || trusted_stop!=242 || trusted_cycles!=443)
   $fatal(1,"validated registered bound disagrees with standalone envelope");
  registered_counts_valid=0;#1;if(trusted_valid)$fatal(1,"validation withdrawal not immediate");
  tick(1);registered_counts_valid=1;tick(1);
  if(trusted_valid)$fatal(1,"validation reenable resurrected old publication");
  velocity_l=-32768;registered_counts=32768;run_solve();
  if(!trusted_valid || !trusted_hold || trusted_stop!=65535 || trusted_l!=0 || trusted_r!=0 || trusted_cycles!=443)
   $fatal(1,"registered signed-min upper bound failed saturation");
  velocity_l=0;registered_counts=12;
  @(negedge clk);sample=1;tick(1);@(negedge clk);sample=0;tick(100);
  if(!trusted_valid || !trusted_hold || trusted_stop!=65535)
   $fatal(1,"lower registered bound discarded conservative published envelope");
  tick(343);if(!trusted_valid || trusted_hold || trusted_stop!=242 || trusted_cycles!=443)
   $fatal(1,"lower registered bound did not publish after 443 clocks");
  age_ms=0;front_mm=65535;request_l=300;request_r=400;registered_counts=400;run_solve();
  if(!valid || published_cap!=255 || limited_r!=400)$fatal(1,"published cap saturation");
  wait(&age_cases_done);
  $display("PASS envelope dynamic distance, linear slowdown, overflow, reversal, missing data; cycles=%0d",solve_cycles);$finish;
 end
 initial begin #250000;$fatal(1,"timeout");end
endmodule







// Boundary and publication-priority coverage for the stored age upper bound.
module envelope_age_case #(parameter B=20)(output reg done=0);
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,sample=0,enabled=1,follow_mode=0,front_valid=1;
 reg[15:0] front_mm=65535,age_ms=0;reg signed[15:0] velocity_l=0,velocity_r=0,request_l=12,request_r=12;
 reg[15:0] registered_counts=12;reg registered_counts_valid=1;
 wire valid,hold,config_ok;wire signed[15:0] limited_l,limited_r;
 wire[15:0] stop_mm,slow_mm,speed_mm_s,latency_ms,solve_cycles;wire[7:0] published_cap;
 rb_lidar_envelope #(.CONTROL_BUDGET_MS(B)) d(.*);
 integer n,base_age,upper;
 task tick(input integer count);repeat(count)begin @(posedge clk);#1;end endtask
 task solve_age;begin @(negedge clk);sample=1;tick(1);@(negedge clk);sample=0;tick(443);end endtask
 initial begin
  tick(2);rst_n=1;
  for(n=0;n<3;n=n+1)begin
   base_age=n==0?0:n==1?65515:65535;age_ms=base_age;solve_age();
   if(!valid || solve_cycles!=443)$fatal(1,"age case did not publish B=%0d base=%0d",B,base_age);
   upper=base_age+B;
   @(negedge clk);age_ms=upper>65535?65535:upper;#1;
   if(!valid)$fatal(1,"inclusive age ceiling overflow B=%0d base=%0d",B,base_age);
   if(upper<65535)begin
    age_ms=upper+1;#1;if(valid)$fatal(1,"age overflow not immediate");
    tick(1);age_ms=base_age;#1;if(valid)$fatal(1,"age bounce revived revoked result");
   end else if(base_age!=0)begin
    age_ms=base_age-1;#1;if(valid)$fatal(1,"age rollback not immediate");
    tick(1);age_ms=base_age;#1;if(valid)$fatal(1,"age rollback bounce revived result");
   end
  end
  age_ms=0;solve_age();
  // Restart at the old publication edge: sample wins and no partial key leaks.
  @(negedge clk);sample=1;tick(1);@(negedge clk);sample=0;tick(442);
  @(negedge clk);age_ms=1;sample=1;tick(1);@(negedge clk);sample=0;
  if(d.published_age!=0)$fatal(1,"sample/publish priority changed");
  tick(443);if(!valid || d.published_age!=1)$fatal(1,"replacement failed");
  rst_n=0;#1;if(valid || published_cap!=0)$fatal(1,"age/cap reset");
  done=1;
 end
endmodule

