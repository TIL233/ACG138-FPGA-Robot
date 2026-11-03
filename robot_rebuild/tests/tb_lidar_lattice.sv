`timescale 1ns/1ps
// Catches unknown-space admission, stale commands, sign reversal and unsafe speed transitions.
module tb_lidar_lattice;
 // Three groups of 36 parallel sector issues, one drain and 3 five-clock scores,
 // followed by one atomic result publication.
 localparam SOLVE_CYCLES=157;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,start=0,cancel=0,map_fresh=1;
 reg [575:0] map_ranges;reg [35:0] map_known={36{1'b1}};
 reg [15:0] map_scan_id=1,map_age_ms=0,measured_max_counts=0,command_epoch=1;
 reg [7:0] request_max=12;reg signed [15:0] goal_cd=0;reg [3:0] previous_candidate=15;
 wire busy,result_valid,no_path,fault;wire [3:0] selected;
 wire signed [15:0] target_l,target_r;wire [8:0] admissible_mask,unknown_mask;
 wire [15:0] clearance_mm,solve_cycles,result_scan_id,result_epoch;
 rb_lidar_lattice dut(.*);
 wire invalid_fault,invalid_valid;
 rb_lidar_lattice #(.ARC_RADIUS_MM(151)) bad_config(
  .clk(clk),.rst_n(rst_n),.start(start),.cancel(cancel),.map_fresh(map_fresh),
  .map_ranges(map_ranges),.map_known(map_known),.map_scan_id(map_scan_id),.map_age_ms(map_age_ms),
  .request_max(request_max),.measured_max_counts(measured_max_counts),.goal_cd(goal_cd),
  .command_epoch(command_epoch),.previous_candidate(previous_candidate),
  .fault(invalid_fault),.result_valid(invalid_valid),.busy(),.no_path(),.selected(),
  .target_l(),.target_r(),.admissible_mask(),.unknown_mask(),.clearance_mm(),
  .solve_cycles(),.result_scan_id(),.result_epoch());
 integer i,abort_clock;
 task tick(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task run_solve;begin @(negedge clk);start=1;tick(1);@(negedge clk);start=0;tick(SOLVE_CYCLES);end endtask
 task cancel_score_at(input integer n);begin
  goal_cd=11459;@(negedge clk);start=1;tick(1);@(negedge clk);start=0;tick(n);
  if(!busy || result_valid || selected!=15 || target_l!=0 || target_r!=0)
   $fatal(1,"partial scoring result escaped before publication");
  cancel=1;tick(1);
  if(busy || result_valid || no_path || selected!=15 || target_l!=0 || target_r!=0 ||
     admissible_mask!=0 || unknown_mask!=0)
   $fatal(1,"score-stage cancel did not clear result and masks");
  cancel=0;tick(SOLVE_CYCLES);
  if(busy || result_valid || no_path || selected!=15 || target_l!=0 || target_r!=0)
   $fatal(1,"cancelled score published later");
  goal_cd=-11459;command_epoch=command_epoch+1'b1;run_solve();
  if(!result_valid || selected!=8 || target_l!=12 || target_r!=6 ||
     result_epoch!=command_epoch || result_scan_id!=map_scan_id || solve_cycles!=SOLVE_CYCLES)
   $fatal(1,"fresh score or epoch mixed with cancelled partial score");
 end endtask
 initial begin
  for(i=0;i<36;i=i+1)map_ranges[i*16+:16]=2000;
  tick(3);rst_n=1;tick(2);
  @(negedge clk);start=1;tick(1);@(negedge clk);start=0;
  for(i=0;i<SOLVE_CYCLES-1;i=i+1)begin
   tick(1);if(result_valid || selected!=15 || target_l!=0 || target_r!=0 || result_epoch!=0 || result_scan_id!=0)
    $fatal(1,"candidate targets or tags published before all groups were scored");
  end
  tick(1);
  if(!result_valid || selected!=7 || target_l!=12 || target_r!=12 || admissible_mask!=9'h1ff || solve_cycles!=SOLVE_CYCLES || solve_cycles>512 || result_scan_id!=1 || result_epoch!=1)$fatal(1,"open corridor selects straight fast with fixed latency");
  if(!invalid_fault || invalid_valid)$fatal(1,"geometry mismatch admitted");
  // For speed 12: ceil(v)=79, inflation=160+53+16=229; straight front=300+229+2=531.
  map_ranges[0+:16]=531;run_solve();if(!admissible_mask[7] || selected!=7 || clearance_mm!=0)$fatal(1,"inclusive conservative threshold");
  map_ranges[0+:16]=530;run_solve();if(admissible_mask[7] || selected!=4)$fatal(1,"one mm below rejects without unsigned clearance wrap");
  map_ranges[0+:16]=2000;
  // The last sector is consumed on the drain clock, with the same inclusive
  // threshold, borrow handling and unknown rejection as the first sector.
  map_ranges[35*16+:16]=531;run_solve();if(!admissible_mask[7] || selected!=7 || clearance_mm!=0)$fatal(1,"last-sector inclusive threshold lost in drain");
  map_ranges[35*16+:16]=530;run_solve();if(admissible_mask[7] || selected!=4)$fatal(1,"last-sector borrow lost in drain");
  map_known[35]=0;run_solve();if(result_valid || !no_path || unknown_mask!=9'h1ff)$fatal(1,"last-sector unknown lost in drain");
  map_known[35]=1;map_ranges[35*16+:16]=2000;
  map_ranges[5*16+:16]=350;run_solve();if(admissible_mask[8] || !admissible_mask[6])$fatal(1,"clockwise sector is right obstruction");
  map_ranges[5*16+:16]=2000;map_ranges[30*16+:16]=350;run_solve();
  if(admissible_mask[6] || !admissible_mask[8])$fatal(1,"counterclockwise sector is left obstruction");
  map_ranges[30*16+:16]=2000;goal_cd=18000;run_solve();if(selected!=6)$fatal(1,"wrapped angle cost tie chooses low id");
  goal_cd=11459;run_solve();if(selected!=6 || target_l!=6 || target_r!=12)$fatal(1,"positive goal selects left");
  goal_cd=-11459;run_solve();if(selected!=8 || target_l!=12 || target_r!=6)$fatal(1,"negative goal selects right");
  goal_cd=0;request_max=6;run_solve();if(selected!=4 || admissible_mask!=9'h03f)$fatal(1,"request speed cap");
  measured_max_counts=7;run_solve();if(result_valid || !no_path || selected!=15 || target_l!=0 || admissible_mask!=0)$fatal(1,"cannot assume already slowed");
  request_max=12;measured_max_counts=12;run_solve();if(admissible_mask!=9'h1c0 || selected!=7)$fatal(1,"measured speed boundary");
  measured_max_counts=13;run_solve();if(result_valid || !no_path)$fatal(1,"all nominal speeds below measured");
  measured_max_counts=0;request_max=12;map_known=0;run_solve();if(result_valid || !no_path || unknown_mask!=9'h1ff)$fatal(1,"unknown map");
  map_known={36{1'b1}};for(i=0;i<36;i=i+1)map_ranges[i*16+:16]=120;
  run_solve();if(result_valid || !no_path || selected!=15 || target_r!=0)$fatal(1,"blocked map");
  for(i=0;i<36;i=i+1)map_ranges[i*16+:16]=2000;
  map_age_ms=350;run_solve();if(!result_valid)$fatal(1,"age 350 inclusive");
  map_age_ms=351;tick(1);if(result_valid || busy)$fatal(1,"age expiration revokes");
  run_solve();if(result_valid)$fatal(1,"stale map admitted");
  map_age_ms=0;run_solve();command_epoch=2;tick(1);if(result_valid)$fatal(1,"epoch revoke");
  @(negedge clk);start=1;tick(1);@(negedge clk);start=0;tick(10);cancel=1;tick(1);if(busy || result_valid)$fatal(1,"cancel in flight");
  cancel=0;run_solve();map_scan_id=2;tick(1);if(result_valid)$fatal(1,"scan revoke");
  run_solve();request_max=6;tick(1);if(result_valid)$fatal(1,"request revoke");
  run_solve();goal_cd=100;tick(1);if(result_valid)$fatal(1,"goal revoke");
  run_solve();map_fresh=0;tick(1);if(result_valid || busy || target_l!=0)$fatal(1,"map loss revokes");
  // Each group's last issue and drain must be cancellable while a processed
  // sector is pending, without a late accumulator or result publication.
  map_fresh=1;goal_cd=0;request_max=12;
  for(abort_clock=35;abort_clock<=37;abort_clock=abort_clock+1)cancel_score_at(abort_clock);
  for(abort_clock=87;abort_clock<=89;abort_clock=abort_clock+1)cancel_score_at(abort_clock);
  for(abort_clock=139;abort_clock<=141;abort_clock=abort_clock+1)cancel_score_at(abort_clock);
  // Last group's first and last candidates cover all five score stages,
  // including the final best update immediately before atomic publication.
  for(abort_clock=142;abort_clock<=146;abort_clock=abort_clock+1)cancel_score_at(abort_clock);
  for(abort_clock=152;abort_clock<=156;abort_clock=abort_clock+1)cancel_score_at(abort_clock);
  // Stale command tags retain cancellation priority on the publication edge.
  goal_cd=0;@(negedge clk);start=1;tick(1);@(negedge clk);start=0;tick(SOLVE_CYCLES-1);
  command_epoch=command_epoch+1'b1;tick(1);
  if(busy || result_valid || no_path || selected!=15 || target_l!=0 || target_r!=0)
   $fatal(1,"new epoch admitted the previous solve on its publication edge");
  run_solve();if(!result_valid || selected!=7 || result_epoch!=command_epoch)$fatal(1,"new epoch did not recover");
  $display("PASS lattice 3 lanes, 9 candidates, goals, request/measured constraints, unknown, stale/cancel; cycles=%0d",solve_cycles);$finish;
 end
 initial begin #200000;$fatal(1,"timeout");end
endmodule

