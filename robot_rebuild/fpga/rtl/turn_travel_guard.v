`timescale 1ns/1ps
// Independent wheel-travel budget for one automatic turn, including corrections.
// enable owns the fault lifecycle; count_enable excludes straight detour travel.
// The geometric budget is a limit, not a claim of calibrated ground angle.
module rb_turn_travel_guard #(
 parameter COUNTS_PER_M=7640, TRACK_MM=0, MARGIN_CD=3000,
 parameter MAX_DELTA_COUNTS=512, MAX_GOAL_CD=18150
)(
 input clk,rst_n,enable,count_enable,new_leg,
 input [15:0] goal_cd,
 input signed [31:0] pos_l,pos_r,
 output reg fault,
 output configured
);
 // Keep configuration arithmetic within 64 bits and reject unmeasured geometry.
 assign configured=(COUNTS_PER_M>=100 && COUNTS_PER_M<=1000000 &&
                    TRACK_MM>=50 && TRACK_MM<=1000 &&
                    MARGIN_CD>=0 && MARGIN_CD<=18000 &&
                    MAX_GOAL_CD>=0 && MAX_GOAL_CD<=20000 &&
                    MAX_DELTA_COUNTS>=1 && MAX_DELTA_COUNTS<=512);
 // pi rounded upward in Q24; round the coefficient upward as well, so the
 // fixed-point calculation cannot make the travel budget smaller than exact pi.
 localparam [63:0] COUNTS_PER_CD_Q24=(COUNTS_PER_M>=100 && COUNTS_PER_M<=1000000 &&
                                   TRACK_MM>=50 && TRACK_MM<=1000)?
   ((64'd1*COUNTS_PER_M*TRACK_MM*64'd52707179+64'd17999999)/64'd18000000):64'd0;
 reg signed [31:0] prev_l,prev_r,delta_l_stage,delta_r_stage;
 wire signed [31:0] delta_l=pos_l-prev_l,delta_r=pos_r-prev_r;
 wire bad_delta=(delta_l>MAX_DELTA_COUNTS || delta_l< -MAX_DELTA_COUNTS ||
                 delta_r>MAX_DELTA_COUNTS || delta_r< -MAX_DELTA_COUNTS);
 wire [32:0] abs_l=delta_l_stage[31]?-$signed({delta_l_stage[31],delta_l_stage}):
                                                       {1'b0,delta_l_stage};
 wire [32:0] abs_r=delta_r_stage[31]?-$signed({delta_r_stage[31],delta_r_stage}):
                                                       {1'b0,delta_r_stage};
 reg delta_valid,pair_valid,leg_active,budget_ready;
 reg [10:0] pair_counts;
 reg [31:0] travel_counts,budget_counts;
 wire [32:0] next_travel={1'b0,travel_counts}+{22'd0,pair_counts};
 reg [1:0] budget_phase;
 reg [16:0] budget_angle;
 reg [63:0] budget_product;

 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   prev_l<=0;prev_r<=0;delta_l_stage<=0;delta_r_stage<=0;
   delta_valid<=0;pair_valid<=0;pair_counts<=0;leg_active<=0;
   travel_counts<=0;budget_counts<=0;budget_ready<=0;
   budget_phase<=0;budget_angle<=0;budget_product<=0;fault<=0;
  end else begin
   // Even when counting is paused, track positions so straight travel is not
   // reintroduced as one large jump when the next turn starts.
   prev_l<=pos_l;prev_r<=pos_r;
   if(!enable)begin
    fault<=0;leg_active<=0;delta_valid<=0;pair_valid<=0;
    travel_counts<=0;budget_counts<=0;budget_ready<=0;budget_phase<=0;
   end else if(fault)begin
    // Starting another leg must never clear a fault in the same maneuver.
    delta_valid<=0;pair_valid<=0;
   end else if(new_leg)begin
    travel_counts<=0;delta_valid<=0;pair_valid<=0;pair_counts<=0;
    budget_ready<=0;budget_phase<=0;leg_active<=0;
    if(!configured || goal_cd>MAX_GOAL_CD)fault<=1;
    else begin
     leg_active<=1;budget_angle<={1'b0,goal_cd}+MARGIN_CD;budget_phase<=1;
    end
   end else begin
    // Bad encoder steps fail at the first sampling edge, without waiting for
    // the three-stage travel path. A rejected step never enters that path.
    if(bad_delta)fault<=1;
    delta_l_stage<=delta_l;delta_r_stage<=delta_r;
    delta_valid<=count_enable && leg_active && !bad_delta;
    pair_counts<=abs_l+abs_r;
    pair_valid<=delta_valid;
    if(pair_valid)begin
     if(next_travel[32])begin travel_counts<=32'hffffffff;fault<=1;end
     else begin
      travel_counts<=next_travel[31:0];
      if(budget_ready && next_travel>{1'b0,budget_counts})fault<=1;
     end
    end
    // Only constant division is used above. A live goal uses one registered
    // constant multiply followed by a shift/round stage, independent of travel.
    if(budget_phase==1)begin
     budget_product<=budget_angle*COUNTS_PER_CD_Q24;budget_phase<=2;
    end else if(budget_phase==2)begin
     budget_counts<=(budget_product+64'd16777215)>>24;
     budget_ready<=1;budget_phase<=0;
    end
   end
  end
 end
endmodule
