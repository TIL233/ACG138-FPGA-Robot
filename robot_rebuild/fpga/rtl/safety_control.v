`timescale 1ns/1ps
// Heading-controlled left90 / right180 scan, then choose the wider direction.
// robot_top supplies independent encoder heading for automatic detours.
// Missing range observations are FAR by user policy; no stale pose samples.
module rb_safety_control #(
 parameter CLK_HZ=50000000, STOP_MM=200, SLOW_MM=400, CLEAR_MM=400,
 parameter SETTLE_MS=200, ADV_COUNTS=3056, SCAN_WAIT_MS=300,
 parameter STEP_TIMEOUT_MS=2500, TURN_TIMEOUT_MS=15000,
 parameter ADVANCE_TIMEOUT_MS=20000, DETOUR_TIMEOUT_MS=90000,
 parameter YAW_SIGN=1, ANGLE_TOLERANCE_CD=150, MAX_YAW_STEP_CD=3000,
 parameter MAX_SETTLED_ERROR_CD=3000, MAX_TURN_CORRECTIONS=4,
 // A measured track is required before automatic scan turns are allowed.
 // Absolute wheel travel remains an independent bound on signed heading.
 parameter TURN_TRACK_MM=0, TURN_COUNTS_PER_M=7640, TURN_TRAVEL_MARGIN_CD=3000,
 parameter FINAL_ANGLE_TOLERANCE_CD=ANGLE_TOLERANCE_CD,
 parameter RELAX_AFTER_CORRECTIONS=MAX_TURN_CORRECTIONS, MIN_PIVOT_COUNTS=2,
 parameter SETTLE_JITTER_COUNTS=0, USE_LIDAR=0
)(
 input clk,rst_n,link_ok,imu_ok,left_ok,right_ok,estop_ok,
 input armed,cmd_update,input [2:0] command,input [7:0] speed,
 input [15:0] dl,dr,input left_update,right_update,
 input signed [31:0] pos_l,pos_r,input motor_fault,
 input signed [15:0] yaw,input yaw_update,
 output reg signed [15:0] target_l,target_r,output reg brake,
 output reg [3:0] state,output reg [15:0] fault,
 output signed [15:0] turn_error,output turn_error_valid,
 input lidar_update,lidar_sides_valid,input [15:0] lidar_left,lidar_right,
 input lidar_front_valid
);
 localparam DRIVE=0,WAIT_STOP=1,TURN=2,WAIT_SCAN=3,ADVANCE=4,
            RETURN=5,WAIT_RETURN=6,FAILED=7;
 localparam SCAN_LEFT=0,SCAN_RIGHT=1,ALIGN_LEFT=2;
 localparam [15:0] FAR=16'hfffe;
 reg [1:0] phase;
 reg [31:0] ms_div,step_ms,total_ms,still_ms,scan_ms;
 wire ms_tick=(ms_div==CLK_HZ/1000-1);
 reg disarm_seen,turn_left,chosen_left,return_finished;
 // Fixed settling anchors, updated only when displacement leaves the band.
 // Checking each tiny step against the previous clock would hide slow creep.
 reg [31:0] prev_l,prev_r;
 // Register the +/-1 neighbours with the fixed anchors, so the production
 // jitter test needs only equality checks, not a live32-bit subtract chain.
 reg [31:0] prev_l_minus,prev_l_plus,prev_r_minus,prev_r_plus;
 reg signed [31:0] adv_l,adv_r;
 reg seen_l,seen_r,pose_seen;
 reg [15:0] sample_l,sample_r,left_clearance,right_clearance,turn_goal;
 reg signed [15:0] origin_yaw,yaw_sample,turn_heading;
 reg correction_active;
 reg [7:0] correction_count;
 reg guard_new_leg;
 reg [1:0] lidar_settled_scans;
 reg [15:0] guard_goal_cd;
 wire turn_guard_fault,turn_guard_configured;
 reg yaw_seen,yaw_bad;
 reg signed [19:0] turn_progress;
 reg turn_done,forward_done;

 function signed [16:0] wrap_angle;
  input signed [17:0] a;
  begin
   if(a>18000)wrap_angle=a-36000;
   else if(a< -18000)wrap_angle=a+36000;
   else wrap_angle=a;
  end
 endfunction
 function [15:0] directed_angle;
  input signed [15:0] target,current;input left;
  reg signed [17:0] delta;
  begin
   delta=left?($signed({{2{target[15]}},target})-$signed({{2{current[15]}},current})):
              ($signed({{2{current[15]}},current})-$signed({{2{target[15]}},target}));
   if(delta<0)delta=delta+36000;
   else if(delta>=36000)delta=delta-36000;
   directed_angle=delta[15:0];
  end
 endfunction
 function angle_in_band;
  input signed [17:0] delta;input integer tolerance;
  begin
   angle_in_band=(delta>=-tolerance && delta<=tolerance) ||
                 delta>=36000-tolerance || delta<=-36000+tolerance;
  end
 endfunction
 wire signed [16:0] signed_yaw=(YAW_SIGN<0)?-$signed({yaw[15],yaw}):$signed({yaw[15],yaw});
 wire yaw_valid=($signed(yaw)>=-18000 && $signed(yaw)<=18000);
 wire signed [16:0] yaw_delta=wrap_angle($signed({signed_yaw[16],signed_yaw})-
                                                      $signed({{2{yaw_sample[15]}},yaw_sample}));
 wire yaw_jump=yaw_seen && (yaw_delta>MAX_YAW_STEP_CD || yaw_delta< -MAX_YAW_STEP_CD);
 wire turning=(state==TURN || state==RETURN);
 // Preserve the budget through braking and all corrections of the same leg.
 // Do not charge the bypass straight or its braking distance to a turn leg.
 wire guard_counting=turning || state==WAIT_SCAN ||
                     (state==WAIT_RETURN && return_finished);
 // The registered maneuver state owns this observer's lifetime. STOP still
 // gates the motor outputs directly and returns the FSM to DRIVE; duplicating
 // the whole live command decoder here creates a long shared enable path.
 wire guard_enabled=(state!=DRIVE && state!=FAILED);
 rb_turn_travel_guard #(.TRACK_MM(TURN_TRACK_MM),.COUNTS_PER_M(TURN_COUNTS_PER_M),
  .MARGIN_CD(TURN_TRAVEL_MARGIN_CD),
  .MAX_GOAL_CD(18000+((FINAL_ANGLE_TOLERANCE_CD>ANGLE_TOLERANCE_CD)?FINAL_ANGLE_TOLERANCE_CD:ANGLE_TOLERANCE_CD))) turn_travel_guard(
  .clk(clk),.rst_n(rst_n),.enable(guard_enabled),.count_enable(guard_counting),
  .new_leg(guard_new_leg),.goal_cd(guard_goal_cd),.pos_l(pos_l),.pos_r(pos_r),
  .fault(turn_guard_fault),.configured(turn_guard_configured));
 wire signed [19:0] directed_step=turn_left?$signed({{3{yaw_delta[16]}},yaw_delta}):
                                                       -$signed({{3{yaw_delta[16]}},yaw_delta});
 wire signed [20:0] turn_threshold=$signed({5'd0,turn_goal})-ANGLE_TOLERANCE_CD;
 wire signed [20:0] remaining_angle=$signed({5'd0,turn_goal})-$signed(turn_progress);
 wire signed [15:0] remaining_limited=remaining_angle>18000?16'sd18000:
                                    remaining_angle<0?16'sd0:remaining_angle[15:0];
 assign turn_error_valid=turning && !turn_done && yaw_seen && !yaw_bad;
 assign turn_error=turn_left?remaining_limited:-remaining_limited;
 wire signed [16:0] left_heading=wrap_angle($signed({{2{origin_yaw[15]}},origin_yaw})+18'sd9000);
 wire signed [16:0] right_heading=wrap_angle($signed({{2{origin_yaw[15]}},origin_yaw})-18'sd9000);
 // A new pose and sonar can arrive on the same edge. Make the terminal
 // decision from that new pose instead of the one-clock older yaw_sample.
 wire pose_input_ok=!yaw_update || (imu_ok && yaw_valid && !yaw_jump);
 // Validity gates the terminal decision below; do not put the yaw-jump
 // subtract/wrap/comparison chain on this datapath's select input as well.
 wire signed [15:0] settled_yaw=yaw_update?signed_yaw[15:0]:yaw_sample;
 // Integrated progress preserves the requested direction of a 180-degree
 // sweep. Its endpoint is only provisional: brake, obtain a fresh settled
 // pose and check the absolute heading before using any range observation.
 wire signed [17:0] fresh_settled_delta=$signed({{2{turn_heading[15]}},turn_heading})-
                                        $signed({{2{signed_yaw[15]}},signed_yaw[15:0]});
 wire signed [17:0] stored_settled_delta=$signed({{2{turn_heading[15]}},turn_heading})-
                                         $signed({{2{yaw_sample[15]}},yaw_sample});
 wire signed [17:0] settled_delta=yaw_update?fresh_settled_delta:stored_settled_delta;
 wire signed [16:0] settled_error=wrap_angle(settled_delta);
 wire [16:0] settled_error_abs=settled_error[16]?-settled_error:settled_error;
 // Compare the three equivalent intervals directly. Keep wrap/abs arithmetic
 // out of the FSM decision path while retaining the identical error value for
 // correction direction and distance, including fresh poses at the +/-180 seam.
 // Calculate both poses in parallel; a same-edge update selects the final
 // one-bit decision, rather than selecting an angle before its arithmetic.
 wire fresh_heading_close=angle_in_band(fresh_settled_delta,ANGLE_TOLERANCE_CD);
 wire stored_heading_close=angle_in_band(stored_settled_delta,ANGLE_TOLERANCE_CD);
 wire fresh_final_close=angle_in_band(fresh_settled_delta,FINAL_ANGLE_TOLERANCE_CD);
 wire stored_final_close=angle_in_band(stored_settled_delta,FINAL_ANGLE_TOLERANCE_CD);
 wire leg_heading_close=yaw_update?fresh_heading_close:stored_heading_close;
 wire leg_final_close=yaw_update?fresh_final_close:stored_final_close;
 wire may_relax=(correction_count>=RELAX_AFTER_CORRECTIONS);
 wire leg_heading_ok=yaw_update?(fresh_heading_close || (may_relax && fresh_final_close)):
                                (stored_heading_close || (may_relax && stored_final_close));
 wire fresh_error_large=!angle_in_band(fresh_settled_delta,MAX_SETTLED_ERROR_CD);
 wire stored_error_large=!angle_in_band(stored_settled_delta,MAX_SETTLED_ERROR_CD);
 wire leg_error_large=yaw_update?fresh_error_large:stored_error_large;
 wire [15:0] return_goal=directed_angle(origin_yaw,settled_yaw,!chosen_left);
 wire signed [31:0] forward_l=$signed(pos_l)-$signed(adv_l);
 wire signed [31:0] forward_r=$signed(pos_r)-$signed(adv_r);
 wire signed [33:0] forward_sum=$signed({{2{forward_l[31]}},forward_l})+
                               $signed({{2{forward_r[31]}},forward_r});
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin yaw_sample<=0;yaw_seen<=0;yaw_bad<=0;turn_progress<=0;turn_done<=0;forward_done<=0;end
  else begin
   yaw_bad<=0;
   if(!imu_ok)yaw_seen<=0;
   else if(yaw_update)begin
    if(!yaw_valid || yaw_jump)begin yaw_seen<=0;yaw_bad<=1;end
    else begin yaw_sample<=signed_yaw[15:0];yaw_seen<=1;end
   end
   if(!turning)turn_progress<=0;
   else if(yaw_update && imu_ok && yaw_valid && yaw_seen && !yaw_jump)
    turn_progress<=turn_progress+directed_step;
   turn_done<=turning && yaw_seen && turn_progress>=turn_threshold;
   forward_done<=(state==ADVANCE && forward_sum>=2*ADV_COUNTS);
  end

 wire healthy=link_ok && imu_ok && estop_ok && !yaw_bad;
 wire [15:0] current_l=left_ok?dl:FAR,current_r=right_ok?dr:FAR;
 wire near=(current_l<STOP_MM || current_r<STOP_MM);
 wire signed [31:0] settle_delta_l=$signed(pos_l)-$signed(prev_l);
 wire signed [31:0] settle_delta_r=$signed(pos_r)-$signed(prev_r);
 wire stationary;
 generate
  if(SETTLE_JITTER_COUNTS==0)begin:strict_settling
   assign stationary=(prev_l==pos_l && prev_r==pos_r);
  end else if(SETTLE_JITTER_COUNTS==1)begin:one_count_settling
   assign stationary=(pos_l==prev_l || pos_l==prev_l_minus || pos_l==prev_l_plus) &&
                     (pos_r==prev_r || pos_r==prev_r_minus || pos_r==prev_r_plus);
  end else begin:general_settling
   assign stationary=(settle_delta_l>=-SETTLE_JITTER_COUNTS && settle_delta_l<=SETTLE_JITTER_COUNTS &&
                      settle_delta_r>=-SETTLE_JITTER_COUNTS && settle_delta_r<=SETTLE_JITTER_COUNTS);
  end
 endgenerate
 wire settled=(still_ms>=SETTLE_MS);
 wire settled_pose=stationary && settled && pose_seen && yaw_seen && pose_input_ok;
 wire sampling=(state==WAIT_SCAN || (state==WAIT_RETURN && return_finished)) &&
                leg_heading_ok && settled_pose;
 wire fresh_l=left_update && left_ok && dl>=20 && dl!=16'hffff;
 wire fresh_r=right_update && right_ok && dr>=20 && dr!=16'hffff;
 // Count a frame on the deadline, and discard samples that became invalid.
 wire [15:0] observed_l=!left_ok?FAR:fresh_l?dl:seen_l?sample_l:FAR;
 wire [15:0] observed_r=!right_ok?FAR:fresh_r?dr:seen_r?sample_r:FAR;
 wire [15:0] observed_distance=(observed_l<observed_r)?observed_l:observed_r;
 // Keep the minimum as stored evidence, but compare its consumers in parallel:
 // min(a,b)<c iff a<c || b<c; c>=min(a,b) iff c>=a || c>=b.
 wire observed_below_clear=(observed_l<CLEAR_MM || observed_r<CLEAR_MM);
 wire left_at_least_observed=(left_clearance>=observed_l || left_clearance>=observed_r);
 wire observed_clear=(observed_l>=CLEAR_MM && observed_r>=CLEAR_MM);
 wire observed_resume_clear=(observed_l>STOP_MM+100 && observed_r>STOP_MM+100);
 wire scan_ready=settled_pose && leg_heading_ok &&
                 (((seen_l || fresh_l) && (seen_r || fresh_r)) || scan_ms>=SCAN_WAIT_MS);
 wire signed [15:0] requested=$signed({8'd0,speed});
 wire signed [15:0] drive_speed=(current_l<SLOW_MM || current_r<SLOW_MM)?(requested>>>1):requested;
 wire signed [15:0] detour_speed=(speed<8)?requested:16'sd8;
 wire signed [15:0] pivot_speed=((correction_active || remaining_angle<1000) && detour_speed>MIN_PIVOT_COUNTS)?MIN_PIVOT_COUNTS:
                              (remaining_angle<3000 && detour_speed>4)?16'sd4:detour_speed;

 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=DRIVE;fault<=0;disarm_seen<=0;ms_div<=0;step_ms<=0;total_ms<=0;still_ms<=0;scan_ms<=0;
   prev_l<=0;prev_r<=0;adv_l<=0;adv_r<=0;seen_l<=0;seen_r<=0;pose_seen<=0;
   prev_l_minus<=32'hffffffff;prev_l_plus<=1;prev_r_minus<=32'hffffffff;prev_r_plus<=1;
   sample_l<=FAR;sample_r<=FAR;left_clearance<=FAR;right_clearance<=FAR;
   origin_yaw<=0;turn_heading<=0;turn_goal<=0;turn_left<=1;chosen_left<=1;phase<=SCAN_LEFT;return_finished<=0;
   correction_active<=0;correction_count<=0;
   guard_new_leg<=0;guard_goal_cd<=0;lidar_settled_scans<=0;
  end else begin
   guard_new_leg<=0;
   // Require two complete publications after settling: the first revolution
   // may still contain measurements gathered while the chassis was braking.
   if((state!=WAIT_STOP && state!=WAIT_SCAN && state!=WAIT_RETURN) || !settled_pose)lidar_settled_scans<=0;
   else if(lidar_update && lidar_settled_scans<2)lidar_settled_scans<=lidar_settled_scans+1'b1;
   if(ms_tick)ms_div<=0;else ms_div<=ms_div+1'b1;
   if(!stationary)begin
    prev_l<=pos_l;prev_r<=pos_r;still_ms<=0;pose_seen<=0;
    prev_l_minus<=pos_l-32'd1;prev_l_plus<=pos_l+32'd1;
    prev_r_minus<=pos_r-32'd1;prev_r_plus<=pos_r+32'd1;
   end
   else begin
    if(ms_tick && still_ms<SETTLE_MS)still_ms<=still_ms+1'b1;
    if(settled && yaw_update && yaw_valid && !yaw_jump)pose_seen<=1;
   end
   if(!sampling || !stationary || !settled)begin
    scan_ms<=0;seen_l<=0;seen_r<=0;sample_l<=FAR;sample_r<=FAR;
   end else begin
    if(ms_tick && scan_ms<SCAN_WAIT_MS)scan_ms<=scan_ms+1'b1;
    if(!left_ok)begin seen_l<=0;sample_l<=FAR;end
    else if(fresh_l)begin seen_l<=1;sample_l<=dl;end
    if(!right_ok)begin seen_r<=0;sample_r<=FAR;end
    else if(fresh_r)begin seen_r<=1;sample_r<=dr;end
   end
   if(ms_tick)begin
    if(step_ms<DETOUR_TIMEOUT_MS+1)step_ms<=step_ms+1'b1;
    if(state!=DRIVE && state!=FAILED && total_ms<DETOUR_TIMEOUT_MS+1)total_ms<=total_ms+1'b1;
   end
   if(cmd_update && !armed)begin
    disarm_seen<=1;fault<=0;state<=DRIVE;step_ms<=0;total_ms<=0;
   end else if(armed && (!healthy || motor_fault))begin
    disarm_seen<=0;state<=FAILED;
    fault<={10'd0,motor_fault,!estop_ok,2'b00,(!imu_ok || yaw_bad),!link_ok};
   end else if(armed && turn_guard_fault && state!=DRIVE && state!=FAILED)begin
    state<=FAILED;fault<=16'h0400;disarm_seen<=0;
   end else if(!armed || command==0 || speed==0)begin
    state<=DRIVE;step_ms<=0;total_ms<=0;
   end else if(fault!=0 || !disarm_seen)state<=FAILED;
   else if(state!=DRIVE && state!=FAILED && total_ms>=DETOUR_TIMEOUT_MS)begin
    state<=FAILED;fault<=16'h0100;disarm_seen<=0;
   end else if(command!=1)begin
    state<=DRIVE;step_ms<=0;total_ms<=0;
   end else case(state)
    DRIVE:begin
     total_ms<=0;step_ms<=0;
     if(near)begin state<=WAIT_STOP;still_ms<=0;pose_seen<=0;end
    end
    WAIT_STOP:begin
     if(settled_pose && !turn_guard_configured)begin
      state<=FAILED;fault<=16'h0400;disarm_seen<=0;
     end else if(settled_pose && (!(USE_LIDAR && lidar_sides_valid) ||
                                 (lidar_update && lidar_settled_scans>=1)))begin
      origin_yaw<=settled_yaw;turn_goal<=9000;
      if(USE_LIDAR && lidar_sides_valid)begin
       // A stationary 360deg scan replaces the physical left/right survey.
       // The selected turn, settling, front recheck and return still use the
       // existing encoder controller, STOP and shared travel budget.
       phase<=ALIGN_LEFT;chosen_left<=lidar_left>=lidar_right;turn_left<=lidar_left>=lidar_right;
       turn_heading<=wrap_angle($signed({{2{settled_yaw[15]}},settled_yaw})+
                                ((lidar_left>=lidar_right)?18'sd9000:-18'sd9000));
      end else begin
       phase<=SCAN_LEFT;turn_left<=1;
       turn_heading<=wrap_angle($signed({{2{settled_yaw[15]}},settled_yaw})+18'sd9000);
      end
      correction_active<=0;correction_count<=0;
      guard_new_leg<=1;guard_goal_cd<=9000;
      step_ms<=0;state<=TURN;
      if(USE_LIDAR && lidar_sides_valid && lidar_left<CLEAR_MM && lidar_right<CLEAR_MM)begin
       state<=FAILED;fault<=16'h0800;disarm_seen<=0;guard_new_leg<=0;
      end
     end else if(step_ms>=STEP_TIMEOUT_MS)begin state<=FAILED;fault<=16'h0200;disarm_seen<=0;end
    end
    TURN:begin
     if(turn_done)begin state<=WAIT_SCAN;step_ms<=0;still_ms<=0;pose_seen<=0;end
     else if(step_ms>=TURN_TIMEOUT_MS || turn_progress< -500)begin state<=FAILED;fault<=16'h0400;disarm_seen<=0;end
    end
    WAIT_SCAN:begin
     if(settled_pose && !leg_heading_ok)begin
      // A large discrepancy is not an ordinary braking correction. Do not
      // silently label (for example) 135 degrees as the left90 scan pose.
      if(leg_error_large || correction_count>=MAX_TURN_CORRECTIONS)begin
       state<=FAILED;fault<=16'h0400;disarm_seen<=0;
      end else begin
       turn_left<=!settled_error[16];turn_goal<=settled_error_abs[15:0];
       correction_active<=1;correction_count<=correction_count+1'b1;
       step_ms<=0;still_ms<=0;pose_seen<=0;state<=TURN;
      end
     end else if(scan_ready && (!(USE_LIDAR && lidar_front_valid) || lidar_settled_scans>=2))begin
      if(phase==SCAN_LEFT)begin
       left_clearance<=observed_distance;phase<=SCAN_RIGHT;turn_left<=0;
       turn_goal<=directed_angle(right_heading[15:0],settled_yaw,1'b0);
       guard_new_leg<=1;guard_goal_cd<=directed_angle(right_heading[15:0],settled_yaw,1'b0);
       turn_heading<=right_heading[15:0];correction_active<=0;correction_count<=0;
       step_ms<=0;state<=TURN;
      end else if(phase==SCAN_RIGHT)begin
       right_clearance<=observed_distance;
       if(left_clearance<CLEAR_MM && observed_below_clear)begin
        state<=FAILED;fault<=16'h0800;disarm_seen<=0;
       end else if(left_at_least_observed)begin
        chosen_left<=1;turn_left<=1;phase<=ALIGN_LEFT;
        turn_goal<=directed_angle(left_heading[15:0],settled_yaw,1'b1);
        guard_new_leg<=1;guard_goal_cd<=directed_angle(left_heading[15:0],settled_yaw,1'b1);
        turn_heading<=left_heading[15:0];correction_active<=0;correction_count<=0;
        step_ms<=0;state<=TURN;
       end else begin
        chosen_left<=0;adv_l<=pos_l;adv_r<=pos_r;step_ms<=0;state<=ADVANCE;
       end
      end else if(observed_clear)begin
       adv_l<=pos_l;adv_r<=pos_r;step_ms<=0;state<=ADVANCE;
      end else begin state<=FAILED;fault<=16'h1000;disarm_seen<=0;end
     end else if(step_ms>=STEP_TIMEOUT_MS)begin state<=FAILED;fault<=16'h0200;disarm_seen<=0;end
    end
    ADVANCE:begin
     if(near)begin state<=FAILED;fault<=16'h1000;disarm_seen<=0;end
     else if(forward_done)begin
      state<=WAIT_RETURN;return_finished<=0;step_ms<=0;still_ms<=0;pose_seen<=0;
     end else if(step_ms>=ADVANCE_TIMEOUT_MS)begin state<=FAILED;fault<=16'h1000;disarm_seen<=0;end
    end
    WAIT_RETURN:begin
     if(!return_finished && settled_pose)begin
      if(return_goal>18000)begin state<=FAILED;fault<=16'h0400;disarm_seen<=0;end
      else begin
       turn_left<=!chosen_left;turn_goal<=return_goal;turn_heading<=origin_yaw;
       guard_new_leg<=1;guard_goal_cd<=return_goal;
       correction_active<=0;correction_count<=0;step_ms<=0;state<=RETURN;
      end
     end else if(return_finished && settled_pose && !leg_heading_ok)begin
      if(leg_error_large || correction_count>=MAX_TURN_CORRECTIONS)begin
       state<=FAILED;fault<=16'h0400;disarm_seen<=0;
      end else begin
       turn_left<=!settled_error[16];turn_goal<=settled_error_abs[15:0];
       correction_active<=1;correction_count<=correction_count+1'b1;
       step_ms<=0;still_ms<=0;pose_seen<=0;state<=RETURN;
      end
     end else if(return_finished && scan_ready && (!(USE_LIDAR && lidar_front_valid) || lidar_settled_scans>=2))begin
      if(observed_resume_clear)begin state<=DRIVE;step_ms<=0;total_ms<=0;end
      else begin state<=FAILED;fault<=16'h1000;disarm_seen<=0;end
     end else if(step_ms>=STEP_TIMEOUT_MS)begin state<=FAILED;fault<=16'h0200;disarm_seen<=0;end
    end
    RETURN:begin
     if(turn_done)begin
      state<=WAIT_RETURN;return_finished<=1;step_ms<=0;still_ms<=0;pose_seen<=0;
     end else if(step_ms>=TURN_TIMEOUT_MS || turn_progress< -500)begin state<=FAILED;fault<=16'h0400;disarm_seen<=0;end
    end
    default:state<=FAILED;
   endcase
  end
 end
 always @* begin
  target_l=0;target_r=0;brake=1;
  if(rst_n && healthy && armed && disarm_seen && fault==0 && !motor_fault &&
     !turn_guard_fault && command!=0 && speed!=0)begin
   case(state)
    DRIVE:case(command)
     1:if(!near)begin target_l=drive_speed;target_r=drive_speed;end
     2:begin target_l=-requested;target_r=-requested;end
     3:if(!near)begin target_l=drive_speed>>>1;target_r=drive_speed;end
     4:if(!near)begin target_l=drive_speed;target_r=drive_speed>>>1;end
     5:begin target_l=-requested;target_r=requested;end
     6:begin target_l=requested;target_r=-requested;end
    endcase
    TURN,RETURN:if(command==1 && !turn_done)begin
     target_l=turn_left?-pivot_speed:pivot_speed;target_r=-target_l;
    end
    ADVANCE:if(command==1 && !near)begin
     target_l=(current_l<SLOW_MM || current_r<SLOW_MM)?(detour_speed>>>1):detour_speed;target_r=target_l;
    end
   endcase
   brake=(target_l==0 && target_r==0);
  end
 end
endmodule
