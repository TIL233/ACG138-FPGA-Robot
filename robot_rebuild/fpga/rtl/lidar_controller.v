`timescale 1ns/1ps
// Optional actuator arbitration around a nominal geometric local planner.
// Observation never grants ARM or changes the original safety controller.
module rb_lidar_controller #(
 parameter CLK_HZ=50000000, COUNTS_PER_M=7640, WHEELBASE_MM=100,
 parameter BODY_RADIUS_MM=120, DECEL_MM_S2=200, INPUT_BUDGET_MS=300
)(
 input clk,rst_n,enabled,control_enabled,original_control_enabled,model_validated,follow_mode,nav_mode,
 input armed,stop_now,healthy,input [2:0] request_cmd,input [7:0] request_speed,
 input [3:0] safety_state,input feedback_update,
 input signed [15:0] velocity_l,velocity_r,goal_cd,input goal_valid,
 input sonar_l_ok,sonar_r_ok,input [15:0] sonar_l,sonar_r,
 input laser_front_ok,input [15:0] laser_front,
 input map_update,map_fresh,input [575:0] map_ranges,input [35:0] map_known,
 input [15:0] map_scan_id,map_age_ms,
 output scope,takeover,hold,follow_limit_valid,dynamic_gate,dynamic_hold,
 output signed [15:0] target_l,target_r,follow_l,follow_r,
 output current,output [3:0] selected,output reg [319:0] diagnostic
);
 localparam GEOMETRY_OK=COUNTS_PER_M==7640 && WHEELBASE_MM==100 &&
  BODY_RADIUS_MM==120 && DECEL_MM_S2==200 && INPUT_BUDGET_MS==300;
 wire validated=model_validated && GEOMETRY_OK;
 wire control_ready=enabled && control_enabled && original_control_enabled && validated;
 wire forward_cmd=request_cmd==1 || request_cmd==3 || request_cmd==4;
 wire route_request=request_cmd==1 || (nav_mode && (request_cmd==3 || request_cmd==4));
 assign scope=control_ready && armed && !stop_now && healthy && !follow_mode &&
              route_request && request_speed!=0 && safety_state==0;
 // Unarmed observation can show a suggestion, with no actuator permission.
 wire [7:0] observed_speed=armed?request_speed:8'd12;
 wire [2:0] observed_cmd=armed?request_cmd:3'd1;
 wire observed_forward=observed_cmd==1 || observed_cmd==3 || observed_cmd==4;
 wire signed [15:0] raw_l=observed_cmd==3?{8'd0,observed_speed}>>1:
                          observed_forward?{8'd0,observed_speed}:16'd0;
 wire signed [15:0] raw_r=observed_cmd==4?{8'd0,observed_speed}>>1:
                          observed_forward?{8'd0,observed_speed}:16'd0;
 function [15:0] abs_count;
  input signed [15:0] value;
  begin abs_count=value[15]?(~value+16'd1):value;end
 endfunction
 wire [15:0] measured_l=abs_count(velocity_l),measured_r=abs_count(velocity_r);
 wire [15:0] measured_live=measured_l>measured_r?measured_l:measured_r;
 wire [15:0] requested_live=raw_l>raw_r?raw_l:raw_r;
 wire [15:0] combined_bound_live=measured_live>requested_live?measured_live:requested_live;
 wire [15:0] sonar_min_l=sonar_l_ok?sonar_l:16'hffff;
 wire [15:0] sonar_min_r=sonar_r_ok?sonar_r:16'hffff;
 wire [15:0] sonar_min=sonar_min_l<sonar_min_r?sonar_min_l:sonar_min_r;
 wire laser_used=laser_front_ok && laser_front<=sonar_min;
 wire [15:0] front_live=laser_used?laser_front:sonar_min;
 wire front_valid_live=laser_front_ok || sonar_l_ok || sonar_r_ok;
 wire [15:0] front_age_live=laser_used?map_age_ms:16'd0;
 wire [66:0] range_raw={sonar_l_ok,sonar_r_ok,sonar_l,sonar_r,laser_front_ok,laser_front,map_age_ms};
 reg [66:0] range_capture,range_key;
 reg [15:0] front,front_age;
 reg front_valid,envelope_sample;
 reg [15:0] front_capture,age_capture,measured_capture,bound_capture;
 reg front_valid_capture,sample_capture,follow_capture,follow_key,bounds_ready;
 reg signed [15:0] velocity_l_capture,velocity_r_capture,request_l_capture,request_r_capture;
 reg signed [15:0] velocity_l_key,velocity_r_key,request_l_key,request_r_key;
 reg [15:0] measured,registered_bound;
 // Validate the derived candidate against the captured raw inputs in its own
 // stage. Signed 17-bit intervals include -32768 without a negation overflow.
 wire signed [16:0] capture_limit=$signed({1'b0,measured_capture});
 wire captured_bound_ok=measured_capture<=16'd32768 &&
  $signed({velocity_l_capture[15],velocity_l_capture})>=-capture_limit &&
  $signed({velocity_l_capture[15],velocity_l_capture})<=capture_limit &&
  $signed({velocity_r_capture[15],velocity_r_capture})>=-capture_limit &&
  $signed({velocity_r_capture[15],velocity_r_capture})<=capture_limit &&
  !request_l_capture[15] && !request_r_capture[15] &&
  bound_capture>=measured_capture && bound_capture>=request_l_capture && bound_capture>=request_r_capture;
 wire speed_pending=velocity_l!=velocity_l_key || velocity_r!=velocity_r_key ||
  raw_l!=request_l_key || raw_r!=request_r_key || follow_mode!=follow_key ||
  velocity_l_capture!=velocity_l_key || velocity_r_capture!=velocity_r_key ||
  request_l_capture!=request_l_key || request_r_capture!=request_r_key || follow_capture!=follow_key;
 // Compare the unfused inputs directly so the immediate permission guard
 // does not traverse sonar min, laser selection and age selection logic.
 // Both occupied slots must drain, including an input returning to its old
 // value before adoption. This guard never enters envelope/cap/epoch logic.
 wire range_pending=range_raw!=range_key || range_capture!=range_key;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin
   range_capture<=0;range_key<=0;
   front<=0;front_age<=0;front_valid<=0;envelope_sample<=0;
   front_capture<=0;age_capture<=0;front_valid_capture<=0;sample_capture<=0;
   measured_capture<=0;bound_capture<=0;follow_capture<=0;follow_key<=0;bounds_ready<=0;
   velocity_l_capture<=0;velocity_r_capture<=0;request_l_capture<=0;request_r_capture<=0;
   velocity_l_key<=0;velocity_r_key<=0;request_l_key<=0;request_r_key<=0;measured<=0;registered_bound<=0;
  end
  else begin
   range_capture<=range_raw;range_key<=range_capture;
   front_capture<=front_live;age_capture<=front_age_live;front_valid_capture<=front_valid_live;
   sample_capture<=feedback_update || map_update;
   measured_capture<=measured_live;bound_capture<=combined_bound_live;
   velocity_l_capture<=velocity_l;velocity_r_capture<=velocity_r;
   request_l_capture<=raw_l;request_r_capture<=raw_r;follow_capture<=follow_mode;
   front<=front_capture;front_age<=age_capture;front_valid<=front_valid_capture;envelope_sample<=sample_capture;
   measured<=measured_capture;registered_bound<=bound_capture;bounds_ready<=captured_bound_ok;
   velocity_l_key<=velocity_l_capture;velocity_r_key<=velocity_r_capture;
   request_l_key<=request_l_capture;request_r_key<=request_r_capture;follow_key<=follow_capture;
  end
 wire env_valid,env_hold,env_config;
 wire [7:0] envelope_cap;
 wire signed [15:0] env_l,env_r;
 wire [15:0] stop_mm,slow_mm,speed_mm_s,latency_ms,env_cycles;
 // Map and coherent wheel samples are events, never a continuously asserted
 // sample input; a level would restart the serial arithmetic every clock.
 rb_lidar_envelope #(.CLK_HZ(CLK_HZ),.COUNTS_PER_M(COUNTS_PER_M),
  .DECEL_MM_S2(DECEL_MM_S2),.INPUT_BUDGET_MS(INPUT_BUDGET_MS),.USE_REGISTERED_COUNTS(1)) envelope(
  .clk(clk),.rst_n(rst_n),.sample(envelope_sample),
  .enabled(enabled && !stop_now),.follow_mode(follow_key),
  .front_valid(front_valid),.front_mm(front),.age_ms(front_age),
  .velocity_l(velocity_l_key),.velocity_r(velocity_r_key),.request_l(request_l_key),.request_r(request_r_key),
  .registered_counts(registered_bound),.registered_counts_valid(bounds_ready),
  .valid(env_valid),.hold(env_hold),.limited_l(env_l),.limited_r(env_r),
  .stop_mm(stop_mm),.slow_mm(slow_mm),.speed_mm_s(speed_mm_s),
  .latency_ms(latency_ms),.solve_cycles(env_cycles),.config_ok(env_config),.published_cap(envelope_cap));
 reg [7:0] cap_q;
 // Consume the raw atomic cap, not valid-gated wheel passthrough values.
 // The envelope result and its cap register are published on adjacent
 // edges. Do not release an old faster plan during that adoption edge.
 wire cap_pending=env_valid && envelope_cap!=cap_q;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)cap_q<=0;
  else if(!enabled || stop_now || !observed_forward)cap_q<=0;
  else if(env_valid)cap_q<=envelope_cap;
 wire [7:0] allowed_speed=cap_q<observed_speed?cap_q:observed_speed;
 reg [15:0] command_epoch,generation;
 reg [7:0] previous_speed;reg [15:0] previous_measured;
 reg signed [15:0] previous_goal;
 reg [5:0] previous_mode;
 wire [5:0] mode={armed,follow_mode,nav_mode,request_cmd};
 wire context_changed=previous_speed!=allowed_speed || previous_measured!=measured ||
                      previous_goal!=goal_cd || previous_mode!=mode;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin command_epoch<=0;previous_speed<=0;previous_measured<=0;previous_goal<=0;previous_mode<=0;end
  else begin
   previous_speed<=allowed_speed;previous_measured<=measured;previous_goal<=goal_cd;previous_mode<=mode;
   if(context_changed)command_epoch<=command_epoch+1'b1;
  end
 wire busy,result_valid,no_path,plan_fault;
 wire signed [15:0] proposed_l,proposed_r;
 wire [8:0] admissible,unknown;
 wire [15:0] clearance,cycles,result_scan,result_epoch;
 reg [3:0] previous_candidate;
 reg was_busy,completion_event;
 wire map_usable=map_fresh && map_age_ms<=350;
 wire cancel=!enabled || stop_now || follow_mode || !observed_forward || !map_usable || !goal_valid || !bounds_ready;
 reg lattice_start,lattice_cancel;
 wire context_match=!context_changed && !cap_pending && result_scan==map_scan_id && result_epoch==command_epoch;
 wire plan_current=result_valid && context_match && map_usable && !cancel && !lattice_cancel;
 assign current=plan_current && !range_pending && !speed_pending;
 wire completed_no_path=no_path && context_match && map_usable && !cancel && !lattice_cancel;
 wire start=enabled && !cancel && !lattice_cancel && !busy && !context_changed && !cap_pending &&
            !(context_match && (result_valid || no_path));
 // Break the command/cap/epoch decode before the lattice's wide control fanout.
 // Live guards above withdraw permission immediately while cancellation moves
 // through this one-clock boundary; a registered start is a single pulse.
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin lattice_start<=0;lattice_cancel<=1;end
  else begin
   lattice_start<=start && !lattice_start;
   lattice_cancel<=cancel || context_changed || cap_pending;
  end
 rb_lidar_lattice #(.COUNTS_PER_M(COUNTS_PER_M),.WHEELBASE_MM(WHEELBASE_MM),
  .BODY_RADIUS_MM(BODY_RADIUS_MM),.INPUT_BUDGET_MS(INPUT_BUDGET_MS),.DECEL_MM_S2(DECEL_MM_S2)) lattice(
  .clk(clk),.rst_n(rst_n),.start(lattice_start),.cancel(lattice_cancel),
  .map_fresh(map_usable),.map_ranges(map_ranges),.map_known(map_known),
  .map_scan_id(map_scan_id),.map_age_ms(map_age_ms),.request_max(allowed_speed),
  .measured_max_counts(measured),.goal_cd(goal_cd),.command_epoch(command_epoch),
  .previous_candidate(previous_candidate),.busy(busy),.result_valid(result_valid),
  .no_path(no_path),.fault(plan_fault),.selected(selected),.target_l(proposed_l),.target_r(proposed_r),
  .admissible_mask(admissible),.unknown_mask(unknown),.clearance_mm(clearance),
  .solve_cycles(cycles),.result_scan_id(result_scan),.result_epoch(result_epoch));
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin generation<=0;previous_candidate<=15;was_busy<=0;completion_event<=0;end
  else begin
   was_busy<=busy;
   // Diagnostic counting is one clock after the accepted completion event.
   // Keep that wide counter off the live envelope/context validity path.
   completion_event<=was_busy && !busy && (plan_current || completed_no_path);
   if(completion_event)generation<=generation+1'b1;
   if(plan_current)previous_candidate<=selected;
  end
 assign dynamic_hold=env_valid && env_hold;
 assign follow_limit_valid=control_ready && armed && follow_mode && forward_cmd &&
                           front_valid_live && !stop_now;
 assign dynamic_gate=scope || follow_limit_valid;
 assign hold=(scope && (!current || !env_valid || env_hold)) ||
             (follow_limit_valid && (range_pending || speed_pending || !env_valid || env_hold));
 assign takeover=scope;
 assign target_l=current?proposed_l:16'sd0;
 assign target_r=current?proposed_r:16'sd0;
 assign follow_l=env_valid && !range_pending && !speed_pending?env_l:16'sd0;
 assign follow_r=env_valid && !range_pending && !speed_pending?env_r:16'sd0;
 reg [31:0] ms_div,time_ms;
 localparam MS_DIV=(CLK_HZ/1000>0)?CLK_HZ/1000:1;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin ms_div<=0;time_ms<=0;end
  else if(ms_div==MS_DIV-1)begin ms_div<=0;time_ms<=time_ms+1'b1;end
  else ms_div<=ms_div+1'b1;
 wire visible_hold=dynamic_hold || (scope && hold);
 wire visible_valid=current && env_valid && !visible_hold;
 wire [7:0] visible_candidate=visible_valid?{4'd0,selected}:8'hff;
 wire [15:0] flags={2'b00,original_control_enabled,follow_mode,
  (plan_fault || !env_config || !GEOMETRY_OK),
  (context_changed || ((!map_usable || !goal_valid) && enabled)),
  (!map_usable || map_known!={36{1'b1}}),completed_no_path,
  visible_hold,(env_valid && (env_l<raw_l || env_r<raw_r)),
  scope,visible_valid,map_usable,validated,enabled,1'b1};
 wire [15:0] selected_cycles=cycles>env_cycles?cycles:env_cycles;
 wire [319:0] diagnostic_next={env_valid?latency_ms:16'd0,command_epoch,goal_cd,
  7'd0,unknown,7'd0,admissible,
  visible_valid?proposed_r:16'sd0,visible_valid?proposed_l:16'sd0,
  8'd9,visible_candidate,selected_cycles,env_valid?speed_mm_s:16'd0,
  visible_valid?clearance:16'd0,env_valid?slow_mm:16'd0,env_valid?stop_mm:16'd0,
  map_age_ms,generation,map_scan_id,time_ms,flags,16'd0};
 // Freeze the complete C8 bundle together. time_ms identifies this snapshot,
 // not the later UART serialization edge, and no field can mix generations.
 always @(posedge clk or negedge rst_n)
  if(!rst_n)diagnostic<=0;else diagnostic<=diagnostic_next;
endmodule

