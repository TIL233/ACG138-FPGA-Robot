`timescale 1ns/1ps
// PI in counts/20ms, duty in 50MHz clocks, 20kHz carrier. No reverse torque
// for overspeed: direction is always the requested direction. Brake is independent.
module rb_wheel_control #(parameter CLK_HZ=50000000, MOT_INV=0,
 parameter FF=16,KP=8,KI=2,MAX_DUTY=2000,SLEW=64,
 parameter DIR_GUARD_WINDOWS=10,WRONG_DIR_WINDOWS=5,PAUSE_ENABLE=0,ACTUATOR_PIPELINE=0)(
 input clk,rst_n,brake,clear_fault,
 input signed [15:0] target,input signed [31:0] position,
 output reg signed [15:0] velocity,output reg fault,
 output wire pwm,in1,in2,input planner_pause,output wire actuator_pending);
 localparam WIN=CLK_HZ/50,PERIOD=CLK_HZ/20000;
 reg [31:0] timer,last_pos;
 reg [15:0] carrier;
 reg signed [31:0] integral;
 reg signed [15:0] duty,previous_target,duty_anchor;
 reg [10:0] pause_cycles;
 reg paused,resuming,soft_pause_q,soft_zero_q;
 localparam PIPE=(PAUSE_ENABLE!=0 && ACTUATOR_PIPELINE!=0);
 // One coherent actuator capture. Raw inputs only reach these small capture
 // registers and the physical output barrier, never the wide PI/monitor CE.
 reg brake_q,reversal_q,timeout_q,drive_pending;
 reg signed [15:0] target_q;
 wire signed [15:0] state_target=PIPE?target_q:target;
 reg monitor_tick,monitor_zero,monitor_target_stall,monitor_previous_stall;
 reg monitor_target_wrong,monitor_previous_wrong;
 reg [5:0] stall;
 reg [7:0] direction_guard,wrong_direction;
 reg [2:0] phase;
 reg signed [15:0] target_hold;
 reg signed [31:0] error_hold,isum_hold,base_hold,raw_hold,limited_hold,integral_next;
 wire signed [31:0] delta=position-$signed(last_pos);
 wire signed [31:0] iclamp=isum_hold>1000?32'sd1000:(isum_hold< -1000?-32'sd1000:isum_hold);
 wire signed [31:0] limited=target_hold>0 ? (raw_hold<0?32'sd0:(raw_hold>MAX_DUTY?MAX_DUTY:raw_hold)) :
                                      (raw_hold>0?32'sd0:(raw_hold< -MAX_DUTY?-MAX_DUTY:raw_hold));
 // Recalculation after a short pause is not another 20ms slew step.
 // Use the same window origin until the next real feedback boundary.
 wire signed [15:0] slew_origin=(PAUSE_ENABLE!=0)?duty_anchor:duty;
 wire signed [31:0] slew=limited_hold>$signed(slew_origin)+SLEW ? $signed(slew_origin)+SLEW :
                              (limited_hold<$signed(slew_origin)-SLEW?$signed(slew_origin)-SLEW:limited_hold);
 wire neg=duty[15] ^ (MOT_INV!=0);
 wire [15:0] mag=duty[15] ? -duty : duty;
 wire pause_request=(PAUSE_ENABLE!=0) && planner_pause;
 // A transient zero target belongs to the planner pause, not a new STOP.
 // The independent brake input and target reversal always clear state.
 // The advanced counter follows the existing one-clock pause queue.
 // After raw edge n, q=raw[n] and count=old_raw_count[n-1]. This compensated
 // threshold is exactly old_raw_count>=1023, including sampled low gaps.
 wire pause_timeout=(PAUSE_ENABLE!=0)?(soft_pause_q && pause_cycles>=1022):(pause_cycles>=1023);
 wire raw_zero_stop=target==0 && !pause_request;
 wire stop_brake=brake || fault || raw_zero_stop || (pause_request && pause_timeout);
 // PIPE captures the complete control bundle. Physical hard/soft/reversal
 // barriers remain immediate; their state cleanup consumes the captured event.
 wire state_pause=(PAUSE_ENABLE!=0)?soft_pause_q:pause_request;
 wire state_zero_stop=(PAUSE_ENABLE!=0)?soft_zero_q:raw_zero_stop;
 wire state_stop_brake=PIPE?(brake_q || fault || state_zero_stop || timeout_q):
                          (brake || fault || state_zero_stop || (pause_request && pause_timeout));
 // Gate a target sign change immediately, rather than leaving old-direction
 // torque active until the next 20ms PI sample. Keep a zero-output window.
 wire direction_changed=target!=0 && previous_target!=0 && target[15]!=previous_target[15];
 wire hard_brake=stop_brake || direction_changed;
 wire state_hard_brake=state_stop_brake || (PIPE?reversal_q:direction_changed);
 wire output_brake=hard_brake || pause_request || state_pause ||
                   ((PAUSE_ENABLE!=0) && soft_zero_q) || paused || resuming ||
                   (PIPE && (drive_pending || target!=target_q || target_q!=target_hold || brake_q || reversal_q || timeout_q));
 // Acceptance is distinct from a physical short brake: a stable zero
 // target is accepted after its captured cleanup and intentionally brakes.
 assign actuator_pending=PIPE && (brake || fault || pause_request || direction_changed ||
                      (target!=target_q) || (target_q!=target_hold) || brake_q || reversal_q ||
                      timeout_q || state_pause || drive_pending);
 // Select already-computed one-bit predicates, never a signed16 target
 // followed by comparisons on the live planner-pause fault path.
 wire target_stall=target>=4 || target<=-4;
 wire previous_stall=previous_target>=4 || previous_target<=-4;
 wire target_wrong=(target>0 && delta<=-2) || (target<0 && delta>=2);
 wire previous_wrong=(previous_target>0 && delta<=-2) || (previous_target<0 && delta>=2);
 wire supervised_stall=pause_request?previous_stall:target_stall;
 wire supervised_wrong=pause_request?previous_wrong:target_wrong;
 // Short brake: IN1=IN2=PWM=1. Reset suppresses all drive signals.
 assign pwm=rst_n && (output_brake || carrier<mag);
 assign in1=rst_n && (output_brake || (!neg && mag!=0));
 assign in2=rst_n && (output_brake || (neg && mag!=0));
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin timer<=0;last_pos<=0;carrier<=0;integral<=0;duty<=0;
   previous_target<=0;duty_anchor<=0;stall<=0;direction_guard<=0;wrong_direction<=0;velocity<=0;fault<=0;phase<=0;
   pause_cycles<=0;paused<=0;resuming<=0;soft_pause_q<=0;soft_zero_q<=0;
   brake_q<=1;reversal_q<=0;timeout_q<=0;target_q<=0;drive_pending<=1;
   monitor_tick<=0;monitor_zero<=0;monitor_target_stall<=0;monitor_previous_stall<=0;
   monitor_target_wrong<=0;monitor_previous_wrong<=0;
   target_hold<=0;error_hold<=0;isum_hold<=0;base_hold<=0;raw_hold<=0;
   limited_hold<=0;integral_next<=0;end
  else begin
   if(carrier==PERIOD-1) carrier<=0;else carrier<=carrier+1;
   if(clear_fault) fault<=0;
   soft_pause_q<=pause_request;
   soft_zero_q<=raw_zero_stop;
   brake_q<=brake;target_q<=target;reversal_q<=direction_changed;
   timeout_q<=pause_request && pause_timeout;
   monitor_tick<=(timer==WIN-1);
   if(timer==WIN-1)begin
    monitor_zero<=(delta==0);monitor_target_stall<=target_stall;
    monitor_previous_stall<=previous_stall;monitor_target_wrong<=target_wrong;
    monitor_previous_wrong<=previous_wrong;
   end
   if(PIPE)begin
    // A captured veto or changed packet wins over any previous calculation.
    // phase==0 is not an acknowledgement: cancellation also enters phase 0.
    if(brake_q || fault || reversal_q || timeout_q || state_pause)drive_pending<=1;
    else if(state_zero_stop)drive_pending<=0;
    else if(state_target!=target_hold)drive_pending<=1;
    else if(phase==4)drive_pending<=0;
   end
   if(!state_pause)pause_cycles<=0;
   else if(pause_cycles<1024)pause_cycles<=pause_cycles+1'b1;
   // Sample at the original 20ms boundary, then calculate in four clocks.
   // A stop bypasses the pipeline; changed targets discard pending results.
   if(state_hard_brake) begin
    duty<=0;duty_anchor<=0;integral<=0;phase<=0;
    if(PIPE && state_zero_stop)target_hold<=0;
    paused<=PIPE?1'b1:state_pause;resuming<=0;
   end
   else if(state_pause)begin
    // Cancel partial arithmetic, retaining the last complete PI state.
    phase<=0;paused<=1;resuming<=0;
   end
   else if(paused || (resuming && phase==0) || (PIPE && state_target!=target_hold))begin
    // Keep the bridge short-braked until the current target and the newest
    // 20ms feedback have produced a complete replacement duty.
    paused<=0;resuming<=1;target_hold<=state_target;
    if(PIPE || (!pause_request && !hard_brake))previous_target<=state_target;
    error_hold<=$signed(state_target)-$signed(velocity);phase<=1;
   end
   else if(phase!=0 && state_target!=target_hold) phase<=0;
   else case(phase)
    1: begin
     // Resume is a recomputation, not an extra 20ms integration step.
     isum_hold<=resuming?integral:integral+error_hold*KI;
     base_hold<=$signed(target_hold)*FF+error_hold*KP;
     phase<=2;
    end
    2: begin integral_next<=iclamp;raw_hold<=base_hold+iclamp;phase<=3;end
    3: begin limited_hold<=limited;phase<=4;end
    4: begin duty<=slew[15:0];integral<=integral_next;phase<=0;resuming<=0;end
    default: phase<=0;
   endcase
   // The legacy path keeps its original raw window-edge priority. PIPE uses
   // the matching captured event, including a STOP on the exact WIN edge.
   if(PIPE?state_hard_brake:hard_brake)begin stall<=0;direction_guard<=0;wrong_direction<=0;end
   if(PIPE?state_stop_brake:stop_brake)previous_target<=0;
   // Window supervision consumes the flags captured with that window's
   // position, raw pause and STOP. It publishes exactly one clock later.
   // A new raw STOP at the window cannot spuriously increment fault state.
   if(PIPE && monitor_tick && !state_hard_brake)begin
    if(monitor_zero && (state_pause?monitor_previous_stall:monitor_target_stall))begin
     if(stall>=49)fault<=1;else stall<=stall+1'b1;
    end else stall<=0;
    if(direction_guard<DIR_GUARD_WINDOWS)begin
     direction_guard<=direction_guard+1'b1;wrong_direction<=0;
    end else if(state_pause?monitor_previous_wrong:monitor_target_wrong)begin
     if(wrong_direction>=WRONG_DIR_WINDOWS-1)fault<=1;
     else wrong_direction<=wrong_direction+1'b1;
    end else wrong_direction<=0;
   end
   if(timer==WIN-1) begin
    timer<=0;last_pos<=position;velocity<=delta[15:0];
    if(!state_hard_brake)duty_anchor<=duty;
    if(delta>512 || delta< -512) fault<=1;
    if(PIPE)begin
     if(!state_stop_brake && reversal_q)previous_target<=state_target;
     else if(!state_hard_brake && !state_pause)previous_target<=state_target;
    end else begin
     if(!stop_brake && direction_changed) previous_target<=target;
     else if(!hard_brake && !pause_request)previous_target<=target;
    end
    if(!state_hard_brake) begin
     if(!state_pause)begin
      if(state_target!=0 && state_target[15]!=previous_target[15] && previous_target!=0) begin duty<=0;integral<=0;stall<=0;phase<=0;end
      else begin target_hold<=state_target;error_hold<=$signed(state_target)-delta;phase<=1;end
     end
    end
    if(!PIPE && !hard_brake)begin
     // Include the legal four-count minimum used by remote/follow/turning.
     if(delta==0 && supervised_stall) begin
      if(stall>=49) fault<=1;else stall<=stall+1;
     end else stall<=0;
     // Allow 200ms for a start/reversal to shed old wheel inertia, then
     // require 100ms of opposite feedback before latching a wiring fault.
     // A little count jitter does not qualify as opposite movement.
     if(direction_guard<DIR_GUARD_WINDOWS) begin
      direction_guard<=direction_guard+1'b1;wrong_direction<=0;
     end else if(supervised_wrong) begin
      if(wrong_direction>=WRONG_DIR_WINDOWS-1) fault<=1;
      else wrong_direction<=wrong_direction+1'b1;
     end else wrong_direction<=0;
    end
   end else timer<=timer+1;
  end
 end
endmodule
