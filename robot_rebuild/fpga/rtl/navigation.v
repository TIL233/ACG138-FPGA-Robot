`timescale 1ns/1ps
// Local, short-range waypoint navigation. Coordinates are millimetres; +Y and
// positive heading are left/CCW from the explicitly reset starting direction.
// COUNTS_PER_M and YAW_SIGN require calibration on the actual chassis.
// The existing safety controller retains ownership of obstacle avoidance.
module rb_navigation #(
 parameter CLK_HZ=50000000, CONTROL_HZ=50, COUNTS_PER_M=7640,
 parameter YAW_SIGN=1, MAX_SPEED=12, SLOW_SPEED=6,
 parameter ARRIVE_MM=120, SLOW_MM=600, SETTLE_MS=200,
 parameter TOTAL_TIMEOUT_MS=180000, PROGRESS_TIMEOUT_MS=20000,
 parameter TURN_TIMEOUT_MS=15000, MAX_DELTA_COUNTS=1024,
 parameter MAX_YAW_STEP_CD=3000, PROGRESS_MM=20,
 parameter LOCAL_STEERING_ENABLE=0
)(
 input clk,rst_n,run,healthy,
 input [3:0] safety_state,
 input signed [31:0] pos_l,pos_r,
 input signed [15:0] yaw,
 input route_write,input [2:0] route_index,
 input signed [15:0] way_x,way_y,
 input route_commit,input [3:0] route_count,
 input route_clear,pose_reset,
 output reg [2:0] command,output reg [7:0] speed,
 output reg active,done,failed,output reg [3:0] state,
 output reg [2:0] index,
 output signed [31:0] x,y,output reg signed [15:0] heading,
 output reg route_ready,pose_ready,
 output signed [15:0] turn_error,output turn_error_valid,
 input local_steering,
 output signed [15:0] goal_error,output goal_error_valid,
 output [2:0] intent_command,output [7:0] intent_speed,output output_permitted
);
 localparam IDLE=0,TURNING=1,DRIVING=2,SETTLING=3,AVOIDING=4,
            COMPLETE=5,FAILED=6,PAUSED=7;
 // Keep enough clocks for the sequential math when a testbench scales CLK_HZ.
 localparam CONTROL_DIV=(CLK_HZ/CONTROL_HZ<32)?32:CLK_HZ/CONTROL_HZ;
 localparam TICK_MS=(1000*CONTROL_DIV+CLK_HZ-1)/CLK_HZ;
 // Pair count is left+right, so the coefficient already includes /2.
 localparam MM_Q16_PER_PAIR=(32768000+COUNTS_PER_M/2)/COUNTS_PER_M;
 reg [31:0] divider;
 wire tick=(divider==CONTROL_DIV-1);
 reg signed [15:0] staged_x[0:7],staged_y[0:7];
 reg signed [15:0] route_x[0:7],route_y[0:7];
 reg [7:0] staged_valid;
 reg [3:0] count;
 reg [7:0] route_epoch,calculation_epoch;
 reg [2:0] calculation_index;
 reg commit_ok,config_error,reset_ok;
 integer k;
 function [7:0] required_mask;
  input [3:0] n;
  begin case(n)
   1:required_mask=8'h01;2:required_mask=8'h03;3:required_mask=8'h07;
   4:required_mask=8'h0f;5:required_mask=8'h1f;6:required_mask=8'h3f;
   7:required_mask=8'h7f;8:required_mask=8'hff;default:required_mask=0;
  endcase end
 endfunction
 function signed [16:0] wrap_angle;
  input signed [17:0] a;
  begin
   if(a>18000) wrap_angle=a-18000-18000;
   else if(a< -18000) wrap_angle=a+18000+18000;
   else wrap_angle=a;
  end
 endfunction
 function signed [16:0] atan_cd;
  input [3:0] i;
  begin case(i)
   0:atan_cd=4500;1:atan_cd=2657;2:atan_cd=1404;3:atan_cd=713;
   4:atan_cd=358;5:atan_cd=179;6:atan_cd=90;7:atan_cd=45;
   8:atan_cd=22;9:atan_cd=11;10:atan_cd=6;11:atan_cd=3;
   12:atan_cd=1;13:atan_cd=1;default:atan_cd=0;
  endcase end
 endfunction
 wire yaw_valid=($signed(yaw)>=-18000 && $signed(yaw)<=18000);
 // Configuration is accepted while run=0; the caller guarantees stopped and
 // disarmed. Staging prevents editing a paused route without committing it.
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   staged_valid<=0;count<=0;route_ready<=0;route_epoch<=0;
   commit_ok<=0;config_error<=0;reset_ok<=0;
   for(k=0;k<8;k=k+1) begin
    staged_x[k]<=0;staged_y[k]<=0;route_x[k]<=0;route_y[k]<=0;
   end
  end else begin
   commit_ok<=0;config_error<=0;reset_ok<=0;
   if(!run) begin
    if(route_clear) begin route_ready<=0;count<=0;staged_valid<=0;route_epoch<=route_epoch+1;end
    else begin
     if(route_write) begin
      staged_x[route_index]<=way_x;staged_y[route_index]<=way_y;
      staged_valid[route_index]<=1;
     end
     if(route_commit) begin
      if(route_count>=1 && route_count<=8 &&
         (staged_valid & required_mask(route_count))==required_mask(route_count)) begin
       for(k=0;k<8;k=k+1) begin route_x[k]<=staged_x[k];route_y[k]<=staged_y[k];end
       count<=route_count;route_ready<=1;commit_ok<=1;route_epoch<=route_epoch+1;
      end else begin route_ready<=0;config_error<=1;end
     end
    end
    if(pose_reset) begin
     if(healthy && yaw_valid) reset_ok<=1;
     else config_error<=1;
    end
   end
  end
 end

 reg signed [47:0] x_q16,y_q16;
 assign x=x_q16>>>16;
 assign y=y_q16>>>16;
 reg signed [31:0] prev_l,prev_r;
 reg signed [15:0] yaw_origin,prev_yaw;
 wire signed [31:0] delta_l=$signed(pos_l)-$signed(prev_l);
 wire signed [31:0] delta_r=$signed(pos_r)-$signed(prev_r);
 wire signed [32:0] delta_sum={delta_l[31],delta_l}+{delta_r[31],delta_r};
 wire signed [16:0] yaw_step=wrap_angle($signed({{2{yaw[15]}},yaw})-
                                           $signed({{2{prev_yaw[15]}},prev_yaw}));
 wire signed [16:0] relative_yaw=wrap_angle(YAW_SIGN*($signed({{2{yaw[15]}},yaw})-
                                           $signed({{2{yaw_origin[15]}},yaw_origin})));
 wire bad_step=(delta_l>MAX_DELTA_COUNTS || delta_l< -MAX_DELTA_COUNTS ||
                delta_r>MAX_DELTA_COUNTS || delta_r< -MAX_DELTA_COUNTS ||
                yaw_step>MAX_YAW_STEP_CD || yaw_step< -MAX_YAW_STEP_CD);
 reg [31:0] still_ms;
 // Fixed anchors share avoidance's stationary +/-1-count tolerance. Check
 // every clock so small successive moves cannot hide cumulative creep.
 reg [31:0] settle_l,settle_r,settle_l_minus,settle_l_plus,settle_r_minus,settle_r_plus;
 wire stationary=(pos_l==settle_l || pos_l==settle_l_minus || pos_l==settle_l_plus) &&
                 (pos_r==settle_r || pos_r==settle_r_minus || pos_r==settle_r_plus);
 reg pose_error,math_done;
 reg [4:0] phase;
 reg [3:0] iteration;
 reg signed [31:0] distance_q16;
 reg signed [19:0] cord_x,cord_y;
 reg signed [16:0] cord_angle;
 reg cord_negative;
 reg signed [19:0] cosine,sine;
 // One shared signed multiplier, used sequentially for pose and steering.
 reg signed [32:0] mul_a;
 reg signed [19:0] mul_b;
 wire signed [52:0] product=mul_a*mul_b;
 reg signed [32:0] dx,dy;
 wire [32:0] ax=dx[32]?-dx:dx;
 wire [32:0] ay=dy[32]?-dy:dy;
 reg [33:0] distance_mm;
 reg signed [53:0] dot,turn_cross,accumulator;
 wire [53:0] cross_abs=turn_cross[53]?-turn_cross:turn_cross;
 wire facing=(!dot[53] && dot!=0);
 wire aligned=facing && cross_abs<=($unsigned(dot)>>3); // about 7 degrees
 wire needs_turn=!facing || cross_abs>($unsigned(dot)>>2); // about 14 degrees
 wire needs_trim=facing && cross_abs>($unsigned(dot)>>4); // about 3.6 degrees
 wire left_turn=!turn_cross[53];
 wire vector_valid;
 rb_vector_angle target_angle(.clk(clk),.rst_n(rst_n),
  .clear(!run || route_write || route_commit || pose_reset || route_clear),
  .start(math_done),.x(dot),.y(turn_cross),.valid(vector_valid),.angle(turn_error));
 assign turn_error_valid=vector_valid && active && state==TURNING;
 assign goal_error=turn_error;
 assign goal_error_valid=vector_valid && active && healthy;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   divider<=0;x_q16<=0;y_q16<=0;heading<=0;pose_ready<=0;
   prev_l<=0;prev_r<=0;yaw_origin<=0;prev_yaw<=0;still_ms<=0;
   settle_l<=0;settle_r<=0;settle_l_minus<=32'hffffffff;settle_l_plus<=1;
   settle_r_minus<=32'hffffffff;settle_r_plus<=1;
   phase<=0;iteration<=0;pose_error<=0;math_done<=0;
   distance_q16<=0;cord_x<=0;cord_y<=0;cord_angle<=0;cord_negative<=0;
   cosine<=0;sine<=0;mul_a<=0;mul_b<=0;dx<=0;dy<=0;
   distance_mm<=0;dot<=0;turn_cross<=0;accumulator<=0;
   calculation_epoch<=0;calculation_index<=0;
  end else begin
   if(tick) divider<=0;else divider<=divider+1;
   pose_error<=0;math_done<=0;
   if(command!=0 || !stationary || (!run && pose_reset))begin
    still_ms<=0;settle_l<=pos_l;settle_r<=pos_r;
    settle_l_minus<=pos_l-32'd1;settle_l_plus<=pos_l+32'd1;
    settle_r_minus<=pos_r-32'd1;settle_r_plus<=pos_r+32'd1;
   end else if(tick && still_ms<SETTLE_MS)still_ms<=still_ms+TICK_MS;
   if(!run && pose_reset) begin
    phase<=0;still_ms<=0;prev_l<=pos_l;prev_r<=pos_r;prev_yaw<=yaw;
    if(healthy && yaw_valid) begin
     x_q16<=0;y_q16<=0;heading<=0;yaw_origin<=yaw;pose_ready<=1;
    end else pose_ready<=0;
   end else begin
    if(tick) begin
     prev_l<=pos_l;prev_r<=pos_r;prev_yaw<=yaw;
     if(pose_ready) begin
      if(!yaw_valid || bad_step || (!healthy && (delta_l!=0 || delta_r!=0))) begin
       // Do not integrate a discontinuity or movement with stale sensors.
       pose_error<=1;pose_ready<=0;phase<=0;
      end else if(healthy && phase==0) begin
       heading<=relative_yaw;
       distance_q16<=delta_sum*MM_Q16_PER_PAIR;
       cord_x<=20'sd19898;cord_y<=0;iteration<=0;
       if(relative_yaw>9000) begin cord_angle<=relative_yaw-18000;cord_negative<=1;end
       else if(relative_yaw< -9000) begin cord_angle<=relative_yaw+18000;cord_negative<=1;end
       else begin cord_angle<=relative_yaw;cord_negative<=0;end
       phase<=1;
      end
     end
    end
    case(phase)
     1: begin
      if(cord_angle>=0) begin
       cord_x<=cord_x-(cord_y>>>iteration);
       cord_y<=cord_y+(cord_x>>>iteration);
       cord_angle<=cord_angle-atan_cd(iteration);
      end else begin
       cord_x<=cord_x+(cord_y>>>iteration);
       cord_y<=cord_y-(cord_x>>>iteration);
       cord_angle<=cord_angle+atan_cd(iteration);
      end
      if(iteration==13) phase<=2;else iteration<=iteration+1;
     end
     2: begin
      cosine<=cord_negative?-cord_x:cord_x;sine<=cord_negative?-cord_y:cord_y;
      phase<=3;
     end
     3: begin mul_a<={{1{distance_q16[31]}},distance_q16};mul_b<=cosine;phase<=4;end
     4: begin x_q16<=x_q16+(product>>>15);mul_b<=sine;phase<=5;end
     5: begin y_q16<=y_q16+(product>>>15);phase<=6;end
     6: begin
      dx<=$signed({{17{route_x[index][15]}},route_x[index]})-$signed({x[31],x});
      dy<=$signed({{17{route_y[index][15]}},route_y[index]})-$signed({y[31],y});
      calculation_epoch<=route_epoch;calculation_index<=index;phase<=7;
     end
     7: begin
      distance_mm<=(ax>=ay)?({1'b0,ax}+({1'b0,ay}>>1)):({1'b0,ay}+({1'b0,ax}>>1));
      mul_a<=dx;mul_b<=cosine;phase<=8;
     end
     8: begin accumulator<=product;mul_a<=dy;mul_b<=sine;phase<=9;end
     9: begin dot<=accumulator+product;mul_a<=dy;mul_b<=cosine;phase<=10;end
     10: begin accumulator<=product;mul_a<=dx;mul_b<=sine;phase<=11;end
     11: begin turn_cross<=accumulator-product;phase<=12;end
     12: begin
      math_done<=(calculation_epoch==route_epoch && calculation_index==index);
      phase<=0;
     end
     default: ;
    endcase
   end
  end
 end

 reg [31:0] total_ms,progress_ms,turn_ms;
 reg [33:0] best_distance;
 reg [2:0] desired_command;
 reg [7:0] desired_speed;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   active<=0;done<=0;failed<=0;state<=IDLE;index<=0;
   total_ms<=0;progress_ms<=0;turn_ms<=0;best_distance<={34{1'b1}};
   desired_command<=0;desired_speed<=0;
  end else begin
   if(tick && active) begin
    if(total_ms<TOTAL_TIMEOUT_MS+TICK_MS) total_ms<=total_ms+TICK_MS;
    if(state==DRIVING && progress_ms<PROGRESS_TIMEOUT_MS+TICK_MS) progress_ms<=progress_ms+TICK_MS;
    if(state==TURNING && turn_ms<TURN_TIMEOUT_MS+TICK_MS) turn_ms<=turn_ms+TICK_MS;
   end
   if(!run && (route_clear || commit_ok || reset_ok)) begin
    active<=0;done<=0;state<=IDLE;index<=0;
    if(commit_ok || reset_ok) failed<=0;
    total_ms<=0;progress_ms<=0;turn_ms<=0;best_distance<={34{1'b1}};
    desired_command<=0;desired_speed<=0;
   end else if(config_error || pose_error || (active && (!healthy || safety_state>=7)) ||
               (active && (total_ms>=TOTAL_TIMEOUT_MS || progress_ms>=PROGRESS_TIMEOUT_MS || turn_ms>=TURN_TIMEOUT_MS))) begin
    active<=0;failed<=1;state<=FAILED;desired_command<=0;desired_speed<=0;
   end else if(!run) begin
    active<=0;desired_command<=0;desired_speed<=0;
    if(active) state<=PAUSED;
   end else if(!active && !failed && !done) begin
    if(!route_ready || !pose_ready || !healthy || safety_state>=7) begin
     failed<=1;state<=FAILED;desired_command<=0;desired_speed<=0;
    end else begin
     active<=1;state<=TURNING;turn_ms<=0;
     desired_command<=0;desired_speed<=0;
    end
   end else if(active) begin
    if(safety_state>=1 && safety_state<=6) begin
     state<=AVOIDING;desired_command<=1;desired_speed<=MAX_SPEED;
     // Safety owns the complete detour. Do not steer it, but keep integrating.
     turn_ms<=0;progress_ms<=0;best_distance<={34{1'b1}};
    end else if(state==AVOIDING) begin
     state<=TURNING;turn_ms<=0;desired_command<=0;desired_speed<=0;
    end else if(math_done) begin
     if(state==SETTLING) begin
      desired_command<=0;desired_speed<=0;
      if(stationary && still_ms>=SETTLE_MS) begin
       if(distance_mm>ARRIVE_MM) begin state<=TURNING;turn_ms<=0;end
       else if({1'b0,index}+1>=count) begin active<=0;done<=1;state<=COMPLETE;end
       else begin
        index<=index+1;state<=TURNING;progress_ms<=0;turn_ms<=0;
        best_distance<={34{1'b1}};
       end
      end
     end else if(distance_mm<=ARRIVE_MM) begin
      state<=SETTLING;desired_command<=0;desired_speed<=0;
     end else begin
      if(distance_mm+PROGRESS_MM<best_distance) begin best_distance<=distance_mm;progress_ms<=0;end
      desired_speed<=(distance_mm<SLOW_MM)?SLOW_SPEED:MAX_SPEED;
      // Calibrated local planning owns angular steering only on the ordinary
      // driving branch; arrival, settling and legacy avoidance retain priority.
      if(LOCAL_STEERING_ENABLE && local_steering) begin
       state<=DRIVING;desired_command<=1;turn_ms<=0;
      end else if(state==TURNING) begin
       if(aligned) begin state<=DRIVING;desired_command<=1;turn_ms<=0;end
       else begin
        desired_command<=left_turn?3'd5:3'd6;
        // Reduce turn speed as the target comes into view (about 27 degrees).
        // This is a heading-feedback decision on FPGA, not a timed RK turn.
        desired_speed<=(facing && cross_abs<=($unsigned(dot)>>1))?
                       ((SLOW_SPEED<4)?SLOW_SPEED:4):SLOW_SPEED;
       end
      end else if(needs_turn) begin
       state<=TURNING;turn_ms<=0;desired_command<=left_turn?3'd5:3'd6;
       desired_speed<=SLOW_SPEED;
      end else desired_command<=needs_trim?(left_turn?3'd3:3'd4):3'd1;
     end
    end
   end
  end
 end
 // Loss of permission stops outputs immediately, independent of the 20ms loop.
 // Ungated intent lets consumers compare data before the live permission gate.
 // These appended sidebands add no state and do not change the public outputs.
 assign intent_command=desired_command;
 assign intent_speed=(desired_speed>MAX_SPEED)?MAX_SPEED:desired_speed;
 assign output_permitted=rst_n && run && active && !failed && healthy && pose_ready && route_ready && safety_state<=6;
 always @* begin
  command=0;speed=0;
  if(rst_n && run && active && !failed && healthy && pose_ready && route_ready && safety_state<=6) begin
   command=desired_command;
   speed=(desired_speed>MAX_SPEED)?MAX_SPEED:desired_speed;
  end
 end
endmodule
