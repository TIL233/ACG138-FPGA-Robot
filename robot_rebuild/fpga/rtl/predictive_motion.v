`timescale 1ns/1ps
// Finite-control-set receding-horizon supervisor, outside the existing wheel PI.
// Three lanes evaluate nine bounded constant-input trajectories in three batches.
// Q8 wheel velocity; Q8 centidegree heading error. No runtime matrix inversion.
module rb_predictive_motion #(
 parameter HORIZON=8, ALPHA_L_SHIFT=2, ALPHA_R_SHIFT=2,
 parameter SLEW_COUNTS=2, YAW_GAIN_Q8=0, MOVE_WEIGHT=32,
 parameter MAX_TARGET=80, MAX_VELOCITY=512, MAX_CYCLES=1024,
 parameter MODEL_VALIDATED=0
)(
 input clk,rst_n,enabled,brake,healthy,clear_fault,sample,
 input heading_valid,input signed [15:0] heading_error,
 input signed [15:0] reference_l,reference_r,velocity_l,velocity_r,input [31:0] sample_time,
 output reg signed [15:0] output_l,output_r,
 output reg busy,fault,result_valid,output reg [15:0] generation,solve_cycles,
 output reg [3:0] selected,output reg [31:0] cost,
 output reg signed [15:0] predicted_error,output [15:0] flags,
 output reg [31:0] result_time,output reg signed [15:0] sampled_l,sampled_r
);
 reg [3:0] phase;
 reg [1:0] batch;
 reg [7:0] step;
 reg [15:0] elapsed;
 reg signed [15:0] ref_l,ref_r,base_l,base_r,chosen_l,chosen_r,start_vl,start_vr,start_error;
 reg use_heading;
 reg [31:0] time_hold;
 reg signed [15:0] candidate_l[0:2],candidate_r[0:2];
 reg signed [19:0] model_l[0:2],model_r[0:2],delta_l[0:2],delta_r[0:2];
 reg signed [15:0] err_l[0:2],err_r[0:2],angle_error[0:2];
 reg signed [31:0] square_l[0:2],square_r[0:2],square_angle[0:2];
 reg [32:0] speed_cost[0:2];
 reg signed [47:0] yaw_product[0:2];
 reg signed [31:0] angle_q8[0:2];
 reg [47:0] lane_cost[0:2],best_cost;
 reg signed [15:0] best_l,best_r,best_error;
 reg [3:0] best_index;
 integer i;
 function signed [15:0] clamp;
  input signed [31:0] value;input signed [15:0] ref_value;
  begin
   if(ref_value>0)clamp=value<0?0:value>ref_value?ref_value:value;
   else if(ref_value<0)clamp=value>0?0:value<ref_value?ref_value:value;
   else clamp=0;
  end
 endfunction
 function signed [15:0] offset;
  input signed [15:0] base,ref_value;input [1:0] slot;
  reg signed [31:0] value;
  reg signed [16:0] narrow_value,narrow_ref;
  begin
   if(SLEW_COUNTS==2)begin
    // Every signed16 base +/- 2 fits signed17, including both extrema.
    narrow_value=$signed({base[15],base})+(slot==0?-17'sd2:slot==2?17'sd2:17'sd0);
    narrow_ref=$signed({ref_value[15],ref_value});
    if(narrow_ref>17'sd0)offset=narrow_value<17'sd0 ? 16'sd0 :
      narrow_value>narrow_ref?ref_value:narrow_value[15:0];
    else if(narrow_ref<17'sd0)offset=narrow_value>17'sd0 ? 16'sd0 :
      narrow_value<narrow_ref?ref_value:narrow_value[15:0];
    else offset=0;
   end else begin
    value=$signed(base)+(slot==0?-SLEW_COUNTS:slot==2?SLEW_COUNTS:0);
    offset=clamp(value,ref_value);
   end
  end
 endfunction
 function signed [31:0] wrap_q8;
  input signed [47:0] angle;
  begin
   if(angle>48'sd4608000)wrap_q8=angle-48'sd9216000;
   else if(angle< -48'sd4608000)wrap_q8=angle+48'sd9216000;
   else wrap_q8=angle;
  end
 endfunction
 function [47:0] move_cost;
  input signed [15:0] l,r,bl,br;
  // Both candidates are within SLEW_COUNTS (<=8) of the bounded base.
  // Keep this arithmetic narrow instead of inferring 32-bit multipliers.
  reg signed [5:0] dl,dr;reg [11:0] sl,sr;
  begin
   dl=$signed(l)-$signed(bl);dr=$signed(r)-$signed(br);
   sl=dl*dl;sr=dr*dr;move_cost=({1'b0,sl}+{1'b0,sr})*MOVE_WEIGHT;
  end
 endfunction
 wire bad_input=reference_l>MAX_TARGET || reference_l< -MAX_TARGET ||
                reference_r>MAX_TARGET || reference_r< -MAX_TARGET ||
                velocity_l>MAX_VELOCITY || velocity_l< -MAX_VELOCITY ||
                velocity_r>MAX_VELOCITY || velocity_r< -MAX_VELOCITY ||
                (heading_valid && (heading_error>18000 || heading_error< -18000));
 wire idle_request=(reference_l==0 && reference_r==0);
 wire model_parameters_ok=HORIZON>=1 && HORIZON<=16 && ALPHA_L_SHIFT>=0 && ALPHA_L_SHIFT<=5 &&
                          ALPHA_R_SHIFT>=0 && ALPHA_R_SHIFT<=5 && YAW_GAIN_Q8>=0 && YAW_GAIN_Q8<=4096 &&
                          SLEW_COUNTS>=1 && SLEW_COUNTS<=8;
 assign flags={7'd0,(cost==32'hffffffff),(MODEL_VALIDATED!=0),1'b1,
               (MODEL_VALIDATED==0),(YAW_GAIN_Q8>0),fault,busy,result_valid,(enabled && model_parameters_ok)};
 // STOP and sign/cap changes bypass the arithmetic pipeline. A new reverse
 // request cannot retain old-direction torque while a new result is pending.
 always @*begin
  output_l=0;output_r=0;
  if(!enabled)begin output_l=reference_l;output_r=reference_r;end
  else if(rst_n && !brake && healthy && !fault && !bad_input && result_valid)begin
   output_l=clamp(chosen_l,reference_l);output_r=clamp(chosen_r,reference_r);
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   phase<=0;batch<=0;step<=0;elapsed<=0;busy<=0;fault<=0;result_valid<=0;
   generation<=0;solve_cycles<=0;selected<=0;cost<=0;predicted_error<=0;
   time_hold<=0;result_time<=0;sampled_l<=0;sampled_r<=0;
   ref_l<=0;ref_r<=0;base_l<=0;base_r<=0;chosen_l<=0;chosen_r<=0;
   start_vl<=0;start_vr<=0;start_error<=0;use_heading<=0;
   best_cost<={48{1'b1}};best_l<=0;best_r<=0;best_error<=0;best_index<=0;
   for(i=0;i<3;i=i+1)begin
    candidate_l[i]<=0;candidate_r[i]<=0;model_l[i]<=0;model_r[i]<=0;delta_l[i]<=0;delta_r[i]<=0;
    err_l[i]<=0;err_r[i]<=0;angle_error[i]<=0;square_l[i]<=0;square_r[i]<=0;square_angle[i]<=0;
    speed_cost[i]<=0;yaw_product[i]<=0;angle_q8[i]<=0;lane_cost[i]<=0;
   end
  end else begin
   if(clear_fault)fault<=0;
   if(busy)elapsed<=elapsed+1'b1;
   if(!enabled || brake || !healthy || idle_request || clear_fault)begin
    phase<=0;busy<=0;result_valid<=0;chosen_l<=0;chosen_r<=0;
   end else if(fault)begin phase<=0;busy<=0;result_valid<=0;end
   else if(!model_parameters_ok || bad_input || (sample && busy) || (busy && elapsed>=MAX_CYCLES))begin
    fault<=1;phase<=0;busy<=0;result_valid<=0;
   end else if(busy && (reference_l!=ref_l || reference_r!=ref_r))begin
    // Discard a trajectory scored against a superseded command. Keep only
    // the old bounded output until the next coherent feedback sample.
    phase<=0;busy<=0;
   end else if(sample)begin
    ref_l<=reference_l;ref_r<=reference_r;
    base_l<=clamp(chosen_l,reference_l);base_r<=clamp(chosen_r,reference_r);
    start_vl<=velocity_l;start_vr<=velocity_r;start_error<=heading_error;
    time_hold<=sample_time;
    use_heading<=heading_valid && YAW_GAIN_Q8>0;
    best_cost<={48{1'b1}};best_l<=0;best_r<=0;best_error<=0;best_index<=0;
    batch<=0;step<=0;elapsed<=0;busy<=1;phase<=1;
   end else case(phase)
    1:begin
     for(i=0;i<3;i=i+1)begin
      candidate_l[i]<=offset(base_l,ref_l,i);
      candidate_r[i]<=offset(base_r,ref_r,batch);
      model_l[i]<=$signed(start_vl)*256;model_r[i]<=$signed(start_vr)*256;
      angle_q8[i]<=use_heading?$signed(start_error)*256:0;
     end
     step<=0;phase<=2;
    end
    2:begin
     for(i=0;i<3;i=i+1)begin
      // Initialize once per batch, after candidate registers have settled.
      // Later horizon steps retain the accumulated cost; first use is phase7.
      if(step==0)lane_cost[i]<=move_cost(candidate_l[i],candidate_r[i],base_l,base_r);
      delta_l[i]<=$signed(candidate_l[i])*256-$signed(model_l[i]);
      delta_r[i]<=$signed(candidate_r[i])*256-$signed(model_r[i]);
     end
     phase<=3;
    end
    3:begin
     for(i=0;i<3;i=i+1)begin
      model_l[i]<=model_l[i]+(delta_l[i]>>>ALPHA_L_SHIFT);
      model_r[i]<=model_r[i]+(delta_r[i]>>>ALPHA_R_SHIFT);
     end
     phase<=4;
    end
    4:begin
     for(i=0;i<3;i=i+1)begin
      err_l[i]<=($signed(ref_l)*256-$signed(model_l[i]))>>>4;
      err_r[i]<=($signed(ref_r)*256-$signed(model_r[i]))>>>4;
      yaw_product[i]<=($signed({model_r[i][19],model_r[i]})-$signed({model_l[i][19],model_l[i]}))*YAW_GAIN_Q8;
     end
     phase<=5;
    end
    5:begin
     for(i=0;i<3;i=i+1)begin
      square_l[i]<=$signed(err_l[i])*$signed(err_l[i]);
      square_r[i]<=$signed(err_r[i])*$signed(err_r[i]);
      angle_q8[i]<=use_heading?wrap_q8($signed(angle_q8[i])-(yaw_product[i]>>>8)):0;
     end
     phase<=6;
    end
    6:begin
     for(i=0;i<3;i=i+1)begin
      speed_cost[i]<={1'b0,square_l[i]}+{1'b0,square_r[i]};
      angle_error[i]<=angle_q8[i]>>>8;
     end
     phase<=7;
    end
    7:begin
     for(i=0;i<3;i=i+1)begin
      square_angle[i]<=$signed(angle_error[i])*$signed(angle_error[i]);
      lane_cost[i]<=lane_cost[i]+speed_cost[i];
     end
     phase<=8;
    end
    8:begin
     for(i=0;i<3;i=i+1)lane_cost[i]<=lane_cost[i]+(use_heading?{16'd0,square_angle[i]}:48'd0);
     if(step==HORIZON-1)phase<=9;else begin step<=step+1'b1;phase<=2;end
    end
    9,10,11:begin
     if(lane_cost[phase-9]<best_cost)begin
      best_cost<=lane_cost[phase-9];best_l<=candidate_l[phase-9];best_r<=candidate_r[phase-9];
      best_error<=angle_error[phase-9];best_index<=batch*3+phase-9;
     end
     if(phase==11)begin
      if(batch==2)phase<=12;else begin batch<=batch+1'b1;phase<=1;end
     end else phase<=phase+1'b1;
    end
    12:begin
     chosen_l<=best_l;chosen_r<=best_r;selected<=best_index;predicted_error<=best_error;
     cost<=best_cost>48'h0000ffffffff ? 32'hffffffff : best_cost[31:0];
     solve_cycles<=elapsed;generation<=generation+1'b1;result_valid<=1;busy<=0;phase<=0;
     result_time<=time_hold;sampled_l<=ref_l;sampled_r<=ref_r;
    end
    default:;
   endcase
  end
 end
endmodule

