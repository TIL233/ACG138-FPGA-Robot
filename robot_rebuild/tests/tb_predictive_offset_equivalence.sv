`timescale 1ns/1ps
module tb_predictive_offset_equivalence;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,enabled=1,brake=0,healthy=1,clear_fault=0,sample=0,heading_valid=1;
 reg signed[15:0] heading_error=0,reference_l=12,reference_r=12,velocity_l=0,velocity_r=0;
 reg[31:0] sample_time=0;
 wire [15:0] c_output_l;
 wire [15:0] c_output_r;
 wire [0:0] c_busy;
 wire [0:0] c_fault;
 wire [0:0] c_result_valid;
 wire [15:0] c_generation;
 wire [15:0] c_solve_cycles;
 wire [3:0] c_selected;
 wire [31:0] c_cost;
 wire [15:0] c_predicted_error;
 wire [15:0] c_flags;
 wire [31:0] c_result_time;
 wire [15:0] c_sampled_l;
 wire [15:0] c_sampled_r;
 rb_predictive_motion #(.YAW_GAIN_Q8(256)) c(.clk(clk),.rst_n(rst_n),.enabled(enabled),.brake(brake),.healthy(healthy),.clear_fault(clear_fault),.sample(sample),.heading_valid(heading_valid),.heading_error(heading_error),.reference_l(reference_l),.reference_r(reference_r),.velocity_l(velocity_l),.velocity_r(velocity_r),.sample_time(sample_time),.output_l(c_output_l),.output_r(c_output_r),.busy(c_busy),.fault(c_fault),.result_valid(c_result_valid),.generation(c_generation),.solve_cycles(c_solve_cycles),.selected(c_selected),.cost(c_cost),.predicted_error(c_predicted_error),.flags(c_flags),.result_time(c_result_time),.sampled_l(c_sampled_l),.sampled_r(c_sampled_r));
 wire [15:0] r_output_l;
 wire [15:0] r_output_r;
 wire [0:0] r_busy;
 wire [0:0] r_fault;
 wire [0:0] r_result_valid;
 wire [15:0] r_generation;
 wire [15:0] r_solve_cycles;
 wire [3:0] r_selected;
 wire [31:0] r_cost;
 wire [15:0] r_predicted_error;
 wire [15:0] r_flags;
 wire [31:0] r_result_time;
 wire [15:0] r_sampled_l;
 wire [15:0] r_sampled_r;
 rb_predictive_reference #(.YAW_GAIN_Q8(256)) r(.clk(clk),.rst_n(rst_n),.enabled(enabled),.brake(brake),.healthy(healthy),.clear_fault(clear_fault),.sample(sample),.heading_valid(heading_valid),.heading_error(heading_error),.reference_l(reference_l),.reference_r(reference_r),.velocity_l(velocity_l),.velocity_r(velocity_r),.sample_time(sample_time),.output_l(r_output_l),.output_r(r_output_r),.busy(r_busy),.fault(r_fault),.result_valid(r_result_valid),.generation(r_generation),.solve_cycles(r_solve_cycles),.selected(r_selected),.cost(r_cost),.predicted_error(r_predicted_error),.flags(r_flags),.result_time(r_result_time),.sampled_l(r_sampled_l),.sampled_r(r_sampled_r));
 rb_predictive_motion #(.SLEW_COUNTS(8)) c8(.clk(clk),.rst_n(rst_n),.enabled(enabled),.brake(brake),.healthy(healthy),.clear_fault(clear_fault),.sample(sample),.heading_valid(heading_valid),.heading_error(heading_error),.reference_l(reference_l),.reference_r(reference_r),.velocity_l(velocity_l),.velocity_r(velocity_r),.sample_time(sample_time),.output_l(),.output_r(),.busy(),.fault(),.result_valid(),.generation(),.solve_cycles(),.selected(),.cost(),.predicted_error(),.flags(),.result_time(),.sampled_l(),.sampled_r());
 rb_predictive_reference #(.SLEW_COUNTS(8)) r8(.clk(clk),.rst_n(rst_n),.enabled(enabled),.brake(brake),.healthy(healthy),.clear_fault(clear_fault),.sample(sample),.heading_valid(heading_valid),.heading_error(heading_error),.reference_l(reference_l),.reference_r(reference_r),.velocity_l(velocity_l),.velocity_r(velocity_r),.sample_time(sample_time),.output_l(),.output_r(),.busy(),.fault(),.result_valid(),.generation(),.solve_cycles(),.selected(),.cost(),.predicted_error(),.flags(),.result_time(),.sampled_l(),.sampled_r());
 rb_predictive_motion #(.SLEW_COUNTS(65536)) cw(.clk(clk),.rst_n(rst_n),.enabled(enabled),.brake(brake),.healthy(healthy),.clear_fault(clear_fault),.sample(sample),.heading_valid(heading_valid),.heading_error(heading_error),.reference_l(reference_l),.reference_r(reference_r),.velocity_l(velocity_l),.velocity_r(velocity_r),.sample_time(sample_time),.output_l(),.output_r(),.busy(),.fault(),.result_valid(),.generation(),.solve_cycles(),.selected(),.cost(),.predicted_error(),.flags(),.result_time(),.sampled_l(),.sampled_r());
 rb_predictive_reference #(.SLEW_COUNTS(65536)) rw(.clk(clk),.rst_n(rst_n),.enabled(enabled),.brake(brake),.healthy(healthy),.clear_fault(clear_fault),.sample(sample),.heading_valid(heading_valid),.heading_error(heading_error),.reference_l(reference_l),.reference_r(reference_r),.velocity_l(velocity_l),.velocity_r(velocity_r),.sample_time(sample_time),.output_l(),.output_r(),.busy(),.fault(),.result_valid(),.generation(),.solve_cycles(),.selected(),.cost(),.predicted_error(),.flags(),.result_time(),.sampled_l(),.sampled_r());
 reg cost_retime_pending=0;
 reg [47:0] prior_cost[0:2],expected_move[0:2];
 integer pi;reg do_phase1,do_phase2;
 always @(posedge clk)begin
  do_phase1=0;do_phase2=0;
  if(!rst_n)cost_retime_pending=0;
  else if(enabled && !brake && healthy && !clear_fault && !c.fault &&
      !c.bad_input && c.model_parameters_ok && !sample &&
      !(c.busy && c.elapsed>=c.MAX_CYCLES) &&
      !(c.busy && (reference_l!=c.ref_l || reference_r!=c.ref_r)) &&
      (reference_l!=0 || reference_r!=0))begin
   do_phase1=c.phase==1;do_phase2=c.phase==2 && c.step==0;
   for(pi=0;pi<3;pi=pi+1)begin
    prior_cost[pi]=c.lane_cost[pi];
    expected_move[pi]=r.move_cost(c.candidate_l[pi],c.candidate_r[pi],c.base_l,c.base_r);
   end
   if(do_phase1)cost_retime_pending=1;
   if(do_phase2)cost_retime_pending=0;
  end
  #1;
  for(pi=0;pi<3;pi=pi+1)begin
   if(do_phase1 && c.lane_cost[pi]!==prior_cost[pi])$fatal(1,"phase1 must only register candidates, not move cost");
   if(do_phase2 && c.lane_cost[pi]!==expected_move[pi])$fatal(1,"phase2 initial movement cost differs");
  end
 end
 integer b,sl,k,j,n,v,rv;reg signed[15:0] fb,fr;reg[1:0] fs;
 task check_state;
 begin
 if({c_output_l,c_output_r,c_busy,c_fault,c_result_valid,c_generation,c_solve_cycles,c_selected,c_cost,c_predicted_error,c_flags,c_result_time,c_sampled_l,c_sampled_r}!=={r_output_l,r_output_r,r_busy,r_fault,r_result_valid,r_generation,r_solve_cycles,r_selected,r_cost,r_predicted_error,r_flags,r_result_time,r_sampled_l,r_sampled_r})$fatal(1,"output mismatch");
 if(c.phase!==r.phase)$fatal(1,"state phase");
 if(c.batch!==r.batch)$fatal(1,"state batch");
 if(c.step!==r.step)$fatal(1,"state step");
 if(c.elapsed!==r.elapsed)$fatal(1,"state elapsed");
 if(c.ref_l!==r.ref_l)$fatal(1,"state ref_l");
 if(c.ref_r!==r.ref_r)$fatal(1,"state ref_r");
 if(c.base_l!==r.base_l)$fatal(1,"state base_l");
 if(c.base_r!==r.base_r)$fatal(1,"state base_r");
 if(c.chosen_l!==r.chosen_l)$fatal(1,"state chosen_l");
 if(c.chosen_r!==r.chosen_r)$fatal(1,"state chosen_r");
 if(c.start_vl!==r.start_vl)$fatal(1,"state start_vl");
 if(c.start_vr!==r.start_vr)$fatal(1,"state start_vr");
 if(c.start_error!==r.start_error)$fatal(1,"state start_error");
 if(c.use_heading!==r.use_heading)$fatal(1,"state use_heading");
 if(c.time_hold!==r.time_hold)$fatal(1,"state time_hold");
 if(c.best_cost!==r.best_cost)$fatal(1,"state best_cost");
 if(c.best_l!==r.best_l)$fatal(1,"state best_l");
 if(c.best_r!==r.best_r)$fatal(1,"state best_r");
 if(c.best_error!==r.best_error)$fatal(1,"state best_error");
 if(c.best_index!==r.best_index)$fatal(1,"state best_index");
 for(j=0;j<3;j=j+1)begin
 if(c.candidate_l[j]!==r.candidate_l[j])$fatal(1,"array candidate_l");
 if(c.candidate_r[j]!==r.candidate_r[j])$fatal(1,"array candidate_r");
 if(c.model_l[j]!==r.model_l[j])$fatal(1,"array model_l");
 if(c.model_r[j]!==r.model_r[j])$fatal(1,"array model_r");
 if(c.delta_l[j]!==r.delta_l[j])$fatal(1,"array delta_l");
 if(c.delta_r[j]!==r.delta_r[j])$fatal(1,"array delta_r");
 if(c.err_l[j]!==r.err_l[j])$fatal(1,"array err_l");
 if(c.err_r[j]!==r.err_r[j])$fatal(1,"array err_r");
 if(c.angle_error[j]!==r.angle_error[j])$fatal(1,"array angle_error");
 if(c.square_l[j]!==r.square_l[j])$fatal(1,"array square_l");
 if(c.square_r[j]!==r.square_r[j])$fatal(1,"array square_r");
 if(c.square_angle[j]!==r.square_angle[j])$fatal(1,"array square_angle");
 if(c.speed_cost[j]!==r.speed_cost[j])$fatal(1,"array speed_cost");
 if(c.yaw_product[j]!==r.yaw_product[j])$fatal(1,"array yaw_product");
 if(c.angle_q8[j]!==r.angle_q8[j])$fatal(1,"array angle_q8");
 if(!cost_retime_pending && c.lane_cost[j]!==r.lane_cost[j])$fatal(1,"array lane_cost");
 end
 end endtask
 initial begin
  // Full base domain; all slots (including unused slot 3), every comparison
  // partition boundary for reference: extrema, zero/sign and value +/- 1.
  for(b=-32768;b<=32767;b=b+1)for(sl=0;sl<4;sl=sl+1)begin
   fb=b;fs=sl;v=b+(sl==0?-2:sl==2?2:0);
   for(k=0;k<8;k=k+1)begin
    case(k)0:rv=-32768;1:rv=32767;2:rv=0;3:rv=-1;4:rv=1;5:rv=v-1;6:rv=v;7:rv=v+1;endcase
    if(rv>=-32768 && rv<=32767)begin
     fr=rv;#1;if(c.offset(fb,fr,fs)!==r.offset(fb,fr,fs))$fatal(1,"offset b=%0d ref=%0d slot=%0d",b,rv,sl);
     if(c8.offset(fb,fr,fs)!==r8.offset(fb,fr,fs) || cw.offset(fb,fr,fs)!==rw.offset(fb,fr,fs))$fatal(1,"fallback offset");
    end
   end
  end
  #20;rst_n=1;
  // Completed trajectories, sign changes, cap reductions, concurrent sample,
  // cancellation, unhealthy, enable bypass and invalid full-width inputs.
  for(n=0;n<500;n=n+1)begin
   @(negedge clk);clear_fault=1;brake=1;sample=0;@(negedge clk);clear_fault=0;brake=0;
   reference_l=($random%81);reference_r=($random%81);
   velocity_l=($random%513);velocity_r=($random%513);heading_error=($random%18001);sample_time=n;
   sample=1;@(negedge clk);sample=0;
   repeat(500)begin
    @(negedge clk);
    if(n%7==0 && c.elapsed==15)reference_l=-reference_l;
    if(n%11==0 && c.elapsed==16)reference_r=reference_r/2;
    if(n%13==0 && c.elapsed==17)sample=1;else sample=0;
    @(posedge clk);#1;check_state();
   end
  end
  for(n=0;n<10000;n=n+1)begin
   @(negedge clk);reference_l=$random;reference_r=$random;velocity_l=$random;velocity_r=$random;
   enabled=n%7!=0;healthy=n%11!=0;brake=n%13==0;sample=n%3==0;clear_fault=n%5==0;heading_error=$random;
   @(posedge clk);#1;check_state();
  end
  $display("PASS offset full base/all slots/complete reference partitions; full state+output 500 trajectories and 10000 invalid/cancel cycles");$finish;
 end
endmodule

// Frozen pre-optimization behavioral reference, independent of the candidate.
`timescale 1ns/1ps
// Finite-control-set receding-horizon supervisor, outside the existing wheel PI.
// Three lanes evaluate nine bounded constant-input trajectories in three batches.
// Q8 wheel velocity; Q8 centidegree heading error. No runtime matrix inversion.
module rb_predictive_reference #(
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
  begin value=$signed(base)+(slot==0?-SLEW_COUNTS:slot==2?SLEW_COUNTS:0);offset=clamp(value,ref_value);end
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
      lane_cost[i]<=move_cost(offset(base_l,ref_l,i),offset(base_r,ref_r,batch),base_l,base_r);
     end
     step<=0;phase<=2;
    end
    2:begin
     for(i=0;i<3;i=i+1)begin
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
