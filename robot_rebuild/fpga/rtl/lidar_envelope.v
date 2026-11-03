// Conservative dynamic stopping envelope. Constants are nominal, not calibrated.
// Shared shift/add multiplication and restoring division bound latency to 443
// clocks with no combinational divider or multiplier DSP requirement.
module rb_lidar_envelope #(
 parameter CLK_HZ=50000000, COUNTS_PER_M=7640, DECEL_MM_S2=200,
 parameter INPUT_BUDGET_MS=300, CONTROL_BUDGET_MS=20,
 parameter BASE_STOP_MM=200, SLOW_EXTRA_MM=200, USE_REGISTERED_COUNTS=0
)(
 input clk,rst_n,sample,enabled,follow_mode,front_valid,
 input [15:0] front_mm,age_ms,
 input signed [15:0] velocity_l,velocity_r,request_l,request_r,
 output valid,hold,output signed [15:0] limited_l,limited_r,
 output reg [15:0] stop_mm,slow_mm,speed_mm_s,latency_ms,solve_cycles,
 output config_ok,
 // Internal pipeline contract: when selected, the caller validates this
 // unsigned upper bound against both raw wheel velocities and requests from
 // the SAME snapshot before asserting valid. Default mode ignores both ports.
 input [15:0] registered_counts,input registered_counts_valid,
 // Raw cap of the atomic publication; meaningful only while valid is high.
 output reg [7:0] published_cap
);
 assign config_ok=CLK_HZ>0 && COUNTS_PER_M>0 && DECEL_MM_S2>0 &&
  COUNTS_PER_M<=1000000000 && DECEL_MM_S2<=1000000000 &&
  INPUT_BUDGET_MS>=0 && INPUT_BUDGET_MS<=65535 && CONTROL_BUDGET_MS>=0 &&
  CONTROL_BUDGET_MS<=65535 && BASE_STOP_MM>=200 && BASE_STOP_MM<=65535 &&
  SLOW_EXTRA_MM>0 && SLOW_EXTRA_MM<=65535;
 wire forward_request=!request_l[15] && !request_r[15] && (request_l!=0 || request_r!=0);
 wire active=enabled && front_valid && forward_request && config_ok &&
             (USE_REGISTERED_COUNTS==0 || registered_counts_valid);
 reg running,result_q,hold_q,follow_q;
 reg signed [15:0] request_l_q,request_r_q,limited_l_q,limited_r_q;
 reg [15:0] front_q,age_q,counts_q;reg [31:0] latency_q,speed_q;
 // Published keys and values remain independent of the background solver.
 // Only operation 4 replaces the complete publication.
 reg published_follow;
 reg signed [15:0] published_request_l,published_request_r;
 reg [15:0] published_front,published_age,published_counts;
 reg [16:0] work_age_limit,published_age_limit;
 reg work_hold;
 reg signed [15:0] work_limited_l;
 reg [15:0] work_stop,work_slow,work_speed,work_latency,work_cycles;
 reg [63:0] reaction_q;reg [15:0] margin_q;
 reg [2:0] operation;reg [6:0] remaining;
 localparam ARITHMETIC=3'd0,ROUND_RESULT=3'd1,USE_RESULT=3'd2,
  DISTANCE_RESULT=3'd3,MARGIN_RESULT=3'd4,LOAD_SCALE=3'd5;
 reg [2:0] phase;
 reg [63:0] ceil_q,distance_sum_q;
 reg [63:0] dividend,divisor,quotient,remainder;
 reg [63:0] multiplicand;reg [31:0] multiplier;reg [5:0] multiply_left;
 wire [63:0] product_next=dividend+(multiplier[0] ? multiplicand : 64'd0);
 wire [64:0] shifted={remainder,dividend[63]};
 wire subtract=shifted>={1'b0,divisor};
 wire [63:0] next_rem=subtract ? shifted-{1'b0,divisor} : shifted[63:0];
 wire [63:0] next_quot={quotient[62:0],subtract};
 // The last restoring step, rounding, distance and scaling loads occupy
 // distinct clocks. No final-bit subtraction feeds the next multiplier.
 wire [63:0] ceil_quot=quotient+(remainder!=0);
 wire [15:0] maximum_counts;
 // A rising speed bound revokes immediately. Natural age progress is covered
 // only for the stated control budget; age rollback identifies a new snapshot.
 wire work_changed=request_l!=request_l_q || request_r!=request_r_q || follow_mode!=follow_q ||
  front_mm!=front_q || maximum_counts>counts_q || age_ms<age_q ||
  {1'b0,age_ms}>work_age_limit;
 wire published_changed=request_l!=published_request_l || request_r!=published_request_r ||
  follow_mode!=published_follow || front_mm!=published_front || maximum_counts>published_counts ||
  age_ms<published_age || {1'b0,age_ms}>published_age_limit;
 assign valid=result_q && active && !published_changed;
 assign hold=valid && hold_q;
 assign limited_l=valid ? limited_l_q : request_l;
 assign limited_r=valid ? limited_r_q : request_r;
 function [15:0] magnitude;
  input signed [15:0] value;
  begin magnitude=value[15] ? (~value+16'd1) : value;end
 endfunction
 wire [15:0] abs_l=magnitude(velocity_l),abs_r=magnitude(velocity_r);
 wire [15:0] measured=abs_l>abs_r ? abs_l : abs_r;
 wire [15:0] requested=request_l>request_r ? request_l : request_r;
 assign maximum_counts=USE_REGISTERED_COUNTS!=0 ? registered_counts :
                       (measured>requested ? measured : requested);
 wire [63:0] floor_distance=follow_q && BASE_STOP_MM<600 ? 64'd600 : BASE_STOP_MM;
 wire [63:0] distance_full=distance_sum_q>floor_distance ? distance_sum_q : floor_distance;
 wire [15:0] distance_sat=distance_full>65535 ? 16'hffff : distance_full[15:0];
 // Both scaled values are nonnegative for an active forward request.
 wire [15:0] publication_max=work_limited_l>$signed(quotient[15:0])?work_limited_l:quotient[15:0];
 wire [31:0] slow_full={16'd0,work_stop}+SLOW_EXTRA_MM;
 wire [15:0] margin_next=front_q<=work_stop ? 16'd0 :
  ({16'd0,front_q}>=slow_full ? SLOW_EXTRA_MM : front_q-work_stop);
 task multiply_divide;
  input [63:0] factor_a,denominator;
  input [31:0] factor_b;
  input [5:0] multiply_cycles;
  begin
   multiplicand<=factor_a;multiplier<=factor_b;multiply_left<=multiply_cycles;
   dividend<=0;divisor<=denominator;quotient<=0;remainder<=0;remaining<=64;
  end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   running<=0;result_q<=0;hold_q<=0;follow_q<=0;request_l_q<=0;request_r_q<=0;
   limited_l_q<=0;limited_r_q<=0;front_q<=0;age_q<=0;counts_q<=0;latency_q<=0;speed_q<=0;reaction_q<=0;
   margin_q<=0;operation<=0;phase<=ARITHMETIC;ceil_q<=0;distance_sum_q<=0;
   remaining<=0;dividend<=0;divisor<=1;quotient<=0;remainder<=0;
   multiplicand<=0;multiplier<=0;multiply_left<=0;
   stop_mm<=0;slow_mm<=0;speed_mm_s<=0;latency_ms<=0;solve_cycles<=0;
   published_follow<=0;published_request_l<=0;published_request_r<=0;
   published_front<=0;published_age<=0;published_counts<=0;
   work_age_limit<=0;published_age_limit<=0;published_cap<=0;
   work_hold<=0;work_limited_l<=0;work_stop<=0;work_slow<=0;
   work_speed<=0;work_latency<=0;work_cycles<=0;
  end else begin
   // Revocation is sticky: restoring an old input cannot revive old output.
   // A still-valid result survives a normal sample and all partial arithmetic.
   if(published_changed)result_q<=0;
   if(!active)begin
   running<=0;result_q<=0;hold_q<=0;
  end else if(sample)begin
   running<=1;operation<=0;phase<=ARITHMETIC;work_cycles<=0;
   request_l_q<=request_l;request_r_q<=request_r;front_q<=front_mm;follow_q<=follow_mode;
   age_q<=age_ms;counts_q<=maximum_counts;
   // Legal age16 + control16 fits 17 bits. Keep this addition off valid's
   // combinational path and publish it with exactly the same snapshot keys.
   work_age_limit<={1'b0,age_ms}+CONTROL_BUDGET_MS;
   latency_q<=INPUT_BUDGET_MS+CONTROL_BUDGET_MS+{16'd0,age_ms};
   if(INPUT_BUDGET_MS+CONTROL_BUDGET_MS+{16'd0,age_ms}>65535)work_latency<=65535;
   else work_latency<=INPUT_BUDGET_MS+CONTROL_BUDGET_MS+{16'd0,age_ms};
   multiply_divide(64'd50000,COUNTS_PER_M,{16'd0,maximum_counts},6'd16);
  end else if(running && work_changed)begin
   running<=0;
  end else if(running)begin
   work_cycles<=work_cycles+1'b1;
   if(work_cycles>=511)begin running<=0;result_q<=0;hold_q<=0;end
   else if(phase==ARITHMETIC && multiply_left!=0)begin
    dividend<=product_next;multiplicand<={multiplicand[62:0],1'b0};
    multiplier<={1'b0,multiplier[31:1]};multiply_left<=multiply_left-1'b1;
   end else if(phase==ARITHMETIC && remaining!=0)begin
    dividend<={dividend[62:0],1'b0};remainder<=next_rem;quotient<=next_quot;
    remaining<=remaining-1'b1;
    if(remaining==1)phase<=ROUND_RESULT;
   end else if(phase==ROUND_RESULT)begin
    if(operation==3)begin
     // Speed scaling rounds DOWN, never increasing either requested wheel.
     work_limited_l<=quotient[15:0];
     multiply_divide({48'd0,request_r_q},SLOW_EXTRA_MM,{16'd0,margin_q},6'd16);
     operation<=4;phase<=ARITHMETIC;
    end else if(operation==4)begin
     limited_l_q<=work_limited_l;limited_r_q<=quotient[15:0];hold_q<=work_hold;
     stop_mm<=work_stop;slow_mm<=work_slow;speed_mm_s<=work_speed;latency_ms<=work_latency;
     solve_cycles<=work_cycles+1'b1;
     published_request_l<=request_l_q;published_request_r<=request_r_q;published_follow<=follow_q;
     published_front<=front_q;published_age<=age_q;published_counts<=counts_q;
     published_age_limit<=work_age_limit;
     published_cap<=publication_max>255?8'hff:publication_max[7:0];
     running<=0;result_q<=1;
    end else begin
     ceil_q<=ceil_quot;phase<=USE_RESULT;
    end
   end else if(phase==USE_RESULT)begin
     case(operation)
      0:begin
       speed_q<=ceil_q[31:0];work_speed<=ceil_q>65535 ? 16'hffff : ceil_q[15:0];
       multiply_divide(ceil_q,64'd1000,latency_q,6'd32);operation<=1;phase<=ARITHMETIC;
      end
      1:begin
       reaction_q<=ceil_q;
       multiply_divide({32'd0,speed_q},2*DECEL_MM_S2,speed_q,6'd32);operation<=2;phase<=ARITHMETIC;
      end
      2:begin
       distance_sum_q<=64'd200+reaction_q+ceil_q;phase<=DISTANCE_RESULT;
      end
      default:begin running<=0;result_q<=0;hold_q<=0;end
     endcase
   end else if(phase==DISTANCE_RESULT)begin
    work_stop<=distance_sat;work_hold<=front_q<=distance_sat;phase<=MARGIN_RESULT;
   end else if(phase==MARGIN_RESULT)begin
    work_slow<=slow_full>65535 ? 16'hffff : slow_full[15:0];
    margin_q<=margin_next;phase<=LOAD_SCALE;
   end else if(phase==LOAD_SCALE)begin
    multiply_divide({48'd0,request_l_q},SLOW_EXTRA_MM,{16'd0,margin_q},6'd16);
    operation<=3;phase<=ARITHMETIC;
   end
  end
  end
 end
endmodule
