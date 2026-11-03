// Three parallel angular comparisons, fixed nominal swept-disc corridors.
// No measured vehicle dynamics, motion compensation, or moving-object prediction.
module rb_lidar_lattice #(
 parameter COUNTS_PER_M=7640, WHEELBASE_MM=100, ARC_RADIUS_MM=150,
 parameter PATH_LENGTH_MM=300, SEGMENTS=8, BODY_RADIUS_MM=120,
 parameter RANGE_ERROR_MM=20, SEGMENT_MARGIN_MM=20,
 parameter INPUT_BUDGET_MS=300, MAX_MAP_AGE_MS=350,
 parameter CONTROL_BUDGET_MS=20, DECEL_MM_S2=200
)(
 input clk,rst_n,start,cancel,map_fresh,
 input [575:0] map_ranges,input [35:0] map_known,
 input [15:0] map_scan_id,map_age_ms,
 input [7:0] request_max,input [15:0] measured_max_counts,
 input signed [15:0] goal_cd,input [15:0] command_epoch,
 input [3:0] previous_candidate,
 output reg busy,result_valid,no_path,fault,output reg [3:0] selected,
 output reg signed [15:0] target_l,target_r,
 output reg [8:0] admissible_mask,unknown_mask,
 output reg [15:0] clearance_mm,solve_cycles,result_scan_id,result_epoch
);
 localparam CONFIG_OK=COUNTS_PER_M==7640 && WHEELBASE_MM==100 && ARC_RADIUS_MM==150 &&
  PATH_LENGTH_MM==300 && SEGMENTS==8 && BODY_RADIUS_MM==120 && RANGE_ERROR_MM==20 &&
  SEGMENT_MARGIN_MM==20 && INPUT_BUDGET_MS==300 && MAX_MAP_AGE_MS==350 &&
  CONTROL_BUDGET_MS==20 && DECEL_MM_S2==200;
 reg [575:0] ranges_q;reg [35:0] known_q;
 reg [15:0] scan_q,epoch_q,measured_q;reg [7:0] request_q;
 reg signed [15:0] goal_q;reg [3:0] previous_q;
 localparam SCAN=3'd0,SCORE_ERROR=3'd1,SCORE_WRAP=3'd2,
  SCORE_ABS=3'd3,SCORE_COST=3'd4,SCORE_BEST=3'd5,PUBLISH=3'd6,SCAN_DRAIN=3'd7;
 reg [1:0] group_q,lane_score;reg [2:0] phase;reg [5:0] sector;
 reg [2:0] lane_ok,lane_unknown;reg [15:0] lane_clear[0:2];
 // One sector is issued every scan clock. Preprocess the ROM/map comparison
 // and difference before the following clock updates each lane's minimum.
 // A single drain consumes sector 35 before scoring or changing groups.
 reg sector_valid_q,sector_known_q;
 reg [2:0] sector_needed_q,sector_below_q;
 reg [15:0] sector_clear_q[0:2];
 reg [31:0] best_cost;reg best_valid;reg [3:0] best_id;
 reg [15:0] best_clear;
 // Explicit decode avoids a DSP multiplier before the threshold ROM. All
 // four two-bit group states retain the exact original 3*group value.
 wire [3:0] base_id=group_q==0 ? 4'd0 : group_q==1 ? 4'd3 :
                    group_q==2 ? 4'd6 : 4'd9;
 wire [15:0] threshold[0:2];
 rb_lidar_lattice_rom r0(base_id,sector,threshold[0]);
 rb_lidar_lattice_rom r1(base_id+4'd1,sector,threshold[1]);
 rb_lidar_lattice_rom r2(base_id+4'd2,sector,threshold[2]);
 wire [7:0] group_speed=group_q==0 ? 8'd2 : group_q==1 ? 8'd6 : 8'd12;
 wire [15:0] range_now=ranges_q[sector*16+:16];
 wire [3:0] score_id=base_id+{2'b0,lane_score};
 wire stale=map_scan_id!=scan_q || command_epoch!=epoch_q || request_max!=request_q ||
  goal_cd!=goal_q || measured_max_counts!=measured_q;
 // Registered score stages keep heading wrap/abs, cost additions and best
 // selection apart. The group and lane stay fixed until SCORE_BEST consumes
 // this score, so no candidate can use another group's partial arithmetic.
 reg signed [31:0] error_q,wrapped_q;
 reg [31:0] angle_cost_q,penalty_q,clear_penalty_q,secondary_cost_q,cost_q;
 wire signed [31:0] heading=lane_score==0 ? 32'sd11459 : lane_score==1 ? 32'sd0 : -32'sd11459;
 wire [31:0] speed_penalty=group_q==0 ? 32'd20000 : group_q==1 ? 32'd12000 : 32'd0;
 wire better=!best_valid || cost_q<best_cost ||
  (cost_q==best_cost && ((lane_score==1 && best_id%3!=1) ||
   ((lane_score==1)==(best_id%3==1) && score_id<best_id)));
 integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   busy<=0;result_valid<=0;no_path<=0;fault<=0;selected<=15;
   target_l<=0;target_r<=0;admissible_mask<=0;unknown_mask<=0;
   clearance_mm<=0;solve_cycles<=0;result_scan_id<=0;result_epoch<=0;
   scan_q<=0;epoch_q<=0;request_q<=0;measured_q<=0;goal_q<=0;previous_q<=15;
   ranges_q<=0;known_q<=0;group_q<=0;phase<=0;sector<=0;lane_score<=0;
   lane_ok<=0;lane_unknown<=0;best_cost<=0;best_valid<=0;best_id<=15;best_clear<=0;
   sector_valid_q<=0;sector_known_q<=0;sector_needed_q<=0;sector_below_q<=0;
   error_q<=0;wrapped_q<=0;angle_cost_q<=0;penalty_q<=0;clear_penalty_q<=0;
   secondary_cost_q<=0;cost_q<=0;
   for(i=0;i<3;i=i+1)begin lane_clear[i]<=65535;sector_clear_q[i]<=0;end
  end else if(cancel || !map_fresh || map_age_ms>350 || !CONFIG_OK)begin
   busy<=0;result_valid<=0;selected<=15;target_l<=0;target_r<=0;
   no_path<=0;fault<=!CONFIG_OK;admissible_mask<=0;unknown_mask<=0;clearance_mm<=0;
   sector_valid_q<=0;
  end else if(start)begin
   busy<=1;result_valid<=0;no_path<=0;fault<=0;selected<=15;target_l<=0;target_r<=0;
   admissible_mask<=0;unknown_mask<=0;clearance_mm<=0;solve_cycles<=0;
   ranges_q<=map_ranges;known_q<=map_known;scan_q<=map_scan_id;epoch_q<=command_epoch;
   request_q<=request_max;measured_q<=measured_max_counts;goal_q<=goal_cd;previous_q<=previous_candidate;
   group_q<=0;phase<=0;sector<=0;lane_score<=0;lane_ok<=3'b111;lane_unknown<=0;
   sector_valid_q<=0;
   best_valid<=0;best_id<=15;best_cost<=32'hffffffff;best_clear<=0;
   for(i=0;i<3;i=i+1)lane_clear[i]<=65535;
  end else if((busy || result_valid || no_path) && stale)begin
   busy<=0;result_valid<=0;no_path<=0;selected<=15;target_l<=0;target_r<=0;
   admissible_mask<=0;unknown_mask<=0;clearance_mm<=0;
   sector_valid_q<=0;
  end else if(busy)begin
   solve_cycles<=solve_cycles+1'b1;
   if(solve_cycles>=511)begin
    busy<=0;result_valid<=0;fault<=1;selected<=15;target_l<=0;target_r<=0;
   end else case(phase)
    SCAN,SCAN_DRAIN:begin
     for(i=0;i<3;i=i+1)begin
      if(sector_valid_q && sector_needed_q[i])begin
       if(!sector_known_q)begin lane_ok[i]<=0;lane_unknown[i]<=1;end
       else if(sector_below_q[i])lane_ok[i]<=0;
       else if(sector_clear_q[i]<lane_clear[i])lane_clear[i]<=sector_clear_q[i];
      end
     end
     if(phase==SCAN)begin
      sector_valid_q<=1;sector_known_q<=known_q[sector];
      for(i=0;i<3;i=i+1)begin
       sector_needed_q[i]<=threshold[i]!=0;
       sector_below_q[i]<=range_now<threshold[i];
       sector_clear_q[i]<=range_now-threshold[i];
      end
      if(sector==35)phase<=SCAN_DRAIN;else sector<=sector+1'b1;
     end else begin
      sector_valid_q<=0;phase<=SCORE_ERROR;lane_score<=0;
     end
    end
    SCORE_ERROR:begin
     error_q<=heading-$signed(goal_q);phase<=SCORE_WRAP;
    end
    SCORE_WRAP:begin
     if(error_q>18000)wrapped_q<=error_q-36000;
     else if(error_q< -18000)wrapped_q<=error_q+36000;
     else wrapped_q<=error_q;
     penalty_q<=speed_penalty+((previous_q<9 && previous_q%3!=lane_score) ? 32'd2000 : 32'd0);
     clear_penalty_q<=32'd1000-(lane_clear[lane_score]>1000 ? 32'd1000 : {16'd0,lane_clear[lane_score]});
     phase<=SCORE_ABS;
    end
    SCORE_ABS:begin
     angle_cost_q<=(wrapped_q<0 ? -wrapped_q : wrapped_q)<<3;
     secondary_cost_q<=penalty_q+clear_penalty_q;phase<=SCORE_COST;
    end
    SCORE_COST:begin
     cost_q<=angle_cost_q+secondary_cost_q;phase<=SCORE_BEST;
    end
    SCORE_BEST:begin
     unknown_mask[score_id]<=lane_unknown[lane_score];
     if(lane_ok[lane_score] && group_speed<=request_q && group_speed>=measured_q)begin
      admissible_mask[score_id]<=1;
      if(better)begin best_valid<=1;best_cost<=cost_q;best_id<=score_id;best_clear<=lane_clear[lane_score];end
     end
     if(lane_score==2)begin
      if(group_q==2)phase<=PUBLISH;
      else begin group_q<=group_q+1'b1;sector<=0;phase<=SCAN;lane_ok<=7;lane_unknown<=0;sector_valid_q<=0;
       for(i=0;i<3;i=i+1)lane_clear[i]<=65535;
      end
     end else begin lane_score<=lane_score+1'b1;phase<=SCORE_ERROR;end
    end
    PUBLISH:begin
     busy<=0;result_valid<=best_valid;no_path<=!best_valid;selected<=best_valid ? best_id : 4'd15;
     clearance_mm<=best_valid ? best_clear : 16'd0;result_scan_id<=scan_q;result_epoch<=epoch_q;
     if(best_valid)begin
      case(best_id)
       0:begin target_l<=1;target_r<=2;end
       1:begin target_l<=2;target_r<=2;end
       2:begin target_l<=2;target_r<=1;end
       3:begin target_l<=3;target_r<=6;end
       4:begin target_l<=6;target_r<=6;end
       5:begin target_l<=6;target_r<=3;end
       6:begin target_l<=6;target_r<=12;end
       7:begin target_l<=12;target_r<=12;end
       8:begin target_l<=12;target_r<=6;end
       default:begin target_l<=0;target_r<=0;end
      endcase
     end else begin target_l<=0;target_r<=0;end
    end
    default:begin busy<=0;result_valid<=0;fault<=1;selected<=15;end
   endcase
  end
 end
endmodule
