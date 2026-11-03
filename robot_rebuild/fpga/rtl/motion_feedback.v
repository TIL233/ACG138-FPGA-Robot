`timescale 1ns/1ps
// One coherent 20ms encoder/IMU snapshot, entirely in the FPGA clock domain.
// TRACK_MM=0 deliberately disables encoder prediction until chassis calibration.
// YAW_SIGN describes the IMU convention, not the encoder wiring convention.
module rb_motion_feedback #(
 parameter CLK_HZ=50000000, CONTROL_HZ=50, COUNTS_PER_M=7640,
 parameter TRACK_MM=0, YAW_SIGN=1, CORRECTION_SHIFT=2, FUSE_ENCODER=1,
 parameter MAX_CORRECTION_CD=200, MAX_DELTA_COUNTS=512,
 parameter MAX_YAW_STEP_CD=3000, IMU_TIMEOUT_MS=200,
 parameter DISAGREE_CD=500, DISAGREE_WINDOWS=10
)(
 input clk,rst_n,clear_fault,imu_ok,imu_update,
 input signed [31:0] pos_l,pos_r,input signed [15:0] imu_yaw,
 output reg update,ready,fault,
 output reg signed [15:0] heading,encoder_heading,innovation,yaw_rate,
 output reg signed [15:0] velocity_l,velocity_r,
 output reg signed [31:0] sample_pos_l,sample_pos_r,
 output reg signed [15:0] sample_yaw,
 output reg [31:0] timestamp_ms,
 output [15:0] flags
);
 localparam WIN=(CLK_HZ/CONTROL_HZ<16)?16:CLK_HZ/CONTROL_HZ;
 localparam MS_DIV=(CLK_HZ/1000<1)?1:CLK_HZ/1000;
 localparam CALIBRATED=(TRACK_MM>=50 && TRACK_MM<=1000 && COUNTS_PER_M>=100);
 // Geometry may be available for independent local turning without changing
 // the main IMU heading used by navigation or enabling global sensor fusion.
 localparam FUSION_ENABLED=CALIBRATED && FUSE_ENCODER;
 // (180/pi)*100 degrees * 1000 mm/m, in Q16. Constant division only.
 localparam signed [31:0] ENC_Q16=CALIBRATED?
                 (64'd375493623808/(64'd1*COUNTS_PER_M*TRACK_MM)):0;
 reg [31:0] timer,ms_div,clock_ms,imu_age;
 reg [2:0] phase;
 reg signed [31:0] prev_l,prev_r,dl_hold,dr_hold;
 reg signed [15:0] latest_yaw,frame_prev,held_yaw,previous_imu;
 reg frame_seen,latest_ok,new_imu,held_new,initialized;
 reg encoder_fault,yaw_fault;
 reg [7:0] disagreement;
 reg signed [63:0] enc_product;
 reg [15:0] enc_fraction;
 reg signed [31:0] enc_interval;
 reg signed [17:0] prediction,enc_prediction;
 reg signed [16:0] correction,error_hold,imu_delta_hold;
 reg signed [31:0] rate_hold;
 reg [31:0] stamp_hold;
 wire ms_tick=(ms_div==MS_DIV-1);
 wire fresh=imu_ok && latest_ok && imu_age<IMU_TIMEOUT_MS;
 wire signed [31:0] dl=pos_l-prev_l,dr=pos_r-prev_r;
 wire bad_encoder=(dl>MAX_DELTA_COUNTS || dl< -MAX_DELTA_COUNTS ||
                   dr>MAX_DELTA_COUNTS || dr< -MAX_DELTA_COUNTS);
 wire yaw_in_range=imu_yaw>=-18000 && imu_yaw<=18000;
 function signed [16:0] wrap;
  input signed [31:0] a;
  begin
   if(a>18000)wrap=a-36000;
   else if(a< -18000)wrap=a+36000;
   else wrap=a;
  end
 endfunction
 wire signed [16:0] frame_delta=wrap($signed(imu_yaw)-$signed(frame_prev));
 wire bad_yaw=!yaw_in_range || (frame_seen &&
                  (frame_delta>MAX_YAW_STEP_CD || frame_delta< -MAX_YAW_STEP_CD));
 wire signed [31:0] enc_step=enc_product>>>16;
 wire signed [16:0] pred_error=wrap($signed(held_yaw)-$signed(prediction));
 wire signed [16:0] correction_raw=pred_error>>>CORRECTION_SHIFT;
 wire signed [16:0] correction_limited=correction_raw>MAX_CORRECTION_CD?MAX_CORRECTION_CD:
                   correction_raw< -MAX_CORRECTION_CD?-MAX_CORRECTION_CD:correction_raw;
 wire signed [16:0] imu_delta=wrap($signed(held_yaw)-$signed(previous_imu));
 wire signed [31:0] interval_error=$signed(imu_delta)-(enc_interval+enc_step);
 wire disagree=interval_error>DISAGREE_CD || interval_error< -DISAGREE_CD;
 assign flags={5'd0,(CALIBRATED!=0),1'b0,yaw_fault,encoder_fault,1'b0,
               (disagreement>=DISAGREE_WINDOWS),fault,fresh,
               (FUSION_ENABLED!=0),ready,initialized};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   timer<=0;ms_div<=0;clock_ms<=0;imu_age<=IMU_TIMEOUT_MS;timestamp_ms<=0;
   phase<=0;prev_l<=0;prev_r<=0;dl_hold<=0;dr_hold<=0;
   latest_yaw<=0;frame_prev<=0;held_yaw<=0;previous_imu<=0;
   frame_seen<=0;latest_ok<=0;new_imu<=0;held_new<=0;initialized<=0;encoder_fault<=0;yaw_fault<=0;
   heading<=0;encoder_heading<=0;innovation<=0;yaw_rate<=0;
   update<=0;ready<=0;fault<=0;velocity_l<=0;velocity_r<=0;
   sample_pos_l<=0;sample_pos_r<=0;sample_yaw<=0;
   disagreement<=0;enc_product<=0;enc_fraction<=0;enc_interval<=0;
   prediction<=0;enc_prediction<=0;correction<=0;error_hold<=0;
   imu_delta_hold<=0;rate_hold<=0;stamp_hold<=0;
  end else begin
   update<=0;
   if(ms_tick)begin
    ms_div<=0;clock_ms<=clock_ms+1'b1;
    if(imu_age<IMU_TIMEOUT_MS)imu_age<=imu_age+1'b1;
   end else ms_div<=ms_div+1'b1;
   if(clear_fault)begin
    fault<=0;encoder_fault<=0;yaw_fault<=0;
    // Normal DISARM packets also carry fault-clear. Preserve a healthy pose:
    // route preparation sends DISARM immediately before its explicit reset.
    if(fault || !ready)begin
     initialized<=0;ready<=0;phase<=0;disagreement<=0;
     frame_seen<=0;latest_ok<=0;new_imu<=0;
    end
   end
   if(!imu_ok)begin latest_ok<=0;frame_seen<=0;ready<=0;initialized<=0;phase<=0;end
   if(imu_update && imu_ok)begin
    frame_prev<=imu_yaw;frame_seen<=yaw_in_range;
    if(bad_yaw)begin fault<=1;yaw_fault<=1;latest_ok<=0;ready<=0;initialized<=0;phase<=0;end
    else begin latest_yaw<=imu_yaw;latest_ok<=1;new_imu<=1;imu_age<=0;end
   end
   if(!fresh)begin ready<=0;initialized<=0;phase<=0;end
   if(timer==WIN-1)begin
    timer<=0;prev_l<=pos_l;prev_r<=pos_r;
    if(fresh && !fault && !clear_fault && !(imu_update && bad_yaw))begin
     if(!initialized)begin
      heading<=latest_yaw;encoder_heading<=latest_yaw;previous_imu<=latest_yaw;
      initialized<=1;ready<=1;update<=1;timestamp_ms<=clock_ms;
      velocity_l<=0;velocity_r<=0;innovation<=0;yaw_rate<=0;
      sample_pos_l<=pos_l;sample_pos_r<=pos_r;sample_yaw<=latest_yaw;
      enc_fraction<=0;enc_interval<=0;disagreement<=0;
      new_imu<=imu_update && imu_ok && !bad_yaw;
     end else if(bad_encoder)begin fault<=1;encoder_fault<=1;ready<=0;phase<=0;end
     else begin
      dl_hold<=dl;dr_hold<=dr;held_yaw<=latest_yaw;held_new<=new_imu;
      new_imu<=imu_update && imu_ok && !bad_yaw;
      stamp_hold<=clock_ms;phase<=1;
     end
    end
   end else timer<=timer+1'b1;
   // All long arithmetic is registered; no division by a live sensor value.
   if(fresh && !fault && !clear_fault && !(imu_update && bad_yaw))case(phase)
    1:begin
     enc_product<=($signed({dr_hold[31],dr_hold})-$signed({dl_hold[31],dl_hold}))*
                   $signed(ENC_Q16)*(YAW_SIGN<0?-1:1)+$signed({1'b0,enc_fraction});
     phase<=2;
    end
    2:begin
     if(CALIBRATED && (enc_step>MAX_YAW_STEP_CD || enc_step< -MAX_YAW_STEP_CD))begin
      fault<=1;encoder_fault<=1;ready<=0;phase<=0;
     end else begin
     prediction<=wrap($signed(heading)+enc_step);
     enc_prediction<=wrap($signed(encoder_heading)+enc_step);
     enc_fraction<=enc_product[15:0];
     imu_delta_hold<=imu_delta;
     if(held_new)begin
      if(CALIBRATED && disagree)begin
       if(disagreement<DISAGREE_WINDOWS)disagreement<=disagreement+1'b1;
      end else disagreement<=0;
      enc_interval<=0;previous_imu<=held_yaw;
     end else enc_interval<=enc_interval+enc_step;
     phase<=3;
     end
    end
    3:begin
     correction<=FUSION_ENABLED?(held_new?correction_limited:17'sd0):17'sd0;
     error_hold<=FUSION_ENABLED?pred_error:17'sd0;
     phase<=4;
    end
    4:begin
     if(FUSION_ENABLED)begin
      heading<=wrap($signed(prediction)+$signed(correction));
      rate_hold<=$signed(wrap($signed(prediction)+$signed(correction)-$signed(heading)))*CONTROL_HZ;
     end else begin
      heading<=held_yaw;
      rate_hold<=$signed(wrap($signed(held_yaw)-$signed(heading)))*CONTROL_HZ;
     end
     encoder_heading<=CALIBRATED?enc_prediction[15:0]:held_yaw;
     velocity_l<=dl_hold[15:0];velocity_r<=dr_hold[15:0];
     sample_pos_l<=prev_l;sample_pos_r<=prev_r;sample_yaw<=held_yaw;
     innovation<=held_new?error_hold:16'sd0;timestamp_ms<=stamp_hold;
     phase<=5;
    end
    5:begin
     yaw_rate<=rate_hold>32767?16'sd32767:rate_hold< -32768?-16'sd32768:rate_hold[15:0];
     ready<=1;update<=1;phase<=0;
    end
    default:;
   endcase
  end
 end
endmodule
