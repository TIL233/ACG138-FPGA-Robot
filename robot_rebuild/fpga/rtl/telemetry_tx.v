`timescale 1ns/1ps
module rb_telemetry #(parameter CLK_HZ=50000000,ENABLE_CAMERA_DIAG=0,CAMERA_DIAG_V2=0,CAMERA_DIAG_V3=0,CAMERA_DIAG_V4=0,REMOTE_FOLLOW=0,REMOTE_NAV=0,ENABLE_MOTION_DIAG=0,MOTION_HZ=10,ENABLE_PREDICTIVE_DIAG=0,REMOTE_SPEED=0,REMOTE_RECORD=0,ENABLE_LIDAR_DIAG=0,ENABLE_LIDAR_PLANNING_DIAG=0,ENABLE_LIDAR_MAP_DIAG=0)(
 input clk,rst_n,input [7:0] state,health,command,speed,
 input [15:0] dl,dr,vl,vr,roll,pitch,yaw,fault,output tx,
 // High nibble is the persistent IR STOP generation; low nibble is nav state.
 input [7:0] nav_state,input [2:0] nav_index,input [3:0] nav_count,input [7:0] nav_flags,
 input signed [31:0] nav_x,nav_y,input signed [15:0] nav_heading,
 input [7:0] nav_ack_seq,nav_ack_status,input [239:0] camera_diag,input [15:0] follow_count,
 input [15:0] nav_mode_count,input [319:0] motion_diag,input [255:0] predictive_diag,input [7:0] follow_speed,input [15:0] record_count,
 input [159:0] lidar_diag,input [319:0] lidar_planning_diag,input [655:0] lidar_map_diag);
 function [15:0] crc_byte;
  input [15:0] crc;input [7:0] b;reg [15:0] c;integer j;
  begin c=crc^{b,8'd0};for(j=0;j<8;j=j+1) c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end
 endfunction
 reg [31:0] timer;reg [7:0] seq;
 reg [319:0] body;reg [15:0] crc;
 localparam INDEX_BITS=ENABLE_LIDAR_MAP_DIAG?7:6;
 reg [INDEX_BITS-1:0] idx;reg sending,start,next_nav,is_nav;reg [7:0] data;wire busy;
 reg is_diag,diag_pending,is_motion,motion_pending,is_predictive,predictive_pending;
 reg is_lidar,lidar_pending;
 reg is_planning,planning_pending;
 reg is_map,map_pending,map_divider,map_load,map_shift;
 wire [7:0] map_byte;
 generate if(ENABLE_LIDAR_MAP_DIAG)begin:map_serializer
  // One parallel load makes all 36 ranges, validity bits and age atomic.
  // Registered enables keep the wide datapath off the UART arbitration path.
  reg [671:0] snapshot;
  always @(posedge clk)begin
   if(map_load)snapshot<={lidar_map_diag,body[15:0]};
   else if(map_shift)snapshot<={8'd0,snapshot[671:8]};
  end
  assign map_byte=snapshot[7:0];
 end else begin:no_map_serializer
  assign map_byte=8'd0;
 end endgenerate
 wire [7:0] payload_byte=(ENABLE_LIDAR_MAP_DIAG && is_map)?map_byte:body[7:0];
 reg [31:0] motion_timer;
 localparam MOTION_DIV=CLK_HZ/((MOTION_HZ>0)?MOTION_HZ:10);
 wire motion_due=ENABLE_MOTION_DIAG && MOTION_HZ>10 && motion_timer>=MOTION_DIV-1;
 wire [INDEX_BITS-1:0] crc_index=(ENABLE_LIDAR_MAP_DIAG && is_map)?7'd86:is_planning?6'd42:is_lidar?6'd22:is_predictive?6'd34:is_motion?6'd42:is_diag?(CAMERA_DIAG_V4?6'd32:CAMERA_DIAG_V3?5'd28:(CAMERA_DIAG_V2?5'd24:5'd12)):(is_nav?(REMOTE_RECORD?6'd27:REMOTE_SPEED?6'd25:REMOTE_NAV?6'd24:REMOTE_FOLLOW?6'd22:6'd20):6'd24);
 rb_uart_tx #(.CLK_HZ(CLK_HZ)) uart(clk,rst_n,start,data,tx,busy);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin timer<=0;seq<=0;body<=0;crc<=16'hffff;idx<=0;
   sending<=0;start<=0;data<=0;next_nav<=0;is_nav<=0;is_diag<=0;diag_pending<=0;is_motion<=0;motion_pending<=0;
   is_predictive<=0;predictive_pending<=0;motion_timer<=0;is_lidar<=0;lidar_pending<=0;is_planning<=0;planning_pending<=0;
   is_map<=0;map_pending<=0;map_divider<=0;map_load<=0;map_shift<=0;end
  else begin
   start<=0;
   map_load<=0;map_shift<=0;
   if(ENABLE_MOTION_DIAG && MOTION_HZ>10)begin
    if(motion_due)begin motion_timer<=0;motion_pending<=1;end
    else motion_timer<=motion_timer+1'b1;
   end
   // Alternate 28-byte legacy and 24-byte navigation snapshots: each 10 Hz.
   if(timer<CLK_HZ/20) timer<=timer+1;
   if(!sending && !busy && !start && diag_pending)begin
    sending<=1;idx<=0;crc<=16'hffff;seq<=seq+1;
    is_diag<=1;is_nav<=0;is_motion<=0;is_predictive<=0;is_lidar<=0;is_planning<=0;diag_pending<=0;
    if(CAMERA_DIAG_V4)body<={camera_diag[239:16],seq,8'd4};
    else if(CAMERA_DIAG_V3)body<={32'd0,camera_diag[207:16],seq,8'd3};
    else if(CAMERA_DIAG_V2)body<={32'd0,camera_diag[175:16],seq,8'd2};
    else body<={96'd0,camera_diag[79:16],seq,8'd1};
   end else if(!sending && !busy && !start && predictive_pending)begin
    sending<=1;idx<=0;crc<=16'hffff;seq<=seq+1;
    is_predictive<=1;is_motion<=0;is_diag<=0;is_nav<=0;is_lidar<=0;is_planning<=0;predictive_pending<=0;
    body<={predictive_diag[255:16],seq,8'd1};
   end else if(!sending && !busy && !start && motion_pending)begin
    sending<=1;idx<=0;crc<=16'hffff;seq<=seq+1;
    is_motion<=1;is_diag<=0;is_predictive<=0;is_nav<=0;is_lidar<=0;is_planning<=0;motion_pending<=motion_due;
    body<={motion_diag[319:16],seq,8'd1};
   end else if(!sending && !busy && !start && lidar_pending)begin
    sending<=1;idx<=0;crc<=16'hffff;seq<=seq+1;
    is_lidar<=1;is_motion<=0;is_diag<=0;is_predictive<=0;is_nav<=0;is_planning<=0;lidar_pending<=0;
    body<={160'd0,lidar_diag[159:16],seq,8'd1};
   end else if(!sending && !busy && !start && planning_pending)begin
    sending<=1;idx<=0;crc<=16'hffff;seq<=seq+1;
    is_planning<=1;is_lidar<=0;is_motion<=0;is_diag<=0;is_predictive<=0;is_nav<=0;planning_pending<=0;
    body<={lidar_planning_diag[319:16],seq,8'd1};
   end else if(!sending && !busy && !start && timer>=CLK_HZ/20) begin
    timer<=0;sending<=1;idx<=0;crc<=16'hffff;seq<=seq+1;
    is_diag<=0;is_motion<=0;is_predictive<=0;is_lidar<=0;is_planning<=0;is_nav<=next_nav;next_nav<=!next_nav;
    if(next_nav)begin
     if(REMOTE_RECORD)body<={record_count,follow_speed,nav_mode_count,follow_count,nav_ack_status,nav_ack_seq,nav_heading,nav_y,nav_x,
                      nav_flags,{4'd0,nav_count},{5'd0,nav_index},nav_state,seq,8'd5};
     else if(REMOTE_SPEED)body<={follow_speed,nav_mode_count,follow_count,nav_ack_status,nav_ack_seq,nav_heading,nav_y,nav_x,
                      nav_flags,{4'd0,nav_count},{5'd0,nav_index},nav_state,seq,8'd4};
     else if(REMOTE_NAV)body<={nav_mode_count,follow_count,nav_ack_status,nav_ack_seq,nav_heading,nav_y,nav_x,
                      nav_flags,{4'd0,nav_count},{5'd0,nav_index},nav_state,seq,8'd3};
     else if(REMOTE_FOLLOW)body<={follow_count,nav_ack_status,nav_ack_seq,nav_heading,nav_y,nav_x,
                      nav_flags,{4'd0,nav_count},{5'd0,nav_index},nav_state,seq,8'd2};
     else body<={32'd0,nav_ack_status,nav_ack_seq,nav_heading,nav_y,nav_x,
                      nav_flags,{4'd0,nav_count},{5'd0,nav_index},nav_state,seq,8'd1};
    end
    else body<={fault,yaw,pitch,roll,vr,vl,dr,dl,speed,command,health,state,seq,8'd1};
   end else if(ENABLE_LIDAR_MAP_DIAG && !sending && !busy && !start && map_pending)begin
    // Lowest priority; emit at most one map per two 10 Hz navigation frames.
    sending<=1;idx<=0;crc<=16'hffff;seq<=seq+1;body<={seq,8'd1};
    is_map<=1;map_load<=1;map_pending<=0;
    is_diag<=0;is_motion<=0;is_predictive<=0;is_lidar<=0;is_planning<=0;is_nav<=0;
   end else if(sending && !busy && !start) begin
    start<=1;idx<=idx+1;
    if(idx==0)data<=8'h5a;
    else if(idx==1)data<=(ENABLE_LIDAR_MAP_DIAG && is_map)?8'hc9:is_planning?8'hc8:is_lidar?8'hb7:is_predictive?8'hf6:is_motion?8'he5:is_diag?8'hd4:(is_nav?8'hc3:8'ha5);
    else if(idx==crc_index)data<=crc[7:0];
    else if(idx==crc_index+1)data<=crc[15:8];
    else if(idx==crc_index+2)data<=13;
    else if(idx==crc_index+3)begin data<=10;sending<=0;is_map<=0;
     if(ENABLE_LIDAR_MAP_DIAG && is_nav)begin
      map_divider<=!map_divider;
      if(!map_divider)map_pending<=1;
     end
     if(ENABLE_CAMERA_DIAG && is_nav)diag_pending<=1;
     if(ENABLE_MOTION_DIAG && MOTION_HZ<=10 && is_nav)motion_pending<=1;
     if(ENABLE_LIDAR_DIAG && is_nav)lidar_pending<=1;
     if(ENABLE_LIDAR_PLANNING_DIAG && is_nav)planning_pending<=1;
     if(ENABLE_PREDICTIVE_DIAG && is_nav)predictive_pending<=1;end
    else begin
     data<=payload_byte;crc<=crc_byte(crc,payload_byte);
     if(ENABLE_LIDAR_MAP_DIAG && is_map)map_shift<=1;
     else body<={8'd0,body[319:8]};
    end
   end
  end
 end
endmodule
