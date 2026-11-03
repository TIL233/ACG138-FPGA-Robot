`timescale 1ns/1ps
module rb_lidar_scan_map #(parameter CLK_HZ=50000000, FRONT_ZERO_Q6=0,
 parameter PACKET_GAP_MS=100)(
 input clk,rst_n,input [7:0] data,input valid,clear,
 output update,fresh,output [2:0] sector_valid,
 output [15:0] front_mm,left_mm,right_mm,scan_id,good_packets,bad_packets,
 output reg map_update,output map_fresh,output reg [575:0] map_ranges,
 output [35:0] map_known,output reg [15:0] map_scan_id,
 output reg [15:0] map_age_ms,output reg [15:0] map_span_ms
);
 wire stream_rst_n=rst_n && !clear;
 wire pv,start,ok,err;wire [15:0] angle,distance;wire [1:0] flags;
 rb_lidar_packet #(.CLK_HZ(CLK_HZ),.GAP_MS(PACKET_GAP_MS)) parser(
  .clk(clk),.rst_n(stream_rst_n),.data(data),.valid(valid),.point_valid(pv),
  .scan_start(start),.angle_q6(angle),.distance_mm(distance),.interference(flags),
  .packet_ok(ok),.packet_error(err));
 // Keep the old three-sector implementation, including its freshness and
 // error semantics. Map invalidation must not change those legacy outputs.
 rb_lidar_sectors #(.CLK_HZ(CLK_HZ),.FRONT_ZERO_Q6(FRONT_ZERO_Q6)) sectors(
  .clk(clk),.rst_n(stream_rst_n),.point_valid(pv),.scan_start(start),
  .packet_error(err),.angle_q6(angle),.distance_mm(distance),.interference(flags),
  .update(update),.fresh(fresh),.sector_valid(sector_valid),.front_mm(front_mm),
  .left_mm(left_mm),.right_mm(right_mm),.scan_id(scan_id),
  .good_packets(good_packets),.bad_packets(bad_packets),.packet_ok(ok));
 localparam MS_DIV=(CLK_HZ/1000>0)?CLK_HZ/1000:1;
 localparam CONFIG_OK=FRONT_ZERO_Q6>=0 && FRONT_ZERO_Q6<23040;
 localparam MAP_LATENCY=2;
 // The legacy branch above consumes the original parser outputs unchanged.
 // Only the map branch crosses these two registers. Start, distance, and
 // usable travel with the angle; an error discards both pending stages.
 reg normalized_valid,normalized_start,normalized_usable;
 reg [14:0] normalized_angle;
 reg [15:0] normalized_distance;
 reg decoded_valid,decoded_start,decoded_usable;
 reg [15:0] decoded_distance;
 reg [35:0] decoded_direct;
 reg [107:0] decoded_coverage;
 wire [15:0] relative_angle=(angle>=FRONT_ZERO_Q6)?angle-FRONT_ZERO_Q6:
                                                     angle+23040-FRONT_ZERO_Q6;
 integer p;
 always @(posedge clk or negedge stream_rst_n)begin
  if(!stream_rst_n)begin
   normalized_valid<=0;normalized_start<=0;normalized_usable<=0;
   normalized_angle<=0;normalized_distance<=0;
   decoded_valid<=0;decoded_start<=0;decoded_usable<=0;
   decoded_distance<=0;decoded_direct<=0;decoded_coverage<=0;
  end else if(err)begin
   normalized_valid<=0;decoded_valid<=0;
  end else begin
   normalized_valid<=pv && CONFIG_OK;
   if(pv)begin
    normalized_angle<=relative_angle[14:0];normalized_start<=start;
    normalized_distance<=distance;
    normalized_usable<=distance>=120 && distance<=8000 && flags<2;
   end
   decoded_valid<=normalized_valid;
   if(normalized_valid)begin
    decoded_start<=normalized_start;decoded_distance<=normalized_distance;
    decoded_usable<=normalized_usable;
    // Static comparisons replace division/remainder and dynamic array muxes.
    // Each local register sees only its own direct and neighbor write masks.
    for(p=0;p<36;p=p+1)begin
     decoded_direct[p]<=normalized_angle>=p*640 && normalized_angle<(p+1)*640;
     decoded_coverage[p*3+:3]<={
      normalized_angle>=p*640+426 && normalized_angle<(p+1)*640,
      normalized_angle>=p*640+213 && normalized_angle<p*640+426,
      normalized_angle>=p*640 && normalized_angle<p*640+213};
    end
   end
  end
 end
 reg collecting,published;
 reg [15:0] points;
 reg [15:0] minima[0:35];reg [1:0] counts[0:35];
 reg [2:0] coverage[0:35];reg [35:0] poisoned,known_snapshot;
 // Relative phases avoid a division circuit and make the span/age upper
 // bound independent of the wall-clock millisecond phase.
 reg [31:0] scan_phase,age_phase;
 reg [15:0] scan_ms,age_ms;
 wire scan_roll=scan_phase==MS_DIV-1;
 wire [15:0] completed_ms=scan_ms+(scan_roll?16'd1:16'd0);
 wire [31:0] completed_phase=scan_roll?32'd0:scan_phase+1'b1;
 wire [15:0] completed_ceil=completed_ms+((completed_phase!=0)?16'd1:16'd0);
 wire [35:0] decoded_nearby=decoded_direct |
      {decoded_direct[34:0],decoded_direct[35]} |
      {decoded_direct[0],decoded_direct[35:1]};
 // Maintain ceil(age) at its transition edges instead of adding phase and
 // floor age across the module boundary. After publication, ceil increases
 // on phase 0 -> 1; wrapping the phase already preserves the previous ceil.
 // Parser errors are visible before the next clock edge consumes them.
 // Clearing the transport similarly withdraws the snapshot immediately.
 assign map_fresh=stream_rst_n && CONFIG_OK &&
                  !err && published && map_age_ms<=350;
 assign map_known=map_fresh?known_snapshot:36'd0;
 integer i;
 always @(posedge clk or negedge stream_rst_n)begin
  if(!stream_rst_n)begin
   collecting<=0;published<=0;points<=0;poisoned<=0;known_snapshot<=0;
   map_update<=0;map_ranges<={576{1'b1}};map_scan_id<=0;map_span_ms<=0;map_age_ms<=16'hffff;
   scan_phase<=0;scan_ms<=0;age_phase<=0;age_ms<=16'hffff;
   for(i=0;i<36;i=i+1)begin minima[i]<=16'hffff;counts[i]<=0;coverage[i]<=0;end
  end else begin
   map_update<=0;
   if(age_ms!=16'hffff)begin
    if(age_phase==0 && map_age_ms!=16'hffff)map_age_ms<=map_age_ms+1'b1;
    if(age_phase==MS_DIV-1)begin age_phase<=0;age_ms<=age_ms+1'b1;end
    else age_phase<=age_phase+1'b1;
   end
   if(map_age_ms>350)published<=0;
   if(collecting)begin
    scan_phase<=completed_phase;scan_ms<=completed_ms;
    if(completed_ms>=350)collecting<=0;
   end
   if(err)begin collecting<=0;published<=0;end
   else if(decoded_valid)begin
    if(decoded_start)begin
     // Current start belongs only to the new revolution. Publish all 36
     // old bins on one edge before initializing the independent work bank.
     if(collecting && completed_ms<350 && points>=100)begin
      for(i=0;i<36;i=i+1)begin
       known_snapshot[i]<=counts[i]>=3 && coverage[i]==7 && !poisoned[i];
       map_ranges[i*16+:16]<=(counts[i]>=3 && coverage[i]==7 && !poisoned[i])?minima[i]:16'hffff;
      end
      published<=1;map_update<=1;map_scan_id<=map_scan_id+1'b1;
      map_span_ms<=completed_ceil;map_age_ms<=completed_ceil;age_ms<=completed_ms;age_phase<=completed_phase;
     end
     // The oldest return was observed MAP_LATENCY clocks before this edge.
     // Carry that latency into both span and age, never rejuvenating a scan.
     collecting<=1;points<=1;
     scan_ms<=MAP_LATENCY/MS_DIV;scan_phase<=MAP_LATENCY%MS_DIV;
     for(i=0;i<36;i=i+1)begin
      minima[i]<=(decoded_usable && decoded_nearby[i])?decoded_distance:16'hffff;
      counts[i]<=(decoded_usable && decoded_direct[i])?1:0;
      coverage[i]<=decoded_usable?decoded_coverage[i*3+:3]:3'd0;
      poisoned[i]<=!decoded_usable && decoded_direct[i];
     end
    end else if(collecting && completed_ms<350)begin
     if(points!=16'hffff)points<=points+1'b1;
     for(i=0;i<36;i=i+1)begin
      if(decoded_direct[i])begin
       if(!decoded_usable)poisoned[i]<=1;
       else begin
        if(counts[i]!=3)counts[i]<=counts[i]+1'b1;
        coverage[i]<=coverage[i]|decoded_coverage[i*3+:3];
       end
      end
      if(decoded_usable && decoded_nearby[i] && decoded_distance<minima[i])
       minima[i]<=decoded_distance;
     end
    end
   end
  end
 end
endmodule
