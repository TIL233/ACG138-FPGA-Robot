`timescale 1ns/1ps
// X3/X3 Pro single-channel, no-intensity AA55 packets, 115200 8N1.
// A whole packet is buffered and XOR-checked BEFORE publishing any point.
// Angles are raw Q6 clockwise; sectors below conservatively include the
// documented triangulation correction rather than claiming corrected mapping.
module rb_lidar_packet #(parameter CLK_HZ=50000000, GAP_MS=5)(
 input clk,rst_n,input [7:0] data,input valid,
 output reg point_valid,scan_start,output reg [15:0] angle_q6,distance_mm,
 output reg [1:0] interference,output reg packet_ok,packet_error
);
 reg [3:0] state;
 reg [7:0] ct,count,index,low;
 reg [15:0] first,last,check_sum,received_sum;
 reg [15:0] samples[0:254];
 reg [31:0] gap;
 localparam GAP_LIMIT=(CLK_HZ/1000)*GAP_MS;
 reg [15:0] dividend,quotient,remainder,step_angle,step_remainder,carry;
 reg [7:0] denominator;
 reg [4:0] divide_bit;
 wire [16:0] shifted_remainder={remainder,dividend[15]};
 wire [15:0] word_in={data,low};
 wire [16:0] advanced_angle={1'b0,angle_q6}+step_angle+
                            ((carry+step_remainder)>=denominator);
 wire [15:0] span=(last>=first)?((last-first)>>1):
                               (16'd23040-((first-last)>>1));
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   state<=0;ct<=0;count<=0;index<=0;low<=0;first<=0;last<=0;
   check_sum<=0;received_sum<=0;gap<=0;point_valid<=0;scan_start<=0;
   angle_q6<=0;distance_mm<=0;interference<=0;packet_ok<=0;packet_error<=0;
   dividend<=0;quotient<=0;remainder<=0;denominator<=1;divide_bit<=0;
   step_angle<=0;step_remainder<=0;carry<=0;
  end else begin
   point_valid<=0;scan_start<=0;packet_ok<=0;packet_error<=0;
   if(valid)gap<=0;else if(gap<GAP_LIMIT)gap<=gap+1'b1;
   if(state!=0 && state<12 && !valid && gap>=GAP_LIMIT-1)begin
    state<=0;packet_error<=1;
   end else if(state>=12 && valid)begin
    // At 50MHz emission finishes before the next UART byte. Never silently
    // overrun if instantiated with an unsupported clock or synthetic source.
    state<=(data==8'haa)?1:0;packet_error<=1;
   end else if(state==12)begin
    angle_q6<=first>>1;index<=0;carry<=0;
    if(count==1)begin step_angle<=0;step_remainder<=0;denominator<=1;state<=14;end
    else begin
     dividend<=span;quotient<=0;remainder<=0;denominator<=count-1'b1;
     divide_bit<=0;state<=13;
    end
   end else if(state==13)begin
    dividend<=dividend<<1;
    if(shifted_remainder>=denominator)begin
     remainder<=shifted_remainder-denominator;quotient<={quotient[14:0],1'b1};
    end else begin remainder<=shifted_remainder;quotient<={quotient[14:0],1'b0};end
    if(divide_bit==15)begin
     step_angle<={quotient[14:0],shifted_remainder>=denominator};
     step_remainder<=(shifted_remainder>=denominator)?shifted_remainder-denominator:shifted_remainder;
     state<=14;
    end else divide_bit<=divide_bit+1'b1;
   end else if(state==14)begin
    // Separate emission from angle advancement so angle and sample agree.
    point_valid<=1;scan_start<=ct[0] && index==0;
    distance_mm<=samples[index]>>2;interference<=samples[index][1:0];
    if(index==count-1'b1)state<=0;else state<=15;
   end else if(state==15)begin
    angle_q6<=(advanced_angle>=23040)?advanced_angle-23040:advanced_angle;
    carry<=(carry+step_remainder>=denominator)?carry+step_remainder-denominator:carry+step_remainder;
    index<=index+1'b1;state<=14;
   end else if(valid)case(state)
    0:if(data==8'haa)state<=1;
    1:if(data==8'h55)state<=2;else state<=(data==8'haa)?1:0;
    2:begin ct<=data;state<=3;end
    3:if(data==0)begin state<=0;packet_error<=1;end
      else begin count<=data;check_sum<=16'h55aa^{data,ct};state<=4;end
    4:begin low<=data;state<=5;end
    5:begin first<=word_in;check_sum<=check_sum^word_in;state<=6;end
    6:begin low<=data;state<=7;end
    7:begin last<=word_in;check_sum<=check_sum^word_in;state<=8;end
    8:begin low<=data;state<=9;end
    9:begin received_sum<=word_in;index<=0;state<=10;end
    10:begin low<=data;state<=11;end
    11:begin
     samples[index]<=word_in;check_sum<=check_sum^word_in;
     if(index==count-1'b1)begin
      if((check_sum^word_in)==received_sum && first[0] && last[0] &&
         first<46080 && last<46080 && (count==1 || span<=7680))begin
       packet_ok<=1;state<=12;
      end else begin packet_error<=1;state<=0;end
     end else begin index<=index+1'b1;state<=10;end
    end
    default:state<=0;
   endcase
  end
 end
endmodule

// Validated USB tunnel byte stream. A transport discontinuity resets both
// packet work and revolution state, including a packet already emitting points.
// USB scheduling can pause between chunks; scan age/freshness still expires
// after 350ms, independently of this more tolerant packet assembly deadline.
module rb_lidar_stream #(parameter CLK_HZ=50000000, FRONT_ZERO_Q6=0,
 parameter PACKET_GAP_MS=100)(
 input clk,rst_n,input [7:0] data,input valid,clear,
 output update,fresh,output [2:0] sector_valid,
 output [15:0] front_mm,left_mm,right_mm,scan_id,good_packets,bad_packets
);
 wire stream_rst_n=rst_n && !clear;
 wire pv,start,ok,err;wire [15:0] angle,distance;wire [1:0] flags;
 rb_lidar_packet #(.CLK_HZ(CLK_HZ),.GAP_MS(PACKET_GAP_MS)) parser(
  clk,stream_rst_n,data,valid,pv,start,angle,distance,flags,ok,err);
 rb_lidar_sectors #(.CLK_HZ(CLK_HZ),.FRONT_ZERO_Q6(FRONT_ZERO_Q6)) sectors(
  clk,stream_rst_n,pv,start,err,angle,distance,flags,update,fresh,sector_valid,
  front_mm,left_mm,right_mm,scan_id,good_packets,bad_packets,ok);
endmodule

// Completed-revolution minima. Unknown is 0xffff, NEVER interpreted as clear.
// Front/right/left padded by 10deg to cover X3 distance-angle correction
// (-8..+2.4deg within 120..8000mm); mounting offset must be calibrated separately.
module rb_lidar_sectors #(parameter CLK_HZ=50000000, FRONT_ZERO_Q6=0,
 parameter TIMEOUT_MS=350,MIN_SCAN_POINTS=100,MIN_SECTOR_POINTS=3)(
 input clk,rst_n,point_valid,scan_start,packet_error,
 input [15:0] angle_q6,distance_mm,input [1:0] interference,
 output reg update,output fresh,output [2:0] sector_valid,
 output reg [15:0] front_mm,left_mm,right_mm,
 output reg [15:0] scan_id,good_packets,bad_packets,input packet_ok
);
 reg collecting,dirty;reg [15:0] point_count;
 reg [15:0] minima[0:2];reg [7:0] counts[0:2];
 reg [2:0] valid_snapshot;
 reg [31:0] scan_age,divider;
 localparam MS_DIV=(CLK_HZ/1000>0)?CLK_HZ/1000:1;
 wire tick=divider==MS_DIV-1;
 wire [15:0] a=(angle_q6>=FRONT_ZERO_Q6)?angle_q6-FRONT_ZERO_Q6:
                                               angle_q6+23040-FRONT_ZERO_Q6;
 wire [2:0] membership={(a>=35*64 && a<=145*64),
                       (a>=215*64 && a<=325*64),
                       (a<=40*64 || a>=320*64)};
 wire usable=distance_mm>=120 && distance_mm<=8000 && interference<2;
 rb_fresh #(.CLK_HZ(CLK_HZ),.TIMEOUT_MS(TIMEOUT_MS)) alive(clk,rst_n,update,fresh);
 assign sector_valid=fresh?valid_snapshot:3'd0;
 integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   collecting<=0;dirty<=0;point_count<=0;valid_snapshot<=0;update<=0;
   scan_age<=TIMEOUT_MS;divider<=0;front_mm<=16'hffff;left_mm<=16'hffff;
   right_mm<=16'hffff;scan_id<=0;good_packets<=0;bad_packets<=0;
   for(i=0;i<3;i=i+1)begin minima[i]<=16'hffff;counts[i]<=0;end
  end else begin
   update<=0;
   divider<=tick?0:divider+1'b1;
   if(tick && scan_age<TIMEOUT_MS)scan_age<=scan_age+1'b1;
   if(packet_ok && good_packets!=16'hffff)good_packets<=good_packets+1'b1;
   if(packet_error)begin dirty<=1;if(bad_packets!=16'hffff)bad_packets<=bad_packets+1'b1;end
   if(point_valid)begin
    if(scan_start)begin
     if(collecting && !dirty && !packet_error && scan_age<TIMEOUT_MS && point_count>=MIN_SCAN_POINTS)begin
      front_mm<=counts[0]>=MIN_SECTOR_POINTS?minima[0]:16'hffff;
      left_mm<=counts[1]>=MIN_SECTOR_POINTS?minima[1]:16'hffff;
      right_mm<=counts[2]>=MIN_SECTOR_POINTS?minima[2]:16'hffff;
      for(i=0;i<3;i=i+1)valid_snapshot[i]<=counts[i]>=MIN_SECTOR_POINTS;
      update<=1;scan_id<=scan_id+1'b1;
     end
     collecting<=1;dirty<=packet_error;scan_age<=0;point_count<=1;
     for(i=0;i<3;i=i+1)begin
      minima[i]<=(usable && membership[i])?distance_mm:16'hffff;
      counts[i]<=(usable && membership[i])?1:0;
     end
    end else if(collecting)begin
     if(point_count!=16'hffff)point_count<=point_count+1'b1;
     for(i=0;i<3;i=i+1)if(usable && membership[i])begin
      if(distance_mm<minima[i])minima[i]<=distance_mm;
      if(counts[i]!=255)counts[i]<=counts[i]+1'b1;
     end
    end
   end
  end
 end
endmodule

module rb_lidar_x3 #(parameter CLK_HZ=50000000, FRONT_ZERO_Q6=0)(
 input clk,rst_n,rx,output update,fresh,output [2:0] sector_valid,
 output [15:0] front_mm,left_mm,right_mm,scan_id,good_packets,bad_packets
);
 wire [7:0] data;wire valid,pv,start,ok,err;wire [15:0] angle,distance;wire [1:0] flags;
 rb_uart_rx #(.CLK_HZ(CLK_HZ)) uart(clk,rst_n,rx,data,valid);
 rb_lidar_packet #(.CLK_HZ(CLK_HZ)) parser(clk,rst_n,data,valid,pv,start,angle,distance,flags,ok,err);
 rb_lidar_sectors #(.CLK_HZ(CLK_HZ),.FRONT_ZERO_Q6(FRONT_ZERO_Q6)) sectors(
  clk,rst_n,pv,start,err,angle,distance,flags,update,fresh,sector_valid,
  front_mm,left_mm,right_mm,scan_id,good_packets,bad_packets,ok);
endmodule
