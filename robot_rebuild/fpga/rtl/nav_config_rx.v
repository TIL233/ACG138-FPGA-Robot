`timescale 1ns/1ps
// Configuration never arms motion. All writes require the top-level stationary,
// disarmed permission for the entire frame. Coordinates are millimetres.
// A5 C3 | version,seq,opcode,index,x_i16LE,y_i16LE,count,zero[5] | CRC16LE | CR LF.
// CRC-CCITT-FALSE covers bytes 2..15. WRITE=1 COMMIT=2 POSE_RESET=3 CLEAR=4.
module rb_nav_config_rx #(parameter CLK_HZ=50000000)(
 input clk,rst_n,input [7:0] data,input valid,input cfg_allowed,
 output reg route_write,output reg [2:0] route_index,
 output reg signed [15:0] route_x,route_y,
 output reg route_commit,output reg [3:0] route_count,
 output reg pose_reset,route_clear,output reg route_ready,
 output reg [7:0] last_ack_seq,last_ack_status);
 function [15:0] crc_byte;
  input [15:0] crc;input [7:0] b;reg [15:0] c;integer j;
  begin c=crc^{b,8'd0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end
 endfunction
 reg [4:0] idx;
 reg [111:0] payload;
 reg [15:0] crc,got_crc;
 reg [7:0] last_seq,written;
 reg have_seq,tail_ok,allowed_frame;
 reg [111:0] last_payload;
 reg [31:0] gap;
 wire signed [15:0] x=payload[47:32],y=payload[63:48];
 wire [7:0] point_count=payload[71:64];
 wire [8:0] required_mask=(9'd1<<point_count)-1'b1;
 localparam GAP_LIMIT=(CLK_HZ/1000)*20;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   idx<=0;payload<=0;crc<=16'hffff;got_crc<=0;last_seq<=0;written<=0;
   have_seq<=0;tail_ok<=0;allowed_frame<=0;last_payload<=0;
   gap<=0;route_write<=0;route_index<=0;route_x<=0;route_y<=0;
   route_commit<=0;route_count<=0;pose_reset<=0;route_clear<=0;route_ready<=0;
   last_ack_seq<=0;last_ack_status<=8'hff;
  end else begin
   route_write<=0;route_commit<=0;pose_reset<=0;route_clear<=0;
   if(gap<GAP_LIMIT)gap<=gap+1;else idx<=0;
   if(idx!=0 && !cfg_allowed)allowed_frame<=0;
   if(valid) begin
    gap<=0;
    if(gap>=GAP_LIMIT)begin idx<=(data==8'ha5)?1:0;tail_ok<=0;allowed_frame<=cfg_allowed;end
    else case(idx)
     0:if(data==8'ha5)begin idx<=1;allowed_frame<=cfg_allowed;end
     1:if(data==8'hc3)begin idx<=2;crc<=16'hffff;end
       else idx<=(data==8'ha5)?1:0;
     2,3,4,5,6,7,8,9,10,11,12,13,14,15:begin
      payload<={data,payload[111:8]};crc<=crc_byte(crc,data);idx<=idx+1;
     end
     16:begin got_crc[7:0]<=data;idx<=17;end
     17:begin got_crc[15:8]<=data;idx<=18;end
     18:begin tail_ok<=(data==13);idx<=19;end
     19:begin
      idx<=0;
      if(tail_ok && data==10 && got_crc==crc)begin
       last_ack_seq<=payload[15:8];last_ack_status<=2;
       if(have_seq && payload[15:8]==last_seq)begin
        // Successful retransmission is acknowledged without repeating RESET.
        if(payload==last_payload)last_ack_status<=0;
       end else if(!cfg_allowed || !allowed_frame)last_ack_status<=1;
       else if(payload[7:0]==1 && payload[111:72]==0)begin
       case(payload[23:16])
        1:if(payload[31:24]<8 && point_count==0 &&
             x>=-16'sd10000 && x<=16'sd10000 && y>=-16'sd10000 && y<=16'sd10000)begin
         route_write<=1;route_index<=payload[26:24];route_x<=x;route_y<=y;
         written[payload[26:24]]<=1;route_ready<=0;
         last_seq<=payload[15:8];last_payload<=payload;have_seq<=1;last_ack_status<=0;
        end
        2:if(payload[63:24]==0 && point_count>=1 && point_count<=8)begin
         if((written & required_mask[7:0])==required_mask[7:0])begin
          route_commit<=1;route_count<=point_count[3:0];route_ready<=1;
          last_seq<=payload[15:8];last_payload<=payload;have_seq<=1;last_ack_status<=0;
         end else last_ack_status<=3;
        end
        3:if(payload[71:24]==0)begin
         pose_reset<=1;last_seq<=payload[15:8];last_payload<=payload;have_seq<=1;last_ack_status<=0;
        end
        4:if(payload[71:24]==0)begin
         route_clear<=1;route_ready<=0;written<=0;route_count<=0;
         last_seq<=payload[15:8];last_payload<=payload;have_seq<=1;last_ack_status<=0;
        end
        default:begin end
       endcase
       end
      end
     end
     default:idx<=0;
    endcase
   end
  end
 end
endmodule
