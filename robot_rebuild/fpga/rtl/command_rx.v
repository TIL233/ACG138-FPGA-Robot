`timescale 1ns/1ps
// 12 bytes: A5 5A version seq arm command speed reserved CRC16LE CR LF.
// CRC-CCITT-FALSE covers bytes 2..7. Commands 0..6, speed <=64.
// reserved bit0 selects waypoint navigation, bit1 direct visual following.
// Modes are exclusive. Direct following never requests reverse/pivot motion.
// Navigation is only an explicit ARM+FORWARD or DISARM+STOP request.
module rb_command_rx #(parameter CLK_HZ=50000000)(
 input clk,rst_n,input [7:0] data,input valid,
 output reg armed,output reg [2:0] command,output reg [7:0] speed,
 output reg update,output wire fresh,output reg mode_nav,output reg mode_follow);
 function [15:0] crc_byte;
  input [15:0] crc;input [7:0] b;reg [15:0] c;integer j;
  begin c=crc^{b,8'd0};for(j=0;j<8;j=j+1) c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end
 endfunction
 reg [3:0] idx;
 reg [47:0] payload;
 reg [15:0] crc,got_crc;
 reg [7:0] last_seq;
 reg have_seq,tail_ok;
 reg [31:0] gap;
 localparam GAP_LIMIT=(CLK_HZ/1000)*20;
 rb_fresh #(.CLK_HZ(CLK_HZ)) alive(clk,rst_n,update,fresh);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin idx<=0;payload<=0;crc<=16'hffff;got_crc<=0;last_seq<=0;
   have_seq<=0;tail_ok<=0;gap<=0;armed<=0;command<=0;speed<=0;update<=0;mode_nav<=0;mode_follow<=0;end
  else begin
   update<=0;
   if(gap<GAP_LIMIT) gap<=gap+1;else idx<=0;
   if(valid) begin
    gap<=0;
    // A byte arriving on the timeout edge must not complete the abandoned
    // packet. It may only start a new header, just as when already idle.
    if(gap>=GAP_LIMIT) begin idx<=(data==8'ha5)?1:0;tail_ok<=0;end
    else case(idx)
     0: if(data==8'ha5) idx<=1;
     1: if(data==8'h5a) begin idx<=2;crc<=16'hffff;end
        else idx<=(data==8'ha5)?1:0;
     2,3,4,5,6,7: begin payload<={data,payload[47:8]};crc<=crc_byte(crc,data);idx<=idx+1;end
     8: begin got_crc[7:0]<=data;idx<=9;end
     9: begin got_crc[15:8]<=data;idx<=10;end
     10: begin tail_ok<=(data==13);idx<=11;end
     11: begin
      idx<=0;
      if(tail_ok && data==10 && got_crc==crc && payload[7:0]==1 &&
         payload[23:16]<=1 && payload[31:24]<=6 && payload[39:32]<=64 && payload[47:42]==0 &&
         !(payload[40] && payload[41]) &&
         (!payload[40] || ((payload[16] && payload[31:24]==1) ||
                           (!payload[16] && payload[31:24]==0))) &&
         (!payload[41] || ((payload[16] && (payload[31:24]==0 || payload[31:24]==1 ||
                                           payload[31:24]==3 || payload[31:24]==4)) ||
                           (!payload[16] && payload[31:24]==0))) &&
         (!have_seq || !fresh || payload[15:8]!=last_seq)) begin
       armed<=payload[16];command<=payload[26:24];speed<=payload[39:32];mode_nav<=payload[40];mode_follow<=payload[41];
       last_seq<=payload[15:8];have_seq<=1;update<=1;
      end
     end
     default:idx<=0;
    endcase
   end
  end
 end
endmodule
