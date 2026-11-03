`timescale 1ns/1ps
// Shared host UART: fixed-size legacy commands/configuration, or an isolated
// raw-lidar tunnel. No tunnel byte (including a rejected frame) reaches control.
// A5 B7 | escaped(version=1,seq,flags,length,payload,CRC16LE) | 7E
// CCITT-FALSE covers the four body header bytes and payload. A5/7D/7E must
// be escaped as 7D,(byte^20); all other escapes are invalid.
// Sequence policy: accept any first sequence after reset/error; thereafter +1
// modulo 256. A gap clears EVERYTHING, drops that frame, and adopts its seq;
// the following contiguous frame may resume. Explicit RESET clears baseline.
// Two 128-byte banks bound storage. Only CRC-validated banks drain to lidar,
// at the physical 115200 8N1 byte pace, including across bank boundaries.
module rb_lidar_bridge_rx #(parameter CLK_HZ=50000000)(
 input clk,rst_n,input [7:0] data,input valid,
 output reg [7:0] control_data,output reg control_valid,
 output reg [7:0] lidar_data,output reg lidar_valid,output reg lidar_clear
);
 localparam IDLE=0,HEADER=1,LEGACY=2,TUNNEL=3,DROP=4;
 localparam TUNNEL_GAP=(CLK_HZ/1000>0)?(CLK_HZ/1000)*5:1;
 localparam LEGACY_GAP=(CLK_HZ/1000>0)?(CLK_HZ/1000)*20:1;
 localparam BYTE_CLOCKS=(CLK_HZ/11520>0)?CLK_HZ/11520:1;
 reg [2:0] state;
 reg [4:0] legacy_left;
 reg [31:0] gap,pace;
 reg escaped_byte;
 reg [7:0] position,frame_seq,frame_flags,frame_length,last_seq;
 reg have_seq;
 reg [15:0] crc,received_crc;
 reg [7:0] bank0[0:127],bank1[0:127];
 reg [7:0] length0,length1;
 reg [1:0] busy,ready;
 reg fill_bank,tx_bank,tx_active,next_bank;
 reg [7:0] tx_index;
 // Legacy header recognition releases two bytes together; this small queue
// retains the original byte order even for a synthetic one-byte/clock source.
 reg [7:0] control_queue[0:3];
 reg [1:0] control_read,control_write;
 reg [2:0] control_count;
 reg [1:0] emit_count;
 reg [7:0] emit0,emit1;
 wire tunnel_state=state==TUNNEL || state==DROP;
 wire timed_out=gap>=((tunnel_state)?TUNNEL_GAP-1:LEGACY_GAP-1);
 wire [7:0] decoded=data^8'h20;
 function [15:0] crc_byte;
  input [15:0] old_crc;input [7:0] b;reg [15:0] c;integer j;
  begin c=old_crc^{b,8'd0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end
 endfunction
 always @* begin
  emit_count=0;emit0=data;emit1=data;
  if(valid)begin
   if((state==IDLE || (!tunnel_state && timed_out)) && data!=8'ha5)emit_count=1;
   else if(state==HEADER && !timed_out && data!=8'hb7 && data!=8'ha5)begin
    emit_count=2;emit0=8'ha5;emit1=data;
   end else if(state==LEGACY && !timed_out)emit_count=1;
  end
 end
 task flush_lidar;
  begin
   busy<=0;ready<=0;tx_active<=0;tx_index<=0;pace<=0;next_bank<=0;
   have_seq<=0;lidar_valid<=0;lidar_clear<=1;
  end
 endtask
 task reject_frame;
  begin flush_lidar();state<=DROP;escaped_byte<=0;end
 endtask
 task accept_byte;
  input [7:0] b;
  begin
   if(position==0)begin
    if(b!=1)reject_frame();else begin crc<=crc_byte(16'hffff,b);position<=1;end
   end else if(position==1)begin frame_seq<=b;crc<=crc_byte(crc,b);position<=2;end
   else if(position==2)begin
    if(b>1)reject_frame();else begin frame_flags<=b;crc<=crc_byte(crc,b);position<=3;end
   end else if(position==3)begin
    if((frame_flags==0 && (b==0 || b>128)) || (frame_flags==1 && b!=0))reject_frame();
    else begin frame_length<=b;crc<=crc_byte(crc,b);position<=4;end
   end else if(position<4+{1'b0,frame_length})begin
    if(fill_bank)bank1[position-4]<=b;else bank0[position-4]<=b;
    crc<=crc_byte(crc,b);position<=position+1'b1;
   end else if(position==4+{1'b0,frame_length})begin received_crc[7:0]<=b;position<=position+1'b1;end
   else if(position==5+{1'b0,frame_length})begin received_crc[15:8]<=b;position<=position+1'b1;end
   else reject_frame();
  end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;legacy_left<=0;gap<=0;pace<=0;escaped_byte<=0;
   position<=0;frame_seq<=0;frame_flags<=0;frame_length<=0;last_seq<=0;have_seq<=0;
   crc<=16'hffff;received_crc<=0;length0<=0;length1<=0;busy<=0;ready<=0;
   fill_bank<=0;tx_bank<=0;tx_active<=0;next_bank<=0;tx_index<=0;
   control_read<=0;control_write<=0;control_count<=0;
   control_data<=0;control_valid<=0;lidar_data<=0;lidar_valid<=0;lidar_clear<=0;
  end else begin
   control_valid<=0;lidar_valid<=0;lidar_clear<=0;
   if(control_count!=0)begin
    control_data<=control_queue[control_read];control_valid<=1;control_read<=control_read+1'b1;
   end
   if(emit_count!=0)begin
    control_queue[control_write]<=emit0;
    if(emit_count==2)control_queue[control_write+2'd1]<=emit1;
    control_write<=control_write+emit_count;
   end
   control_count<=control_count+emit_count-(control_count!=0);
   if(pace!=0)pace<=pace-1'b1;
   if(!tx_active)begin
    if(ready[next_bank])begin tx_active<=1;tx_bank<=next_bank;tx_index<=0;end
    else if(ready[!next_bank])begin tx_active<=1;tx_bank<=!next_bank;tx_index<=0;end
   end else if(pace==0)begin
    lidar_data<=tx_bank?bank1[tx_index]:bank0[tx_index];lidar_valid<=1;pace<=BYTE_CLOCKS-1;
    if(tx_index+1'b1==(tx_bank?length1:length0))begin
     busy[tx_bank]<=0;ready[tx_bank]<=0;tx_active<=0;next_bank<=!tx_bank;
    end else tx_index<=tx_index+1'b1;
   end
   if(valid)gap<=0;else if(gap<LEGACY_GAP)gap<=gap+1'b1;
   if(state!=IDLE && state!=DROP && timed_out)begin
    // A timed-out tunnel remains quarantined through its delayed remainder.
    // DROP has already cleared once; it waits for7E or a fresh unescaped A5
    // without repeatedly clearing an idle sensor every5ms.
    if(tunnel_state)begin flush_lidar();state<=DROP;end else state<=IDLE;
    escaped_byte<=0;
    // Timeout-edge input cannot complete an abandoned frame. A new A5 may
    // still start a header immediately, recovering STOP after a stalled send.
    if(valid && data==8'ha5)state<=HEADER;
    else if(valid && tunnel_state && data==8'h7e)state<=IDLE;
   end else if(valid)case(state)
    IDLE:if(data==8'ha5)state<=HEADER;
    HEADER:begin
     if(data==8'hb7)begin
      state<=TUNNEL;position<=0;escaped_byte<=0;frame_length<=0;crc<=16'hffff;
      if(!busy[0])begin fill_bank<=0;busy[0]<=1;end
      else if(!busy[1])begin fill_bank<=1;busy[1]<=1;end
      else reject_frame();
     end else if(data==8'h5a || data==8'hc3)begin
      state<=LEGACY;legacy_left<=(data==8'h5a)?10:18;
     end else state<=(data==8'ha5)?HEADER:IDLE;
    end
    LEGACY:begin
     if(legacy_left==1)state<=IDLE;else legacy_left<=legacy_left-1'b1;
    end
    TUNNEL:begin
     if(data==8'ha5)begin flush_lidar();state<=HEADER;escaped_byte<=0;end
     else if(escaped_byte)begin
      escaped_byte<=0;
      if(data==8'h85 || data==8'h5d || data==8'h5e)accept_byte(decoded);
      else reject_frame();
     end else if(data==8'h7d)escaped_byte<=1;
     else if(data==8'h7e)begin
      state<=IDLE;
      if(position!=6+{1'b0,frame_length} || crc!=received_crc)flush_lidar();
      else if(frame_flags==1)flush_lidar();
      else if(have_seq && frame_seq!=last_seq+8'd1)begin
       flush_lidar();have_seq<=1;last_seq<=frame_seq;
      end else begin
       ready[fill_bank]<=1;
       if(fill_bank)length1<=frame_length;else length0<=frame_length;
       have_seq<=1;last_seq<=frame_seq;
      end
     end else accept_byte(data);
    end
    DROP:begin
     if(data==8'ha5)begin state<=HEADER;escaped_byte<=0;end
     else if(data==8'h7e)state<=IDLE;
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
