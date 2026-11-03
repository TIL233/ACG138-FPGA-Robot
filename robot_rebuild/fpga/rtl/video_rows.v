`timescale 1ns/1ps
// Two owned line buffers. Producer never overwrites a published line. An overrun
// abandons the remainder of the frame; sequence + row protect the host from mixing.
module rb_video_rows #(parameter WIDTH=640,HEIGHT=480,WARMUP=10)(
 input pclk,prst_n,input vsync,href,input [7:0] data,
 input txclk,trst_n,input req,done,
 output reg start,output reg [7:0] payload,
 output reg [15:0] dropped_frames
);
 reg [15:0] memory[0:2047]; // 2 x 1024 words, synchronous SDP -> block RAM
 reg bank,accept,drop_frame,frame_started,vs_d,href_d;
 reg [3:0] warm;
 reg [12:0] bytes;
 reg [7:0] high_byte;
 reg [15:0] row,frame_no;
 reg [1:0] publish,ack,ack1,ack2,pub1,pub2;
 reg [15:0] row0,row1,frame0,frame1;
 wire bank_free=(publish[bank]==ack2[bank]);
 // RAM port A: no reset or asynchronous reads on the RAM itself.
 always @(posedge pclk)
  if(prst_n && href && href_d && accept && !drop_frame && bytes[0] && bytes<2*WIDTH)
   memory[{bank,bytes[10:1]}]<={high_byte,data};
 always @(posedge pclk or negedge prst_n) begin
  if(!prst_n) begin
   bank<=0;accept<=0;drop_frame<=1;frame_started<=0;vs_d<=0;href_d<=0;warm<=0;bytes<=0;
   high_byte<=0;row<=0;frame_no<=0;publish<=0;ack1<=0;ack2<=0;
   row0<=0;row1<=0;frame0<=0;frame1<=0;dropped_frames<=0;
  end else begin
   ack1<=ack;ack2<=ack1;vs_d<=vsync;href_d<=href;
   if(vsync && !vs_d) begin
    if(frame_started && drop_frame && warm==WARMUP) dropped_frames<=dropped_frames+1;
    frame_no<=frame_no+1;row<=0;bytes<=0;accept<=0;drop_frame<=0;frame_started<=1;
    if(warm<WARMUP) warm<=warm+1;
   end else if(!vsync && href) begin
    if(!href_d) begin
     bytes<=1;high_byte<=data;
     accept<=bank_free && !drop_frame && warm==WARMUP && row<HEIGHT;
     if(!bank_free) drop_frame<=1;
    end else begin
     if(bytes<8191) bytes<=bytes+1;
     if(!bytes[0]) high_byte<=data;
     if(bytes>=2*WIDTH) drop_frame<=1;
    end
   end else if(href_d && !href) begin
    accept<=0;row<=row+1;
    if(accept && !drop_frame && bytes==2*WIDTH) begin
     publish[bank]<=~publish[bank];
     if(bank) begin row1<=row;frame1<=frame_no;end
     else begin row0<=row;frame0<=frame_no;end
     bank<=~bank;
    end else if(warm==WARMUP) drop_frame<=1;
   end
  end
 end

 reg next_bank,busy,payload_is_header;
 reg [5:0] ifg;
 reg [15:0] index,send_row,send_frame;
 reg [15:0] q;
 wire [15:0] read_pixel=(index>=15)?((index-15)>>1):16'd0;
 // Advance the synchronous prefetch only when the payload consumer advances.
 // In particular an odd (high-byte) index points at the next pixel: loading
 // that word during a req pause would replace the high byte still pending.
 always @(posedge txclk)
  if(!busy || req) q<=memory[{next_bank,read_pixel[9:0]}];
 always @(posedge txclk or negedge trst_n) begin
  if(!trst_n) begin pub1<=0;pub2<=0;ack<=0;next_bank<=0;busy<=0;start<=0;payload_is_header<=1;
   ifg<=0;index<=0;send_row<=0;send_frame<=0;end
  else begin
   pub1<=publish;pub2<=pub1;start<=0;
   if(ifg!=0) ifg<=ifg-1;
   if(!busy && ifg==0 && pub2[next_bank]!=ack[next_bank]) begin
    busy<=1;index<=0;start<=1;payload_is_header<=1;
    send_row<=next_bank?row1:row0;send_frame<=next_bank?frame1:frame0;
   end
   if(busy && req) begin
    index<=index+1;
    // Predecode the header/pixel boundary on the consumer's advance edge.
    // The current requested byte is still selected before this register
    // changes, so the same-edge req/data contract gains no pipeline latency.
    if(payload_is_header && index[3:0]==4'd15)payload_is_header<=0;
   end
   if(busy && done) begin
    ack[next_bank]<=pub2[next_bank];next_bank<=~next_bank;busy<=0;ifg<=32;
   end
  end
 end
 // 16-byte header: RBV1, frame,row,width,height (u16LE), format=1,flags=0,row_bytes.
 // A registered region bit keeps the full index/address range decode out of
 // the 125MHz payload-to-UDP path. Both it and index hold during req pauses.
 always @* begin
  payload=index[0]?q[15:8]:q[7:0];
  if(payload_is_header)case(index[3:0])
   0:payload="R";1:payload="B";2:payload="V";3:payload="1";
   4:payload=send_frame[7:0];5:payload=send_frame[15:8];
   6:payload=send_row[7:0];7:payload=send_row[15:8];
   8:payload=WIDTH%256;9:payload=WIDTH/256;
   10:payload=HEIGHT%256;11:payload=HEIGHT/256;
   12:payload=1;13:payload=0;
   14:payload=(WIDTH*2)%256;15:payload=(WIDTH*2)/256;
   default:payload=0;
  endcase
 end
endmodule
