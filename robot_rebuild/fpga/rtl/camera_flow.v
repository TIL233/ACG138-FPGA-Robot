`timescale 1ns/1ps
// Diagnostic-only counters. Gray-coded source registers cross into the 50MHz
// reference domain. Values are deltas per window, never control inputs.
module rb_camera_flow #(parameter CLK_HZ=50000000)(
 input clk,rst_n,pclk,prst_n,vsync,href,
 input txclk,trst_n,start,done,input [15:0] dropped,
 output reg [159:0] flow);
 reg request;
 (* ASYNC_REG="TRUE" *) reg [1:0] req_sync,ack_sync;
 reg acknowledge;
 reg [15:0] line_bytes,last_line,max_line;
 reg [31:0] line_snapshot,line_received;
 function [15:0] gray(input [15:0] x);begin gray=x^(x>>1);end endfunction
 function [15:0] binary(input [15:0] x);
  integer i;begin binary[15]=x[15];for(i=14;i>=0;i=i-1)binary[i]=binary[i+1]^x[i];end
 endfunction
 reg [7:0] prescale;reg [15:0] clocks,lines,frames;
 reg [15:0] cg,lg,fg,dg;reg vd,hd;
 reg [3:0] hp,vp;reg [15:0] hc,vc,hg,vg;
 always @(posedge pclk or negedge prst_n)begin
  if(!prst_n)begin prescale<=0;clocks<=0;lines<=0;frames<=0;
   cg<=0;lg<=0;fg<=0;dg<=0;vd<=0;hd<=0;
   hp<=0;vp<=0;hc<=0;vc<=0;hg<=0;vg<=0;
   req_sync<=0;acknowledge<=0;line_bytes<=0;last_line<=0;max_line<=0;line_snapshot<=0;end
  else begin
   prescale<=prescale+1'b1;
   req_sync<={req_sync[0],request};
   if(href)begin
    if(!hd)line_bytes<=1;
    else if(line_bytes!=65535)line_bytes<=line_bytes+1'b1;
   end
   if(hd && !href)begin
    last_line<=line_bytes;
    if(line_bytes>max_line)max_line<=line_bytes;
   end
   if(req_sync[1]!=acknowledge)begin
    line_snapshot<={max_line,last_line};max_line<=0;acknowledge<=req_sync[1];
   end
   if(href)begin hp<=hp+1'b1;if(hp==15)begin hc<=hc+1'b1;hg<=gray(hc+16'd1);end end
   if(vsync)begin vp<=vp+1'b1;if(vp==15)begin vc<=vc+1'b1;vg<=gray(vc+16'd1);end end
   if(prescale==255)begin clocks<=clocks+1'b1;cg<=gray(clocks+16'd1);end
   vd<=vsync;hd<=href;
   if(vsync && !vd)begin frames<=frames+1'b1;fg<=gray(frames+16'd1);end
   if(href && !hd)begin lines<=lines+1'b1;lg<=gray(lines+16'd1);end
   dg<=gray(dropped);
  end
 end
 reg [15:0] starts,finishes,sg,eg;reg sd,ed;
 always @(posedge txclk or negedge trst_n)begin
  if(!trst_n)begin starts<=0;finishes<=0;sg<=0;eg<=0;sd<=0;ed<=0;end
  else begin
   sd<=start;ed<=done;
   if(start && !sd)begin starts<=starts+1'b1;sg<=gray(starts+16'd1);end
   if(done && !ed)begin finishes<=finishes+1'b1;eg<=gray(finishes+16'd1);end
  end
 end
 (* ASYNC_REG="TRUE" *) reg [127:0] sync1,sync2;
 reg [15:0] ph,pv;
 wire [15:0] nh=binary(sync2[111:96]),nv=binary(sync2[127:112]);
 reg [15:0] pc,pl,pf,ps,pe,pd;reg [31:0] timer;
 wire [15:0] nc=binary(sync2[15:0]),nl=binary(sync2[31:16]),nf=binary(sync2[47:32]);
 wire [15:0] ns=binary(sync2[63:48]),ne=binary(sync2[79:64]),nd=binary(sync2[95:80]);
 wire [15:0] dc=nc-pc,dl=nl-pl,df=nf-pf,ds=ns-ps,de=ne-pe,dd=nd-pd;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sync1<=0;sync2<=0;pc<=0;pl<=0;pf<=0;ps<=0;pe<=0;pd<=0;ph<=0;pv<=0;timer<=0;flow<=0;
   request<=0;ack_sync<=0;line_received<=0;end
  else begin
   sync1<={vg,hg,dg,eg,sg,fg,lg,cg};sync2<=sync1;
   ack_sync<={ack_sync[0],acknowledge};
   if(ack_sync[1]==request)line_received<=line_snapshot;
   if(timer>=CLK_HZ/10-1)begin
    request<=!request;
    timer<=0;flow<={line_received,nv-pv,nh-ph,dd,de,ds,df,dl,dc};pc<=nc;pl<=nl;pf<=nf;ps<=ns;pe<=ne;pd<=nd;ph<=nh;pv<=nv;
   end else timer<=timer+1'b1;
  end
 end
endmodule
