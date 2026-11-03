`timescale 1ns/1ps
// Real 50MHz parser timing and normal production sector thresholds. Only the
// host byte source is accelerated; the bridge still emits 115200 8N1 cadence.
module tb_usb_lidar_stream;
 reg clk=0;always #10 clk=~clk;reg rst_n=0,valid=0;reg [7:0] data=0;
 wire [7:0] cd,ld;wire cv,lv,clear,update,fresh;wire [2:0] mask;
 wire [15:0] front,left,right,id,good,bad;
 rb_lidar_bridge_rx bridge(clk,rst_n,data,valid,cd,cv,ld,lv,clear);
 rb_lidar_stream core(clk,rst_n,ld,lv,clear,update,fresh,mask,front,left,right,id,good,bad);
 wire armed;rb_command_rx commands(.clk(clk),.rst_n(rst_n),.data(cd),.valid(cv),.armed(armed),
  .command(),.speed(),.update(),.fresh(),.mode_nav(),.mode_follow());
 reg [7:0] payload[0:127],raw[0:511];reg [7:0] seq=0;
 integer bytes=0,controls=0,clears=0,updates=0,points=0,i,j,k,b0,p0,g0;
 always @(posedge clk)begin
  if(lv)bytes=bytes+1;if(cv)controls=controls+1;if(clear)clears=clears+1;
  if(update)updates=updates+1;if(core.pv)points=points+1;
  if(armed)$fatal(1,"lidar traffic armed motors");
 end
 function [15:0] cb(input [15:0] crc,input [7:0] b);
  reg [15:0] c;integer t;begin c=crc^{b,8'd0};for(t=0;t<8;t=t+1)c=c[15]?(c<<1)^16'h1021:c<<1;cb=c;end
 endfunction
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task send(input [7:0] b);begin @(negedge clk);data=b;valid=1;@(negedge clk);valid=0;end endtask
 task escaped(input [7:0] b);begin
  if(b==8'ha5 || b==8'h7d || b==8'h7e)begin send(8'h7d);send(b^8'h20);end else send(b);
 end endtask
 task frame(input integer n,input bit corrupt,input bit reset_frame);
  reg [15:0] c;integer t;begin
   c=cb(cb(cb(cb(16'hffff,1),seq),reset_frame),n);
   send(8'ha5);send(8'hb7);escaped(1);escaped(seq);escaped(reset_frame);escaped(n);
   for(t=0;t<n;t=t+1)begin escaped(payload[t]);c=cb(c,payload[t]);end
   escaped(c[7:0]^corrupt);escaped(c[15:8]);send(8'h7e);seq=seq+1'b1;ticks(10);
  end
 endtask
 task reset_stream;begin frame(0,0,1);ticks(10);end endtask
 // These packet checksums are literal hand-checked fixtures; their expected
 // ranges are independent of the tunnel checksum builder above.
 task fixture(input integer which);
  integer t;reg [15:0] a,w,cs;integer count;begin
   case(which)
    0:begin count=40;a=16'h0001;w=16'h07d0;cs=16'h7dab;end // 0deg,500mm,start
    1:begin count=40;a=16'h2d01;w=16'h0c80;cs=16'h7daa;end // 90deg,800mm
    2:begin count=40;a=16'h8701;w=16'h12c0;cs=16'h7daa;end // 270deg,1200mm
    default:begin count=1;a=16'h0001;w=16'h07d0;cs=16'h537b;end
   endcase
   raw[0]=8'haa;raw[1]=8'h55;raw[2]=(which==0 || which==3);raw[3]=count;
   raw[4]=a[7:0];raw[5]=a[15:8];raw[6]=a[7:0];raw[7]=a[15:8];raw[8]=cs[7:0];raw[9]=cs[15:8];
   for(t=0;t<count;t=t+1)begin raw[10+2*t]=w[7:0];raw[11+2*t]=w[15:8];end
  end
 endtask
 task raw_frame(input integer offset,input integer n);
  integer t;begin for(t=0;t<n;t=t+1)payload[t]=raw[offset+t];frame(n,0,0);ticks(n*4340+100);end
 endtask
 task revolution;
  begin fixture(0);raw_frame(0,90);fixture(1);raw_frame(0,90);fixture(2);raw_frame(0,90);fixture(3);raw_frame(0,12);end
 endtask
 initial begin
  ticks(5);rst_n=1;ticks(5);
  revolution();
  if(!fresh || mask!=7 || front!=500 || left!=1200 || right!=800 || id!=1 || good!=4 || bad!=0 || controls!=0)
   $fatal(1,"literal X3 scan/sector mismatch fresh=%0d mask=%0d F/L/R=%0d/%0d/%0d good/bad=%0d/%0d",fresh,mask,front,left,right,good,bad);
  // A corrupt tunnel immediately invalidates published sectors and discards
  // both pending bytes and a partially assembled raw packet.
  fixture(3);for(i=0;i<12;i=i+1)payload[i]=raw[i];frame(12,1,0);ticks(100);
  if(fresh || mask || id || good)$fatal(1,"bad tunnel retained old published scan");
  revolution();if(!fresh || id!=1)$fatal(1,"corrupt tunnel reconnect failed");
  // A gap drops its frame; the next contiguous sequence resumes cleanly.
  seq=seq+1'b1;frame(12,0,0);ticks(100);
  if(fresh || mask || good)$fatal(1,"sequence gap did not clear old scan");
  revolution();if(!fresh || id!=1)$fatal(1,"sequence gap recovery failed");
  // Parser must tolerate normal USB chunk scheduling gaps above physical5ms.
  reset_stream();fixture(3);raw_frame(0,6);ticks(1000000);raw_frame(6,6);
  if(good!=1 || bad!=0)$fatal(1,"20ms USB chunk gap invalidated raw packet");
  reset_stream();fixture(3);raw_frame(0,6);ticks(5050000);raw_frame(6,6);
  if(good!=0 || bad!=1)$fatal(1,"100ms raw assembly expiry failed");
  raw_frame(0,12);if(good!=1)$fatal(1,"raw parser did not recover after expiry");
  // RESET while a valid packet is already computing/emitting must suppress
  // all old point emissions, not merely empty transport receive storage.
  reset_stream();fixture(0);b0=bytes;for(i=0;i<90;i=i+1)payload[i]=raw[i];frame(90,0,0);
  wait(core.pv);reset_stream();p0=points;ticks(10000);
  if(points!=p0 || fresh || mask || good || id)$fatal(1,"old packet emitted after RESET");
  revolution();if(!fresh)$fatal(1,"explicit reconnect failed");
  ticks(17550000);
  if(fresh || mask)$fatal(1,"USB disconnect did not expire sectors in350ms");
  fixture(3);raw_frame(0,12);
  if(fresh)$fatal(1,"old revolution revived after reconnect");
  $display("PASS USB lidar 50MHz: literal full scan/3sectors, corruption, sequence, RESET mid-emission,20ms scheduling,100ms packet expiry,350ms freshness");$finish;
 end
 initial begin #1000000000;$fatal(1,"USB stream timeout");end
endmodule
