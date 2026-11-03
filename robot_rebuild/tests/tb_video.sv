`timescale 1ns/1ps
module tb_video;
 reg pc=0,tc=0;always #10 pc=~pc;always #4 tc=~tc;
 reg rst_n=0,vs=0,href=0;reg [7:0] data=0;
 wire start,done,req;wire [7:0] payload,gmii;wire gclk,en;wire [15:0] drops;
 rb_video_rows #(.WIDTH(4),.HEIGHT(2),.WARMUP(0)) dut(pc,rst_n,vs,href,data,tc,rst_n,req,done,start,payload,drops);
 eth_udp_tx_gmii udp(.clk125m(tc),.reset_p(!rst_n),.tx_en_pulse(start),.tx_done(done),
 .dst_mac(48'hffffffffffff),.src_mac(48'h020000000002),.dst_ip(32'hc0a80003),.src_ip(32'hc0a80002),
 .dst_port(16'd6102),.src_port(16'd5000),.data_length(16'd24),.payload_req_o(req),.payload_dat_i(payload),
 .gmii_tx_clk(gclk),.gmii_txen(en),.gmii_txd(gmii));
 reg [7:0] bytes[0:199];integer count=0,frames=0,fd;reg lasten=0;
 always @(negedge tc) begin
  if(en) begin bytes[count]=gmii;count=count+1;$fwrite(fd,"%02x",gmii);end
  else if(lasten) begin
   $fwrite(fd,"\n");
   if(count!=78) $fatal(1,"Ethernet length %0d",count);
   if({bytes[50],bytes[51],bytes[52],bytes[53]}!=32'h52425631) $fatal(1,"RBV1 missing");
   if(bytes[56]!=frames || bytes[57]!=0) $fatal(1,"row metadata");
   if(bytes[66]!=8'h00 || bytes[67]!=8'hf8 || bytes[68]!=8'he0 || bytes[69]!=8'h07 ||
      bytes[70]!=8'h1f || bytes[71]!=8'h00 || bytes[72]!=8'hff || bytes[73]!=8'hff)
      $fatal(1,"RGB565 byte order or synchronous RAM prefetch");
   frames=frames+1;count=0;
  end
  lasten=en;
 end
 task pixel_byte(input [7:0] b);begin @(negedge pc);href=1;data=b;end endtask
 task line;begin
  pixel_byte(8'hf8);pixel_byte(0);pixel_byte(8'h07);pixel_byte(8'he0);
  pixel_byte(0);pixel_byte(8'h1f);pixel_byte(8'hff);pixel_byte(8'hff);
  @(negedge pc);href=0;
 end endtask
 initial begin
  fd=$fopen("ethernet.hex","w");
  repeat(5) @(negedge pc);rst_n=1;vs=1;@(negedge pc);vs=0;
  repeat(3) @(negedge pc);line;
  repeat(100) @(negedge pc);line;
  wait(frames==2);repeat(10) @(negedge tc);
  $fclose(fd);$display("PASS video: dual-clock RGB565 rows through real Ethernet transmitter");$finish;
 end
 initial begin #100000;$fatal(1,"test hung");end
endmodule
