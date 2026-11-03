`timescale 1ns/1ps
module tb_video_flow;
 reg pc=0,tc=0;always #10 pc=~pc;always #4 tc=~tc;
 reg rst_n=0,vs=0,href=0;reg [7:0] data=0;
 reg req=0,done=0;wire start;wire [7:0] payload;wire [15:0] drops;
 rb_video_rows #(.WIDTH(4),.HEIGHT(3),.WARMUP(0)) dut(
  pc,rst_n,vs,href,data,tc,rst_n,req,done,start,payload,drops);
 task begin_frame;
  begin @(negedge pc);vs=1;@(negedge pc);vs=0;repeat(3) @(negedge pc);end
 endtask
 task line(input [15:0] base,input integer pixels);
  integer i;reg [15:0] word;
  begin
   for(i=0;i<pixels;i=i+1) begin
    word=base+i;
    @(negedge pc);href=1;data=word[15:8];
    @(negedge pc);data=word[7:0];
   end
   @(negedge pc);href=0;repeat(4) @(negedge pc);
  end
 endtask
 task consume(input integer expected_frame,input integer expected_row,input [15:0] base);
  integer i;reg [7:0] bytes[0:23];reg [15:0] word;
  begin
   wait(dut.busy);
   for(i=0;i<24;i=i+1) begin
    @(negedge tc);req=0;
    // Pause across the last header/first pixel boundary as well as RGB high
    // bytes. Region predecode and synchronous prefetch must advance only with
    // the consumer, while the published buffer remains owned until done.
    if(i==15 || i==16 || (i>=16 && i[0])) repeat(4) @(negedge tc);
    req=1;@(posedge tc);bytes[i]=payload;#1;
   end
   @(negedge tc);req=0;done=1;@(negedge tc);done=0;
   if({bytes[5],bytes[4]}!=expected_frame || {bytes[7],bytes[6]}!=expected_row)
    $fatal(1,"video ownership/frame metadata frame=%0d row=%0d",{bytes[5],bytes[4]},{bytes[7],bytes[6]});
   for(i=0;i<4;i=i+1) begin
    word=base+i;
    if({bytes[17+2*i],bytes[16+2*i]}!=word)
     $fatal(1,"video stalled prefetch/overwrite pixel=%0d got=%h expected=%h",i,{bytes[17+2*i],bytes[16+2*i]},word);
   end
  end
 endtask
 initial begin
  repeat(5) @(negedge pc);rst_n=1;begin_frame;
  // Hold both banks pending; a third row must drop the rest of the frame
  // without overwriting either previously published row.
  line(16'h1234,4);line(16'h2345,4);line(16'h3456,4);
  if(!dut.drop_frame) $fatal(1,"producer failed to detect buffer overrun");
  consume(1,0,16'h1234);consume(1,1,16'h2345);
  repeat(40) @(negedge pc);
  begin_frame;
  if(drops!=1) $fatal(1,"overrun must count one dropped frame");
  line(16'h4567,4);consume(2,0,16'h4567);
  // A short row is never published, and the rest of that frame is discarded.
  line(16'h5678,3);line(16'h6789,4);repeat(40) @(negedge pc);
  if(dut.busy || dut.publish!=dut.ack) $fatal(1,"short row published or pending buffer leaked");
  begin_frame;
  if(drops!=2) $fatal(1,"short row drop count");
  line(16'h789a,4);consume(3,0,16'h789a);
  line(16'h89ab,4);consume(3,1,16'h89ab);
  line(16'h9abc,4);consume(3,2,16'h9abc);
  $display("PASS video flow: consumer pauses, bank ownership, overrun/short-row drop and frame recovery");
  $finish;
 end
 initial begin #100000;$fatal(1,"test hung");end
endmodule
