`timescale 1ns/1ps
// Real packets and parser, accelerated millisecond clock. The independent
// legacy stream comparison catches any change to original sector semantics.
module tb_lidar_scan_map;
 localparam HZ=100000;
 reg clk=0; always #5 clk=~clk;
 reg rst_n=0,valid=0,clear=0; reg [7:0] data=0;
 wire update,fresh,map_update,map_fresh; wire [2:0] sector_valid;
 wire [15:0] front_mm,left_mm,right_mm,scan_id,good_packets,bad_packets;
 wire [575:0] map_ranges; wire [35:0] map_known;
 wire [15:0] map_scan_id,map_age_ms,map_span_ms;
 wire lu,lf; wire [2:0] lv; wire [15:0] lfront,lleft,lright,lid,lg,lb;
 wire invalid_offset_fresh;
 rb_lidar_scan_map #(.CLK_HZ(HZ),.FRONT_ZERO_Q6(23040)) invalid_offset(
  .clk(clk),.rst_n(rst_n),.data(data),.valid(valid),.clear(clear),
  .update(),.fresh(),.sector_valid(),.front_mm(),.left_mm(),.right_mm(),
  .scan_id(),.good_packets(),.bad_packets(),.map_update(),.map_fresh(invalid_offset_fresh),
  .map_ranges(),.map_known(),.map_scan_id(),.map_age_ms(),.map_span_ms());
 rb_lidar_scan_map #(.CLK_HZ(HZ),.FRONT_ZERO_Q6(320)) dut(
  .clk(clk),.rst_n(rst_n),.data(data),.valid(valid),.clear(clear),
  .update(update),.fresh(fresh),.sector_valid(sector_valid),
  .front_mm(front_mm),.left_mm(left_mm),.right_mm(right_mm),
  .scan_id(scan_id),.good_packets(good_packets),.bad_packets(bad_packets),
  .map_update(map_update),.map_fresh(map_fresh),.map_ranges(map_ranges),
  .map_known(map_known),.map_scan_id(map_scan_id),.map_age_ms(map_age_ms),.map_span_ms(map_span_ms));
 rb_lidar_stream #(.CLK_HZ(HZ),.FRONT_ZERO_Q6(320)) legacy(
  .clk(clk),.rst_n(rst_n),.data(data),.valid(valid),.clear(clear),
  .update(lu),.fresh(lf),.sector_valid(lv),.front_mm(lfront),.left_mm(lleft),
  .right_mm(lright),.scan_id(lid),.good_packets(lg),.bad_packets(lb));
 integer publications=0,i,saved_id,phase;
 integer clock_cycles=0,raw_start_cycle=-1,completed_start_cycle=-1,completed_span_cycles=0;
 reg [575:0] saved_ranges;
 reg [35:0] saved_known; reg was_update=0;
 // Independent age reference uses the original parser acceptance edge, not
 // the delayed map start. This catches any latency that makes data younger.
 always @(posedge clk)begin
  clock_cycles=clock_cycles+1;
  if(!rst_n || clear || dut.err)begin raw_start_cycle=-1;completed_start_cycle=-1;end
  else if(dut.pv && dut.start)begin
   completed_start_cycle=raw_start_cycle;
   completed_span_cycles=clock_cycles-raw_start_cycle;
   raw_start_cycle=clock_cycles;
  end
  #1;
  if(rst_n && !clear && map_age_ms !== ((dut.age_ms==16'hffff)?16'hffff:
       dut.age_ms+((dut.age_phase!=0)?16'd1:16'd0)))
   $fatal(1,"registered age differs from the exact floor/phase ceiling reference");
  if(rst_n && !clear && !dut.err && map_update)begin
   if(completed_start_cycle<0 || map_age_ms*(HZ/1000)<clock_cycles-completed_start_cycle)
    $fatal(1,"map age omitted parser-to-map pipeline latency");
   if(map_span_ms*(HZ/1000)<completed_span_cycles)$fatal(1,"map span became younger than raw revolution");
  end
 end
 always @(negedge clk) if(rst_n) begin
  if(invalid_offset_fresh)$fatal(1,"invalid angle offset published a usable map");
  if({update,fresh,sector_valid,front_mm,left_mm,right_mm,scan_id,good_packets,bad_packets}
    !=={lu,lf,lv,lfront,lleft,lright,lid,lg,lb}) $fatal(1,"legacy outputs changed");
  if(map_update) publications=publications+1;
  if(map_update && was_update) $fatal(1,"map_update longer than one cycle");
  if(dut.err && (map_fresh || map_known)) $fatal(1,"parser error did not immediately withdraw map");
  if(map_age_ms>350 && (map_fresh || map_known)) $fatal(1,"oldest map age over350ms retained validity");
  was_update=map_update;
 end
 task ticks(input integer n); repeat(n) begin @(posedge clk); #1; end endtask
 task send(input [7:0] b); begin @(negedge clk); data=b;valid=1;@(negedge clk);valid=0;end endtask
 // Bin i starts at raw angle 5 + i*10 degrees. For n=3 the parser
 // interpolates offsets 0,213,426: one direct sample in each sub-bin.
 task packet(input bit start,input integer a,input integer n,input integer mm,input integer flags,input bit corrupt);
  reg [15:0] angle,last_angle,word_,sum; integer k;
  begin
   angle=(a*2)|1; last_angle=(n==3)?((((a+426)%23040)*2)|1):angle;word_=(mm*4)|flags;
   sum=16'h55aa ^ ((n<<8)|start) ^ angle ^ last_angle;
   for(k=0;k<n;k=k+1)sum=sum^word_;
   if(corrupt)sum=sum^1;
   send(8'haa);send(8'h55);send(start);send(n);
   send(angle[7:0]);send(angle[15:8]);send(last_angle[7:0]);send(last_angle[15:8]);
   send(sum[7:0]);send(sum[15:8]);
   for(k=0;k<n;k=k+1)begin send(word_[7:0]);send(word_[15:8]);end
   ticks(40+2*n);
  end
 endtask
 task begin_scan(input integer base); packet(1,320,3,base,0,0);endtask
 task rest_scan(input integer base,input integer poisoned);
  integer b; begin
   for(b=1;b<36;b=b+1)begin
    packet(0,320+b*640,3,base+b,0,0);
    if(b==poisoned)packet(0,320+b*640,1,100,0,0);
   end
  end
 endtask
 task invalidate; begin @(negedge clk);clear=1;#1;
  if(map_fresh || map_known) $fatal(1,"clear not immediate");
  ticks(2);@(negedge clk);clear=0;ticks(2);end endtask
 task expect_all(input integer base);
  integer b; begin
   if(!map_fresh || map_known!==36'hfffffffff) $fatal(1,"completed 108-point scan base=%0d not published atomically: fresh=%b known=%h",base,map_fresh,map_known);
   for(b=0;b<36;b=b+1)if(map_ranges[b*16+:16]!==base+((b==0 || b==35)?0:b-1))$fatal(1,"bin %0d neighbor-conservative minimum wrong: got %0d",b,map_ranges[b*16+:16]);
   if(map_age_ms<map_span_ms || map_span_ms==0 || map_span_ms>=350)$fatal(1,"age omits completed span");
  end
 endtask
 initial begin
  ticks(4);rst_n=1;ticks(4);
  if(map_fresh || map_known || map_scan_id) $fatal(1,"reset map valid");
  begin_scan(1000);rest_scan(1000,-1);
  if(map_fresh || publications) $fatal(1,"partial first revolution published");
  begin_scan(2000);expect_all(1000);
  saved_id=map_scan_id;saved_ranges=map_ranges;saved_known=map_known;
  rest_scan(2000,7);
  if(map_ranges!==saved_ranges || map_known!==saved_known || map_scan_id!=saved_id)$fatal(1,"in-flight revolution tore published snapshot");
  begin_scan(3000);
  if(!map_fresh || map_known[7] || map_known!=(36'hfffffffff ^ (36'd1<<7)))$fatal(1,"invalid distance did not poison entire bin");
  for(i=0;i<36;i=i+1)if(i!=7 && map_ranges[i*16+:16]!=2000+((i==0 || i==35)?0:i-1))$fatal(1,"cross-revolution range mixing");
  if(map_ranges[7*16+:16]!=16'hffff)$fatal(1,"unknown bin range not sentinel");
  packet(0,320,3,4000,0,1);
  if(map_fresh || map_known)$fatal(1,"bad checksum retained published map");
  saved_id=map_scan_id;rest_scan(3000,-1);begin_scan(4000);
  if(map_scan_id!=saved_id || map_fresh)$fatal(1,"dirty revolution published");
  rest_scan(4000,-1);begin_scan(5000);expect_all(4000);
  invalidate();begin_scan(6000);rest_scan(6000,-1);begin_scan(7000);expect_all(6000);
  // Expiry includes scan acquisition time, not just time since publication.
  ticks((351-map_age_ms)*100);
  if(map_fresh || map_known || map_age_ms<=350)$fatal(1,"map did not expire by total age351ms");
  saved_id=map_scan_id;begin_scan(7500);
  if(map_fresh || map_scan_id!=saved_id)$fatal(1,"expired accumulation revived old revolution");
  rest_scan(7500,-1);begin_scan(7600);expect_all(7500);
  invalidate();
  // Fewer than 100 total points is never a publishable revolution.
  packet(1,320,99,1000,0,0);begin_scan(2000);
  if(map_fresh || map_scan_id)$fatal(1,"99-point revolution accepted");
  // Packet assembly gap must invalidate an existing snapshot too.
  invalidate();begin_scan(1000);rest_scan(1000,-1);begin_scan(2000);expect_all(1000);
  send(8'haa);send(8'h55);ticks(10010);
  if(map_fresh || map_known)$fatal(1,"packet gap did not revoke snapshot");
  // Clear during the real parser's buffered point emission discards both
  // its remaining points and all previously accumulated map coverage.
  invalidate();begin_scan(1000);rest_scan(1000,-1);begin_scan(2000);expect_all(1000);
  fork
   packet(0,320,100,700,0,0);
   begin wait(dut.pv);invalidate();end
  join
  if(map_fresh || map_known || map_scan_id || good_packets)$fatal(1,"clear mid-emission retained old scan state");
  begin_scan(3000);rest_scan(3000,-1);begin_scan(4000);expect_all(3000);
  // A checked packet may be interrupted by an illegal byte while its first
  // start point is still in the map pipeline. Do not publish the preceding
  // revolution transiently, or let that point resurrect the work bank.
  invalidate();begin_scan(1000);rest_scan(1000,-1);saved_id=publications;
  fork
   packet(1,320,100,600,0,0);
   begin wait(dut.pv && dut.start);send(8'h00);end
  join
  if(map_fresh || map_known || map_scan_id || publications!=saved_id)
   $fatal(1,"packet error during start pipeline published or revived a snapshot");
  // A new, clean start and complete revolution are required for recovery.
  begin_scan(2500);rest_scan(2500,-1);begin_scan(3500);expect_all(2500);
  // Clear before normalization, with normalized payload pending, and with
  // decoded payload pending. None may reach the accumulator after clear.
  for(phase=0;phase<3;phase=phase+1)begin
   invalidate();begin_scan(1000);rest_scan(1000,-1);
   fork
    packet(1,320,100,600,0,0);
    begin wait(dut.pv && dut.start);ticks(phase);invalidate();end
   join
   if(map_fresh || map_known || map_scan_id || good_packets)
    $fatal(1,"clear phase%0d retained an in-flight map point",phase);
  end
  begin_scan(2500);rest_scan(2500,-1);begin_scan(3500);expect_all(2500);
  invalidate();begin_scan(1000);rest_scan(1000,-1);
  // A bad interference sample poisons the bin even after good samples.
  packet(0,320,1,999,2,0);begin_scan(2000);
  if(!map_fresh || map_known[0] || map_ranges[15:0]!=16'hffff)$fatal(1,"interference bin remained known");
  invalidate();
  // Repeated samples at one angle must not manufacture direct coverage;
  // a close return spreads ranges but cannot make either neighbor known.
  packet(1,320,4,1000,0,0);rest_scan(1000,-1);begin_scan(2000);
  if(!map_fresh || map_known[0] || !map_known[1] || !map_known[35])$fatal(1,"direct sub-bin coverage not enforced");
  invalidate();packet(1,320,100,333,0,0);begin_scan(2000);
  if(!map_fresh || map_known)$fatal(1,"diffusion manufactured angular coverage");
  ticks(6600000);
  if(map_age_ms!=65535 || map_fresh || map_known)$fatal(1,"age did not saturate");
  $display("PASS lidar_scan_map: 36 bins atomic, legacy equivalence, thresholds, poison, errors, clear, gap, expiry, cross-scan, saturated age");$finish;
 end
 initial begin #100000000;$fatal(1,"test timeout");end
endmodule
