`timescale 1ns/1ps
// Decode the actual FPGA UART of all profiles. Change the sensor snapshot in
// the middle of a C9 transmission to catch torn distances, age or valid bits.
module lidar_map_wire_case #(parameter TAKEOVER=1, DISPLAY_ONLY=0)(input clk, rst_n, output reg done=0);
 wire tx, valid;
 wire [7:0] data;
 reg fresh=1;
 reg [15:0] scan_id=16'h1234, age=16'd121;
 reg [35:0] known=36'h98124abde;
 reg [575:0] ranges;
 robot_top #(.CLK_HZ(1000000), .LIDAR_CONTROL(TAKEOVER),
             .LIDAR_ADVANCED_CONTROL(TAKEOVER), .LIDAR_MODEL_VALIDATED(TAKEOVER),
             .LIDAR_MAP_DISPLAY(DISPLAY_ONLY)) dut(
   .clk50m(clk), .uart_tx(tx), .estop_n(1'b1), .uart_rx(1'b1), .imu_rx(1'b1),
   .ultra_l_rx(1'b1), .ultra_r_rx(1'b1), .ir_rx(1'b1), .lidar_rx(1'b1),
   .enc_l_a(1'b0), .enc_l_b(1'b0), .enc_r_a(1'b0), .enc_r_b(1'b0),
   .cam1_pclk(1'b0), .cam1_href(1'b0), .cam1_vsync(1'b0), .cam1_data(8'd0));
 rb_uart_rx #(.CLK_HZ(1000000)) receiver(clk,rst_n,tx,data,valid);

 function automatic [15:0] range_for(input integer i, input [35:0] mask);
  range_for = mask[i] ? 16'd544+i*43 : 16'hffff;
 endfunction
 initial begin
  int i;
  for(i=0;i<36;i++) ranges[i*16+:16]=range_for(i,known);
  force dut.rst_n=rst_n;
  force dut.lidar_map_fresh=fresh;
  force dut.lidar_map_scan_id=scan_id;
  force dut.lidar_map_age=age;
  force dut.lidar_map_known=known;
  force dut.lidar_map_ranges=ranges;
 end
 reg [7:0] frame[0:89];
 integer offset=0, length=0, maps=0, nav=0, motion=0, planning=0;
 string trace_path;
 always @(posedge clk) if(valid) begin : check_wire
  reg [15:0] crc;
  int i,j;
  frame[offset]=data;
  if(offset==0 && data!==8'h5a) $fatal(1,"UART frame lost sync");
  if(offset==1) case(data)
   8'ha5:length=28; 8'hc3:begin length=31;nav++;end
   8'hd4:length=36; 8'he5:begin length=46;motion++;end
   8'hf6:length=38; 8'hb7:length=26;
   8'hc8:begin length=46;planning++;end
   8'hc9:begin
    length=90;
    if(!TAKEOVER && !DISPLAY_ONLY)$fatal(1,"Original observation image emitted a map");
    if(maps==0)begin
     fresh=0; known=0; scan_id=16'h1235; age=16'hffff; ranges={576{1'b1}};
    end
   end
   default:$fatal(1,"Unexpected UART type %h",data);
  endcase
  offset++;
  if(offset==length)begin
   if(frame[1]==8'hc9)begin
    crc=16'hffff;
    for(i=2;i<86;i++)begin
     crc=crc^{frame[i],8'h00};
     for(j=0;j<8;j++) crc=crc[15]?(crc<<1)^16'h1021:(crc<<1);
    end
    if(frame[2]!==1 || {frame[87],frame[86]}!==crc || {frame[88],frame[89]}!==16'h0d0a)
     $fatal(1,"C9 version, CRC or terminator");
    if(maps==0)begin
     if(frame[4]!==1 || {frame[6],frame[5]}!==16'h1234 || {frame[8],frame[7]}!==121 ||
        {frame[13],frame[12],frame[11],frame[10],frame[9]}!==40'h098124abde)
      $fatal(1,"C9 snapshot metadata");
     for(i=0;i<36;i++)if({frame[15+i*2],frame[14+i*2]}!==range_for(i,36'h98124abde))
      $fatal(1,"C9 mixed/reversed/missing range at %0d",i);
     if((TAKEOVER && $value$plusargs("MAP_HEX=%s",trace_path)) ||
        (DISPLAY_ONLY && $value$plusargs("DISPLAY_HEX=%s",trace_path)))begin
      int fd;
      fd=$fopen(trace_path,"w");
      if(!fd)$fatal(1,"Unable to write MAP_HEX fixture");
      for(i=0;i<90;i++)$fwrite(fd,"%02x",frame[i]);
      $fwrite(fd,"\n");$fclose(fd);
     end
    end else if(frame[4]!==0 || {frame[6],frame[5]}!==16'h1235 ||
                {frame[8],frame[7]}!==16'hffff || {frame[13],frame[12],frame[11],frame[10],frame[9]}!==0)
     $fatal(1,"C9 stale snapshot did not propagate");
    maps++;
   end
   if(DISPLAY_ONLY && frame[1]==8'hc8 &&
      (frame[4][5]!==1'b0 || frame[4][2]!==1'b0 || frame[5][5]!==1'b0))
    $fatal(1,"Display-only image advertises model validation or wheel takeover");
   if(DISPLAY_ONLY && frame[1]==8'hb7 && frame[4][5]!==1'b0)
    $fatal(1,"Display-only image enabled legacy lidar control");
   offset=0; length=0;
  end
 end
 initial begin
  @(posedge rst_n); repeat(500000) @(posedge clk);
  if(((TAKEOVER || DISPLAY_ONLY) && (maps<2 || maps>3)) || nav<4 || motion<20 || planning<4)
   $fatal(1,"Map rate/telemetry fairness profile=%0d maps=%0d nav=%0d motion=%0d planning=%0d",TAKEOVER,maps,nav,motion,planning);
  done=1;
 end
endmodule

module tb_lidar_map_wire;
 reg clk=0,rst_n=0; always #5 clk=~clk;
 wire active_done,observe_done,display_done;
 lidar_map_wire_case #(1) active(clk,rst_n,active_done);
 lidar_map_wire_case #(0) observe(clk,rst_n,observe_done);
 lidar_map_wire_case #(0,1) display_only(clk,rst_n,display_done);
 initial begin
  repeat(5) @(negedge clk);rst_n=1;
  wait(active_done && observe_done && display_done);
  $display("PASS C9 map: exact UART snapshot, stale map, fairness, display-only control off, legacy observation isolation");
  $finish;
 end
endmodule
