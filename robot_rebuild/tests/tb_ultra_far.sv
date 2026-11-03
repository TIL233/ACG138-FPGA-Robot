`timescale 1ns/1ps
module tb_ultra_far;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,rx=1,armed=0,cmd_update=0;
 wire [15:0] distance,zero_distance;wire update,fresh,zero_update,zero_fresh;
 rb_ultrasonic #(.CLK_HZ(1000000),.TIMEOUT_MS(5)) sensor(clk,rst,rx,distance,update,fresh);
 rb_ultrasonic #(.CLK_HZ(1000000),.TIMEOUT_MS(5),.NO_ECHO_CODE(0)) zero_profile(
  clk,rst,rx,zero_distance,zero_update,zero_fresh);
 wire signed [15:0] tl,tr;wire brake;wire [3:0] state;wire [15:0] fault;
 rb_safety_control #(.CLK_HZ(1000000)) control(
  .clk(clk),.rst_n(rst),.link_ok(1'b1),.imu_ok(1'b1),.left_ok(fresh),.right_ok(1'b1),.estop_ok(1'b1),
  .armed(armed),.cmd_update(cmd_update),.command(3'd1),.speed(8'd16),
  .dl(distance),.dr(16'd2000),.left_update(update),.right_update(1'b1),.pos_l(32'd0),.pos_r(32'd0),.motor_fault(1'b0),
  .yaw(16'sd0),.yaw_update(1'b1),.target_l(tl),.target_r(tr),.brake(brake),.state(state),.fault(fault));
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task uart_byte(input [7:0] b);
 integer k;begin
  @(negedge clk);rx=0;ticks(9);
  for(k=0;k<8;k=k+1)begin @(negedge clk);rx=b[k];ticks(9);end
  @(negedge clk);rx=1;ticks(12);
 end endtask
 task packet(input [15:0] value,input bad);
 reg [7:0] sum;begin sum=8'hff+value[15:8]+value[7:0];
  uart_byte(8'hff);uart_byte(value[15:8]);uart_byte(value[7:0]);uart_byte(sum^{7'd0,bad});ticks(4);
 end endtask
 task rearm;begin
  @(negedge clk);armed=0;cmd_update=1;ticks(2);
  @(negedge clk);cmd_update=0;armed=1;ticks(3);
 end endtask
 initial begin
  ticks(4);rst=1;ticks(4);
  if(fresh)$fatal(1,"startup with no frames cannot be FAR");
  packet(16'hffff,0);
  if(!fresh || distance!=16'hfffe)$fatal(1,"checked no-echo frame becomes normalized FAR");
  rearm();
  if(brake || tl!=16 || tr!=16)$fatal(1,"FAR keeps real motion control available");
  packet(16'd5000,0);
  if(!fresh || distance!=16'hfffe || brake)$fatal(1,"checked numeric overrange allowed");
  packet(16'd150,0);ticks(3);
  if(!fresh || distance!=150 || !brake)$fatal(1,"returning close obstacle must stop immediately");
  packet(16'hffff,0);rearm();
  if(brake)$fatal(1,"explicit rearm after fresh FAR");
  packet(16'hfffe,0);ticks(3);
  if(fresh || brake)$fatal(1,"raw interference stays invalid but does not gate motion");
  packet(16'hfffd,0);
  if(fresh)$fatal(1,"reserved raw error cannot become numeric FAR");
  packet(16'hffff,0);rearm();packet(16'hffff,1);ticks(3);
  if(fresh || brake)$fatal(1,"bad checksum invalidates telemetry without gating motion");
  packet(16'hffff,0);rearm();ticks(5100);
  if(fresh || brake)$fatal(1,"UART silence stays invalid but permits motion");
  packet(0,0);
  if(fresh || !zero_fresh || zero_distance!=16'hfffe)$fatal(1,"zero-code support requires explicit module profile");
  packet(19,0);
  if(fresh || zero_fresh)$fatal(1,"positive blind-zone values remain invalid");
  $display("PASS ultrasound FAR: real UART, valid near stops, missing/interference/CRC invalid but motion allowed, zero profile");$finish;
 end
 initial begin #1000000;$fatal(1,"FAR test hung");end
endmodule
