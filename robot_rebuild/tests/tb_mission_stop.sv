`timescale 1ns/1ps
// Uses the same simulation-only video primitives defined by
// tb_navigation_integration.sv; compile all benches with run_rtl.ps1.
module tb_mission_stop;
 reg clk=0;always #5 clk=~clk;
 robot_top #(.CLK_HZ(1000000)) dut(
  .clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),
  .uart_tx(),.motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),
  .motor_r_pwm(),.motor_r_in1(),.motor_r_in2());
 reg stop_valid=0,uart_update=0,uart_armed=0;
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit ok,input string why);if(!ok)$fatal(1,"%s",why);endtask
 task stop_event(input bit simultaneous_disarm);
  begin
   @(negedge clk);stop_valid=1;uart_update=simultaneous_disarm;cycles(1);
   @(negedge clk);stop_valid=0;uart_update=0;cycles(2);
  end
 endtask
 integer i;

 // Decode actual telemetry bytes, including CRC, rather than inspecting a
 // private packing register. State 7 and epoch 10 must occupy the same byte.
 reg tele_rst=0;
 wire tx,rx_valid;wire[7:0] rx_data;
 rb_telemetry #(.CLK_HZ(1000000)) tele(
  .clk(clk),.rst_n(tele_rst),.state(8'd0),.health(8'h3f),
  .command(8'd0),.speed(8'd0),.dl(16'd1000),.dr(16'd1000),
  .vl(16'd0),.vr(16'd0),.roll(16'd0),.pitch(16'd0),.yaw(16'd0),
  .fault(16'd0),.tx(tx),.nav_state(8'ha7),.nav_index(3'd0),
  .nav_count(4'd1),.nav_flags(8'hc8),.nav_x(32'sd123),
  .nav_y(-32'sd456),.nav_heading(16'sd789),.nav_ack_seq(8'd21),
  .nav_ack_status(8'd0));
 rb_uart_rx #(.CLK_HZ(1000000)) receiver(clk,tele_rst,tx,rx_data,rx_valid);
 reg [7:0] bytes[0:51];integer count=0;
 always @(posedge clk)if(rx_valid && count<52)begin bytes[count]<=rx_data;count<=count+1;end
 function[15:0] crcbyte(input[15:0] crc,input[7:0] b);
  reg[15:0] c;integer j;begin
   c=crc^{b,8'd0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;
   crcbyte=c;
  end
 endfunction
 reg[15:0] crc;
 initial begin
  force dut.rst_n=0;
  force dut.ir_key=8'h1c;force dut.ir_valid=stop_valid;force dut.ir_repeat=0;
  force dut.uart_update=uart_update;force dut.uart_armed=uart_armed;
  cycles(4);force dut.rst_n=1;cycles(4);
  check(dut.ir_stop_epoch==0,"power-up epoch must be zero");
  check(dut.nav_flags[7:6]==2'b11,"mission and navigation capabilities must both be advertised");
  stop_event(1);
  check(dut.ir_stop_epoch==1 && dut.ir_selected,"coincident IR STOP and DISARM must preserve the event");
  @(negedge clk);uart_update=1;cycles(1);@(negedge clk);uart_update=0;cycles(2);
  check(dut.ir_stop_epoch==1 && !dut.ir_selected,"later DISARM may release ownership but never erase STOP epoch");
  repeat(6)begin
   @(negedge clk);uart_update=1;cycles(1);@(negedge clk);uart_update=0;cycles(2);
  end
  check(dut.ir_stop_epoch==1,"DISARM heartbeats must not change epoch");
  for(i=0;i<14;i=i+1)stop_event(0);
  check(dut.ir_stop_epoch==15,"each decoded physical STOP increments exactly once");
  stop_event(1);
  check(dut.ir_stop_epoch==0,"STOP epoch wraps from 15 to zero");
  @(negedge clk);uart_update=1;uart_armed=1;cycles(1);
  @(negedge clk);uart_update=0;cycles(2);
  check(dut.ir_stop_epoch==0,"ARM packets must not modify STOP history");
  tele_rst=1;
  wait(count==52);cycles(2);
  check(bytes[28]==8'h5a && bytes[29]==8'hc3 && bytes[32]==8'ha7,
   "navigation wire state must carry STOP generation in its high nibble");
  check(bytes[35]==8'hc8 && bytes[50]==13 && bytes[51]==10,"navigation flags and 24-byte framing must be unchanged");
  crc=16'hffff;for(i=30;i<48;i=i+1)crc=crcbyte(crc,bytes[i]);
  check({bytes[49],bytes[48]}==crc,"navigation STOP generation must be covered by CRC");
  $display("PASS mission STOP epoch: coincident DISARM/history/heartbeat/wrap/wire CRC");
  $finish;
 end
 initial begin #3000000;$fatal(1,"mission STOP telemetry timeout");end
endmodule
