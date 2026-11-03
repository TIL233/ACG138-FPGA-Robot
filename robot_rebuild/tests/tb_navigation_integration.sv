`timescale 1ns/1ps
// Exercise the real top-level motion arbitration. Video stays in reset via the
// simulation-only PLL model below; this bench does not validate video timing.
module tb_navigation_integration;
 reg clk=0;always #5 clk=~clk;
 wire uart_tx;
 robot_top #(.CLK_HZ(1000000)) dut(
  .clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),
  .uart_tx(uart_tx),
  .motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),
  .motor_r_pwm(),.motor_r_in1(),.motor_r_in2());
 reg [7:0] rx=0,key=0;
 reg rx_valid=0,key_valid=0;
 reg signed [31:0] pl=0,pr=0;
 reg [15:0] distance_l=1000,distance_r=1000;
 integer command_updates=0;
 always @(posedge clk) if(dut.rst_n && dut.uart_update) command_updates<=command_updates+1;
 task cycles(input integer n);repeat(n) begin @(posedge clk);#1;end endtask
 task check(input bit good,input string message);
  if(!good) $fatal(1,"%s nav=%d safety=%d cmd=%d brake=%b fault=%h",message,
   dut.nav_state,dut.state,dut.effective_command,dut.brake,dut.fault);
 endtask
 function [15:0] crcbyte(input [15:0] a,input [7:0] b);
  reg [15:0] c;integer j;begin
   c=a^{b,8'd0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;
   crcbyte=c;
  end
 endfunction
 task send(input [7:0] value);begin
  @(negedge clk);rx=value;rx_valid=1;cycles(1);
  @(negedge clk);rx_valid=0;
 end endtask
 task motion(input [7:0] seq,input bit arm,input bit nav,input bit simultaneous_ir_stop=0,input [7:0] requested_speed=8);reg [7:0] b[0:5];
  reg [15:0] crc;integer i;begin
   b[0]=1;b[1]=seq;b[2]={7'd0,arm};b[3]=arm?1:0;b[4]=requested_speed;b[5]={7'd0,nav};
   crc=16'hffff;send(8'ha5);send(8'h5a);
   for(i=0;i<6;i=i+1)begin send(b[i]);crc=crcbyte(crc,b[i]);end
   send(crc[7:0]);send(crc[15:8]);send(13);send(10);
   if(simultaneous_ir_stop)begin
    key=8'h1c;key_valid=1;cycles(1);@(negedge clk);key_valid=0;
   end
   cycles(5);
  end
 endtask
 task configure(input [7:0] seq,input [7:0] op,input [7:0] count_value);
  reg [7:0] b[0:13];reg [15:0] crc;integer i;begin
   for(i=0;i<14;i=i+1)b[i]=0;
   b[0]=1;b[1]=seq;b[2]=op;
   if(op==1)begin b[4]=8'he8;b[5]=3;end // waypoint (1000, 0) mm
   b[8]=count_value;
   crc=16'hffff;send(8'ha5);send(8'hc3);
   for(i=0;i<14;i=i+1)begin send(b[i]);crc=crcbyte(crc,b[i]);end
   send(crc[7:0]);send(crc[15:8]);send(13);send(10);cycles(5);
  end
 endtask
 task wait_drive;begin
  cycles(22000);
  check(dut.nav_active && dut.effective_command==1 && !dut.brake,
   "configured navigation must reach forward drive");
 end endtask
 initial begin
  force dut.rst_n=0;
  force dut.rx_data=rx;force dut.rx_valid=rx_valid;
  force dut.ir_key=key;force dut.ir_valid=key_valid;force dut.ir_repeat=0;
  force dut.imu_ok=1;force dut.lok=1;force dut.rok=1;
  force dut.yaw=0;force dut.imu_update=1;force dut.pos_l=pl;force dut.pos_r=pr;
  force dut.dl=distance_l;force dut.dr=distance_r;
  force dut.lfault=0;force dut.rfault=0;
  cycles(5);force dut.rst_n=1;cycles(200010);
  check(dut.cfg_allowed,"stationary disarmed chassis accepts configuration");
  configure(1,1,0);configure(2,2,1);
  check(dut.nav_ready && !dut.armed && dut.brake && command_updates==0,
   "configuration must never ARM or publish a command heartbeat");
  motion(1,1,1);
  check(dut.nav_failed && dut.brake && !dut.nav_pose_ready,
   "route without a reset pose must not move");
  motion(2,0,0);configure(3,3,0);
  check(dut.nav_pose_ready && !dut.nav_failed,"explicit stopped pose reset recovers navigation");
  motion(3,1,1);wait_drive;
  // A trimmed forward request still needs to enter safety's detour FSM.
  force dut.navigation.desired_command=3;force dut.navigation.desired_speed=8;
  distance_l=190;#1;
  check(dut.effective_command==1,
   "near obstacle must normalize navigation curve to protected forward");
  cycles(1);
  check(dut.brake,"coherent range/request bundle brakes safety on the next edge");
  cycles(3);
  check(dut.state==1,"navigation curve must enter stop-before-detour");
  release dut.navigation.desired_command;release dut.navigation.desired_speed;
  distance_l=1000;
  motion(4,0,0);configure(4,3,0);motion(5,1,1);wait_drive;
  // The IR STOP gates the real motor safety outputs before the next edge.
  @(negedge clk);key=8'h1c;key_valid=1;#1;
  // Ordinary FSM commands are now registered. The final wheel-brake gate,
  // rather than that FSM's one-clock delayed brake output, must stop hardware
  // before the next edge (the bridge outputs are checked by tb_top_brake_pipeline).
  check(dut.effective_command==0 && dut.wheel_brake && !dut.nav_run,
   "same-cycle IR STOP must override active navigation");
  cycles(1);@(negedge clk);key_valid=0;cycles(3);
  check(dut.ir_selected && !dut.armed && !dut.nav_selected && dut.brake,
   "IR STOP must latch remote ownership and exit navigation");
  motion(6,1,1);
  check(!dut.nav_selected && !dut.armed && dut.brake,
   "stale UART nav ARM cannot regain control after IR STOP");
  motion(7,0,0);configure(5,3,0);motion(8,1,1);wait_drive;
  force dut.uart_fresh=0;#1;
  check(dut.brake,"UART heartbeat loss must gate motor outputs immediately");
  cycles(3);force dut.uart_fresh=1;cycles(3);
  check(dut.brake && dut.fault!=0 && dut.nav_failed,
   "link recovery must not resume navigation without explicit recovery");
  release dut.uart_fresh;
  motion(9,0,0);cycles(5);
  check(dut.cfg_allowed,"disarmed stable chassis is configurable after recovery");
  @(negedge clk);pl=2;#1;
  check(!dut.cfg_allowed,"encoder movement outside the one-count stationary band must immediately block configuration");
  cycles(100000);check(!dut.cfg_allowed,"configuration requires the full fresh stationary window");
  cycles(100010);check(dut.cfg_allowed,"configuration returns after 200ms of observed stillness");
  configure(6,3,0);motion(10,1,1);wait_drive;
  force dut.imu_ok=0;#1;
  check(dut.brake && dut.nav_command==0,"IMU invalidity must stop navigation immediately");
  cycles(3);force dut.imu_ok=1;cycles(3);
  check(dut.nav_failed && dut.brake && dut.fault!=0,
   "sensor recovery must not resume after a latched fault");
  motion(11,0,0);cycles(22000);configure(7,3,0);motion(12,1,1);wait_drive;
  // Feed bounded, forward odometry increments until within arrival tolerance.
  repeat(14)begin
   @(negedge clk);pl=pl+500;pr=pr+500;cycles(20000);
  end
  motion(13,1,1);cycles(250000);
  check(dut.nav_done && !dut.nav_active && dut.brake && dut.effective_command==0,
   "completed route must brake without falling through to the UART forward command");
  motion(14,1,1);cycles(22000);
  check(dut.nav_done && dut.brake,"navigation heartbeat must not restart a completed route");
  motion(15,0,0,1);
  check(dut.ir_selected && !dut.armed,
   "IR STOP must retain ownership even when coincident with UART DISARM");
  motion(16,1,1);
  check(dut.ir_selected && !dut.armed && dut.brake,
   "UART ARM after coincident stops must not steal remote STOP ownership");
  motion(17,0,0);configure(8,3,0);motion(18,1,1,0,0);cycles(22000);
  check(!dut.nav_run && !dut.nav_active && dut.brake,
   "zero-speed navigation request must retain the ordinary zero-speed stop semantics");
  motion(19,1,1);wait_drive;
  $display("PASS navigation integration: config/readiness/curve-detour/IR STOP/heartbeat/stationarity/sensor/completion/zero-speed");
  $finish;
 end
 initial begin #18000000;$fatal(1,"navigation integration timeout");end
endmodule

// Simulation-only vendor abstractions. A locked-low PLL keeps camera and
// Ethernet outside the scope of this motor-arbitration integration test.
module Gowin_PLL_MOD(output lock,clkout0,clkout1,clkout2,output [7:0] mdrdo,
 input clkin,reset,mdclk,input [1:0] mdopc,input mdainc,input [7:0] mdwdi);
 assign lock=0;assign clkout0=clkin;assign clkout1=clkin;assign clkout2=clkin;
 assign mdrdo=0;
endmodule
module ODDR(output Q0,Q1,input D0,D1,TX,CLK);
 assign Q0=CLK?D0:D1;assign Q1=TX;
endmodule

