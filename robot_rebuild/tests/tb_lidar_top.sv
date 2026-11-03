`timescale 1ns/1ps
module tb_lidar_top;
 reg clk=0;always #5 clk=~clk;
 robot_top #(.CLK_HZ(1000000),.LIDAR_CONTROL(1)) dut(
  .clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),.lidar_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0));
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task command(input bit a,input bit f);
  begin @(negedge clk);force dut.uart_armed=a;force dut.uart_command=a?1:0;
   force dut.uart_follow=f;force dut.uart_update=1;ticks(2);
   @(negedge clk);force dut.uart_update=0;ticks(20);
  end
 endtask
 initial begin
  force dut.rst_n=0;force dut.ir_valid=0;force dut.ir_repeat=0;force dut.uart_update=0;
  force dut.uart_nav=0;force dut.uart_speed=8;force dut.uart_fresh=1;
  force dut.imu_ok=1;force dut.lok=0;force dut.rok=0;
  force dut.yaw=0;force dut.imu_update=1;force dut.pos_l=0;force dut.pos_r=0;
  force dut.lu=0;force dut.ru=0;force dut.lidar_valid=1;force dut.lidar_front=600;
  ticks(4);force dut.rst_n=1;ticks(22000);
  force dut.lidar_valid=3'b111;force dut.lidar_fresh=1;force dut.lidar_left=1200;force dut.lidar_right=900;
  #1;if(dut.lidar_flags[3] || dut.lidar_diag[111:96]!=16'hffff || dut.lidar_flags[2:0]!=7)$fatal(1,"rear wire slot must stay reserved");
  force dut.lidar_valid=1;
  command(0,0);command(1,1);
  if(!dut.follow_selected || !dut.wheel_brake || dut.state!=0 || dut.fault!=0 || !dut.health[5])$fatal(1,"follow hold must reach motor gate without detour/fault");
  force dut.lidar_front=601;ticks(50);
  if(dut.wheel_brake || dut.state!=0)$fatal(1,"follow hold release");
  force dut.lidar_front=150;force dut.lidar_valid=0;ticks(50);
  if(dut.wheel_brake)$fatal(1,"stale lidar near must expire");
  command(0,0);command(1,0);force dut.lidar_valid=1;ticks(10);
  if(dut.state!=1 || !dut.brake || dut.control_dl!=150)$fatal(1,"lidar near starts normal avoidance");
  @(negedge clk);force dut.ir_key=8'h1c;force dut.ir_valid=1;ticks(2);
  if(!dut.wheel_brake || dut.armed)$fatal(1,"physical STOP wins");
  $display("PASS lidar top fusion, follow motor hold/recovery, detour initiation and STOP");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
