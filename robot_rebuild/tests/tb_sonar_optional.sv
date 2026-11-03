`timescale 1ns/1ps
module tb_sonar_optional;
 reg clk=0;always #5 clk=~clk;
 reg signed [15:0] angle=0;
 reg signed [31:0] pl=0,pr=0;
 robot_top #(.CLK_HZ(1000000)) dut(.clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0));
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task reset;
 begin force dut.rst_n=0;angle=0;pl=0;pr=0;ticks(4);force dut.rst_n=1;ticks(22000);end endtask
 task turn(input integer delta);
 integer remaining,increment,value;
 begin
  remaining=delta;
  while(remaining!=0)begin
   increment=(remaining>0)?1000:-1000;
   @(negedge clk);value=$signed(angle)+increment;
   if(value>18000)value=value-36000;
   if(value< -18000)value=value+36000;
   // 100mm track / 7640counts/m: about67 counts per wheel per10deg.
   // Avoidance now observes wheel heading independently of the raw IMU.
   pl=pl-(increment>0?67:-67);pr=pr+(increment>0?67:-67);
   angle=value;remaining=remaining-increment;ticks(20005);
  end
  ticks(4);
 end endtask
 task uart(input bit a,input [2:0] cmd,input bit f);
 begin
  @(negedge clk);force dut.uart_armed=a;force dut.uart_command=cmd;
  force dut.uart_follow=f;force dut.uart_update=1;
  ticks(2);@(negedge clk);force dut.uart_update=0;ticks(8);
 end endtask
 task key(input [7:0] k);
 begin
  @(negedge clk);force dut.ir_key=k;force dut.ir_valid=1;
  ticks(1);@(negedge clk);force dut.ir_valid=0;ticks(8);
 end endtask
 initial begin
  force dut.ir_valid=0;force dut.ir_repeat=0;force dut.uart_update=0;
  force dut.uart_nav=0;force dut.uart_speed=16;force dut.uart_fresh=1;
  force dut.imu_ok=1;force dut.lok=0;force dut.rok=0;
  force dut.yaw=angle;force dut.imu_update=1;
  force dut.pos_l=pl;force dut.pos_r=pr;
  force dut.lu=0;force dut.ru=0;force dut.dl=100;force dut.dr=150;
  reset();uart(0,0,0);uart(1,1,0);
  if(dut.state!=0 || dut.fault || dut.wheel_brake || dut.target_l!=16 || dut.target_r!=16)
   $fatal(1,"missing sonar must permit manual forward and ignore stale near values");
  if(dut.health[3:2]!=0)$fatal(1,"missing sonar must remain invalid in telemetry");
  force dut.lok=1;force dut.dl=150;ticks(8);
  if(dut.state!=1 || !dut.brake)$fatal(1,"one valid near side still starts manual avoidance");
  // Losing both probes mid-detour must not wait forever for missing updates.
  force dut.lok=0;ticks(225000);
  if(dut.state!=2 || dut.fault)$fatal(1,"missing scan updates must not cause settle timeout");
  turn(9000);ticks(525000);
  if(dut.state!=2 || dut.safety.turn_left || dut.safety.left_clearance!=16'hfffe || dut.fault)
   $fatal(1,"missing left scan is FAR and still performs right180");
  turn(-18000);ticks(525000);
  if(dut.state!=2 || !dut.safety.turn_left || dut.safety.right_clearance!=16'hfffe || dut.fault)
   $fatal(1,"missing right scan is FAR, both missing choose left");
  turn(18000);ticks(525000);
  if(dut.state!=4 || dut.fault || dut.brake)$fatal(1,"missing probes permit chosen-side departure");
  reset();force dut.lok=1;force dut.rok=1;force dut.dl=100;force dut.dr=150;
  uart(0,0,0);uart(1,1,1);ticks(20);
  if(!dut.follow_selected || dut.state!=0 || dut.brake || dut.target_l!=16 || dut.target_r!=16)
   $fatal(1,"direct visual follow must not enter autonomous obstacle turn");
  key(8'h1c);
  if(dut.armed || dut.follow_selected || !dut.wheel_brake)$fatal(1,"physical STOP overrides follow mode");
  key(8'h18);
  if(dut.follow_selected || dut.state!=1)$fatal(1,"manual IR takeover must restore valid-near avoidance");
  reset();force dut.lok=0;force dut.rok=0;uart(0,0,0);uart(1,1,1);
  force dut.imu_ok=0;ticks(8);
  if(!dut.wheel_brake || dut.fault!=2)$fatal(1,"IMU failure must still brake without sonar faults");
  force dut.imu_ok=1;ticks(22000);uart(0,0,0);uart(1,1,1);
  force dut.uart_fresh=0;ticks(8);
  if(!dut.wheel_brake || dut.fault!=1)$fatal(1,"command link timeout must still brake");
  force dut.uart_fresh=1;uart(0,0,0);uart(1,1,1);
  force dut.stop_permission=0;ticks(8);
  if(!dut.wheel_brake || dut.fault!=16)$fatal(1,"stop permission must still brake");
  $display("PASS optional sonar: manual missing/stale, truthful health, missing scan, follow no detour, IR takeover, IMU/link/stop gates");$finish;
 end
 initial begin #40000000;$fatal(1,"optional sonar timeout");end
endmodule
