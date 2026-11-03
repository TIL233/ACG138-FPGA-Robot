`timescale 1ns/1ps
module tb_usb_lidar_top;
 reg clk=0;always #5 clk=~clk;reg uart=1;
 robot_top #(.CLK_HZ(1000000)) dut(
  .clk50m(clk),.estop_n(1'b1),.uart_rx(uart),.imu_rx(1'b1),.lidar_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),
  .cam1_pclk(1'b0),.cam1_href(1'b0),.cam1_vsync(1'b0),.cam1_data(8'd0),
  .uart_tx(),.motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),
  .motor_r_pwm(),.motor_r_in1(),.motor_r_in2(),.cam1_xclk(),.cam1_pwdn(),.cam1_rst_n(),
  .cam1_scl(),.cam1_sda(),.rgmii_tx_clk(),.rgmii_tx_ctl(),.rgmii_txd());
 robot_top #(.CLK_HZ(1000000),.LIDAR_USB_FORWARD(0)) legacy(
  .clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),.lidar_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),
  .cam1_pclk(1'b0),.cam1_href(1'b0),.cam1_vsync(1'b0),.cam1_data(8'd0),
  .uart_tx(),.motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),
  .motor_r_pwm(),.motor_r_in1(),.motor_r_in2(),.cam1_xclk(),.cam1_pwdn(),.cam1_rst_n(),
  .cam1_scl(),.cam1_sda(),.rgmii_tx_clk(),.rgmii_tx_ctl(),.rgmii_txd());
 reg [7:0] p[0:31];integer i,control_updates=0,writes=0;
 always @(posedge clk)begin
  if(dut.uart_update)control_updates=control_updates+1;
  if(dut.cfg_write)writes=writes+1;
 end
 function [15:0] cb(input [15:0] crc,input [7:0] b);
  reg [15:0] c;integer j;begin c=crc^{b,8'd0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;cb=c;end
 endfunction
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task send(input [7:0] b);integer j;begin
  @(negedge clk);uart=0;repeat(9)@(negedge clk);
  for(j=0;j<8;j=j+1)begin uart=b[j];repeat(9)@(negedge clk);end
  uart=1;repeat(9)@(negedge clk);
 end endtask
 task escaped(input [7:0] b);begin
  if(b==8'ha5 || b==8'h7d || b==8'h7e)begin send(8'h7d);send(b^8'h20);end else send(b);
 end endtask
 task command(input [7:0] seq,input bit arm);
  integer j;reg [15:0] c;begin
   p[0]=8'ha5;p[1]=8'h5a;p[2]=1;p[3]=seq;p[4]=arm;p[5]=arm;p[6]=arm?8:0;p[7]=0;
   c=16'hffff;for(j=2;j<8;j=j+1)c=cb(c,p[j]);
   p[8]=c[7:0];p[9]=c[15:8];p[10]=13;p[11]=10;
  end
 endtask
 task tunnel(input bit corrupt);
  integer j;reg [15:0] c;begin
   c=cb(cb(cb(cb(16'hffff,1),0),0),12);
   send(8'ha5);send(8'hb7);escaped(1);escaped(0);escaped(0);escaped(12);
   for(j=0;j<12;j=j+1)begin escaped(p[j]);c=cb(c,p[j]);end
   escaped(c[7:0]^corrupt);escaped(c[15:8]);send(8'h7e);ticks(1500);
  end
 endtask
 initial begin
  force dut.rst_n=0;force legacy.rst_n=0;ticks(5);force dut.rst_n=1;force legacy.rst_n=1;ticks(100);
  if(dut.lidar_flags[7:6]!=3 || dut.lidar_flags[5] || dut.lidar_flags[3] ||
     legacy.lidar_flags[7:6]!=1 || legacy.lidar_flags[5])$fatal(1,"capability/source/receive-only defaults");
  command(1,1);tunnel(1);tunnel(0);
  if(dut.uart_armed || control_updates || writes)$fatal(1,"real host UART tunnel armed/modified route");
  command(2,1);for(i=0;i<12;i=i+1)send(p[i]);ticks(50);
  if(!dut.uart_armed || control_updates!=1)$fatal(1,"demux command aliases not connected");
  send(8'ha5);send(8'hb7);send(1);send(1);send(0);send(100);send(3);
  command(3,0);for(i=0;i<12;i=i+1)send(p[i]);ticks(50);
  if(dut.uart_armed || control_updates!=2 || !dut.wheel_brake)$fatal(1,"truncated-frame STOP did not reach top motor gate");
  // Top-level nav_config must consume the same post-demux aliases.
  force dut.cfg_allowed=1;
  p[0]=8'ha5;p[1]=8'hc3;p[2]=1;p[3]=9;p[4]=1;p[5]=0;p[6]=8'hb7;p[7]=8'ha5;
  for(i=8;i<16;i=i+1)p[i]=0;
  begin:nav_crc reg [15:0] c;c=16'hffff;for(i=2;i<16;i=i+1)c=cb(c,p[i]);p[16]=c[7:0];p[17]=c[15:8];end
  p[18]=13;p[19]=10;
  // Coordinates outside supported range reject semantically, but a complete
  // post-demux frame must still be CRC-checked and acknowledged with seq9.
  for(i=0;i<20;i=i+1)send(p[i]);ticks(50);
  if(dut.cfg_ack_seq!=9 || dut.cfg_ack_status!=2 || writes)$fatal(1,"nav_config post-demux aliases disconnected");
  $display("PASS USB lidar top: real shared UART isolation,ARM/STOP recovery,nav aliases,USB/legacy diagnostics and control disabled default");$finish;
 end
 initial begin #1000000;$fatal(1,"USB top timeout");end
endmodule
