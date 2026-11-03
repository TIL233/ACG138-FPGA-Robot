`timescale 1ns/1ps
// Real top-level permission and configuration parser; replay the observed
// stationary right-wheel {-1,0} pattern at 20ms intervals.
module tb_nav_config_jitter;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,arm=0,nav_run=0,valid=0;
 reg signed [31:0] pl=0,pr=0;
 reg [7:0] data=0;
 integer jitter_clock=0;reg jitter=1;
 robot_top #(.CLK_HZ(1000000)) dut(.clk50m(clk),.estop_n(1'b1),
  .uart_rx(1'b1),.imu_rx(1'b1),.ultra_l_rx(1'b1),.ultra_r_rx(1'b1),
  .ir_rx(1'b1),.lidar_rx(1'b1),.enc_l_a(1'b0),.enc_l_b(1'b0),
  .enc_r_a(1'b0),.enc_r_b(1'b0),.cam1_pclk(1'b0),.cam1_href(1'b0),
  .cam1_vsync(1'b0),.cam1_data(8'd0),.cam1_sda(),.uart_tx(),
  .motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),
  .motor_r_pwm(),.motor_r_in1(),.motor_r_in2(),.cam1_xclk(),.cam1_pwdn(),
  .cam1_rst_n(),.cam1_scl(),.rgmii_tx_clk(),.rgmii_tx_ctl(),.rgmii_txd());
 always @(negedge clk)if(rst_n && jitter)begin
  jitter_clock=jitter_clock+1;
  if(jitter_clock==20000)begin jitter_clock=0;pr=(pr==0)?-1:0;end
 end
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit yes,input string message);
  if(!yes)$fatal(1,"%s allowed=%b still=%0d ack=%0d/%0d pos=%0d/%0d",message,
   dut.cfg_allowed,dut.cfg_still,dut.cfg_ack_seq,dut.cfg_ack_status,pl,pr);
 endtask
 function [15:0] crcbyte(input [15:0] a,input [7:0] b);
  reg [15:0] c;integer k;begin c=a^{b,8'd0};
   for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;crcbyte=c;
  end
 endfunction
 task send(input [7:0] b);begin
  @(negedge clk);data=b;valid=1;cycles(1);@(negedge clk);valid=0;
 end endtask
 task cfg_frame(input [7:0] seq,input [7:0] opcode);
  reg [7:0] bytes[0:13];reg [15:0] crc;integer k;
  begin
   for(k=0;k<14;k=k+1)bytes[k]=0;
   bytes[0]=1;bytes[1]=seq;bytes[2]=opcode;
   if(opcode==2)bytes[8]=1;
   send(8'ha5);send(8'hc3);crc=16'hffff;
   for(k=0;k<14;k=k+1)begin send(bytes[k]);crc=crcbyte(crc,bytes[k]);end
   send(crc[7:0]);send(crc[15:8]);send(13);send(10);cycles(4);
  end
 endtask
 initial begin
  force dut.rst_n=rst_n;force dut.pos_l=pl;force dut.pos_r=pr;
  force dut.armed=arm;force dut.nav_run=nav_run;
  force dut.rx_data=data;force dut.rx_valid=valid;
  // Keep the actual20ms feedback producer active, including its +/-1
  // velocity samples from this same encoder-position jitter.
  force dut.imu_ok=1;force dut.imu_update=1;force dut.yaw=16'd0;
  cycles(4);rst_n=1;cycles(190000);
  check(!dut.cfg_allowed,"bounded jitter cannot skip the full200ms window");
  cycles(30000);cfg_frame(1,4);
  check(dut.cfg_ack_seq==1 && dut.cfg_ack_status==0,
   "observed stationary jitter permits a real accepted configuration ACK");
  cfg_frame(7,1);cfg_frame(8,2);cfg_frame(9,3);
  check(dut.cfg_ack_seq==9 && dut.cfg_ack_status==0 && dut.nav_ready && dut.nav_pose_ready,
   "jitter permits route commit and an explicit healthy pose reset without ARM");
  // Two counts of cumulative displacement leave the fixed band even though
  // every individual step is just one count.
  jitter=0;
  for(integer k=0;k<12;k=k+1)begin
   @(negedge clk);pl=pl+1;cycles(50000);
   if(k>0)check(!dut.cfg_allowed,"slow monotonic drift cannot qualify as stopped");
  end
  cfg_frame(2,3);check(dut.cfg_ack_status==1,"drifting chassis cannot reset pose and receives busy ACK");
  cycles(220000);cfg_frame(3,4);check(dut.cfg_ack_status==0,"a genuinely stopped chassis recovers");
  arm=1;cycles(3);cfg_frame(4,4);check(dut.cfg_ack_status==1,"ARM prohibits configuration");
  cycles(220000);arm=0;cycles(3);
  check(dut.cfg_allowed,"DISARM preserves an already observed stationary chassis");
  cycles(220000);nav_run=1;cycles(3);cfg_frame(5,4);
  check(dut.cfg_ack_status==1,"running navigation prohibits configuration");
  nav_run=0;cycles(220000);cfg_frame(6,4);
  check(dut.cfg_ack_status==0,"STOP permits configuration on the observed stationary chassis");
  $display("PASS nav configuration jitter: real ACK, full window, fixed anchors, drift, ARM and STOP");$finish;
 end
 initial begin #30000000;$fatal(1,"navigation configuration jitter hung");end
endmodule
