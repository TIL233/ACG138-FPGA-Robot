`timescale 1ns/1ps
// Real robot_top command parser/arbitration/request bundle/safety FSM. Only
// UART bytes and sensor observations are injected; no FSM state is forced.
module tb_follow_detour_cancel;
 reg clk=0;always #5 clk=~clk;
 reg [7:0] rx=0,key=0;reg rx_valid=0,key_valid=0;
 reg [7:0] packet_seq=0;
 reg [15:0] distance_l=1000,distance_r=1000;
 reg range_l_update=0,range_r_update=0;
 robot_top #(.CLK_HZ(1000000)) dut(
  .clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),
  .cam1_pclk(1'b0),.cam1_href(1'b0),.cam1_vsync(1'b0),.cam1_data(8'd0),
  .uart_tx(),.motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),
  .motor_r_pwm(),.motor_r_in1(),.motor_r_in2(),
  .cam1_xclk(),.cam1_pwdn(),.cam1_rst_n(),.cam1_scl(),.cam1_sda(),
  .rgmii_tx_clk(),.rgmii_tx_ctl(),.rgmii_txd());
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit good,input string message);
  if(!good)$fatal(1,"%s state=%0d follow=%b armed=%b request=%0d staged=%0d staged_follow=%b ranges=%0d/%0d fault=%h",
   message,dut.state,dut.follow_selected,dut.armed,dut.effective_command,
   dut.safety_command_in,dut.safety_follow_in,dut.safety_dl_in,dut.safety_dr_in,dut.fault);
 endtask
 function [15:0] crcbyte(input [15:0] a,input [7:0] b);
  reg [15:0] value;integer bit_index;
  begin
   value=a^{b,8'd0};
   for(bit_index=0;bit_index<8;bit_index=bit_index+1)
    value=value[15]?(value<<1)^16'h1021:value<<1;
   crcbyte=value;
  end
 endfunction
 task send(input [7:0] value);begin
  @(negedge clk);rx=value;rx_valid=1;cycles(1);
  @(negedge clk);rx_valid=0;
 end endtask
 task motion(input bit arm,input bit follow_mode,input bit near_on_commit=0,input bit stop_on_commit=0);
  reg [7:0] body[0:5];reg [15:0] crc;integer byte_index;
  begin
   packet_seq=packet_seq+1'b1;
   body[0]=1;body[1]=packet_seq;body[2]={7'd0,arm};body[3]=arm?1:0;
   body[4]=16;body[5]=follow_mode?2:0;
   crc=16'hffff;send(8'ha5);send(8'h5a);
   for(byte_index=0;byte_index<6;byte_index=byte_index+1)begin
    send(body[byte_index]);crc=crcbyte(crc,body[byte_index]);
   end
   send(crc[7:0]);send(crc[15:8]);send(13);
   // The previous manual request sees near sonar on precisely the edge that
   // commits FOLLOW. One old coherent bundle may enter WAIT_STOP, but the
   // following one-shot STOP must cancel it instead of merely masking FAR.
   @(negedge clk);rx=10;rx_valid=1;
   if(near_on_commit)begin
    distance_l=150;distance_r=180;range_l_update=1;range_r_update=1;
   end
   if(stop_on_commit)begin
    key=8'h1c;key_valid=1;#1;
    check(dut.ir_stop_now && dut.wheel_brake && dut.effective_command==0,
          "physical STOP wins before the coincident FOLLOW packet commit");
   end
   cycles(1);
   if(near_on_commit)check(dut.safety_dl_in==150 && dut.safety_dr_in==180 &&
       dut.safety_lok_in && dut.safety_rok_in && dut.safety_lu_in && dut.safety_ru_in,
       "near values, validity and update pulses enter the old manual bundle coherently");
   @(negedge clk);rx_valid=0;range_l_update=0;range_r_update=0;key_valid=0;
  end
 endtask
 task reset_manual(input bit start_near);begin
  force dut.rst_n=0;rx=0;rx_valid=0;key=0;key_valid=0;packet_seq=0;
  distance_l=start_near?150:1000;distance_r=start_near?180:1000;
  range_l_update=0;range_r_update=0;cycles(5);
  force dut.rst_n=1;cycles(22000);
  check(dut.motion_ok,"real motion feedback initializes from fresh sensor observations");
  motion(0,0);cycles(6);motion(1,0);cycles(6);
  check(dut.armed && !dut.follow_selected && dut.fault==0,
        "CRC-protected manual forward owns control before the transition");
  if(start_near)check(dut.state==1 && dut.brake,"manual near command is waiting before its scan");
  else check(dut.state==0 && !dut.brake && dut.target_l==16 && dut.target_r==16,
             "clear manual forward is driving before the transition");
 end endtask
 task accept_follow;begin
  check(dut.uart_follow && dut.follow_selected,"real command parser committed FOLLOW");
  cycles(1);
  check(dut.safety_follow_in && dut.safety_command_in==0 && dut.safety_update_in &&
        dut.safety_armed_in && dut.safety_dl_in==16'hfffe && dut.safety_dr_in==16'hfffe,
        "entering FOLLOW inserts exactly one coherent STOP and FAR bundle");
  cycles(1);
  check(dut.safety_follow_in && dut.safety_command_in==1 && !dut.safety_update_in &&
        dut.state==0 && dut.fault==0,"old detour is canceled before normal FOLLOW forward resumes");
  cycles(2);
 end endtask
 task hold_follow;
  integer elapsed;
  begin
   // Longer than the actual200ms settling interval at the configured1MHz.
   for(elapsed=0;elapsed<230000;elapsed=elapsed+1)begin
    cycles(1);
    check(dut.follow_selected && dut.state==0 && dut.fault==0 &&
          dut.target_l==16 && dut.target_r==16 && !dut.brake,
          "FOLLOW cannot resume a pending/manual scan after the settling window");
   end
  end
 endtask
 task wait_for_turn;
  integer elapsed;
  begin
   elapsed=0;
   while(dut.state!=2 && dut.fault==0 && elapsed<260000)begin cycles(1);elapsed=elapsed+1;end
   check(dut.state==2 && dut.target_l<0 && dut.target_r>0 && !dut.brake,
         "manual near maneuver really entered automatic left scanning");
  end
 endtask
 integer i;
 initial begin
  force dut.rx_data=rx;force dut.rx_valid=rx_valid;
  force dut.ir_key=key;force dut.ir_valid=key_valid;force dut.ir_repeat=0;
  force dut.imu_ok=1;force dut.imu_update=1;force dut.yaw=0;
  force dut.pos_l=0;force dut.pos_r=0;
  force dut.dl=distance_l;force dut.dr=distance_r;
  force dut.lok=1;force dut.rok=1;
  force dut.lu=range_l_update;force dut.ru=range_r_update;
  force dut.lfault=0;force dut.rfault=0;force dut.predictive_fault=0;

  reset_manual(0);motion(1,1,1);accept_follow();hold_follow();
  reset_manual(1);cycles(50000);check(dut.state==1,"WAIT_STOP transition is exercised before it can turn");
  motion(1,1);accept_follow();hold_follow();
  reset_manual(1);wait_for_turn();motion(1,1);accept_follow();hold_follow();

  // Raw IR STOP is independent of the newly registered sonar/mode bundle.
  @(negedge clk);key=8'h1c;key_valid=1;#1;
  check(dut.wheel_brake && dut.effective_command==0 &&
        dut.motor_l_pwm && dut.motor_l_in1 && dut.motor_l_in2 &&
        dut.motor_r_pwm && dut.motor_r_in1 && dut.motor_r_in2,
        "IR STOP immediately short-brakes both physical bridge outputs before the next edge");
  cycles(1);@(negedge clk);key_valid=0;cycles(6);
  check(!dut.armed && !dut.follow_selected && dut.ir_selected && dut.state==0 && dut.brake && dut.fault==0,
        "physical STOP latches remote ownership and cancels FOLLOW");
  cycles(230000);check(!dut.armed && dut.state==0 && dut.wheel_brake,
        "stopped FOLLOW never restarts after another settling interval");

  reset_manual(0);motion(1,1,1,1);cycles(8);
  check(dut.uart_follow && dut.ir_selected && !dut.follow_selected && !dut.armed &&
        dut.state==0 && dut.brake && dut.wheel_brake && dut.fault==0,
        "same-edge FOLLOW packet, near sonar and physical STOP cannot start a detour or motion");
  cycles(230000);check(!dut.armed && dut.state==0 && dut.wheel_brake,
        "coincident STOP remains latched beyond the full settling interval");
  $display("PASS follow detour cancellation: real CRC/top arbitration, near commit race, WAIT_STOP/TURN cancellation, coherent sonar bundle, no delayed scan and raw/coincident IR STOP");
  $finish;
 end
 initial begin #30000000;$fatal(1,"follow detour cancellation test hung");end
endmodule
