`timescale 1ns/1ps
// Real top-level routing, real wheel estimator and navigation FSM. The stimulus
// represents known wheel travel with IMU reporting one quarter of yaw change.
module nav_heading_fixture #(parameter USE_ENCODER=1,TRACK_MM=100)(
 input clk,rst_n,run,imu_ok,wr,commit,pose_reset,
 input [2:0] wi,input signed [15:0] wx,wy,yaw,
 input signed [31:0] pl,pr,
 output [2:0] cmd,idx,output done,failed,
 output signed [31:0] x,y,output signed [15:0] heading,goal);
 robot_top #(.CLK_HZ(50000),.NAV_USE_ENCODER(USE_ENCODER),.AVOID_TRACK_MM(TRACK_MM)) dut(
  .clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),
  .ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),
  .enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),
  .uart_tx(),.motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),
  .motor_r_pwm(),.motor_r_in1(),.motor_r_in2(),
  .cam1_pclk(1'b0),.cam1_href(1'b0),.cam1_vsync(1'b0),.cam1_data(8'd0),
  .cam1_xclk(),.cam1_pwdn(),.cam1_rst_n(),.cam1_scl(),.cam1_sda(),
  .rgmii_tx_clk(),.rgmii_tx_ctl(),.rgmii_txd());
 initial begin
  force dut.rst_n=rst_n;
  force dut.imu_ok=imu_ok;force dut.imu_update=1'b1;force dut.yaw=yaw;
  force dut.pos_l=pl;force dut.pos_r=pr;
  force dut.lok=1;force dut.rok=1;force dut.dl=2000;force dut.dr=2000;
  force dut.lfault=0;force dut.rfault=0;
  force dut.link_ok=1;force dut.armed=run;
  force dut.command_update=pose_reset;
  force dut.nav_run=run;force dut.nav_selected=run;
  force dut.cfg_write=wr;force dut.cfg_index=wi;force dut.cfg_x=wx;force dut.cfg_y=wy;
  force dut.cfg_commit=commit;force dut.cfg_count=3;force dut.cfg_clear=0;force dut.cfg_reset=pose_reset;
 end
 assign cmd=dut.nav_command;assign idx=dut.nav_index;
 assign done=dut.nav_done;assign failed=dut.nav_failed;
 assign x=dut.nav_x;assign y=dut.nav_y;assign heading=dut.nav_heading;assign goal=dut.heading_goal;
 // The real producer publishes phase4 data before phase5 update. At the
 // consumer edge the selected-heading register must already contain that
 // same snapshot, including when wheel heading differs greatly from IMU.
 integer checked_heading_updates=0,checked_predictive_samples=0;
 reg sampled_straight=0,sampled_nav=0;
 reg signed [15:0] sampled_target_l=0,sampled_target_r=0;
 reg signed [15:0] expected_reference_l=0,expected_reference_r=0;
 reg [31:0] expected_stamp=0;
 always @(posedge clk)begin
  if(!rst_n)begin
   checked_heading_updates=0;checked_predictive_samples=0;sampled_straight=0;
  end else begin
   if(dut.motion_update)begin
    if(!dut.heading_control.cancel &&
       dut.heading_selected_sample !== (dut.nav_selected?dut.navigation_heading:dut.motion_heading))
     $fatal(1,"heading update consumed a source register from the wrong motion snapshot");
    if(!dut.heading_control.cancel)checked_heading_updates=checked_heading_updates+1;
    sampled_straight=0;
    // Observe the corrected references after the outer loop consumes this
    // update. The existing predictive sample delay must preserve them.
    #1;
    if(!dut.heading_control.cancel && dut.heading_holding)begin
     sampled_straight=1;sampled_nav=dut.nav_selected;
     sampled_target_l=dut.safety_target_l;sampled_target_r=dut.safety_target_r;
     expected_reference_l=dut.heading_target_l;expected_reference_r=dut.heading_target_r;
     expected_stamp=dut.motion_stamp;
    end
   end else if(dut.predictive_sample && sampled_straight &&
               !dut.predictive_brake_stage && !dut.heading_source_changed &&
               dut.nav_selected==sampled_nav &&
               dut.safety_target_l==sampled_target_l && dut.safety_target_r==sampled_target_r)begin
    if(dut.predictive_ref_stage_l!==expected_reference_l ||
       dut.predictive_ref_stage_r!==expected_reference_r || dut.motion_stamp!==expected_stamp)
     $fatal(1,"predictive reference/sample tag does not belong to the consumed heading generation");
    checked_predictive_samples=checked_predictive_samples+1;
    sampled_straight=0;
   end
  end
 end
endmodule

module tb_navigation_encoder_heading;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,run=0,ok=1,wr=0,commit=0,pose_reset=0;
 reg [2:0] wi=0;reg signed [15:0] wx=0,wy=0,yaw=0;
 reg signed [31:0] pl=0,pr=0;
 wire [2:0] cmd,old_cmd,idx,old_idx;
 wire done,failed,old_done,old_failed;
 wire signed [31:0] x,y,old_x,old_y;
 wire signed [15:0] heading,old_heading,goal,old_goal;
 nav_heading_fixture current_path(clk,rst_n,run,ok,wr,commit,pose_reset,wi,wx,wy,yaw,pl,pr,cmd,idx,done,failed,x,y,heading,goal);
 nav_heading_fixture #(.USE_ENCODER(0)) old_path(clk,rst_n,run,ok,wr,commit,pose_reset,wi,wx,wy,yaw,pl,pr,old_cmd,old_idx,old_done,old_failed,old_x,old_y,old_heading,old_goal);
 wire [2:0] invalid_cmd,invalid_idx;wire invalid_done,invalid_failed;
 wire signed [31:0] invalid_x,invalid_y;wire signed [15:0] invalid_h,invalid_goal;
 nav_heading_fixture #(.TRACK_MM(0)) invalid_geometry(clk,rst_n,run,ok,wr,commit,pose_reset,wi,wx,wy,yaw,pl,pr,invalid_cmd,invalid_idx,invalid_done,invalid_failed,invalid_x,invalid_y,invalid_h,invalid_goal);
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task samples(input integer n);cycles(n*1000+50);endtask
 task check(input bit v,input string message);
  if(!v)$fatal(1,"%s cmd=%0d old=%0d idx=%0d h=%0d oldh=%0d x=%0d y=%0d failed=%b healthy=%b route=%b pose=%b motion=%h safety=%h pred=%b",message,cmd,old_cmd,idx,heading,old_heading,x,y,failed,current_path.dut.navigation.healthy,current_path.dut.nav_ready,current_path.dut.nav_pose_ready,current_path.dut.motion_flags,current_path.dut.fault,current_path.dut.predictive_fault);
 endtask
 task point(input integer i,input integer px,input integer py);
  begin @(negedge clk);wi=i;wx=px;wy=py;wr=1;cycles(2);@(negedge clk);wr=0;cycles(2);end
 endtask
 task translate(input integer distance_counts);
  begin @(negedge clk);pl=pl+distance_counts;pr=pr+distance_counts;samples(2);end
 endtask
 integer i;
 initial begin
  cycles(5);rst_n=1;samples(3);
  point(0,1000,0);point(1,1000,1000);point(2,0,1000);
  commit=1;cycles(2);commit=0;cycles(3);
  pose_reset=1;cycles(2);pose_reset=0;samples(2);run=1;samples(2);
  check(cmd==1 && old_cmd==1,"both start towards first point");
  check(invalid_failed && invalid_cmd==0,"encoder navigation cannot start with absent wheel geometry");
  repeat(20)translate(382);
  samples(15);
  check(idx==1 && old_idx==1 && cmd==5 && old_cmd==5,"example second point requests left turn");
  for(i=1;i<=30;i=i+1)begin
   @(negedge clk);pl=pl-20;pr=pr+20;yaw=i*75;samples(2);
  end
  samples(3);
  check(heading>8900 && heading<9100 && old_heading==2250,"independent encoder heading retains geometric ninety degrees");
  check(cmd==1 && old_cmd==5,"at geometric ninety degrees fixed navigation drives, old IMU-only navigation still pivots left");
  check(goal>8000 && goal<9200,"navigation straight control anchors to the same heading as its pose estimator");
  $display("REPRO: geometric_yaw=%0d IMU_yaw=%0d old_command=%0d fixed_command=%0d",heading,old_heading,old_cmd,cmd);
  repeat(20)translate(382);
  samples(15);
  check(idx==2 && cmd==5 && !failed && x>950 && x<1050 && y>950 && y<1050,"second leg projects travel onto Y before next left turn");
  for(i=31;i<=60;i=i+1)begin
   @(negedge clk);pl=pl-20;pr=pr+20;yaw=i*75;samples(2);
  end
  samples(3);check(cmd==1 && !failed,"second left turn aligns across 180 degrees");
  repeat(20)translate(382);
  samples(15);check(done && !failed && cmd==0 && x>-80 && x<80 && y>950 && y<1050,"all three example points finish with the same global origin");
  run=0;samples(2);point(0,0,-1000);commit=1;cycles(2);commit=0;cycles(3);
  pose_reset=1;cycles(2);pose_reset=0;samples(2);run=1;samples(2);
  check(cmd==6 && !failed,"negative Y requests a right turn from the new origin");
  for(i=1;i<=30;i=i+1)begin
   @(negedge clk);pl=pl+20;pr=pr-20;yaw=4500-i*75;samples(2);
  end
  samples(3);check(heading>-9100 && heading<-8900 && cmd==1 && !failed,"right turn uses signed wheel heading relative to explicit new origin");
  ok=0;#1;check(cmd==0,"encoder navigation retains immediate IMU-health stop");cycles(4);
  check(failed,"health loss remains a latched navigation failure");
  ok=1;samples(4);check(cmd==0 && failed,"feedback recovery cannot auto-restart");
  check(current_path.checked_heading_updates>10 && current_path.checked_predictive_samples>10,
        "real producer exercised same-generation heading and predictive reference assertions");
  $display("PASS navigation encoder heading: IMU-quarter-yaw reproduction/example three points/right turn/new origin/consistent straight feedback/invalid geometry/health stop");
  $finish;
 end
 initial begin #20000000;$fatal(1,"navigation encoder heading test timed out");end
endmodule
