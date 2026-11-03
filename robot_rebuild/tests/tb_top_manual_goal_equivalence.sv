`timescale 1ns/1ps
`define ROBOT_PINS .clk50m(clk),.estop_n(1'b1),.uart_rx(1'b1),.imu_rx(1'b1),.lidar_rx(1'b1),.ultra_l_rx(1'b1),.ultra_r_rx(1'b1),.ir_rx(1'b1),.enc_l_a(1'b0),.enc_l_b(1'b0),.enc_r_a(1'b0),.enc_r_b(1'b0),.cam1_pclk(1'b0),.cam1_href(1'b0),.cam1_vsync(1'b0),.cam1_data(8'd0),.uart_tx(),.motor_stby(),.motor_l_pwm(),.motor_l_in1(),.motor_l_in2(),.motor_r_pwm(),.motor_r_in1(),.motor_r_in2(),.cam1_xclk(),.cam1_pwdn(),.cam1_rst_n(),.cam1_scl(),.cam1_sda(),.rgmii_tx_clk(),.rgmii_tx_ctl(),.rgmii_txd()
module tb_top_manual_goal_equivalence;
 reg clk=0;always #5 clk=~clk;
 reg nav=0,follow=0,stop_now=0,lok=0,rok=0,laser_ok=0;
 reg [2:0] navcmd=0,cmd=0;reg[15:0] left_mm=0,right_mm=0,front_mm=0;
 reg signed[15:0] goal=0;wire signed[15:0] old_goal;
 robot_top #(.CLK_HZ(1000000),.LIDAR_CONTROL(1),.LIDAR_ADVANCED_CONTROL(1),.LIDAR_MODEL_VALIDATED(1)) dut(`ROBOT_PINS);
 assign old_goal=nav?goal:dut.effective_command==3?16'sd9000:dut.effective_command==4?-16'sd9000:16'sd0;
 reg signed [15:0] snap_goal=0;
 reg [7:0] manual_speed=0,nav_speed=0,snap_speed=0;
 reg [2:0] snap_cmd=0;reg nav_goal_valid=1;
 reg [6:0] permission=7'b1110111;reg [3:0] nav_safety=0;
 wire permitted=permission[6] && permission[5] && permission[4] && !permission[3] &&
  permission[2] && permission[1] && permission[0] && nav_safety<=6;
 wire old_pending=dut.lidar_request_cmd!=dut.effective_command || dut.lidar_request_speed!=dut.effective_speed ||
  dut.lidar_request_armed!=dut.armed || dut.lidar_request_follow!=dut.follow_selected || dut.lidar_request_nav!=dut.nav_selected ||
  dut.lidar_request_goal!=dut.lidar_goal_cd || dut.lidar_request_goal_valid!=dut.lidar_goal_valid;
 integer m,c,d,g,ss,v,sc,p,s,allow;reg[15:0] points[0:3];
 task check;begin #1;
  if(dut.nav_output_permitted!==permitted || dut.nav_intent_command!==navcmd ||
     dut.nav_intent_speed!==((nav_speed>12)?8'd12:nav_speed))
   $fatal(1,"navigation intent/permission sideband differs from upstream contract");
  if(dut.nav_command!==(permitted?navcmd:3'd0) ||
     dut.nav_speed!==(permitted?((nav_speed>12)?8'd12:nav_speed):8'd0))
   $fatal(1,"original navigation permission/clamp output differs");
  if(dut.lidar_goal_cd!==old_goal)$fatal(1,"actual TOP goal differs");
  if(dut.lidar_context_pending!==old_pending || dut.lidar_context_guard!==old_pending)
   $fatal(1,"actual TOP context guard differs from original full expression");
 end endtask
 initial begin
  force dut.rst_n=0;force dut.nav_selected=nav;force dut.follow_selected=follow;force dut.ir_stop_now=stop_now;
  force dut.navigation.desired_command=navcmd;force dut.command=cmd;force dut.nav_goal_error=goal;
  force dut.navigation.rst_n=permission[6];force dut.navigation.run=permission[5];
  force dut.navigation.active=permission[4];force dut.navigation.failed=permission[3];
  force dut.navigation.healthy=permission[2];force dut.navigation.pose_ready=permission[1];
  force dut.navigation.route_ready=permission[0];force dut.navigation.safety_state=nav_safety;
  force dut.lok=lok;force dut.rok=rok;force dut.dl=left_mm;force dut.dr=right_mm;
  force dut.lidar_valid={2'b00,laser_ok};force dut.lidar_front=front_mm;
  force dut.speed=manual_speed;force dut.navigation.desired_speed=nav_speed;force dut.nav_goal_valid=nav_goal_valid;
  force dut.armed=0;force dut.lidar_request_armed=0;
  force dut.lidar_request_follow=follow;force dut.lidar_request_nav=nav;
  force dut.lidar_request_goal_valid=dut.lidar_goal_valid;
  force dut.lidar_request_cmd=dut.effective_command;force dut.lidar_request_speed=dut.effective_speed;
  force dut.lidar_request_goal=dut.lidar_goal_cd;
  points[0]=0;points[1]=199;points[2]=200;points[3]=65535;
  for(m=0;m<64;m=m+1)begin
   {nav,follow,stop_now,lok,rok,laser_ok}=m;
   for(c=0;c<64;c=c+1)begin
    {navcmd,cmd}=c;
    for(d=0;d<64;d=d+1)begin
     left_mm=points[d&3];right_mm=points[(d>>2)&3];front_mm=points[(d>>4)&3];
     goal=-32768;check();goal=32767;check();
    end
   end
  end
  nav=1;
  for(g=-32768;g<=32767;g=g+1)begin goal=g;stop_now=0;check();stop_now=1;check();end
  nav=0;cmd=3;stop_now=0;check();stop_now=1;check();if(dut.lidar_goal_cd!=0)$fatal(1,"manual STOP delayed");
  // Isolate speed: all 256 snapshots x 256 source speeds in each mode,
  // while the unselected source differs and no other mismatch can mask it.
  force dut.lidar_request_speed=snap_speed;
  for(allow=0;allow<2;allow=allow+1)for(m=0;m<2;m=m+1)begin
   permission=allow?7'b1110111:7'b1100111;
   nav=m;
   for(ss=0;ss<256;ss=ss+1)for(v=0;v<256;v=v+1)begin
    snap_speed=ss;manual_speed=m?255-v:v;nav_speed=m?v:255-v;check();
    if(dut.lidar_context_pending!==(ss!=(m?(allow?((v>12)?12:v):0):v)))$fatal(1,"isolated speed comparator");
   end
  end
  force dut.lidar_request_speed=dut.effective_speed;
  // Every permission predicate and safety-state encoding. Use real upstream
  // desired registers, never force the gated navigation command/speed outputs.
  nav=1;follow=0;stop_now=0;left_mm=199;lok=1;
  force dut.lidar_request_cmd=snap_cmd;
  for(p=0;p<128;p=p+1)for(s=0;s<16;s=s+1)begin
   permission=p;nav_safety=s;
   for(m=0;m<8;m=m+1)begin
    nav=m&1;stop_now=(m>>1)&1;follow=(m>>2)&1;
    for(c=0;c<8;c=c+1)for(sc=0;sc<8;sc=sc+1)begin navcmd=c;cmd=7-c;snap_cmd=sc;check();end
   end
   force dut.lidar_request_cmd=dut.effective_command;
   for(v=0;v<256;v=v+1)begin nav_speed=v;check();end
   force dut.lidar_request_cmd=snap_cmd;
  end
  permission=7'b1110111;nav_safety=0;
  // Every command/snapshot encoding under both modes, STOP, follow and near.
  force dut.lidar_request_cmd=snap_cmd;
  for(m=0;m<64;m=m+1)begin
   {nav,follow,stop_now,lok,rok,laser_ok}=m;
   left_mm=199;right_mm=200;front_mm=199;
   for(c=0;c<64;c=c+1)for(sc=0;sc<8;sc=sc+1)begin
    {navcmd,cmd}=c;snap_cmd=sc;check();
    if(dut.lidar_context_pending!==(snap_cmd!=dut.effective_command))$fatal(1,"isolated command comparator");
   end
  end
  force dut.lidar_request_cmd=dut.effective_command;
  force dut.lidar_request_goal=snap_goal;
  // Entire goal domain as both live and snapshot values, checking equal and
  // unequal navigation pairs; every snapshot vs all manual goal constants.
  for(g=-32768;g<=32767;g=g+1)begin
   nav=1;goal=g;snap_goal=g;stop_now=0;check();stop_now=1;check();
   snap_goal=g^16'h8000;check();
   nav=0;snap_goal=g;
   for(c=0;c<8;c=c+1)begin cmd=c;stop_now=0;check();stop_now=1;check();end
  end
  force dut.lidar_request_goal=dut.lidar_goal_cd;
  // Boolean keys retain their original mismatches independent of wide data.
  nav=1;nav_goal_valid=0;force dut.lidar_request_goal_valid=1;check();
  if(!dut.lidar_context_pending)$fatal(1,"goal-valid mismatch lost");
  force dut.lidar_request_goal_valid=dut.lidar_goal_valid;
  $display("PASS actual TOP manual/nav/STOP/near/follow goal/context equivalence, exhaustive speed pairs, command encodings and signed16 goal domains");$finish;
 end
endmodule
