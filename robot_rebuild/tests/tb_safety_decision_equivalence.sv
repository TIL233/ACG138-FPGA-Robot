`timescale 1ns/1ps
// Check the actual combinational decisions against independent minimum and
// wrapped-angle definitions. No clock delay may be added to same-edge data.
module tb_safety_decision_equivalence;
 reg clk=0,rst_n=0,yu=0;
 reg signed [15:0] yaw=0,target=0,stored=0;
 reg [7:0] corrections=0;
 reg [15:0] range_l=0,range_r=0,left_saved=0;
 wire signed [15:0] tl,tr;
 wire brake;wire [3:0] state;wire [15:0] fault;
 rb_safety_control #(.CLK_HZ(1000),.TURN_TRACK_MM(100),
  .ANGLE_TOLERANCE_CD(500),.FINAL_ANGLE_TOLERANCE_CD(1000),
  .RELAX_AFTER_CORRECTIONS(1),.SETTLE_JITTER_COUNTS(1)) dut(
  .clk(clk),.rst_n(rst_n),.link_ok(1'b1),.imu_ok(1'b1),
  .left_ok(1'b1),.right_ok(1'b1),.estop_ok(1'b1),
  .armed(1'b0),.cmd_update(1'b0),.command(3'd0),.speed(8'd16),
  .dl(16'd1000),.dr(16'd1000),.left_update(1'b0),.right_update(1'b0),
  .pos_l(32'sd0),.pos_r(32'sd0),.motor_fault(1'b0),.yaw(yaw),.yaw_update(yu),
  .target_l(tl),.target_r(tr),.brake(brake),.state(state),.fault(fault),
  .turn_error(),.turn_error_valid());

 task check_pose;
  integer chosen,delta,error,magnitude;
  bit close,final_close,ok,error_large;
  begin
   #1;
   chosen=yu?$signed(yaw):$signed(stored);
   delta=$signed(target)-chosen;error=delta;
   if(error>18000)error=error-36000;
   else if(error< -18000)error=error+36000;
   magnitude=(error<0)?-error:error;
   close=(magnitude<=500);final_close=(magnitude<=1000);
   ok=close || (corrections>=1 && final_close);error_large=(magnitude>3000);
   if(dut.settled_delta!==delta || dut.settled_error!==error ||
      dut.settled_error_abs!==magnitude || dut.leg_heading_close!==close ||
      dut.leg_final_close!==final_close || dut.leg_heading_ok!==ok ||
      dut.leg_error_large!==error_large)
    $fatal(1,"pose decision changed target=%0d fresh=%0d stored=%0d update=%b correction=%0d raw=%0d wrapped=%0d",
     $signed(target),$signed(yaw),$signed(stored),yu,corrections,delta,error);
  end
 endtask
 task check_ranges;
  reg [15:0] minimum;
  begin
   #1;minimum=(range_l<range_r)?range_l:range_r;
   if(dut.observed_distance!==minimum ||
      dut.observed_below_clear!==(minimum<400) ||
      dut.left_at_least_observed!==(left_saved>=minimum) ||
      dut.observed_clear!==(minimum>=400) ||
      dut.observed_resume_clear!==(minimum>300))
    $fatal(1,"minimum decision changed left=%0d right=%0d saved=%0d minimum=%0d",
     range_l,range_r,left_saved,minimum);
  end
 endtask
 function [15:0] boundary;
  input integer i;
  begin
   case(i)
    0:boundary=0;1:boundary=19;2:boundary=20;3:boundary=199;
    4:boundary=200;5:boundary=299;6:boundary=300;7:boundary=301;
    8:boundary=399;9:boundary=400;10:boundary=401;11:boundary=1000;
    12:boundary=65533;13:boundary=65534;default:boundary=65535;
   endcase
  end
 endfunction
 integer delta,a,b,c,k,pose_checks=0,range_checks=0;
 reg signed [15:0] current;
 reg [31:0] random_state=32'h5a39c017;
 initial begin
  force dut.turn_heading=target;
  force dut.yaw_sample=stored;
  force dut.correction_count=corrections;
  force dut.observed_l=range_l;
  force dut.observed_r=range_r;
  force dut.left_clearance=left_saved;
  // Cover every legal target-current difference, both sides of the +/-180
  // seam, with contradictory fresh/stored poses and both relaxation states.
  for(delta=-36000;delta<=36000;delta=delta+1)begin
   target=(delta>=0)?18000:-18000;current=$signed(target)-delta;
   for(k=0;k<2;k=k+1)begin
    corrections=k;yaw=current;stored=target;
    yu=0;check_pose();yu=1;check_pose();
    yaw=target;stored=current;
    yu=0;check_pose();yu=1;check_pose();pose_checks=pose_checks+4;
   end
  end
  // Exact threshold/tie cases and full-width FAR/sentinel values must keep
  // their original inequality, especially400 inclusive and300 exclusive.
  for(a=0;a<15;a=a+1)for(b=0;b<15;b=b+1)for(c=0;c<15;c=c+1)begin
   range_l=boundary(a);range_r=boundary(b);left_saved=boundary(c);
   check_ranges();range_checks=range_checks+1;
  end
  for(k=0;k<10000;k=k+1)begin
   random_state={random_state[30:0],random_state[31]^random_state[21]^random_state[1]^random_state[0]};
   range_l=random_state[15:0];range_r=random_state[31:16];
   random_state={random_state[30:0],random_state[31]^random_state[21]^random_state[1]^random_state[0]};
   left_saved=random_state[15:0];check_ranges();range_checks=range_checks+1;
  end
  $display("PASS safety decision equivalence: %0d exhaustive fresh/stored wrapped-pose decisions, %0d minimum/tie/threshold/sentinel decisions, no added cycle",pose_checks,range_checks);
  $finish;
 end
 initial begin #1000000;$fatal(1,"safety decision equivalence test hung");end
endmodule
