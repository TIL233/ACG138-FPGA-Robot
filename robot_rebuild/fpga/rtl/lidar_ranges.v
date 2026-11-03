`timescale 1ns/1ps
// Optional source fusion. Sonar telemetry retains its original meaning.
// Missing lidar never erases an existing sonar obstacle or claims free space.
module rb_lidar_ranges #(parameter ENABLE_CONTROL=0)(
 input follow_mode,input [15:0] sonar_l,sonar_r,input sonar_l_ok,sonar_r_ok,
 input sonar_l_update,sonar_r_update,lidar_update,input [2:0] lidar_valid,
 input [15:0] lidar_front,
 output [15:0] control_l,control_r,output left_ok,right_ok,left_update,right_update,
 output follow_hold
);
 wire front_ok=ENABLE_CONTROL && lidar_valid[0];
 wire [15:0] base_l=sonar_l_ok?sonar_l:16'hfffe;
 wire [15:0] base_r=sonar_r_ok?sonar_r:16'hfffe;
 assign control_l=follow_mode?16'hfffe:((front_ok && lidar_front<base_l)?lidar_front:base_l);
 assign control_r=follow_mode?16'hfffe:((front_ok && lidar_front<base_r)?lidar_front:base_r);
 assign left_ok=sonar_l_ok || front_ok;
 assign right_ok=sonar_r_ok || front_ok;
 assign left_update=sonar_l_update || (front_ok && lidar_update);
 assign right_update=sonar_r_update || (front_ok && lidar_update);
 // Follow never enters automatic detours; hold at 60cm without latching fault.
 assign follow_hold=follow_mode && front_ok && lidar_front<=600;
endmodule
