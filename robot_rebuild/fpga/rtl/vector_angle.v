`timescale 1ns/1ps
// Sequential normalization + CORDIC atan2 for the navigation target vector.
// Positive output is a physical left/CCW turn; centidegrees, wrap-safe.
module rb_vector_angle(
 input clk,rst_n,clear,start,input signed [53:0] x,y,
 output reg valid,output reg signed [15:0] angle
);
 reg [1:0] phase;reg [3:0] iteration;
 reg signed [54:0] nx,ny;
 reg signed [19:0] vx,vy;
 reg signed [17:0] accumulator;
 function signed [17:0] atan_cd;
  input [3:0] n;
  begin case(n)
   0:atan_cd=4500;1:atan_cd=2657;2:atan_cd=1404;3:atan_cd=713;
   4:atan_cd=358;5:atan_cd=179;6:atan_cd=90;7:atan_cd=45;
   8:atan_cd=22;9:atan_cd=11;10:atan_cd=6;11:atan_cd=3;
   12:atan_cd=1;13:atan_cd=1;default:atan_cd=0;
  endcase end
 endfunction
 function signed [15:0] wrap;
  input signed [17:0] a;
  begin if(a>18000)wrap=a-36000;else if(a< -18000)wrap=a+36000;else wrap=a;end
 endfunction
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin phase<=0;iteration<=0;nx<=0;ny<=0;vx<=0;vy<=0;accumulator<=0;valid<=0;angle<=0;end
  else if(clear)begin phase<=0;valid<=0;end
  else if(start)begin
   nx<={x[53],x};ny<={y[53],y};valid<=0;phase<=(x==0 && y==0)?0:1;
  end else case(phase)
   1:begin
    if(nx>131071 || nx< -131071 || ny>131071 || ny< -131071)begin nx<=nx>>>1;ny<=ny>>>1;end
    else begin
     // Scale small vectors up as well; otherwise the integer CORDIC would
     // lose a nearby waypoint's angle after the first few shifts.
     if(nx<32768 && nx> -32768 && ny<32768 && ny> -32768)begin nx<=nx<<<1;ny<=ny<<<1;end
     else begin
      if(nx<0)begin vx<=-nx;vy<=-ny;accumulator<=ny>=0?18'sd18000:-18'sd18000;end
      else begin vx<=nx;vy<=ny;accumulator<=0;end
      iteration<=0;phase<=2;
     end
    end
   end
   2:begin
    if(vy>=0)begin vx<=vx+(vy>>>iteration);vy<=vy-(vx>>>iteration);accumulator<=accumulator+atan_cd(iteration);end
    else begin vx<=vx-(vy>>>iteration);vy<=vy+(vx>>>iteration);accumulator<=accumulator-atan_cd(iteration);end
    if(iteration==13)phase<=3;else iteration<=iteration+1'b1;
   end
   3:begin angle<=wrap(accumulator);valid<=1;phase<=0;end
   default:;
  endcase
endmodule
