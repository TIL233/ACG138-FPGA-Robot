`timescale 1ns/1ps
module tb_navigation_near_equivalence;
 reg follow=0,lok=0,rok=0;reg [2:0] valid=0;
 reg [15:0] dl=0,dr=0,front=0;wire [15:0] l0,r0,l1,r1;
 rb_lidar_ranges #(.ENABLE_CONTROL(0)) r_off(follow,dl,dr,lok,rok,1'b0,1'b0,1'b0,valid,front,l0,r0,,,,,);
 rb_lidar_ranges #(.ENABLE_CONTROL(1)) r_on(follow,dl,dr,lok,rok,1'b0,1'b0,1'b0,valid,front,l1,r1,,,,,);
 wire near_off=!follow && ((lok && dl<200)||(rok && dr<200));
 wire near_on=!follow && ((lok && dl<200)||(rok && dr<200)||(valid[0] && front<200));
 integer a,b,c,m,v,ch;reg [15:0] points[0:7];
 task check;begin
  #1;if(near_off!==((l0<200)||(r0<200)) || near_on!==((l1<200)||(r1<200)))
   $fatal(1,"near predicate differs from actual fusion l=%0d r=%0d f=%0d mode=%0d",dl,dr,front,m);
 end endtask
 initial begin
  points[0]=0;points[1]=1;points[2]=198;points[3]=199;points[4]=200;points[5]=201;points[6]=65534;points[7]=65535;
  for(m=0;m<64;m=m+1)begin
   {follow,lok,rok,valid}=m;
   for(a=0;a<8;a=a+1)for(b=0;b<8;b=b+1)for(c=0;c<8;c=c+1)begin
    dl=points[a];dr=points[b];front=points[c];check();
   end
  end
  for(m=0;m<16;m=m+1)begin
   {follow,lok,rok,valid[0]}=m;valid[2:1]=0;
   for(ch=0;ch<3;ch=ch+1)for(v=0;v<65536;v=v+1)begin
    dl=200;dr=200;front=200;
    case(ch)0:dl=v;1:dr=v;2:front=v;endcase
    check();
   end
  end
  $display("PASS navigation near predicate: full mode/cross-boundary fusion and each full unsigned16 distance");$finish;
 end
endmodule
