`timescale 1ns/1ps
module tb_angles;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,start=0;reg signed [15:0] q0,q1,q2,q3;
 wire signed [15:0] roll,pitch,yaw;wire done,busy;
 quat2euler dut(clk,rst_n,start,q0,q1,q2,q3,roll,pitch,yaw,done,busy);
 integer fd,n,w,x,y,z,r,p,h,count=0,failures=0,d;
 task check(input integer actual,expected,input string axis);
  d=actual-expected;if(d>18000)d=d-36000;if(d< -18000)d=d+36000;
  if(actual>18005 || actual< -18005 || d>30 || d< -30) begin
   $display("FAIL vector %0d %s actual=%0d expected=%0d",count,axis,actual,expected);failures=failures+1;
  end
 endtask
 initial begin
  repeat(3) @(negedge clk);rst_n=1;
  fd=$fopen("angle_vectors.txt","r");if(!fd)$fatal(1,"missing vectors");
  while(!$feof(fd)) begin
   n=$fscanf(fd,"%d %d %d %d %d %d %d\n",w,x,y,z,r,p,h);
   if(n==7) begin
    @(negedge clk);q0=w;q1=x;q2=y;q3=z;start=1;
    @(negedge clk);start=0;wait(done);#1;
    check(roll,r,"roll");check(pitch,p,"pitch");check(yaw,h,"yaw");count=count+1;
   end
  end
  if(failures)$fatal(1,"%0d angle errors",failures);
  $display("PASS %0d quaternion vectors against floating-point reference",count);$finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
