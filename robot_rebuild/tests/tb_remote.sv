`timescale 1ns/1ps
module tb_remote;
 reg clk=0;always #500 clk=~clk; // 1 MHz exercises pulse edges off the 10us tick.
 reg rst=0,ir=1;
 wire [7:0] key;wire valid,rep;
 rb_ir_nec #(.CLK_HZ(1000000)) dec(clk,rst,ir,key,valid,rep);
 reg ua=0,uu=0,uf=0;reg [2:0] uc=0;reg [7:0] us=16;
 wire armed,update,fresh,selected;wire [2:0] command;wire [7:0] speed;
 rb_remote_control #(.CLK_HZ(1000000)) dut(clk,rst,key,valid,rep,ua,uc,us,uu,uf,
  armed,command,speed,update,fresh,selected);
 reg imu=1,left_ok=1,right_ok=1,estop=1;
 reg [15:0] dl=1000,dr=1000;
 wire signed [15:0] tl,tr;wire brake;wire [3:0] state;wire [15:0] fault;
 rb_safety_control #(.CLK_HZ(1000000)) safety(
  .clk(clk),.rst_n(rst),.link_ok(fresh),.imu_ok(imu),.left_ok(left_ok),.right_ok(right_ok),
  .estop_ok(estop),.armed(armed),.cmd_update(update),.command(command),.speed(speed),
  .dl(dl),.dr(dr),.left_update(1'b0),.right_update(1'b0),.pos_l(32'sd0),.pos_r(32'sd0),
  .motor_fault(1'b0),.yaw(16'sd0),.yaw_update(1'b1),.target_l(tl),.target_r(tr),.brake(brake),.state(state),.fault(fault));
 integer count=0,repeats=0;
 always @(posedge clk) if(valid) begin count=count+1;if(rep) repeats=repeats+1;end
 task pulse(input bit level,input integer us_time);
  begin ir=level; #(us_time*1000);end
 endtask
 task frame(input [7:0] cmd,input bit corrupt=0,input [7:0] addr=0);
  reg [31:0] word;integer n;
  begin
   word={~cmd,cmd,8'hff,addr};if(corrupt) word[24]=~word[24];
   pulse(1,2000);pulse(0,9000);pulse(1,4500);
   for(n=0;n<32;n=n+1) begin pulse(0,560);pulse(1,word[n]?1690:560);end
   pulse(0,560);pulse(1,100);
  end
 endtask
 task repeat_frame;
  begin pulse(1,40000);pulse(0,9000);pulse(1,2250);pulse(0,560);pulse(1,100);end
 endtask
 task check(input bit good,input string what);
  if(!good) $fatal(1,"%s",what);
 endtask
 task uart(input bit arm,input [2:0] cmd);
  begin @(negedge clk);ua=arm;uc=cmd;uf=1;uu=1;@(negedge clk);uu=0;repeat(4) @(negedge clk);end
 endtask
 integer before_count;
 initial begin
  #2000;rst=1;#730;
  repeat_frame();check(count==0,"orphan repeat rejected");
  frame(8'h18);check(!selected && brake,"motion cannot acquire control without STOP");
  frame(8'h1c);check(selected && !armed && fresh && brake,"STOP acquires and disarms");
  frame(8'h18);check(armed && command==1 && !brake && tl==16 && tr==16,"forward");
  repeat_frame();check(repeats==1 && fresh && !brake,"held key repeat");
  frame(8'h52);check(command==2 && tl==-16 && tr==-16,"reverse");
  frame(8'h08);check(command==3 && tl==8 && tr==16,"left");
  frame(8'h5a);check(command==4 && tl==16 && tr==8,"right");
  frame(8'h0c);check(command==5 && tl==-16 && tr==16,"spin left");
  frame(8'h5e);check(command==6 && tl==16 && tr==-16,"spin right");
  frame(8'h15);check(speed==18,"speed up");repeat_frame();check(speed==18,"speed repeat does not ramp");
  frame(8'h07);check(speed==16,"speed down");
  before_count=count;frame(8'h18,1);check(count==before_count,"corrupt complement rejected");
  before_count=count;frame(8'h18,0,8'h12);check(count==before_count,"other address rejected");
  // Unsupported EQ cannot enable the old open-loop self-test.
  frame(8'h09);check(command==6 && speed==16,"unsupported EQ ignored");
  pulse(1,410000);check(fresh && !brake && fault==0,"click command persists after release");
  frame(8'h18);check(fresh && !brake && command==1,"direction switches without STOP");
  frame(8'h1c);check(!armed && fault==0,"STOP clears fault");
  frame(8'h18);check(!brake,"motion restored after STOP");
  dl=199;#2000;check(brake && tl==0 && tr==0,"near obstacle still stops immediately");
  dl=1000;frame(8'h1c);frame(8'h18);
  estop=0;#2000;check(brake,"emergency stop preserved");estop=1;
  frame(8'h18);check(brake,"restoring emergency does not auto resume");
  frame(8'h1c);imu=0;frame(8'h18);check(brake,"missing IMU inhibits remote");imu=1;
  frame(8'h1c);left_ok=0;frame(8'h18);check(!brake,"missing range permits remote by user policy");left_ok=1;
  frame(8'h1c);frame(8'h18);check(!brake,"ready for arbitration");
  uart(1,2);check(selected && command==1,"armed UART cannot steal remote");
  uart(0,0);check(!selected && !armed && brake && fault==0,"UART DISARM handoff is observed");
  uart(1,2);check(command==2 && !brake,"UART can drive after explicit DISARM");
  frame(8'h1c);check(selected && brake,"remote STOP overrides UART motion");
  frame(8'h18);check(!brake,"remote resumes after explicit handoff");
  frame(8'h1c);pulse(1,410000);repeat_frame();check(brake && !armed,"late repeat cannot resume after STOP");
  // Glitch and malformed leader must not form a key.
  before_count=count;pulse(0,20);pulse(1,20);pulse(0,2000);pulse(1,20000);
  check(count==before_count,"noise rejected");
  $display("PASS remote NEC, click latch, repeats, mapping, arbitration and safety integration");$finish;
 end
 initial begin #(64'd10000000000);$fatal(1,"remote test timeout");end
endmodule
