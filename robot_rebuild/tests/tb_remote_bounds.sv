`timescale 1ns/1ps
module tb_remote_bounds;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;reg [7:0] key=0;reg valid=0,rep=0;
 reg ua=0,uu=0,uf=0;reg [2:0] uc=0;reg [7:0] us=16;
 wire armed,update,fresh,selected;wire [2:0] command;wire [7:0] speed;
 rb_remote_control #(.CLK_HZ(1000)) dut(
  clk,rst,key,valid,rep,ua,uc,us,uu,uf,armed,command,speed,update,fresh,selected);
 task check(input bit good,input string what);if(!good) $fatal(1,"%s",what);endtask
 task cycles(input integer n);repeat(n) begin @(posedge clk);#1;end endtask
 task press(input [7:0] value,input bit repeat_value=0);
  begin @(negedge clk);key=value;rep=repeat_value;valid=1;
   cycles(1);@(negedge clk);valid=0;rep=0;end
 endtask
 initial begin
  cycles(2);rst=1;cycles(2);
  press(8'h15);check(selected && !armed && speed==18 && fresh,"stopped speed up before STOP");
  press(8'h18);check(!armed,"initial motion still requires STOP preparation");
  press(8'h07);check(speed==16 && !armed,"stopped speed down");
  press(8'h1c);press(8'h18);check(fresh && armed && command==1,"remote starts");
  cycles(1000);check(fresh && armed && command==1,"click motion persists through silence");
  press(8'h52);check(command==2 && armed,"direct direction switch");
  press(8'h15);check(speed==18 && command==2,"running speed update");
  press(8'h15,1);check(speed==18,"speed repeat does not ramp");
  press(8'h1c);check(!armed && command==0,"STOP latches stop");
  press(8'h18,1);check(!armed,"repeat cannot restart after STOP");
  press(8'h15);check(speed==20 && !armed,"STOP permits speed adjustment");
  repeat(30) press(8'h15);check(speed==64,"speed maximum saturated");
  repeat(40) press(8'h07);check(speed==4,"speed minimum saturated");
  press(8'h09);check(speed==4 && !armed,"unknown key ignored");
  press(8'h18);check(armed,"fresh direction starts after STOP");
  @(negedge clk);ua=0;uu=1;uf=1;cycles(1);@(negedge clk);uu=0;cycles(3);
  check(!selected && !armed,"UART DISARM stops and hands off");
  press(8'h15,1);check(!selected,"repeat speed cannot acquire control");
  press(8'h15);check(selected && !armed && command==0,"full speed key acquires stopped");
  press(8'h18);check(!armed,"handoff speed key does not prepare motion");
  press(8'h1c);press(8'h18);check(armed,"STOP prepares motion after handoff");
  $display("PASS click remote: persistence, direct switch, idle speed, bounds, repeat and handoff");$finish;
 end
 initial begin #100000;$fatal(1,"click remote timeout");end
endmodule
