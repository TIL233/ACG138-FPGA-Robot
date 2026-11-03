`timescale 1ns/1ps
module tb_camera_sccb_read;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;wire done,scl,ha,la,timed;wire [2:0]phase;wire [15:0]chip;
 tri1 sda;reg absent=0;
 rb_camera_probe #(.CLK_HZ(1000000),.SCL_HZ(100000)) dut(clk,rst,done,phase,chip,ha,la,timed,scl,sda);
 wire [7:0] value=(dut.address==16'h300a) ? 8'h56 : 8'h40;
 // Controlled slave on the actual SDA wire. No force of finished, rddata or
 // ACK outputs: both existing I2C layers and byte sampling execute normally.
 wire acknowledge=dut.bus.i2c_bit_shift.state==8'b00010000;
 wire reading=dut.bus.i2c_bit_shift.state==8'b00001000;
 wire bit_value=value[7-dut.bus.i2c_bit_shift.cnt[4:2]];
 assign sda=(!absent && (acknowledge || (reading && !bit_value)))?1'b0:1'bz;
 task ticks(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 initial begin
  ticks(4);rst=1;wait(done);ticks(2);
  if(chip!=16'h5640 || !ha || !la || timed)$fatal(1,"real receive path must return 0x5640");
  rst=0;absent=1;ticks(4);rst=1;wait(done);ticks(2);
  if(chip!=16'hffff || ha || la || timed)$fatal(1,"absent slave must NACK/read FF");
  $display("PASS camera SCCB read: real controllers and SDA sampling, ID byte order, absent slave NACK");$finish;
 end
 initial begin #3000000;$fatal(1,"SCCB read timeout");end
endmodule
