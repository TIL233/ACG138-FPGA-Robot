`timescale 1ns/1ps
module tb_dvp_init;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,done=0,nack=0;
 wire ready,pwdn,scl,transaction,nacked;wire [7:0] index;
 tri1 sda;
 rb_dvp_init #(.STARTUP_CYCLES(4)) dut(clk,rst,ready,pwdn,scl,sda,index,transaction,nacked);
 integer requests=0;
 always @(posedge clk)if(dut.write_request)requests<=requests+1;
 task acknowledge(input bit failed);
  begin
   wait(dut.write_request);@(negedge clk);nack=failed;
   repeat(3)@(negedge clk);done=1;
   @(negedge clk);done=0;
  end
 endtask
 integer k,before_count;
 initial begin
  force dut.finished=done;force dut.nack=nack;
  repeat(3)@(negedge clk);rst=1;
  acknowledge(1);@(negedge clk);
  if(index!=0 || ready)$fatal(1,"NACK must not advance initialization");
  for(k=0;k<253;k=k+1)begin
   wait(dut.write_request);
   if(index!=k)$fatal(1,"register skipped or repeated after ACK");
   if(k==252 && dut.lut[23:8]!=16'h503d)$fatal(1,"final test-pattern-off register missing");
   acknowledge(0);
  end
  repeat(4)@(negedge clk);
  if(!ready || index!=253 || pwdn || requests!=254)$fatal(1,"must ACK all 253 registers exactly once plus one retry");
  before_count=requests;repeat(40)@(negedge clk);
  if(requests!=before_count)$fatal(1,"no extra writes/autofocus after completion");
  rst=0;repeat(2)@(negedge clk);
  if(ready || index!=0)$fatal(1,"reset must clear initialization");
  $display("PASS DVP initializer: all 253 RGB writes, NACK retry, final register and no extra traffic");$finish;
 end
 initial begin #100000;$fatal(1,"DVP initializer timeout");end
endmodule
