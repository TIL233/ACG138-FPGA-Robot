`timescale 1ns/1ps
module fresh_equivalence #(parameter LIMIT=3)(input clk,rst_n,update);
 wire fresh;
 reg [31:0] reference_age;
 rb_fresh #(.CLK_HZ(1000),.TIMEOUT_MS(LIMIT)) dut(clk,rst_n,update,fresh);
 always @(posedge clk or negedge rst_n)
  if(!rst_n) reference_age<=LIMIT;
  else if(update) reference_age<=0;
  else if(reference_age<LIMIT) reference_age<=reference_age+1;
 always @(posedge clk) begin
  #1;
  if(fresh !== (reference_age<LIMIT))
   $fatal(1,"fresh boundary mismatch LIMIT=%0d age=%0d fresh=%b",LIMIT,reference_age,fresh);
 end
endmodule

module tb_fresh;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,update=0;
 fresh_equivalence #(.LIMIT(1)) single_cycle(clk,rst_n,update);
 fresh_equivalence #(.LIMIT(3)) three_cycles(clk,rst_n,update);
 fresh_equivalence #(.LIMIT(17)) long_timeout(clk,rst_n,update);
 integer i;
 initial begin
  repeat(3) @(negedge clk);
  rst_n=1;
  for(i=0;i<200;i=i+1) begin
   @(negedge clk);
   update=(i==0 || i==3 || i==4 || i==20 || i==37 || (i>=60 && i<80));
   if(i==100) rst_n=0;
   if(i==103) rst_n=1;
  end
  @(negedge clk);
  $display("PASS fresh: exact timeout/update/reset/saturation equivalence");
  $finish;
 end
endmodule
