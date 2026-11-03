`timescale 1ns/1ps
// A bounded, read-only SCCB probe before the normal register initialization.
// Failure releases the existing initializer as well: diagnostics never hold it.
module rb_camera_probe #(parameter CLK_HZ=50000000,SCL_HZ=100000)(
 input clk,rst_n,output reg done,output reg [2:0] phase,
 output reg [15:0] chip_id,output reg high_ack,low_ack,timed_out,
 output scl,inout sda);
 reg request;reg [15:0] address;reg [31:0] timer;
 wire finished,nack;wire [7:0] value;
 i2c_control #(.SYS_CLOCK(CLK_HZ),.SCL_CLOCK(SCL_HZ)) bus(
  .Clk(clk),.Rst_n(rst_n && !done),.wrreg_req(1'b0),.rdreg_req(request),
  .addr(address),.addr_mode(1'b1),.wrdata(8'd0),.rddata(value),.device_id(8'h78),
  .RW_Done(finished),.ack(nack),.dly_cnt_max(32'd0),.i2c_sclk(scl),.i2c_sdat(sda));
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin done<=0;phase<=0;chip_id<=0;high_ack<=0;low_ack<=0;
   timed_out<=0;request<=0;address<=16'h300a;timer<=0;end
  else begin
   request<=0;
   case(phase)
    0:if(timer>=CLK_HZ/40)begin timer<=0;phase<=1;end else timer<=timer+1;
    1:begin address<=16'h300a;request<=1;timer<=0;phase<=2;end
    2:if(finished)begin chip_id[15:8]<=value;high_ack<=!nack;timer<=0;phase<=3;end
      else if(timer>=CLK_HZ/50)begin timed_out<=1;done<=1;phase<=5;end
      else timer<=timer+1;
    3:begin address<=16'h300b;request<=1;timer<=0;phase<=4;end
    4:if(finished)begin chip_id[7:0]<=value;low_ack<=!nack;done<=1;phase<=5;end
      else if(timer>=CLK_HZ/50)begin timed_out<=1;done<=1;phase<=5;end
      else timer<=timer+1;
    default:begin done<=1;phase<=5;end
   endcase
  end
 end
endmodule
