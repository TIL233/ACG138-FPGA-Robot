`timescale 1ns/1ps
// Fixed RGB565 DVP path. Register table and SCCB engine are supplied board IP.
// This module intentionally does not contain JPEG, autofocus or other sensors.
// OV5647 CSI-2 modules on RK FPC connectors cannot use this parallel interface.
module rb_dvp_init #(
 parameter IMAGE_WIDTH=640,IMAGE_HEIGHT=480,IMAGE_FLIP_EN=1,IMAGE_MIRROR_EN=0,
 parameter STARTUP_CYCLES=1050624
)(
 input Clk,Rst_n,output reg Init_Done,
 output camera_pwdn,output i2c_sclk,inout i2c_sdat,
 output [7:0] diag_index,output diag_done,diag_nack
);
 reg [31:0] startup;
 reg [7:0] index;
 reg [1:0] state;
 reg write_request;
 wire [23:0] lut;
 wire finished,nack;
 assign camera_pwdn=1'b0;
 assign diag_index=index;
 assign diag_done=finished;
 assign diag_nack=nack;
 ov5640_init_table_rgb #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),
  .IMAGE_FLIP_EN(IMAGE_FLIP_EN),.IMAGE_MIRROR_EN(IMAGE_MIRROR_EN)) registers(
  .addr(index),.clk(Clk),.q(lut));
 i2c_control bus(.Clk(Clk),.Rst_n(Rst_n),.wrreg_req(write_request),.rdreg_req(1'b0),
  .addr(lut[23:8]),.addr_mode(1'b1),.wrdata(lut[7:0]),.rddata(),.device_id(8'h78),
  .RW_Done(finished),.ack(nack),.dly_cnt_max(index==1?32'h40000:32'd0),
  .i2c_sclk(i2c_sclk),.i2c_sdat(i2c_sdat));
 always @(posedge Clk or negedge Rst_n)begin
  if(!Rst_n)begin startup<=0;index<=0;state<=0;write_request<=0;Init_Done<=0;end
  else begin
   write_request<=0;
   if(startup<STARTUP_CYCLES)startup<=startup+1'b1;
   if(!Init_Done)case(state)
    0:if(startup>=STARTUP_CYCLES)state<=1;
    1:begin write_request<=1;state<=2;end
    2:if(finished)begin
       if(!nack)begin
        if(index==252)begin Init_Done<=1;index<=253;end
        else index<=index+1'b1;
       end
       state<=1; // NACK retries this same register; it never claims success.
      end
    default:state<=0;
   endcase
  end
 end
endmodule
