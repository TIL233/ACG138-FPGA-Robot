`timescale 1ns/1ps
// DYP automatic UART FF high low sum. Only checked no-target/overrange frames
// count as FAR; UART silence, checksum failure and interference remain faults.
// NO_ECHO_CODE is a module-profile setting, not an inference from missing UART.
module rb_ultrasonic #(parameter CLK_HZ=50000000, BAUD=115200, MAX_MM=4600,
 parameter NO_ECHO_CODE=16'hffff,TIMEOUT_MS=500)(
 input clk,rst_n,rx,output reg [15:0] distance_mm,
 output reg update,output wire fresh);
 wire [7:0] byte_data;wire byte_valid;
 rb_uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) uart(clk,rst_n,rx,byte_data,byte_valid);
 reg [1:0] state;
 reg [7:0] hi,lo;
 reg [31:0] gap;
 reg good;
 localparam GAP_LIMIT=(CLK_HZ/1000)*20;
 wire timed;
 wire [7:0] sum=8'hff+hi+lo;
 localparam [15:0] FAR=16'hfffe; // Normalized telemetry sentinel, NOT a raw error.
 wire [15:0] raw_distance={hi,lo};
 rb_fresh #(.CLK_HZ(CLK_HZ),.TIMEOUT_MS(TIMEOUT_MS)) alive(clk,rst_n,update,timed);
 assign fresh=timed && good;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin state<=0;hi<=0;lo<=0;gap<=0;distance_mm<=16'hffff;update<=0;good<=0;end
  else begin
   update<=0;
   if(gap<GAP_LIMIT) gap<=gap+1;
   else state<=0;
   if(byte_valid) begin
    gap<=0;
    // Discard a partial frame before consuming a byte at/after its deadline.
    // Nonblocking state<=0 alone would be overridden by the old case state.
    if(gap>=GAP_LIMIT) state<=(byte_data==8'hff)?1:0;
    else case(state)
     0: if(byte_data==8'hff) state<=1;
     1: begin hi<=byte_data;state<=2;end
     2: begin lo<=byte_data;state<=3;end
     3: begin
      state<=0;
      if(byte_data==sum) begin
       // Raw FFFE is DYP acoustic interference; reserve FFFD as an error too.
       // These must never be mistaken for a large numeric distance.
       if(raw_distance==16'hfffe || raw_distance==16'hfffd) begin
        good<=0;distance_mm<=16'hffff;
       end else if(raw_distance==NO_ECHO_CODE || (raw_distance>MAX_MM && raw_distance<16'hfffd)) begin
        distance_mm<=FAR;update<=1;good<=1;
       end else if(raw_distance>=20 && raw_distance<=MAX_MM) begin
        distance_mm<=raw_distance;update<=1;good<=1;
       end else begin good<=0;distance_mm<=16'hffff;end
      end else begin good<=0;distance_mm<=16'hffff;if(byte_data==8'hff) state<=1;end
     end
    endcase
   end
  end
 end
endmodule
