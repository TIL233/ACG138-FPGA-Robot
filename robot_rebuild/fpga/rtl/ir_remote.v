`timescale 1ns/1ps
// NEC/HT6221, active-low demodulated input. A command is accepted only after
// the complete frame including stop pulse. Repeat frames never invent a key.
module rb_ir_nec #(parameter CLK_HZ=50000000, ADDRESS=8'h00)(
 input clk,rst_n,ir_rx,output reg [7:0] key,
 output reg valid,output reg repeat_key
);
 localparam DIV=(CLK_HZ/100000<1)?1:CLK_HZ/100000;
 reg [15:0] divider;
 wire tick=divider==DIV-1;
 reg [2:0] sync_ir;
 reg previous;
 wire falling=previous && !sync_ir[2];
 wire rising=!previous && sync_ir[2];
 reg [11:0] duration;
 reg [14:0] age;
 reg cached;
 reg [31:0] bits;
 reg [5:0] index;
 reg [2:0] state;
 localparam IDLE=0,LEAD=1,GAP=2,BIT_LOW=3,BIT_HIGH=4,STOP=5,REPEAT_STOP=6;
 wire short_pulse=duration>=40 && duration<=72;
 wire long_pulse=duration>=130 && duration<=210;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   divider<=0;sync_ir<=7;previous<=1;duration<=0;age<=15000;cached<=0;
   bits<=0;index<=0;state<=IDLE;key<=0;valid<=0;repeat_key<=0;
  end else begin
   sync_ir<={sync_ir[1:0],ir_rx};previous<=sync_ir[2];
   divider<=tick?0:divider+1'b1;
   if(falling || rising) duration<=0;
   else if(tick && duration!=4095) duration<=duration+1'b1;
   if(tick && age<15000) age<=age+1'b1;
   valid<=0;repeat_key<=0;
   if(duration>=1200 && !falling && !rising) state<=IDLE;
   else case(state)
    IDLE: if(falling) begin state<=LEAD;bits<=0;index<=0;end
    LEAD: if(rising) state<=(duration>=700 && duration<=1100)?GAP:IDLE;
    GAP: if(falling) begin
     if(duration>=350 && duration<=560) state<=BIT_LOW;
     else if(duration>=180 && duration<=280) state<=REPEAT_STOP;
     else state<=IDLE;
    end
    BIT_LOW: if(rising) state<=short_pulse?BIT_HIGH:IDLE;
    BIT_HIGH: if(falling) begin
     if(short_pulse || long_pulse) begin
      bits<={long_pulse,bits[31:1]};
      if(index==31) state<=STOP;
      else begin index<=index+1'b1;state<=BIT_LOW;end
     end else state<=IDLE;
    end
    STOP: if(rising) begin
     state<=IDLE;
     // Fixed handset address, plus command complement. Address complement
     // is intentionally not required (HT6221 variants use extended address).
     if(short_pulse && bits[7:0]==ADDRESS && bits[31:24]==~bits[23:16]) begin
      key<=bits[23:16];valid<=1;cached<=1;age<=0;
     end else cached<=0;
    end
    REPEAT_STOP: if(rising) begin
     state<=IDLE;
     if(short_pulse && cached && age<15000) begin valid<=1;repeat_key<=1;age<=0;end
    end
    default: state<=IDLE;
   endcase
  end
 end
endmodule

// Click-to-run: key 5 stops and prepares remote control; a complete direction
// frame latches motion until STOP, reset, UART DISARM, or a safety fault.
// Silence/repeat frames do not stop or re-enable motion. This handset has no
// continuous heartbeat, so IR silence is not a disconnection indication.
module rb_remote_control #(parameter CLK_HZ=50000000)(
 input clk,rst_n,input [7:0] key,input key_valid,key_repeat,
 input uart_armed,input [2:0] uart_command,input [7:0] uart_speed,
 input uart_update,uart_fresh,
 output armed,output [2:0] command,output [7:0] speed,
 output update,output fresh,output reg ir_selected
);
 reg ir_armed,ir_ready,ir_update,ir_fresh,uart_stop;
 reg [2:0] ir_command;
 reg [7:0] ir_speed;
 reg [2:0] mapped;
 always @* begin
  case(key)
   8'h18:mapped=1;8'h52:mapped=2;8'h08:mapped=3;
   8'h5a:mapped=4;8'h0c:mapped=5;8'h5e:mapped=6;
   default:mapped=0;
  endcase
 end
 assign armed=uart_stop?1'b0:(ir_selected?ir_armed:uart_armed);
 assign command=uart_stop?3'd0:(ir_selected?ir_command:uart_command);
 assign speed=uart_stop?8'd0:(ir_selected?ir_speed:uart_speed);
 assign update=uart_stop || (ir_selected?ir_update:uart_update);
 assign fresh=uart_stop || (ir_selected?ir_fresh:uart_fresh);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   ir_selected<=0;ir_armed<=0;ir_ready<=0;ir_update<=0;uart_stop<=0;
   ir_command<=0;ir_speed<=16;ir_fresh<=0;
  end else begin
   ir_update<=0;
   uart_stop<=uart_update && !uart_armed;
   if(key_valid && key==8'h1c) begin
    ir_selected<=1;ir_armed<=0;ir_command<=0;ir_ready<=1;
    ir_update<=1;ir_fresh<=1;
   end else if(uart_update && !uart_armed) begin
    ir_selected<=0;ir_armed<=0;ir_ready<=0;ir_command<=0;ir_fresh<=0;
   end else if(key_valid && !key_repeat && (key==8'h15 || key==8'h07)) begin
    // Speed changes work while stopped as well as moving. Claiming control
    // from UART stops first and still requires key 5 before starting motion.
    if(key==8'h15 && ir_speed<64) ir_speed<=ir_speed+8'd2;
    if(key==8'h07 && ir_speed>4) ir_speed<=ir_speed-8'd2;
    if(!ir_selected) begin ir_armed<=0;ir_ready<=0;ir_command<=0;end
    ir_selected<=1;ir_update<=1;ir_fresh<=1;
   end else if(ir_selected && ir_ready && key_valid && !key_repeat && mapped!=0) begin
    ir_armed<=1;ir_command<=mapped;ir_update<=1;ir_fresh<=1;
   end
  end
 end
endmodule
