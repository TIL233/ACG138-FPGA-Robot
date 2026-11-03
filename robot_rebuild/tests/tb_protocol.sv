`timescale 1ns/1ps
module tb_protocol;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0;reg [7:0] d=0;reg valid=0;
 wire armed,update,fresh,mode_nav,mode_follow;wire [2:0] command;wire [7:0] speed;
 rb_command_rx #(.CLK_HZ(1000000)) dut(clk,rst_n,d,valid,armed,command,speed,update,fresh,mode_nav,mode_follow);
 integer updates=0;
 always @(posedge clk) if(update) updates<=updates+1;
 function [15:0] crcbyte(input [15:0] a,input [7:0] b);
  reg [15:0] c;integer k;begin c=a^{b,8'd0};for(k=0;k<8;k=k+1) c=c[15]?(c<<1)^16'h1021:c<<1;crcbyte=c;end
 endfunction
 task cycles(input integer n);repeat(n) begin @(posedge clk);#1;end endtask
 task send(input [7:0] b);begin @(negedge clk);d=b;valid=1;cycles(1);@(negedge clk);valid=0;end endtask
 task packet(input [7:0] seq,input [7:0] cmd,input bit bad);
  reg [15:0] crc;reg [7:0] body[0:5];integer k;
  begin body[0]=1;body[1]=seq;body[2]=1;body[3]=cmd;body[4]=16;body[5]=0;
   crc=16'hffff;send(8'ha5);send(8'h5a);
   for(k=0;k<6;k=k+1) begin send(body[k]);crc=crcbyte(crc,body[k]);end
   send(crc[7:0]^{7'd0,bad});send(crc[15:8]);send(13);send(10);cycles(3);
  end
 endtask
 task prefix(input [7:0] seq,input [7:0] cmd);
  reg [15:0] crc;reg [7:0] body[0:5];integer k;
  begin body[0]=1;body[1]=seq;body[2]=1;body[3]=cmd;body[4]=16;body[5]=0;
   crc=16'hffff;send(8'ha5);send(8'h5a);
   for(k=0;k<6;k=k+1) begin send(body[k]);crc=crcbyte(crc,body[k]);end
   send(crc[7:0]);send(crc[15:8]);send(13);
  end
 endtask
 task check(input bit yes,input string msg);if(!yes) $fatal(1,"%s",msg);endtask
 reg sensor_rx=1;wire [15:0] distance_mm;wire su,sf;
 rb_ultrasonic #(.CLK_HZ(1000000)) sensor(clk,rst_n,sensor_rx,distance_mm,su,sf);
 task uart_byte(input [7:0] b);integer k;begin
  @(negedge clk);sensor_rx=0;cycles(9);
  for(k=0;k<8;k=k+1) begin @(negedge clk);sensor_rx=b[k];cycles(9);end
  @(negedge clk);sensor_rx=1;cycles(12);
 end endtask
 wire tx;wire [7:0] td;wire tv;
 rb_telemetry #(.CLK_HZ(1000000)) tele(clk,rst_n,8'd3,8'h9f,8'd1,8'd16,
  16'd199,16'hffff,-16'sd16,16'sd16,-16'sd1234,16'sd250,16'sd17999,16'h0800,tx,
  8'd2,3'd1,4'd3,8'h99,-32'sd1000,32'sd2500,-16'sd9000,8'd7,8'd0,240'd0,16'd0,16'd0,320'd0,256'd0);
 rb_uart_rx #(.CLK_HZ(1000000)) tr(clk,rst_n,tx,td,tv);
 reg [7:0] received[0:51];integer received_n=0;
 always @(posedge clk) if(tv && received_n<52) begin received[received_n]<=td;received_n<=received_n+1;end
 integer k;reg [15:0] tc;
 initial begin
  cycles(4);rst_n=1;cycles(4);
  packet(1,1,0);check(armed && command==1 && speed==16 && fresh && updates==1,"command fields/CRC");
  packet(1,2,0);check(command==1 && updates==1,"duplicate sequence rejected");
  packet(2,2,1);check(command==1 && updates==1,"bad CRC rejected");
  packet(2,7,0);check(command==1 && updates==1,"invalid command rejected");
  send(8'ha5);send(8'h5a);send(1);cycles(21000);packet(2,2,0);
  check(command==2 && updates==2,"partial packet timeout and resync");
  // The old state must be abandoned even when LF arrives on the very first
  // expired edge. Waiting longer would miss this nonblocking-assignment bug.
  prefix(3,3);cycles(20000);send(10);cycles(3);
  check(command==2 && updates==2,"deadline LF cannot complete an expired command");
  packet(3,3,0);check(command==3 && updates==3,"command recovers after exact deadline");
  uart_byte(8'hff);uart_byte(0);uart_byte(199);uart_byte(8'hc6);cycles(3);
  check(sf && distance_mm==199,"valid sensor mm");
  uart_byte(8'hff);uart_byte(0);uart_byte(200);uart_byte(0);cycles(3);
  check(!sf,"bad sensor checksum invalidates driving freshness");
  uart_byte(8'hff);uart_byte(0);uart_byte(0);uart_byte(8'hff);cycles(3);
  check(!sf && distance_mm==16'hffff,"no echo invalidates immediately");
  uart_byte(8'hff);uart_byte(1);uart_byte(0);uart_byte(0);cycles(3);
  check(sf && distance_mm==256,"sensor recovers with checksum");
  // Align a physical UART checksum byte with the first expired parser edge.
  // At DIV=9 its decoder publishes at edge 89 and the parser consumes at 90.
  uart_byte(8'hff);uart_byte(1);uart_byte(1);
  wait(sensor.gap==19911);uart_byte(1);cycles(3);
  check(distance_mm==256,"deadline checksum cannot complete an expired sensor frame");
  uart_byte(8'hff);uart_byte(1);uart_byte(1);uart_byte(1);cycles(3);
  check(sf && distance_mm==257,"sensor recovers after exact deadline");
  wait(received_n>=28);cycles(3);
  check(received[0]==8'h5a && received[1]==8'ha5 && received[26]==13 && received[27]==10,"telemetry framing");
  check(received[4]==3 && received[5]==8'h9f && received[8]==199 && received[9]==0,"telemetry field offsets");
  check(received[12]==8'hf0 && received[13]==8'hff && received[16]==8'h2e && received[17]==8'hfb,"signed telemetry units");
  tc=16'hffff;for(k=2;k<24;k=k+1) tc=crcbyte(tc,received[k]);
  check({received[25],received[24]}==tc,"telemetry CRC");
  wait(received_n==52);cycles(3);
  check(received[28]==8'h5a && received[29]==8'hc3 && received[50]==13 && received[51]==10,"navigation telemetry framing");
  check(received[32]==2 && received[33]==1 && received[34]==3 && received[35]==8'h99,"navigation state snapshot");
  check({received[39],received[38],received[37],received[36]}==-32'sd1000 &&
        {received[43],received[42],received[41],received[40]}==32'd2500,"navigation signed coordinates");
  check({received[45],received[44]}==-16'sd9000 && received[46]==7 && received[47]==0,"navigation heading and ACK");
  tc=16'hffff;for(k=30;k<48;k=k+1)tc=crcbyte(tc,received[k]);
  check({received[49],received[48]}==tc,"navigation telemetry CRC");
  cycles(510000);check(!fresh && !sf,"command and sensor timeouts");
  $display("PASS protocol: UART/CRC/replay/timeout/sensor/telemetry");$finish;
 end
 initial begin #10000000;$fatal(1,"test hung");end
endmodule
