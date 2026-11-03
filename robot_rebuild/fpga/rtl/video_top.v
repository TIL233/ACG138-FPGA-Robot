`timescale 1ns/1ps
module rb_reset_sync(input clk,async_n,output wire rst_n);
 reg [2:0] pipe;
 always @(posedge clk or negedge async_n)
  if(!async_n) pipe<=0;else pipe<={pipe[1:0],1'b1};
 assign rst_n=pipe[2];
endmodule

module rb_video_top #(parameter WIDTH=640,HEIGHT=480)(
 input clk50m,rst_n,
 input cam1_pclk,cam1_href,cam1_vsync,input [7:0] cam1_data,
 output cam1_xclk,cam1_pwdn,cam1_rst_n,cam1_scl,inout cam1_sda,
 output rgmii_tx_clk,rgmii_tx_ctl,output [3:0] rgmii_txd,
 output initialized,output reg [239:0] camera_diag,output reg camera_diag_toggle
);
 wire c50,c125,c24,locked,reset50,reset125,resetp;
 Gowin_PLL pll(.clkin(clk50m),.clkout0(c50),.clkout1(c125),.clkout2(c24),
  .lock(locked),.mdclk(clk50m),.reset(~rst_n));
 rb_reset_sync r50(c50,rst_n && locked,reset50);
 rb_reset_sync r125(c125,rst_n && locked,reset125);
 rb_reset_sync rp(cam1_pclk,rst_n && locked,resetp);
 assign cam1_xclk=c24;
 wire probe_done,probe_high_ack,probe_low_ack,probe_timeout,probe_scl,init_scl;
 wire [2:0] probe_phase;wire [15:0] probe_id;
 rb_camera_probe probe(c50,reset50,probe_done,probe_phase,probe_id,
  probe_high_ack,probe_low_ack,probe_timeout,probe_scl,cam1_sda);
 assign cam1_scl=probe_done?init_scl:probe_scl;
 wire [7:0] init_index;wire init_transaction,init_nack;
 rb_dvp_init #(
  .IMAGE_WIDTH(WIDTH),.IMAGE_HEIGHT(HEIGHT),.IMAGE_FLIP_EN(1),.IMAGE_MIRROR_EN(0)) init(
  .Clk(c50),.Rst_n(reset50 && probe_done),.Init_Done(initialized),
  .camera_pwdn(cam1_pwdn),.i2c_sclk(init_scl),.i2c_sdat(cam1_sda),
  .diag_index(init_index),.diag_done(init_transaction),.diag_nack(init_nack));
 assign cam1_rst_n=reset50;
 reg [15:0] init_nacks;reg [7:0] last_nack_index;
 reg [24:0] idle_timer;reg [22:0] diag_timer;
 wire [159:0] camera_flow;wire [15:0] dropped_frames;
 wire [7:0] diagnostic_phase=!probe_done?{5'd0,probe_phase}:(initialized?8'd7:8'd6);
 wire [7:0] diagnostic_flags={probe_timeout || idle_timer>=25000000,cam1_scl,cam1_sda,
  initialized,probe_id==16'h5640 && probe_high_ack && probe_low_ack,
  probe_low_ack,probe_high_ack,probe_done};
 always @(posedge c50 or negedge reset50) begin
  if(!reset50)begin init_nacks<=0;last_nack_index<=8'hff;idle_timer<=0;
   diag_timer<=0;camera_diag<=0;camera_diag_toggle<=0;end
  else begin
   if(!probe_done || initialized || init_transaction)idle_timer<=0;
   else if(idle_timer<25000000)idle_timer<=idle_timer+1'b1;
   if(probe_done && !initialized && init_transaction && init_nack)begin
    if(init_nacks!=16'hffff)init_nacks<=init_nacks+1'b1;
    last_nack_index<=init_index;
   end
   if(diag_timer>=4999999)begin
    diag_timer<=0;camera_diag_toggle<=!camera_diag_toggle;
    // Version/sequence are supplied by the UART transmitter.
    camera_diag<={camera_flow,last_nack_index,diagnostic_flags,init_nacks,
                  probe_id[7:0],probe_id[15:8],init_index,diagnostic_phase,16'd0};
   end else diag_timer<=diag_timer+1'b1;
  end
 end
 wire start,done,req;wire [7:0] payload,gmii_data;wire gmii_clk,gmii_en;
 rb_video_rows #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) rows(
  .pclk(cam1_pclk),.prst_n(resetp),.vsync(cam1_vsync),.href(cam1_href),.data(cam1_data),
  .txclk(c125),.trst_n(reset125),.req(req),.done(done),.start(start),.payload(payload),.dropped_frames(dropped_frames));
 rb_camera_flow flow_monitor(.clk(c50),.rst_n(reset50),.pclk(cam1_pclk),.prst_n(resetp),
  .vsync(cam1_vsync),.href(cam1_href),.txclk(c125),.trst_n(reset125),.start(start),.done(done),
  .dropped(dropped_frames),.flow(camera_flow));
 eth_udp_tx_gmii udp(.clk125m(c125),.reset_p(~reset125),.tx_en_pulse(start),.tx_done(done),
  .dst_mac(48'hffffffffffff),.src_mac(48'h020000000002),
  .dst_ip(32'hc0a80003),.src_ip(32'hc0a80002),.dst_port(16'd6102),.src_port(16'd5000),
  .data_length(16+WIDTH*2),.payload_req_o(req),.payload_dat_i(payload),
  .gmii_tx_clk(gmii_clk),.gmii_txen(gmii_en),.gmii_txd(gmii_data));
 gmii_to_rgmii rgmii(.reset_n(reset125),.gmii_tx_clk(gmii_clk),.gmii_txd(gmii_data),
  .gmii_txen(gmii_en),.gmii_txer(1'b0),.rgmii_tx_clk(rgmii_tx_clk),
  .rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_tx_ctl));
endmodule
