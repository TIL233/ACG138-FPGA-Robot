`timescale 1ns/1ps
// Outer heading loop. Wheel PI/PWM remains the inner loop. Explicit curves and
// pivots pass through; stop, command change and feedback loss clear the anchor.
module rb_heading_control #(
 parameter YAW_SIGN=1, DEADBAND_CD=100, CD_PER_TRIM=200, MAX_TRIM=6
)(
 input clk,rst_n,brake,healthy,feedback_update,
 input signed [15:0] heading,
 input signed [15:0] target_l,target_r,
 output reg signed [15:0] corrected_l,corrected_r,
 output reg holding,output reg signed [15:0] goal,error,trim
);
 reg signed [15:0] previous_target;
 wire straight=target_l!=0 && target_l==target_r;
 wire reversed=holding && target_l[15]!=previous_target[15];
 wire cancel=brake || !healthy || !straight || reversed;
 function signed [16:0] wrap;
  input signed [17:0] a;
  begin if(a>18000)wrap=a-36000;else if(a< -18000)wrap=a+36000;else wrap=a;end
 endfunction
 wire signed [16:0] native_error=wrap($signed({{2{goal[15]}},goal})-
                                            $signed({{2{heading[15]}},heading}));
 wire signed [16:0] signed_error=(YAW_SIGN<0)?-native_error:native_error;
 wire signed [16:0] wanted=(signed_error>DEADBAND_CD || signed_error< -DEADBAND_CD)?
                                   signed_error/CD_PER_TRIM:17'sd0;
 wire signed [16:0] magnitude=target_l[15]?-$signed({target_l[15],target_l}):$signed({1'b0,target_l});
 wire signed [16:0] limit=((magnitude>>>2)<MAX_TRIM)?(magnitude>>>2):MAX_TRIM;
 wire signed [15:0] limited=wanted>limit?limit:wanted< -limit?-limit:wanted;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin holding<=0;goal<=0;error<=0;trim<=0;previous_target<=0;end
  else if(cancel)begin holding<=0;error<=0;trim<=0;previous_target<=target_l;end
  else if(feedback_update)begin
   previous_target<=target_l;
   if(!holding)begin holding<=1;goal<=heading;error<=0;trim<=0;end
   else begin error<=signed_error[15:0];trim<=limited;end
  end
 end
 // The combinational bypass is essential: STOP/curves must not wait 20ms.
 // Clamp again for a speed reduction between feedback samples.
 wire signed [15:0] applied=trim>limit?limit:trim< -limit?-limit:trim;
 always @*begin
  corrected_l=target_l;corrected_r=target_r;
  if(!cancel && holding)begin corrected_l=target_l-applied;corrected_r=target_r+applied;end
 end
endmodule
