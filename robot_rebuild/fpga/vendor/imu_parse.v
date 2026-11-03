//==============================================================================
// imu_parse.v -- 亚博智能 9 轴 IMU (实测 "7E 23" 帧协议) 解析
//----------------------------------------------------------------------------
// ★★ 重要: 这颗 9 轴模块不是维特 0x55 协议! ★★
//   实测帧格式 (两次独立采样确认, 见 sim/imu_7e_proto_check.py):
//
//     7E 23 <LEN> <TYPE> <LEN-4 字节载荷>              小端
//       LEN  = 帧总长度, 就直接写在 TYPE1 位置:
//              0x17 = 23 字节
//              0x15 = 21 字节   <- 姿态四元数帧
//              0x11 = 17 字节
//       TYPE = 0x16 姿态四元数 (载荷前 16 字节 = 4 个 IEEE754 float32 小端, w,x,y,z)
//              0x04 9 x int16 (陀螺 / 线加速度 / 加速度)
//              0x26 6 x int16 (未识别, 疑为磁力计原始值)
//     三种帧按固定顺序循环: 23 + 21 + 17 = 61 字节/周期, 输出率约 45Hz
//     校验和算法未知 (实测两种常见算法都不符), 但帧头 7E 23 极稳定,
//     靠"长度切帧"足够可靠; 本模块再叠加"四元数模长≈1"自检, 可有效挡掉错帧。
//
// ---- 本版(2026/10/4): 四元数 -> 真正的欧拉角 ----
//   模长自检通过的帧交给 src/imu/quat2euler.v (定点 CORDIC atan2 + 逐位开方),
//   输出的 roll/pitch/yaw 是 0.01° 单位的**姿态角**, 直接进 display_overlay 的 R/P/Y。
//   精度见 sim/quat2euler_check.py: |pitch|<=60° 时 <=0.019°, 万向锁附近 ~0.4°。
//   (旧版把四元数分量 ×100 直接显示, 仅用于打通链路, 已删除)
//
// ---- 波特率未知 -> 自动扫描 ----
//   115200 / 460800 / 230400 / 921600 / 57600 / 38400 / 19200 / 9600, 每档 100ms,
//   收到一个"模长合格"的四元数帧即锁定。
//
// ---- 实现注意 (Gowin 综合器限制) ----
//   不要在"连续赋值"里读寄存器数组或调用含 if/case 的函数, 否则报
//   ERROR (SP00018): Synthesis process cannot run due to error. error bus name set。
//   因此浮点转换 / 模长自检 / 输出缩放全部放进一个多周期状态机(同时缩短组合链)。
//
// 端口与旧版保持完全一致, 顶层无需改动。
//==============================================================================
`timescale 1ns / 1ps

module imu_parse #(
    parameter CLK_FREQ   = 75_000_000,      // ★ 本工程 clk_vga 实际就是 75.000MHz
    parameter SCAN_MS    = 100,             // 每档波特率尝试时长(ms)
    parameter RELOCK_MS  = 500              // 无有效帧后恢复波特率扫描
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         uart_rx,            // 接模块 TX

    output reg  [15:0]  yaw,                // 姿态角 yaw   (0.01° 单位, 有符号)
    output reg  [15:0]  roll,               // 姿态角 roll  (0.01° 单位, 有符号)
    output reg  [15:0]  pitch,              // 姿态角 pitch (0.01° 单位, 有符号)
    output reg          data_valid,         // 解析出一个合格四元数帧时给 1 拍
    output reg          frame_error,        // 模长自检失败
    output reg  [31:0]  frame_ok_cnt,       // 累计合格帧数

    // ---- 调试输出 ----
    output reg  [7:0]   debug_last_byte,    // 最近收到的字节
    output reg  [31:0]  o_rx_byte_cnt,      // 累计收到的字节数

    // ---- 任务2 用: 0x04 帧(9 x int16)里预判的"陀螺 Z"(偏航角速度) ----
    //   帧结构: 7E 23 17 04 | A0(2) A1(2) A2(2) ... A8(2) | 尾(1)
    //   帧内字节偏移: [4][5]=A0  [6][7]=A1  [8][9]=A2 ...
    //   豆包实测推测 A0~A2 = 陀螺 X/Y/Z(量纲未确认), 这里先输出 A2 供上机转动确认
    output reg  signed [15:0] o_gyro_z,
    output reg                o_gyro_z_valid
);

    //==================================================================
    // 1. 波特率分频 (用组合 case 实现, 避免"连续赋值里调用函数")
    //==================================================================
    reg  [2:0]  baud_idx;
    reg         baud_lock;
    reg  [15:0] baud_div;
    localparam integer RELOCK_MAX = (CLK_FREQ / 1000) * RELOCK_MS;
    reg [31:0] good_age;
    wire relock = baud_lock && (good_age >= RELOCK_MAX-1);
    wire baud_change;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) good_age <= 0;
        else if(data_valid || !baud_lock) good_age <= 0;
        else if(good_age < RELOCK_MAX) good_age <= good_age + 1'b1;
    end

    always @(*) begin
        case (baud_idx)
        3'd0:    baud_div = CLK_FREQ / 115200;
        3'd1:    baud_div = CLK_FREQ / 460800;
        3'd2:    baud_div = CLK_FREQ / 230400;
        3'd3:    baud_div = CLK_FREQ / 921600;
        3'd4:    baud_div = CLK_FREQ / 57600;
        3'd5:    baud_div = CLK_FREQ / 38400;
        3'd6:    baud_div = CLK_FREQ / 19200;
        default: baud_div = CLK_FREQ / 9600;
        endcase
    end

    //==================================================================
    // 2. UART 接收 (8N1, LSB first, 采样点落在每一位中间)
    //==================================================================
    reg [2:0]  rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rx_sync <= 3'b111;
        else        rx_sync <= {rx_sync[1:0], uart_rx};
    end

    wire rx_negedge = (rx_sync[2:1] == 2'b10);
    wire rx_level   = rx_sync[2];

    reg [15:0] baud_cnt;
    reg [3:0]  bit_idx;
    reg        first_bit, receiving;
    reg [7:0]  rx_shift, rx_byte;
    reg        rx_byte_valid;

    // 起始位只等半个位周期(采样点落到起始位中点), 之后每整位周期采一位
    wire [15:0] sample_thr = first_bit ? {1'b0, baud_div[15:1]} : (baud_div - 16'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt <= 16'd0; bit_idx <= 4'd0; first_bit <= 1'b1; receiving <= 1'b0;
            rx_shift <= 8'd0;  rx_byte <= 8'd0;  rx_byte_valid <= 1'b0;
        end else begin
            rx_byte_valid <= 1'b0;
            if (relock || baud_change) begin
                baud_cnt <= 0;
                bit_idx <= 0;
                first_bit <= 1'b1;
                receiving <= 1'b0;
            end else if (!receiving) begin
                baud_cnt  <= 16'd0;
                bit_idx   <= 4'd0;
                first_bit <= 1'b1;
                if (rx_negedge) receiving <= 1'b1;
            end else begin
                baud_cnt <= baud_cnt + 1'b1;
                if (baud_cnt == sample_thr) begin
                    baud_cnt  <= 16'd0;
                    first_bit <= 1'b0;
                    case (bit_idx)
                    4'd0: begin                          // 起始位中点必须为低
                        if (rx_level != 1'b0) receiving <= 1'b0;
                        else                  bit_idx   <= 4'd1;
                    end
                    4'd1, 4'd2, 4'd3, 4'd4,
                    4'd5, 4'd6, 4'd7, 4'd8: begin        // 8 个数据位
                        rx_shift <= {rx_level, rx_shift[7:1]};
                        bit_idx  <= bit_idx + 1'b1;
                    end
                    default: begin                       // 停止位中点必须为高
                        if (rx_level == 1'b1) begin
                            rx_byte       <= rx_shift;
                            rx_byte_valid <= 1'b1;
                        end
                        receiving <= 1'b0;
                        bit_idx   <= 4'd0;
                    end
                    endcase
                end
            end
        end
    end

    //==================================================================
    // 3. "7E 23 <LEN> <TYPE> <载荷>" 帧切分
    //==================================================================
    localparam [2:0] S_HDR0 = 3'd0;     // 等 0x7E
    localparam [2:0] S_HDR1 = 3'd1;     // 等 0x23
    localparam [2:0] S_LEN  = 3'd2;     // 读长度
    localparam [2:0] S_TYPE = 3'd3;     // 读类型
    localparam [2:0] S_PAY  = 3'd4;     // 收载荷

    reg [2:0]  st;
    reg [7:0]  f_len, f_type, f_cnt;
    // 四元数帧的 4 个 float: 用小端顺序移入 4 个 32 位移位寄存器
    // (★ 不要用寄存器数组: 16 项数组"边写边一次读出全部元素"会让 Gowin 综合器
    //    报 ERROR (SP00018) error bus name set)
    reg [31:0] qsh0, qsh1, qsh2, qsh3;
    reg        q_done;                  // 收满一个四元数帧时给 1 拍
    reg [7:0]  gz_l, gz_h;              // 0x04 帧里被选中的 int16 低/高字节
    // 三字段轮显: 0x04 帧的 A0/A1/A2 三个 int16 每约 2s 轮换一次, 便于上机甩一次模块
    // 就定位到"哪个字段是偏航角速度"。OSD 用一个同周期的本地计数器显示字段号,
    // 两个计数器同源同模(都从 rst_n 起跑, clk_vga, 周期相同), 因此始终同步。
    localparam integer SEL_TICK = CLK_FREQ * 2;          // 2 秒
    reg [27:0] sel_cnt;
    reg [1:0]  sel2;
    reg [1:0]  sel2_f;                   // ★ 修 #11: 每帧开始时锁存的 sel2, 整帧采集用同一个值

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sel_cnt <= 28'd0;
            sel2    <= 2'd0;

        end else if (sel_cnt >= SEL_TICK - 1) begin
            sel_cnt <= 28'd0;
            sel2    <= (sel2 == 2'd2) ? 2'd0 : (sel2 + 2'd1);
        end else begin
            sel_cnt <= sel_cnt + 28'd1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_HDR0; f_len <= 8'd0; f_type <= 8'd0; f_cnt <= 8'd0;
            q_done <= 1'b0;
            qsh0 <= 32'd0; qsh1 <= 32'd0; qsh2 <= 32'd0; qsh3 <= 32'd0;
            gz_l <= 8'd0; gz_h <= 8'd0;
            sel2_f <= 2'd0;
            o_gyro_z <= 16'sd0; o_gyro_z_valid <= 1'b0;
        end else begin
            q_done <= 1'b0;
            o_gyro_z_valid <= 1'b0;          // ★ 修 #10: 默认清零, 只在收到 0x04 帧时置 1 一拍(单周期脉冲)
            // A partial frame from a lost baud lock must not span recovery.
            if (relock || baud_change) begin
                st <= S_HDR0;
                f_cnt <= 0;
            end else if (rx_byte_valid) begin
                case (st)
                S_HDR0: if (rx_byte == 8'h7E) st <= S_HDR1;

                S_HDR1: if (rx_byte == 8'h23)      st <= S_LEN;
                        else if (rx_byte == 8'h7E) st <= S_HDR1;
                        else                       st <= S_HDR0;

                S_LEN:  begin
                            f_len <= rx_byte;
                            if (rx_byte < 8'd5 || rx_byte > 8'd64) st <= S_HDR0;
                            else                                   st <= S_TYPE;
                        end

                S_TYPE: begin
                            f_type <= rx_byte;
                            f_cnt  <= 8'd0;
                            sel2_f <= sel2;                 // ★ 修 #11: 帧头锁存, 整帧采集用同一个字段选择
                            if (rx_byte == 8'h16) begin
                                qsh0 <= 32'd0; qsh1 <= 32'd0;
                                qsh2 <= 32'd0; qsh3 <= 32'd0;
                            end
                            st <= S_PAY;
                        end

                S_PAY:  begin
                            // ★ 必须限制 f_cnt < 16: 四元数帧载荷是 17 字节(16 字节 float + 1 字节尾),
                            //   若不加这个限制, 第 17 个尾字节会因 f_cnt[3:2]=0 又被移进 qsh0,
                            //   把 w 冲掉 -> 模长自检必然失败 -> 所有帧都被判错帧(屏幕恒 0.0, 方块红)
                            if (f_type == 8'h16 && f_cnt < 8'd16) begin
                                case (f_cnt[3:2])            // 每 4 字节拼成一个 float(小端)
                                2'd0:    qsh0 <= {rx_byte, qsh0[31:8]};
                                2'd1:    qsh1 <= {rx_byte, qsh1[31:8]};
                                2'd2:    qsh2 <= {rx_byte, qsh2[31:8]};
                                default: qsh3 <= {rx_byte, qsh3[31:8]};
                                endcase
                            end
                            f_cnt <= f_cnt + 1'b1;
                            // ---- 0x04 帧(9 x int16): 按 sel2 轮换捕获 A0/A1/A2 ----
                            //   A0 = 载荷偏移 0-1, A1 = 2-3, A2 = 4-5 (对应帧字节 4-9)
                            if (f_type == 8'h04) begin
                                if (f_cnt == {sel2_f, 1'b0})         gz_l <= rx_byte;
                                if (f_cnt == {sel2_f, 1'b0} + 8'd1)  gz_h <= rx_byte;
                            end
                            if (f_cnt + 8'd1 >= f_len - 8'd4) begin
                                st <= S_HDR0;
                                if (f_type == 8'h16 && f_len == 8'd21) q_done <= 1'b1;
                                if (f_type == 8'h04) begin
                                    o_gyro_z       <= {gz_h, gz_l};   // int16 小端, 有符号
                                    o_gyro_z_valid <= 1'b1;
                                end
                            end
                        end

                default: st <= S_HDR0;
                endcase
            end
        end
    end

    //==================================================================
    // 4. 多周期状态机: 四元数帧 -> float32 转 Q15 -> 模长自检 -> 欧拉角
    //    每一拍只做一件事, 组合链短; 45Hz 帧率下时间绰绰有余
    //    ★ 2026/10/4: 模长合格后交给 quat2euler.v 算真正的姿态角
    //      (roll/pitch/yaw 输出 0.01° 单位, 与 display_overlay 的 R/P/Y 一致)
    //==================================================================
    localparam [3:0] C_IDLE  = 4'd0;
    localparam [3:0] C_CV0   = 4'd1;
    localparam [3:0] C_CV1   = 4'd2;
    localparam [3:0] C_CV2   = 4'd3;
    localparam [3:0] C_CV3   = 4'd4;
    localparam [3:0] C_ACC0  = 4'd5;
    localparam [3:0] C_ACC1  = 4'd6;
    localparam [3:0] C_CHK   = 4'd7;
    localparam [3:0] C_EULER = 4'd8;

    reg [3:0]  cst;
    reg [15:0] n0, n1, n2, n3;
    reg signed [31:0] acc;
    reg        eu_start;

    // IEEE754 float32(小端) -> Q1.15
    function [15:0] f2q15;
        input [31:0] f;
        reg [23:0] S;
        reg [7:0]  e;
        reg [8:0]  sh;
        reg [31:0] t1, t2;
        reg [15:0] mag;
        begin
            S = {1'b1, f[22:0]};
            e = f[30:23];
            if (e == 8'd0) begin
                f2q15 = 16'd0;                       // 0 / 非规格化
            end else if (e >= 8'd136) begin
                mag   = 16'h7FFF;                    // |value| >= 2 -> 饱和
                f2q15 = f[31] ? (~mag + 16'd1) : mag;
            end else begin
                sh = 9'd135 - {1'b0, e};
                if (sh >= 9'd32) begin
                    mag = 16'd0;                     // 太小的值在 Q15 下就是 0
                end else begin
                    // ★ 不用变量位移(变量位移会被 Gowin 综合器报
                    //   ERROR (SP00018) error bus name set), 改为两级常数移位:
                    //   先按 8 位一组移, 再按 1 位一组移
                    case (sh[4:3])
                    2'd0:    t1 = {8'd0, S};
                    2'd1:    t1 = {8'd0, S} >> 8;
                    2'd2:    t1 = {8'd0, S} >> 16;
                    default: t1 = {8'd0, S} >> 24;
                    endcase
                    case (sh[2:0])
                    3'd0:    t2 = t1;
                    3'd1:    t2 = t1 >> 1;
                    3'd2:    t2 = t1 >> 2;
                    3'd3:    t2 = t1 >> 3;
                    3'd4:    t2 = t1 >> 4;
                    3'd5:    t2 = t1 >> 5;
                    3'd6:    t2 = t1 >> 6;
                    default: t2 = t1 >> 7;
                    endcase
                    mag = (t2 > 32'd32767) ? 16'h7FFF : t2[15:0];
                end
                f2q15 = f[31] ? (~mag + 16'd1) : mag;
            end
        end
    endfunction

    wire signed [31:0] n0s = $signed(n0);
    wire signed [31:0] n1s = $signed(n1);
    wire signed [31:0] n2s = $signed(n2);
    wire signed [31:0] n3s = $signed(n3);
    // Reject IEEE NaN/Inf before saturation can turn them into unit quaternions.
    wire q_finite = qsh0[30:23]!=8'hff && qsh1[30:23]!=8'hff &&
                    qsh2[30:23]!=8'hff && qsh3[30:23]!=8'hff;

    //------------------------------------------------------------------
    // 四元数 -> 欧拉角 (0.01° 单位), 见 src/imu/quat2euler.v
    //------------------------------------------------------------------
    wire signed [15:0] eu_roll, eu_pitch, eu_yaw;
    wire               eu_done;

    quat2euler u_quat2euler (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (eu_start),
        .q0      (n0),
        .q1      (n1),
        .q2      (n2),
        .q3      (n3),
        .roll_o  (eu_roll),
        .pitch_o (eu_pitch),
        .yaw_o   (eu_yaw),
        .done    (eu_done),
        .busy    ()
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cst <= C_IDLE;
            baud_lock <= 1'b0;
            n0 <= 16'd0; n1 <= 16'd0; n2 <= 16'd0; n3 <= 16'd0;
            acc <= 32'sd0;
            eu_start <= 1'b0;
            roll <= 16'd0; pitch <= 16'd0; yaw <= 16'd0;
            data_valid <= 1'b0; frame_error <= 1'b0; frame_ok_cnt <= 32'd0;
        end else begin
            data_valid  <= 1'b0;
            frame_error <= 1'b0;
            eu_start    <= 1'b0;

            if(relock || baud_change) begin
                baud_lock <= 1'b0;
                cst <= C_IDLE;
            end else case (cst)
            C_IDLE: if (q_done) begin
                if(q_finite) cst <= C_CV0;
                else frame_error <= 1'b1;
            end

            C_CV0:  begin
                        n0  <= f2q15(qsh0);          // w
                        cst <= C_CV1;
                    end
            C_CV1:  begin
                        n1  <= f2q15(qsh1);          // x
                        cst <= C_CV2;
                    end
            C_CV2:  begin
                        n2  <= f2q15(qsh2);          // y
                        cst <= C_CV3;
                    end
            C_CV3:  begin
                        n3  <= f2q15(qsh3);          // z
                        cst <= C_ACC0;
                    end
            C_ACC0: begin
                        acc <= (n0s*n0s) + (n1s*n1s);
                        cst <= C_ACC1;
                    end
            C_ACC1: begin
                        acc <= acc + (n2s*n2s) + (n3s*n3s);     // Q30, 1.0 == 2^30
                        cst <= C_CHK;
                    end
            C_CHK:  begin
                        // 模长自检: 0.90 ~ 1.10 (Q30)
                        if ((acc > 32'sd966_000_000) && (acc < 32'sd1_180_000_000)) begin
                            eu_start  <= 1'b1;       // 交给 quat2euler 算姿态角
                            baud_lock <= 1'b1;       // 模长合格 -> 锁定波特率
                            cst       <= C_EULER;
                        end else begin
                            frame_error  <= 1'b1;
                            cst          <= C_IDLE;
                        end
                    end
            C_EULER: begin
                        if (eu_done) begin
                            roll         <= eu_roll;
                            pitch        <= eu_pitch;
                            yaw          <= eu_yaw;
                            data_valid   <= 1'b1;
                            frame_ok_cnt <= frame_ok_cnt + 1'b1;
                            cst          <= C_IDLE;
                        end
                    end
            default: cst <= C_IDLE;
            endcase
        end
    end

    //==================================================================
    // 5. 波特率扫描
    //==================================================================
    localparam integer SCAN_W = 24;
    localparam [SCAN_W-1:0] SCAN_MAX = (CLK_FREQ / 1000) * SCAN_MS;
    reg  [SCAN_W-1:0] scan_timer;
    wire scan_timeout = (scan_timer >= SCAN_MAX);
    assign baud_change = !baud_lock && scan_timeout;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_idx   <= 3'd0;
            scan_timer <= {SCAN_W{1'b0}};

        end else if (data_valid || relock) begin
            scan_timer <= {SCAN_W{1'b0}};               // 有合格帧 -> 计时器清零
        end else if (!baud_lock && scan_timeout) begin
            baud_idx   <= baud_idx + 1'b1;              // 本档试完 -> 换下一档
            scan_timer <= {SCAN_W{1'b0}};
        end else if (!scan_timeout) begin
            scan_timer <= scan_timer + 1'b1;
        end
    end

    //==================================================================
    // 6. 调试计数
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debug_last_byte <= 8'd0;
            o_rx_byte_cnt   <= 32'd0;
        end else if (rx_byte_valid) begin
            debug_last_byte <= rx_byte;
            o_rx_byte_cnt   <= o_rx_byte_cnt + 1'b1;
        end
    end

endmodule

//==============================================================================
// quat2euler -- 四元数(Q1.15) -> 欧拉角(0.01度) 子模块（与 imu_parse 同文件）
//   ★ 并入同文件的原因: GUI 工程的文件清单由 IDE 维护, 新增独立 .v 文件在 IDE 里
//     会报 "Instantiating unknown module"(EX3937) —— 单文件对 GUI 与命令行都稳。
//   （原 src/imu/quat2euler.v 已删除, 内容原样搬到这里）
//==============================================================================
module quat2euler (
    input  wire                clk,
    input  wire                rst_n,
    input  wire                start,          // 1 拍脉冲: q0..q3 有效
    input  wire signed [15:0]  q0,             // w  Q1.15
    input  wire signed [15:0]  q1,             // x
    input  wire signed [15:0]  q2,             // y
    input  wire signed [15:0]  q3,             // z

    output reg  signed [15:0]  roll_o,         // 0.01° 单位(与 display_overlay 的 R/P/Y 一致)
    output reg  signed [15:0]  pitch_o,
    output reg  signed [15:0]  yaw_o,
    output reg                 done,           // 结果有效, 1 拍
    output reg                 busy
);

    //------------------------------------------------------------------
    // 状态
    //------------------------------------------------------------------
    localparam [3:0] S_IDLE  = 4'd0,
                     S_MLOAD = 4'd1,        // 装第 1 个乘积的乘数
                     S_MUL   = 4'd2,        // 逐个取出乘积 + 装下一个乘数
                     S_ARG   = 4'd3,        // 5 个 Q1.15 参数
                     S_CPRE  = 4'd4,        // CORDIC 预处理(选参数 + 必要时反相)
                     S_CIT   = 4'd5,        // CORDIC 16 次迭代
                     S_CPOST = 4'd6,        // 反相补偿 + 存结果
                     S_SQ    = 4'd7,        // 1 - v^2
                     S_SQIT  = 4'd8,        // 16 拍整数开方
                     S_OUT   = 4'd9,        // 取绝对值 + 加 5(四舍五入用)
                     S_OUT2  = 4'd10,       // 精确有界 /10 的常数倒数乘法, 走 DSP
                     S_OUT3  = 4'd11;       // 右移 16 + 补符号 + 输出

    reg [3:0] st;
    reg [3:0] midx;
    reg [1:0] tsk;                             // 0=roll 1=pitch 2=yaw
    reg [4:0] iter;

    reg signed [15:0] Q0, Q1r, Q2r, Q3r;
    reg signed [31:0] P01, P23, P11, P22, P33, P02, P31, P03, P12;
    reg signed [15:0] A1, B1, A2, B3, A3;
    reg signed [15:0] RSQ;                      // sqrt(1-v^2), Q1.15
    reg signed [18:0] ZR0, ZR1, ZR2;            // 三个角, 0.001°

    reg signed [18:0] CX, CY, CZ;               // CORDIC 工作寄存器
    reg               CFLIP;

    reg [31:0] SQN;
    reg [17:0] SQREM;
    reg [16:0] SQROOT;

    //------------------------------------------------------------------
    // 乘法器（串行复用）: ma/mb 在第 N 拍装载, 第 N+1 拍取 mprod
    //------------------------------------------------------------------
    reg  signed [15:0] ma, mb;
    wire signed [31:0] mprod = ma * mb;

    //------------------------------------------------------------------
    // 5 个参数（组合, 显式饱和, 不调用含 if/case 的函数）
    //------------------------------------------------------------------
    wire signed [31:0] acc_a = P01 + P23;
    wire signed [31:0] acc_b = P11 + P22;
    wire signed [31:0] acc_c = P02 - P31;
    wire signed [31:0] acc_d = P22 + P33;
    wire signed [31:0] acc_e = P03 + P12;

    wire signed [31:0] arg_a = acc_a >>> 14;
    wire signed [31:0] arg_b = 32'sd32768 - (acc_b >>> 14);
    wire signed [31:0] arg_c = acc_c >>> 14;
    wire signed [31:0] arg_d = 32'sd32768 - (acc_d >>> 14);
    wire signed [31:0] arg_e = acc_e >>> 14;

    wire signed [15:0] A1_w = (arg_a >  32'sd32767) ?  16'sd32767 :
                              (arg_a < -32'sd32767) ? -16'sd32767 : arg_a[15:0];
    wire signed [15:0] B1_w = (arg_b >  32'sd32767) ?  16'sd32767 :
                              (arg_b < -32'sd32767) ? -16'sd32767 : arg_b[15:0];
    wire signed [15:0] A2_w = (arg_c >  32'sd32767) ?  16'sd32767 :
                              (arg_c < -32'sd32767) ? -16'sd32767 : arg_c[15:0];
    wire signed [15:0] B3_w = (arg_d >  32'sd32767) ?  16'sd32767 :
                              (arg_d < -32'sd32767) ? -16'sd32767 : arg_d[15:0];
    wire signed [15:0] A3_w = (arg_e >  32'sd32767) ?  16'sd32767 :
                              (arg_e < -32'sd32767) ? -16'sd32767 : arg_e[15:0];

    //------------------------------------------------------------------
    // CORDIC 一拍: 常数移位用组合 always(变量位移会被 Gowin 报错)
    //------------------------------------------------------------------
    reg signed [18:0] ysh, xsh;
    always @(*) begin
        case (iter[3:0])
        4'd0:    begin ysh = CY >>> 0;  xsh = CX >>> 0;  end
        4'd1:    begin ysh = CY >>> 1;  xsh = CX >>> 1;  end
        4'd2:    begin ysh = CY >>> 2;  xsh = CX >>> 2;  end
        4'd3:    begin ysh = CY >>> 3;  xsh = CX >>> 3;  end
        4'd4:    begin ysh = CY >>> 4;  xsh = CX >>> 4;  end
        4'd5:    begin ysh = CY >>> 5;  xsh = CX >>> 5;  end
        4'd6:    begin ysh = CY >>> 6;  xsh = CX >>> 6;  end
        4'd7:    begin ysh = CY >>> 7;  xsh = CX >>> 7;  end
        4'd8:    begin ysh = CY >>> 8;  xsh = CX >>> 8;  end
        4'd9:    begin ysh = CY >>> 9;  xsh = CX >>> 9;  end
        4'd10:   begin ysh = CY >>> 10; xsh = CX >>> 10; end
        4'd11:   begin ysh = CY >>> 11; xsh = CX >>> 11; end
        4'd12:   begin ysh = CY >>> 12; xsh = CX >>> 12; end
        4'd13:   begin ysh = CY >>> 13; xsh = CX >>> 13; end
        4'd14:   begin ysh = CY >>> 14; xsh = CX >>> 14; end
        default: begin ysh = CY >>> 15; xsh = CX >>> 15; end
        endcase
    end

    // atan(2^-i) 的 0.001° 常数(与 Python 模型逐项一致)
    reg signed [18:0] atan_i;
    always @(*) begin
        case (iter[3:0])
        4'd0:    atan_i = 19'sd45000;   // atan(1)   = 45.0000°
        4'd1:    atan_i = 19'sd26565;   // atan(1/2) = 26.5651°
        4'd2:    atan_i = 19'sd14036;
        4'd3:    atan_i = 19'sd7125;
        4'd4:    atan_i = 19'sd3576;
        4'd5:    atan_i = 19'sd1790;
        4'd6:    atan_i = 19'sd895;
        4'd7:    atan_i = 19'sd448;
        4'd8:    atan_i = 19'sd224;
        4'd9:    atan_i = 19'sd112;
        4'd10:   atan_i = 19'sd56;
        4'd11:   atan_i = 19'sd28;
        4'd12:   atan_i = 19'sd14;
        4'd13:   atan_i = 19'sd7;
        4'd14:   atan_i = 19'sd3;
        default: atan_i = 19'sd2;
        endcase
    end

    wire               y_pos = (CY >= 19'sd0);          // 含 0 必须走这一支
    wire signed [18:0] CX_n  = y_pos ? (CX + ysh) : (CX - ysh);
    wire signed [18:0] CY_n  = y_pos ? (CY - xsh) : (CY + xsh);
    wire signed [18:0] CZ_n  = y_pos ? (CZ + atan_i) : (CZ - atan_i);

    // 反相补偿: θ = θ_flipped − 180°, 再归一到 (−180°, 180°]
    // Select the quadrant before adding: CZ-180000 can be below -262144,
    // overflowing 19 bits before the old normalization comparison.
    wire signed [18:0] cz_fix = (CZ <= 19'sd0) ? (CZ + 19'sd180000) : (CZ - 19'sd180000);
    wire signed [18:0] cz_res = CFLIP ? cz_fix : CZ;

    //------------------------------------------------------------------
    // 整数开方一拍
    //------------------------------------------------------------------
    wire [17:0] rem_n   = (SQREM << 2) | {16'd0, SQN[31:30]};
    wire [16:0] root_sh = {SQROOT[15:0], 1'b0};         // root << 1
    wire [17:0] test_n  = {root_sh, 1'b0} + 18'd1;      // 4*previous_root+1
    wire        sq_ge   = (rem_n >= test_n);
    wire [16:0] root_n  = sq_ge ? (root_sh + 17'd1) : root_sh;
    wire [15:0] root_q  = (root_n > 17'd32767) ? 16'd32767 : root_n[15:0];

    //------------------------------------------------------------------
    // CORDIC 输入源(按 tsk 选) 与 0.01° 换算(显式 wire)
    //------------------------------------------------------------------
    wire signed [15:0] cx_src = (tsk == 2'd1) ? RSQ : ((tsk == 2'd0) ? B1 : B3);
    wire signed [15:0] cy_src = (tsk == 2'd1) ? A2  : ((tsk == 2'd0) ? A1 : A3);
    // Q1.15 -> Q1.16: 显式符号扩展后左移 1 ({符号,符号,值,0} = 19 位)
    wire signed [18:0] cx_ext = {cx_src[15], cx_src[15], cx_src, 1'b0};
    wire signed [18:0] cy_ext = {cy_src[15], cy_src[15], cy_src, 1'b0};

    wire signed [18:0] z0_abs = ZR0[18] ? (~ZR0 + 19'sd1) : ZR0;
    wire signed [18:0] z1_abs = ZR1[18] ? (~ZR1 + 19'sd1) : ZR1;
    wire signed [18:0] z2_abs = ZR2[18] ? (~ZR2 + 19'sd1) : ZR2;
    reg  signed [18:0] z0_r5, z1_r5, z2_r5;                   // |角| + 5 (第 1 拍)
    // For integer 0<=x<=180005, floor(x*104858/2^20)=floor(x/10):
    // reciprocal error is at most 0.069, below the 0.1 fractional grid.
    // The old Q16 reciprocal could turn 179.999 degrees into 180.01.
    reg  signed [35:0] p0_r, p1_r, p2_r;
    wire signed [35:0] p10_0 = z0_r5 * 36'sd104858;
    wire signed [35:0] p10_1 = z1_r5 * 36'sd104858;
    wire signed [35:0] p10_2 = z2_r5 * 36'sd104858;
    wire        [15:0] q10_0 = p0_r[35:20];
    wire        [15:0] q10_1 = p1_r[35:20];
    wire        [15:0] q10_2 = p2_r[35:20];
    wire signed [15:0] roll_w  = ZR0[18] ? -q10_0 : q10_0;
    wire signed [15:0] pitch_w = ZR1[18] ? -q10_1 : q10_1;
    wire signed [15:0] yaw_w   = ZR2[18] ? -q10_2 : q10_2;

    //------------------------------------------------------------------
    // 主状态机
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st   <= S_IDLE;
            midx <= 4'd0;
            tsk <= 2'd0;
            iter <= 5'd0;
            Q0 <= 16'sd0; Q1r <= 16'sd0; Q2r <= 16'sd0; Q3r <= 16'sd0;
            P01 <= 32'sd0; P23 <= 32'sd0; P11 <= 32'sd0; P22 <= 32'sd0; P33 <= 32'sd0;
            P02 <= 32'sd0; P31 <= 32'sd0; P03 <= 32'sd0; P12 <= 32'sd0;
            A1 <= 16'sd0; B1 <= 16'sd0; A2 <= 16'sd0; B3 <= 16'sd0; A3 <= 16'sd0;
            RSQ <= 16'sd0;
            ZR0 <= 19'sd0; ZR1 <= 19'sd0; ZR2 <= 19'sd0;
            CX <= 19'sd0; CY <= 19'sd0; CZ <= 19'sd0; CFLIP <= 1'b0;
            SQN <= 32'd0; SQREM <= 18'd0; SQROOT <= 17'd0;
            z0_r5 <= 19'sd0; z1_r5 <= 19'sd0; z2_r5 <= 19'sd0;
            p0_r <= 36'sd0; p1_r <= 36'sd0; p2_r <= 36'sd0;
            ma <= 16'sd0; mb <= 16'sd0;
            roll_o <= 16'sd0; pitch_o <= 16'sd0; yaw_o <= 16'sd0;
            done <= 1'b0; busy <= 1'b0;
        end else begin
            done <= 1'b0;
            case (st)
            //------------------------------------------------
            S_IDLE: begin
                if (start) begin
                    Q0 <= q0; Q1r <= q1; Q2r <= q2; Q3r <= q3;
                    busy <= 1'b1;
                    st   <= S_MLOAD;
                end else begin
                    busy <= 1'b0;
                end
            end
            //------------------------------------------------
            S_MLOAD: begin                     // 装乘积 0 (w*x) 的乘数
                ma   <= Q0;
                mb   <= Q1r;
                midx <= 4'd0;
                st   <= S_MUL;
            end
            //------------------------------------------------
            // 第 midx 拍: 取出"上一拍装好的乘数"的乘积, 同时装下一个乘数
            //   0:p01=w*x  1:p23=y*z  2:p11=x*x  3:p22=y*y  4:p33=z*z
            //   5:p02=w*y  6:p31=z*x  7:p03=w*z  8:p12=x*y
            S_MUL: begin
                case (midx)
                4'd0: P01 <= mprod;
                4'd1: P23 <= mprod;
                4'd2: P11 <= mprod;
                4'd3: P22 <= mprod;
                4'd4: P33 <= mprod;
                4'd5: P02 <= mprod;
                4'd6: P31 <= mprod;
                4'd7: P03 <= mprod;
                default: P12 <= mprod;
                endcase
                case (midx)                     // 装下一个乘积的乘数
                4'd0: begin ma <= Q2r; mb <= Q3r; end
                4'd1: begin ma <= Q1r; mb <= Q1r; end
                4'd2: begin ma <= Q2r; mb <= Q2r; end
                4'd3: begin ma <= Q3r; mb <= Q3r; end
                4'd4: begin ma <= Q0;  mb <= Q2r; end
                4'd5: begin ma <= Q3r; mb <= Q1r; end
                4'd6: begin ma <= Q0;  mb <= Q3r; end
                4'd7: begin ma <= Q1r; mb <= Q2r; end
                default: begin ma <= 16'sd0; mb <= 16'sd0; end
                endcase
                if (midx == 4'd8) st <= S_ARG;
                else              midx <= midx + 4'd1;
            end
            //------------------------------------------------
            S_ARG: begin
                A1 <= A1_w; B1 <= B1_w; A2 <= A2_w; B3 <= B3_w; A3 <= A3_w;
                tsk <= 2'd0;
                st   <= S_CPRE;
            end
            //------------------------------------------------
            S_CPRE: begin
                // 选本轮 (y,x) 并做 x<0 的反相(等价于角度最后 −180°)
                if (cx_src < 16'sd0) begin
                    CX    <= -cx_ext;
                    CY    <= -cy_ext;
                    CFLIP <= 1'b1;
                end else begin
                    CX    <= cx_ext;
                    CY    <= cy_ext;
                    CFLIP <= 1'b0;
                end
                CZ   <= 19'sd0;
                iter <= 5'd0;
                st   <= S_CIT;
            end
            //------------------------------------------------
            S_CIT: begin
                CX <= CX_n;
                CY <= CY_n;
                CZ <= CZ_n;
                if (iter == 5'd15) st   <= S_CPOST;
                else               iter <= iter + 5'd1;
            end
            //------------------------------------------------
            S_CPOST: begin
                case (tsk)
                2'd0:    ZR0 <= cz_res;         // roll
                2'd1:    ZR1 <= cz_res;         // pitch
                default: ZR2 <= cz_res;         // yaw
                endcase
                tsk <= tsk + 2'd1;
                if (tsk == 2'd2) st <= S_OUT;
                else              st <= S_SQ;   // roll 完成后先算 sqrt 供 pitch 用
            end
            //------------------------------------------------
            S_SQ: begin
                SQN    <= 32'sd1073741824 - (A2 * A2);   // 1 − v², Q2.30
                SQREM  <= 18'd0;
                SQROOT <= 17'd0;
                iter   <= 5'd0;
                st     <= S_SQIT;
            end
            //------------------------------------------------
            S_SQIT: begin
                SQREM  <= sq_ge ? (rem_n - test_n) : rem_n;
                SQROOT <= root_n;
                SQN    <= {SQN[29:0], 2'b00};
                if (iter == 5'd15) begin
                    RSQ <= root_q;              // Q1.15 的 sqrt(1−v²)
                    st  <= S_CPRE;              // 下一轮 CORDIC = pitch
                end else begin
                    iter <= iter + 5'd1;
                end
            end
            //------------------------------------------------
            S_OUT: begin
                // 第 1 拍: 取绝对值 + 加 5（四舍五入的一半）
                z0_r5 <= z0_abs + 19'sd5;
                z1_r5 <= z1_abs + 19'sd5;
                z2_r5 <= z2_abs + 19'sd5;
                st    <= S_OUT2;
            end
            //------------------------------------------------
            S_OUT2: begin
                // 第 2 拍: 精确有界倒数乘法，保持原三拍输出时序。
                p0_r <= p10_0;
                p1_r <= p10_1;
                p2_r <= p10_2;
                st   <= S_OUT3;
            end
            //------------------------------------------------
            S_OUT3: begin
                // 第 3 拍: 右移 20 + 补符号 + 输出（这一拍只有移位/取反/二选一）
                roll_o  <= roll_w;
                pitch_o <= pitch_w;
                yaw_o   <= yaw_w;
                done    <= 1'b1;
                busy    <= 1'b0;
                st      <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end

endmodule

