module qr_decode(
input clk,                           //clock input
input srstn,                         //synchronous reset (active low)
input qr_decode_start,               //start decoding for one QR code
                                     //1: start (one-cycle pulse)
input sram_rdata,                    //read data from SRAM
output reg [11:0] sram_raddr,        //read address to SRAM

output reg decode_valid,                 //decoded code is valid
output reg [7:0] decode_jis8_code,       //decoded JIS8 code
output reg qr_decode_finish              //1: decoding one QR code is finished
);
parameter [399:0] mask_pattern0 = 400'b1010101010101010010101010101010110101010101010100101010101010101101010101010101001010101010101011010101010101010010101010101010110101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010100101010110101010010101011010101001010101101010100101010110101010;
parameter [399:0] mask_pattern1 = 400'b1111111111111111000000000000000011111111111111110000000000000000111111111111111100000000000000001111111111111111000000000000000011111111111111111111111110000000000000000000000000111111111111111111111111100000000000000000000000001111111111111111111111111000000000000000000000000011111111111111111111111110000000000000000000000000111111110000000011111111000000001111111100000000111111110000000011111111;
parameter [399:0] mask_pattern2 = 400'b1001001001001001100100100100100110010010010010011001001001001001100100100100100110010010010010011001001001001001100100100100100110010010010010010010010011001001001001001001001001100100100100100100100100110010010010010010010010011001001001001001001001001100100100100100100100100110010010010010010010010011001001001001001001001001010010010100100101001001010010010100100101001001010010010100100101001001;
parameter [399:0] mask_pattern3 = 400'b1001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001001100100100010010001001001100100100010010001001001100100100010010001001001;
parameter [399:0] mask_pattern4 = 400'b1000111000111000011100011100011101110001110001111000111000111000100011100011100001110001110001110111000111000111100011100011100010001110001110001110001110111000111000111000111000011100011100011100011100010001110001110001110001111000111000111000111000111011100011100011100011100001110001110001110001110001000111000111000111000111001110001100011111000111001110000011100011000111110001110011100000111000;
parameter [399:0] mask_pattern5 = 400'b1111111111111111100000100000100010010010010010011010101010101010100100100100100110000010000010001111111111111111100000100000100010010010010010010010010011010101010101010101010101100100100100100100100100110000010000010000010000011111111111111111111111111100000100000100000100000110010010010010010010010011010101010101010101010101010010010000100011111111000010000100100110101010010010010000100011111111;
parameter [399:0] mask_pattern6 = 400'b1111111111111111111000111000111011011011011011011010101010101010101101101101101110001110001110001111111111111111111000111000111011011011011011011011011011010101010101010101010101101101101101101101101101110001110001110001110001111111111111111111111111111111000111000111000111000111011011011011011011011011010101010101010101010101110110110011100011111111100011100110110110101010110110110011100011111111;
parameter [399:0] mask_pattern7 = 400'b1010101010101010000111000111000110001110001110000101010101010101111000111000111001110001110001111010101010101010000111000111000110001110001110001110001110101010101010101010101010111000111000111000111000101110001110001110001110001010101010101010101010101000111000111000111000111010001110001110001110001110101010101010101010101010100011101100011110101010011100010011100001010101100011101100011110101010;



localparam [3:0] IDLE = 0;
localparam [3:0] DETECT_POSITION_AND_ROTATION = 1;
localparam [3:0] FIND_MASK_PATTERN = 2;
localparam [3:0] DE_MASKING_1 = 3;
localparam [3:0] DE_MASKING_2 = 4;
localparam [3:0] DE_MASKING_3 = 5;
localparam [3:0] CALCULATE_SYNDROME_1 = 6;
localparam [3:0] CALCULATE_SYNDROME_2 = 7;
localparam [3:0] SOLVE_EQUATION = 8;
localparam [3:0] FIND_ERROR_POSITION = 9;
localparam [3:0] SOLVE_EQUATION_2 = 10;
localparam [3:0] FIX_ERROR = 11;
localparam [3:0] OUTPUT = 12;
localparam [3:0] FINISH = 13;

wire enable;
wire pos_rot_finish;
wire [2:0] mode;
wire [11:0] position;

assign enable = 1;

wire [11:0] pos_rot_sram_raddr;


detect_rotation u1(
.clk(clk),
.srstn(srstn),
.enable(enable),
.sram_rdata(sram_rdata),
.sram_raddr(pos_rot_sram_raddr),
.mode(mode),
.position(position),
.finish(pos_rot_finish)
);




reg [3:0] state;
reg [3:0] state_n;

reg [2:0] mask;
reg [2:0] mask_n;
wire [2:0] real_mask;

assign real_mask = mask ^ 3'b101;

reg [11:0] mask_raddr;

always@(*)begin
    case(state)
        DETECT_POSITION_AND_ROTATION:begin
            sram_raddr = pos_rot_sram_raddr;
        end
        FIND_MASK_PATTERN:begin
            sram_raddr = mask_raddr;
        end
        default:begin
            sram_raddr = mask_raddr;
        end
    endcase
end

reg decode_valid_n;
reg [7:0] decode_jis8_code_n;
reg qr_decode_finish_n;


reg [1:0] mask_cnt;
reg [1:0] mask_cnt_n;

reg [399:0] code;
reg code_n;

reg [4:0] x_cnt;
reg [4:0] x_cnt_n;
reg [4:0] y_cnt;
reg [4:0] y_cnt_n;

wire [4:0] real_x_cnt;
wire [4:0] real_y_cnt;

reg [8:0] read_cnt;
reg [8:0] read_cnt_n;
reg [5:0] out_cnt;
reg [5:0] out_cnt_n;

wire[7:0] c [0:43];
reg [7:0] c_correct[0:27];
reg [7:0] offset[0:27];
reg [7:0] offset_n[0:27];

wire [7:0] length;

reg [5:0] gf_cnt;
reg [5:0] gf_cnt_n;

reg[7:0] in1_sys,in2_sys,in3_sys,in4_sys,in5_sys;
reg[7:0] in1_sys_n,in2_sys_n,in3_sys_n,in4_sys_n,in5_sys_n;
reg start_sys;
reg start_sys_n;
wire finish_sys;
wire [7:0] sigma_1, sigma_2, sigma_3, sigma_4;




assign length = {c_correct[0][3:0],c_correct[1][7:4]};

assign real_x_cnt = (mode == 1) ? x_cnt : (mode == 2)? y_cnt : (mode == 3)? 24-x_cnt : 24-y_cnt; 
assign real_y_cnt = (mode == 1) ? y_cnt : (mode == 2)? 24-x_cnt : (mode == 3)? 24-y_cnt : x_cnt; 



assign c[ 0] = {code[  0], code[  1], code[ 16], code[ 17], code[ 32], code[ 33], code[ 48], code[ 49]};
assign c[ 1] = {code[ 64], code[ 65], code[ 80], code[ 81], code[ 96], code[ 97], code[112], code[113]};
assign c[ 2] = {code[128], code[129], code[153], code[154], code[178], code[179], code[203], code[204]};
assign c[ 3] = {code[228], code[229], code[253], code[254], code[278], code[279], code[303], code[304]};
assign c[ 4] = {code[305], code[306], code[280], code[281], code[255], code[256], code[230], code[231]};
assign c[ 5] = {code[205], code[206], code[180], code[181], code[155], code[156], code[130], code[131]};
assign c[ 6] = {code[114], code[115], code[ 98], code[ 99], code[ 82], code[ 83], code[ 66], code[ 67]};
assign c[ 7] = {code[ 50], code[ 51], code[ 34], code[ 35], code[ 18], code[ 19], code[  2], code[  3]};
assign c[ 8] = {code[  4], code[  5], code[ 20], code[ 21], code[ 36], code[ 37], code[ 52], code[ 53]};
assign c[ 9] = {code[157], code[158], code[182], code[183], code[207], code[208], code[232], code[233]};
assign c[10] = {code[257], code[258], code[282], code[283], code[307], code[308], code[309], code[310]};
assign c[11] = {code[284], code[285], code[259], code[260], code[234], code[235], code[209], code[210]};
assign c[12] = {code[184], code[185], code[159], code[160], code[ 54], code[ 55], code[ 38], code[ 39]};
assign c[13] = {code[ 22], code[ 23], code[  6], code[  7], code[  8], code[  9], code[ 24], code[ 25]};
assign c[14] = {code[ 40], code[ 41], code[ 56], code[ 57], code[ 73], code[ 89], code[105], code[121]};
assign c[15] = {code[137], code[161], code[162], code[186], code[187], code[211], code[212], code[236]};
assign c[16] = {code[237], code[261], code[262], code[286], code[287], code[311], code[312], code[328]};
assign c[17] = {code[329], code[336], code[337], code[352], code[353], code[360], code[361], code[368]};
assign c[18] = {code[369], code[376], code[377], code[384], code[385], code[392], code[393], code[394]};
assign c[19] = {code[395], code[386], code[387], code[378], code[379], code[370], code[371], code[362]};
assign c[20] = {code[363], code[354], code[355], code[338], code[339], code[330], code[331], code[313]};
assign c[21] = {code[314], code[288], code[289], code[263], code[264], code[238], code[239], code[213]};
assign c[22] = {code[214], code[188], code[189], code[163], code[164], code[138], code[139], code[122]};
assign c[23] = {code[123], code[106], code[107], code[ 90], code[ 91], code[ 74], code[ 75], code[ 58]};
assign c[24] = {code[ 59], code[ 42], code[ 43], code[ 26], code[ 27], code[ 10], code[ 11], code[ 12]};
assign c[25] = {code[ 13], code[ 28], code[ 29], code[ 44], code[ 45], code[ 60], code[ 61], code[ 76]};
assign c[26] = {code[ 77], code[ 92], code[ 93], code[108], code[109], code[124], code[125], code[140]};
assign c[27] = {code[141], code[165], code[166], code[190], code[191], code[215], code[216], code[240]};
assign c[28] = {code[241], code[265], code[266], code[290], code[291], code[315], code[316], code[332]};
assign c[29] = {code[333], code[340], code[341], code[356], code[357], code[364], code[365], code[372]};
assign c[30] = {code[373], code[380], code[381], code[388], code[389], code[396], code[397], code[398]};
assign c[31] = {code[399], code[390], code[391], code[382], code[383], code[374], code[375], code[366]};
assign c[32] = {code[367], code[358], code[359], code[342], code[343], code[334], code[335], code[317]};
assign c[33] = {code[318], code[292], code[293], code[267], code[268], code[242], code[243], code[217]};
assign c[34] = {code[218], code[192], code[193], code[167], code[168], code[142], code[143], code[126]};
assign c[35] = {code[127], code[110], code[111], code[ 94], code[ 95], code[ 78], code[ 79], code[ 62]};
assign c[36] = {code[ 63], code[ 46], code[ 47], code[ 30], code[ 31], code[ 14], code[ 15], code[144]};
assign c[37] = {code[145], code[169], code[170], code[194], code[195], code[219], code[220], code[244]};
assign c[38] = {code[245], code[269], code[270], code[294], code[295], code[319], code[320], code[322]};
assign c[39] = {code[323], code[297], code[298], code[272], code[273], code[247], code[248], code[222]};
assign c[40] = {code[223], code[197], code[198], code[172], code[173], code[147], code[148], code[149]};
assign c[41] = {code[150], code[174], code[175], code[199], code[200], code[224], code[225], code[249]};
assign c[42] = {code[250], code[274], code[275], code[299], code[300], code[324], code[325], code[326]};
assign c[43] = {code[327], code[301], code[302], code[276], code[277], code[251], code[252], code[226]};

wire [7:0] log_out;
wire [7:0] log_in;
log u2(
.in(c[gf_cnt]),
.out(log_out)
);

wire [7:0] antilog_out;
wire [7:0] antilog_in;

antilog u3(
.in(antilog_in),
.out(antilog_out)
);


//wire [7:0] test_0 = c_correct[0];
//wire [7:0] test_1 = c_correct[1];

integer k;
always@(*)begin
    for(k = 0 ; k < 28 ; k = k + 1)begin
        c_correct[k] = c[k] ^ offset[k];
    end
end

reg [7:0] syndrome [0:7];
wire [7:0] tmp_s;

reg [7:0] ss [0:7];
wire [7:0] ss_n [0:1];
add_gf ssu1 (.in1(ss[6]),.in2(8'd6),.out(ss_n[0]));
add_gf ssu2 (.in1(ss[7]),.in2(8'd7),.out(ss_n[1]));

always@(posedge clk)begin
    if(~srstn)begin
        ss[0] <= 0;
        ss[1] <= 0;
        ss[2] <= 0;
        ss[3] <= 0;
        ss[4] <= 0;
        ss[5] <= 0;
        ss[6] <= 0;
        ss[7] <= 0;
    end
    else begin
        if(state == CALCULATE_SYNDROME_1)begin
            ss[0] <= ss[0];
            ss[1] <= ss[1] + 1;
            ss[2] <= ss[2] + 2;
            ss[3] <= ss[3] + 3;
            ss[4] <= ss[4] + 4;
            ss[5] <= ss[5] + 5;
            ss[6] <= ss_n[0];
            ss[7] <= ss_n[1];
        end
        else begin
            ss[0] <= ss[0];
            ss[1] <= ss[1];
            ss[2] <= ss[2];
            ss[3] <= ss[3];
            ss[4] <= ss[4];
            ss[5] <= ss[5];
            ss[6] <= ss[6];
            ss[7] <= ss[7];
        end
    end
end

reg [2:0]  gf_cnt_y, gf_cnt_y_n;

add_gf add_u1(.in1(log_out),.in2(ss[gf_cnt_y]),.out(antilog_in));

always@(posedge clk)begin
    if(~srstn)begin
        syndrome[0] <= 0;
        syndrome[1] <= 0;
        syndrome[2] <= 0;
        syndrome[3] <= 0;
        syndrome[4] <= 0;
        syndrome[5] <= 0;
        syndrome[6] <= 0;
        syndrome[7] <= 0;
    end
    else begin
        if(state == CALCULATE_SYNDROME_2)begin
            syndrome[gf_cnt_y] <= syndrome[gf_cnt_y] ^ antilog_out;
        end
    end
end

//assign log_in = c[gf_cnt];

wire enable_err_pos;
assign enable_err_pos = finish_sys;
wire [7:0] i1, i2,i3,i4;
wire finish_err_pos;
wire [2:0] error_count;

reg [7:0] i1_tmp, i2_tmp,i3_tmp,i4_tmp;
wire [7:0] i1_anti, i2_anti,i3_anti,i4_anti;
reg [7:0] i2_anti_delay1;
reg [7:0] i3_anti_delay1, i3_anti_delay2;
reg [7:0] i4_anti_delay1, i4_anti_delay2, i4_anti_delay3;

antilog aa1 (.in(i1_tmp),.out(i1_anti));
antilog aa2 (.in(i2_tmp),.out(i2_anti));
antilog aa3 (.in(i3_tmp),.out(i3_anti));
antilog aa4 (.in(i4_tmp),.out(i4_anti));

error_position epu1(
.sigma_1(sigma_1),
.sigma_2(sigma_2),
.sigma_3(sigma_3),
.sigma_4(sigma_4),
.clk(clk),
.srstn(srstn),
.enable(enable_err_pos),

.error_count(error_count),
.i1(i1),
.i2(i2),
.i3(i3),
.i4(i4),
.finish(finish_err_pos)
);

always@(posedge clk)begin
    i2_anti_delay1 <= i2_anti;
    i3_anti_delay1 <= i3_anti;
    i3_anti_delay2 <= i3_anti_delay1;
    i4_anti_delay1 <= i4_anti;
    i4_anti_delay2 <= i4_anti_delay1;
    i4_anti_delay3 <= i4_anti_delay2;
end

always@(posedge clk)begin
    if(state == SOLVE_EQUATION_2)begin
        i1_tmp <= i1_tmp + i1;
        i2_tmp <= i2_tmp + i2;
        i3_tmp <= i3_tmp + i3;
        i4_tmp <= i4_tmp + i4;
    end
    else begin
        i1_tmp <= i1;
        i2_tmp <= i2;
        i3_tmp <= i3;
        i4_tmp <= i4;
    end
end


reg [1:0] fix_error_cnt;
reg [1:0] fix_error_cnt_n;
reg [7:0] anti_in_fix_error;
wire [7:0] anti_out_fix_error;

antilog aaa (.in(anti_in_fix_error),.out(anti_out_fix_error));

integer n;

always@(posedge clk)begin
    if(~srstn)begin
        for( n = 0 ; n < 28 ; n = n +1)begin
            offset[n] <= 0;
        end
    end
    else begin
        for( n = 0 ; n < 28 ; n = n +1)begin
            offset[n] <= offset_n[n];
        end
    end
end

always@(posedge clk)begin
    fix_error_cnt <= fix_error_cnt_n;
end

integer m;
reg [7:0] sigma_tmp;
always@(*)begin
    if(state == FIX_ERROR)begin
        fix_error_cnt_n = fix_error_cnt + 1;
        for ( m = 0 ; m < 28 ; m = m +1)begin
            offset_n[m] = offset[m];
        end

        case(fix_error_cnt)
            0:begin
                sigma_tmp = (error_count == 3) ? sigma_3 : sigma_4;
                anti_in_fix_error = ({1'b0, sigma_tmp} + {1'b0, i1} >= 255)? sigma_tmp+i1 +1 : sigma_tmp + i1;
                offset_n[43-i1] =  anti_out_fix_error;
            end
            1:begin
                sigma_tmp = (error_count == 3) ? sigma_2 : sigma_3;
                anti_in_fix_error = ({1'b0, sigma_tmp} + {1'b0, i2} >= 255)? sigma_tmp+i2 +1 : sigma_tmp + i2;
                offset_n[43-i2] =  anti_out_fix_error;
            end
            2:begin
                sigma_tmp = (error_count == 3) ? sigma_1 : sigma_2;
                anti_in_fix_error = ({1'b0, sigma_tmp} + {1'b0, i3} >= 255)? sigma_tmp+i3 +1 : sigma_tmp + i3;
                offset_n[43-i3] =  anti_out_fix_error;
            end
            3:begin
                sigma_tmp = sigma_1;
                anti_in_fix_error = ({1'b0, sigma_tmp} + {1'b0, i4} >= 255)? sigma_tmp+i4 +1 : sigma_tmp + i4;
                offset_n[43-i4] =  anti_out_fix_error;
            end
            default:begin
                sigma_tmp = 0;
                anti_in_fix_error = 0;
            end
        endcase
    end
    else begin
        fix_error_cnt_n = 0;
        anti_in_fix_error = 0 ;
        sigma_tmp = 0;
        for ( m = 0 ; m < 28 ; m = m +1)begin
            offset_n[m] = offset[m];
        end

    end
end

//wire [7:0] test_c9  = c[9];
//wire [7:0] test_c10  = c[10];
//wire [7:0] test_c11  = c[11];
//wire [7:0] test_c12  = c[12];





reg [2:0] sys_cnt;
reg [2:0] sys_cnt_n;
wire mode_sys;
assign mode_sys = (state == SOLVE_EQUATION_2)? (error_count == 3)? 1 : 0 :0;

systolic sys1(
.in1(in1_sys),
.in2(in2_sys),
.in3(in3_sys),
.in4(in4_sys),
.in5(in5_sys),
.clk(clk),
.srstn(srstn),
.start(start_sys),
.mode(mode_sys),

.sigma_1(sigma_1),
.sigma_2(sigma_2),
.sigma_3(sigma_3),
.sigma_4(sigma_4),
.finish(finish_sys)

);

always@(posedge clk)begin
    in1_sys <= in1_sys_n;
    in2_sys <= in2_sys_n;
    in3_sys <= in3_sys_n;
    in4_sys <= in4_sys_n;
    in5_sys <= in5_sys_n;
    start_sys <= start_sys_n;
    sys_cnt <= sys_cnt_n;
end
always@(*)begin
    if(state == SOLVE_EQUATION)begin
        sys_cnt_n = sys_cnt + 1;
        start_sys_n = 1;
        case(sys_cnt)
            0:begin
                in1_sys_n = syndrome[0];
                in2_sys_n = 0;
                in3_sys_n = 0;
                in4_sys_n = 0;
                in5_sys_n = 0;
            end
            1:begin
                in1_sys_n = syndrome[1];
                in2_sys_n = syndrome[1];
                in3_sys_n = 0;
                in4_sys_n = 0;
                in5_sys_n = 0;
            end
            2:begin
                in1_sys_n = syndrome[2];
                in2_sys_n = syndrome[2];
                in3_sys_n = syndrome[2];
                in4_sys_n = 0;
                in5_sys_n = 0;
            end
            3:begin
                in1_sys_n = syndrome[3];
                in2_sys_n = syndrome[3];
                in3_sys_n = syndrome[3];
                in4_sys_n = syndrome[3];
                in5_sys_n = 0;
            end
            4:begin
                in1_sys_n = 0;
                in2_sys_n = syndrome[4];
                in3_sys_n = syndrome[4];
                in4_sys_n = syndrome[4];
                in5_sys_n = syndrome[4];
            end
            5:begin
                in1_sys_n = 0;
                in2_sys_n = 0;
                in3_sys_n = syndrome[5];
                in4_sys_n = syndrome[5];
                in5_sys_n = syndrome[5];
            end
            6:begin
                in1_sys_n = 0;
                in2_sys_n = 0;
                in3_sys_n = 0;
                in4_sys_n = syndrome[6];
                in5_sys_n = syndrome[6];
            end
            7:begin
                in1_sys_n = 0;
                in2_sys_n = 0;
                in3_sys_n = 0;
                in4_sys_n = 0;
                in5_sys_n = syndrome[7];
            end
            default : begin
                in1_sys_n = 0;
                in2_sys_n = 0;
                in3_sys_n = 0;
                in4_sys_n = 0;
                in5_sys_n = 0;            
            end
        endcase
    end
    else if (state == SOLVE_EQUATION_2) begin
        sys_cnt_n = sys_cnt + 1;
        start_sys_n = 1;
        in1_sys_n = i1_anti;
        in2_sys_n = i2_anti_delay1;
        in3_sys_n = i3_anti_delay2;
        in4_sys_n = i4_anti_delay3;
    
        case(sys_cnt)
            4:begin
                in5_sys_n = syndrome[0];
            end
            5:begin
                in5_sys_n = syndrome[1];
            end
            6:begin
                in5_sys_n = syndrome[2];
            end
            7:begin
                in5_sys_n = syndrome[3];
            end
            default:begin
                in5_sys_n = 0;
            end
        endcase

    end
    else begin
        sys_cnt_n = 0;
        in1_sys_n = 0;
        in2_sys_n = 0;
        in3_sys_n = 0;
        in4_sys_n = 0;
        in5_sys_n = 0;    
        start_sys_n = 0;
    end
end

//wire [7:0] s0 = syndrome[0];
//wire [7:0] s1 = syndrome[1];
//wire [7:0] s2 = syndrome[2];
//wire [7:0] s3 = syndrome[3];
//wire [7:0] s4 = syndrome[4];
//wire [7:0] s5 = syndrome[5];
//wire [7:0] s6 = syndrome[6];
//wire [7:0] s7 = syndrome[7];


always@(posedge clk)begin
    if(~srstn)begin
        state <= IDLE;
    end
    else begin
        state <= state_n;
    end
end

always@(posedge clk)begin
    if(~srstn)begin
        mask_cnt <= 0;
        x_cnt <= 0 ;
        y_cnt <= 0;
    end
    else begin
        mask_cnt <= mask_cnt_n;
        x_cnt <= x_cnt_n;
        y_cnt <= y_cnt_n;
    end
end

integer i ;
always@(posedge clk)begin
    if(state == DE_MASKING_1 || state == DE_MASKING_2 || state == DE_MASKING_3)begin
        for(i = 399 ; i>0 ;i = i -1)begin
            code[i] <= code[i-1];
        end 
        code[0] <= code_n;
    end
    else begin
        code <= code;
    end
    if(state == FIND_MASK_PATTERN)begin
        mask[2] <= mask[1];
        mask[1] <= mask[0];
        mask[0] <= mask_n;
    end
    else begin
        mask <= mask;
    end
    read_cnt <= read_cnt_n;
    out_cnt <= out_cnt_n;
    gf_cnt <= gf_cnt_n;
    gf_cnt_y <= gf_cnt_y_n;
end

always@(*)begin
    case(state)
        IDLE:begin
            state_n = (qr_decode_start)? DETECT_POSITION_AND_ROTATION:IDLE;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
            gf_cnt_n = 43;
            gf_cnt_y_n = 0;
        end
        DETECT_POSITION_AND_ROTATION:begin
            state_n = (pos_rot_finish) ? FIND_MASK_PATTERN : DETECT_POSITION_AND_ROTATION;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
            gf_cnt_n = 43;
            gf_cnt_y_n = 0;
        end
        FIND_MASK_PATTERN:begin
            state_n = (mask_cnt == 2)? DE_MASKING_1 : FIND_MASK_PATTERN;
            mask_raddr = (mode == 1 || mode == 4)? position+8*64+2 + mask_cnt : position + 16*64+22 - mask_cnt;
            mask_n = sram_rdata;
            mask_cnt_n = mask_cnt + 1;
            code_n = 0;
            x_cnt_n = 9;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
            gf_cnt_n = 43;
            gf_cnt_y_n = 0;
        end
        DE_MASKING_1:begin    
            state_n = (x_cnt == 16 && y_cnt == 8)? DE_MASKING_2 :DE_MASKING_1;
            mask_raddr = position + 64*real_y_cnt + real_x_cnt;
            mask_n = 0;
            mask_cnt_n = 0;
            case(real_mask)
                0:begin code_n = sram_rdata ^ mask_pattern0[read_cnt]; end
                1:begin code_n = sram_rdata ^ mask_pattern1[read_cnt]; end
                2:begin code_n = sram_rdata ^ mask_pattern2[read_cnt]; end
                3:begin code_n = sram_rdata ^ mask_pattern3[read_cnt]; end
                4:begin code_n = sram_rdata ^ mask_pattern4[read_cnt]; end
                5:begin code_n = sram_rdata ^ mask_pattern5[read_cnt]; end
                6:begin code_n = sram_rdata ^ mask_pattern6[read_cnt]; end
                7:begin code_n = sram_rdata ^ mask_pattern7[read_cnt]; end
                default:begin code_n = 0; end
            endcase
            x_cnt_n = (x_cnt == 16)? (y_cnt == 8)? 0  : 9 : x_cnt + 1;
            y_cnt_n = (x_cnt == 16)? y_cnt + 1 : y_cnt;
            read_cnt_n = read_cnt + 1;
            out_cnt_n = 0;
            gf_cnt_n = 43;
            gf_cnt_y_n = 0;
        end
        DE_MASKING_2:begin
            state_n = (x_cnt == 24 &&  y_cnt == 16)? DE_MASKING_3 : DE_MASKING_2;
            mask_raddr = position + 64*real_y_cnt + real_x_cnt;
            mask_n = 0;
            mask_cnt_n = 0;
            case(real_mask)
                0:begin code_n = sram_rdata ^ mask_pattern0[read_cnt]; end
                1:begin code_n = sram_rdata ^ mask_pattern1[read_cnt]; end
                2:begin code_n = sram_rdata ^ mask_pattern2[read_cnt]; end
                3:begin code_n = sram_rdata ^ mask_pattern3[read_cnt]; end
                4:begin code_n = sram_rdata ^ mask_pattern4[read_cnt]; end
                5:begin code_n = sram_rdata ^ mask_pattern5[read_cnt]; end
                6:begin code_n = sram_rdata ^ mask_pattern6[read_cnt]; end
                7:begin code_n = sram_rdata ^ mask_pattern7[read_cnt]; end
                default:begin code_n = 0; end
            endcase
            x_cnt_n = (x_cnt == 24)? (y_cnt == 16)? 9 : 0 : x_cnt + 1;
            y_cnt_n = (x_cnt == 24)? y_cnt + 1 : y_cnt;
            read_cnt_n = read_cnt + 1;
            out_cnt_n = 0;
            gf_cnt_n = 43;
            gf_cnt_y_n = 0;
        end
        DE_MASKING_3:begin
            state_n = (x_cnt == 24 && y_cnt == 24)? CALCULATE_SYNDROME_2 : DE_MASKING_3;
            mask_raddr = position + 64*real_y_cnt + real_x_cnt;
            mask_n = 0;
            mask_cnt_n = 0;
            case(real_mask)
                0:begin code_n = sram_rdata ^ mask_pattern0[read_cnt]; end
                1:begin code_n = sram_rdata ^ mask_pattern1[read_cnt]; end
                2:begin code_n = sram_rdata ^ mask_pattern2[read_cnt]; end
                3:begin code_n = sram_rdata ^ mask_pattern3[read_cnt]; end
                4:begin code_n = sram_rdata ^ mask_pattern4[read_cnt]; end
                5:begin code_n = sram_rdata ^ mask_pattern5[read_cnt]; end
                6:begin code_n = sram_rdata ^ mask_pattern6[read_cnt]; end
                7:begin code_n = sram_rdata ^ mask_pattern7[read_cnt]; end
                default:begin code_n = 0; end
            endcase
            x_cnt_n = (x_cnt == 24)? 9 :x_cnt + 1;
            y_cnt_n = (x_cnt == 24)? y_cnt + 1:y_cnt;
            read_cnt_n = read_cnt + 1;
            out_cnt_n = 0;
            gf_cnt_n = 43;
            gf_cnt_y_n = 0;
        end
        CALCULATE_SYNDROME_1:begin
            state_n = CALCULATE_SYNDROME_2;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
            gf_cnt_n = gf_cnt - 1;
            gf_cnt_y_n = 0;
        end
        CALCULATE_SYNDROME_2:begin
            state_n = (gf_cnt_y == 7) ? (gf_cnt == 0)? SOLVE_EQUATION :CALCULATE_SYNDROME_1: CALCULATE_SYNDROME_2;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
            gf_cnt_n =  gf_cnt;
            gf_cnt_y_n = gf_cnt_y + 1;
        end
        SOLVE_EQUATION:begin
            state_n = (finish_sys == 1)? FIND_ERROR_POSITION : SOLVE_EQUATION;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
            gf_cnt_n =  0;
            gf_cnt_y_n = 0;
        end
        FIND_ERROR_POSITION:begin
            state_n = (finish_err_pos)? SOLVE_EQUATION_2 : FIND_ERROR_POSITION;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
            gf_cnt_n =  0;
            gf_cnt_y_n = 0;
        end
        SOLVE_EQUATION_2:begin
            state_n = (finish_sys)? FIX_ERROR : SOLVE_EQUATION_2;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
            gf_cnt_n =  0;
            gf_cnt_y_n = 0;
        end
        FIX_ERROR : begin
            state_n = (fix_error_cnt == error_count - 1)? OUTPUT : FIX_ERROR;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
            gf_cnt_n =  0;
            gf_cnt_y_n = 0;    
        end
        OUTPUT:begin
            state_n = (out_cnt == length-1)? FINISH : OUTPUT;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = out_cnt + 1;
            gf_cnt_n = 0;
            gf_cnt_y_n = 0;    
        end
        FINISH:begin
            state_n = FINISH;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
            gf_cnt_n = 0;
            gf_cnt_y_n = 0;    
        end
        default:begin
            state_n = 0;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
            gf_cnt_n = 0;
            gf_cnt_y_n = 0;    
        end
    endcase
end

always@(*)begin
    decode_jis8_code_n = {c_correct[out_cnt+1][3:0],c_correct[out_cnt+2][7:4]};
    decode_valid_n = (state == OUTPUT)?1:0;
    qr_decode_finish_n = (state == FINISH)?1:0;
end
always@(posedge clk)begin
    if(~srstn)begin
        decode_valid <= 0;
    end
    else begin
        decode_valid <= decode_valid_n;
    end
end

always@(posedge clk)begin
    decode_jis8_code <= decode_jis8_code_n;
    qr_decode_finish <= qr_decode_finish_n;
end

endmodule


module detect_rotation(
input clk,
input srstn,
input enable,
input sram_rdata,
output reg  [11:0] sram_raddr,
output reg [2:0] mode,
output reg [11:0] position,
output finish
);

localparam [3:0] IDLE = 0;
localparam [3:0] CHECK_ROW_19 = 7;
localparam [3:0] CHECK_ROW_29 = 8;
localparam [3:0] CHECK_ROW_9  = 9;
localparam [3:0] CHECK_ROW_34 =10;
localparam [3:0] CHECK_ROW_24 =11;
localparam [3:0] CHECK_ROW_14 =12;
localparam [3:0] CHECK_ROW_4  =13;


localparam [3:0] FIND_FIRST_1_CODE = 1;
localparam [3:0] IDENTIFY_POSITION_DETECTION_PATTERN = 2;
localparam [3:0] COMPARE = 3;
localparam [3:0] CHECK_IF_180 = 4;
localparam [3:0] FINISH = 5;
localparam [3:0] FIND_POSITION = 6;


reg [3:0] state;
reg [3:0] state_n;


always@(posedge clk)begin
    if(~srstn)begin
        state <= IDLE;
    end
    else begin
        state <= state_n;
    end
end

reg [11:0] sram_raddr_n;
reg [11:0] position_n;
reg [48:0] buffer;
reg buffer_n;
reg [2:0] x_cnt;
reg [2:0] x_cnt_n;
reg [2:0] y_cnt;
reg [2:0] y_cnt_n;

reg [2:0] mode_n;

reg [1:0] check_pattern_times;
reg [1:0] check_pattern_times_n;

reg [7:0] min_col, min_col_n; 

wire [48:0] pattern = 49'b1111111_1000001_1011101_1011101_1011101_1000001_1111111;
wire is_pattern;
wire [5:0] row;
reg [5:0] start_row;
reg [5:0] start_row_n;
assign row = sram_raddr >> 6;

assign is_pattern = buffer == pattern;
assign finish = state == FINISH;

integer i;
always@(posedge clk)begin
    if(~srstn)begin
        sram_raddr <= 0;
        x_cnt <= 0;
        y_cnt <= 0;
        mode <= 0;
        check_pattern_times <= 0;
        position <= 0;
        for(i = 0 ; i<49;i = i +1)begin
            buffer[i] <= 0;
        end
    end
    else begin
        sram_raddr <= sram_raddr_n;
        x_cnt <= x_cnt_n;
        y_cnt <= y_cnt_n;
        mode <= mode_n;
        check_pattern_times <= check_pattern_times_n;
        position <= position_n;
        for(i = 48 ; i >= 1;i = i - 1)begin
            buffer[i] <= buffer[i-1];
        end
        buffer[0] <= buffer_n;
    end
end

always@(posedge clk)begin
    min_col <= min_col_n;
    start_row <= start_row_n;
end
always@(*)begin
    case(state)
        IDLE:begin
            state_n = (enable)? CHECK_ROW_19 : IDLE;      
            sram_raddr_n = 19*64; 
            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = 0;
            check_pattern_times_n = 0;
            position_n = 0;

            min_col_n = 0;
            start_row_n = 0;

        end
        CHECK_ROW_19:begin
            state_n = (sram_rdata == 0)?(sram_raddr ==  19*64 + 63)? CHECK_ROW_29 : CHECK_ROW_19 : CHECK_ROW_9;
            sram_raddr_n = (sram_rdata == 0)?(sram_raddr == 19*64 + 63)? 64*29 : sram_raddr + 1 : (sram_raddr -64*19-24>0)? 64*9 + (sram_raddr - 64*19 -24) : 64*9 ;
            
            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = 0;
            check_pattern_times_n = 0;
            position_n = 0;
            
            min_col_n = (sram_rdata == 1) ? (sram_raddr -64*19-24>0)? sram_raddr - 64*19-24 : 0 : min_col;
            start_row_n = 0;

        end
        CHECK_ROW_29:begin
            state_n = (sram_rdata == 0)?(sram_raddr == 64*29+63)? CHECK_ROW_34 : CHECK_ROW_29 : CHECK_ROW_24;
            sram_raddr_n = (sram_rdata == 0)?(sram_raddr == 64*29+63)? 64*34 : sram_raddr + 1 : (sram_raddr - 64*29 -24 > 0)? 64*24 + (sram_raddr - 64*29-24) : 64*24;

            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = 0;
            check_pattern_times_n = 0;
            position_n = 0;

            min_col_n = (sram_rdata == 1) ? (sram_raddr -64*29-24>0)? sram_raddr - 64*29-24 : 0: min_col;
            start_row_n = 0;
        end

        CHECK_ROW_9:begin
            state_n = (sram_rdata == 0)?(sram_raddr == 64*9+63)? CHECK_ROW_14 : CHECK_ROW_9 : CHECK_ROW_4;
            sram_raddr_n = (sram_rdata == 0)?(sram_raddr == 64*9+63)? 64*14 : sram_raddr + 1 : (sram_raddr - 64*9 -24 > 0)? 64*4 + (sram_raddr - 64*9-24) : 64*4;
            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = 0;
            check_pattern_times_n = 0;
            position_n = 0;
            
            min_col_n = (sram_rdata == 1) ? (sram_raddr -64*9-24>0)? sram_raddr - 64*9-24 : 0: min_col;
            start_row_n = 0;
        end

        CHECK_ROW_34:begin
            state_n = (sram_rdata == 0)?(sram_raddr == 64*34+63)? FIND_FIRST_1_CODE : CHECK_ROW_34 : FIND_FIRST_1_CODE;
            sram_raddr_n = (sram_rdata == 0)?(sram_raddr == 64*34+63)? 64*35 : sram_raddr + 1 : (sram_raddr - 64*34 -24 > 0)? 64*30 + (sram_raddr - 64*34-24) : 64*30;
            
            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = 0;
            check_pattern_times_n = 0;
            position_n = 0;

            min_col_n = (sram_rdata == 1) ? (sram_raddr -64*34-24>0)? sram_raddr - 64*34-24 : 0: min_col;
            start_row_n = (sram_rdata == 0)? 35 : 30;

        end
        CHECK_ROW_24:begin
            state_n = (sram_rdata == 0)?(sram_raddr == 64*24+63)? FIND_FIRST_1_CODE : CHECK_ROW_24 : FIND_FIRST_1_CODE;
            sram_raddr_n = (sram_rdata == 0)?(sram_raddr == 64*24+63)? 64*25 : sram_raddr + 1 : (sram_raddr - 64*24 -24 > 0)? 64*20 + (sram_raddr - 64*34-24) : 64*20;
        
            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = 0;
            check_pattern_times_n = 0;
            position_n = 0;
            
            min_col_n = (sram_rdata == 1) ? (sram_raddr -64*24-24>0)? sram_raddr - 64*24-24 : 0: min_col;
            start_row_n = (sram_rdata == 0)? 25 : 20;

        end
        CHECK_ROW_14:begin
            state_n = (sram_rdata == 0)?(sram_raddr == 64*14+63)? FIND_FIRST_1_CODE : CHECK_ROW_14 : FIND_FIRST_1_CODE;
            sram_raddr_n = (sram_rdata == 0)?(sram_raddr == 64*14+63)? 64*15 : sram_raddr + 1 : (sram_raddr - 64*14 -24 > 0)? 64*10 + (sram_raddr - 64*14-24) : 64*10;

            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = 0;
            check_pattern_times_n = 0;
            position_n = 0;

            min_col_n = (sram_rdata == 1) ? (sram_raddr -64*14-24>0)? sram_raddr - 64*14-24 : 0: min_col;
            start_row_n = (sram_rdata == 0)? 15 : 10;
        end
        CHECK_ROW_4:begin
            state_n = (sram_rdata == 0)?(sram_raddr == 64*4+63)? FIND_FIRST_1_CODE : CHECK_ROW_4 : FIND_FIRST_1_CODE;
            sram_raddr_n = (sram_rdata == 0)?(sram_raddr == 64*4 +63)? 64* 5 : sram_raddr + 1 : (sram_raddr - 64* 4 -24 > 0)? 64* 0 + (sram_raddr - 64* 4-24) : 64*0;

            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = 0;
            check_pattern_times_n = 0;
            position_n = 0;

            min_col_n = (sram_rdata == 1) ? (sram_raddr -64*4-24>0)? sram_raddr - 64*4-24 : 0: min_col;
            start_row_n = (sram_rdata == 0)? 5 : 0;
        end

        FIND_FIRST_1_CODE:begin
            state_n = (sram_rdata == 1)? CHECK_IF_180 : FIND_FIRST_1_CODE;
            sram_raddr_n = (sram_rdata != 1)? sram_raddr + 1 : sram_raddr+24;
            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = 0;
            check_pattern_times_n = 0;
            position_n = sram_raddr;

            min_col_n = min_col;
            start_row_n = start_row;
        end
        CHECK_IF_180:begin
            state_n = (sram_rdata == 0)? FIND_POSITION : IDENTIFY_POSITION_DETECTION_PATTERN;
            sram_raddr_n = (sram_rdata == 0)? sram_raddr-1 : sram_raddr - 24;
            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = (sram_rdata == 0)? 3:0;
            check_pattern_times_n = 0;
            position_n = position;
            min_col_n = min_col;
            start_row_n = start_row;
        end
        IDENTIFY_POSITION_DETECTION_PATTERN:begin
            state_n = (x_cnt == 6 && y_cnt == 6)? COMPARE : IDENTIFY_POSITION_DETECTION_PATTERN;
            sram_raddr_n = (x_cnt == 6)? sram_raddr+64-6 : sram_raddr + 1;
            x_cnt_n = (x_cnt == 6)? 0 :x_cnt + 1; 
            y_cnt_n = (x_cnt == 6)? y_cnt + 1 : y_cnt ; 
            buffer_n = sram_rdata;
            mode_n = 0;
            check_pattern_times_n = check_pattern_times;
            position_n = position;
            min_col_n = min_col;
            start_row_n = start_row;
        end
        COMPARE:begin
            state_n = (is_pattern)? (check_pattern_times != 2)?IDENTIFY_POSITION_DETECTION_PATTERN:FINISH : FINISH;
            if(is_pattern == 1)begin
                case(check_pattern_times)
                    0: sram_raddr_n = sram_raddr-64+6-64*6+12;
                    1: sram_raddr_n = sram_raddr+64*11;
                    2: sram_raddr_n = sram_raddr-64+6-64*6-12-6;
                    default: sram_raddr_n = 0;
                endcase
            end
            else begin
                sram_raddr_n = 0;
            end

            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;

            if(is_pattern == 0)begin
                case(check_pattern_times)
                    0 : mode_n = 3;
                    1 : mode_n = 2;
                    2 : mode_n = 1;
                    default : mode_n = 0;
                endcase
            end
            else begin
                mode_n = (check_pattern_times != 2) ? 0:4;
            end

            check_pattern_times_n = check_pattern_times + 1;
            position_n = position;
            min_col_n = min_col;
            start_row_n = start_row;
        
        end
        FIND_POSITION:begin
            state_n = (sram_rdata == 1)? FINISH : FIND_POSITION;
            sram_raddr_n = sram_raddr-1;
            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = mode;
            check_pattern_times_n = 0;
            position_n = sram_raddr-24;
            min_col_n = min_col;
            start_row_n = start_row;
        end
        FINISH:begin
            state_n = state;
            sram_raddr_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = mode;
            check_pattern_times_n = 0;
            position_n = position;
            min_col_n = min_col;
            start_row_n = start_row;
        end

        default:begin
            state_n = IDLE;
            sram_raddr_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = 0;
            check_pattern_times_n = 0;
            position_n = 0;
            min_col_n = 0;
            start_row_n = start_row;
        end
    endcase
end

endmodule
module systolic(
input [7:0] in1,
input [7:0] in2,
input [7:0] in3,
input [7:0] in4,
input [7:0] in5,
input clk,
input start,
input srstn,
input mode,

output reg [7:0] sigma_1,
output reg [7:0] sigma_2,
output reg [7:0] sigma_3,
output reg [7:0] sigma_4,
output reg finish

);
wire finish_n;

wire[7:0] w_p11_p12; 
wire[7:0] w_p12_p13; 
wire[7:0] w_p13_p14; 
wire[7:0] w_p14_p15; 
wire[7:0] w_p15_p16; 

wire[7:0] w_p21_p22;
wire[7:0] w_p22_p23;
wire[7:0] w_p23_p24;
wire[7:0] w_p24_p25;

wire[7:0] w_p31_p32;
wire[7:0] w_p32_p33;
wire[7:0] w_p33_p34;

wire[7:0] w_p41_p42;
wire[7:0] w_p42_p43;

wire s_p11_p12;
wire s_p12_p13;
wire s_p13_p14;
wire s_p14_p15;
wire s_p15_p16;

wire s_p21_p22;
wire s_p22_p23;
wire s_p23_p24;
wire s_p24_p25;

wire s_p31_p32;
wire s_p32_p33;
wire s_p33_p34;

wire s_p41_p42;
wire s_p42_p43;

wire[7:0] w_p12_p21;
wire[7:0] w_p13_p22;
wire[7:0] w_p14_p23;
wire[7:0] w_p15_p24;

wire[7:0] w_p22_p31; 
wire[7:0] w_p23_p32; 
wire[7:0] w_p24_p33; 

wire[7:0] w_p32_p41; 
wire[7:0] w_p33_p42; 

wire[7:0] w_p42_p51; 

wire[7:0] in_g_p11;
wire[7:0] in_g_p12;
wire[7:0] in_g_p13;
wire[7:0] in_g_p14;
wire[7:0] in_g_p15;

wire[7:0] in_g_p21;
wire[7:0] in_g_p22;
wire[7:0] in_g_p23;
wire[7:0] in_g_p24;

wire[7:0] in_g_p31;
wire[7:0] in_g_p32;
wire[7:0] in_g_p33;

wire[7:0] in_g_p41;
wire[7:0] in_g_p42;

wire [7:0] w_p12_p21_tmp;
wire [7:0] w_p13_p22_tmp;
wire [7:0] w_p14_p23_tmp;
wire [7:0] w_p15_p24_tmp;

//circle p11 (.clk(clk),.in(in1),.out(w_p11_p12),.sign(s_p11_p12),.in_g(in_g_p11));
//rect   p12 (.clk(clk),.in(in2),.in_c(w_p11_p12),.in_sign(s_p11_p12),.out_c(w_p12_p13),.out_sign(s_p12_p13),.out(w_p12_p21),.in_g(in_g_p12));
//rect   p13 (.clk(clk),.in(in3),.in_c(w_p12_p13),.in_sign(s_p12_p13),.out_c(w_p13_p14),.out_sign(s_p13_p14),.out(w_p13_p22),.in_g(in_g_p13));
//rect   p14 (.clk(clk),.in(in4),.in_c(w_p13_p14),.in_sign(s_p13_p14),.out_c(w_p14_p15),.out_sign(s_p14_p15),.out(w_p14_p23),.in_g(in_g_p14));
//rect   p15 (.clk(clk),.in(in5),.in_c(w_p14_p15),.in_sign(s_p14_p15),.out_c(w_p15_p16),.out_sign(s_p15_p16),.out(w_p15_p24),.in_g(in_g_p15));

circle p11 (.clk(clk),.in(in1),.out(w_p11_p12),.sign(s_p11_p12),.in_g(in_g_p11));
rect   p12 (.clk(clk),.in(in2),.in_c(w_p11_p12),.in_sign(s_p11_p12),.out_c(w_p12_p13),.out_sign(s_p12_p13),.out(w_p12_p21_tmp),.in_g(in_g_p12));
rect   p13 (.clk(clk),.in(in3),.in_c(w_p12_p13),.in_sign(s_p12_p13),.out_c(w_p13_p14),.out_sign(s_p13_p14),.out(w_p13_p22_tmp),.in_g(in_g_p13));
rect   p14 (.clk(clk),.in(in4),.in_c(w_p13_p14),.in_sign(s_p13_p14),.out_c(w_p14_p15),.out_sign(s_p14_p15),.out(w_p14_p23_tmp),.in_g(in_g_p14));
rect   p15 (.clk(clk),.in(in5),.in_c(w_p14_p15),.in_sign(s_p14_p15),.out_c(w_p15_p16),.out_sign(s_p15_p16),.out(w_p15_p24_tmp),.in_g(in_g_p15));




reg [7:0] in1_d1, in1_d2;
reg [7:0] in2_d1, in2_d2;
reg [7:0] in3_d1, in3_d2;
reg [7:0] in5_d1;

always@(posedge clk)begin
    in1_d1 <= in1;
    in1_d2 <= in1_d1;
    
    in2_d1 <= in2;
    in2_d2 <= in2_d1;
    
    in3_d1 <= in3;
    in3_d2 <= in3_d1;
    
    in5_d1 <= in5;
end

assign w_p12_p21 = (mode == 0) ? w_p12_p21_tmp : in1_d2;
assign w_p13_p22 = (mode == 0) ? w_p13_p22_tmp : in2_d2;
assign w_p14_p23 = (mode == 0) ? w_p14_p23_tmp : in3_d2;
assign w_p15_p24 = (mode == 0) ? w_p15_p24_tmp : in5_d1;

circle p21 (.clk(clk),.in(w_p12_p21),.out(w_p21_p22),.sign(s_p21_p22),.in_g(in_g_p21));
rect   p22 (.clk(clk),.in(w_p13_p22),.in_c(w_p21_p22),.in_sign(s_p21_p22),.out_c(w_p22_p23),.out_sign(s_p22_p23),.out(w_p22_p31),.in_g(in_g_p22));
rect   p23 (.clk(clk),.in(w_p14_p23),.in_c(w_p22_p23),.in_sign(s_p22_p23),.out_c(w_p23_p24),.out_sign(s_p23_p24),.out(w_p23_p32),.in_g(in_g_p23));
rect   p24 (.clk(clk),.in(w_p15_p24),.in_c(w_p23_p24),.in_sign(s_p23_p24),.out_c(w_p24_p25),.out_sign(s_p24_p25),.out(w_p24_p33),.in_g(in_g_p24));

circle p31 (.clk(clk),.in(w_p22_p31),.out(w_p31_p32),.sign(s_p31_p32),.in_g(in_g_p31));
rect   p32 (.clk(clk),.in(w_p23_p32),.in_c(w_p31_p32),.in_sign(s_p31_p32),.out_c(w_p32_p33),.out_sign(s_p32_p33),.out(w_p32_p41),.in_g(in_g_p32));
rect   p33 (.clk(clk),.in(w_p24_p33),.in_c(w_p32_p33),.in_sign(s_p32_p33),.out_c(w_p33_p34),.out_sign(s_p33_p34),.out(w_p33_p42),.in_g(in_g_p33));


circle p41 (.clk(clk),.in(w_p32_p41),.out(w_p41_p42),.sign(s_p41_p42),.in_g(in_g_p41));
rect   p42 (.clk(clk),.in(w_p33_p42),.in_c(w_p41_p42),.in_sign(s_p41_p42),.out_c(w_p42_p43),.out_sign(s_p42_p43),.out(w_p42_p51),.in_g(in_g_p42));

reg [4:0] cnt;

always@(posedge clk)begin
    if(~srstn)begin
        cnt <= 0;
    end
    else begin
        if(start)begin
            cnt <= cnt + 1;
        end
        else begin
            cnt <= 0;
        end

    end
end

reg [7:0] sigma_1_n, sigma_2_n, sigma_3_n, sigma_4_n;



reg [7:0] in_g_p41_delay1;

reg [7:0] in_g_p32_delay1;
reg [7:0] in_g_p32_delay2;
reg [7:0] w_p24_p33_delay1;
reg [7:0] in_g_p31_delay1;
reg [7:0] in_g_p31_delay2;
reg [7:0] in_g_p31_delay3;

reg [7:0] log_u1_in;
wire [7:0] log_u1_out;
reg [7:0] anti_u1_in;
wire [7:0] anti_u1_out;

log logu1(.in(log_u1_in),.out(log_u1_out));
antilog antiu1(.in(anti_u1_in),.out(anti_u1_out));

reg [7:0] log_u2_in;
wire [7:0] log_u2_out;
reg [7:0] anti_u2_in;
wire [7:0] anti_u2_out;
reg [7:0] anti_u3_in;
wire [7:0] anti_u3_out;

log logu2(.in(log_u2_in),.out(log_u2_out));
antilog antiu2(.in(anti_u2_in),.out(anti_u2_out));
antilog antiu3(.in(anti_u3_in),.out(anti_u3_out));

reg [7:0] w_p15_p24_delay1;
reg [7:0] w_p15_p24_delay2;
reg [7:0] in_g_p23_delay1;
reg [7:0] in_g_p23_delay2;
reg [7:0] in_g_p23_delay3;
reg [7:0] in_g_p22_delay1;
reg [7:0] in_g_p22_delay2;
reg [7:0] in_g_p22_delay3;
reg [7:0] in_g_p22_delay4;
reg [7:0] in_g_p21_delay1;
reg [7:0] in_g_p21_delay2;
reg [7:0] in_g_p21_delay3;
reg [7:0] in_g_p21_delay4;
reg [7:0] in_g_p21_delay5;


reg [7:0] log_u3_in;
wire [7:0] log_u3_out;
reg [7:0] anti_u4_in;
wire [7:0] anti_u4_out;
reg [7:0] anti_u5_in;
wire [7:0] anti_u5_out;
reg [7:0] anti_u6_in;
wire [7:0] anti_u6_out;

log logu3(.in(log_u3_in),.out(log_u3_out));
antilog antiu4(.in(anti_u4_in),.out(anti_u4_out));
antilog antiu5(.in(anti_u5_in),.out(anti_u5_out));
antilog antiu6(.in(anti_u6_in),.out(anti_u6_out));


reg [7:0] in5_delay1;
reg [7:0] in5_delay2;
reg [7:0] in5_delay3;
reg [7:0] in_g_p14_delay1;
reg [7:0] in_g_p14_delay2;
reg [7:0] in_g_p14_delay3;
reg [7:0] in_g_p14_delay4;
reg [7:0] in_g_p13_delay1;
reg [7:0] in_g_p13_delay2;
reg [7:0] in_g_p13_delay3;
reg [7:0] in_g_p13_delay4;
reg [7:0] in_g_p13_delay5;
reg [7:0] in_g_p12_delay1;
reg [7:0] in_g_p12_delay2;
reg [7:0] in_g_p12_delay3;
reg [7:0] in_g_p12_delay4;
reg [7:0] in_g_p12_delay5;
reg [7:0] in_g_p12_delay6;
reg [7:0] in_g_p11_delay1;
reg [7:0] in_g_p11_delay2;
reg [7:0] in_g_p11_delay3;
reg [7:0] in_g_p11_delay4;
reg [7:0] in_g_p11_delay5;
reg [7:0] in_g_p11_delay6;
reg [7:0] in_g_p11_delay7;


always@(posedge clk)begin
    in_g_p41_delay1 <= in_g_p41;
    
    in_g_p32_delay1 <= in_g_p32;
    in_g_p32_delay2 <= in_g_p32_delay1;
    w_p24_p33_delay1 <= w_p24_p33;
    in_g_p31_delay1 <= in_g_p31;
    in_g_p31_delay2 <= in_g_p31_delay1;
    in_g_p31_delay3 <= in_g_p31_delay2;

    w_p15_p24_delay1 <= w_p15_p24;
    w_p15_p24_delay2 <= w_p15_p24_delay1;
    in_g_p23_delay1 <= in_g_p23;
    in_g_p23_delay2 <= in_g_p23_delay1;
    in_g_p23_delay3 <= in_g_p23_delay2;
    in_g_p22_delay1 <= in_g_p22;
    in_g_p22_delay2 <= in_g_p22_delay1;
    in_g_p22_delay3 <= in_g_p22_delay2;
    in_g_p22_delay4 <= in_g_p22_delay3;
    in_g_p21_delay1 <= in_g_p21;
    in_g_p21_delay2 <= in_g_p21_delay1;
    in_g_p21_delay3 <= in_g_p21_delay2;
    in_g_p21_delay4 <= in_g_p21_delay3;
    in_g_p21_delay5 <= in_g_p21_delay4;

   // in5_delay1 <= in5;
    in5_delay1 <= (mode == 0) ? in5 : in5_d1;
    in5_delay2 <= in5_delay1;
    in5_delay3 <= in5_delay2;
    in_g_p14_delay1 <= in_g_p14;
    in_g_p14_delay2 <= in_g_p14_delay1;
    in_g_p14_delay3 <= in_g_p14_delay2;
    in_g_p14_delay4 <= in_g_p14_delay3;
    in_g_p13_delay1 <= in_g_p13;
    in_g_p13_delay2 <= in_g_p13_delay1;
    in_g_p13_delay3 <= in_g_p13_delay2;
    in_g_p13_delay4 <= in_g_p13_delay3;
    in_g_p13_delay5 <= in_g_p13_delay4;
    in_g_p12_delay1 <= in_g_p12;
    in_g_p12_delay2 <= in_g_p12_delay1;
    in_g_p12_delay3 <= in_g_p12_delay2;
    in_g_p12_delay4 <= in_g_p12_delay3;
    in_g_p12_delay5 <= in_g_p12_delay4;
    in_g_p12_delay6 <= in_g_p12_delay5;
    in_g_p11_delay1 <= in_g_p11;
    in_g_p11_delay2 <= in_g_p11_delay1;
    in_g_p11_delay3 <= in_g_p11_delay2;
    in_g_p11_delay4 <= in_g_p11_delay3;
    in_g_p11_delay5 <= in_g_p11_delay4;
    in_g_p11_delay6 <= in_g_p11_delay5;
    in_g_p11_delay7 <= in_g_p11_delay6;

end

always@(posedge clk)begin
    sigma_1 <= sigma_1_n;
    sigma_2 <= sigma_2_n;
    sigma_3 <= sigma_3_n;
    sigma_4 <= sigma_4_n;
    finish <= finish_n;
end

always@(*)begin
    sigma_1_n = (cnt == 7)? (in_g_p42 >= in_g_p41_delay1)? in_g_p42 - in_g_p41_delay1 : in_g_p42 - in_g_p41_delay1 - 1 : sigma_1;
end

always@(*)begin
    anti_u1_in = ({1'b0,sigma_1} + {1'b0,in_g_p32_delay2} >= 255)? sigma_1 + in_g_p32_delay2 + 1 : sigma_1 + in_g_p32_delay2;
    log_u1_in = anti_u1_out ^ w_p24_p33_delay1;
    sigma_2_n = (cnt == 8)? (log_u1_out >= in_g_p31_delay3)? log_u1_out - in_g_p31_delay3 : log_u1_out - in_g_p31_delay3-1 : sigma_2;
end

always@(*)begin
    anti_u2_in = ({1'b0,sigma_1} + {1'b0,in_g_p23_delay3} >= 255)? sigma_1 + in_g_p23_delay3 + 1 : sigma_1 + in_g_p23_delay3;
    anti_u3_in = ({1'b0,sigma_2} + {1'b0,in_g_p22_delay4} >= 255)? sigma_2 + in_g_p22_delay4 + 1 : sigma_2 + in_g_p22_delay4;
    log_u2_in = anti_u2_out ^ anti_u3_out ^ w_p15_p24_delay2;
    sigma_3_n = (cnt == 9)? (log_u2_out >= in_g_p21_delay5)? log_u2_out - in_g_p21_delay5 : log_u2_out - in_g_p21_delay5-1 : sigma_3;
end

always@(*)begin
    anti_u4_in = ({1'b0,sigma_1} + {1'b0,in_g_p14_delay4} >= 255)? sigma_1 + in_g_p14_delay4 + 1 : sigma_1 + in_g_p14_delay4;
    anti_u5_in = ({1'b0,sigma_2} + {1'b0,in_g_p13_delay5} >= 255)? sigma_2 + in_g_p13_delay5 + 1 : sigma_2 + in_g_p13_delay5;
    anti_u6_in = ({1'b0,sigma_3} + {1'b0,in_g_p12_delay6} >= 255)? sigma_3 + in_g_p12_delay6 + 1 : sigma_3 + in_g_p12_delay6;
    log_u3_in = anti_u4_out ^ anti_u5_out ^ anti_u6_out ^ in5_delay3;
    sigma_4_n = (cnt == 10)? (log_u3_out >= in_g_p11_delay7)? log_u3_out - in_g_p11_delay7 : log_u3_out - in_g_p11_delay7-1 : sigma_4;
    
end


assign finish_n = (cnt == 10)?1:0;


endmodule

module add_gf(
input [7:0] in1,
input [7:0] in2,
output reg [7:0] out
);


always@(*)begin
    if({1'b0, in1} + {1'b0,in2} >= 255)begin
        out = in1 + in2 + 1;
    end
    else begin
        out = in1+in2;
    end
end
endmodule



module rect(
input clk,
input [7:0] in,
input [7:0] in_c,
input in_sign,

output reg out_sign,
output reg [7:0]out_c,
output reg [7:0] out,
output [7:0] in_g
);


wire [7:0] in1;
reg [7:0] in2;
reg [7:0] in3;
wire [7:0] in4;
reg [8:0] tmp;

log u1 (.in(in),.out(in1));
antilog u2(.in(in3),.out(in4));
assign in_g = in1;

always@(*)begin
    tmp =  {1'b0,in1} + {1'b0,in_c};
    if(in_sign == 0)begin
        if(tmp >= 255)begin
            in2 = tmp[7:0] + 8'b1;
        end
        else begin
            in2 = tmp[7:0];
        end
    end
    else begin
        if(in1 >= in_c)begin
            in2 = in1-in_c;
        end
        else begin
            in2 = in1-in_c-1;
        end
    end
end

always@(posedge clk)begin
    in3 <= in2;
    out_sign <= in_sign;
    out_c <= in_c;
end

always@(*)begin
    out = in4 ^ in;
end


endmodule

module circle(
input clk,
input [7:0] in,
output reg [7:0] out,
output reg sign,
output [7:0]in_g
);

reg [7:0] in2;
reg [8:0] tmp;

wire [7:0] in1;

log u1 (.in(in),.out(in1));
assign in_g = in1;

always@(posedge clk)begin 
    in2 <= in1;
end

always@(*)begin
    sign = (in1 > in2) ? 0:1;
    out = (in1 > in2) ? in1-in2 : in2-in1;
end

endmodule

module log(
input [7:0] in,
output reg [7:0] out
);

always@(*)begin
    (* synthesis, parallel_case *)
    case(in)
        1:begin out = 0; end
        2:begin out = 1; end
        3:begin out = 25; end
        4:begin out = 2; end
        5:begin out = 50; end
        6:begin out = 26; end
        7:begin out = 198; end
        8:begin out = 3; end
        9:begin out = 223; end
        10:begin out = 51; end
        11:begin out = 238; end
        12:begin out = 27; end
        13:begin out = 104; end
        14:begin out = 199; end
        15:begin out = 75; end
        16:begin out = 4; end
        17:begin out = 100; end
        18:begin out = 224; end
        19:begin out = 14; end
        20:begin out = 52; end
        21:begin out = 141; end
        22:begin out = 239; end
        23:begin out = 129; end
        24:begin out = 28; end
        25:begin out = 193; end
        26:begin out = 105; end
        27:begin out = 248; end
        28:begin out = 200; end
        29:begin out = 8; end
        30:begin out = 76; end
        31:begin out = 113; end
        32:begin out = 5; end
        33:begin out = 138; end
        34:begin out = 101; end
        35:begin out = 47; end
        36:begin out = 225; end
        37:begin out = 36; end
        38:begin out = 15; end
        39:begin out = 33; end
        40:begin out = 53; end
        41:begin out = 147; end
        42:begin out = 142; end
        43:begin out = 218; end
        44:begin out = 240; end
        45:begin out = 18; end
        46:begin out = 130; end
        47:begin out = 69; end
        48:begin out = 29; end
        49:begin out = 181; end
        50:begin out = 194; end
        51:begin out = 125; end
        52:begin out = 106; end
        53:begin out = 39; end
        54:begin out = 249; end
        55:begin out = 185; end
        56:begin out = 201; end
        57:begin out = 154; end
        58:begin out = 9; end
        59:begin out = 120; end
        60:begin out = 77; end
        61:begin out = 228; end
        62:begin out = 114; end
        63:begin out = 166; end
        64:begin out = 6; end
        65:begin out = 191; end
        66:begin out = 139; end
        67:begin out = 98; end
        68:begin out = 102; end
        69:begin out = 221; end
        70:begin out = 48; end
        71:begin out = 253; end
        72:begin out = 226; end
        73:begin out = 152; end
        74:begin out = 37; end
        75:begin out = 179; end
        76:begin out = 16; end
        77:begin out = 145; end
        78:begin out = 34; end
        79:begin out = 136; end
        80:begin out = 54; end
        81:begin out = 208; end
        82:begin out = 148; end
        83:begin out = 206; end
        84:begin out = 143; end
        85:begin out = 150; end
        86:begin out = 219; end
        87:begin out = 189; end
        88:begin out = 241; end
        89:begin out = 210; end
        90:begin out = 19; end
        91:begin out = 92; end
        92:begin out = 131; end
        93:begin out = 56; end
        94:begin out = 70; end
        95:begin out = 64; end
        96:begin out = 30; end
        97:begin out = 66; end
        98:begin out = 182; end
        99:begin out = 163; end
        100:begin out = 195; end
        101:begin out = 72; end
        102:begin out = 126; end
        103:begin out = 110; end
        104:begin out = 107; end
        105:begin out = 58; end
        106:begin out = 40; end
        107:begin out = 84; end
        108:begin out = 250; end
        109:begin out = 133; end
        110:begin out = 186; end
        111:begin out = 61; end
        112:begin out = 202; end
        113:begin out = 94; end
        114:begin out = 155; end
        115:begin out = 159; end
        116:begin out = 10; end
        117:begin out = 21; end
        118:begin out = 121; end
        119:begin out = 43; end
        120:begin out = 78; end
        121:begin out = 212; end
        122:begin out = 229; end
        123:begin out = 172; end
        124:begin out = 115; end
        125:begin out = 243; end
        126:begin out = 167; end
        127:begin out = 87; end
        128:begin out = 7; end
        129:begin out = 112; end
        130:begin out = 192; end
        131:begin out = 247; end
        132:begin out = 140; end
        133:begin out = 128; end
        134:begin out = 99; end
        135:begin out = 13; end
        136:begin out = 103; end
        137:begin out = 74; end
        138:begin out = 222; end
        139:begin out = 237; end
        140:begin out = 49; end
        141:begin out = 197; end
        142:begin out = 254; end
        143:begin out = 24; end
        144:begin out = 227; end
        145:begin out = 165; end
        146:begin out = 153; end
        147:begin out = 119; end
        148:begin out = 38; end
        149:begin out = 184; end
        150:begin out = 180; end
        151:begin out = 124; end
        152:begin out = 17; end
        153:begin out = 68; end
        154:begin out = 146; end
        155:begin out = 217; end
        156:begin out = 35; end
        157:begin out = 32; end
        158:begin out = 137; end
        159:begin out = 46; end
        160:begin out = 55; end
        161:begin out = 63; end
        162:begin out = 209; end
        163:begin out = 91; end
        164:begin out = 149; end
        165:begin out = 188; end
        166:begin out = 207; end
        167:begin out = 205; end
        168:begin out = 144; end
        169:begin out = 135; end
        170:begin out = 151; end
        171:begin out = 178; end
        172:begin out = 220; end
        173:begin out = 252; end
        174:begin out = 190; end
        175:begin out = 97; end
        176:begin out = 242; end
        177:begin out = 86; end
        178:begin out = 211; end
        179:begin out = 171; end
        180:begin out = 20; end
        181:begin out = 42; end
        182:begin out = 93; end
        183:begin out = 158; end
        184:begin out = 132; end
        185:begin out = 60; end
        186:begin out = 57; end
        187:begin out = 83; end
        188:begin out = 71; end
        189:begin out = 109; end
        190:begin out = 65; end
        191:begin out = 162; end
        192:begin out = 31; end
        193:begin out = 45; end
        194:begin out = 67; end
        195:begin out = 216; end
        196:begin out = 183; end
        197:begin out = 123; end
        198:begin out = 164; end
        199:begin out = 118; end
        200:begin out = 196; end
        201:begin out = 23; end
        202:begin out = 73; end
        203:begin out = 236; end
        204:begin out = 127; end
        205:begin out = 12; end
        206:begin out = 111; end
        207:begin out = 246; end
        208:begin out = 108; end
        209:begin out = 161; end
        210:begin out = 59; end
        211:begin out = 82; end
        212:begin out = 41; end
        213:begin out = 157; end
        214:begin out = 85; end
        215:begin out = 170; end
        216:begin out = 251; end
        217:begin out = 96; end
        218:begin out = 134; end
        219:begin out = 177; end
        220:begin out = 187; end
        221:begin out = 204; end
        222:begin out = 62; end
        223:begin out = 90; end
        224:begin out = 203; end
        225:begin out = 89; end
        226:begin out = 95; end
        227:begin out = 176; end
        228:begin out = 156; end
        229:begin out = 169; end
        230:begin out = 160; end
        231:begin out = 81; end
        232:begin out = 11; end
        233:begin out = 245; end
        234:begin out = 22; end
        235:begin out = 235; end
        236:begin out = 122; end
        237:begin out = 117; end
        238:begin out = 44; end
        239:begin out = 215; end
        240:begin out = 79; end
        241:begin out = 174; end
        242:begin out = 213; end
        243:begin out = 233; end
        244:begin out = 230; end
        245:begin out = 231; end
        246:begin out = 173; end
        247:begin out = 232; end
        248:begin out = 116; end
        249:begin out = 214; end
        250:begin out = 244; end
        251:begin out = 234; end
        252:begin out = 168; end
        253:begin out = 80; end
        254:begin out = 88; end
        255:begin out = 175; end
        default:begin
            out = 0;
        end
    endcase
end

endmodule 

module antilog(
input [7:0] in,
output reg [7:0] out
);
always@(*)begin
    (* synthesis, parallel_case *)
    case(in)
        0:begin out = 1; end
        1:begin out = 2; end
        2:begin out = 4; end
        3:begin out = 8; end
        4:begin out = 16; end
        5:begin out = 32; end
        6:begin out = 64; end
        7:begin out = 128; end
        8:begin out = 29; end
        9:begin out = 58; end
        10:begin out = 116; end
        11:begin out = 232; end
        12:begin out = 205; end
        13:begin out = 135; end
        14:begin out = 19; end
        15:begin out = 38; end
        16:begin out = 76; end
        17:begin out = 152; end
        18:begin out = 45; end
        19:begin out = 90; end
        20:begin out = 180; end
        21:begin out = 117; end
        22:begin out = 234; end
        23:begin out = 201; end
        24:begin out = 143; end
        25:begin out = 3; end
        26:begin out = 6; end
        27:begin out = 12; end
        28:begin out = 24; end
        29:begin out = 48; end
        30:begin out = 96; end
        31:begin out = 192; end
        32:begin out = 157; end
        33:begin out = 39; end
        34:begin out = 78; end
        35:begin out = 156; end
        36:begin out = 37; end
        37:begin out = 74; end
        38:begin out = 148; end
        39:begin out = 53; end
        40:begin out = 106; end
        41:begin out = 212; end
        42:begin out = 181; end
        43:begin out = 119; end
        44:begin out = 238; end
        45:begin out = 193; end
        46:begin out = 159; end
        47:begin out = 35; end
        48:begin out = 70; end
        49:begin out = 140; end
        50:begin out = 5; end
        51:begin out = 10; end
        52:begin out = 20; end
        53:begin out = 40; end
        54:begin out = 80; end
        55:begin out = 160; end
        56:begin out = 93; end
        57:begin out = 186; end
        58:begin out = 105; end
        59:begin out = 210; end
        60:begin out = 185; end
        61:begin out = 111; end
        62:begin out = 222; end
        63:begin out = 161; end
        64:begin out = 95; end
        65:begin out = 190; end
        66:begin out = 97; end
        67:begin out = 194; end
        68:begin out = 153; end
        69:begin out = 47; end
        70:begin out = 94; end
        71:begin out = 188; end
        72:begin out = 101; end
        73:begin out = 202; end
        74:begin out = 137; end
        75:begin out = 15; end
        76:begin out = 30; end
        77:begin out = 60; end
        78:begin out = 120; end
        79:begin out = 240; end
        80:begin out = 253; end
        81:begin out = 231; end
        82:begin out = 211; end
        83:begin out = 187; end
        84:begin out = 107; end
        85:begin out = 214; end
        86:begin out = 177; end
        87:begin out = 127; end
        88:begin out = 254; end
        89:begin out = 225; end
        90:begin out = 223; end
        91:begin out = 163; end
        92:begin out = 91; end
        93:begin out = 182; end
        94:begin out = 113; end
        95:begin out = 226; end
        96:begin out = 217; end
        97:begin out = 175; end
        98:begin out = 67; end
        99:begin out = 134; end
        100:begin out = 17; end
        101:begin out = 34; end
        102:begin out = 68; end
        103:begin out = 136; end
        104:begin out = 13; end
        105:begin out = 26; end
        106:begin out = 52; end
        107:begin out = 104; end
        108:begin out = 208; end
        109:begin out = 189; end
        110:begin out = 103; end
        111:begin out = 206; end
        112:begin out = 129; end
        113:begin out = 31; end
        114:begin out = 62; end
        115:begin out = 124; end
        116:begin out = 248; end
        117:begin out = 237; end
        118:begin out = 199; end
        119:begin out = 147; end
        120:begin out = 59; end
        121:begin out = 118; end
        122:begin out = 236; end
        123:begin out = 197; end
        124:begin out = 151; end
        125:begin out = 51; end
        126:begin out = 102; end
        127:begin out = 204; end
        128:begin out = 133; end
        129:begin out = 23; end
        130:begin out = 46; end
        131:begin out = 92; end
        132:begin out = 184; end
        133:begin out = 109; end
        134:begin out = 218; end
        135:begin out = 169; end
        136:begin out = 79; end
        137:begin out = 158; end
        138:begin out = 33; end
        139:begin out = 66; end
        140:begin out = 132; end
        141:begin out = 21; end
        142:begin out = 42; end
        143:begin out = 84; end
        144:begin out = 168; end
        145:begin out = 77; end
        146:begin out = 154; end
        147:begin out = 41; end
        148:begin out = 82; end
        149:begin out = 164; end
        150:begin out = 85; end
        151:begin out = 170; end
        152:begin out = 73; end
        153:begin out = 146; end
        154:begin out = 57; end
        155:begin out = 114; end
        156:begin out = 228; end
        157:begin out = 213; end
        158:begin out = 183; end
        159:begin out = 115; end
        160:begin out = 230; end
        161:begin out = 209; end
        162:begin out = 191; end
        163:begin out = 99; end
        164:begin out = 198; end
        165:begin out = 145; end
        166:begin out = 63; end
        167:begin out = 126; end
        168:begin out = 252; end
        169:begin out = 229; end
        170:begin out = 215; end
        171:begin out = 179; end
        172:begin out = 123; end
        173:begin out = 246; end
        174:begin out = 241; end
        175:begin out = 255; end
        176:begin out = 227; end
        177:begin out = 219; end
        178:begin out = 171; end
        179:begin out = 75; end
        180:begin out = 150; end
        181:begin out = 49; end
        182:begin out = 98; end
        183:begin out = 196; end
        184:begin out = 149; end
        185:begin out = 55; end
        186:begin out = 110; end
        187:begin out = 220; end
        188:begin out = 165; end
        189:begin out = 87; end
        190:begin out = 174; end
        191:begin out = 65; end
        192:begin out = 130; end
        193:begin out = 25; end
        194:begin out = 50; end
        195:begin out = 100; end
        196:begin out = 200; end
        197:begin out = 141; end
        198:begin out = 7; end
        199:begin out = 14; end
        200:begin out = 28; end
        201:begin out = 56; end
        202:begin out = 112; end
        203:begin out = 224; end
        204:begin out = 221; end
        205:begin out = 167; end
        206:begin out = 83; end
        207:begin out = 166; end
        208:begin out = 81; end
        209:begin out = 162; end
        210:begin out = 89; end
        211:begin out = 178; end
        212:begin out = 121; end
        213:begin out = 242; end
        214:begin out = 249; end
        215:begin out = 239; end
        216:begin out = 195; end
        217:begin out = 155; end
        218:begin out = 43; end
        219:begin out = 86; end
        220:begin out = 172; end
        221:begin out = 69; end
        222:begin out = 138; end
        223:begin out = 9; end
        224:begin out = 18; end
        225:begin out = 36; end
        226:begin out = 72; end
        227:begin out = 144; end
        228:begin out = 61; end
        229:begin out = 122; end
        230:begin out = 244; end
        231:begin out = 245; end
        232:begin out = 247; end
        233:begin out = 243; end
        234:begin out = 251; end
        235:begin out = 235; end
        236:begin out = 203; end
        237:begin out = 139; end
        238:begin out = 11; end
        239:begin out = 22; end
        240:begin out = 44; end
        241:begin out = 88; end
        242:begin out = 176; end
        243:begin out = 125; end
        244:begin out = 250; end
        245:begin out = 233; end
        246:begin out = 207; end
        247:begin out = 131; end
        248:begin out = 27; end
        249:begin out = 54; end
        250:begin out = 108; end
        251:begin out = 216; end
        252:begin out = 173; end
        253:begin out = 71; end
        254:begin out = 142; end
        default:begin
            out = 0;
        end
    endcase
end
endmodule


module error_position(
input [7:0] sigma_1,
input [7:0] sigma_2,
input [7:0] sigma_3,
input [7:0] sigma_4,
input clk,
input srstn,
input enable,

output[7:0] i1,
output[7:0] i2,
output[7:0] i3,
output[7:0] i4,
output reg finish,
output reg [2:0] error_count 

);

localparam [3:0] IDLE = 0;
localparam [3:0] TRY1 = 1;
localparam [3:0] TRY2 = 2;
localparam [3:0] TRY3 = 3;
localparam [3:0] TRY4 = 4;
localparam [3:0] TRY5 = 5;
localparam [3:0] CHECK = 6;
localparam [3:0] FINISH = 7;



reg [3:0] state, state_n;
reg [7:0] try_i, try_i_n;
reg [7:0] try_i_2, try_i_2_n;
reg [7:0] det, det_n;

reg [7:0] anti_in; 
wire [7:0]anti_out;

reg [7:0] result [0:3];
reg [7:0] result_n [3:0];

reg [2:0] error_count_n;

antilog anti1 (.in(anti_in),.out(anti_out));

always@(posedge clk)begin
    if(~srstn)begin
        state <= 0;
    end
    else begin
        state <= state_n;
    end
end
always@(posedge clk)begin
    try_i <= try_i_n;
    try_i_2 <= try_i_2_n;
    det <= det_n;
    result[0] <= result_n[0];
    result[1] <= result_n[1];
    result[2] <= result_n[2];
    result[3] <= result_n[3];
    error_count <= error_count_n;
end

assign i1 = result[0];
assign i2 = result[1];
assign i3 = result[2];
assign i4 = result[3];

always@(*)begin
    if(state == CHECK)begin
        if(det == 0)begin
            result_n[3] = result[2];
            result_n[2] = result[1];
            result_n[1] = result[0];
            result_n[0] = try_i;
        end
        else begin
            result_n[0] = result[0];
            result_n[1] = result[1];
            result_n[2] = result[2];
            result_n[3] = result[3];
        end
    end
    else begin
        result_n[0] = result[0];
        result_n[1] = result[1];
        result_n[2] = result[2];
        result_n[3] = result[3];
    end
end

always@(*)begin
    if(state == IDLE)begin
        error_count_n = 0 ;
    end
    else if (state == CHECK)begin
        error_count_n = (det == 0)? error_count + 1 : error_count;
    end
    else begin
        error_count_n = error_count;
    end
end

always@(*)begin
    case(state)
        IDLE:begin
            state_n = (enable)? TRY1 : IDLE;
            anti_in = 0;
            try_i_n = 43;
            try_i_2_n = 43;
            det_n = 0;
            finish = 0;
        end
        TRY1:begin
            state_n = TRY2;
            anti_in = sigma_4;
            try_i_n = try_i;
            try_i_2_n = try_i;
            det_n = anti_out;
            finish = 0;
        end
        TRY2:begin
            state_n = TRY3;
            anti_in = ({1'b0, sigma_3} + {1'b0, try_i_2} >= 255)? sigma_3 + try_i_2 + 1 : sigma_3 + try_i_2  ;
            try_i_n = try_i;
            try_i_2_n = try_i_2 + try_i;
            det_n = det ^ anti_out;
            finish = 0;
        end
        TRY3:begin
            state_n = TRY4;
            anti_in = ({1'b0, sigma_2} + {1'b0, try_i_2} >= 255)? sigma_2 + try_i_2 + 1 : sigma_2 + try_i_2  ;
            try_i_n = try_i;
            try_i_2_n = try_i_2 + try_i;
            det_n = det ^ anti_out;
            finish = 0;
        end
        TRY4:begin
            state_n = TRY5;
            anti_in = ({1'b0, sigma_1} + {1'b0, try_i_2} >= 255)? sigma_1 + try_i_2 + 1 : sigma_1 + try_i_2  ;
            try_i_n = try_i;
            try_i_2_n = try_i_2 + try_i;
            det_n = det ^ anti_out;
            finish = 0;
        end
        TRY5:begin
            state_n = CHECK;
            anti_in = try_i_2;
            try_i_n = try_i;
            try_i_2_n = try_i_2;
            det_n = det ^ anti_out;
            finish = 0;
        end
        CHECK:begin
            state_n = (try_i == 16) ? FINISH : TRY2;
            anti_in = sigma_4;
            try_i_n = try_i - 1;
            try_i_2_n = try_i - 1;
            det_n = anti_out;
            finish = 0;
        end
        FINISH:begin
            state_n = FINISH;
            anti_in = 0;
            try_i_n = try_i;
            try_i_2_n = try_i_2;
            det_n = 0;
            finish = 1;
        end
        default:begin
            state_n = 0;
            anti_in = 0;
            try_i_n = try_i;
            try_i_2_n = try_i_2;
            det_n = 0;
            finish = 0;
        end
    endcase
end




endmodule



















