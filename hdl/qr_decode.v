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
localparam [3:0] OUTPUT = 6;
localparam [3:0] FINISH = 7;

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

//assign sram_raddr = pos_rot_sram_raddr;

reg [3:0] state;
reg [3:0] state_n;

reg [2:0] mask;
reg [2:0] mask_n;
wire [2:0] real_mask;

assign real_mask = mask ^ 3'b101;

reg [11:0] mask_raddr;
//reg [11:0] mask_raddr_n;

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
reg [8:0] out_cnt;
reg [8:0] out_cnt_n;

wire[7:0] c [0:44];
wire [7:0] length;
assign length = {c[0][3:0],c[1][7:4]};

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


wire [7:0] test_0 = c[6];
wire [7:0] test_1 = c[7];


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
   // mask_raddr <= mask_raddr_n;
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
end

always@(*)begin
    case(state)
        IDLE:begin
            state_n = (qr_decode_start)? DETECT_POSITION_AND_ROTATION:IDLE;
            //mask_raddr_n = 0;
            mask_raddr = 0;
            
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
        end
        DETECT_POSITION_AND_ROTATION:begin
            state_n = (pos_rot_finish) ? FIND_MASK_PATTERN : DETECT_POSITION_AND_ROTATION;
            //mask_raddr_n = (mode == 1 || mode == 4)? position+8*64+2: position + 16*64+22;
            mask_raddr = 0;
            mask_n = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
        end
        FIND_MASK_PATTERN:begin
            state_n = (mask_cnt == 2)? DE_MASKING_1 : FIND_MASK_PATTERN;
            //mask_raddr_n = (mask_cnt == 2)? position + 64*real_y_cnt + real_x_cnt :  (mode == 1 || mode == 4)? mask_raddr + 1 : mask_raddr -1;
            mask_raddr = (mode == 1 || mode == 4)? position+8*64+2 + mask_cnt : position + 16*64+22 - mask_cnt;
            
            mask_n = sram_rdata;
            mask_cnt_n = mask_cnt + 1;
            code_n = 0;
            x_cnt_n = 9;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
        end
        DE_MASKING_1:begin    
            state_n = (x_cnt == 16 && y_cnt == 8)? DE_MASKING_2 :DE_MASKING_1;
            mask_raddr = position + 64*real_y_cnt + real_x_cnt;
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
        end
        DE_MASKING_2:begin
            state_n = (x_cnt == 24 &&  y_cnt == 16)? DE_MASKING_3 : DE_MASKING_2;
            mask_raddr = position + 64*real_y_cnt + real_x_cnt;
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
        end
        DE_MASKING_3:begin
            state_n = (x_cnt == 24 && y_cnt == 24)? OUTPUT : DE_MASKING_3;
            mask_raddr = position + 64*real_y_cnt + real_x_cnt;
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
        end
        OUTPUT:begin
            state_n = (out_cnt == length-1)? FINISH : OUTPUT;
            mask_raddr = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = out_cnt + 1;
        end
        FINISH:begin
            state_n = FINISH;
            mask_raddr = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
            read_cnt_n = 0;
            out_cnt_n = 0;
        end
        default:begin
            state_n = 0;
            mask_raddr = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;
        end
    endcase
end

always@(*)begin
    decode_jis8_code_n = {c[out_cnt+1][3:0],c[out_cnt+2][7:4]};
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



wire [48:0] pattern = 49'b1111111_1000001_1011101_1011101_1011101_1000001_1111111;
wire is_pattern;
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


always@(*)begin
    case(state)
        IDLE:begin
            state_n = (enable)? FIND_FIRST_1_CODE : IDLE;      
            sram_raddr_n = 0; 
            x_cnt_n = 0;
            y_cnt_n = 0;
            buffer_n = 0;
            mode_n = 0;
            check_pattern_times_n = 0;
            position_n = 0;
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
                    default : mode = 0;
                endcase
            end
            else begin
                mode_n = (check_pattern_times != 2) ? 0:4;
            end

            check_pattern_times_n = check_pattern_times + 1;
            position_n = position;
        
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
        end
    endcase
end

endmodule
