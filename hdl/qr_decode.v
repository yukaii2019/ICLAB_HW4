module qr_decode(
input clk,                           //clock input
input srstn,                         //synchronous reset (active low)
input qr_decode_start,               //start decoding for one QR code
                                     //1: start (one-cycle pulse)
input sram_rdata,                    //read data from SRAM
output reg [11:0] sram_raddr,        //read address to SRAM

output decode_valid,                 //decoded code is valid
output [7:0] decode_jis8_code,       //decoded JIS8 code
output qr_decode_finish              //1: decoding one QR code is finished
);

localparam [3:0] IDLE = 0;
localparam [3:0] DETECT_POSITION_AND_ROTATION = 1;
localparam [3:0] FIND_MASK_PATTERN = 2;
localparam [3:0] DE_MASKING_1 = 3;
localparam [3:0] DE_MASKING_2 = 4;
localparam [3:0] DE_MASKING_3 = 5;
localparam [3:0] TMP = 6;

wire enable;
wire pos_rot_finish;
wire [2:0] mode;
wire [11:0] position;

assign qr_decode_finish = 0;
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

assign real_mask = mask ^ 3'b010;

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

assign real_x_cnt = (mode == 1) ? x_cnt : (mode == 2)? y_cnt : (mode == 3)? 24-x_cnt : 24-y_cnt; 
assign real_y_cnt = (mode == 1) ? y_cnt : (mode == 2)? 24-x_cnt : (mode == 3)? 24-y_cnt : x_cnt; 



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
        end
        DE_MASKING_1:begin    
            state_n = (x_cnt == 16 && y_cnt == 8)? DE_MASKING_2 :DE_MASKING_1;
            mask_raddr = position + 64*real_y_cnt + real_x_cnt;
            mask_cnt_n = 0;
            code_n = sram_rdata;
            x_cnt_n = (x_cnt == 16)? (y_cnt == 8)? 0  : 9 : x_cnt + 1;
            y_cnt_n = (x_cnt == 16)? y_cnt + 1 : y_cnt;
        end
        DE_MASKING_2:begin
            state_n = (x_cnt == 24 &&  y_cnt == 16)? DE_MASKING_3 : DE_MASKING_2;
            mask_raddr = position + 64*real_y_cnt + real_x_cnt;
            mask_cnt_n = 0;
            code_n = sram_rdata;
            x_cnt_n = (x_cnt == 24)? (y_cnt == 16)? 9 : 0 : x_cnt + 1;
            y_cnt_n = (x_cnt == 24)? y_cnt + 1 : y_cnt;
        end
        DE_MASKING_3:begin
            state_n = (x_cnt == 24 && y_cnt == 24)? TMP : DE_MASKING_3;
            mask_raddr = position + 64*real_y_cnt + real_x_cnt;
            mask_cnt_n = 0;
            code_n = sram_rdata;
            x_cnt_n = (x_cnt == 24)? 9 :x_cnt + 1;
            y_cnt_n = (x_cnt == 24)? y_cnt + 1:y_cnt;
        end
        TMP:begin
            state_n = TMP;
            mask_raddr = 0;
            mask_cnt_n = 0;
            code_n = 0;
            x_cnt_n = 0;
            y_cnt_n = 0;

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
