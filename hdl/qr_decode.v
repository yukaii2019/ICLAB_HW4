module qr_decode(
input clk,                           //clock input
input srstn,                         //synchronous reset (active low)
input qr_decode_start,               //start decoding for one QR code
                                     //1: start (one-cycle pulse)
input sram_rdata,                    //read data from SRAM
output [11:0] sram_raddr,        //read address to SRAM

output decode_valid,                 //decoded code is valid
output [7:0] decode_jis8_code,       //decoded JIS8 code
output qr_decode_finish              //1: decoding one QR code is finished
);

wire enable;
wire finish;
wire [2:0] mode;
wire [11:0] position;

assign qr_decode_finish = 0;
assign enable = 1;


detect_rotation u1(
.clk(clk),
.srstn(srstn),
.enable(enable),
.sram_rdata(sram_rdata),
.sram_raddr(sram_raddr),
.mode(mode),
.position(position),
.finish(finish)
);



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

parameter [3:0] IDLE = 0;
parameter [3:0] FIND_FIRST_1_CODE = 1;
parameter [3:0] IDENTIFY_POSITION_DETECTION_PATTERN = 2;
parameter [3:0] COMPARE = 3;
parameter [3:0] CHECK_IF_180 = 4;
parameter [3:0] FINISH = 5;
parameter [3:0] FIND_POSITION = 6;


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
