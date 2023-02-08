
`timescale 1ns/10ps

module  CONV#(
            //================================
            //  PARAMETERS
            //================================
            parameter W  = 20,
            parameter B  = 8,
            parameter ADDR_W = 12,
            parameter MEM_SEL = 4,
            parameter CONV_IMG_W = 64,
            parameter MAX_POOLING_IMG_W = CONV_IMG_W / 2,
            parameter RELU_IMG_W        = CONV_IMG_W / 2,
            parameter FLATTEN_IMG_W = MAX_POOLING_IMG_W
        )(
            input					clk,
            input					reset,
            output reg				busy,
            input					ready,

            output reg[ADDR_W-1:0]		iaddr,
            input     [W-1:0]			idata,

            output reg			cwr,
            output reg[ADDR_W-1:0]	 		caddr_wr,
            output reg[W-1:0]	 	cdata_wr,

            output reg			crd,
            output reg[ADDR_W-1:0]	 	caddr_rd,
            input[W-1:0]	 		cdata_rd,

            output reg[MEM_SEL-1:0]	 	csel
        );
//================================
//  I/O
//================================
reg[W-1:0] data_buf;

//================================
//  CONTROLLER(CTR)
//================================
//LV1_FSM
reg[2:0] cur_state_LV1_ff;
reg[2:0] next_state_LV1;

//LV2_FSM
reg[2:0] cur_state_LV2_ff;
reg[2:0] next_state_LV2;

//KERNAL_STATUS
reg cur_kernal_ff;

//==================
//  stateS
//==================
//LV1_FSM
localparam IDLE = 3'd0;
localparam L0   = 3'd1;
localparam L1   = 3'd2;
localparam L2   = 3'd3;
localparam DONE = 3'd4;

wire state_IDLE     = cur_state_LV1_ff == IDLE;
wire state_L0       = cur_state_LV1_ff == L0;
wire state_L1       = cur_state_LV1_ff == L1;
wire state_L2       = cur_state_LV1_ff == L2;
wire state_DONE     = cur_state_LV1_ff == DONE;

//CURRENT KERNAL
localparam K0   = 1'b0;
localparam K1   = 1'b1;

wire state_KERNAL0       = cur_kernal_ff == K0;
wire state_KERNAL1       = cur_kernal_ff == K1;

//LV2_FSM
//_____________L0________________
//CONV
localparam RD_PIXEL_L0     = 3'd0;
localparam MAC             = 3'd1;
localparam ADD_BIAS        = 3'd2;
localparam ROUND           = 3'd3;

wire state_RD_PIXEL_L0   = (state_L0)  && (cur_state_LV2_ff == RD_PIXEL_L0) ;
wire state_MAC		 	 = (state_L0)  && (cur_state_LV2_ff == MAC) 		;
wire state_ADD_BIAS		 = (state_L0)  && (cur_state_LV2_ff == ADD_BIAS)    ;
wire state_ROUND		 = (state_L0)  && (cur_state_LV2_ff == ROUND)       ;

//RELU
localparam RELU            = 3'd4;
//WriteBack
localparam WB_L0           = 3'd5;

wire state_RELU              = (state_L0)  && (cur_state_LV2_ff == RELU)            ;
wire state_WB_L0		 	 = (state_L0)  && (cur_state_LV2_ff == WB_L0) 		   ;

//_____________L1________________
//MAXPOOLING
localparam RD_PIXEL_L1     = 3'd0;
localparam COMPARE_FOR_MAX = 3'd1;
//WriteBack
localparam WB_L1           = 3'd2;

wire state_RD_PIXEL_L1              = (state_L1)  && (cur_state_LV2_ff == RD_PIXEL_L0)            ;
wire state_COMPARE_FOR_MAX_L1		= (state_L1)  && (cur_state_LV2_ff == COMPARE_FOR_MAX) 		   ;
wire state_WB_L1		 	 	    = (state_L1)  && (cur_state_LV2_ff == WB_L1)        ;

//_____________L2________________
//Flatten
localparam RD_PIXEL_L2     = 3'd0;
//WriteBack
localparam FLATTEN_WB      = 3'd1;


wire state_RD_PIXEL_L2              = (state_L2)  && (cur_state_LV2_ff == RD_PIXEL_L2)            ;
wire state_FLATTEN_WB				= (state_L2)  && (cur_state_LV2_ff == FLATTEN_WB) 		   ;

//Exception
localparam EXCEPTION = 3'd7;
wire EXCEPTION_OCCURS = cur_state_LV1_ff == EXCEPTION || cur_state_LV2_ff == EXCEPTION || cur_kernal_ff == EXCEPTION;

//================================
//  Memory management Unit(MMU)
//================================
reg[ADDR_W-1:0] mem_addr_cnt; // MAX 4096

//================================
//  BUS
//================================
reg signed[W-1:0] common_data_bus;

//================================
//  Address Generation Unit(AGU)
//================================
localparam AGU_W = 8;
reg[AGU_W - 1:0] row_ptr;
reg[AGU_W - 1:0] col_ptr;

reg[AGU_W - 1:0] offset_row;
reg[AGU_W - 1:0] offset_col;

reg[4:0] offset_cnt;


//================================
//  Arithmetic Logic Unit(ALU)
//================================
//KERNAL0
localparam signed K0_00 =                20'h0A89E;  // Pixel 0:  6.586609e-01
localparam signed K0_01 =                20'h092D5;  // Pixel 1:  5.735626e-01
localparam signed K0_02 =                20'h06D43;  // Pixel 2:  4.268036e-01
localparam signed K0_10 =                20'h01004;  // Pixel 3:  6.256104e-02
localparam signed K0_11 =                20'hF8F71;  // Pixel 4: -4.396820e-01
localparam signed K0_12 =                20'hF6E54;  // Pixel 5: -5.690308e-01
localparam signed K0_20 =                20'hFA6D7;  // Pixel 6: -3.482819e-01
localparam signed K0_21 =                20'hFC834;  // Pixel 7: -2.179565e-01
localparam signed K0_22 =                20'hFAC19;  // Pixel 8: -3.277435e-01
//KERNAL1
localparam signed K1_00 = 20'hFDB55;  // Pixel 0: -1.432343e-01
localparam signed K1_01 = 20'h02992;  // Pixel 1:  1.623840e-01
localparam signed K1_02 = 20'hFC994;  // Pixel 2: -2.125854e-01
localparam signed K1_10 = 20'h050FD;  // Pixel 3:  3.163605e-01
localparam signed K1_11 = 20'h02F20;  // Pixel 4:  1.840820e-01
localparam signed K1_12 = 20'h0202D;  // Pixel 5:  1.256866e-01
localparam signed K1_20 = 20'h03BD7;  // Pixel 6:  2.337494e-01
localparam signed K1_21 = 20'hFD369;  // Pixel 7: -1.741791e-01
localparam signed K1_22 = 20'h05E68;   // Pixel 8:  3.687744e-01

//ADD_BIAS
localparam signed [W-1:0] CNNBIAS0 = 20'h01310;  // Pixel 0: 7.446289e-02
localparam signed [W-1:0] CNNBIAS1 = 20'hF7295;  // Pixel 1: -5.524139e-01

reg signed[(2*W+4)-1:0] alu_out_ff;
reg signed[W-1:0]       result_temp_ff;

//================================
//  CONTROL FLAGS
//================================
wire L0_right_bound_reach_flag = (col_ptr == (CONV_IMG_W)) ;
wire L0_bottom_reach_flag = (row_ptr == (CONV_IMG_W));

wire L0_done_flag = conv_relu_done_flag && state_KERNAL1 && state_L0;
wire L1_done_flag = (row_ptr == (MAX_POOLING_IMG_W-1)) && (col_ptr == (MAX_POOLING_IMG_W - 1)) && state_KERNAL1 && state_L1 ;
wire L2_done_flag = (row_ptr == (FLATTEN_IMG_W-1)) && (state_KERNAL1) && state_L2;

wire L1_right_bound_reach_flag = (col_ptr == (MAX_POOLING_IMG_W-1));
wire L1_bottom_reach_flag = (row_ptr == (MAX_POOLING_IMG_W-1));
wire max_pooling_done_flag = L1_bottom_reach_flag && L1_right_bound_reach_flag;

wire mac_done_flag = state_MAC && (offset_cnt == 'd8);
wire conv_relu_done_flag = L0_bottom_reach_flag && L0_right_bound_reach_flag && state_WB_L0;
wire local_max_found_flag = state_COMPARE_FOR_MAX_L1 && (offset_cnt == 'd3);

wire flatten_done_flag = (row_ptr == (FLATTEN_IMG_W - 1)) && (col_ptr == (FLATTEN_IMG_W - 1));

wire zero_pad_flag = state_MAC && (offset_row == 0) || (offset_col == 0) || (offset_row == 65) || (offset_col == 65);

//================================================================
//  MAIN DESIGN
//================================================================
//================================
//  I/O
//================================
//data buffer
always @(posedge clk or posedge reset)
begin: DATA_BUFFER
    if(reset)
    begin
        data_buf <=  'd0;
    end
    else if(state_RD_PIXEL_L0 || state_MAC)
    begin
        data_buf <=  idata;
    end
    else if(state_RD_PIXEL_L1 || state_RD_PIXEL_L2)
    begin
        data_buf <=  cdata_rd;
    end
    else
    begin
        data_buf <=  data_buf;
    end
end

//busy
always @(posedge clk or posedge reset)
begin
    if(reset)
    begin
        busy <= 1'b0;
    end
    else if(state_DONE || state_IDLE)
    begin
        busy <= 1'b0;
    end
    else
    begin
        busy <= 1'b1;
    end
end

//================================
//  CONTROLLER(CTR)
//================================
//LV1_FSM
always @(posedge clk or posedge reset)
begin: LV1_FSM_CUR
    if(reset)
    begin
        cur_state_LV1_ff <=  IDLE ;
    end
    else
    begin
        cur_state_LV1_ff <=  next_state_LV1;
    end
end

always @(*)
begin: LV1_FSM_NEXT
    case(cur_state_LV1_ff)
        IDLE:
        begin
            next_state_LV1 = ready 		  ? L0 : IDLE;
        end
        L0:
        begin
            next_state_LV1 = L0_done_flag  ? L1 : L0;
        end
        L1:
        begin
            next_state_LV1 = L1_done_flag  ? L2 : L1;
        end
        L2:
        begin
            next_state_LV1 = L2_done_flag  ? DONE:L2;
        end
        DONE:
        begin
            next_state_LV1 = DONE;
        end
        default:
        begin
            next_state_LV1 = EXCEPTION;
        end
    endcase
end

//LV2_FSM
always @(posedge clk or posedge reset)
begin: LV2_FSM_CUR
    if(reset)
    begin
        cur_state_LV2_ff <=  RD_PIXEL_L0;
    end
    else
    begin
        cur_state_LV2_ff <=  next_state_LV2;
    end
end


always @(*)
begin: LV2_FSM_NEXT
    case(cur_state_LV1_ff)
        L0:
        begin
            case(cur_state_LV2_ff)
                RD_PIXEL_L0:
                begin
                    next_state_LV2 = MAC;
                end
                MAC:
                begin
                    next_state_LV2 = mac_done_flag ? ROUND : RD_PIXEL_L0;
                end
                ADD_BIAS:
                begin
                    next_state_LV2 = RELU;
                end
                ROUND:
                begin
                    next_state_LV2 = ADD_BIAS;
                end
                RELU:
                begin
                    next_state_LV2 = WB_L0;
                end
                WB_L0:
                begin
                    next_state_LV2 = conv_relu_done_flag ? (L0_done_flag ? RD_PIXEL_L1: RD_PIXEL_L0) : RD_PIXEL_L0;
                end
                default:
                begin
                    next_state_LV2 = RD_PIXEL_L0;
                end
            endcase
        end
        L1:
        begin
            case(cur_state_LV2_ff)
                RD_PIXEL_L1:
                begin
                    next_state_LV2 = COMPARE_FOR_MAX;
                end
                COMPARE_FOR_MAX:
                begin
                    next_state_LV2 = local_max_found_flag ? WB_L1 : COMPARE_FOR_MAX;
                end
                WB_L1:
                begin
                    next_state_LV2 = max_pooling_done_flag ? (L1_done_flag ? RD_PIXEL_L2 : RD_PIXEL_L1) : RD_PIXEL_L1;
                end
                default:
                begin
                    next_state_LV2 = RD_PIXEL_L1;
                end
            endcase
        end
        L2:
        begin
            case(cur_state_LV2_ff)
                RD_PIXEL_L2:
                begin
                    next_state_LV2 = FLATTEN_WB ;
                end
                FLATTEN_WB:
                begin
                    next_state_LV2 = L2_done_flag ? RD_PIXEL_L2 : RD_PIXEL_L2;
                end
                default:
                begin
                    next_state_LV2 = RD_PIXEL_L2;
                end
            endcase
        end
        default:
        begin
            next_state_LV2 = EXCEPTION;
        end
    endcase
end

// current kernal status register
always @(posedge clk or posedge reset)
begin: CURRENT_KERNAL
    if(reset)
    begin
        cur_kernal_ff <=  K0;
    end
    else
    begin
        case(cur_state_LV1_ff)
            L0:
            begin
                cur_kernal_ff <=  conv_relu_done_flag ? ~cur_kernal_ff : cur_kernal_ff;
            end
            L1:
            begin
                cur_kernal_ff <=  max_pooling_done_flag ? ~cur_kernal_ff : cur_kernal_ff;
            end
            L2:
            begin
                cur_kernal_ff <=  flatten_done_flag     ? ~cur_kernal_ff : cur_kernal_ff;
            end
            default:
            begin
                cur_kernal_ff <= cur_kernal_ff;
            end
        endcase
    end
end

//================================
//  BUS
//================================
always @(*)
begin:BUS
    if(state_RD_PIXEL_L0 || state_RD_PIXEL_L1 || state_RD_PIXEL_L2 || state_MAC || state_COMPARE_FOR_MAX_L1 || state_FLATTEN_WB)
    begin
        common_data_bus = data_buf;
    end
    else if(state_WB_L0 || state_WB_L1)
    begin
        common_data_bus = alu_out_ff[19:0];
    end
    else
    begin
        common_data_bus = 'd404;
    end
end

//================================
//  Address Generation Unit(AGU)
//================================
always @(posedge clk or posedge reset)
begin: PTRS
    if(reset)
    begin
        row_ptr <=  'd1;
        col_ptr <=  'd1;
    end
    else if(L0_done_flag || L1_done_flag)
    begin
        row_ptr <=  'd0;
        col_ptr <=  'd0;
    end
    else if(state_WB_L0)
    begin
        if(conv_relu_done_flag)
        begin
            row_ptr <=  'd1;
            col_ptr <=  'd1;
        end
        else
        begin
            row_ptr <=  L0_right_bound_reach_flag ? row_ptr + 1 : row_ptr;
            col_ptr <=  L0_right_bound_reach_flag ? 1 : col_ptr + 1;
        end
    end
    else if(state_WB_L1)
    begin
        row_ptr <=  L1_right_bound_reach_flag ? row_ptr + 1 : row_ptr;
        col_ptr <=  L1_right_bound_reach_flag ? 0 : col_ptr + 1;
    end
    else
    begin
        row_ptr <=  row_ptr;
        col_ptr <=  col_ptr;
    end
end

always @(*)
begin: OFFSETED_PTRS
    case(cur_state_LV1_ff)
        L0:
        begin
            case(offset_cnt)
                'd0:
                begin
                    offset_col = col_ptr -1;
                    offset_row = row_ptr -1;
                end
                'd1:
                begin
                    offset_col = col_ptr;
                    offset_row = row_ptr-1;
                end
                'd2:
                begin
                    offset_col = col_ptr + 1;
                    offset_row = row_ptr -1;
                end
                'd3:
                begin
                    offset_col = col_ptr-1;
                    offset_row = row_ptr;
                end
                'd4:
                begin
                    offset_col = col_ptr;
                    offset_row = row_ptr;
                end
                'd5:
                begin
                    offset_col = col_ptr+1;
                    offset_row = row_ptr;
                end
                'd6:
                begin
                    offset_col = col_ptr-1;
                    offset_row = row_ptr+1;
                end
                'd7:
                begin
                    offset_col = col_ptr;
                    offset_row = row_ptr+1;
                end
                'd8:
                begin
                    offset_col = col_ptr+1;
                    offset_row = row_ptr+1;
                end
                default:
                begin
                    offset_col = col_ptr;
                    offset_row = row_ptr;
                end
            endcase
        end
        L1:
        begin
            case(offset_cnt)
                'd0:
                begin
                    offset_col = col_ptr;
                    offset_row = row_ptr;
                end
                'd1:
                begin
                    offset_col = col_ptr+1;
                    offset_row = row_ptr;
                end
                'd2:
                begin
                    offset_col = col_ptr;
                    offset_row = row_ptr+1;
                end
                'd3:
                begin
                    offset_col = col_ptr+1;
                    offset_row = row_ptr+1;
                end
                default:
                begin
                    offset_col = col_ptr;
                    offset_row = row_ptr;
                end
            endcase
        end
        default:
        begin
            offset_col = col_ptr;
            offset_row = row_ptr;
        end
    endcase
end

always @(posedge clk or posedge reset)
begin: OFFSET_CNT
    if(reset)
    begin
        offset_cnt <= 'd0;
    end
    else if(state_MAC)
    begin
        offset_cnt <= mac_done_flag ? 0 : offset_cnt + 1;
    end
    else if(state_COMPARE_FOR_MAX_L1)
    begin
        offset_cnt <= local_max_found_flag ? 0 : offset_cnt + 1;
    end
    else
    begin
        offset_cnt <= offset_cnt;
    end
end

//================================
//  Memory management Unit(MMU)
//================================
//MEM_ACCESS
localparam NO_ACCESS= 3'b000;
localparam L0_MEM0_ACCESS= 3'b001;
localparam L0_MEM1_ACCESS= 3'b010;
localparam L1_MEM0_ACCESS= 3'b011;
localparam L1_MEM1_ACCESS= 3'b100;
localparam L2_MEM_ACCESS= 3'b101;

//csel
always @(*)
begin: CHIP_SELECT
    if(state_WB_L0 || state_RD_PIXEL_L1 || state_COMPARE_FOR_MAX_L1)
    begin
        csel = state_KERNAL0 ? L0_MEM0_ACCESS: L0_MEM1_ACCESS;
    end
    else if(state_WB_L1 || state_RD_PIXEL_L2)
    begin
        csel = state_KERNAL0 ? L1_MEM0_ACCESS: L1_MEM1_ACCESS ;
    end
    else if(state_FLATTEN_WB)
    begin
        csel = L2_MEM_ACCESS;
    end
end

//iaddr
always @(*)
begin: IN_ADDR
    if(state_RD_PIXEL_L0 || state_MAC)
    begin
        iaddr = (offset_col-1) % CONV_IMG_W + (offset_row-1)* CONV_IMG_W;
    end
    else
    begin
        iaddr = 'd404;
    end
end

//crd, caddr_rd
always @(*)
begin: CHIP_RD
    if(state_RD_PIXEL_L1||state_RD_PIXEL_L2 || state_COMPARE_FOR_MAX_L1)
    begin
        crd = 1;
        caddr_rd = offset_col + offset_row * MAX_POOLING_IMG_W ;
    end
    else
    begin
        crd = 0;
        caddr_rd = 'd0;
    end
end

//cwr,cdata_wr,caddr_wr
always @(*)
begin: CHIP_WR
    if(state_WB_L0)
    begin
        cwr = 1;
        caddr_wr = (col_ptr-1) + (row_ptr-1) * CONV_IMG_W ;
        cdata_wr = common_data_bus;
    end
    else if(state_WB_L1)
    begin
        cwr = 1;
        caddr_wr = col_ptr + row_ptr * MAX_POOLING_IMG_W;
        cdata_wr = common_data_bus;
    end
    else if(state_FLATTEN_WB)
    begin
        cwr = 1;
        caddr_wr = mem_addr_cnt;
        cdata_wr = common_data_bus;
    end
    else
    begin
        cwr = 0;
        caddr_wr = 0;
        cdata_wr = 0;
    end
end

wire signed[W-1:0] rounded_value = alu_out_ff[35:16];
wire[2*W+3 : 0] twos_complement = alu_out_ff[43] ? (~alu_out_ff + 'b1) : alu_out_ff;

//================================
//  Arithmetic Logic Unit(ALU)
//================================
always @(posedge clk or posedge reset)
begin: ALU
    if(reset)
    begin
        alu_out_ff <=  'd0;
    end
    else if(state_MAC)
    begin
        if(zero_pad_flag)
        begin
            alu_out_ff <= alu_out_ff;
        end
        else
        begin
            case(offset_cnt)
                'd0:
                begin
                    alu_out_ff <= state_KERNAL0 ? ((common_data_bus * K0_00) + (alu_out_ff)) :
                        ((common_data_bus * K1_00) + (alu_out_ff));
                end
                'd1:
                begin
                    alu_out_ff <= state_KERNAL0 ? ((common_data_bus * K0_01) + (alu_out_ff)) :
                        ((common_data_bus * K1_01) + (alu_out_ff));
                end
                'd2:
                begin
                    alu_out_ff <= state_KERNAL0 ? ((common_data_bus * K0_02) + (alu_out_ff)) :
                        ((common_data_bus * K1_02) + (alu_out_ff));
                end
                'd3:
                begin
                    alu_out_ff <= state_KERNAL0 ? ((common_data_bus * K0_10) + (alu_out_ff)) :
                        ((common_data_bus * K1_10) + (alu_out_ff));
                end
                'd4:
                begin
                    alu_out_ff <= state_KERNAL0 ? ((common_data_bus * K0_11) + (alu_out_ff)) :
                        ((common_data_bus * K1_11) + (alu_out_ff));
                end
                'd5:
                begin
                    alu_out_ff <= state_KERNAL0 ? ((common_data_bus * K0_12) + (alu_out_ff)) :
                        ((common_data_bus * K1_12) + (alu_out_ff));
                end
                'd6:
                begin
                    alu_out_ff <= state_KERNAL0 ? ((common_data_bus * K0_20) + (alu_out_ff)) :
                        ((common_data_bus * K1_20) + (alu_out_ff));
                end
                'd7:
                begin
                    alu_out_ff <= state_KERNAL0 ? ((common_data_bus * K0_21) + (alu_out_ff)) :
                        ((common_data_bus * K1_21) + (alu_out_ff));
                end
                'd8:
                begin
                    alu_out_ff <= state_KERNAL0 ? ((common_data_bus * K0_22) + (alu_out_ff)) :
                        ((common_data_bus * K1_22) + (alu_out_ff));
                end
                default:
                begin
                    alu_out_ff <=  alu_out_ff;
                end
            endcase
        end
    end
    else if(state_ADD_BIAS)
    begin
        alu_out_ff <=  state_KERNAL0 ? alu_out_ff + $signed(CNNBIAS0)
                   : alu_out_ff + $signed(CNNBIAS1);
        // bias also needs to be concatenated.
    end
    else if(state_ROUND)
    begin
        // alu_out_ff <=  alu_out_ff[43] ? ~(twos_complement + {28'b0,twos_complement[15],15'b0}) + 1'b1 :
        //  (twos_complement + {28'b0,twos_complement[15],15'b0}); //When rounding must concatenate!
        alu_out_ff <= { {24{alu_out_ff[43]}}, ((alu_out_ff[35:15] + $signed({20'b0,(alu_out_ff[15])})) >> 1)} ; //When rounding must concatenate!
    end
    else if(state_RELU)
    begin
        //  alu_out_ff <=  (rounded_value[19]) ? 'd0 : alu_out_ff;
        alu_out_ff <=  (rounded_value >= $signed('d0)) ? alu_out_ff : 'd0;
    end
    else if(state_COMPARE_FOR_MAX_L1)
    begin
        alu_out_ff <=  ((common_data_bus) > (alu_out_ff)) ? common_data_bus : alu_out_ff;
    end
    else
    begin
        alu_out_ff <=  alu_out_ff;
    end
end

endmodule
