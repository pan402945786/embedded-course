`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:02:55 11/01/2016 
// Design Name: 
// Module Name:    fpga 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`include "register.v"
`define  DELAY            25'd24000000
module fpga(
	//--------------	Clock Input ----------------------------------------------//
//	input			      CLOCK_48,				 //	24 MHz
	//--------------	LED ------------------------------------------------------//
	output	[3:0]	   LED,					    //	LED [3:0]
	//--------------  SDRAM Interface ------------------------------------------//
//	inout	   [15:0]	SDRAM_DQ,				 //	SDRAM Data bus 16 Bits
//	output	[12:0]	SDRAM_ADDR,				 //	SDRAM Address bus 13 Bits
//	output	[1:0]	   SDRAM_DQM,				 //	SDRAM Data Mask 
//	output			   SDRAM_WE_N,				 //	SDRAM Write Enable
//	output			   SDRAM_CAS_N,			 //	SDRAM Column Address Strobe
//	output			   SDRAM_RAS_N,			 //	SDRAM Row Address Strobe
//	output	[1:0]	   SDRAM_BA,				 //	SDRAM Bank Address 
//	output			   SDRAM_CLK,				 //	SDRAM Clock	
//	output			   SDRAM_CS_N,				 //	SDRAM Chip Select
//	output			   SDRAM_CKE,				 //	SDRAM Clock Enable
	//-------------	USB Interface ------------------------------------------//
	inout	   [15:0]	USB_DATA,				 //	USB Data bus 16 Bits
	output	[1:0]	   USB_ADDR,				 //	USB Address bus 2 Bits
	output			   USB_SLRD,				 //	USB Read Enable
	output			   USB_SLWR,				 //	USB Write Enable
	output			   USB_SLOE,				 //	USB Output Enable
	input			      USB_FLAGA,				 //	USB Flag
	input			      USB_FLAGD,				 //	USB Flag
	output			   USB_PKEND,				 //	USB Packet end
	//output			   USB_WU2,				    //	USB Wake Up USB2
	
	output WB_RST,
	output WB_STB,
	output WB_WE,
	output WB_SEL,
	output WB_CYC,
	output WB_ADDR,
	output WB_DATA_I,
	output WB_DATA_O,
	output WB_STALL,
	output WB_ACK,
	
	output SDRAM_DQ,
	output SDRAM_ADDR,
	output SDRAM_DQM,
	output SDRAM_WE_N,
	output SDRAM_CAS_N,
	output SDRAM_RAS_N,
	output SDRAM_BA,
	output SDRAM_CLK,
	output SDRAM_CS_N,
	output SDRAM_CKE,
	output STATE,
	output BUFF0,
	output BUFF1,
	output BUFF2,
	output BUFF3,
	output BUFF4,
	output SDRAM_STATE,
	output SDRAM_CMD,
	input			      USB_IFCLK				 //	USB Clock inout
//	input			      USB_CLK_OUT,			 //	USB Clock Output
//	input	   [1:0]	   USB_INT,				    //   USB Interrupt
	//------------    GPIO --------------------------------------------------//
//	inout	   [33:0]	GPIOA,					 //   GPIOA, Can Be Used as Differential Pairs
//	inout	   [33:0]	GPIOB					    //   GPIOB, Can Be Used as Differential Pairs
	);

//reg [24 :0] counter = 0;
//reg updown = 0;
//
//always @(posedge CLOCK_48) begin
//   if (counter == `DELAY) begin
//	    counter <= 0;
//		 updown <= ~updown;
//	end
//	else begin
//	   counter <= counter + 1'b1;
//	end
//end

localparam IDLE = 4'b0000;
localparam SELECT_READ_FIFO = 4'b0001;
localparam READ_FROM_USB = 4'b0010;

localparam SELECT_WRITE_SDRAM = 4'b0011;
localparam WRITE_TO_SDRAM = 4'b0100;
localparam CLEAR_TEMP1 = 4'b1001;
localparam SELECT_READ_SDRAM = 4'b0101;
localparam READ_WAIT = 4'b1011;
localparam WRITE_WAIT = 4'b1100;
localparam READ_FROM_SDRAM = 4'b0110;
localparam RESET_READ = 4'b1010;
localparam CLEAR_TEMP2 = 4'b1010;

localparam SELECT_WRITE_FIFO = 4'b0111;
localparam WRITE_TO_USB = 4'b1000;

localparam RESET_WISHBONE = 4'b1101;


localparam DATA_WIDTH = 16;

wire WB_RST;
wire WB_STB;
wire WB_WE;
wire [3:0] WB_SEL;
wire WB_CYC;
wire [31:0]  WB_ADDR;
wire [31:0]  WB_DATA_I;
wire [31:0] WB_DATA_O;

reg wb_rst = 0;
reg wb_stb = 0;
reg wb_we = 0;
reg [3:0] wb_sel = 0;
reg wb_cyc = 0;
reg [31:0] wb_addr = 0;
reg [31:0] wb_data_i = 0;

reg[31:0] data_read_from_sdram [`MAXPKG - 1 : 0];
reg [`LOGMAXPKG - 1 : 0] sdram_counter = 0;
reg [12:0] sdram_addr_temp = 0;

reg read_finish = 0;

assign WB_RST = wb_rst;
assign WB_STB = wb_stb;
assign WB_WE = wb_we;
assign WB_SEL = wb_sel;
assign WB_CYC = wb_cyc;
assign WB_ADDR = wb_addr;
assign WB_DATA_I = wb_data_i;

wire [15:0]SDRAM_DQ;
wire [12:0]SDRAM_ADDR;
wire [1:0]SDRAM_DQM;
wire SDRAM_WE_N;
wire SDRAM_CAS_N;
wire SDRAM_RAS_N;
wire [1:0]SDRAM_BA;
wire SDRAM_CLK;
wire SDRAM_CS_N;
wire SDRAM_CKE;

reg [3 : 0]              state = 0;
reg [3 : 0]              state_nxt = 0;
reg [DATA_WIDTH - 1 : 0] buff [`MAXPKG - 1 : 0];
reg [`LOGMAXPKG - 1 : 0] counter = 0;
reg [`LOGMAXPKG - 1 : 0] number = 0;
reg                      usb_slrd = 1'b1;
reg                      usb_slwr = 1'b1;
reg                      usb_sloe = 1'b1;
reg                      usb_pkend = 1'b1;
reg [1 : 0]              usb_addr = 2'b00;
reg [DATA_WIDTH - 1 :  0]usb_data = 0;

reg [3:0] led = 4'b0;
reg [24 :0] delay_counter = 0;

assign USB_ADDR = usb_addr;
assign USB_SLRD = usb_slrd;
assign USB_SLWR = usb_slwr;
assign USB_SLOE = usb_sloe;
assign USB_PKEND= usb_pkend;
assign USB_DATA= (usb_sloe == 1'b1)? usb_data : 'bz;
assign LED = led;


wire[3:0] STATE;
assign STATE = state;

wire[15:0] BUFF0;
wire[15:0] BUFF1;
wire[15:0] BUFF2;
wire[15:0] BUFF3;
wire[15:0] BUFF4;
assign BUFF0 = buff[0];
assign BUFF1 = buff[1];
assign BUFF2 = buff[2];
assign BUFF3 = buff[3];
assign BUFF4 = buff[4];

wire[3:0] SDRAM_STATE;
wire [3:0] SDRAM_CMD;

sdram #(
.DATA_WIDTH (16)
) u_sdram (
   .clk_i (USB_IFCLK),
   .rst_i(WB_RST),
	.stb_i (WB_STB),
	.we_i (WB_WE),
	.sel_i (WB_SEL),
   .cyc_i(WB_CYC),
	.addr_i (WB_ADDR),
	.data_i (WB_DATA_I),
	.data_o (WB_DATA_O),
   .stall_o(WB_STALL),
	.ack_o (WB_ACK),

   .sdram_data_io (SDRAM_DQ),
	.sdram_addr_o  (SDRAM_ADDR),
	.sdram_dqm_o   (SDRAM_DQM),
   .sdram_we_o    (SDRAM_WE_N),
	.sdram_cas_o   (SDRAM_CAS_N),
	.sdram_ras_o   (SDRAM_RAS_N),
	.sdram_ba_o    (SDRAM_BA),
	.sdram_clk_o   (SDRAM_CLK),
	.sdram_cs_o    (SDRAM_CS_N),
	.sdram_cke_o   (SDRAM_CKE),
	.SDRAM_STATE (SDRAM_STATE),
	.SDRAM_CMD (SDRAM_CMD)
);

//IS42VM16400K r_sdram (
//   .dq (SDRAM_DQ),
//	.addr  (SDRAM_ADDR),
//	.dqm   (SDRAM_DQM),
//   .web    (SDRAM_WE_N),
//	.casb   (SDRAM_CAS_N),
//	.rasb   (SDRAM_RAS_N),
//	.ba    (SDRAM_BA),
//	.clk   (SDRAM_CLK),
//	.csb    (SDRAM_CS_N),
//	.cke   (SDRAM_CKE)
//);

MT48LC8M16A2 r_sdram (
   .dq (SDRAM_DQ),
	.addr  (SDRAM_ADDR),
	.dqm   (SDRAM_DQM),
   .web    (SDRAM_WE_N),
	.casb   (SDRAM_CAS_N),
	.rasb   (SDRAM_RAS_N),
	.ba    (SDRAM_BA),
	.clk   (SDRAM_CLK),
	.csb    (SDRAM_CS_N),
	.cke   (SDRAM_CKE)
);

/*
状态机
*/
always @(posedge USB_IFCLK) begin
   state <= state_nxt;
end
always @(*) begin
   case (state)
	    IDLE: begin
			  // EP2 not empty
		     if (USB_FLAGA == 1'b1) begin
			  // Transform to SELECT_READ_FIFO
			      state_nxt = SELECT_READ_FIFO;
			  end
			  else begin
			      state_nxt = IDLE;
			  end
			  $display("IDLE\n");
			  // led = IDLE;
		 end
		 SELECT_READ_FIFO: begin
		    state_nxt = READ_FROM_USB;
			 $display("SELECT_READ_FIFO\n");
			 // led = SELECT_READ_FIFO;
		 end
		 READ_FROM_USB: begin
		    if((counter == `MAXPKG - 1) || (USB_FLAGA == 1'b0))begin
			     state_nxt = RESET_WISHBONE;
          end
          else begin
			     state_nxt = READ_FROM_USB;
			 end
			 $display("READ_FROM_USB\n");
			 // led = READ_FROM_USB;
		 end
		 
		 // 新增
		 RESET_WISHBONE: begin
//			#50;
			state_nxt = SELECT_WRITE_SDRAM;
			$display("RESET_WISHBONE\n");
		 end
		 
		 SELECT_WRITE_SDRAM: begin
			state_nxt = WRITE_WAIT;
			$display("SELECT_WRITE_SDRAM\n");
			// led = SELECT_WRITE_SDRAM;
		 end
		 WRITE_WAIT: begin
		 $display("WRITE_WAIT\n");
			if (WB_ACK == 1'b1) begin
				state_nxt = WRITE_TO_SDRAM;
			end
			else begin
				state_nxt = WRITE_WAIT;
			end
		 end
		 WRITE_TO_SDRAM: begin
//		 	if (delay_counter == `DELAY) begin
//				 delay_counter <= 0;
//			end
//			else begin
//				delay_counter <= delay_counter + 1'b1;
//			end
			#300;
			state_nxt = SELECT_READ_SDRAM;
//			if (sdram_counter == number) begin
//				state_nxt = CLEAR_TEMP1;
//			end
//			else begin
//				state_nxt = SELECT_WRITE_SDRAM;
//			end
			$display("WRITE_TO_SDRAM\n");
			// led = WRITE_TO_SDRAM;
		 end
		 SELECT_READ_SDRAM: begin
			 state_nxt = READ_WAIT;
			 $display("SELECT_READ_SDRAM\n");
			 // led = SELECT_READ_SDRAM;
		 end
		 
		 READ_WAIT: begin
			if (WB_ACK == 1'b1) begin
				state_nxt = READ_FROM_SDRAM;
			end
			else begin
				state_nxt = READ_WAIT;
			end
			$display("READ_WAIT\n");
		 end
		 
		 READ_FROM_SDRAM: begin
			if (read_finish == 1) begin
				state_nxt = SELECT_WRITE_FIFO;
			end
			else begin
				state_nxt = READ_FROM_SDRAM;
			end
			$display("READ_FROM_SDRAM\n");
			// led = READ_FROM_SDRAM;
		 end

		 // end新增
		 
		 SELECT_WRITE_FIFO: begin
		     if(USB_FLAGD == 1'b1) begin
		         state_nxt = WRITE_TO_USB;
			  end
			  else begin
			      state_nxt = SELECT_WRITE_FIFO;
			  end
			  $display("SELECT_WRITE_FIFO\n");
			  // led = SELECT_WRITE_FIFO;
		 end
		 WRITE_TO_USB: begin
		    if ((counter >= number) || (USB_FLAGD == 1'b0)) begin
			     state_nxt = IDLE;
			 end
			 else begin
			     state_nxt = WRITE_TO_USB;
			 end
			 $display("WRITE_TO_USB\n");
			 // led = WRITE_TO_USB;
		 end
		 default: begin
		     state_nxt = IDLE;
			  $display("default\n");
			  // led = 4'b1111;
		 end
	endcase
end

always @(posedge USB_IFCLK) begin
    case (state)
	    IDLE: begin
		    usb_slrd <= 1'b1;
			 usb_slwr <= 1'b1;
			 usb_sloe <= 1'b1;
			 usb_pkend <= 1'b1;
			 usb_addr <= 2'b00;
			 counter <= 0;
			 number <= 0;
			 usb_data <= 0;
			 sdram_counter <= 0;
		 end
		 SELECT_READ_FIFO: begin
		    usb_slrd <= 1'b1;
			 usb_slwr <= 1'b1;
			 // sloe low
			 usb_sloe <= 1'b0;
			 usb_pkend <= 1'b1;
			 // select EP2
			 usb_addr <= 2'b00;
			 counter <= 0;
			 sdram_counter <= 0;
			 number <= 0;
			 usb_data <= 0;
		 end
		 READ_FROM_USB: begin
		    // If EP2 is not empty, to read
		    usb_slrd <= ~USB_FLAGA;
			 usb_slwr <= 1'b1;
			 // sloe low
			 usb_sloe <= 1'b0;
			 usb_pkend <= 1'b1;
			 // select EP2
			 usb_addr <= 2'b00;
			 // If EP2 is not empty
			 if(~usb_slrd) begin
			     counter <= counter + 1'b1;
			 end
			 // To store into buffer
			 buff[counter] <= USB_DATA;
			 // To record the written number of words
			 number <= counter;
			 usb_data <= 0;
		 end
		 // 新增
		 RESET_WISHBONE: begin
//			wb_rst <= 1'b1;
//			#50;
//			wb_rst <= 1'b0;
		 end
		 
		 SELECT_WRITE_SDRAM: begin
			//wb_rst <= 1'b0;
			wb_cyc <= 1'b1;
			wb_stb <= 1'b1;
			wb_we <= 1'b1;
			wb_addr  <= 0;
			wb_sel <= 4'b1111;
			wb_data_i <= buff[3];
//			led[0]<=WB_ACK;
//			led[3:1]<=buff[0][2:0];
		 end
		 WRITE_WAIT: begin
		 end
		 WRITE_TO_SDRAM: begin
			wb_stb <= 1'b0;
			wb_cyc <= 1'b0;
		 end
		 SELECT_READ_SDRAM: begin
			wb_cyc <= 1'b1;
			wb_stb <= 1'b1;
			wb_we <= 1'b0;
			wb_sel <= 4'b1111;
			wb_addr <= 0;
		 end
		 READ_WAIT: begin
			led<=WB_DATA_O[3:0];
		 end
		 READ_FROM_SDRAM: begin
			//led<=WB_ACK;
			data_read_from_sdram[0] <= WB_DATA_O;
			//led<=WB_DATA_O[3:0];
			//led<=4'b1010;
			read_finish <= 1;
		 end
		 // end新增
		 SELECT_WRITE_FIFO: begin
		    usb_slrd <= 1'b1;
			 usb_slwr <= 1'b1;
			 usb_sloe <= 1'b1;
			 if(USB_FLAGD == 1'b1) begin
			      usb_pkend <= 1'b1;
			 end
			 else begin
			      usb_pkend <= 1'b0;
			 end
			 // select EP6
			 usb_addr <= 2'b10;
			 counter <= 0;
			 usb_data <= 0;
		 end
		 WRITE_TO_USB: begin
		    usb_slrd <= 1'b1;
			 // If EP6 is not full, to write
			 usb_slwr <= ~USB_FLAGD;
			 usb_sloe <= 1'b1;
			 // If EP6 is full or written completely, submit data
			 if((USB_FLAGD == 1'b0) || (counter == number)) begin
			     usb_pkend <= 1'b0;   
			 end
			 else begin
			     usb_pkend <= 1'b1;
			 end
			 // select EP6
			 usb_addr <= 2'b10;
			 counter <= counter + 1'b1;
			 usb_data <= data_read_from_sdram[0];
		 end
		 default: begin
		    usb_slrd <= 1'b1;
			 usb_slwr <= 1'b1;
			 usb_sloe <= 1'b1;
			 usb_pkend <= 1'b1;
			 usb_addr <= 2'b00;
			 counter <= 0;
			 number <= 0;
			 usb_data <= 0;
		 end
	 endcase
end



endmodule
