`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   10:59:47 12/13/2020
// Design Name:   fpga
// Module Name:   D:/ISE_Proj/usb_demo/fpga_tb.v
// Project Name:  usb_demo
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: fpga
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////
`include "register.v"
module fpga_tb;

	// Inputs
	reg USB_FLAGA;
	reg USB_FLAGB;
	reg USB_FLAGC;
	reg USB_FLAGD;
	reg USB_IFCLK;

	// Outputs
	wire [3:0] LED;
	wire [1:0] USB_ADDR;
	wire USB_SLRD;
	wire USB_SLWR;
	wire USB_SLOE;
	wire USB_PKEND;
	wire WB_RST;
	wire WB_STB;
	wire WB_WE;
	wire [3:0] WB_SEL;
	wire WB_CYC;
	wire [31:0]  WB_ADDR;
	wire [31:0]  WB_DATA_I;
	wire [31:0]  WB_DATA_O;
	wire WB_STALL;
	wire WB_ACK;
	
	wire [3:0] STATE;
	wire [3:0] SDRAM_STATE;
	wire [4:0] SDRAM_CMD;
	wire OUTPUT_SDRAM_CLK;
	wire [31:0] CYCLE_COUNT;
	
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
	

	// Bidirs
	wire [15:0] USB_DATA;
	
	// Instantiate the Unit Under Test (UUT)
	fpga uut (
		.LED(LED), 
		.USB_DATA(USB_DATA), 
		.USB_ADDR(USB_ADDR), 
		.USB_SLRD(USB_SLRD), 
		.USB_SLWR(USB_SLWR), 
		.USB_SLOE(USB_SLOE), 
		.USB_FLAGA(USB_FLAGA),  
		.USB_FLAGD(USB_FLAGD), 
		.USB_PKEND(USB_PKEND), 
		.SDRAM_DQ (SDRAM_DQ),
		.SDRAM_ADDR (SDRAM_ADDR),
		.SDRAM_DQM (SDRAM_DQM),
		.SDRAM_WE_N (SDRAM_WE_N),
		.SDRAM_CAS_N (SDRAM_CAS_N),
		.SDRAM_RAS_N (SDRAM_RAS_N),
		.SDRAM_BA (SDRAM_BA),
		.SDRAM_CLK (SDRAM_CLK),
		.SDRAM_CS_N (SDRAM_CS_N),
		.SDRAM_CKE (SDRAM_CKE),
		.SDRAM_STATE (SDRAM_STATE),
		.SDRAM_CMD (SDRAM_CMD),
		.WB_RST(WB_RST),
		.WB_STB(WB_STB),
		.WB_WE(WB_WE),
		.WB_SEL(WB_SEL),
		.WB_CYC(WB_CYC),
		.WB_ADDR(WB_ADDR),
		.WB_DATA_I(WB_DATA_I),
		.WB_DATA_O(WB_DATA_O),
		.WB_STALL(WB_STALL),
		.WB_ACK(WB_ACK),
		.STATE(STATE),
		.OUTPUT_SDRAM_CLK(OUTPUT_SDRAM_CLK),
		.CYCLE_COUNT(CYCLE_COUNT),
		.USB_IFCLK(USB_IFCLK)
	);
	reg [15:0] in_fifo[0:255];
	reg [8:0] in_fifo_addr;
	reg [15:0] out_fifo[0:255];
	reg [8:0] out_fifo_addr;
	integer i;
	initial begin
		// Initialize Inputs
		USB_FLAGA = 0;
		USB_FLAGB = 0;
		USB_FLAGC = 0;
		USB_FLAGD = 0;
		USB_IFCLK = 0;
		in_fifo_addr = 0;
		out_fifo_addr = 0;
		
		//for (i = 0; i < 256; i = i + 1) begin
		for (i = 0; i < 5; i = i + 1) begin
			out_fifo[i] = i;
		end
		// Wait 100 ns for global reset to finish

		// Add stimulus here
		
	end
   initial begin
		forever #10 USB_IFCLK = ~USB_IFCLK;
	end

	always @(posedge USB_IFCLK) 
	begin
		if (out_fifo_addr >= 9'b00000_0100) 
			USB_FLAGA <= 0;
		else
			USB_FLAGA <= 1;
	end
	
	always @(posedge USB_IFCLK)
	begin
		if (~USB_SLOE & ~USB_SLRD & USB_FLAGA)
			out_fifo_addr <= out_fifo_addr + 1;
		else
			out_fifo_addr <= out_fifo_addr;
	end 
	
	always @(posedge USB_IFCLK) 
	begin
		if (in_fifo_addr == 9'b10000_0000) 
			USB_FLAGD <= 0;
		else
			USB_FLAGD <= 1;
	end
	
	always @(posedge USB_IFCLK)
	begin
		if (~USB_SLWR & USB_FLAGD)
			in_fifo_addr <= in_fifo_addr + 1;
		else
			in_fifo_addr <= in_fifo_addr;
	end 
	
	always @(posedge USB_IFCLK)
	begin
		if (~USB_SLWR & USB_FLAGD)
			in_fifo[in_fifo_addr] <= USB_DATA;
		else
			in_fifo[in_fifo_addr] <= in_fifo[in_fifo_addr];
	end 
	reg USB_SLOE_R;
	always @(posedge USB_IFCLK)
	begin
		USB_SLOE_R <= USB_SLOE;
	end
	assign USB_DATA = ~USB_SLOE ? out_fifo[out_fifo_addr] : 16'bz;
	
endmodule

