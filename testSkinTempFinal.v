// --------------------------------------------------------------------
// Copyright (c) 2005 by Terasic Technologies Inc. 
// --------------------------------------------------------------------
//
//Error (176310): Can't place multiple pins assigned to pin location Pin_AE24 (IOC_X65_Y2_N2)

// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// --------------------------------------------------------------------
//           
//                     Terasic Technologies Inc
//                     356 Fu-Shin E. Rd Sec. 1. JhuBei City,
//                     HsinChu County, Taiwan
//                     302
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// --------------------------------------------------------------------
// Major Functions:	DE2 TV Box
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date :| Changes Made:
//   V1.0 :| Joe Yang	       :| 05/07/05  :| Initial Revision
//   V1.1 :| Johnny Chen       :| 05/09/05  :| Changed YCbCr2RGB Block,
//											   RGB output 8 Bits => 10 Bits
//   V1.2 :| Johnny Chen	   :| 05/10/05  :| H_SYNC & V_SYNC Timing fixed.
//   V1.3 :| Johnny Chen       :| 05/11/16  :| Added FLASH Address FL_ADDR[21:20]
//   V1.4 :| Joe Yang	       :| 06/07/20  :| Modify Output Color
//	  V2.0 :| Johnny Chen	   :| 06/11/20	:| New Version for DE2 v2.X PCB.
// --------------------------------------------------------------------

module testSkinTempFinal
	(
		////////////////////	Clock Input	 	////////////////////	 
		CLOCK_27,							//	27 MHz
		CLOCK_50,							//	50 MHz
		// EXT_CLOCK,						//	External Clock
		////////////////////	Push Button		////////////////////
		KEY,							//	Button[3:0]
		////////////////////	DPDT Switch		////////////////////
		SW,						//	DPDT Switch[17:0]
		////////////////////	7-SEG Dispaly	////////////////////
		
		HEX0,							//	Seven Segment Digital 0
		HEX1,							//	Seven Segment Digital 1
		HEX2,							//	Seven Segment Digital 2
		HEX3,							//	Seven Segment Digital 3
		HEX4,							//	Seven Segment Digital 4
		HEX5,							//	Seven Segment Digital 5
		HEX6,							//	Seven Segment Digital 6
		HEX7,							//	Seven Segment Digital 7
		
		////////////////////////	LED		////////////////////////
		
		LEDG,						//	LED Green[8:0]
		LEDR,						//	LED Red[17:0]
		
		////////////////////////	UART	////////////////////////
		/*
		UART_TXD,						//	UART Transmitter
		UART_RXD,						//	UART Receiver
		*/
		////////////////////////	IRDA	////////////////////////
		/*
		IRDA_TXD,						//	IRDA Transmitter
		IRDA_RXD,						//	IRDA Receiver
		*/
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,						//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 12 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask 
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 0
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	Flash Interface		////////////////
		/*
		FL_DQ,							//	FLASH Data bus 8 Bits
		FL_ADDR,						//	FLASH Address bus 22 Bits
		FL_WE_N,						//	FLASH Write Enable
		FL_RST_N,						//	FLASH Reset
		FL_OE_N,						//	FLASH Output Enable
		FL_CE_N,						//	FLASH Chip Enable
		*/
		////////////////////	SRAM Interface		////////////////
		/*
		SRAM_DQ,							//	SRAM Data bus 16 Bits
		SRAM_ADDR,						//	SRAM Adress bus 18 Bits
		SRAM_UB_N,						//	SRAM High-byte Data Mask 
		SRAM_LB_N,						//	SRAM Low-byte Data Mask 
		SRAM_WE_N,						//	SRAM Write Enable
		SRAM_CE_N,						//	SRAM Chip Enable
		SRAM_OE_N,						//	SRAM Output Enable
		*/
		////////////////////	ISP1362 Interface	////////////////
		/*
		OTG_DATA,						//	ISP1362 Data bus 16 Bits
		OTG_ADDR,						//	ISP1362 Address 2 Bits
		OTG_CS_N,						//	ISP1362 Chip Select
		OTG_RD_N,						//	ISP1362 Write
		OTG_WR_N,						//	ISP1362 Read
		OTG_RST_N,						//	ISP1362 Reset
		OTG_FSPEED,						//	USB Full Speed,	0 = Enable, Z = Disable
		OTG_LSPEED,						//	USB Low Speed, 	0 = Enable, Z = Disable
		OTG_INT0,						//	ISP1362 Interrupt 0
		OTG_INT1,						//	ISP1362 Interrupt 1
		OTG_DREQ0,						//	ISP1362 DMA Request 0
		OTG_DREQ1,						//	ISP1362 DMA Request 1
		OTG_DACK0_N,					//	ISP1362 DMA Acknowledge 0
		OTG_DACK1_N,					//	ISP1362 DMA Acknowledge 1
		*/
		////////////////////	LCD Module 16X2		////////////////
		/*
		LCD_ON,							//	LCD Power ON/OFF
		LCD_BLON,						//	LCD Back Light ON/OFF
		LCD_RW,							//	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN,							//	LCD Enable
		LCD_RS,							//	LCD Command/Data Select, 0 = Command, 1 = Data
		LCD_DATA,						//	LCD Data bus 8 bits
		*/
		////////////////////	SD_Card Interface	////////////////
		/*
		SD_DAT,							//	SD Card Data
		SD_DAT3,						//	SD Card Data 3
		SD_CMD,							//	SD Card Command Signal
		SD_CLK,							//	SD Card Clock
		*/
		////////////////////	USB JTAG link	////////////////////
		/*
		TDI,  							// CPLD -> FPGA (data in)
		TCK,  							// CPLD -> FPGA (clk)
		TCS,  							// CPLD -> FPGA (CS)
	    TDO,  							// FPGA -> CPLD (data out)
		 */
		////////////////////	I2C		////////////////////////////
		I2C_SDAT,						//	I2C Data
		I2C_SCLK,						//	I2C Clock
		////////////////////	PS2		////////////////////////////
		/*
		PS2_DAT,						//	PS2 Data
		PS2_CLK,						//	PS2 Clock
		*/
		////////////////////	VGA		////////////////////////////
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,  						//	VGA Blue[9:0]
		////////////	Ethernet Interface	////////////////////////
		/*
		ENET_DATA,						//	DM9000A DATA bus 16Bits
		ENET_CMD,						//	DM9000A Command/Data Select, 0 = Command, 1 = Data
		ENET_CS_N,						//	DM9000A Chip Select
		ENET_WR_N,						//	DM9000A Write
		ENET_RD_N,						//	DM9000A Read
		ENET_RST_N,						//	DM9000A Reset
		ENET_INT,						//	DM9000A Interrupt
		ENET_CLK,						//	DM9000A Clock 25 MHz
		*/
		////////////////	Audio CODEC		////////////////////////
		/*
		AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
		AUD_ADCDAT,						//	Audio CODEC ADC Data
		AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
		AUD_DACDAT,						//	Audio CODEC DAC Data
		AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
		AUD_XCK,						//	Audio CODEC Chip Clock
		*/
		////////////////	TV Decoder		////////////////////////
		TD_DATA,    					//	TV Decoder Data bus 8 bits
		TD_HS,							//	TV Decoder H_SYNC
		TD_VS,							//	TV Decoder V_SYNC
		TD_RESET,						//	TV Decoder Reset
		TD_CLK,							//	TV Decoder Line Locked Clock
		////////////////////	GPIO	////////////////////////////
		/*
		GPIO_0,							//	GPIO Connection 0
		GPIO_1							//	GPIO Connection 1
		*/
	);

////////////////////////	Clock Input	 	////////////////////////
input			CLOCK_27;					//	27 MHz
input			CLOCK_50;					//	50 MHz
// input			EXT_CLOCK;				//	External Clock
////////////////////////	Push Button		////////////////////////
input	[3:0]	KEY;					//	Button[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	[17:0]	SW;				//	DPDT Switch[17:0]
////////////////////////	7-SEG Dispaly	////////////////////////

output	[6:0]	HEX0;					//	Seven Segment Digital 0
output	[6:0]	HEX1;					//	Seven Segment Digital 1
output	[6:0]	HEX2;					//	Seven Segment Digital 2
output	[6:0]	HEX3;					//	Seven Segment Digital 3
output	[6:0]	HEX4;					//	Seven Segment Digital 4
output	[6:0]	HEX5;					//	Seven Segment Digital 5
output	[6:0]	HEX6;					//	Seven Segment Digital 6
output	[6:0]	HEX7;					//	Seven Segment Digital 7

////////////////////////////	LED		////////////////////////////
output	[8:0]	LEDG;				//	LED Green[8:0]
output	[17:0]LEDR;				//	LED Red[17:0]
////////////////////////////	UART	////////////////////////////
/*
output		UART_TXD;				//	UART Transmitter
input			UART_RXD;				//	UART Receiver
*/
////////////////////////////	IRDA	////////////////////////////
/*
output		IRDA_TXD;				//	IRDA Transmitter
input			IRDA_RXD;				//	IRDA Receiver
*/
///////////////////////		SDRAM Interface	////////////////////////
inout	[15:0]	DRAM_DQ;				//	SDRAM Data bus 16 Bits
output[11:0]   DRAM_ADDR;				//	SDRAM Address bus 12 Bits
output			DRAM_LDQM;				//	SDRAM Low-byte Data Mask 
output			DRAM_UDQM;				//	SDRAM High-byte Data Mask
output			DRAM_WE_N;				//	SDRAM Write Enable
output			DRAM_CAS_N;				//	SDRAM Column Address Strobe
output			DRAM_RAS_N;				//	SDRAM Row Address Strobe
output			DRAM_CS_N;				//	SDRAM Chip Select
output			DRAM_BA_0;				//	SDRAM Bank Address 0
output			DRAM_BA_1;				//	SDRAM Bank Address 0
output			DRAM_CLK;				//	SDRAM Clock
output			DRAM_CKE;				//	SDRAM Clock Enable
////////////////////////	Flash Interface	////////////////////////
/*
inout	[7:0]	FL_DQ;					//	FLASH Data bus 8 Bits
output	[21:0]	FL_ADDR;				//	FLASH Address bus 22 Bits
output			FL_WE_N;				//	FLASH Write Enable
output			FL_RST_N;				//	FLASH Reset
output			FL_OE_N;				//	FLASH Output Enable
output			FL_CE_N;				//	FLASH Chip Enable
*/
////////////////////////	SRAM Interface	////////////////////////
/*
inout		[15:0]	SRAM_DQ;				//	SRAM Data bus 16 Bits
output	[17:0]	SRAM_ADDR;			//	SRAM Address bus 18 Bits
output			SRAM_UB_N;				//	SRAM High-byte Data Mask 
output			SRAM_LB_N;				//	SRAM Low-byte Data Mask 
output			SRAM_WE_N;				//	SRAM Write Enable
output			SRAM_CE_N;				//	SRAM Chip Enable
output			SRAM_OE_N;				//	SRAM Output Enable
*/
////////////////////	ISP1362 Interface	////////////////////////
/*
inout	[15:0]	OTG_DATA;				//	ISP1362 Data bus 16 Bits
output	[1:0]	OTG_ADDR;				//	ISP1362 Address 2 Bits
output			OTG_CS_N;				//	ISP1362 Chip Select
output			OTG_RD_N;				//	ISP1362 Write
output			OTG_WR_N;				//	ISP1362 Read
output			OTG_RST_N;				//	ISP1362 Reset
output			OTG_FSPEED;				//	USB Full Speed,	0 = Enable, Z = Disable
output			OTG_LSPEED;				//	USB Low Speed, 	0 = Enable, Z = Disable
input			OTG_INT0;				//	ISP1362 Interrupt 0
input			OTG_INT1;				//	ISP1362 Interrupt 1
input			OTG_DREQ0;				//	ISP1362 DMA Request 0
input			OTG_DREQ1;				//	ISP1362 DMA Request 1
output			OTG_DACK0_N;			//	ISP1362 DMA Acknowledge 0
output			OTG_DACK1_N;			//	ISP1362 DMA Acknowledge 1
*/
////////////////////	LCD Module 16X2	////////////////////////////
/*
inout	[7:0]	LCD_DATA;				//	LCD Data bus 8 bits
output			LCD_ON;					//	LCD Power ON/OFF
output			LCD_BLON;				//	LCD Back Light ON/OFF
output			LCD_RW;					//	LCD Read/Write Select, 0 = Write, 1 = Read
output			LCD_EN;					//	LCD Enable
output			LCD_RS;					//	LCD Command/Data Select, 0 = Command, 1 = Data
*/
////////////////////	SD Card Interface	////////////////////////
/*
inout			SD_DAT;					//	SD Card Data
inout			SD_DAT3;				//	SD Card Data 3
inout			SD_CMD;					//	SD Card Command Signal
output			SD_CLK;					//	SD Card Clock
*/
////////////////////////	I2C		////////////////////////////////
inout			I2C_SDAT;				//	I2C Data
output		I2C_SCLK;				//	I2C Clock
////////////////////////	PS2		////////////////////////////////
/*
input		 	PS2_DAT;				//	PS2 Data
input			PS2_CLK;				//	PS2 Clock
*/
////////////////////	USB JTAG link	////////////////////////////
/*
input  			TDI;					// CPLD -> FPGA (data in)
input  			TCK;					// CPLD -> FPGA (clk)
input  			TCS;					// CPLD -> FPGA (CS)
output 			TDO;					// FPGA -> CPLD (data out)
*/
////////////////////////	VGA			////////////////////////////
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
////////////////	Ethernet Interface	////////////////////////////
/*
inout	[15:0]	ENET_DATA;				//	DM9000A DATA bus 16Bits
output			ENET_CMD;				//	DM9000A Command/Data Select, 0 = Command, 1 = Data
output			ENET_CS_N;				//	DM9000A Chip Select
output			ENET_WR_N;				//	DM9000A Write
output			ENET_RD_N;				//	DM9000A Read
output			ENET_RST_N;				//	DM9000A Reset
input				ENET_INT;				//	DM9000A Interrupt
output			ENET_CLK;				//	DM9000A Clock 25 MHz
*/
////////////////////	Audio CODEC		////////////////////////////
/*
inout			AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			AUD_ADCDAT;				//	Audio CODEC ADC Data
inout			AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output		AUD_DACDAT;				//	Audio CODEC DAC Data
inout			AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output		AUD_XCK;				//	Audio CODEC Chip Clock
*/
////////////////////	TV Decoder		////////////////////////////
input	[7:0]	TD_DATA;    			//	TV Decoder Data bus 8 bits
input			TD_HS;					//	TV Decoder H_SYNC
input			TD_VS;					//	TV Decoder V_SYNC
output		TD_RESET;				//	TV Decoder Reset
input			TD_CLK;					//	TV Decoder Line Locked Clock
////////////////////////	GPIO	////////////////////////////////
/*
inout	[35:0]	GPIO_0;					//	GPIO Connection 0
inout	[35:0]	GPIO_1;					//	GPIO Connection 1
*/
////////////////////////////////////////////////////////////////////



//	Enable TV Decoder
assign	TD_RESET	=	KEY[0];
wire [9:0] testSkin_R, testSkin_G, testSkin_B;
reg[9:0] VGA_Red, VGA_Green, VGA_Blue;
//reg [9:0] VGA_X1, VGA_Y1;

/*
// SRAM control
assign SRAM_ADDR = {VGA_X1[9:1], VGA_Y1[9:1]};
assign SRAM_DQ = VGA_X1[0] ? avg_out : 16'hzzzz;

assign SRAM_UB_N = 0;
assign SRAM_LB_N = 0;
assign SRAM_CE_N = 0;
assign SRAM_WE_N = VGA_X1[0] ? 1'b0 : 1'b1;
assign SRAM_OE_N = 0;

assign avg_in = VGA_X1[0] ? avg_in : SRAM_DQ;					// 10 bits
assign avg_out = avg_in - (avg_in >> 2) + (fltr2 >> 2);
assign avg2 = avg_out << 4;
*/
/*
wire[0:0] reset;
assign reset = KEY[0];
*/
						
I2C_AV_Config 		u1	(	//	Host Side
							.iCLK(CLOCK_50),
							.iRST_N(KEY[0]),
							//	I2C Side
							.I2C_SCLK(I2C_SCLK),
							.I2C_SDAT(I2C_SDAT)	);

//	TV Decoder Stable Check  Bruce Land's website
TD_Detect			u2	(.oTD_Stable(TD_Stable),
							.iTD_VS(TD_VS),
							.iTD_HS(TD_HS),
							.iRST_N(KEY[0])	);

//	Reset Delay Timer		Bruce Land's website
Reset_Delay			u3	(	.iCLK(CLOCK_50),
							.iRST(TD_Stable),
							.oRST_0(DLY0),
							.oRST_1(DLY1),
							.oRST_2(DLY2));

//	ITU-R 656 to YUV 4:2:2		Bruce Land's website
ITU_656_Decoder		u4	(	//	TV Decoder Input
							.iTD_DATA(TD_DATA),
							//	Position Output
							.oTV_X(TV_X),
							//	YUV 4:2:2 Output
							.oYCbCr(YCbCr),
							.oDVAL(TV_DVAL),
							//	Control Signals
							.iSwap_CbCr(Quotient[0]),
							.iSkip(Remain==4'h0),
							.iRST_N(DLY1),
							.iCLK_27(TD_CLK)	);

//	For Down Sample 720 to 640
DIV 				u5	(	.aclr(!DLY0),	
							.clock(TD_CLK),
							.denom(4'h9),
							.numer(TV_X),
							.quotient(Quotient),
							.remain(Remain));

//	SDRAM frame buffer

Sdram_Control_4Port	u6	(	//	HOST Side  Not there need to instantiate or find!!
						    .REF_CLK(CLOCK_27),
							.CLK_18(AUD_CTRL_CLK),
						    .RESET_N(1'b1),
							//	FIFO Write Side 1
						    .WR1_DATA(YCbCr),
							.WR1(TV_DVAL),
							.WR1_FULL(WR1_FULL),
							.WR1_ADDR(0),
							.WR1_MAX_ADDR(640*507),		//	525-18
							.WR1_LENGTH(9'h80),
							.WR1_LOAD(!DLY0),
							.WR1_CLK(TD_CLK),
							//	FIFO Read Side 1
						    .RD1_DATA(m1YCbCr),
				        	.RD1(m1VGA_Read),
				        	.RD1_ADDR(640*13),			//	Read odd field and bypess blanking
							.RD1_MAX_ADDR(640*253),
							.RD1_LENGTH(9'h80),
				        	.RD1_LOAD(!DLY0),
							.RD1_CLK(CLOCK_27),
							//	FIFO Read Side 2
						    .RD2_DATA(m2YCbCr),
				        	.RD2(m2VGA_Read),
				        	.RD2_ADDR(640*267),			//	Read even field and bypess blanking
							.RD2_MAX_ADDR(640*507),
							.RD2_LENGTH(9'h80),
				        	.RD2_LOAD(!DLY0),
							.RD2_CLK(CLOCK_27),
							//	SDRAM Side
						    .SA(DRAM_ADDR),
						    .BA({DRAM_BA_1,DRAM_BA_0}),
						    .CS_N(DRAM_CS_N),
						    .CKE(DRAM_CKE),
						    .RAS_N(DRAM_RAS_N),
				            .CAS_N(DRAM_CAS_N),
				            .WE_N(DRAM_WE_N),
						    .DQ(DRAM_DQ),
				            .DQM({DRAM_UDQM,DRAM_LDQM}),
							.SDR_CLK(DRAM_CLK)	);
							
//	YUV 4:2:2 to YUV 4:4:4
YUV422_to_444		u7	(	//	YUV 4:2:2 Input		//Bruce Land's website
							.iYCbCr(mYCbCr),
							//	YUV	4:4:4 Output
							.oY(mY),
							.oCb(mCb),
							.oCr(mCr),
							//	Control Signals
							.iX(VGA_X),
							.iCLK(CLOCK_27),
							.iRST_N(DLY0));
 
//	YCbCr 8-bit to RGB-10 bit 			Bruce Land's website
YCbCr2RGB 			u8	(	//	Output Side
							.Red(original_R),
							.Green(original_G),
							.Blue(original_B),
							.oDVAL(mDVAL),
							//	Input Side
							.iY(mY),
							.iCb(mCb),
							.iCr(mCr),
							.iDVAL(VGA_Read),
							//	Control Signal
							.iRESET(!DLY2),
							.iCLK(CLOCK_27));

//	VGA Controller			 From Bruce in a box
VGA_Ctrl			u9	(	//	Host Side
							.iRed(VGA_Red), //testSkin
							.iGreen(VGA_Green),
							.iBlue(VGA_Blue),
							//.iRed(original_R),
							//.iGreen(original_G),
							//.iBlue(original_B),
							.oCurrent_X(VGA_X),
							.oCurrent_Y(VGA_Y),
							.oRequest(VGA_Read),
							//	VGA Side
							.oVGA_R(VGA_R),
							.oVGA_G(VGA_G),
							.oVGA_B(VGA_B),
							.oVGA_HS(VGA_HS),
							.oVGA_VS(VGA_VS),
							.oVGA_SYNC(VGA_SYNC),
							.oVGA_BLANK(VGA_BLANK),
							.oVGA_CLOCK(VGA_CLK),
							//	Control Signal
							.iCLK(CLOCK_27),
							.iRST_N(DLY2)	);

//	For ITU-R 656 Decoder
wire	[15:0]	YCbCr;
wire	[9:0]	TV_X;
wire			TV_DVAL;

//	For VGA Controller
wire	[9:0]	original_R;
wire	[9:0]	original_G;
wire	[9:0]	original_B;
wire	[10:0]	VGA_X;
wire	[10:0]	VGA_Y;
wire			VGA_Read;	//	VGA data request
wire			m1VGA_Read;	//	Read odd field
wire			m2VGA_Read;	//	Read even field

//	For YUV 4:2:2 to YUV 4:4:4
wire	[7:0]	mY;
wire	[7:0]	mCb;
wire	[7:0]	mCr;

//	For field select
wire	[15:0]	mYCbCr;
wire	[15:0]	mYCbCr_d;
wire	[15:0]	m1YCbCr;
wire	[15:0]	m2YCbCr;
wire	[15:0]	m3YCbCr;

//	For Delay Timer
wire			TD_Stable;
wire			DLY0;
wire			DLY1;
wire			DLY2;

//	For Down Sample
wire	[3:0]	Remain;
wire	[9:0]	Quotient;

assign	m1VGA_Read	=	VGA_Y[0]		?	1'b0		: VGA_Read	;
assign	m2VGA_Read	=	VGA_Y[0]		?	VGA_Read	: 1'b0		;
assign	mYCbCr_d	   =	!VGA_Y[0]	?	m1YCbCr	: m2YCbCr	;
assign	mYCbCr = m5YCbCr;

wire		mDVAL;

// For filtering out background noise 
/*
reg [2879:0] SRAM;
reg [8:0] SRAM2[2879:0];
reg [1:0] curState, nextState;
reg [8:0] start, i, x, y, y2;
*/
reg [9:0] pixelR_out;
reg [9:0] pixelG_out;
reg [9:0] pixelB_out;

reg [9:0] filtcolor_R;
reg [9:0] filtcolor_G;
reg [9:0] filtcolor_B;

/*
parameter [1:0] init = 2'b00;
parameter [1:0] i_inc = 2'b01;
parameter [1:0] start_inc = 2'b10;
parameter [1:0] y2_inc = 2'b11;
*/

//	Line buffer, delay one line
Line_Buffer u10	(	.clken(VGA_Read),
					.clock(CLOCK_27),
					.shiftin(mYCbCr_d),
					.shiftout(m3YCbCr));

Line_Buffer u11	(	.clken(VGA_Read),
					.clock(CLOCK_27),
					.shiftin(m3YCbCr),
					.shiftout(m4YCbCr));

wire	[15:0]	m4YCbCr;
wire	[15:0]	m5YCbCr;
wire	[8:0]	Tmp1,Tmp2;
wire	[7:0]	Tmp3,Tmp4;

assign	Tmp1	=	m4YCbCr[7:0]+mYCbCr_d[7:0];
assign	Tmp2	=	m4YCbCr[15:8]+mYCbCr_d[15:8];
assign	Tmp3	=	Tmp1[8:2]+m3YCbCr[7:1];
assign	Tmp4	=	Tmp2[8:2]+m3YCbCr[15:9];
assign	m5YCbCr	=	{Tmp4,Tmp3};


/*///////////////////////////////////////////////// Display Message ///////////////////////////////////////////////////////////////////////// 
// Displaying the message: "PUSH S 17 to bEGIn"   																						                                //
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

reg [6:0] hex0, hex1, hex2, hex3, hex4, hex5;
 
 assign HEX0 = hex0;
 assign HEX1 = hex1;
 assign HEX2 = hex2;
 assign HEX3 = hex3;
 assign HEX4 = hex4;
 assign HEX5 = hex5;
 
 wire display;
 assign display = SW[17];

 reg [6:0] one    	= 7'b1001000;
 reg [6:0] two    	= 7'b1111001;
 reg [6:0] three  	= 7'b1000010;
 reg [6:0] four   	= 7'b0000110;
 reg [6:0] five   	= 7'b0000011;
 reg [6:0] six    	= 7'b1111111;
 reg [6:0] seven  	= 7'b0100011;
 reg [6:0] eight  	= 7'b0000111;
 reg [6:0] nine   	= 7'b1111111;
 reg [6:0] ten    	= 7'b1111000;
 reg [6:0] eleven 	= 7'b1111001;
 reg [6:0] twelve    = 7'b1111111;
 reg [6:0] thirteen  = 7'b0010010;
 reg [6:0] fourteen  = 7'b1111111;
 reg [6:0] fifteen   = 7'b0001001;
 reg [6:0] sixteen   = 7'b0010010;
 reg [6:0] seventeen = 7'b1000001;
 reg [6:0] eighteen  = 7'b0001100;
 reg [6:0] nineteen  = 7'b1111111;

 always @ (posedge CLOCK_50)
// if (!display)
 begin
  hex5 = eighteen;
  hex4 = seventeen;
  hex3 = sixteen;
  hex2 = fifteen;
  hex1 = fourteen;
  hex0 = thirteen;
 end
 /*else
 begin
	  hex5 = 7'b1111111;
	  hex4 = 7'b1111111;
	  hex3 = 7'b1111111;
	  hex2 = 7'b1111111;
	  hex1 = 7'b1111111;
	  hex0 = 7'b1111111;
 end 
 */
// Rate Divider
 
 reg pulse;
 reg [27:0] RateOut; 
 
 always @ (posedge CLOCK_50)
  if (pulse)
   RateOut <= 28'd0;
  else
   RateOut <= RateOut+1;
 
// Comparator 
 always @ *
  pulse = (RateOut == 28'd24999999); 
  
// CharToHex converter

always @ (posedge CLOCK_50)
begin
	if (pulse)
	begin
		nineteen <= eighteen;
		eighteen <= seventeen;
		seventeen <= sixteen;
		sixteen <= fifteen;
		fifteen <= fourteen;
		fourteen <= thirteen;
		thirteen <= twelve;
		twelve <= eleven;
		eleven <= ten;
		ten <= nine;
		nine <= eight;
		eight <= seven;
		seven <= six;
		six <= five;
		five <= four;
		four <= three;
		three <= two;
		two <= one;
		one <= nineteen;
	end
end

////////////////////////////////////////////////// Output Mode Selection /////////////////////////////////////////////////////////////////////////	
// controlled by switches 17,16,14 and 0, 1, 2											  									 													 //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


always @ (posedge CLOCK_50)
	begin
		if(SW[17]==0) // testSkin is off
			begin
				if(SW[2])
					VGA_Red = original_R;
				else
					VGA_Red = 0;
				if(SW[1])
					VGA_Green = original_G;
				else
					VGA_Green = 0;
				if(SW[0])
					VGA_Blue = original_B;
				else
					VGA_Blue = 0;
			end
		else
			if(SW[16]==0)
				begin
					VGA_Red = testSkin_R;
					VGA_Green = testSkin_G;
					VGA_Blue = testSkin_B;
				end
			else
				begin
					if (SW[15]==0)
					begin
						VGA_Red = pixelR_out;
						VGA_Green = pixelG_out;
						VGA_Blue = pixelB_out;	
					end
					else			
					begin
						VGA_Red = filtcolor_R;
						VGA_Green = filtcolor_G;
						VGA_Blue = filtcolor_B;
					end
				end
	end

/*///////////////////////////////////////////////// Detecting Skin Colored Pixels /////////////////////////////////////////////////////////////////	
// for each pixel, outputs white if the pixel meets the skin color threshold, or black otherwise																//
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
testSkin ts(CLOCK_50, original_R, original_G, original_B , testSkin_R, testSkin_G, testSkin_B);


/*//////////////////////////////////////////////////// Background Noise Filtering /////////////////////////////////////////////////////////////////
// for each pixel, if # of neighboring pixels in a 9x9 board >= 75% the pixel is skin colored																	 //
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


reg [6:0] raddress [80:0];
reg [6:0] waddress [80:0];
reg [80:0] wEnable ;
reg [80:0] rEnable ;
reg [80:0] data ;
wire [80:0] q;
//wire [6:0] RAM_index;
reg [6:0] x, y;
//wire [6:0] RAM_INDEX;

genvar i;

generate
    for (i=0; i <81; i=i+1)
    begin
	  :MEM 	RAM2PORT	RAM2PORT_inst0 (
				.clock ( CLOCK_50 ),
				.data ( data[i] ),
				.rdaddress ( raddress[i] ),
				.wraddress ( waddress[i] ),
				.rden ( rEnable[i] ),
				.wren ( wEnable[i] ),
				.q ( q[i] )
				);
    end
endgenerate
// storing the pixels from testSkin (in sync with VGA_X and VGA_Y)

reg [17:0] xx;
assign LEDR = xx;
integer indx;
reg [3:0]cnt;

integer percent;

always@(posedge KEY[3])
begin
	if(cnt<4 && SW[16])
		cnt <= cnt + 1;
	else
		cnt <= 0;
end
always@(*)
case(cnt)
	3'b001:percent <= 80*81;
	3'b010:percent <= 85*81;
	3'b011:percent <= 90*81;
	3'b100:percent <= 95*81;
	default:percent <= 75*81;
endcase

binary_to_hex_decoder BtoH(cnt,HEX7);


always@(posedge CLOCK_50)
begin
	indx = (9*VGA_Y)%81 + VGA_X%9;
	wEnable[indx]=1;
	waddress[indx] = VGA_X/9;
	data[indx] = testSkin_R[0]; // B or W
end
integer m, n,sum,index;


always@(posedge CLOCK_50)
begin
	//wEnable[9*y + x%9] = 0;
	//raddress[9*VGA_Y + VGA_X%9] = VGA_X/9; // current pixel
	//xx[1] = q[9*y + x%9];
	if(VGA_Y>3 && VGA_X>3 && VGA_Y<476 && VGA_X<636 && testSkin_R[0]==1) //a skin pixel
	begin
		for(m=0,sum=0;m<=8;m=m+1)
			for(n=0;n<=8;n=n+1)
			begin
				//index = (9*(VGA_Y+4-m))%81 + (VGA_X+4-n)%9;
				index = 9*m+n;
				rEnable[index] = 1;
				raddress[index] = ((VGA_X+4-n)/9);
				sum = sum + q[index];
			end
		xx = sum;

		if(sum*100 >= percent)
		begin
			if (SW[15] == 1)
			begin
				filtcolor_R = original_R;
				filtcolor_G = original_G;
				filtcolor_B = original_B;
			end
			else
			begin
				pixelR_out = 10'b1111111111;
				pixelG_out = 10'b1111111111;
				pixelB_out = 10'b1111111111;
			end
		end
		else
		begin
			if (SW[15] == 1)
			begin
				filtcolor_R = 10'b0000000000;
				filtcolor_G = 10'b0000000000;
				filtcolor_B = 10'b0000000000;
			end
			else
			begin
				pixelR_out = 10'b0000000000;
				pixelG_out = 10'b0000000000;
				pixelB_out = 10'b0000000000;
			end
		end
	end
	else
	begin
		pixelR_out = 10'b0000000000;
		pixelG_out = 10'b0000000000;
		pixelB_out = 10'b0000000000;
	end
end

endmodule
