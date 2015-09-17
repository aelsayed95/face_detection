module charToHexDecoder (CLOCK_50, display, hex0, hex1, hex2, hex3, hex4, hex5);

	input CLOCK_50;
	input display;

	output hex0, hex1, hex2, hex3, hex4, hex5;
	
	reg [4:0] one 			= 5'b10001;
	reg [4:0] two 			= 5'b10000;
	reg [4:0] three 		= 5'b01111;
	reg [4:0] four 		= 5'b01110;
	reg [4:0] five 		= 5'b01101;
	reg [4:0] six 			= 5'b01100;
	reg [4:0] seven 		= 5'b01011;
	reg [4:0] eight 		= 5'b01010;
	reg [4:0] nine		   = 5'b01001;
	reg [4:0] ten 			= 5'b01000;
	reg [4:0] eleven     = 5'b00111;
	reg [4:0] twelve     = 5'b00110;
	reg [4:0] thirteen   = 5'b00101;
	reg [4:0] fourteen   = 5'b00100;
	reg [4:0] fifteen    = 5'b00011;
	reg [4:0] sixteen    = 5'b00010;
	reg [4:0] seventeen  = 5'b00001;
	reg [4:0] eighteen   = 5'b00000;
	
	always @ (posedge CLOCK_50)
	begin
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
		one <= eighteen;
	end
	charToHex c0(display,six, hex0);
	charToHex c1(display,five, hex1);
	charToHex c2(display,four, hex2);
	charToHex c3(display,three, hex3);
	charToHex c4(display,two, hex4);
	charToHex c5(display,one, hex5);

endmodule
	
	
	
	
	