module Rate_Divider (clock, clear, Q);
	
	input clock, clear;
	output reg [27:0] Q; // log base 2 200 million accounts for maximum counting in case of 0.25 Hz
	
	always @ (posedge clock)
		if (clear)
			Q <= 28'd0;
		else
			Q <= Q+1;
			
endmodule
