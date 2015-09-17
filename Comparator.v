module Comparator (c, Q, pulse);

	input [1:0] c;
	input [27:0]Q;
	output reg pulse;
	
	always @ *
		case (c)
			2'b00:
				pulse = (Q == 28'd1); //50 Hz
			2'b01:
				pulse = (Q == 28'd49999999); //1 Hz
			2'b10:
				pulse = (Q == 28'd99999999); //0.5 Hz
			2'b11:
				pulse = (Q == 28'd199999999); //0.5 Hz
		endcase
				
endmodule
