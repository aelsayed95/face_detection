module charToHex (display, char, hex);

	input display;
	input [4:0] char;
	output [6:0] hex;
	
	//integer HEX;
	reg [6:0]HEX;
	assign hex = HEX;
	always @(char)
	if(!display)
		case(char)
			5'b00000:
				begin
					HEX[0]=1;
					HEX[1]=1;
					HEX[2]=1;
					HEX[3]=1;
					HEX[4]=1;
					HEX[5]=1;
					HEX[6]=1;
				end
			5'b00001:
				begin
					HEX[0]=0;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=1;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=1;
				end
			5'b00010:
				begin
					HEX[0]=1;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=1;
					HEX[4]=1;
					HEX[5]=1;
					HEX[6]=1;
				end
			5'b00011:
				begin
					HEX[0]=0;
					HEX[1]=1;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=1;
				end

			5'b00100:
				begin
					HEX[0]=0;
					HEX[1]=1;
					HEX[2]=1;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=0;
				end

			5'b00101:
				begin
					HEX[0]=1;
					HEX[1]=1;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=0;
				end

			5'b00110:
				begin
					HEX[0]=1;
					HEX[1]=1;
					HEX[2]=1;
					HEX[3]=1;
					HEX[4]=1;
					HEX[5]=1;
					HEX[6]=1;
				end

			5'b00111:
				begin
					HEX[0]=0;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=1;
				end

			5'b01000:
				begin
					HEX[0]=1;
					HEX[1]=1;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=0;
				end

			5'b01001:
				begin
					HEX[0]=1;
					HEX[1]=1;
					HEX[2]=1;
					HEX[3]=1;
					HEX[4]=1;
					HEX[5]=1;
					HEX[6]=1;
				end

			5'b01010:
				begin
					HEX[0]=0;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=1;
					HEX[4]=1;
					HEX[5]=1;
					HEX[6]=1;
				end

			5'b01011:
				begin
					HEX[0]=1;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=1;
					HEX[4]=1;
					HEX[5]=1;
					HEX[6]=1;
				end

			5'b01100:
				begin
				HEX[0]=0;
					HEX[1]=1;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=1;
					HEX[5]=0;
					HEX[6]=0;
				end

			5'b01101:
				begin
					HEX[0]=1;
					HEX[1]=1;
					HEX[2]=1;
					HEX[3]=1;
					HEX[4]=1;
					HEX[5]=1;
					HEX[6]=1;
				end
			5'b01110:
				begin
					HEX[0]=1;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=1;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=0;
				end
			5'b01111:
				begin
					HEX[0]=0;
					HEX[1]=1;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=1;
					HEX[5]=0;
					HEX[6]=0;
				end

			5'b10000:
				begin
					HEX[0]=1;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=1;
				end
			5'b10001:
				begin
					HEX[0]=0;
					HEX[1]=0;
					HEX[2]=1;
					HEX[3]=1;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=0;
				end
			
			default:
				begin
					HEX[0]=1;
					HEX[1]=1;
					HEX[2]=1;
					HEX[3]=1;
					HEX[4]=1;
					HEX[5]=1;
					HEX[6]=1;
				end
				
		endcase
	else
	begin
		HEX[0]=1;
		HEX[1]=1;
		HEX[2]=1;
		HEX[3]=1;
		HEX[4]=1;
		HEX[5]=1;
		HEX[6]=1;
	end
endmodule
