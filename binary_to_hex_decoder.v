module binary_to_hex_decoder (cc,HEXX);
	input [3:0] cc;
	output [6:0] HEXX;
	
	//integer HEX;
	wire [3:0] c;
	assign c = cc;
	assign HEXX = HEX;
	reg [6:0]HEX;
	always @(c)
		case(c)
			4'b0000:
				begin
					HEX[0]=0;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=1;
				end
			4'b0001:
				begin
					HEX[0]=1;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=1;
					HEX[4]=1;
					HEX[5]=1;
					HEX[6]=1;
				end
			4'b0010:
				begin
					HEX[0]=0;
					HEX[1]=0;
					HEX[2]=1;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=1;
					HEX[6]=0;
				end

			4'b0011:
				begin
					HEX[0]=0;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=1;
					HEX[5]=1;
					HEX[6]=0;
				end

			4'b0100:
				begin
					HEX[0]=1;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=1;
					HEX[4]=1;
					HEX[5]=0;
					HEX[6]=0;
				end

			4'b0101:
				begin
					HEX[0]=0;
					HEX[1]=1;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=1;
					HEX[5]=0;
					HEX[6]=0;
				end

			4'b0110:
				begin
					HEX[0]=0;
					HEX[1]=1;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=0;
				end

			4'b0111:
				begin
					HEX[0]=0;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=1;
					HEX[4]=1;
					HEX[5]=1;
					HEX[6]=1;
				end

			4'b1000:
				begin
					HEX[0]=0;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=0;
				end

			4'b1001:
				begin
					HEX[0]=0;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=1;
					HEX[5]=0;
					HEX[6]=0;
				end

			4'b1010:
				begin
					HEX[0]=0;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=1;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=0;
				end

			4'b1011:
				begin
					HEX[0]=1;
					HEX[1]=1;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=0;
				end

			4'b1100:
				begin
					HEX[0]=0;
					HEX[1]=1;
					HEX[2]=1;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=1;
				end

			4'b1101:
				begin
					HEX[0]=1;
					HEX[1]=0;
					HEX[2]=0;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=1;
					HEX[6]=0;
				end

			4'b1110:
				begin
					HEX[0]=0;
					HEX[1]=1;
					HEX[2]=1;
					HEX[3]=0;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=0;
				end

			4'b1111:
				begin
					HEX[0]=0;
					HEX[1]=1;
					HEX[2]=1;
					HEX[3]=1;
					HEX[4]=0;
					HEX[5]=0;
					HEX[6]=0;
				end
		endcase
endmodule
