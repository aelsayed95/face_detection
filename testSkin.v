module testSkin(CLOCK_50, r_in, g_in, b_in, r_out, g_out, b_out);

	input CLOCK_50;
	input [9:0] r_in, g_in, b_in;
	output reg [9:0] r_out, g_out, b_out;	
	reg [10:0] max, min;
	always@(*)
		begin
			if(r_in > g_in && r_in > b_in)max = r_in;
			else if(g_in > r_in && g_in > b_in)max = g_in;
			else /*if(b_in > r_in && b_in > g_in)*/ max = b_in;
			if(r_in < g_in && r_in < b_in)min = r_in;
			else if(g_in < r_in && g_in < b_in)min = g_in;
			else /*if(b_in < r_in && b_in < g_in)*/min = b_in;	
		end
		
	always@(posedge CLOCK_50)
		begin		
			if(r_in>10'd95 && g_in>10'd40 && b_in>10'd20 && r_in > g_in && r_in-g_in >10'd15 && r_in > b_in && (max-min)>10'd15)
				begin
					r_out <= 10'b1111111111;
					g_out <= 10'b1111111111;
					b_out <= 10'b1111111111;
				end
			else
				begin
					r_out <= 10'b0000000000;
					g_out <= 10'b0000000000;
					b_out <= 10'b0000000000;
				end
		end
		
endmodule

