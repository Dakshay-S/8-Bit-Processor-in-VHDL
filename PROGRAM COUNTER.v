module pro_counter(in,clk, reset, out);
input[7:0] in;
input clk;
input reset;
output reg [7:0] out;

always @(posedge clk )
begin
	if(reset) out<= 8'b00000000;
	else out<=in;

end
	
endmodule