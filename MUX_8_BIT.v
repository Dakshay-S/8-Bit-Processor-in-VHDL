module mux_8_bit(a,b,sel,out);
input [7:0] a;
input [7:0] b;
input sel;

output  reg [7:0]out;

always @(*) 
  out <= sel ?  a : b ; 

endmodule