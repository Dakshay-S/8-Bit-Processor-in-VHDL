module mux_3_bit(a,b,sel,out);
input [2:0] a;
input [2:0] b;
input sel;

output  reg [2:0]out;

always @(*) 
begin
  out <= sel ?  a : b ;
end
endmodule




