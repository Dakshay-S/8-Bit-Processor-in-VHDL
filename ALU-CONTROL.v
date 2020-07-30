module alu_control(opcode  , func , out);
input [3:0] opcode;
input  [2:0] func;
output reg [2:0] out;

always @ (*)
begin
if(opcode == 4'b0000)
out <= func;

else if(opcode == 4'b1000)
out <= 3'b010;

else
out <= 3'b000;

end
endmodule