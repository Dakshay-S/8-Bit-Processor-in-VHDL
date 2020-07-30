module pc_incrmntr(pc,sum);
input [7:0] pc;

output reg [7:0] sum;

always @ (*)
begin
sum <= pc+1;
end
endmodule