module sign_ext(a,out);
input [5:0] a;
output reg [7:0] out;
always @ (*)
begin
 if(a[5]==1'b0)
                 out <= {2'b00,a};
            else
                 out <= {2'b11,a};
end
endmodule
