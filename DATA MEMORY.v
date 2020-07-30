module dmem(input[7:0] addr,input[7:0] in,input read,input write,input clk,output[7:0] out);
	

reg[7:0] out;

reg[7:0] DATA[255:0];



always @(posedge clk)

begin
#1;
   
	if(read==1)
	begin
	out<=DATA[addr];
    end
    
	else if(write==1) DATA[addr]<=in;

end
endmodule