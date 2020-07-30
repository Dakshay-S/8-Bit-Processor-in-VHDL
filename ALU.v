module alu(       
      input          [7:0]     a,          //src1  
      input          [7:0]     b,          //src2  
      input          [2:0]     alu_ctrl_in,     //function sel  
      output     reg     [7:0]     result,          //result       
      output zero  
   );  
 always @(*)  
 begin   
      case(alu_ctrl_in)  
      3'b000: result = a + b; // add  
      3'b010: result = a - b; // sub  
      3'b100: result = a & b; // and  
      3'b101: result = a | b; // or  
     
      default:result = a + b; // add  
      endcase  
 end  
 assign zero = (result==8'd0) ? 1'b1: 1'b0;  
 endmodule 