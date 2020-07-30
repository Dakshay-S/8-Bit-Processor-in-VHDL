module CONTROL_UNIT
(

input [3:0]  opcode ,

output reg  reg_sel , 
output reg  alu_src ,
output reg  reg_w_enbl , 
output reg  br ,
output reg  jump ,
output reg  m_r_enbl , 
output reg  m_w_enbl ,
output reg  mem_mux ,

output reg  [3:0] to_alu_ctrl

);

always @(opcode)
begin

to_alu_ctrl <= opcode;

if(opcode == 4'b0000)
begin
reg_sel <= 1'b1;    alu_src <= 1'b0;    reg_w_enbl<= 1'b1;     br <= 1'b0;   
jump <= 1'b0;       m_r_enbl <= 1'b0;   m_w_enbl <= 1'b0;       mem_mux <= 1'b0;    
end

else if(opcode == 4'b0100)
begin
reg_sel <= 1'b1;    alu_src <= 1'b1;    reg_w_enbl<= 1'b1;     br <= 1'b0;   
jump <= 1'b0;       m_r_enbl <= 1'b0;   m_w_enbl <= 1'b0;       mem_mux <= 1'b0;    
end


else if(opcode == 4'b1011)
begin
reg_sel <= 1'b1;    alu_src <= 1'b1;    reg_w_enbl<= 1'b1;     br <= 1'b0;   
jump <= 1'b0;       m_r_enbl <= 1'b1;   m_w_enbl <= 1'b0;       mem_mux <= 1'b1;    
end

else if(opcode == 4'b1111)
begin
reg_sel <= 1'b0;    alu_src <= 1'b1;    reg_w_enbl<= 1'b0;     br <= 1'b0;   
jump <= 1'b0;       m_r_enbl <= 1'b0;   m_w_enbl <= 1'b1;       mem_mux <= 1'b0;    
end

else if(opcode == 4'b1000)
begin
reg_sel <= 1'b0;    alu_src <= 1'b0;    reg_w_enbl<= 1'b0;     br <= 1'b1;   
jump <= 1'b0;       m_r_enbl <= 1'b0;   m_w_enbl <= 1'b0;       mem_mux <= 1'b0;    
end

else if(opcode == 4'b0010)
begin
reg_sel <= 1'b1;    alu_src <= 1'b0;    reg_w_enbl<= 1'b0;     br <= 1'b0;   
jump <= 1'b1;       m_r_enbl <= 1'b0;   m_w_enbl <= 1'b0;       mem_mux <= 1'b0;    
end


else
begin
reg_sel <= 1'b0;    alu_src <= 1'b0;    reg_w_enbl<= 1'b0;     br <= 1'b0;   
jump <= 1'b0;       m_r_enbl <= 1'b0;   m_w_enbl <= 1'b0;       mem_mux <= 1'b0;    
end


end
endmodule


