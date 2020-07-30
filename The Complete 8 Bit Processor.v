module andg(out , a , b);
input a ,b;
output reg out;

always @ (*)
begin
out <= a&b;

end
endmodule





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






module pc_incrmntr(pc,sum);
input [7:0] pc;

output reg [7:0] sum;

always @ (pc)
begin
sum <= pc+1;
end
endmodule







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








module imem (input [7:0] Addr , output [15:0] rd);

reg [15:0] RAM[256:0];
initial


begin
RAM[8'b00000000]=16'b0100110000000101;
RAM[8'b00000001]=16'b0100111000001010;
RAM[8'b00000010]=16'b1111110000000000;
RAM[8'b00000011]=16'b1111111000000001;
RAM[8'b00000100]=16'b1011001000000000;
RAM[8'b00000101]=16'b1011010000000001;
RAM[8'b00000110]=16'b0000011011010000;
RAM[8'b00000111]=16'b0100001001111111;
RAM[8'b00001000]=16'b1000001000001010;
RAM[8'b00001001]=16'b0010000000000110;

end



assign rd = RAM[Addr]; 
endmodule








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





module mux_8_bit(a,b,sel,out);
input [7:0] a;
input [7:0] b;
input sel;

output  reg [7:0]out;

always @(*) 
  out <= sel ?  a : b ; 

endmodule




module reg_file  
(  
      input                    clk,  
      input                    rst,  
      // write port  
      input                    reg_write_en,
      input          [2:0]     reg_write_dest,  
      input          [7:0]     reg_write_data,  
      //read port 1  
      input          [2:0]     reg_read_addr_1,  
      output         [7:0]     reg_read_data_1,  
      //read port 2  
      input          [2:0]     reg_read_addr_2,  
      output         [7:0]     reg_read_data_2  
);  
     
      reg [7:0] reg_array [7:0];  
    
            
      always @ (posedge clk)   
           if(rst) begin  
                reg_array[0] <= 8'b00000000;  
                reg_array[1] <= 8'b00000000;  
                reg_array[2] <= 8'b00000000;  
                reg_array[3] <= 8'b00000000;  
                reg_array[4] <= 8'b00000000;  
                reg_array[5] <= 8'b00000000;  
                reg_array[6] <= 8'b00000000;  
                reg_array[7] <= 8'b00000000;       
           end  
 	        
        always @(negedge clk)
            begin  
                if(reg_write_en)
                begin
                     reg_array[reg_write_dest] <= reg_write_data;  
                     end
           end  
       
      assign reg_read_data_1 = ( reg_read_addr_1 == 0)? 8'b0 : reg_array[reg_read_addr_1];  
      assign reg_read_data_2 = ( reg_read_addr_2 == 0)? 8'b0 : reg_array[reg_read_addr_2];  
 endmodule  








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









module cpu();

reg  clk = 1'b0;
reg pc_rst , reg_rst;
wire [7:0] pc_in , pc_out , inc_pc;

wire reg_sel , alu_src , reg_w_enbl ,  br , jump , m_r_enbl ,  m_w_enbl ,  mem_mux_sel ;
wire [3:0] to_alu_ctrl;

wire [15:0]inst;
wire [2:0] mx_t_reg  , alu_ctrl_t_alu;
wire [7:0] reg_w_data  , reg_a_data , reg_b_data , alu_out , mem_out;
wire [7:0] ext_imm , mx_t_alu  , imm_mux_out;
wire zero , and_t_mx;

pro_counter   PC(pc_in , clk , pc_rst , pc_out);
imem          I_MEM(pc_out  ,  inst );
CONTROL_UNIT  CU(inst[15:12] ,reg_sel ,alu_src ,reg_w_enbl ,br,jump , m_r_enbl , m_w_enbl , mem_mux_sel , to_alu_ctrl);

mux_3_bit    REG_MUX(inst[5:3] , inst[11:9] , reg_sel  , mx_t_reg);

reg_file     REG_FILE(clk , reg_rst , reg_w_enbl , inst[11:9] , reg_w_data , inst[8:6] , reg_a_data , mx_t_reg , reg_b_data );

sign_ext     SIGNEXT(inst[5:0] , ext_imm);

mux_8_bit   ALU_MUX(ext_imm , reg_b_data , alu_src , mx_t_alu);
alu_control ALU_CTRL(to_alu_ctrl , inst[2:0] , alu_ctrl_t_alu );
alu         ALU(reg_a_data , mx_t_alu , alu_ctrl_t_alu  , alu_out , zero);
dmem        D_MEM(alu_out , reg_b_data , m_r_enbl , m_w_enbl , clk , mem_out);
mux_8_bit   MEM_MUX(mem_out , alu_out , mem_mux_sel ,reg_w_data );
pc_incrmntr PC_INCR(pc_out , inc_pc);
andg         ANDG(and_t_mx ,zero ,br);
mux_8_bit   IMM_MUX( ext_imm , inc_pc , and_t_mx , imm_mux_out);
mux_8_bit   JUM_MUX( inst[7:0] , imm_mux_out , jump , pc_in);


always #5 clk =  ~clk;



initial
begin
pc_rst = 1;
reg_rst = 1;
$monitor($time , "   reg _1 %d  reg_2 = %d   reg_3 = %d  data[0] = %d  data[0] = %d", REG_FILE.reg_array[1] , REG_FILE.reg_array[2] ,  REG_FILE.reg_array[3]  , D_MEM.DATA[0]  ,  D_MEM.DATA[1]);

#6;
pc_rst = 0;
reg_rst = 0;


#1000;
$finish;

end



endmodule




