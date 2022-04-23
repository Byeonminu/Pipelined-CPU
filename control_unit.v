`include "opcodes.v"

module ControlUnit (input [6:0] opcode,
                    output alu_src, mem_to_reg, write_enable, mem_read, mem_write, is_ecall,
                    output reg [1:0] alu_op);


reg [5:0] control;

assign {alu_src, mem_to_reg, write_enable, mem_read, mem_write, is_ecall} = control;

always @(*) begin
case (opcode)
    7'b0110011 : control = 6'b011000; // R-type
    7'b0000011 : control = 6'b101100; // lw-type
    7'b0100011 : control = 6'b100010; // s-type
    7'b0010011 : control = 6'b111000; // I-type
    7'b1110011 : control = 6'b000001; // ecall
    default : control = 6'b000000;
endcase
end

always @(*) begin
case (opcode) 
    7'b0110011 : alu_op = 2'b10; // R-type
    7'b0000011 : alu_op = 2'b00; // lw-type
    7'b0100011 : alu_op = 2'b00; // s-type
    7'b0010011 : alu_op = 2'b11; // I-type
    default : alu_op = 2'b00;
endcase
end

endmodule






/*

module ControlUnit (
                    input [6:0] opcode,
                    output reg mem_read,
                    output reg mem_write,
                    output reg mem_to_reg,
                    output reg write_enable,
                    output reg alu_src,
                    output reg [1:0] aluop,
                    output reg is_ecall);









always @(cur_state or opcode) begin
    if(cur_state == `IF) begin  // IF
          
        
        if(opcode == `ADD || opcode == `ADDI || opcode == `LW || opcode == `SW  || opcode == `ECALL ) begin
        
        mem_read =      1;
        pc_write = 0;
        mem_write = 0;
        write_enable = 0;

        end
       

       
    end
    else if(cur_state == `ID) begin
       
        
        aluop =     2'b00;
        if(opcode == `ADD || opcode == `ADDI || opcode == `LW || opcode == `SW) begin
        
        pc_write =      0;
      
        end
       
        else if (opcode == `ECALL) begin

            pc_write =      1;
        end
    end
    else if(cur_state == `EX) begin // EX
       
        case (opcode) 
        7'b0110011 : aluop = 2'b10; // R-type
        7'b0000011 : aluop = 2'b00; // lw-type
        7'b0100011 : aluop = 2'b00; // s-type
        7'b0010011 : aluop = 2'b11; // I-type
        endcase
        if(opcode == `ADD) begin // R type
        mem_to_reg =    0;
        end
        
        
    end
    else if(cur_state == `MEM) begin //MEM
      
        if(opcode == `LW) begin // LW type
        
        mem_read =      1;      
        mem_to_reg =    1;
        
        end
        else if(opcode == `SW) begin // SW type
    
        pc_write =      1;
        mem_write =     1;

     
        aluop =     2'b00;
        end
    end 
    else if(cur_state == `WB) begin
       
        case (opcode) 
        default : aluop = 2'b00;
        endcase
        if(opcode == `ADD) begin // R type
       
        pc_write =      1;
       
        
     
        write_enable =  1;
        end
        else if(opcode == `ADDI) begin//I type
        pc_write =      1;
       
        mem_to_reg =    0;
     
        write_enable =  1;

        end
        
        else if(opcode == `LW) begin // LW type
   
        pc_write =      1;       
        
        
        write_enable =  1;
        end
       
    end

end



endmodule

*/