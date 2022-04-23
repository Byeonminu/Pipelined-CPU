module ForwardingUnit (input [4:0] rs1,
			input [4:0] rs2,
			input [4:0] EX_MEM_rd,
            input [4:0] MEM_WB_rd,
		    input EX_MEM_reg_write,
            input MEM_WB_reg_write,
            output [1:0] ForwardA,
            output [1:0] ForwardB
            );

    if ((rs1 != 5'b0) && (rs1 == EX_MEM_rd)
        assign ForwardA = 2'b10;
    else if ((rs1 != 5'b0) && (rs1 == MEM_WB_rd)
        assign ForwardA = 2'b01;
    else
        assign ForwardA = 2'b00;
        
    if ((rs2 != 5'b0) && (rs2 == EX_MEM_rd)
        assign ForwardB = 2'b10;
    else if ((rs2 != 5'b0) && (rs2 == MEM_WB_rd)
        assign ForwardB = 2'b01;
    else
        assign ForwardB = 2'b00;
    


endmodule

module MuxControl (input [4:0] rs1,
			input [4:0] rs2,
			input [4:0] EX_MEM_rd,
            
            );

endmodule