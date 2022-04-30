`include "opcodes.v"

module HazardDetectionUnit (input [31:0] IF_ID_inst,
                            input [4:0] ID_EX_rd,
                            input ID_EX_mem_read,
                            input [4:0] EX_MEM_rd,
                            input EX_MEM_mem_read,
                            output reg pc_write,
                            output reg IF_ID_write,
                            output reg is_stall);

always @(*) begin
    if (ID_EX_mem_read == 1) begin
        if (IF_ID_inst[19:15] == ID_EX_rd || IF_ID_inst[24:20] == ID_EX_rd) begin
            pc_write = 0;
            IF_ID_write = 0;
            is_stall = 1;
        end
        else begin
            pc_write = 1;
            IF_ID_write = 1;
            is_stall = 0;
        end
    end
    else if(ID_EX_rd == 17 && IF_ID_inst[6:0] == `ECALL) begin // 1 stall
        pc_write = 0;
        IF_ID_write = 0;
        is_stall = 1;
    end
    else if (EX_MEM_mem_read == 1) begin
        if (EX_MEM_rd == 17 && IF_ID_inst[6:0] == `ECALL) begin // load 2 stall
            pc_write = 0; 
            IF_ID_write = 0;
            is_stall = 1;
        end
        else begin
            pc_write = 1;
            IF_ID_write = 1;
            is_stall = 0;
        end
    end
    else begin
        pc_write = 1;
        IF_ID_write = 1;
        is_stall = 0;
    end
    

end


endmodule