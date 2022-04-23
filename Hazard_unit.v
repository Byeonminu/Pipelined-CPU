`include "opcodes.v"

module HazardDetectionUnit (input [31:0] IF_ID_inst,
                            input [4:0] ID_EX_rd,
                            input ID_EX_mem_read,
                            output pc_write,
                            output IF_ID_write,
                            output is_stall);

    if (ID_EX_mem_read == 1) begin
        if (IF_ID_inst[19:15] == ID_EX_rd || IF_ID_inst[24:20] == ID_EX_rd) begin
            assign pc_write = 0;
            assign IF_ID_write = 0;
            assign is_stall = 1;
        end
        else begin
            assign pc_write = 1;
            assign IF_ID_write = 1;
            assign is_stall = 0;
        end
    end
    else begin
        assign pc_write = 1;
        assign IF_ID_write = 1;
        assign is_stall = 0;
    end

endmodule