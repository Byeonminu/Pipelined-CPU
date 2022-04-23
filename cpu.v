// Submit this file with other files you created.
// Do not touch port declarations of the module 'CPU'.

// Guidelines
// 1. It is highly recommened to `define opcodes and something useful.
// 2. You can modify modules (except InstMemory, DataMemory, and RegisterFile)
// (e.g., port declarations, remove modules, define new modules, ...)
// 3. You might need to describe combinational logics to drive them into the module (e.g., mux, and, or, ...)
// 4. `include files if required
`include "mux.v"
`include "pc.v"
`include "Memory.v"
`include "control_unit.v"
`include "ImmediateGenerator.v"
`include "opcodes.v"
`include "alu.v"
`include "RegisterFile.v"
`include "Forwarding.v"
`include "Hazard_unit.v"

module CPU(input reset,       // positive reset signal
           input clk,         // clock signal
           output is_halted); // Whehther to finish simulation
  /***** Wire declarations *****/
  /***** Register declarations *****/
  // You need to modify the width of registers
  // In addition, 
  // 1. You might need other pipeline registers that are not described below
  // 2. You might not need registers described below
  /***** IF/ID pipeline registers *****/
  reg [31:0] IF_ID_inst;           // will be used in ID stage
  /***** ID/EX pipeline registers *****/
  // From the control unit
  reg [1:0] ID_EX_alu_op;         // will be used in EX stage
  reg ID_EX_alu_src;        // will be used in EX stage
  reg ID_EX_mem_write;      // will be used in MEM stage
  reg ID_EX_mem_read;       // will be used in MEM stage
  reg ID_EX_mem_to_reg;     // will be used in WB stage
  reg ID_EX_reg_write;      // will be used in WB stage
  // From others
  reg [31:0] ID_EX_rs1_data;
  reg [31:0] ID_EX_rs2_data;
  reg [31:0] ID_EX_imm;
  reg [31:0] ID_EX_ALU_ctrl_unit_input;
  reg [4:0] ID_EX_rd;
  reg [4:0] ID_EX_rs1;
  reg [4:0] ID_EX_rs2;
  wire IF_ID_write; // signal

  /***** EX/MEM pipeline registers *****/
  // From the control unit
  reg EX_MEM_mem_write;     // will be used in MEM stage
  reg EX_MEM_mem_read;      // will be used in MEM stage
  reg EX_MEM_is_branch;     // will be used in MEM stage
  reg EX_MEM_mem_to_reg;    // will be used in WB stage
  reg EX_MEM_reg_write;     // will be used in WB stage
  // From others
  reg [31:0] EX_MEM_alu_out;
  reg [31:0] EX_MEM_dmem_data;
  reg [4:0] EX_MEM_rd;

  /***** MEM/WB pipeline registers *****/
  // From the control unit
  reg MEM_WB_mem_to_reg;    // will be used in WB stage
  reg MEM_WB_reg_write;     // will be used in WB stage
  // From others
  reg [31:0] MEM_WB_mem_to_reg_src_1;
  reg [31:0] MEM_WB_mem_to_reg_src_2;
  reg [4:0] MEM_WB_rd;



  //PC
  wire [31:0] next_pc;
  wire [31:0] current_pc;
  wire pc_write;
  //Inst mem
  wire [31:0] dout_inst;

  // RegisterFile
  wire [4:0] rs1;
  
  
  wire [31:0] rs1_dout;
  wire [31:0] rs2_dout;


  //control unit
  wire mem_read;
  wire mem_write;
  wire mem_to_reg;
  wire alu_src;
  wire write_enable;
  wire [1:0] alu_op;
  wire is_ecall;


  //ImmediateGenerator
  wire [31:0] imm_gen_out;


  // alu / alu_control
  wire [3:0] alu_control;
  wire [31:0] alu_in_1;
  wire [31:0] alu_in_2;
  wire [31:0] alu_result;

  wire [31:0] alu_in_2_temp;
  
  //Data mem
  wire [31:0] dout_mem;
  
  wire [31:0] mem_to_reg_temp;

  //forwarding | hazard
  wire [1:0] ForwardA;
  wire [1:0] ForwardB;
  wire is_stall;
  wire WB_rs1;
  wire WB_rs2;
  wire [31:0] rs1_temp;
  wire [31:0] rs2_temp;

  mux_2_to_1 rs1_forward(rs1_dout, mem_to_reg_temp, WB_rs1, rs1_temp);
  mux_2_to_1 rs2_forward(rs2_dout, mem_to_reg_temp, WB_rs2, rs2_temp);
  
  mux_2_to_1 mem_reg(MEM_WB_mem_to_reg_src_1, MEM_WB_mem_to_reg_src_2, MEM_WB_mem_to_reg, mem_to_reg_temp);
  mux_4_to_1 alu_input1(ID_EX_rs1_data, mem_to_reg_temp, EX_MEM_alu_out, ForwardA, alu_in_1);
  mux_4_to_1 alu_input2(ID_EX_rs2_data, mem_to_reg_temp, EX_MEM_alu_out, ForwardB, alu_in_2_temp);
  mux_2_to_1 reg_imm(alu_in_2_temp, ID_EX_imm, ID_EX_alu_src, alu_in_2);

  assign rs1 = (is_ecall) ? 17: IF_ID_inst[19:15];


  assign is_halted = (is_ecall && (rs1_dout == 10)) ? 1 : 0;
  
  assign next_pc = current_pc + 4;

  always @(*) begin

    $display("is_ecall is %b", is_ecall);
  end

  integer i;
  always @(*) begin
    if(current_pc >= 8'ha8) begin
      for (i = 0; i < 32; i = i + 1)
        $display("%d %x\n", i, reg_file.rf[i]);
        $finish();
    end
  end


  always @(*) begin
    $display("------ ------- ------ ------ ----- current_pc %x\n", current_pc);
    // $display("------ ------- ------ ------ ----- IF_ID_inst %x\n", IF_ID_inst);
    
    
  end
  // always @(*) begin

  //   $display("alu_in_1 %x\n", alu_in_1);
  //   $display("alu_in_2 %x\n", alu_in_2);
   
  // end
   always @(*) begin

   $display("EX_MEM_alu_out %x\n", EX_MEM_alu_out);
  end
   

   always @(*) begin
     $display("MEM_WB_mem_to_reg_src_1 %x", MEM_WB_mem_to_reg_src_1);
     $display("MEM_WB_mem_to_reg_src_2 %x", MEM_WB_mem_to_reg_src_2);
     $display("MEM_WB_mem_to_reg %x", MEM_WB_mem_to_reg);
   end
  // ---------- Update program counter ----------
  // PC must be updated on the rising edge (positive edge) of the clock.
  PC pc(
    .reset(reset),       // input (Use reset to initialize PC. Initial value must be 0)
    .clk(clk),         // input
    .next_pc(next_pc),     // input
    .current_pc(current_pc),   // output
    .pc_write(pc_write) // input
  );
  
  // ---------- Instruction Memory ----------
  InstMemory imem(
    .reset(reset),   // input
    .clk(clk),     // input
    .addr(current_pc),    // input
    .dout(dout_inst)     // output
  );

  // Update IF/ID pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      IF_ID_inst <= 32'b0;
    end
    else begin
      if(IF_ID_write)
        IF_ID_inst <= dout_inst;
    end
  end

  // ---------- Register File ----------
  RegisterFile reg_file (
    .reset (reset),        // input
    .clk (clk),          // input
    .rs1 (rs1),          // input
    .rs2 (IF_ID_inst[24:20]),          // input
    .rd (MEM_WB_rd),           // input
    .rd_din (mem_to_reg_temp),       // input
    .write_enable (MEM_WB_reg_write),    // input
    .rs1_dout (rs1_dout),     // output
    .rs2_dout (rs2_dout)     // output
  );


  // ---------- Control Unit ----------
  ControlUnit ctrl_unit (
    .opcode(IF_ID_inst[6:0]),  // input
    .mem_read(mem_read),      // output
    .mem_to_reg(mem_to_reg),    // output
    .mem_write(mem_write),     // output
    .alu_src(alu_src),       // output
    .write_enable(write_enable),  // output
    .alu_op(alu_op),        // output
    .is_ecall(is_ecall)       // output (ecall inst)
  );

  // ---------- Immediate Generator ----------
  ImmediateGenerator imm_gen(
    .part_of_inst(IF_ID_inst),  // input
    .imm_gen_out(imm_gen_out)    // output
  );

  // Update ID/EX pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      ID_EX_alu_op <= 2'b0;      
      ID_EX_alu_src <= 0;     
      ID_EX_mem_write <= 0;
      ID_EX_mem_read <= 0;    
      ID_EX_mem_to_reg <= 0;
      ID_EX_reg_write <= 0;  
      ID_EX_rs1_data <= 32'b0;
      ID_EX_rs2_data <= 32'b0;
      ID_EX_imm <= 32'b0;
      ID_EX_ALU_ctrl_unit_input <= 32'b0;
      ID_EX_rd <= 5'b0;
      ID_EX_rs1 <= 5'b0;
      ID_EX_rs2 <= 5'b0;
    end
    else begin
      if(is_stall) begin
        ID_EX_alu_op <= 2'b0;
        ID_EX_alu_src <= 0; 
        ID_EX_mem_write <= 0;
        ID_EX_mem_read <= 0; 
        ID_EX_mem_to_reg <= 0;
        ID_EX_reg_write <= 0;
        ID_EX_rs1 <= 5'b0;
        ID_EX_rs2 <= 5'b0;
        ID_EX_rd <=  5'b0;
        ID_EX_rs1_data <= 32'b0;
        ID_EX_rs2_data <= 32'b0;
        ID_EX_imm <= 32'b0;
        ID_EX_ALU_ctrl_unit_input <= 32'b0;
      end
      else begin
      ID_EX_alu_op <= alu_op; 
      ID_EX_alu_src <= alu_src; 
      ID_EX_mem_write <= mem_write;
      ID_EX_mem_read <= mem_read; 
      ID_EX_mem_to_reg <= mem_to_reg;
      ID_EX_reg_write <= write_enable;
      ID_EX_rs1_data <= rs1_temp;
      ID_EX_rs2_data <= rs2_temp;
      ID_EX_imm <= imm_gen_out;
      ID_EX_ALU_ctrl_unit_input <= IF_ID_inst;
      if(IF_ID_inst[6:0] == `ADD) begin
        ID_EX_rs1 <= IF_ID_inst[19:15];
        ID_EX_rs2 <= IF_ID_inst[24:20];
        ID_EX_rd <=  IF_ID_inst[11:7];
      end
      else if(IF_ID_inst[6:0] == `ADDI) begin
        ID_EX_rs1 <= IF_ID_inst[19:15];
        ID_EX_rs2 <= 5'b0;
        ID_EX_rd <=  IF_ID_inst[11:7];
      end
      else if(IF_ID_inst[6:0] == `LW) begin
        ID_EX_rs1 <= IF_ID_inst[19:15];
        ID_EX_rs2 <= 5'b0;
        ID_EX_rd <=  IF_ID_inst[11:7];
      end
      else if(IF_ID_inst[6:0] == `SW) begin
        ID_EX_rs1 <= IF_ID_inst[19:15];
        ID_EX_rs2 <= IF_ID_inst[24:20];
        ID_EX_rd <=  5'b0;
      end
      end
      
      // $display("ID/EX");
      // $display("ID_EX_alu_op");
    end
  end

  // ---------- ALU Control Unit ----------
  ALUControlUnit alu_ctrl_unit (
    .part_of_inst(ID_EX_ALU_ctrl_unit_input),  // input
    .alu_op(ID_EX_alu_op),
    .alu_control(alu_control)         // output
  );

  // ---------- ALU ----------
  ALU alu (
    .alu_control(alu_control),      // input
    .alu_in_1(alu_in_1),    // input  
    .alu_in_2(alu_in_2),    // input
    .alu_result(alu_result)  // output
  );

  // Update EX/MEM pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      EX_MEM_mem_write <= 0;    
      EX_MEM_mem_read <= 0;      
      // EX_MEM_is_branch <= 0;    
      EX_MEM_mem_to_reg <= 0;    
      EX_MEM_reg_write <= 0;     
      EX_MEM_alu_out <= 32'b0;
      EX_MEM_dmem_data <= 32'b0; 
      EX_MEM_rd = 5'b0;
    end
    else begin
      EX_MEM_mem_write <= ID_EX_mem_write;
      EX_MEM_mem_read <= ID_EX_mem_read;     
      //EX_MEM_is_branch <=    
      EX_MEM_mem_to_reg <= ID_EX_mem_to_reg;   
      EX_MEM_reg_write <= ID_EX_reg_write;     
      EX_MEM_alu_out <= alu_result; 
      EX_MEM_dmem_data <= alu_in_2_temp; 
      EX_MEM_rd <= ID_EX_rd;
    end
  end

  // ---------- Data Memory ----------
  DataMemory dmem(
    .reset (reset),      // input
    .clk (clk),        // input
    .addr (EX_MEM_alu_out),       // input
    .din (EX_MEM_dmem_data),        // input
    .mem_read (EX_MEM_mem_read),   // input
    .mem_write (EX_MEM_mem_write),  // input
    .dout (dout_mem)        // output
  );

  // Update MEM/WB pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      MEM_WB_mem_to_reg <= 0;
      MEM_WB_reg_write <= 0;
      MEM_WB_mem_to_reg_src_1 <= 32'b0;
      MEM_WB_mem_to_reg_src_2 <= 32'b0;
      MEM_WB_rd <= 5'b0;
    end
    else begin
      MEM_WB_mem_to_reg <= EX_MEM_mem_to_reg;
      MEM_WB_reg_write <= EX_MEM_reg_write;
      MEM_WB_mem_to_reg_src_1 <= dout_mem;
      MEM_WB_mem_to_reg_src_2 <= EX_MEM_alu_out;
      MEM_WB_rd <= EX_MEM_rd;
    end
  end

  ForwardingUnit forward(
    .rs1(ID_EX_rs1), //input
    .rs2(ID_EX_rs2), //input
    .EX_MEM_rd(EX_MEM_rd), //input
    .MEM_WB_rd(MEM_WB_rd), //input
    .EX_MEM_reg_write(EX_MEM_reg_write), //input
    .MEM_WB_reg_write(MEM_WB_reg_write), //input
    .ForwardA(ForwardA), //output
    .ForwardB(ForwardB) //output
  );

  HazardDetectionUnit hazard(
   .IF_ID_inst(IF_ID_inst), //input
   .ID_EX_rd(ID_EX_rd), //input
   .ID_EX_mem_read(ID_EX_mem_read), //input
   .pc_write(pc_write), //output
   .IF_ID_write(IF_ID_write), //output
   .is_stall(is_stall) //output
 );
 

 MuxControl mux_control(
  .IF_ID_inst(IF_ID_inst),
  .MEM_WB_rd(MEM_WB_rd),
  .WB_rs1(WB_rs1),
  .WB_rs2(WB_rs2)
 );
  
endmodule
