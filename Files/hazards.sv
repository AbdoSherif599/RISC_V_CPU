module data_forwarding_unit(
    input logic [31:0] current_instruction,EX_MEM_instruction, MEM_WB_instruction,
    input logic Reg_write_EX_MEM, Reg_write_MEM_WB,
    output logic [1:0] ctrl_alu_src1, ctrl_alu_src2,
    output logic [1:0] sw_ctrl
);
    reg [4:0] rd_ex_mem, rd_mem_wb;
    reg [4:0] rs1, rs2;
    reg sw_flag;
    assign rs1 = current_instruction[19:15]; // rs1 field
    assign rs2 = current_instruction[24:20]; // rs2 field
    assign rd_ex_mem = EX_MEM_instruction[11:7]; // rd field from EX/MEM stage
    assign rd_mem_wb = MEM_WB_instruction[11:7]; // rd field
    assign sw_flag = (current_instruction[6:0] == 7'b0100011); // Check if the instruction is a store word (SW)

    //0100011 sw opcode

    always_comb begin
        if (Reg_write_EX_MEM && (rd_ex_mem != 0) && (rd_ex_mem == rs1)) begin
            ctrl_alu_src1 = 2'b01; // Forward from EX/MEM stage to ALU src1
        end else if (Reg_write_MEM_WB && (rd_mem_wb != 0) && (rd_mem_wb == rs1)) begin
            ctrl_alu_src1 = 2'b10; // Forward from MEM/WB stage to ALU src1
        end else begin
            ctrl_alu_src1 = 2'b00; // No forwarding for ALU src1
        end

        if (Reg_write_EX_MEM && (rd_ex_mem != 0) && (rd_ex_mem == rs2)) begin
            ctrl_alu_src2 = 2'b01; // Forward from EX/MEM stage to ALU src2
        end else if (Reg_write_MEM_WB && (rd_mem_wb != 0) && (rd_mem_wb == rs2)) begin
            ctrl_alu_src2 = 2'b10; // Forward from MEM/WB stage to ALU src2
        end else begin
            ctrl_alu_src2 = 2'b00; // No forwarding for ALU src2
        end
        // Handle store word (SW) instruction
        if (sw_flag && (rd_ex_mem != 0) && EX_MEM_instruction[6:0]==7'b0000011) begin //
            sw_ctrl = 2'b11; // Forward from EX/MEM stage to ALU src1
        end else if (sw_flag && (rd_ex_mem != 0) && (rd_ex_mem == rs2)) begin
            sw_ctrl = 2'b01; // Forward from EX/MEM stage to ALU src1
        end else if (sw_flag && (rd_mem_wb != 0) && (rd_mem_wb == rs2)) begin
            sw_ctrl = 2'b10; // Forward from MEM/WB stage to ALU src1
        end else begin
            sw_ctrl = 2'b00; // No forwarding for ALU src1
        end

    end
endmodule



module hazard_detection_unit(
    input logic [6:0] opcode, // Opcode of the current instruction
    input logic Mem_read_EX_MEM, 
    input logic [4:0] rd_ex_mem,
    input logic [4:0] rs1, rs2,
    output logic stall
);
    //0100011 sw opcode
    always_comb begin
        if (opcode == 7'b0100011) begin // sw instruction
            stall = 1'b0; // No stall for sw instructions
        end else if (Mem_read_EX_MEM  && ((rd_ex_mem == rs1) || (rd_ex_mem == rs2))) begin
            stall = 1'b1; // Stall if EX/MEM stage writes to a register used in the current instruction
        end else begin
            stall = 1'b0; // No stall needed
        end
    end
endmodule

module branch_predictor(
    input logic branch,
    input logic Zero,
    output logic flush
);

    always_comb begin
        if (branch && Zero) begin
            flush = 1'b1; // Flush the pipeline if branch is taken
        end else begin
            flush = 1'b0; // No flush needed
        end
    end

endmodule