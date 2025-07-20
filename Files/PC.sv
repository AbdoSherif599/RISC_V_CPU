module PC_updater (
    input  logic        clk,
    input  logic        rst,
    input  logic        branch,
    input  logic        stall,
    input  logic [31:0] branch_offset, // already sign-extended and left-shifted if needed
    output logic [31:0] PC
);
    reg [31:0] PC_4,PC_reg,PC_reg2,branched_PC,branched_PC_reg;

    assign PC_4=PC+ 32'h00000004; // Increment PC by 4
    assign branched_PC = PC_reg2 + branch_offset; // Calculate the branched PC
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            PC <= 32'h00000000;
        else begin
                PC_reg <= PC; // Store PC+4 for next cycle
                PC_reg2 <= PC_reg; // Store PC+4 for next cycle
                branched_PC_reg <= branched_PC; // Store branched PC for next cycle
                if (stall) begin
                    PC<=PC;
                end else if (branch) begin
                    PC <= branched_PC_reg; // Update PC to branched address if branch is taken
                end else begin
                    PC <= PC_4; // Otherwise, continue with PC+4
                end
            end
    end


endmodule
