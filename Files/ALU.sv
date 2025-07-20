module ALU(
    input [31:0] A,
    input [31:0] B,
    input [3:0] ALUOp,
    output reg [31:0] Result,
    output reg Zero
    output reg negative
);

typedef enum logic[3:0] { ADD= 4'b0010,
                          SUB = 4'b0110,
                          SLT = 4'b0111,
                          XOR = 4'b1100,
                          AND_ = 4'b0000,
                          OR_ = 4'b0001,
                          SLL = 4'b1000,
                          SRL = 4'b1001,
                          SRA = 4'b1010   } operation;


assign Zero = A-B == 0; // Set Zero flag if A - B is zero
assign negative = Result[31]; // Set negative flag if the result is negative
always @(*) begin
    case (ALUOp)
        ADD: Result = (A) + (B); // ADD
        SUB: Result = $signed(A) - $signed(B); // SUB
        SLT: Result = $signed(A) < $signed(B)?1'b1:1'b0; // AND
        XOR: Result = A ^ B; // OR
        AND_: Result = A & B; // XOR
        OR_: Result = (A | B); // NOR
        SLL: Result = A << B[4:0]; // SLL
        SRL: Result = A >> B[4:0]; // SRL
        SRA: Result = $signed(A) >>> B[4:0]; // SRA
        default: Result = 32'h00000000; // Default case
        
    endcase
end
    
endmodule



module sign_extend(
    input [11:0] offset_load,input [11:0] offset_store,input [12:0] offset_branch, input [1:0] ctrl_sign_extend,
    output reg [31:0] out
);

    always @(*) begin
        case (ctrl_sign_extend)
            2'b00: out = {{20{offset_load[11]}}, offset_load}; // Load
            2'b01: out = {{20{offset_store[11]}}, offset_store}; // Store
            2'b10: out = {{19{offset_branch[12]}}, offset_branch}; // Branch
            default: out = 32'h00000000; // Default case
        endcase
    end
    
endmodule
/*
0x002081b3
0x00322023
0x00022283
0xfe318ae3
*/