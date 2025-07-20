module Control_Unit(opcode,branch,mem_read,mem_to_register,ALU_op,mem_write,ALU_src,Reg_write,ctrl_sign_extend);


typedef enum logic[6:0] {R_format=7'h33,I_format_load=7'h03 ,S_format=7'h23,B_format=7'h63 ,I_format=7'h13 } op_code;
output reg Reg_write,mem_write,mem_read,mem_to_register,ALU_src,branch;
output reg [1:0] ALU_op, ctrl_sign_extend;
input [6:0] opcode;
always @(*) begin
    case (opcode)
        R_format:begin    //R format
            ALU_src=0;
            mem_to_register=0;
            Reg_write=1;
            mem_read=0;
            mem_write=0;
            branch=0;
            ALU_op=2'b10;
            ctrl_sign_extend=2'bxx; // No sign extension needed for R-type         
        end
        I_format:begin    //I format
            ALU_src=1;
            mem_to_register=0;
            Reg_write=1;
            mem_read=0;
            mem_write=0;
            branch=0;
            ALU_op=2'b11;
            ctrl_sign_extend=2'b00; // I-type sign extension         
        end
        I_format_load:begin     // lw 0x23     0010  0011
            ALU_src=1;
            mem_to_register=1;
            Reg_write=1;
            mem_read=1;
            mem_write=0;
            branch=0;
            ALU_op=2'b11;
            ctrl_sign_extend=2'b00; // Load sign extension         
        end
        S_format:begin   // sw 0x2B     0010 1011 
            ALU_src=1;
            mem_to_register=1'bx;
            Reg_write=0;
            mem_read=0;
            mem_write=1;
            branch=0;
            ALU_op=2'b00;
            ctrl_sign_extend=2'b01; // Store sign extension         
        end
        B_format:begin  // beq  0x04 0000 0100
            ALU_src=0;
            mem_to_register=1'bx;
            Reg_write=0;
            mem_read=0;
            mem_write=0;
            branch=1;
            ALU_op=2'b01;
            ctrl_sign_extend=2'b10; // Branch sign extension         
        end
        default:begin
            ALU_src=1'bx;
            mem_to_register=1'bx;
            Reg_write=1'bx;
            mem_read=1'bx;
            mem_write=1'bx;
            branch=1'bx;
            ALU_op=2'bx;         
        end
    endcase
end

endmodule

module ALU_ctrl_unit (
    input  logic [1:0] alu_op,
    input  logic [2:0] funct3,
    input  logic [6:0] funct7,
    output logic [3:0] alu_ctrl_lines
);

    typedef enum logic [9:0] {
        ADD_R = 10'b0000000000,
        SUB_R = 10'b0100000000,
        SLL_R = 10'b0000000001,
        SLT_R = 10'b0000000010,
        XOR_R = 10'b0000000100,
        SRL_R = 10'b0000000101,
        SRA_R = 10'b0100000101,
        OR_R  = 10'b0000000110,
        AND_R = 10'b0000000111
    } R_format;
    typedef enum logic[2:0] { 
        ADD_I = 3'b000,
        SLL_I = 3'b001,
        SLT_I = 3'b010,
        SRA_I = 3'b011,
        XOR_I = 3'b100,
        SRL_I = 3'b101,
        OR_I  = 3'b110,
        AND_I = 3'b111
     } I_format;

    logic [9:0] funct_total;
    assign funct_total = {funct7, funct3};

    always_comb begin
        case (alu_op)
            2'b00: alu_ctrl_lines = 4'b0010; // For load/store (do ADD)
            2'b01: alu_ctrl_lines = 4'b0110; // For BEQ (do SUB)
            2'b10: begin                     // R-type
                case (funct_total)
                    ADD_R: alu_ctrl_lines = 4'b0010;
                    SUB_R: alu_ctrl_lines = 4'b0110;
                    SLT_R: alu_ctrl_lines = 4'b0111;
                    XOR_R: alu_ctrl_lines = 4'b1100;
                    AND_R: alu_ctrl_lines = 4'b0000;
                    OR_R : alu_ctrl_lines = 4'b0001;
                    SLL_R: alu_ctrl_lines = 4'b1000;
                    SRL_R: alu_ctrl_lines = 4'b1001;
                    SRA_R: alu_ctrl_lines = 4'b1010;
                    default: alu_ctrl_lines = 4'b1111; // Undefined
                endcase
            end
            2'b11: begin
                case (funct3)
                    ADD_I: alu_ctrl_lines = 4'b0010; // ADD
                    SLL_I: alu_ctrl_lines = 4'b1000; // SLL
                    SLT_I: alu_ctrl_lines = 4'b0111; // SLT
                    SRA_I: alu_ctrl_lines = 4'b1010; // SRA
                    XOR_I: alu_ctrl_lines = 4'b1100; // XOR
                    SRL_I: alu_ctrl_lines = (funct7[5]) ? 4'b1010 : 4'b1001; // SRA or SRL
                    OR_I : alu_ctrl_lines = 4'b0001; // OR
                    AND_I: alu_ctrl_lines = 4'b0000; // AND
                    default: alu_ctrl_lines = 4'b1111; // Undefined
                    
                endcase
            end
            default: alu_ctrl_lines = 4'b1111;
        endcase
    end

endmodule


module branch_control(
    input  logic B_format,
    input  logic Zero,
    input  logic negative,
    input logic [2:0] funct3,
    output logic branch
);
    always_comb begin
        case (funct3)
            3'b000: branch = B_format && Zero; // BEQ
            3'b001: branch = B_format && !Zero; // BNE
            3'b100: branch = B_format && negative; // BLT
            3'b101: branch = B_format && !negative; // BGE
            default: branch = 1'b0; // No branch for other funct3 values
            
        endcase
    end
    
endmodule