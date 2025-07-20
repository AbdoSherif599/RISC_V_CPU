module Risc_V(input bit clk, input bit rst);
    wire branch, mem_read, ALU_src, mem_write;
    wire [31:0] address, instruction;
    wire [1:0] ctrl_sign_extend, ALUOp_ctrl;
    wire [4:0] read_reg1, read_reg2, write_reg;
    wire [31:0] write_data_reg, read_data1, read_data2;
    wire [31:0] A, B, Result;
    wire [3:0] ALUOp,ALU_opcode;
    wire Zero;

        wire [11:0] offset_load_Iformat, offset_store;
    wire [12:0] offset_branch;
    wire [31:0] sign_extended_out;
    reg [31:0] sign_extended_out2,read_data1_reg,read_data2_reg,read_data2_reg2; // Register to hold the current instruction
    reg branch1,branch2,branch3,Zero2; // Register to hold the branch signal
    reg negative1, negative2; // Register to hold the negative flag

    wire [31:0] write_data, read_data;
    //wire mem_write, mem_read;
    reg [31:0] instruction_reg,instruction_reg2, instruction_reg3,instruction_reg4; // Register to hold the current instruction
    reg [4:0] write_reg1, write_reg2,write_reg3; // Register to hold the instruction being processed
    reg reg_write1, reg_write2, reg_write3, reg_write4; // Register to hold the write register
    reg mem_to_register1, mem_to_register2, mem_to_register3, mem_to_register4; // Register to hold the memory to register signal
    reg [1:0] ALUOp_ctrl1, ALUOp_ctrl2; // Register to hold the ALU operation control signal
    reg ALU_src1, ALU_src2; // Register to hold the ALU source signal
    reg [31:0] Result2, Result3,read_data_reg; // Register to hold the ALU result
    reg mem_read1,mem_read2,mem_read3, mem_write1,mem_write2,mem_write3; // Register to hold the memory read and write signals
    wire [1:0] ctrl_alu_src1, ctrl_alu_src2;

    wire [1:0]sw_source_ctrl;
    wire stall,branch4;

    Control_Unit control_unit_inst (
        .opcode(instruction_reg[6:0]),
        .branch(branch1),
        .mem_read(mem_read1),
        .mem_to_register(mem_to_register1),
        .ALU_op(ALUOp_ctrl1),
        .mem_write(mem_write1),
        .ALU_src(ALU_src1),
        .Reg_write(reg_write1),
        .ctrl_sign_extend(ctrl_sign_extend)
    );
    // Program Counter
    wire [31:0] PC;
    PC_updater pc_updater_inst (
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .branch(branch4), // from control unit
        .branch_offset(sign_extended_out2), // from sign extend module
        .PC(PC)
    );
    // Control signals
    branch_control branch_control_inst (
        .branch(branch3), // from control unit
        .Zero(Zero2), // Zero signal from ALU
        .negative(negative2), // Negative flag from ALU
        .funct3(instruction_reg3[14:12]), // funct3 field from instruction
        .branch_signal(branch4) // Output branch signal
    );
    
    // Instantiate instruction memory
    
    //assign address = PC; // PC provides the address for instruction memory
    instructions_mem instruction_mem_inst (
        .clk(clk),
        .rst(rst),
        .address(PC),
        .instruction(instruction)
    );

    // Instantiate register file

    assign read_reg1 = instruction_reg[19:15]; // rs1
    assign read_reg2 = instruction_reg[24:20]; // rs2
    assign write_reg = write_reg3; // rd
    assign write_data_reg =(mem_to_register4) ? read_data_reg : Result3 ; // Data to write back to register file
    Register_file reg_file_inst (
        .clk(clk),
        .rst(rst),
        .read_reg1(read_reg1),
        .read_reg2(read_reg2),
        .write_reg(write_reg),
        .write_data(write_data_reg),
        .reg_write(reg_write4),// from control unit
        .read_data1(read_data1),
        .read_data2(read_data2)
    );

    ALU_ctrl_unit alu_ctrl_unit_inst (
        .alu_op(ALUOp_ctrl2), // from control unit
        .funct3(instruction_reg2[14:12]), // funct3 field from instruction
        .funct7(instruction_reg2[31:25]), // funct7 field from instruction
        .alu_ctrl_lines(ALUOp) // Output ALU operation
    );
    // Instantiate ALU
   // assign A = read_data1_reg; // First operand from register file
    wire [31:0] B_stage1;
    assign A = (ctrl_alu_src1 == 2'b10) ? write_data_reg : (ctrl_alu_src1 == 2'b01) ? Result2 : read_data1_reg; // Forwarding logic for ALU source 1
    assign B_stage1 = (ctrl_alu_src2 == 2'b10) ? write_data_reg : (ctrl_alu_src2 == 2'b01) ? Result2 : read_data2_reg;
    assign B = (ALU_src2) ? sign_extended_out2 : B_stage1; //
    assign ALU_opcode = ALUOp; // ALU operation from control unit

    ALU alu_inst (
        .A(A),
        .B(B),
        .ALUOp(ALU_opcode),
        .Result(Result),
        .Zero(Zero),
        .negative(negative1) // Negative flag from ALU
    );

    // Instantiate sign extend module

    assign offset_load_Iformat = instruction_reg[31:20]; // Load immediate
    assign offset_store = {instruction_reg[31:25],instruction_reg[11:7]}; // Store immediate
    assign offset_branch = {instruction_reg[31],instruction_reg[7],instruction_reg[30:25],instruction_reg[11:8],1'b0}; // Branch immediate

    sign_extend sign_extend_inst (
        .offset_load(offset_load_Iformat),
        .offset_store(offset_store),
        .offset_branch(offset_branch),
        .ctrl_sign_extend(ctrl_sign_extend),//from control unit
        .out(sign_extended_out)
    );

    

    // Instantiate data memory
    wire [31:0] sw_source_data;
    assign sw_source_data = (sw_source_ctrl == 2'b11)?read_data:(sw_source_ctrl == 2'b10) ? write_data_reg : 
                            (sw_source_ctrl == 2'b01) ? Result2 : read_data2_reg;
    assign write_data = read_data2_reg2; // Data to write to memory
    assign address = Result2; // Address for memory access
    data_mem data_mem_inst (
        .clk(clk),
        .rst(rst),
        .address(address),
        .write_data(write_data),
        .mem_write(mem_write3),
        .mem_read(mem_read3),
        .read_data(read_data)
    );
    // Data forwarding unit
    
    data_forwarding_unit data_forwarding_unit_inst (
        .current_instruction(instruction_reg2), // Current instruction
        .EX_MEM_instruction(instruction_reg3), // Instruction from EX/MEM stage
        .MEM_WB_instruction(instruction_reg4), // Instruction from MEM/WB stage
        .Reg_write_EX_MEM(reg_write3), // Register write signal from EX/MEM stage
        .Reg_write_MEM_WB(reg_write4), // Register write signal from MEM/WB stage
        .ctrl_alu_src1(ctrl_alu_src1), // Control signal for ALU source 1
        .ctrl_alu_src2(ctrl_alu_src2), // Control signal for ALU source 2
        .sw_ctrl(sw_source_ctrl) // Control signal for store word instruction
    );

    // Hazard detection unit
    
    hazard_detection_unit hazard_detection_unit_inst (
        .opcode(instruction_reg[6:0]), // Opcode of the current instruction
        .Mem_read_EX_MEM(mem_read2), // Memory read signal from EX/MEM stage
        .rd_ex_mem(instruction_reg2[11:7]), // Write register from EX/MEM stage
        .rs1(instruction_reg[19:15]), // Read register 1 from current instruction
        .rs2(instruction_reg[24:20]), // Read register 2 from current instruction
        .stall(stall) // Stall signal output
    );
    // Branch predictor
    wire flush;
    branch_predictor branch_predictor_inst (
        .branch(branch3), // Branch signal from control unit
        .Zero(Zero2), // Zero signal from ALU
        .flush(flush) // Stall signal output
    );
    // IF 
    always @(posedge clk ) begin
        instruction_reg  <= (stall ) ? instruction_reg: (flush)? 0 : instruction;
        reg_write2       <= (stall || flush)? 0:reg_write1; // Update the register write signal
        mem_to_register2 <= (stall ) ? mem_to_register2: (flush)? 0:mem_to_register1; // Update memory to register signal
        ALUOp_ctrl2      <= (stall ) ? ALUOp_ctrl2: (flush)? 0:ALUOp_ctrl1; // Update ALU operation control signal
        ALU_src2         <= (stall ) ? ALU_src2: (flush)? 0: ALU_src1; // Update ALU source signal
        branch2          <= (stall || flush)? 0: branch1; // Update branch signal for next cycle
        mem_read2        <= (stall || flush)? 0:mem_read1; // Update memory read signal for next cycle
        mem_write2 <= (stall || flush)? 0:mem_write1; // Update memory write signal for next cycle

    end
    // ID/EX
    always @(posedge clk) begin
        instruction_reg2 <= flush ? 32'b0 : instruction_reg; // Store the current instruction for next cycle
        reg_write3 <= flush ? 0 : reg_write2; // Update the register write signal for next cycle
        mem_to_register3 <= flush ? 0 : mem_to_register2; // Update memory to register signal for next
        branch3 <= flush ? 0 : branch2; // Update branch signal for next cycle
        mem_read3 <= flush ? 0 : mem_read2; // Update memory read signal for next cycle
    end
    always @(posedge clk) begin
         // Update instruction register with the fetched instruction
        instruction_reg3 <= instruction_reg2; // Store the current instruction for next cycle
        instruction_reg4 <= instruction_reg3; // Store the current instruction for next cycle
    end

    always @(posedge clk) begin
        write_reg1 <= instruction_reg[11:7]; // Store the read register 1
        write_reg2 <= write_reg1; // Store the read register 2
        write_reg3 <= write_reg2; // Store the write register
        //write_reg4 <= write_reg3; // Store rs1 for next cycle
        
        reg_write4 <= reg_write3; // Update the register write signal for next cycle
        mem_to_register4 <= mem_to_register3; // Update memory to register signal for next cycle
        sign_extended_out2 <= sign_extended_out; // Update the sign extended output for next cycle
        read_data1_reg <= read_data1; // Store read data 1 for next cycle
        read_data2_reg <= read_data2; // Store read data 2 for next cycle
        read_data2_reg2 <= sw_source_data; // Store read data for next cycle
        read_data_reg<= read_data; // Store read data from memory for next cycle
        Result2 <= Result; // Store the ALU result for next cycle
        Result3 <= Result2; // Store the ALU result for next cycle
        Zero2 <= Zero; // Update Zero signal for next cycle
        negative2 <= negative1; // Update negative flag for next cycle
        mem_write3 <= mem_write2; // Update memory write signal for next cycle
    end
endmodule

