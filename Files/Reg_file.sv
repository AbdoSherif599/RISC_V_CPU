module Register_file(
    input clk,
    input rst,
    input [4:0] read_reg1,
    input [4:0] read_reg2,
    input [4:0] write_reg,
    input [31:0] write_data,
    input reg_write,
    output reg [31:0] read_data1,
    output reg [31:0] read_data2
);

    reg [31:0] registers [31:0]; // 32 registers of 32 bits each

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            integer i;
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= i;
            end
        end else if (reg_write && write_reg != 5'b00000) begin
            registers[write_reg] <= write_data;
        end
    end

    always @(*) begin
        // Read data from the specified registers
        read_data1 = (read_reg1 != 5'b00000) ? registers[read_reg1] : 32'h00000000;
        read_data2 = (read_reg2 != 5'b00000) ? registers[read_reg2] : 32'h00000000;
    end
    
endmodule