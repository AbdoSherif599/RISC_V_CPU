module instructions_mem(
    input clk,
    input rst,
    input [31:0] address,
    output reg [31:0] instruction
);

    // Memory array to hold instructions
    reg [7:0] mem [0:4095];//4 kb

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset memory to zero
            integer i;
            for (i = 0; i < 4096; i = i + 1) begin
                mem[i] <= 8'h00;
            end
        end else begin
            // Fetch instruction from memory at the specified address
             // Assuming address is word-aligned
        end
    end
    always_comb begin
        instruction = {mem[address],mem[address+1],mem[address+2],mem[address+3]};
    end
endmodule
