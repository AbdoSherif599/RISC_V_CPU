module data_mem(
    input clk,
    input rst,
    input [31:0] address,
    input [31:0] write_data,
    input mem_write,mem_read,
    output reg [31:0] read_data
);

    // Memory array to hold data
    reg [7:0] mem[0:16383];// 16 kb

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset memory to zero
            integer i;
            for (i = 0; i < 16384; i = i + 1) begin
                mem[i] <= 8'h00;
            end
        end else  begin 
            if (mem_write) begin
           {mem[address],mem[address+1],mem[address+2],mem[address+3]} <= write_data; 
            end
            
        end 
    end

    always_comb begin
        if (mem_read) begin
                read_data={mem[address],mem[address+1],mem[address+2],mem[address+3]};
            end
    end
    
endmodule