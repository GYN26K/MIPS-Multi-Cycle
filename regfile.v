module reg_file (
    input clk ,
    input RegWrite ,
    input [4:0] rs , 
    input [4:0] rt , 
    input [4:0] rd ,
    input [31:0] write_data , 
    output [31:0] read_data1 ,
    output [31:0] read_data2
);

    reg [31:0] regs [31:0]; 

    assign read_data1 = (rs == 0) ? 32'b0 : regs[rs];
    assign read_data2 = (rt == 0) ? 32'b0 : regs[rt];

    always @(posedge clk) begin
        if (RegWrite && rd != 0) begin 
            regs[rd] <= write_data;
        end
    end
endmodule