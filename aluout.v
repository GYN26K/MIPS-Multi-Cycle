module ALUout(
    input clk , 
    input [31:0] ALU_result ,
    output reg [31:0] alu_out
);

    always @ (posedge clk) begin 
        alu_out <= ALU_result ;
    end

endmodule