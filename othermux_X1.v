module x1 (
    input [1:0] PCSrc ,
    input [31:0] ALU_result ,
    input [31:0] ALU_out ,
    input [31:0] jump_addr,
    output reg [31:0] pc_next
);

    always @ (*) begin 
        case (PCSrc)
            2'b00: pc_next = ALU_result;
            2'b01: pc_next = ALU_out;
            2'b10: pc_next = jump_addr;
            default: pc_next = 32'b0;
        endcase
    end

endmodule