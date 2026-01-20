module alu_srcb_mux (
    input  [1:0]  alusrcb,
    input  [31:0] B,
    input  [31:0] sign_imm,
    input  [31:0] sign_imm_shift2,
    output reg [31:0] srcB
);

    always @(*) begin
        case (alusrcb)
            2'b00: srcB = B;
            2'b01: srcB = 32'd4;
            2'b10: srcB = sign_imm;
            2'b11: srcB = sign_imm_shift2;
            default: srcB = 32'b0;
        endcase
    end

endmodule