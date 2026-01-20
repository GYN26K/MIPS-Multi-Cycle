module sign_extend(
    input [15:0] imm ,
    output [31:0] imm_ext
);

    assign imm_ext = {{16{imm[15]}} , imm} ;

endmodule
