module alu_srca_mux (
    input alusrca,      
    input [31:0] pc,            
    input [31:0] A,             
    output reg [31:0] srcA      
);

    always @(*) begin
        if (alusrca)
            srcA = A;
        else
            srcA = pc;
    end

endmodule