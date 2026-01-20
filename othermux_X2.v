module x2(
    input IorD ,
    input [31:0] pc ,
    input [31:0] ALU_out ,
    output reg [31:0] addr
);

    always @ (*) begin 
        if(IorD) begin 
            addr <= ALU_out ;
        end
        else begin 
            addr <= pc;
        end
    end

endmodule