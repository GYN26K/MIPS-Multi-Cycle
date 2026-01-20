module x4 (
    input memtoreg ,
    input [31:0] data ,
    input [31:0] alu_out ,
    output reg [31:0] wd3
);

    always @ (*) begin 
        if(memtoreg) begin 
            wd3 <= data ;
        end
        else begin 
            wd3 <= alu_out ;
        end
    end

endmodule