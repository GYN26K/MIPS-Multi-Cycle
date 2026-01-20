module instr_reg (
    input clk , 
    input IRwrite ,
    input [31:0] instr_in ,
    output reg [31:0] instr 
); 

    always @ (posedge clk ) begin 
            if(IRwrite) begin 
                instr <= instr_in ;
            end
    end

endmodule