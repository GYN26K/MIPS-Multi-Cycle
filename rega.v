module rega (
    input clk , 
    input [31:0] data_in ,
    output reg [31:0] A 
);

    always @ (posedge clk) begin 
        A <= data_in ;
    end

endmodule
