module regb (
    input clk , 
    input [31:0] data_in ,
    output reg [31:0] B  
);

    always @ (posedge clk) begin 
        B <= data_in ;
    end

endmodule 