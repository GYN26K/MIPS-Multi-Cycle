module mdr( 
    input clk , 
    input [31:0] mem_data,
    output reg [31:0] data     
);

    always @ (posedge clk) begin 
        data <= mem_data;
    end

endmodule