module instr_data (
    input clk ,
    input mem_write ,
    input [31:0] addr ,
    input [31:0] write_data ,
    output [31:0] read_data 
);
    reg [31:0] mem [255:0];

    assign read_data = mem[addr >> 2] ;

    always @ (posedge clk) begin 
        if(mem_write) begin 
            mem[addr >> 2] <= write_data ;
        end
    end

endmodule