module x3(
    input reg_dst ,
    input [4:0] rt ,
    input [4:0] rd ,
    output reg [4:0] a3  
);

    always @ (*) begin 
        if(reg_dst) begin 
            a3 <= rd ;
        end
        else begin 
            a3 <= rt ;
        end
    end

endmodule