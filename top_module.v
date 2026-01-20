module ctrl_unit (
    input clk ,
    input reset ,
    input [5:0] opcode ,
    input zero ,

    output reg PCwrite ,
    output reg Branch ,
    output reg IRwrite ,
    output reg RegWrite ,
    output reg mem_write ,
    output reg IorD ,
    output reg alusrca ,
    output reg [1:0] alusrcb ,
    output reg [1:0] PCSrc ,
    output reg reg_dst ,
    output reg memtoreg ,
    output reg [1:0] ALUOp
);

    reg [3:0] state ;
    reg [3:0] nextstate ;

    parameter fetch = 4'b0001 ; 
    parameter decode = 4'b0010 ; 
    parameter memadr = 4'b0011 ; 
    parameter memread = 4'b0100 ; 
    parameter memwriteback = 4'b0101 ; 
    parameter memwrite = 4'b0110 ; 
    parameter execute = 4'b0111 ; 
    parameter ALUwriteback = 4'b1000 ; 
    parameter branch = 4'b1001 ; 
    parameter addiexecute = 4'b1010 ; 
    parameter addiwriteback = 4'b1011 ; 
    parameter jump = 4'b1100 ;

    parameter rtype_op = 6'b000000 ;
    parameter lw_op = 6'b100011 ;
    parameter sw_op = 6'b101011 ;
    parameter beq_op = 6'b000100 ;
    parameter addi_op = 6'b001000 ;
    parameter j_op = 6'b000010 ;

    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= fetch ;
        else
            state <= nextstate ;
    end

    always @(*) begin
        case (state)
            fetch: nextstate = decode ;

            decode: begin
                case (opcode) 
                    lw_op: nextstate = memadr ;
                    sw_op: nextstate = memadr ;
                    rtype_op: nextstate = execute ;
                    beq_op: nextstate = branch ;
                    addi_op: nextstate = addiexecute ;
                    j_op: nextstate = jump ;
                    default:nextstate = fetch ;
                
                endcase
            end

            memadr: begin
                if (opcode == lw_op) begin 
                    nextstate = memread ;
                end
                else begin
                    nextstate = memwrite ;
                end
            end

            memread: nextstate = memwriteback ;
            memwriteback: nextstate = fetch ;
            memwrite: nextstate = fetch ;
            execute: nextstate = ALUwriteback ;
            ALUwriteback: nextstate = fetch ;
            branch: nextstate = fetch ;
            addiexecute: nextstate = addiwriteback ;
            addiwriteback: nextstate = fetch ;
            jump: nextstate = fetch ;

            default: nextstate = fetch ;
        endcase
    end

    always @(*) begin

        PCwrite   = 0 ;
        Branch    = 0 ;
        IRwrite   = 0 ;
        RegWrite  = 0 ;
        mem_write = 0 ;

        IorD      = 0 ;
        alusrca   = 0 ;
        alusrcb   = 2'b00 ;
        PCSrc     = 2'b00 ;
        reg_dst   = 0 ;
        memtoreg  = 0 ;
        ALUOp     = 2'b00 ;

        case (state)
            fetch: begin
                IorD = 0 ;
                alusrca = 0 ;
                PCwrite = 1 ;
                IRwrite = 1 ;
                alusrca = 0 ;
                alusrcb = 2'b01 ;
                ALUOp   = 2'b00 ;
                PCSrc   = 2'b00 ;
            end

            decode: begin
                alusrca = 0 ;
                alusrcb = 2'b11 ;
                ALUOp   = 2'b00 ;
            end

            memadr: begin
                alusrca = 1 ;
                alusrcb = 2'b10 ;
                ALUOp   = 2'b00 ;
            end

            memread: begin
                IorD = 1 ;
            end

            memwriteback: begin
                RegWrite = 1 ;
                reg_dst  = 0 ;
                memtoreg = 1 ;
            end

            memwrite: begin
                IorD      = 1 ;
                mem_write = 1 ;
            end

            execute: begin
                alusrca = 1 ;
                alusrcb = 2'b00 ;
                ALUOp   = 2'b10 ;
            end

            ALUwriteback: begin
                RegWrite = 1 ;
                reg_dst  = 1 ;
                memtoreg = 0 ;
            end

            branch: begin
                alusrca = 1 ;
                alusrcb = 2'b00 ;
                ALUOp   = 2'b01 ;
                PCSrc   = 2'b01 ;
                Branch  = 1 ;
            end

            addiexecute: begin
                alusrca = 1 ;
                alusrcb = 2'b10 ;
                ALUOp   = 2'b00 ;
            end

            addiwriteback: begin
                RegWrite = 1 ;
                reg_dst  = 0 ;
                memtoreg = 0 ;
            end

            jump: begin
                PCwrite = 1 ;
                PCSrc   = 2'b10 ;
            end
        endcase
    end

endmodule