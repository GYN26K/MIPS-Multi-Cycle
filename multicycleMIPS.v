// MULTI SINGLE MIPS PROCESSOR 
// REFERENCE - DIGITAL DESIGN AND COMPUTER ARCHITECTURE BY HARRIS HARRIS

// THE PROGRAM COUNTER WHICH CONTAINS THE INSTR ADDR 
// IT GIVES THE NEXT PROGRAM INSTR 
module program_counter(
    input clk , 
    input reset , 
    input pc_en ,
    input [31:0] pc_next , 
    output reg [31:0] pc
);

    always @ (posedge clk) begin 
            if(reset) begin 
                pc <= 32'b0;
            end
            else if(pc_en) begin 
                pc <= pc_next;
            end
    end

endmodule

// REGISTER FILE 
// CONTAINS THE REGS THAT WILL USED TEMP

module reg_file (
    input clk ,
    input RegWrite ,
    input [4:0] rs , 
    input [4:0] rt , 
    input [4:0] rd ,
    input [31:0] write_data , 
    output [31:0] read_data1 ,
    output [31:0] read_data2
);

    reg [31:0] regs [31:0]; 

    assign read_data1 = (rs == 0) ? 32'b0 : regs[rs];
    assign read_data2 = (rt == 0) ? 32'b0 : regs[rt];

    always @(posedge clk) begin
        if (RegWrite && rd != 0) begin 
            regs[rd] <= write_data;
        end
    end
endmodule

// ALU BLOCK
// ARTHIMATIC LOGIC UNIT OF THE PROCESSOR 

module alu (
    input [31:0] A , 
    input [31:0] B , 
    input [3:0] alu_ctrl ,
    output reg [31:0] result , 
    output zero 
);

    always @(*) begin
        case (alu_ctrl)
            4'b0010: result = A + B;
            4'b0110: result = A - B;
            4'b0000: result = A & B;
            4'b0001: result = A | B;
            4'b0111: result = (A < B) ? 32'b1 : 32'b0;
            default: result = 32'b0;
        endcase
    end

    assign zero = (result == 0);
    
endmodule

// SIGNEXT MODULE 
// FOR THE IMM TYPE FUNC USING THE IMM

module sign_extend(
    input [15:0] imm ,
    output [31:0] imm_ext
);

    assign imm_ext = {{16{imm[15]}} , imm} ;

endmodule

// INSTR REG 
// STORES THS INSTR FETCCHED IN STATE S0 

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

// TAKING INPUTDS INTO REG A AND REG B 
// REG A 

module rega (
    input clk , 
    input [31:0] data_in ,
    output reg [31:0] A 
);

    always @ (posedge clk) begin 
        A <= data_in ;
    end

endmodule

// REG B

module regb (
    input clk , 
    input [31:0] data_in ,
    output reg [31:0] B  
);

    always @ (posedge clk) begin 
        B <= data_in ;
    end

endmodule 

// ALUOUT REG 
// TO STORE THE VALUE THAT MUST BE SAVED FOR NEXT CYCLE

module ALUout(
    input clk , 
    input [31:0] ALU_result ,
    output reg [31:0] alu_out
);

    always @ (posedge clk) begin 
        alu_out <= ALU_result ;
    end

endmodule

// MEMORY DATA REG 
// TO STORE THE DATA FOR THE NEXT CYCLE 

module mdr( 
    input clk , 
    input [31:0] mem_data,
    output reg [31:0] data     
);

    always @ (posedge clk) begin 
        data <= mem_data;
    end

endmodule

// SHIFT LEFT 2 
// FOR BEQ FUNC TYPE PURPOSE 

module shift_left(
    input [31:0] in , 
    output [31:0] out 
);

    assign out = (in << 2) ;

endmodule

// INSTR AND DATA MEMO 
// BOTH ARE COMBINED IN MULTPLE CYCLE AS WE CAN SAVE COST BY USING THE SAME HARDWARE AGAIN AGAIN 

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

// NOW WE NEED MUXS TO BE CODED 

// MUX A 

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

// MUX B 

module alu_srcb_mux (
    input  [1:0]  alusrcb,
    input  [31:0] B,
    input  [31:0] sign_imm,
    input  [31:0] sign_imm_shift2,
    output reg [31:0] srcB
);

    always @(*) begin
        case (alusrcb)
            2'b00: srcB = B;
            2'b01: srcB = 32'd4;
            2'b10: srcB = sign_imm;
            2'b11: srcB = sign_imm_shift2;
            default: srcB = 32'b0;
        endcase
    end

endmodule

// MUX X1 IN THE DAIGRAM 
// USED FOR NEXT C VALUE

module x1 (
    input [1:0] PCSrc ,
    input [31:0] ALU_result ,
    input [31:0] ALU_out ,
    input [31:0] jump_addr,
    output reg [31:0] pc_next
);

    always @ (*) begin 
        case (PCSrc)
            2'b00: pc_next = ALU_result;
            2'b01: pc_next = ALU_out;
            2'b10: pc_next = jump_addr;
            default: pc_next = 32'b0;
        endcase
    end

endmodule

// MUX X2 
// FOR MEMO ADDR SELECT 

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

// MUX X3 
// FOR rt AND rd

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

// MUX X4 
// FOR WRITEBACK DATA

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

// FOR THE ALU OPERATIONS

module alu_control (
    input [1:0] ALUOp ,
    input [5:0] funct ,
    output reg [3:0] alu_ctrl
);

    always @(*) begin
        case (ALUOp)
            2'b00: alu_ctrl = 4'b0010;   
            2'b01: alu_ctrl = 4'b0110;   
            2'b10: begin                
                case (funct)
                    6'b100000: alu_ctrl = 4'b0010;
                    6'b100010: alu_ctrl = 4'b0110; 
                    6'b100100: alu_ctrl = 4'b0000; 
                    6'b100101: alu_ctrl = 4'b0001; 
                    6'b101010: alu_ctrl = 4'b0111; 
                    default:   alu_ctrl = 4'b0000;
                endcase
            end
            default: alu_ctrl = 4'b0000;
        endcase
    end

endmodule


// THE CONTROL UNIT 
// CONTAINS ALL THE STATES AND DECIDES WHERE TO GO DEPENDING ON THE CURRENT STATE

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

// CPU 
// CONNECTS ALL THE MODULES AND STATES 

// CPU 
// CONNECTS ALL THE MODULES AND STATES 

module cpu(
    input clk ,
    input reset 
);

wire [31:0] pc_next ;
wire [31:0] pc ;
wire pc_en ;
wire [31:0] instr;
wire [5:0] opcode;
wire [4:0] rs ;
wire [4:0] rt ;
wire [4:0] rd ;
wire [4:0] shamt;
wire [5:0] funct;
wire [31:0] write_data;
wire [31:0] read_data1;
wire [31:0] read_data2;

wire [31:0] A ;
wire [31:0] B ;
wire [3:0] alu_ctrl ;
wire [31:0] result ;
wire zero;
wire [15:0] imm ;
wire [31:0] imm_ext;
wire IRwrite;
wire [31:0] instr_in ;
wire [31:0] data_in ;
wire [31:0] ALU_result ;
wire [31:0] alu_out ;
wire [31:0] mem_data ;
wire [31:0] data ;
wire [31:0] in ;
wire [31:0] out ;

wire mem_write ;
wire [31:0] addr ;
wire [31:0] read_data;
wire alusrca;
wire [31:0] srcA ;
wire [1:0] alusrcb ;
wire [31:0] sign_imm;
wire [31:0] sign_imm_shift2;
wire [31:0]srcB ;
wire [1:0] PCSrc;
wire [31:0] ALU_out;
wire [31:0] jump_addr ;
wire IorD;
wire [4:0] a3;
wire [1:0] ALUOp;
wire memtoreg ;
wire [31:0]wd3;
wire PCwrite;
wire Branch;
wire RegWrite;
wire reg_dst;

assign opcode = instr[31:26];
assign rs = instr[25:21];
assign rt = instr[20:16];
assign rd = instr[15:11];
assign shamt = instr[10:6];
assign funct = instr[5:0];
assign imm = instr[15:0];

assign instr_in = read_data;
assign mem_data = read_data;

assign data_in = read_data1;

assign sign_imm = imm_ext;
assign sign_imm_shift2 = imm_ext << 2;

assign pc_en = PCwrite | (Branch & zero);

assign jump_addr = { pc[31:28] , instr[25:0] , 2'b00 };

program_counter PC(clk , reset , pc_en , pc_next , pc );

reg_file regf(clk , RegWrite , rs , rt , a3 , wd3 ,
 read_data1 , read_data2);

rega rega(clk , read_data1 , A) ;

alu_control aluctrl(ALUOp , funct , alu_ctrl);

regb regb (clk , read_data2 , B);

alu_srca_mux amux(alusrca , pc , A , srcA);

alu_srcb_mux bmux(alusrcb , B , sign_imm , sign_imm_shift2 ,
 srcB);

alu alu(srcA , srcB , alu_ctrl , result , zero);

assign ALU_result = result;

ALUout aluout (clk, ALU_result , alu_out);

mdr mdr(clk , mem_data , data);

sign_extend sign (imm , imm_ext);

instr_reg instrcpu (clk, IRwrite , instr_in , instr);

instr_data id(clk , mem_write , addr , B , read_data);

x1 x1 (PCSrc , ALU_result , alu_out , jump_addr , pc_next);

x2 x2(IorD , pc , alu_out , addr);

x3 x3 (reg_dst , rt , rd , a3);

x4 x4 (memtoreg , data , alu_out , wd3);

ctrl_unit ctrlunit(clk , reset , opcode , zero ,
                    PCwrite , Branch , IRwrite , RegWrite ,
                    mem_write , IorD , alusrca , alusrcb ,
                    PCSrc , reg_dst , memtoreg , ALUOp
);

endmodule


// TESTBENCH 

// COMPLETELY GENERATED BY AI 




