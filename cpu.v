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