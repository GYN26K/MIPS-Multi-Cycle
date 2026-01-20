`timescale 1ns/1ps

module cpu_tb;

    reg clk;
    reg reset;

    // Instantiate CPU
    cpu dut (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation (10ns period)
    always #5 clk = ~clk;

    initial begin
        clk = 0;
        reset = 1;

        #20;
        reset = 0;   // release reset

        #500;

        // Observe registers
        $display("R8 (t0)  = %d", dut.regf.regs[8]);
        $display("R9 (t1)  = %d", dut.regf.regs[9]);
        $display("R10 (t2) = %d", dut.regf.regs[10]);

        $finish;
    end

    // Initialize instruction memory (no integer, no loop)
    initial begin
        // Clear only first few words we use
        dut.id.mem[0] = 32'b0;
        dut.id.mem[1] = 32'b0;
        dut.id.mem[2] = 32'b0;
        dut.id.mem[3] = 32'b0;
        dut.id.mem[4] = 32'b0;

        /*
            Program:
            addi $t0, $zero, 5    -> 0x20080005
            addi $t1, $zero, 10   -> 0x2009000A
            add  $t2, $t0, $t1    -> 0x01095020
        */
        dut.id.mem[0] = 32'h20080005;
        dut.id.mem[1] = 32'h2009000A;
        dut.id.mem[2] = 32'h01095020;
    end

    // VCD dump
    initial begin
        $dumpfile("cpu.vcd");
        $dumpvars(0, cpu_tb);
    end

endmodule