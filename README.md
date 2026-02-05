# MIPS Multi-Cycle Processor (Harris & Harris Architecture)

## Overview

This project implements a **Multi-Cycle MIPS Processor** based on the architecture described in:

> David A. Patterson & John L. Hennessy style adapted in  
> **"Digital Design and Computer Architecture" by David Harris and Sarah Harris**

The design follows the exact multi-cycle datapath and control methodology presented in the Harris & Harris textbook. 
Each instruction executes over multiple clock cycles using a finite state machine (FSM) controller and a shared datapath.

The processor supports a subset of the MIPS32 instruction set as described in the book.

---

## Architecture Reference

This implementation follows:

- Multi-cycle datapath from Harris & Harris
- Moore FSM-based control unit
- Unified memory (instruction + data)
- Separate registers for intermediate storage:
  - IR
  - MDR
  - A
  - B
  - ALUOut

---

## Multi-Cycle Operation

Each instruction is broken into five conceptual stages:

1. Instruction Fetch (IF)
2. Instruction Decode / Register Fetch (ID)
3. Execute / Address Calculation (EX)
4. Memory Access (MEM)
5. Write Back (WB)

Unlike a single-cycle design:
- Functional units are reused across cycles
- CPI > 1
- Hardware cost is reduced compared to single-cycle

---

## Datapath Components (As per Harris & Harris)

### Core Registers
- PC (Program Counter)
- IR (Instruction Register)
- MDR (Memory Data Register)
- A (Register File Read Data 1 latch)
- B (Register File Read Data 2 latch)
- ALUOut (ALU result register)

### Functional Units
- Register File (32 × 32-bit)
- ALU
- Sign Extend
- Shift Left 2 (for branch offset)
- Unified Memory

### Multiplexers
- IorD
- RegDst
- MemtoReg
- ALUSrcA
- ALUSrcB
- PCSource

---

## Supported Instructions (Harris Subset)

### R-Type
- add
- sub
- and
- or
- slt

### I-Type
- lw
- sw
- beq
- addi

### J-Type
- j

All instruction formats follow standard MIPS encoding.

---

## Control Unit (FSM)

The control unit is implemented as a Moore finite state machine exactly as shown in Harris & Harris.

### Main States

| State | Description |
|-------|------------|
| S0 | Instruction Fetch |
| S1 | Instruction Decode |
| S2 | Memory Address Computation (lw/sw) |
| S3 | Memory Read (lw) |
| S4 | Memory Write (sw) |
| S5 | Write Back (lw) |
| S6 | R-type Execute |
| S7 | R-type Write Back |
| S8 | Branch (beq) |
| S9 | Jump (j) |

---

## Control Signals (As in Book)

- PCWrite
- PCWriteCond
- IorD
- MemRead
- MemWrite
- IRWrite
- MemtoReg
- RegDst
- RegWrite
- ALUSrcA
- ALUSrcB
- PCSource
- ALUOp



