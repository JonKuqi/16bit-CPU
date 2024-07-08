# 16-Bit CPU in Verilog
This project implements a fully functional 16-bit CPU in Verilog, designed to support elementary operations.

## 16-bit Architecture
The CPU operates on 16-bit data and addresses.

## Instruction Set
Includes a set of elementary operations such as:
- addition,
- subtraction,
- logical operations (AND, OR, XOR),
- shift operations (left and right shifts),
- load/store operations,
- branching

## Components
Registers: Includes general-purpose registers and special-purpose registers like the program counter (PC) and status register.

Control Unit: Manages instruction decoding and execution flow.

ALU: Makes elementary operations

DataPath: Connects all thhe components together execpt Control Unit.

Shifter (Shamtuesi): Makes shifting operations on signed numbers left and right, multiplying or dividing with the numbers that are a power of 2.

Memory: Instruction Memory and DataMemory as separate modules.

CPU: Connects Control Unit and DataPath.

## Files
CPU-Vcode: Verilog module defining the CPU architecture, registers, ALU (Arithmetic Logic Unit), control unit, CPU, DataMemory, DataPath, InstructionMemory, Sumators, RegisterFile, Shifting, muxes.

Mem-Data: Data Memory and Instruction Memory. (.mem files)


## Schematic

![image](https://github.com/JonKuqi/16bit-CPU/assets/116517705/eb01996d-daa5-485b-9fee-b49703070ddd)
