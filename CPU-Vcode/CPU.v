`timescale 1ns / 1ps
module CPU(input Clock);


wire [3:0] opcode;
wire RegDst, Branch, MemRead, MemWrite, RegWrite, MemToReg, ALUSrc;
wire [1:0] ALUOp;


//Pa perfunduar!!
DatapathV2 DP
(
Clock,
RegDst, Branch, MemRead, MemWrite, RegWrite, MemToReg, ALUSrc, Shift, ALUOp,
opcode
);

//Inicializimi i COntrol Unit
//Perfunduar
CU ControlUnit(opcode,
RegDst,  
Branch, 
MemRead, 
MemToReg,
ALUOp,
MemWrite, 
ALUSrc,
RegWrite,
Shift
);

endmodule

