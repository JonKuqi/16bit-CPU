`timescale 1ns / 1ps

/

//Hyrje ne CPU - CLock CPU_IN_1
module CPU(input Clock);

//TELAT E BRENDSHEM TE CPU, SHIH CPU.PDF
wire [3:0] opcode; //D_OUT_1
//CU_OUT_x
wire RegDst, Jump, Branch, MemRead, MemWrite, RegWrite, MemToReg, ALUSrc;
wire [1:0] ALUOp;

//inicializimi i Datapath   

//Pa perfunduar!!
Datapath DP
(
Clock,
RegDst, Branch, MemRead, MemWrite, RegWrite, MemToReg, ALUSrc,
ALUOp,
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
RegWrite
);

endmodule
