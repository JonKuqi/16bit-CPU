`timescale 1ns / 1ps

//Perfunduar si komponente, nuk eshte testuar?

module CU(
    input [3:0] OPCODE, //HYRJA NGA D_OUT_1
    output reg RegDst, //DALJET E CU, CU_OUT_x
    output reg Branch,
    output reg MemRead,
    output reg MemToReg,
    output reg[1:0] AluOp,
    output reg MemWrite,
    output reg AluSrc,
    output reg RegWrite
    );
    
    
always @ (OPCODE)
begin
//dekoderi
case(OPCODE)
6'b0000: //PER AND, OR, XOR
    begin
    RegDst = 1;
    AluSrc = 0;
	MemToReg = 0;
	RegWrite = 1;
	MemRead = 0;
	MemWrite = 0;
	AluOp[1] = 1;
    AluOp[0] = 0;	
	Branch =0;
    end
	
6'b0001: //ADD dhe SUB
    begin
    RegDst = 1;
    AluSrc = 0;
	MemToReg = 0;
	RegWrite = 1;
	MemRead = 0;
	MemWrite = 0;
	AluOp[1] = 1;
    AluOp[0] = 0;	
	Branch =0;
    end
	
6'b1001: //ADDI
    begin
    RegDst = 0;
    AluSrc = 1;
	MemToReg = 0;
	RegWrite = 1;
	MemRead = 0;
	MemWrite = 0;
	AluOp[1] = 1;
    AluOp[0] = 1;	
	Branch =0;
    end
	
6'b1010: //SUBI
    begin
    RegDst = 0;
    AluSrc = 1;
	MemToReg = 0;
	RegWrite = 1;
	MemRead = 0;
	MemWrite = 0;
	AluOp[1] = 1;
    AluOp[0] = 1;	
	Branch =0;
    end	
	
6'b1011: //SLTI
    begin
    RegDst = 0;
    AluSrc = 1;
	MemToReg = 0;
	RegWrite = 1;
	MemRead = 0;
	MemWrite = 0;
	AluOp[1] = 1;
    AluOp[0] = 1;	
	Branch =0;
    end	
	
6'b1100: //LW
    begin
    RegDst = 0;
    AluSrc = 1;
	MemToReg = 1;
	RegWrite = 1;
	MemRead = 1;
	MemWrite = 0;
	AluOp[1] = 0;
    AluOp[0] = 0;	
	Branch =0;
    end	
6'b1101: //SW
    begin
    RegDst = 0;
    AluSrc = 1;
	MemToReg = 0;
	RegWrite = 0;
	MemRead = 0;
	MemWrite = 1;
	AluOp[1] = 0;
    AluOp[0] = 0;	
	Branch =0;
    end	
6'b1111: //BEQ
    begin
    RegDst = 0;
    AluSrc = 0;
	MemToReg = 0;
	RegWrite = 0;
	MemRead = 0;
	MemWrite = 0;
	AluOp[1] = 0;
    AluOp[0] = 1;	
	Branch = 1;
6'b0010: //SLL dhe SRA
    begin
    RegDst = 1;
    AluSrc = 0;
	MemToReg = 0;
	RegWrite = 1;
	MemRead = 0;
	MemWrite = 0;
	AluOp[1] = 1;
    AluOp[0] = 0;	
	Branch = 0;
    end	
endcase

end

endmodule
