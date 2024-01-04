`timescale 1ns / 1ps

//Perfunduar si komponente, Eshte testuar me sukses!

`timescale 1ns / 1ps

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
4'b0000: //PER AND, OR, XOR
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
	
4'b0001: //ADD dhe SUB
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
	
4'b1001: //ADDI
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
	
4'b1010: //SUBI
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
	
4'b1011: //SLTI
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
	
4'b1100: //LW
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
4'b1101: //SW
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
4'b1111: //BEQ
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
    end
4'b0010: //SLL dhe SRA
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





//TESTIMI


`timescale 1ns / 1ps

module Test_CU();
  reg[3:0] opcode;
  wire RegDst, Branch, MemRead, MemToReg, MemWrite, AluSrc,RegWrite;
  wire[1:0] AluOp;
  
  initial
    $monitor("opcode=%b, RegDst=%b, Branch=%b, MemRead=%b, MemToReg=%b, MemWrite=%b , AluSrc=%b, RegWrite=%b,  AluOp=%b", opcode, RegDst, Branch, MemRead, MemToReg, MemWrite, AluSrc, RegWrite, AluOp);
  
  initial
    begin
      #0 opcode=4'b0000; 
      #5 opcode=4'b0001; 
      #5 opcode=4'b1001;
      #5 opcode=4'b1010;
      #5 opcode=4'b1011;
      #5 opcode=4'b1100;
      #5 opcode=4'b1101;
      #5 opcode=4'b1111;
      #5 opcode=4'b0010;
      
      #5 $finish;
    end
  
  
  CU CONTROL(opcode, RegDst, Branch, MemRead, MemToReg, AluOp,  MemWrite, AluSrc, RegWrite);
  
endmodule
