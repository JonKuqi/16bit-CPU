`timescale 1ns / 1ps


//Mundet me pase gabime, logjika besoj qe eshte e duhur

//ALU Control (CU_OUT_x, Function Code nga R-formati, Opcode, T19) - eshte shtuar ALUOp per I-format qe nuk eshte ne foto po kerkohet ne detyre
module ALUControl(


input [1:0] ALUOp,
input [1:0] Funct,
input [4:0] opcode,
output reg [3:0] ALUCtrl
);

always @(ALUOp)
begin
case(ALUOp) // Pyet per vleren e ALUOp, 00-lw,sw; 01-beq,bne, 10-R-format, 11 - I-format
    2'b00: ALUCtrl = 4'b0100; //LW+SW (mbledhje)
    2'b01: ALUCtrl = 4'b1100; //BEQ+BNE (zbritje)
    2'b10: //R-Format sipas FUNCT
	 case(opcode) //R-Format
          4'b0XXX:	 
		  case(opcode)
		  4'b0000:
           case(Funct)
            2'b00: ALUCtrl = 4'b0000; //AND
		    2'b01: ALUCtrl = 4'b0010; //OR
			2'b10: ALUCtrl = 4'b0011; //XOR
			endcase
		   4'b0001:
			 case(Funct)
            2'b00: ALUCtrl = 4'b0100; //ADD
			2'b01: ALUCtrl = 4'b1100; //SUB
			endcase
		   4'b0010:
		    case(Funct)
            2'b00: ALUCtrl = 4'b0110; //SLL
			2'b01: ALUCtrl = 4'b0111; //SRA
			endcase
	
        endcase
	2'b11:  //Ne baze te OPCODEVE I-format	
	 case(opcode)
          4'b1XXX:	
       case(opcode)
		4'b1001:
			 ALUCtrl = 4'b0101; //ADDI
		4'b1010:
			 ALUCtrl = 4'b1101; //SUBI
		4'b1011:
			 ALUCtrl = 4'b0001; //SLTI
		endcase
		
      endcase
	  	
 endcase
end

endmodule
