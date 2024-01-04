`timescale 1ns / 1ps
//--------------------------------------------------------------------------------------
module ALU1(
    input A,
    input B,
    input CIN,
    input AInvert,
    input BInvert,
    input Less,
    input [2:0] Op,
    output Result,
    output CarryOut
    );
    
   wire JoA, JoB, mA, mB, dhe_teli, ose_teli, mb_teli, xor_teli, slt_teli; 
   //mb teli -> teli i mbledhjes
   
   assign JoA = ~A;
   assign JoB = ~B;
   
   mux2ne1 muxA(A, JoA, AInvert, mA);
   mux2ne1 muxB(B, JoB, BInvert, mB);
   
   
   assign dhe_teli = mA & mB;
   assign ose_teli = mA | mB;
   assign xor_teli = mA ^ mB;
   assign slt_teli = 0;
   
   Mbledhesi m1(mA, mB, CIN, mb_teli, CarryOut);
   
 
mux5ne1 MuxiKryesor(dhe_teli, slt_teli, ose_teli, xor_teli, mb_teli, Less, Op, Result); 

    
endmodule


module ALU16(
    input [15:0] A,
    input [15:0] B,
    input AInvert,
    input [3:0] Op,
    output Zero,
    output [15:0] Result,
    output Overflow,
    output CarryOut
    );
	
//CIN edhe Bin gjeth bashk po shkojn edhe e bajm ni bNegate t dyjat me ni ven
// less vjen nga set
    wire BNegate;
    wire [14:0] COUT;
    //LIDH 16 ALU 1-biteshe
	//pasi cout varet nga qdo alu, less ne qdo alu brenda eshte zero 
  
  
  assign BNegate = Op[3];
//(A,B,CIN,AInvet,BInvert,Less,OP,Result,CarryOut)
  ALU1 ALU0(A[0], B[0], BNegate, AInvert, BNegate, Result[15], Op[2:0], Result[0], COUT[0]);
    ALU1 ALU1(A[1], B[1], COUT[0], AInvert, BNegate, 0, Op[2:0], Result[1], COUT[1]);
    ALU1 ALU2(A[2], B[2], COUT[1], AInvert, BNegate, 0, Op[2:0], Result[2], COUT[2]);
	ALU1 ALU3(A[3], B[3], COUT[2], AInvert, BNegate, 0, Op[2:0], Result[3], COUT[3]);
    ALU1 ALU4(A[4], B[4], COUT[3], AInvert, BNegate, 0, Op[2:0], Result[4], COUT[4]);
    ALU1 ALU5(A[5], B[5], COUT[4], AInvert, BNegate, 0, Op[2:0], Result[5], COUT[5]);
    ALU1 ALU6(A[6], B[6], COUT[5], AInvert, BNegate, 0, Op[2:0], Result[6], COUT[6]);
    ALU1 ALU7(A[7], B[7], COUT[6], AInvert, BNegate, 0, Op[2:0], Result[7], COUT[7]);
    ALU1 ALU8(A[8], B[8], COUT[7], AInvert, BNegate, 0, Op[2:0], Result[8], COUT[8]);
    ALU1 ALU9(A[9], B[9], COUT[8], AInvert, BNegate, 0, Op[2:0], Result[9], COUT[9]);
    ALU1 ALU10(A[10], B[10], COUT[9], AInvert, BNegate, 0, Op[2:0], Result[10], COUT[10]);
    ALU1 ALU11(A[11], B[11], COUT[10], AInvert, BNegate, 0, Op[2:0], Result[11], COUT[11]);
    ALU1 ALU12(A[12], B[12], COUT[11], AInvert, BNegate, 0, Op[2:0], Result[12], COUT[12]);
    ALU1 ALU13(A[13], B[13], COUT[12], AInvert, BNegate, 0, Op[2:0], Result[13], COUT[13]);
    ALU1 ALU14(A[14], B[14], COUT[13], AInvert, BNegate, 0, Op[2:0], Result[14], COUT[14]);
    ALU1 ALU15(A[15], B[15], COUT[14], AInvert, BNegate, 0, Op[2:0], Result[15], CarryOut);
 
 
    
assign Zero = ~(Result[0] | Result[1] | 
                Result[2] | Result[3] | 
                Result[4] | Result[5] | 
                Result[6] | Result[7] | 
                Result[8] | Result[9] | 
                Result[10] | Result[11] | 
                Result[12] | Result[13] | 
                Result[14] | Result[15] ); 
                    
assign Overflow = COUT[14] ^ CarryOut;
	  
  //Overflow nese dy numra pozitiv jen mledh edhe ka dalnegativ, edhe kur dy numra negativ jen mledh ka dal pozitiv, veq do dy raste si veqori t komplementit te 2shit
  
 //Carry in dhe carry ou nuk perputhen pra kemi overflow
 
 //ideja, shamt behet si hyrje edhe ekziston ni case: per i cili merr ALUCtrl dhe nese esshte i njejte me ge SLL dhe SRA 
 //mbishkruhet rezultati
    
    
endmodule
//--------------------------------------------------------------------------------------
module ALUControl(
input [1:0] ALUOp,
input [1:0] Funct,
input [3:0] opcode,
output reg [3:0] ALUCtrl
);

always @(ALUOp, opcode, Funct)
begin
  case(ALUOp) // Pyet per vleren e ALUOp, 00-lw,sw; 01-beq,bne, 10-R-format, 11 - I-format
    2'b00: ALUCtrl = 4'b0100; //LW+SW (mbledhje)
    2'b01: ALUCtrl = 4'b1100; //BEQ+BNE (zbritje)
    2'b10: begin //R-Format sipas FUNCT 

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
     end
			   
	
         
	2'b11:  //Ne baze te OPCODEVE I-format	
	
       case(opcode)
		4'b1001:
			 ALUCtrl = 4'b0101; //ADDI
		4'b1010:
			 ALUCtrl = 4'b1101; //SUBI
		4'b1011:
			 ALUCtrl = 4'b0001; //SLTI
		
       endcase
	  	
     endcase
 end
endmodule
//--------------------------------------------------------------------------------------
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
  
  
//--------------------------------------------------------------------------------------
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
  
//--------------------------------------------------------------------------------------  
module RegisterFile(
input wire[1:0] RS,
input wire[1:0] RT,
input wire[1:0] RD,
input wire[15:0] WriteData,
input wire RegWrite,
input wire Clock,
//pasi element qe kena me shkru n ta ka clock
output wire[15:0] ReadRS,
output wire[15:0] ReadRT
    );
    
reg[15:0] Registers[3:0];
//16 regjista me ka 16 bit numri i bitve mas reg

//Reseto te gjithe regjistrat ne 0
integer i;
initial 
begin
for(i=0;i<16;i=i+1)
   Registers[i] <= 16'd0; 
end

//Shkruaj ne regjiter
always @(posedge Clock)
begin
if(RegWrite) Registers[RD] <= WriteData;
end

//lexo regjistrat ReadData1, ReadData2
//i qet ne dalje regjistrat rt dhe rs
assign ReadRS = Registers[RS];
assign ReadRT = Registers[RT];
endmodule
  
//--------------------------------------------------------------------------------------  
module DataMemory(
input wire[15:0] Address,
input wire[15:0] WriteData,
input wire MemWrite,
input wire MemRead,
input wire Clock,
output wire[15:0] ReadData
);

reg[7:0] dataMem[0:127];

initial
$readmemb("dataMemory.mem", dataMem);


//kur clock edhe memWrite, mujna me shkru
always@(posedge Clock)
begin
    if(MemWrite) 
        begin
            //bigEndian
            dataMem[Address + 32'd0] <= WriteData[15:8];
            dataMem[Address + 32'd1] <= WriteData[7:0];
           end
end

//per me shkru ne file, sban me shti n posedge clock se ndodhin do komplikime, prandaj e bajm me
//teh negativ
always@(negedge Clock)
begin
$writememb("dataMemory.mem", dataMem);
end

 
 assign ReadData[15:8] = dataMem[Address + 32'd0];
 assign ReadData[7:0] = dataMem[Address + 32'd1];
endmodule  
  
//--------------------------------------------------------------------------------------  
  module Datapath(
input Clock, //HYRJE NGA CPU - TELI CPU_IN_1
input RegDst, Branch, MemRead, 
MemWrite, RegWrite, MemToReg, ALUSrc, //HYRJET NGA CU - TELAT CU_OUT_x
input [1:0] ALUOp, //HYRJE NGA CU - TELAT CU_OUT_x
output [3:0] opcode //DALJE PER NE CU - TELI D_OUT_1
);

//SHIKO FOTO DATAPATH.PDF
reg[15:0] pc_initial; // Regjistri PC
wire [15:0] pc_next, pc4, pcbeq; //TELAT: T1, T2, T3, T4
wire [15:0] instruction; //TELI T5
wire [1:0] mux_regfile; //TELI T6
wire[15:0] readData1, readData2, writeData, //TELAT T7-T9 
mux_ALU, ALU_Out, Zgjerimi, memToMux, //TELAT T10-T13
shifter2beq, shifter2jump, branchAdderToMux, beqAddress, jumpAddress; //TELAT T14-T18 
wire[3:0] ALUCtrl; //TELI T19
wire zerof, overflow, carryout; // TELAT T20-T22
wire andMuxBranch; //TELI T23

initial
begin
    pc_initial = 16'd10; //inicializimi fillesat i PC ne adresen 10
end

always@(posedge Clock)
begin
    pc_initial <= pc_next; //azhurimi i PC ne cdo teh pozitiv me adresen e ardhshme
    
end

//T2 - PC rritet per 2 (ne sistemet 16 biteshe) per te gjitha instruksionet pervec BEQ
assign pc4 = pc_initial + 2; 



//DUHET BERE NJE MBLEDHES 





//T14 - pergatitja e adreses per kercim ne BEQ (164 bit si MSB, 8 bit nga pjesa imediate, 2 bit shtyrje majtas (x2) 
assign shifter2beq = {{7{instruction[7]}}, instruction[7:0], 1'b0};


//Instr mem //inicializimi i IM (PC adresa hyrje, teli instruction dajle)
InstructionMemory IM(pc_initial, instruction); 

//T6 - Percaktimi nese RD eshte RD (te R-formati) apo RD = RT (te I-formati) - MUX M1 ne foto
assign mux_regfile = (RegDst == 1'b1) ? instruction[7:6] : instruction[9:8]; 

// T12 - Zgjerimi nga 8 ne 16 bit - 16 bit si MSB dhe pjesa e instruction[15:0] - S1 ne foto
assign Zgjerimi = {{8{instruction[7]}}, instruction[7:0]};  

//REGFILE
//inicializimi i RF(RS, RT, T6 (RD[RD=RD || RD=RT]), T9, CU_OUT_x, CPU_IN_1, T7, T8)
RegisterFile RF(instruction[11:10], instruction[9:8], mux_regfile, writeData, RegWrite, Clock, readData1, readData2 ); 

// T10 - Percaktimi nese hyrja e MUX-it M2 para ALU eshte Regjstri 2 i RF apo vlera imediate e instruksionit 
assign mux_ALU = (ALUSrc == 1'b1) ? Zgjerimi : readData2; 

//inicializimi i ALU Control (CU_OUT_x, Function Code nga R-formati, Opcode, T19) - eshte shtuar ALUOp per I-format qe nuk eshte ne foto po kerkohet ne detyre 
ALUControl AC(ALUOp, instruction[1:0], instruction[15:12], ALUCtrl); 

//inicializimi i ALU (T7, T10, T19[3], T19[2], T19[1:0], T20, T11, T21, T22)
ALU16 ALU(readData1, mux_ALU, 0, ALUCtrl[3:0], zerof, ALU_Out, overflow, carryout);




//inicializimi i Data Memory (T11, T8, CU_OUT_x, CU_OUT_x, CPU_IN_1, T13) 
DataMemory DM(ALU_Out, readData2, MemWrite, MemRead, Clock, memToMux);

//T9 - Teli qe i dergon te dhenat nga MUX - M3 ne Regfile
assign writeData = (MemToReg == 1'b1) ? memToMux : ALU_Out;

//T23 - Teli qe del nga porta DHE ne pjesen e eperme te fotos (shikon nese plotesohet kushti per BEQ
assign andMuxBranch = zerof & Branch;

//T17, Teli qe mban adresen ne te cilen do te kercej programi kur kushti BEQ plotesohet
assign beqAddress = pc4 + shifter2beq; 

//T3 - Teli qe del nga Mux M4 ne foto qe kontrollon nese kemi BEQ apo PC+4
assign pcbeq = (andMuxBranch == 1'b1) ? beqAddress : pc4;



//Teli D_OUT_1 qe i dergohet CU
assign opcode = instruction[15:12];

endmodule
  //--------------------------------------------------------------------------------------
module InstructionMemory(
input wire[15:0] PCAddress,
output wire[15:0] Instruction);

//deklarohen sa adresa i kemi dhe sa bitshe jane ato adresa
reg[7:0] instrMem[127:0];
//me ka 4 bit
//lexohen nga jashte
initial
$readmemb("instructionMemory.mem", instrMem);
//me b mas readmem pasi file i shkrum ne binar


//dekoder
  assign Instruction[15:8] = instrMem[PCAddress];
  assign Instruction[7:0] = instrMem[PCAddress + 32'd1];

endmodule


//--------------------------------------------------------------------------------------
module Mbledhesi(
    input A,
    input B,
    input CIN,
    output Shuma,
    output COUT
    );
    
    assign Shuma = A ^ B ^ CIN;
    assign COUT = CIN & A | CIN & B | A & B;
endmodule
  
 //-------------------------------------------------------------------------------------- 
  module mux2ne1(
    input Hyrja0,
    input Hyrja1,
    input S,
    output Dalja
    );
    
    assign Dalja = S ? Hyrja1 : Hyrja0;
endmodule

  //--------------------------------------------------------------------------------------
  module mux5ne1(
    input oAND,
	input oSLTI,
	input oOR,
	input oXOR,
	input oADDSUB,
	input Less,
	input [2:0]AluCtrl,
	output Dalja
);
assign Dalja = AluCtrl[2] ? (AluCtrl[1] ?  Less : oADDSUB) : (AluCtrl[1] ? (AluCtrl[0] ? oXOR : oOR) : (AluCtrl[0] ? oSLTI : oAND)); 

endmodule








//Testingu

// Code your testbench here
// or browse Examples
`timescale 1ns / 1ps

module cputest();


reg Clock;

integer i;
initial
begin
for(i=0; i < 30; i=i+1) //30x nderro nga Clock 0 - 1, 30 tehe pozitive
begin
#10 Clock = 0;
#10 Clock = 1;
end

#10 $finish;
end



CPU cpu32(Clock);
endmodule
