`timescale 1ns / 1ps

//Testuar me sukses!!

module ALU16(
    input [15:0] A,
    input [15:0] B,
    input AInvert,
    input [3:0] Op,
    output Zero,
    output [15:0] FinalResult,
    output Overflow,
    output CarryOut
    );
	
//CIN edhe Bin gjeth bashk po shkojn edhe e bajm ni bNegate t dyjat me ni ven
// less vjen nga set
    wire BNegate;
    wire [14:0] COUT;
    wire [15:0] slti_teli;
    
  wire [15:0]Result;
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
	  assign slti_teli = {{15{1'b0}}, Result[15]};
   assign FinalResult = (Op == 4'b0001) ? slti_teli : Result;  
  

  //Overflow nese dy numra pozitiv jen mledh edhe ka dalnegativ, edhe kur dy numra negativ jen mledh ka dal pozitiv, veq do dy raste si veqori t komplementit te 2shit
  
 //Carry in dhe carry ou nuk perputhen pra kemi overflow
 
 //ideja, shamt behet si hyrje edhe ekziston ni case: per i cili merr ALUCtrl dhe nese esshte i njejte me ge SLL dhe SRA 
 //mbishkruhet rezultati
endmodule



//Testimi




`timescale 1ns / 1ps
module testALU16bit();

  reg [15:0]A, B;
  reg AInvert;
  reg [3:0] Op;
  wire[15:0] Result;
  wire Overflow, CarryOut, Zero;

initial
  $monitor("A=%b, B=%b, AInvert=%b, Op=%b, Result=%b, CarryOut=%b, Zero=%b, Overflow=%b" , A, B, AInvert, Op, Result, CarryOut, Zero, Overflow);


initial
begin
//AND
#0 A=16'd100; B=16'd85; AInvert=1'b0; Op=4'b0100; //ADD
#10 A=16'd100; B=16'd95; AInvert=1'b0; Op=4'b1100; //SUB
#10 A=16'b1001001001011111; B=16'b1001001001011111; AInvert=1'b0; Op=4'b0000;  //AND
 #10 A=16'b1001001001011111; B=16'b0000000000000000; AInvert=1'b0; Op=4'b0010;
  #10 A=16'b1001001001011111; B=16'b1001001001011111; AInvert=1'b0; Op=4'b0011;
#10 $finish; 
end

  ALU16 ALUTest(A, B, AInvert, Op, Zero, Result, Overflow, CarryOut);

endmodule


  
  