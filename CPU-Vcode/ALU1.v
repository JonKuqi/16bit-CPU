`timescale 1ns / 1ps


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
    
   wire JoA, JoB, mA, mB, dhe_teli, ose_teli, mb_teli, xor_teli, slt_teli, Bneg; 
   //mb teli -> teli i mbledhjes
   
   assign JoA = ~A;
   assign JoB = ~B;

  
   assign Bneg = (Op == 3'b001) ? 1'b1 : BInvert; 
   
   mux2ne1 muxA(A, JoA, AInvert, mA);
   mux2ne1 muxB(B, JoB, BInvert, mB);
   
   
   assign dhe_teli = mA & mB;
   assign ose_teli = mA | mB;
   assign xor_teli = mA ^ mB;
   assign slt_teli = 0;
   
   Mbledhesi m1(mA, mB, CIN, mb_teli, CarryOut);
   
   //mux4ne1 MuxiKryesor(dhe_teli, ose_teli, mb_teli, Less, Op, Result);
mux5ne1 MuxiKryesor(dhe_teli, slt_teli, ose_teli, xor_teli, mb_teli, Less, Op, Result); 

	
	//duhet mu ba ni Mux8ne1(dhe_teli, ose_teli, mb_teli, less, op, Result);
	//ky multiplekser ne baze te op code i cili te na eshte 3 bitesh zgjedh se cilat nga keto tela do aktivizohen
	//sh. 000 and, 010 XOR kto tre jan bitat e fundit te ALUCtrl
    
endmodule











//testimi i sakte? pa marre slti
module testALU1bit();

reg A, B, CIN, LESS, AInvert, BInvert;
  reg [2:0] Operation;
wire Result, COUT;

initial
$monitor("A=%b, B=%b, CIN=%b, LESS=%b, AInvert=%b, BInvert=%b, Operation=%b, Result=%b, COUT=%b", A, B, CIN, LESS, AInvert, BInvert, Operation, Result, COUT);


initial
begin
//AND
#0 A=1'b0; B=1'b0; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b000;
#10 A=1'b0; B=1'b1; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b000;
#10 A=1'b1; B=1'b0; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b000;
#10 A=1'b1; B=1'b1; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b000;
//OR
#10 A=1'b0; B=1'b0; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b010;
#10 A=1'b0; B=1'b1; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b010;
#10 A=1'b1; B=1'b0; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b010;
#10 A=1'b1; B=1'b1; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b010;
//ADD
#10 A=1'b0; B=1'b0; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b100;
#10 A=1'b0; B=1'b1; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b100;
#10 A=1'b1; B=1'b0; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b100;
#10 A=1'b1; B=1'b1; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b100;
#10 A=1'b1; B=1'b1; CIN=1'b1; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b100;
//SUB
#10 A=1'b0; B=1'b0; CIN=1'b1; LESS=1'b0; AInvert=1'b0; BInvert=1'b1; Operation=3'b100;
#10 A=1'b0; B=1'b1; CIN=1'b1; LESS=1'b0; AInvert=1'b0; BInvert=1'b1; Operation=3'b100;
#10 A=1'b1; B=1'b0; CIN=1'b1; LESS=1'b0; AInvert=1'b0; BInvert=1'b1; Operation=3'b100;
#10 A=1'b1; B=1'b1; CIN=1'b1; LESS=1'b0; AInvert=1'b0; BInvert=1'b1; Operation=3'b100;
//XOR
#10 A=1'b0; B=1'b0; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b011;
#10 A=1'b0; B=1'b1; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b011;
#10 A=1'b1; B=1'b0; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b011;
#10 A=1'b1; B=1'b1; CIN=1'b0; LESS=1'b0; AInvert=1'b0; BInvert=1'b0; Operation=3'b011;
#10 $finish; 
end

  ALU1 ALUTest(A, B, CIN, AInvert, BInvert, LESS, Operation, Result, COUT);

endmodule


