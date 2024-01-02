`timescale 1ns / 1ps


module ALU1(
    input A,
    input B,
    input CIN,
    input AInvert,
    input BInvert,
    input Less,
    input [1:0] Op,
    output Result,
    output CarryOut
    );
    
   wire JoA, JoB, mA, mB, dhe_teli, ose_teli, mb_teli; 
   
   assign JoA = ~A;
   assign JoB = ~B;
   
   mux2ne1 muxA(A, JoA, AInvert, mA);
   mux2ne1 muxB(B, JoB, BInvert, mB);
   
   assign dhe_teli = mA & mB;
   assign ose_teli = mA | mB;
   
   Mbledhesi m1(mA, mB, CIN, mb_teli, CarryOut);
   
   mux4ne1 MuxiKryesor(dhe_teli, ose_teli, mb_teli, Less, Op, Result);
    
    
endmodule
