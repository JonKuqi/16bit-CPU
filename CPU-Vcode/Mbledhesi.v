`timescale 1ns / 1ps
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