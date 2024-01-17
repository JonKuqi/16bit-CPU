`timescale 1ns / 1ps

module Shifting(
  input[15:0] Hyrja,
  input[3:0] Shamt,
  input[1:0] Funct,
  output reg[15:0] Result
);
  
  always @*
    case(Funct)
      2'b00: Result = Hyrja << Shamt;
      
      2'b01: Result = Hyrja >> Shamt;
    endcase
  
endmodule










//Testing


module Shifting_Test();
  reg[15:0] Hyrja;
  reg[3:0] Shamt;
  reg[1:0] Funct;
  wire[15:0] Result;
 
  
  initial
    $monitor("Hyrja=%b, Shamt=%b, Funct=%b, Result=%b", Hyrja, Shamt, Funct, Result);
  
  initial
    begin
      #0 Hyrja=16'd1; Shamt=4'b0010; Funct=2'b00; 
      #5 Hyrja=16'b0000010000000000; Shamt=4'b0110; Funct=2'b01; 
     
    end
  
  
  Shifting SH(Hyrja, Shamt, Funct, Result);
  
endmodule