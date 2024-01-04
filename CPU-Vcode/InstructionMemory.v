`timescale 1ns / 1ps

//komponenta eshte perfuduar dhe testuar, file testimit me poshte.

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


//E testuar, eshte korrekte
//pjesa e testimit 

module InstructionMemory_Test();
  reg[15:0] PC;
  wire[15:0] Instruction;
  
  initial
    $monitor("PC=%b, Instruction=%b", PC, Instruction);
  initial
    begin
      $dumpfile("dump.vcd");
      $dumpvars;
      #0 PC=16'd0;
    end
  
  InstructionMemory IM(PC, Instruction);
  
endmodule