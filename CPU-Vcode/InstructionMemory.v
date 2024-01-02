`timescale 1ns / 1ps



module InstructionMemory(
input wire[15:0] PCAddress,
output wire[15:0] Instruction);

//deklarohen sa adresa i kemi dhe sa bitshe jane ato adresa
reg[3:0] instrMem[127:0];
//me ka 4 bit
//lexohen nga jashte
initial
$readmemb("instructionMemory.mem", instrMem);
//me b mas readmem pasi file i shkrum ne binar


//dekoder
  assign Instruction[15:12] = instrMem[PCAddress];
  assign Instruction[11:8] = instrMem[PCAddress + 16'd1];
  assign Instruction[7:4] = instrMem[PCAddress + 16'd2];
assign Instruction[3:0] = instrMem[PCAddress + 16'd3];


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