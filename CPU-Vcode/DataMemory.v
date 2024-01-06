`timescale 1ns / 1ps
//Komponenta e kryer dhe e testuar, file i testimit me poshte.

module DataMemory(
input wire[15:0] Address,
input wire[15:0] WriteData,
input wire MemWrite,
input wire MemRead,
input wire Clock,
output wire[15:0] ReadData
);

reg[7:0] dataMem[127:0];
//radhitja mundet mu kan gabim!!

initial
$readmemb("dataMemory.mem", dataMem);


//kur clock edhe memWrite, mujna me shkru
always@(posedge Clock)
begin
    if(MemWrite) 
        begin
            //bigEndian
            dataMem[Address + 16'd0] <= WriteData[15:8];
            dataMem[Address + 16'd1] <= WriteData[7:0];
           end
end

//per me shkru ne file, sban me shti n posedge clock se ndodhin do komplikime, prandaj e bajm me
//teh negativ
always@(negedge Clock)
begin
$writememb("dataMemory.mem", dataMem);
end

 
 assign ReadData[15:8] = dataMem[Address + 16'd0];
 assign ReadData[7:0] = dataMem[Address + 16'd1];
endmodule


//Testimni

module DataMemory_Test();
  reg[15:0] Address;
  reg[15:0] WriteData;
  reg MemWrite;
  reg MemRead;
  reg Clock;
  wire[15:0] ReadData;
  
  
  initial
    $monitor("Address=%0b, WriteData=%0b, MemWrite=%b, MemRead=%b, Clock=%b, ReadData=%0b", Address, WriteData, MemWrite, MemRead, Clock, ReadData);
  
  initial
    begin
      #0 Clock=1'b0;
      #10 MemWrite=1'b1; Address=16'd2; WriteData=16'h1234; MemRead=1'b0;
      #10 Clock=1'b1;
      #10 Clock=1'b0; MemWrite=1'b0;
      #10 MemRead=1'b1; Address=16'd2;
      #10 $finish;
      
    end
  
  DataMemory DM(Address, WriteData, MemWrite, MemRead, Clock, ReadData);
  
endmodule


//nuk update file aty per aty po duhet me kliku "Download files after run" tani aty t del file i nreqen