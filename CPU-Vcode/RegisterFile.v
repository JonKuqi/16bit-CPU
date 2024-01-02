// Code your design here
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
    
reg[15:0] Registers[15:0];
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




//Testimi ne rregull

module RegisterFile_Test();
  
  reg[1:0] RS, RT, RD;
  reg RegWrite, Clock;
  reg[15:0] WriteData;
  wire[15:0] ReadRS, ReadRT;
  
  initial
    $monitor("RS=%b, RT=%b, RD=%b, RegWrite=%b, Clock=%b, WriteData=%d, ReadRS=%d, ReadRT=%d", RS, RT, RD, RegWrite, Clock, WriteData, ReadRS, ReadRT);
  initial
    begin
      $dumpfile("dump.vcd");
      $dumpvars;
      #0 Clock=1'b0;
      #5 RD=2'd4; WriteData = 15'd5; RegWrite=1'b1;
      #5 Clock=1'b1;
      #5 Clock=1'b0; RegWrite= 1'b1;
      #5 RD=2'd2; WriteData = 15'd7; RegWrite=1'b1;
      #5 Clock=1'b1;
      #5 Clock=1'b0; RegWrite=0;
      #5 RS=2'd4; RT=2'd2;
      #5 $finish;
    end
  
  RegisterFile RF(RS, RT, RD, WriteData, RegWrite, Clock, ReadRS, ReadRT);
  
endmodule
