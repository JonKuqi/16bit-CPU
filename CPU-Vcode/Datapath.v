`timescale 1ns / 1ps


module DatapathV2(
input Clock, 
input RegDst, Branch, MemRead, 
MemWrite, RegWrite, MemToReg, ALUSrc, Shift,
input [1:0] ALUOp, 
output [3:0] opcode 
);


reg[15:0] pc_initial; 
wire [15:0] pc_next, pc2, pcbeq; 
wire [15:0] instruction;


wire [1:0] mux_RtRd; //
wire[15:0] readData1, readData2, writeData, 
mux_ALU_RtImmediate, ALU_Rezultati, ZgjerimiImediates, memToMux, 
shifter2beq, branchAdderToMux, beqAddress, shiftingResult, mux_AluShift; 
wire[3:0] ALUCtrl; 
wire zerof, overflow, carryout;
wire andBranch; 

initial
begin
    pc_initial = 16'd10; //inicimi fillestar adresa 10
end

always@(posedge Clock)
begin
    pc_initial <= pc_next; 
    
end

Mbledhesi16bit mbledhesiPC (pc_initial,16'b10,SUM,COUT);
assign pc2 = SUM;

assign shifter2beq = {{7{instruction[7]}}, instruction[7:0], 1'b0}; // tu e shumzu me dy


InstructionMemory IM(pc_initial, instruction); 

assign mux_RtRd = (RegDst == 1'b1) ? instruction[7:6] : instruction[9:8]; 
//varesisht nga funksioni e qon si write register RT apo RD


assign ZgjerimiImediates = {{8{instruction[7]}}, instruction[7:0]};  



RegisterFile RF(instruction[11:10], instruction[9:8], mux_RtRd, writeData, RegWrite, Clock, readData1, readData2 ); 
 
assign mux_ALU_RtImmediate = (ALUSrc == 1'b1) ? ZgjerimiImediates : readData2; 
//varesisht R format apo I format 

ALUControl AC(ALUOp, instruction[1:0], instruction[15:12], ALUCtrl); 

ALU16 ALU(readData1, mux_ALU_RtImmediate, 1'b0, ALUCtrl, zerof, ALU_Rezultati, overflow, carryout);


Shifting SH(readData1, instruction[5:2], instruction[1:0], shiftingResult);

assign mux_AluShift = (Shift == 1'b1) ? shiftingResult : ALU_Rezultati;

DataMemory DM(mux_AluShift, readData2, MemWrite, MemRead, Clock, memToMux);

assign writeData = (MemToReg == 1'b1) ? memToMux : mux_AluShift;

assign andBranch = zerof & Branch;

Mbledhesi16bit mbledhesiBeq (pc2,shifter2beq,SUM,COUT);
assign beqAddress = SUM;

assign pc_next = (andBranch == 1'b1) ? beqAddress : pc2;

assign opcode = instruction[15:12];

endmodule