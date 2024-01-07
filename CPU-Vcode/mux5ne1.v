`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/22/2022 05:30:27 PM
// Design Name: 
// Module Name: mux4ne1
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module mux5ne1(
    input oAND,
	input oSLTI,
	input oOR,
	input oXOR,
	input oADDSUB,
	input Less,
	input [2:0]AluCtrl,
	output Dalja
);
assign Dalja = AluCtrl[2] ? (AluCtrl[1] ?  Less : oADDSUB) : (AluCtrl[1] ? (AluCtrl[0] ? oXOR : oOR) : (AluCtrl[0] ? oSLTI : oAND)); 

endmodule


module mux2ne1(
    input Hyrja0,
    input Hyrja1,
    input S,
    output Dalja
    );
    
    assign Dalja = S ? Hyrja1 : Hyrja0;
endmodule



