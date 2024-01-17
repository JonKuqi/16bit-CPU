module Mbledhesi16bit(
input [15:0] A,
input [15:0] B,
output [15:0] SUM,
output COUT
);

wire [14:0] carry;


Mbledhesi m1 (A[0],B[0],1'b0,SUM[0],carry[0]);
Mbledhesi m2 (A[1],B[1],carry[0],SUM[1],carry[1]);
Mbledhesi m3 (A[2],B[2],carry[1],SUM[2],carry[2]);
Mbledhesi m4 (A[3],B[3],carry[2],SUM[3],carry[3]);
Mbledhesi m5 (A[4],B[4],carry[3],SUM[4],carry[4]);
Mbledhesi m6 (A[5],B[5],carry[4],SUM[5],carry[5]);
Mbledhesi m7 (A[6],B[6],carry[5],SUM[6],carry[6]);
Mbledhesi m8 (A[7],B[7],carry[6],SUM[7],carry[7]);
Mbledhesi m9 (A[8],B[8],carry[7],SUM[8],carry[8]);
Mbledhesi m10 (A[9],B[9],carry[8],SUM[9],carry[9]);
Mbledhesi m11 (A[10],B[10],carry[9],SUM[10],carry[10]);
Mbledhesi m12 (A[11],B[11],carry[10],SUM[11],carry[11]);
Mbledhesi m13 (A[12],B[12],carry[11],SUM[12],carry[12]);
Mbledhesi m14 (A[13],B[13],carry[12],SUM[13],carry[13]);
Mbledhesi m15 (A[14],B[14],carry[13],SUM[14],carry[14]);
Mbledhesi m16 (A[15],B[15],carry[14],SUM[15],COUT);

endmodule
















//Testimi

module testMbledhesi16bit();

  reg [15:0] A;
  reg [15:0] B;
  wire [15:0] SUM;
  wire COUT;
 
initial
  $monitor("A=%b, B=%b, SUM=%b, COUT=%b", A, B, SUM, COUT);
initial
begin
#0 A=16'd10; B=16'd9;
#10 A=16'd20; B=16'd9;
#10 A=16'd15; B=16'd9;
#10 $stop;
end
           
Mbledhesi16bit Mbledhesi16bitTest(A, B, SUM, COUT);

endmodule