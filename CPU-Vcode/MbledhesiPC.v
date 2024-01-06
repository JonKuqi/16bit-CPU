module MbledhesiPC(
input [15:0] pc_initial,
input [15:0] nr2,
output [15:0] pc2,
output COUT
);

wire [15:0] carry;


Mbledhesi m1(pc_initial[0],1'b0,1'b0,pc2[0],carry[0]);
Mbledhesi m2(pc_initial[1],1'b0,carry[0],pc2[1],carry[1]);
Mbledhesi m3(pc_initial[2],1'b0,carry[1],pc2[2],carry[2]);
Mbledhesi m4(pc_initial[3],1'b0,carry[2],pc2[3],carry[3]);
Mbledhesi m5(pc_initial[4],1'b0,carry[3],pc2[4],carry[4]);
Mbledhesi m6(pc_initial[5],1'b0,carry[4],pc2[5],carry[5]);
Mbledhesi m7(pc_initial[6],1'b0,carry[5],pc2[6],carry[6]);
Mbledhesi m8(pc_initial[7],1'b0,carry[6],pc2[7],carry[7]);
Mbledhesi m9(pc_initial[8],1'b0,carry[7],pc2[8],carry[8]);
Mbledhesi m10(pc_initial[9],1'b0,carry[8],pc2[9],carry[9]);
Mbledhesi m11(pc_initial[10],1'b0,carry[9],pc2[10],carry[10]);
Mbledhesi m12(pc_initial[11],1'b0,carry[10],pc2[11],carry[11]);
Mbledhesi m13(pc_initial[12],1'b0,carry[11],pc2[12],carry[12]);
Mbledhesi m14(pc_initial[13],1'b0,carry[12],pc2[13],carry[13]);
Mbledhesi m15(pc_initial[14],1'b1,carry[13],pc2[14],carry[14]);
Mbledhesi m16(pc_initial[15],1'b0,carry[14],pc2[15],COUT);


end module




