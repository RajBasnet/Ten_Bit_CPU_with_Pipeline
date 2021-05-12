`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/04/2020 08:06:54 PM
// Design Name: 
// Module Name: ALU_module
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


module ALU_module(input ALU_Control, input [9:0] A, input [9:0] B, output wire [9:0] ALU_output);
                
                reg Cin;
                initial begin
                    Cin = 0;
                end
                
                wire Cout, C0;
                wire [9:0] Sum;
                
                adder_10 ALU1(A, B, Cin, Sum, Cout);
                grt_10 ALU2(A, B, C0);
                
                mux ALU_mux(.A1(Sum), .A2({000000000,C0}), .Sel(ALU_Control), .Y(ALU_output));
endmodule

module mux(input [9:0] A1, input [9:0] A2, input Sel, output reg [9:0] Y);

    always @(Sel, A1, A2)
        begin
            if (Sel == 0) 
            begin
                Y <= A1;
            end
            else if (Sel == 1)
            begin 
                Y <= A2;
            end
        end
endmodule

module adder(input A, input B, input Cin, output Sum, output Cout);
        
        wire w1, w2, w3;
        and(w1, A, B);
        and(w2, A, Cin);
        and (w3, B, Cin);
        or(Cout, w1, w2, w3);
        xor(Sum, A, B, Cin);

endmodule


module adder_10(input [9:0] A, input [9:0] B, input Cin, output [9:0] Sum, output Cout);

        wire [9:1] C;
        adder adder0 (A[0], B[0], Cin, Sum[0], C[1]),
              adder1 (A[1], B[1], C[1], Sum[1], C[2]),
              adder2 (A[2], B[2], C[2], Sum[2], C[3]),
              adder3 (A[3], B[3], C[3], Sum[3], C[4]),
              adder4 (A[4], B[4], C[4], Sum[4], C[5]),
              adder5 (A[5], B[5], C[5], Sum[5], C[6]),
              adder6 (A[6], B[6], C[6], Sum[6], C[7]),
              adder7 (A[7], B[7], C[7], Sum[7], C[8]),
              adder8 (A[8], B[8], C[8], Sum[8], C[9]),
              adder9 (A[9], B[9], C[9], Sum[9], Cout);
        
endmodule

module grt_10(input [9:0] A, input [9:0] B, output C0);
    
    wire G1, G2; 
    wire x1, x2, x3, x4;
    wire S1, S2, S3, S4, S2_;
    wire A9_, B9_;
    
    assign G1 = A > B;
    assign G2 = A < B;
    
    not(A9_, A[9]);
    not(B9_, B[9]);
    
    and(S1, A[9], B[9]);
    or(S2, A[9], B[9]);
    and(S3, A[9], B9_);
    and(S4, A9_, B[9]);
    
    not(S2_, S2);
    
    and(x1, S2_, G1);
    and(x2, S1, G2);
    and(x3, S3, 0);
    and(x4, S4, 1);
    
    or(C0, x1, x2, x3, x4);
   
endmodule