`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/04/2020 08:09:14 PM
// Design Name: 
// Module Name: ALU_test
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


module ALU_test();

reg ALU_Control; reg [9:0] A; reg [9:0] B;
wire [9:0] ALU_output; 
    
    ALU_module ALU_test(.ALU_Control(ALU_Control), .A(A), .B(B), .ALU_output(ALU_output));
      
    initial
        begin
            
            A = 10'b0011010111;
            B = 10'b0000101110;
            ALU_Control = 0;
             
            #100
            A = 10'b1110101110;
            B = 10'b0100111110;
            ALU_Control = 0;
            
            #100
            A = 10'b1110101110;
            B = 10'b1100111110;
            ALU_Control = 0;
            
            #100
            A = 10'b1010101010;
            B = 10'b1100111110;
            ALU_Control = 1;
            
            #100
            A = 10'b0011001010;
            B = 10'b0000110101;
            ALU_Control = 1;
            
            #100
            A = 10'b1110101110;
            B = 10'b0100111110;
            ALU_Control = 1;
            
        end
endmodule
