`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/04/2020 10:42:24 PM
// Design Name: 
// Module Name: DataMemory
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


module DataMemory(input clk, input [3:0] address, input [9:0] write_data, input write_en, input mem_read, 
                    output [9:0] read_data);
        integer i;
        reg [9:0] ram [15:0];
        wire [3:0] ram_address = address;
        reg [1:0] program;
        
        initial begin
            program = 2'b10;
            case(program)
            2'b11:
                for(i = 0; i < 16; i=i+1)
                if (i == 0 || i == 1)
                    begin
                        ram[0] <= 10'b0000001000;
                        ram[1] <= 10'b0000000101;
                    end
                else
                    begin
                        ram[i] <= 10'b0000000000;
                    end
            2'b10:
                for(i = 0; i < 16; i=i+1)
                if (i < 8)
                    begin
                        ram[0] <= 10'b0000001010;
                        ram[1] <= 10'b0000001111;
                        ram[2] <= 10'b0000001100;
                        ram[3] <= 10'b0000001011;
                        ram[4] <= 10'b0000001101;
                        ram[5] <= 10'b0000001111;
                        ram[6] <= 10'b0000000111;
                        ram[7] <= 10'b0000000000;
                    end
                else
                    begin
                        ram[i] <= 10'b0000000000;
                    end
            endcase
        end
        
        always @(posedge clk) begin
            if (write_en)
                ram[ram_address] <= write_data;
        end
        assign read_data = (mem_read) ? ram[ram_address] : 10'b0000000000;
endmodule
