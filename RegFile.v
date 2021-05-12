`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/04/2020 06:53:49 PM
// Design Name: 
// Module Name: RegFile
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


module RegFile(input clk, input reset, input write_en, input [2:0] reg_write_dest, input [9:0] write_data, 
                input [2:0] read_addr_1, output [9:0] read_data_1, input [2:0] read_addr_2, output [9:0] read_data_2);
      reg [9:0]  reg_array [7:0];  
      wire [9:0] C0, T0, R0, R1, R2, R3, R5, R7; 
      
      always @ (posedge clk or posedge reset) begin  
           if(reset) begin  
                reg_array[0] <= 10'b0;  
                reg_array[1] <= 10'b0;  
                reg_array[2] <= 10'b0;  
                reg_array[3] <= 10'b0;  
                reg_array[4] <= 10'b0;  
                reg_array[5] <= 10'b0;  
                reg_array[6] <= 10'b0;  
                reg_array[7] <= 10'b0;       
           end  
           else begin  
                if(write_en) begin  
                     reg_array[reg_write_dest] <= write_data;  
                end  
           end  
      end  
      assign read_data_1 = ( read_addr_1 == 0)? 10'b0 : reg_array[read_addr_1];  
      assign read_data_2 = ( read_addr_2 == 0)? 10'b0 : reg_array[read_addr_2];  
      assign C0 = (read_addr_1 == 3'b000) ? reg_array[reg_write_dest] : C0; 
      assign T0 = (read_addr_1 == 3'b001) ? reg_array[reg_write_dest] : T0;
      assign R0 = (read_addr_1 == 3'b010) ? reg_array[reg_write_dest] : R0;
      assign R1 = (read_addr_1 == 3'b011) ? reg_array[reg_write_dest] : R1;
      assign R2 = (read_addr_1 == 3'b100) ? reg_array[reg_write_dest] : R2;
      assign R3 = (read_addr_1 == 3'b101) ? reg_array[reg_write_dest] : R3;
      assign R5 = (read_addr_1 == 3'b110) ? reg_array[reg_write_dest] : R5;
      assign R7 = (read_addr_1 == 3'b111) ? reg_array[reg_write_dest] : R7;
endmodule
