`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/05/2020 08:26:56 AM
// Design Name: 
// Module Name: CPU_10bits_pipelined
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


module CPU_10bits_pipelined();
endmodule


/*
	This example shows part of a instruction pipeline. Only the first two stages are given: fetch and decode.
	For this system, there is only two instructions: addi and shift right. 
	9-bit instruction format: inst[8] is the opcode, inst[7] selects the register, and inst[6:0] is the immediate value.
*/

module pipeTB();
	reg clk, rst;

	pipeLine pipe0(clk, rst);

	always #5 clk = ~clk;

	initial begin
		clk = 0;
		rst = 1;

		#15 rst = 0;
	end

endmodule


module pipeLine(clk, rst);
	input clk, rst; 
	wire [9:0] decode_out, decode_out_reg;
	wire [9:0] inst, fetch_reg;
	wire [9:0] imm;
	wire ALU_OP, regSel;

	FetchStage fetch0(clk, rst, inst);
	// instead of sending the instruction directly to the next stage, we store it in a register first
	reg9Bit fetchReg(~clk, rst, 1'b1, inst, fetch_reg);  	// notice that the register stores data on the falling edge of the clock

	DecodeStage d0(fetch_reg, decode_out);
	// create a register the stores all data passed to the next stage
	reg11Bit decodeReg(~clk, rst, 1'b1, decode_out, decode_out_reg);   // notice that the register stores on the falling edge of the clock

	// decode_out contains 3 signals: ALU_OP, regSel, and the immidiate value/shift amount
	assign ALU_OP = decode_out_reg[10];
	assign regSel = decode_out_reg[9];
	assign imm = decode_out_reg[8:0];

	// the next stage goes here
endmodule


/* 	every clock cycle, this module fetches the next instruction 
   	from memory and increments the program counter. 
*/
module Fetch_Decode_Stage(input clk, input reset); // goes to the instruction memory
	
	reg [9:0] PC;
    wire [9:0] address;
    
	assign address = PC;

	// increment the program counter
	always @(posedge clk) begin
		if (reset) begin
			PC <= 10'b0000000000;
		end
		else begin
			PC <= PC + 1;
		end
	end
	
	wire [9:0] instruction;
	// instruction memory goes here 
	InstructionMemory IM(.clock(clk), .address(address), .instruction(instruction));
	/*initial begin 
		inst = 9'b000000000;
		#15
		inst = 9'b00000011;  	  // shift r0, 3

		#10 inst = 9'b100000001;  // add r0, 1
		#10 inst = 9'b111111111;  // add r1, -1
		#10 inst = 9'b010000101;  // shift r1, 5
		#10 inst = 9'b110001111;  // add r1, 15
	end */	
	
	wire [1:0] alu_op;
    wire alu_src;
    wire mem_write;
    wire mem_read;
    wire mem_to_reg;
    wire write_back;
    wire branch;
    wire reg_write;
    wire [1:0] ALU_Control;
    
	Control_Unit control(.opcode(instruction[9:7]), .alu_op(alu_op), .alu_src(alu_src), .mem_write(mem_write), .mem_read(mem_read),
                         .mem_to_reg(mem_to_reg), .write_back(write_back), .branch(branch), .reg_write(reg_write), .ALU_Control(ALU_Control));
    
    wire [9:0] C0;  
    assign C0 = 10'b0000000001;                   
    always @(branch) begin
           if (branch == 1 && instruction[9:7] == 3'b110 && C0 == 10'b0000000001) 
                PC = PC + {{3{instruction[6]}},instruction[6:0]}; 
           if (branch == 1 && instruction[9:7] == 3'b101 && C0 == 10'b0000000000)
                PC = PC + {{3{instruction[6]}},instruction[6:0]}; 
           if (instruction[9:7] == 3'b111)
                PC = 10'bxxxxxxxxxx;
    end
    
    wire [2:0] reg_dst;    
    wire [2:0] Reg1;
    wire [2:0] Reg2;
    wire [9:0] Reg_or_Num; 
    wire [9:0] write_data; 
    wire [9:0] read_data;
    wire [9:0] read_data_1;
    wire [9:0] read_data_2;  
    
    assign Reg1 = (instruction[9:7] == 3'b101 || instruction[9:7] == 3'b110) ? 3'bxxx: instruction[6:4]; 
    assign Reg2 = instruction[2:0];
                        
    RegFile reg_file(.clk(clk), .reset(reset), .write_en(reg_write), .reg_write_dest(reg_dst), .write_data(write_data),
                     .read_addr_1(Reg1), .read_data_1(read_data_1), .read_addr_2(Reg2), .read_data_2(read_data_2));

endmodule


// In this example, there are only 2 instructions: add register with immediate, and NOP
// There are only 2 registers: r0 and r1.
module Execute_Memory_Stage(input clk, input [9:0] instruction, output [9:0] ALU_output);
	
	wire read_data_1;
	wire read_data_2;
	wire alu_src;
	wire Reg_or_Num;
	wire [2:0] ALU_Control;
	wire write_data;
	wire mem_to_reg;
	wire mem_write;
	wire mem_read;
	wire read_data;
	wire C0;
	
	mux M0(.A1({{6{instruction[3]}},instruction[3:0]}), .A2(read_data_2), .Sel(alu_src), .Y(Reg_or_Num));
    
    ALU_module ALU(.ALU_Control(ALU_Control[0]), .A(read_data_1), .B(Reg_or_Num), .ALU_output(ALU_output)); 
    
    DataMemory DM(.clk(clk), .address(instruction[3:0]), .write_data(read_data_1), .write_en(mem_write), .mem_read(mem_read), .read_data(read_data));
    
    assign write_data = (mem_to_reg == 1)? read_data: ALU_output;  
    assign C0 = (instruction[9:7] == 3'b100)? write_data : C0;
endmodule

module reg12Bit(input clk, reset, en, input [11:0] data_in, output reg [11:0]data_out);

	always @(posedge clk) begin
		if (reset) 
			data_out <= 12'b000000000000;
		else if (en == 1'b1)
			data_out <= data_in;
	end

endmodule

module reg10Bit(input clk, reset, en, input [9:0] data_in,  output reg [9:0] data_out);

	always @(posedge clk) begin
		if (reset) 
			data_out <= 10'b0000000000;
		else if (en == 1'b1)
			data_out <= data_in;
	end

endmodule

module InstructionMemory(input clock, input [9:0] address, output reg [9:0] instruction);
    reg [9:0] memory[34:0];
    reg [1:0] program;
    integer i;
    integer mem0 = 0, mem8 = 8, distance = 29;
    
    initial begin
        program = 2'b10;
        case(program)
        2'b10:
            begin
            memory[0] <= 10'b0000100000;
            memory[1] <= 10'b0000110001;
            memory[2] <= 10'b0101010000;
            memory[3] <= 10'b0111000010;
            memory[4] <= 10'b0100111111;
            memory[5] <= 10'b1000110101;
            memory[6] <= 10'b1101111100;
            memory[7] <= 10'b0101001100;
            memory[8] <= 10'b0011000010;
            memory[9] <= 10'b111xxxxxxx;
            end
        2'b11:
            begin
            i = 4; 
            memory[0] <= 10'b0000110000;
            memory[1] <= 10'b1000110010;
            memory[2] <= 10'b1010011101; //beq
            memory[3] <= 10'b0010111000;
            
            while(i < 32)
                begin
                mem0 = mem0 + 1;
                mem8 = mem8 + 1;
                distance = distance - 4;
                memory[i] = {3'b000, 3'b011, mem0[3:0]};
                    i = i + 1;
                memory[i] = 10'b1000110000;
                    i = i + 1;
                memory[i] = {3'b101, distance[6:0]};
                    i = i + 1; 
                memory[i] = {3'b001, 3'b011, mem8[3:0]};
                    i = i + 1;
                end
            memory[32] <= 10'b0001001111; 
            memory[33] <= 10'b0001001111; 
            memory[34] <= 10'b111xxxxxxx;
            end
        endcase
    end
    always @(address)
    begin
        instruction <= memory[address];
    end
endmodule

module DataMemory(input clk, input [3:0] address, input [9:0] write_data, input write_en, input mem_read, 
                    output [9:0] read_data);
        integer i;
        reg [9:0] ram [15:0];
        wire [3:0] ram_address = address;
        reg [1:0] program;
        
        initial begin
            program = 2'b10;
            case(program)
            2'b10:
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
            2'b11:
                for(i = 0; i < 16; i=i+1)
                if (i < 8 || i == 15)
                    begin
                        ram[0] <= 10'b0000001010;
                        ram[1] <= 10'b0000001111;
                        ram[2] <= 10'b0000001100;
                        ram[3] <= 10'b0000001011;
                        ram[4] <= 10'b0000001101;
                        ram[5] <= 10'b0000001111;
                        ram[6] <= 10'b0000000111;
                        ram[7] <= 10'b0000000000;
                        ram[15] <= 10'b0000001010;
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

module Control_Unit(input[2:0] opcode,  output reg[1:0] alu_op, reg alu_src, mem_write, mem_read, mem_to_reg, write_back,
                    branch, reg_write, sign_or_zero, output reg[1:0] ALU_Control);
        
     always @(alu_op)    
     casex (alu_op)  
      2'b00: ALU_Control = 2'b00;  //adder ALU
      2'b01: ALU_Control = 2'b01;   // grt ALU
      2'b10: ALU_Control = 2'b10;  //send to data memory
      2'b11: ALU_Control = 2'b11; //halt
      default: ALU_Control = 2'b11;  
      endcase 
      
        always @(*)  
     begin  
          case(opcode)   
          3'b000: begin // load
                    alu_op = 2'b10; 
                    alu_src = 1'b1;
                    mem_write = 1'b0;
                    mem_read = 1'b1; 
                    mem_to_reg = 1'b1;
                    write_back = 1'b1;
                    branch = 1'b0;    
                    reg_write = 1'b1;  
                    sign_or_zero = 1'b1;  
                  end   
                    
          3'b001: begin // store
                    alu_op = 2'b10;
                    alu_src = 1'b1; 
                    mem_write = 1'b1;
                    mem_read = 1'b0; 
                    mem_to_reg = 1'b0;
                    write_back = 1'b0;
                    branch = 1'b0;  
                    reg_write = 1'b0;  
                    sign_or_zero = 1'b1;  
                  end   
          3'b010: begin // addi  
                    alu_op = 2'b00; 
                    alu_src = 1'b0;
                    mem_write = 1'b0;
                    mem_read = 1'b0;  
                    mem_to_reg = 1'b0;  
                    write_back = 1'b1; 
                    branch = 1'b0; 
                    reg_write = 1'b1;  
                    sign_or_zero = 1'b1;  
                  end 
                  
          3'b011: begin // add 
                    alu_op = 2'b00; 
                    alu_src = 1'b1;
                    mem_write = 1'b0;
                    mem_read = 1'b0;  
                    mem_to_reg = 1'b0;  
                    write_back = 1'b1;   
                    branch = 1'b0; 
                    reg_write = 1'b1;  
                    sign_or_zero = 1'b1;  
                  end 
                    
          3'b100: begin // grt
                    alu_op = 2'b01;
                    alu_src = 1'b1;
                    mem_write = 1'b0;
                    mem_read = 1'b0;  
                    mem_to_reg = 1'b0;  
                    write_back = 1'b1;
                    branch = 1'b0; 
                    reg_write = 1'b0;  
                    sign_or_zero = 1'b1;  
                  end 
                    
          3'b101: begin // beq  
                    alu_op = 2'bxx; 
                    alu_src = 1'bx;
                    mem_write = 1'b0;
                    mem_read = 1'b0;
                    mem_to_reg = 1'b0; 
                    write_back = 1'b0;
                    branch = 1'b1; 
                    reg_write = 1'b0;  
                    sign_or_zero = 1'bx;  
                  end 
          
          3'b110: begin // bne 
                    alu_op = 2'bxx; 
                    alu_src = 1'bx;
                    mem_write = 1'b0;
                    mem_read = 1'b0;
                    mem_to_reg = 1'b0; 
                    write_back = 1'b0;
                    branch = 1'b1; 
                    reg_write = 1'b0;  
                    sign_or_zero = 1'bx;  
                    end
                    
          3'b111: begin //  halt  
                    alu_op = 2'b11;
                    alu_src = 1'bx; 
                    mem_write = 1'bx;
                    mem_read = 1'bx;
                    mem_to_reg = 1'b1; 
                    write_back = 1'b1;
                    branch = 1'bx; 
                    reg_write = 1'b1;  
                    sign_or_zero = 1'bx;  
                    end  
                    
          default: begin  
                    alu_op = 2'bxx;
                    alu_src = 1'bx; 
                    mem_write = 1'bx;
                    mem_read = 1'bx;
                    mem_to_reg = 1'bx; 
                    write_back = 1'bx;
                    branch = 1'bx; 
                    reg_write = 1'bx;  
                    sign_or_zero = 1'bx;   
                    end  
          endcase  
          end 
endmodule

module RegFile(input clk, input reset, input write_en, input [2:0] reg_write_dest, input [9:0] write_data, 
                input [2:0] read_addr_1, output [9:0] read_data_1, input [2:0] read_addr_2, output [9:0] read_data_2  
 );  
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
