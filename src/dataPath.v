module MIPS(clk,rst);
  input clk,rst;
  
  wire IorD,IRWrite,PCen,ALUSrcA,memRead,memWrite,zero,regWrite;
  wire [1:0]regDst;
  wire [1:0]memToReg;
  wire [1:0]ALUSrcB;
  wire [1:0]PCSrc;
  wire [2:0]ALUOperation;
  wire [5:0]OPC;
  wire [5:0]func;
  
  DataPath dp(clk,rst,IorD,IRWrite,PCen,regDst,ALUSrcA,ALUSrcB,regWrite,ALUOperation,memRead,memWrite,memToReg,PCSrc,OPC,func,zero);
  controller CU(clk,rst,OPC,func,zero,PCen,IorD,memRead,memWrite,IRWrite,regWrite,ALUSrcA,ALUSrcB,ALUOperation,PCSrc,regDst,memToReg);
endmodule

module DataPath(clk,rst,IorD,IRWrite,PCen,regDst,ALUSrcA,ALUSrcB,RegWrite,ALUOperation,memRead,memWrite,memToReg,PCSrc,OPC,func,zero);
  input clk;
  input rst;
  input PCen;
  input [1:0]regDst;
  input ALUSrcA;
  input [1:0]ALUSrcB;
  input IorD;
  input [2:0]ALUOperation;
  input memRead;
  input memWrite;
  input [1:0]memToReg;
  input IRWrite;
  input [1:0]PCSrc;
  input RegWrite;
  output [5:0]OPC;
  output [5:0]func;
  output zero;
  
  
  wire [31:0]newPC;
  wire [31:0]PC;
  wire [31:0]address;
  wire [4:0]writeReg;
  wire [31:0]writeData;
  wire [31:0]readData1;
  wire [31:0]readData2;
  wire [31:0]memDataRegOut;
  wire [31:0]extended;
  wire [31:0]ALUResult;
  wire [31:0]ALUOut;
  wire [31:0]readData;
  wire [27:0]shifted28;
  wire [31:0]shifted32;
  wire [31:26]IROut0;
  wire [25:21]IROut1;
  wire [20:16]IROut2;
  wire [15:0]IROut3;
  wire [31:0]ALUIn1;
  wire [31:0]ALUIn2;
  wire [31:0]OutA;
  wire [31:0]OutB;
  
  assign OPC = IROut0;
  assign func = IROut3[5:0];
  assign four = {29'b0, 3'b100};
  //
  PCReg PC1 (newPC, clk, rst, PCen, PC);
  mux2 MX2_1(PC, ALUOut, IorD, address);
  Memory M (address, OutB, memRead, memWrite, clk, rst, readData);
  IR IR (readData, IRWrite, IROut0, IROut1, IROut2, IROut3, clk, rst);
  register_32_bit MemoryDataRegister (readData, memDataRegOut, clk, rst);
  //
  mux3 MX3_1 (IROut2, IROut3[15:11], 5'b11111, regDst, writeReg);
  mux4 MX3_2 (ALUOut, memDataRegOut, PC,, memToReg, writeData);
  registerFile RF(IROut1, IROut2, writeReg, writeData, RegWrite, clk, rst, readData1, readData2);
  register_32_bit A (readData1, OutA, clk, rst);
  register_32_bit B (readData2, OutB, clk, rst);
  //
  signExtend SE(IROut3, extended);
  shL2_32 SHL32(extended, shifted32);
  mux2 MX2_2(PC, OutA, ALUSrcA, ALUIn1);
  mux4 MX4_1(OutB, 32'b00000000000000000000000000000100, extended, shifted32, ALUSrcB, ALUIn2);
  ALU alu(ALUIn1, ALUIn2, ALUOperation, zero, ALUResult);
  register_32_bit ALUOut1 (ALUResult, ALUOut, clk, rst);
  //
  shL2_26 SHL26({IROut1, IROut2, IROut3}, shifted28);
  mux4 MX4_2 (ALUResult, ALUOut, {PC[31:28],shifted28}, OutA, PCSrc, newPC);
endmodule

module signExtend (Input, Output);
  input [15:0]Input;
  output [31:0]Output;
  
  assign Output = {{16{Input[15]}}, Input[15:0]};
endmodule

module mux4 (A, B, C, D, S, out);
  input [31:0]A;
  input [31:0]B;
  input [31:0]C;
  input [31:0]D;
  input [1:0]S;
  output reg [31:0]out;
  
  always @ (A, B, C, S) begin
    out = 32'b0;
    case(S)
      2'b00: out = A;
      2'b01: out = B;
      2'b10: out = C;
      2'b11: out = D;
    endcase
  end
endmodule

module mux3 (A, B, C, S, out);
  input [4:0]A;
  input [4:0]B;
  input [4:0]C;
  input [1:0]S;
  output reg [4:0]out;
  
  always @ (A, B, C, S) begin
    out = 5'b0;
    if (S == 2'b00)
      out = A;
    else if (S == 2'b01)
      out = B;
    else if (S == 2'b10)
      out = C;
    else
      out = 5'b0;
  end
endmodule

module mux2 (A, B, S, out);
  input [31:0]A;
  input [31:0]B;
  input S;
  output reg [31:0]out;
  
  always @ (A, B, S) begin
    out = 32'b0;
    if (S == 1'b0) out = A;
    else out = B;
  end
endmodule

module ALU (A, B, ALUOperation, zero, ALUResult);
  input [31:0]A;
  input [31:0]B;
  input [2:0]ALUOperation;
  output zero;
  output reg [31:0]ALUResult;
  assign zero = (ALUResult == 32'b0) ? 1'b1 : 1'b0;
  always @ (A or B or ALUOperation) begin
    case (ALUOperation)
      3'b000:  //AND
        ALUResult = A & B;
      3'b001: //OR
        ALUResult = A | B;
      3'b010:  //ADD
        ALUResult = A + B;
      3'b110:  //SUB
        ALUResult = A - B;
      3'b111: //SLT
        ALUResult = (A < B);
      default: ALUResult = A + B;
    endcase
  end
endmodule

module PCReg (newPC, clk, rst, PCen, PC);
  input [31:0]newPC;
  input clk;
  input rst;
  input PCen;
  output reg [31:0]PC;
  always @ (posedge clk, posedge rst) begin
    if (rst == 1'b1)
      PC <= 32'b0;
    else if (PCen == 1'b1)
      PC <= newPC;
  end
endmodule

module IR (IRIn, IRWrite, IROut0, IROut1, IROut2, IROut3, clk, rst);
  input [31:0]IRIn;
  input IRWrite, clk, rst;
  output reg [5:0]IROut0; 
  output reg [4:0]IROut1;
  output reg [4:0]IROut2;
  output reg [15:0]IROut3;
  always @ (posedge clk, posedge rst) begin
    if (rst == 1'b1)
      {IROut0, IROut1, IROut2, IROut3} <= 32'b0;
    else if (IRWrite == 1'b1) begin
      IROut0 <= IRIn[31:26];
      IROut1 <= IRIn[25:21];
      IROut2 <= IRIn[20:16];
      IROut3 <= IRIn[15:0];
    end
  end
endmodule

module register_32_bit (In, Out, clk, rst);
  input [31:0]In;
  input clk, rst;
  output reg [31:0]Out;
  always @ (posedge clk, posedge rst) begin
    if (rst == 1'b1)
      Out <= 32'b0;
    else
      Out <= In;
  end
endmodule

module Memory (address, writeData, memRead, memWrite, clk, rst, readData);
  input [31:0]address;
  input [31:0]writeData;
  input memRead;
  input memWrite;
  input clk;
  input rst;
  output [31:0]readData;
  reg [7:0] memory[0:2047];
  
  always @(posedge clk,posedge rst) begin
    if (rst == 1'b1) $readmemb("memory.txt",memory);
    else begin
      if (memWrite) begin
        memory[address] <= writeData[31:24];
        memory[address+1] <= writeData[23:16];
        memory[address+2] <= writeData[15:8];
        memory[address+3] <= writeData[7:0];
      end
    end
  end
  assign readData = {memory[address],memory[address+1],memory[address+2],memory[address+3]};
endmodule

module registerFile (input [4:0]readReg1,input [4:0]readReg2,input [4:0]writeReg,input [31:0]writeData,input regWrite,input clk,input rst,output [31:0]data1,output [31:0]data2);
  reg [31:0] R[0:31];
  assign data1 = R[readReg1];
  assign data2 = R[readReg2];
  always @(posedge clk, posedge rst)begin
    if (rst == 1'b1) R[0] <= 32'b0;
    else begin
      if (regWrite == 1'b1) R[writeReg] <= writeData;
    end
  end
endmodule

module shL2_26 (input [25:0]A, output [27:0]S);
  assign S = {A, 2'b00};
endmodule

module shL2_32 (input [31:0]A,output [31:0]S);
  assign S = {A[29:0], 2'b00};
endmodule


module test();
  reg clk,rst;
  MIPS mips(clk,rst);
  initial begin
    #20 rst = 1'b0;
    #20 rst = 1'b1;
    #20 rst = 1'b0;
    #20;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
        #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
    #100 clk = 1'b1;
    #100 clk = 1'b0;
  end
endmodule
