

module controller(input clk,rst,input [5:0]OPC,input [5:0]func,zero,output PCen,output reg IorD,memRead,memWrite,IRWrite,regWrite,ALUSrcA,output reg[1:0]ALUSrcB,output reg [2:0]ALUOperation,output reg[1:0]PCSrc,output reg[1:0]regDst,output reg[1:0]memToReg);
  reg [4:0]ps;
  reg [4:0]ns;
  reg [1:0]ALUOp;
  reg PCWrite;
  reg [1:0]PCWriteCond;
  
  always @(ps or OPC or func or zero) begin
    ns = 5'b00000;
    case(ps)
      5'b00000: ns = 5'b00001; // Instruction fetch
      5'b00001: begin // Instruction decode
        case (OPC)
          6'b000000: ns = (func == 6'b001000)?5'b10001:5'b00010; // RT (Including JR)
          6'b000010: ns = 5'b00100; //J
          6'b100011: ns = 5'b00101; //Lw
          6'b101011: ns = 5'b01000; //Sw
          6'b000100: ns = 5'b01010; //Beq
          6'b000101: ns = 5'b01011; //Bne
          6'b000001: ns = 5'b01100; //Jal
          6'b001000: ns = 5'b01101; //Addi
          6'b001100: ns = 5'b01111; //Andi
        endcase
      end
      5'b00010: ns = 5'b00011; // RT (without JR) 1
      5'b00011: ns = 5'b00000; // RT (without JR) 2
      5'b00100: ns = 5'b00000; // J
      5'b00101: ns = 5'b00110; // LW 1
      5'b00110: ns = 5'b00111; // LW 2
      5'b00111: ns = 5'b00000; // LW 3
      5'b01000: ns = 5'b01001; // SW 1
      5'b01001: ns = 5'b00000; // SW 2
      5'b01010: ns = 5'b00000; // Beq
      5'b01011: ns = 5'b00000; // Bne
      5'b01100: ns = 5'b00000; // Jal
      5'b01101: ns = 5'b01110; // Addi 1
      5'b01110: ns = 5'b00000; // Addi 2
      5'b01111: ns = 5'b10000; // Andi 1 
      5'b10000: ns = 5'b00000; // Andi 2
      5'b10001: ns = 5'b00000; // JR
    endcase
  end
  
  always @(ps or OPC or func or zero) begin
    {IorD,memRead,memWrite,IRWrite,regWrite,ALUSrcA,PCWrite} = 7'b0000000;
    ALUSrcB = 2'b00;
    regDst = 2'b00;
    PCSrc = 2'b00;
    ALUOp = 2'b00;
    memToReg = 2'b00;
    PCWriteCond = 2'b00;
    case(ps)
      5'b00000: begin // Instruction fetch
        {IorD,memRead,memWrite,IRWrite,regWrite,ALUSrcA,PCWrite} = 7'b0101001;
        ALUSrcB = 2'b01;
        PCSrc = 2'b00;
        ALUOp = 2'b00;
        PCWriteCond = 2'b00;
        end
      5'b00001: begin // Instruction decode
        ALUSrcA = 1'b0;
        ALUSrcB = 2'b11;
        ALUOp = 2'b00;
        end
      5'b00010: begin // RT (without JR) 1
        ALUSrcA = 1'b1;
        ALUSrcB = 2'b00;
        ALUOp = 2'b10;
        end
      5'b00011: begin // RT (without JR) 2
        regWrite = 1'b1;
        regDst = 2'b01;
        memToReg = 2'b00;
        end
      5'b00100: begin // J
        PCWrite = 1'b1;
        PCSrc = 2'b10;
        end
      5'b00101: begin // LW 1
        ALUSrcA = 1'b1;
        ALUSrcB = 2'b10;
        ALUOp = 2'b00;
        end
      5'b00110: begin // LW 2
        memRead = 1'b1;
        IorD = 1'b1;
        end
      5'b00111: begin // LW 3
        regWrite = 1'b1;
        regDst = 2'b00;
        memToReg = 2'b01;
        end
      5'b01000: begin // SW 1
        ALUSrcA = 1'b1;
        ALUSrcB = 2'b10;
        ALUOp = 2'b00;
        end
      5'b01001: begin // SW 2
        memWrite = 1'b1;
        IorD = 1'b1;
        end
      5'b01010: begin // Beq
        ALUSrcA = 1'b1;
        ALUSrcB = 2'b00;
        ALUOp = 2'b01;
        PCSrc = 2'b01;
        PCWriteCond = 2'b01;
        end
      5'b01011: begin // Bne
        ALUSrcA = 1'b1;
        ALUSrcB = 2'b00;
        ALUOp = 2'b01;
        PCSrc = 2'b01;
        PCWriteCond = 2'b10;
        end
      5'b01100: begin // Jal
        PCWrite = 1'b1;
        PCSrc = 2'b10;
        regWrite = 1'b1;
        regDst = 2'b10;
        memToReg = 2'b10;
        end
      5'b01101: begin // Addi 1
        ALUSrcA = 1'b1;
        ALUSrcB = 2'b10;
        ALUOp = 2'b00;
        end
      5'b01110: begin // Addi 2
        regWrite = 1'b1;
        regDst = 2'b00;
        memToReg = 2'b00;
        end
      5'b01111: begin // Andi 1
        ALUSrcA = 1'b1;
        ALUSrcB = 2'b10;
        ALUOp = 2'b11;
        end
      5'b10000: begin // Andi 2
        regWrite = 1'b1;
        regDst = 2'b00;
        memToReg = 2'b00;
        end
      5'b10001: begin // JR
        PCWrite = 1'b1;
        PCSrc = 2'b11;
        end
    endcase
  end
  
  always @(posedge clk,posedge rst) begin
    if (rst) ps <= 5'b00000;
    else ps <= ns;
  end
  
  assign PCen = PCWrite | ((PCWriteCond == 2'b01) & zero) | ((PCWriteCond == 2'b10) & ~zero);
  
  always @(ALUOp or func) begin
    case (ALUOp)
      2'b00: ALUOperation = 3'b010;  //OPC: Lw and Sw and Addi
                                  //ALU Operation: Add
      2'b01: ALUOperation = 3'b110;  //OPC: Beq
                                  //ALU Operation: Sub
      2'b10: case (func) //R-Type
            6'b100000: ALUOperation = 3'b010; //Add
            6'b100010: ALUOperation = 3'b110; //Sub
            6'b100100: ALUOperation = 3'b000; //And
            6'b100101: ALUOperation = 3'b001; //Or
            6'b101010: ALUOperation = 3'b111; //Slt
            default: ALUOperation = 3'b010;
          endcase
      2'b11: ALUOperation = 3'b000; //OPC: Andi
                                 //ALU Operation: And
      default: ALUOperation = 3'b010;
    endcase
  end
endmodule

