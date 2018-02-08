
module topPiplined;

reg clk;
wire [31:0] addr, inst, readData1, readData2, extendedInstruction , toALU,fetchDecodeOut, decodeExecuteOut_rd1, decodeExecuteOut_rd2, decodeExecuteOut_sign, executeMemory_aluResult, executeMemory_rd2, memoryWriteBack_data, memoryWriteBack_aluResult ;
wire [31:0] fetchDecodeAddrIN, fetchDecodeAddrOUT, DecodeExecuteAddrIN, DecodeExecuteAddrOUT;
wire decodeExecute_alusrc, executeMemory_alusrc, decodeExecute_aluOp, executeMemory_aluOp, decodeExecute_memWrite, executeMemory_memWrite, decodeExecute_memToReg, executeMemory_memToReg, memoryWriteBack_memToReg;
wire [4:0] decodeExecute_writeRegister, executeMemory_writeRegister, memoryWriteBack_writeRegister, multiplexer_writeRegister;

wire reg2Loc, ubranch, branch, memRead, memtoReg, memWrite, aluSrc, regWrite, zero;
wire [4:0] toRegister1,toRegister2, writeRegister;
wire [1:0] ALUOp;
wire [3:0] ALUControl;
wire [4:0] Rm, Rn, Rd;
wire [31:0] aluResult, readDataFromMemory, dataWriteBackToRegister;
wire iCacheMiss, cacheMiss;
wire [28:0] blockAddr;
wire [31:0] RegData0;
wire [31:0] RegData1;
wire [31:0] RegData2;
wire [31:0] RegData3;
wire [31:0] RegData4;
wire [31:0] RegData5;
wire [31:0] RegData6;
wire [31:0] RegData7;
wire [31:0] RegData8;
wire [31:0] RegData9;
wire [31:0] RegData10;
wire [31:0] RegData11;
wire [31:0] RegData12;
wire [31:0] RegData13;
wire [31:0] RegData14;
wire [31:0] RegData15;
wire [31:0] RegData16;
wire [31:0] RegData17;
wire [31:0] RegData18;
wire [31:0] RegData19;
wire [31:0] RegData20;
wire [31:0] RegData21;
wire [31:0] RegData22;
wire [31:0] RegData23;
wire [31:0] RegData24;
wire [31:0] RegData25;
wire [31:0] RegData26;
wire [31:0] RegData27;
wire [31:0] RegData28;
wire [31:0] RegData29;
wire [31:0] RegData30;
wire [31:0] RegData31;


initial begin
    clk = 1;
    
    repeat(10)begin
    #1 clk = 1; 
    #1 clk = 0; 
    
 
    $display("FetchDecodeOut: %h", fetchDecodeOut );
    $display("DecodeExecuteOut - ReadData1: %d, ReadData2: %d, SignExtend: %d", decodeExecuteOut_rd1, decodeExecuteOut_rd2,decodeExecuteOut_sign);
    $display("ExecuteMemoryOut - ALU Result: %d, ReadData2: %d",executeMemory_aluResult, executeMemory_rd2  );
    $display("MemoryWriteBackOut - Data Read: %h, ALU Result: %d",  memoryWriteBack_data, memoryWriteBack_aluResult);
    $display("WriteBackToRegisterOut - Data: %h, Register: R%d",dataWriteBackToRegister ,multiplexer_writeRegister );
    $display("\n-RegisterData -\nR00: %h\nR01: %h\nR02: %h\nR03: %h\nR04: %h\nR05: %h\nR06: %h\nR07: %h\nR08: %h\nR09: %h\nR10: %h\nR11: %h\nR12: %h\nR13: %h\nR14: %h\nR15: %h\nR16: %h\nR17: %h\nR18: %h\nR19: %h\nR20: %h\nR21: %h\nR22: %h\nR23: %h\nR24: %h\nR25: %h\nR25: %h\nR27: %h\nR28: %h\nR29: %h\nR30: %h\nR31: %h",
        RegData0,RegData1,RegData2,RegData3,RegData4,RegData5,RegData6,RegData7,RegData8,RegData9,RegData10,RegData11,RegData12,RegData13,RegData14,RegData15,RegData16,RegData17,RegData8,RegData19,RegData20,RegData21,RegData22,RegData23,RegData24,RegData25,RegData26,RegData27,RegData28,RegData29,RegData30,RegData31);
    $display("\n");
   
    end

end 



PC_PCArthmetic myPC(
                        .clk(clk),
                        .startingPC(32'h0000FFC),
                        .address(addr),
                        .signExtend(decodeExecuteOut_sign),
                        .unconditionalBranchFlag(ubranch),
                        .branchFlag(branch), 
                        .zeroFlag(zero)
 
 );
 
 instructionCache iCache(
                         .data(inst), 
                         .missFlag(iCacheMiss), 
                         .PC(addr), 
                         .inputData(0),
                         .writeData(0)
);


fetchDecodeReg FD(
                        .enable(clk),
                        .in(inst) ,
                        .out(fetchDecodeOut)
);
                       



controller myController
(
                        .Reg2Loc(reg2Loc),
                        .UncondBranch(ubranch),
                        .Branch(branch), 
                        .MemRead(memRead), 
                        .MemWrite(memWrite), 
                        .MemToReg(memtoReg), 
                        .ALUSrc(decodeExecute_alusrc),
                        .RegWrite(regWrite),
                        .ALUOp(decodeExecute_aluOp),
                        .Rm(Rm),
                        .Rn(Rn), 
                        .Rd(Rd), 
                        .WriteReg(decodeExecute_writeRegister),
                        .ALUControl(ALUControl),
                        .inst(fetchDecodeOut),
                        .signExtendOut(extendedInstruction)
);
    
    
    
    
      
 //mux to give the register the write register number     
 
//Rm is always inst[20:16];
//Rd is always inst[4:0];
//Rn is always inst[9:5];
 multiplexer10_5 ToRegisterMultiplexer(
                        .option1(Rm), 
                        .option2(Rd),
                        .out(toRegister2),
                        .select(reg2Loc)
);
 
 
 
registers myRegsiters(
                      .RegWrite(regWrite),
                      .ReadRegister1(Rn),
                      .ReadRegister2(toRegister2),
                      .WriteRegister(multiplexer_writeRegister),
                      .WriteData(dataWriteBackToRegister), // from multiplexer, 
                      .ReadData1(readData1),
                      .ReadData2(readData2),
                    .RegData0(RegData0),
                    .RegData1(RegData1),
                    .RegData2(RegData2),
                    .RegData3(RegData3),
                    .RegData4(RegData4),
                    .RegData5(RegData5),
                    .RegData6(RegData6),
                    .RegData7(RegData7),
                    .RegData8(RegData8),
                    .RegData9(RegData9),
                    .RegData10(RegData10),
                    .RegData11(RegData11),
                    .RegData12(RegData12),
                    .RegData13(RegData13),
                    .RegData14(RegData14),
                    .RegData15(RegData15),
                    .RegData16(RegData16),
                    .RegData17(RegData17),
                    .RegData18(RegData18),
                    .RegData19(RegData19),
                    .RegData20(RegData20),
                    .RegData21(RegData21),
                    .RegData22(RegData22),
                    .RegData23(RegData23),
                    .RegData24(RegData24),
                    .RegData25(RegData25),
                    .RegData26(RegData26),
                    .RegData27(RegData27),
                    .RegData28(RegData28),
                    .RegData29(RegData29),
                    .RegData30(RegData30),
                    .RegData31(RegData31)                 
                      
);
 
 DecodeExecuteReg DE(
                    .enable(clk),
                    .i1(readData1),
                    .i2(readData2),
                    .i3(extendedInstruction),
                    .i4(decodeExecute_alusrc),
                    .i5(decodeExecute_aluOp),
                    .i6(memWrite),
                    .i7(memtoReg),
                    .i8(decodeExecute_writeRegister),
                    .o1(decodeExecuteOut_rd1),
                    .o2(decodeExecuteOut_rd2),
                    .o3(decodeExecuteOut_sign),
                    .o4(executeMemory_alusrc),
                    .o5(executeMemory_aluOp),
                    .o6(decodeExecute_memWrite),
                    .o7(decodeExecute_memToReg),
                    .o8(executeMemory_writeRegister)

);
 
 
 //select either immediate or second register for the second ALU input      
multiplexer64_32 ToALUMultiplexer(
                     .option1(decodeExecuteOut_rd2),
                     .option2(decodeExecuteOut_sign),
                     .out(toALU),
                     .select(executeMemory_alusrc)
 );

ALU myALU(
                    .input1(decodeExecuteOut_rd1),
                    .input2(toALU),
                    .result(aluResult),
                    .zero(zero),
                    .ALUControl(ALUControl)
);



 ExecuteMemoryReg EM(
                     .enable(clk),
                     .ALUresultIN(aluResult),
                     .readdata2IN(decodeExecuteOut_rd2),
                     .ALUresultOUT(executeMemory_aluResult), 
                     .readdata2OUT(executeMemory_rd2),
                     .memWriteIN(decodeExecute_memWrite),
                     .memWriteOUT(executeMemory_memWrite),
                     .memToRegIN(decodeExecute_memToReg),
                     .memToRegOUT(execueMemory_memToReg),
                     .writeRegIN(executeMemory_writeRegister),
                     .writeRegOUT(memoryWriteBack_writeRegister)
 
  );


dataCache cache(        
                    .data(readDataFromMemory), 
                    .blockAddress(blockAddr),
					.missFlag(cacheMiss), 
					.addr(executeMemory_aluResult), 
					.inputData(executeMemory_rd2),
					.writeData(executeMemory_memWrite)
);


MemoryWriteBackReg MWB(
                    
                    .enable(clk),
                    .readDataIN(readDataFromMemory),
                    .readDataOUT(memoryWriteBack_data),
                    .aluResultIN(executeMemory_aluResult),
                    .aluResultOUT(memoryWriteBack_aluResult),
                    .destinationRegIN(),
                    .destinationRegOUT(),
                    .memToRegIN(execueMemory_memToReg),
                    .memToRegOUT(memoryWriteBack_memToReg),
                    .writeRegIN(memoryWriteBack_writeRegister),
                    .writeRegOUT(multiplexer_writeRegister)


);

multiplexer64_32 dataToRegMultiplexer(
                     .option1(memoryWriteBack_aluResult),
                     .option2(memoryWriteBack_data),
                     .out(dataWriteBackToRegister),
                     .select(memoryWriteBack_memToReg)
);

endmodule

//every clock cycle outputs memory address of instruction
module PC_PCArthmetic(clk,startingPC, address,signExtend, unconditionalBranchFlag, branchFlag, zeroFlag );

	input clk;
	input [31:0] signExtend; //sign extended insturction
	input unconditionalBranchFlag; //flag from controller
	input branchFlag; //flag from controller
	input zeroFlag; // flag from ALU
	output reg [31:0] address; //address of current instruction
	input [31:0] startingPC;
	reg [31:0] nextPC, pcPlus4;
	reg pcplus4, select, andResult, orResult;
	reg [31:0] branchPC;
	
	initial begin
	   select = 0;
	   nextPC = startingPC;
	   pcPlus4 = startingPC + 4;
	end
    always @(posedge clk)begin
        address = (select == 1) ?  branchPC : pcPlus4;
         pcPlus4 = address+4;
    end
  
    always@(signExtend)begin
      branchPC = address + signExtend;

    end
    
    always @(zeroFlag, branchFlag, unconditionalBranchFlag)begin
 
            andResult = zeroFlag & branchFlag;
            orResult = andResult | unconditionalBranchFlag;
            select = orResult;
      end 
    

endmodule

//outputs 32 bit instruction based at given memory address
module instructionCache(output reg [31:0]data, 
						output reg missFlag, 
						input [31:0]PC, 
						input [31:0]inputData,
						input writeData); //inputData and writeData would be used if we had an imperfect cache that would go to a lower level and get more data to load into this iCache.  Here we include but ignore it.

	reg [31:0]memoryAddress[0:15];
	reg [31:0]memoryData[0:15]; //could be expanded, but here we just use 16
	reg [6:0]i;

	always@(PC, inputData) begin : search

		for(i = 0; i < 16; i=i+1) begin //how many lines there are
			if(PC == memoryAddress[i]) begin
				data = memoryData[i];
				missFlag = 0;
				disable search;
			end //if PC==
		end //for

		//if you reach here, you didn't find it
		data = 0;
		missFlag = 1;
	end //always


	initial begin

		memoryAddress[0] = 32'h00001000; memoryData[0] = 32'h91002294; //ADDI R20, R20, #8
		memoryAddress[1] = 32'h00001004; memoryData[1] = 32'hB20000E3; //ORRI R3, R7, #0 
		memoryAddress[2] = 32'h00001008; memoryData[2] = 32'hD1000333; //SUBI R19, R25, #0
		memoryAddress[3] = 32'h0000100C; memoryData[3] = 32'h910006D6; //ADDI X22, X22, #1
		memoryAddress[4] = 32'h00001010; memoryData[4] = 32'hEA001028;//EORR R4, R1, R0
		memoryAddress[5] = 32'h00001014; memoryData[5] = 32'hF8400281; //LDUR R1 [R20, #0]
		//memoryAddress[5] = 32'h00001014; memoryData[5] = 32'hCB000165;//SUB R5, R11, R0
		//memoryAddress[6] = 32'h00001018; memoryData[6] = 32'hF85F4284; //LDUR R4, [R20 #-12]
		/*memoryAddress[6] = 32'h00001018; memoryData[6] = 32'h17FFFFFA; //B #3FFFFFA (-6)
		memoryAddress[7] = 32'h0000101C; memoryData[7] = 32'h17FFFFFA;
		memoryAddress[8] = 32'h00001020; memoryData[8] = 32'h17FFFFFA;
		memoryAddress[9] = 32'h00001024; memoryData[9] = 32'h17FFFFFA;
		memoryAddress[10] = 32'h00001028; memoryData[10] = 32'h17FFFFFA;
		memoryAddress[11] = 32'h0000102C; memoryData[11] = 32'h17FFFFFA;		
		memoryAddress[12] = 32'h00001030; memoryData[12] = 32'h17FFFFFA;
		memoryAddress[13] = 32'h00001034; memoryData[13] = 32'h17FFFFFA;
		memoryAddress[14] = 32'h00001038; memoryData[14] = 32'h17FFFFFA;
		memoryAddress[15] = 32'h0000103C; memoryData[15] = 32'h17FFFFFA;*/

	end

	
endmodule

module controller(
	output reg Reg2Loc, UncondBranch, Branch, MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, 
	output reg [1:0] ALUOp,
	output reg [4:0] Rm, Rn, Rd, WriteReg, 
	output  reg [3:0] ALUControl,
	output reg [31:0] signExtendOut,
	input [31:0] inst);

	reg [10:0] opcode;
	
	always@(inst)begin
	
		opcode = inst[31:21];
		
		//determine which type the instruction is
		
		//check R type
		//1 0 0 0 1 0 1 x x x x
		//_ _ _ _ _ _ _ _ _ _ _
		
		if((opcode[10] == 1) && (opcode[6] == 1) && (opcode[5] == 0) && (opcode[4] == 1))begin
		
			Rm = inst[20:16];
			Rd = inst[4:0];
			Rn = inst[9:5];
			Reg2Loc = 0;
			UncondBranch = 0;
			Branch = 0;
			MemRead = 0;
			MemWrite = 0;
			MemToReg = 0;
			ALUSrc = 0;
			RegWrite = 1;
			ALUOp = 2'b10;
			
			
			case(opcode[9:7]) 
			3'b000: begin
				if(opcode[3] == 1'b1) begin
					 //$display("%h : This  is an ADD.", inst); 
					 ALUControl = 4'b0010;
				end
				else begin 
					 //$display("This  is an AND.", inst) ; 
					 ALUControl = 4'b0110;
				end
			end
			3'b110:begin
				 //$display("%h :This  is an EORR.", inst);
				 ALUControl = 4'b1001;
			end
			3'b100: begin
				 //$display("%h :This  is an SUB.", inst);
				 ALUControl = 4'b1010;
			end
			3'b010: begin
				 //$display("%h :This  is an ORR.", inst);
				 ALUControl = 4'b0100;
			end
			
			endcase
		
		end
		
		//check I type
		//1 0 0 1 0 0 x x x x x
		//_ _ _ _ _ _ _ _ _ _ _
		else if ((opcode[10] == 1) && (opcode[7] == 1) && (opcode[6] == 0) && (opcode[5] == 0))begin
		
		
			Rm = 5'bx;
			Rd = inst[4:0];
			Rn = inst[9:5];
			Reg2Loc = 1'bx;
			UncondBranch = 0;
			Branch = 0;
			MemRead = 0;
			MemWrite = 0;
			MemToReg = 0;
			ALUSrc = 1;
			RegWrite = 1;
			ALUOp = 2'b10;
			signExtendOut = {{32{inst[21]}}, inst[21:10]};
		
		
			//determine which tpye of immediate 
			case(opcode[9:8])
				
				2'b00: begin
					if(opcode[4] == 1'b1) begin
						 //$display("%h :This  is an ANDI.", inst); 
						 ALUControl = 4'b0110;
					end
					else begin
						 //$display("%h :This  is an ADDI.", inst) ;
						ALUControl = 4'b0010; 
					end
					
				end
				2'b01: begin 
					 //$display("%h :This  is an ORRI.", inst) ;
					 ALUControl = 4'b0100;
				end
				2'b10: begin
					if(opcode[3] == 1'b1) begin
						 //$display("%h :This  is an EORI.", inst); 
						 ALUControl = 4'b1001;
					end
					else begin
						 //$display("%h :This  is an SUBI.", inst) ;
						 ALUControl = 4'b1010;
					end
						
				end 

						
			endcase
			
			
		end
		
		//Check LDUR/STUR
		// 1 1 1 1 1 x x x x x x
		// _ _ _ _ _ _ _ _ _ _ _
		else if(opcode[10:6] == 5'b11111)
		  
		begin
		
			Rm = 5'bx;
			Rd = inst[4:0];
			Rn = inst[9:5];
			UncondBranch = 0;
			Branch = 0;
			ALUSrc = 1;
			ALUOp = 2'b00;
			ALUControl = 4'b0010;
			
			signExtendOut = {{32{inst[19]}}, inst[19:12]};
		
		
			if(opcode[1] == 1) begin 
				 //$display("%h :This  is an LDUR.", inst) ;
				Reg2Loc = 1'bx;
				MemToReg = 1;
				MemRead = 1;
				MemWrite = 0;
				RegWrite = 1;
			end
			else begin
				 //$display("%h :This  is an STUR.", inst);
				Reg2Loc = 1;
				MemToReg = 1'bx;
				MemRead = 0;
				MemWrite = 1;
				RegWrite = 0;
			end
		end
		
		//check CBZ/CBNZ
		// 1 0 1 1 0 1 x x x x x
		// _ _ _ _ _ _ _ _ _ _ _
		
		else if(opcode[10:5] == 6'b101101)begin
		

			Rm = 5'bx;
			Rd = 5'bx;
			Rn = inst[4:0];
			Reg2Loc = 1'bx;
			UncondBranch = 0;
			Branch = 1;
			MemRead = 0;
			MemWrite = 0;
			MemToReg = 1'bx;
			ALUSrc = 1'bx;
			RegWrite = 0;
			ALUOp = 2'b01;
			ALUControl = 4'b0111;
			signExtendOut = {{32{inst[23]}}, inst[23:5]};
                    
		
		
		
			if(opcode[3] == 1) begin
				 //$display("%h :This  is an CBNZ.", inst); 
				 ALUControl = 4'b1111;
				 
			end
			else begin
				 //$display("%h :This  is an CBZ.", inst);
				 ALUControl = 4'b0111;
			end
		end
		
		//check MOVK
		// 1 1 1 1 0 x x x x x x
		// _ _ _ _ _ _ _ _ _ _ _
		
		else if(opcode[10:6] == 6'b11110)begin
			 //$display("%h :This  is an IM TYPE .", inst);
	
			Rm = 5'bx;
			
			//move to
			Rd = inst[4:0];
			
			//move from
			Rn = inst[9:5];
			
			Reg2Loc = 0;
			UncondBranch = 0;
			Branch = 0;
			MemRead = 0;
			MemWrite = 0;
			MemToReg = 0;
			ALUSrc = 0;
			RegWrite = 1;
			ALUOp = 2'b10;
			ALUControl =  4'b1101;
		end
		
		//check branches
		// x 0 0 1 0 1 x x x x x
		// _ _ _ _ _ _ _ _ _ _ _
		
		else if(opcode[9:5] == 5'b00101)begin
		
		
		
			Rm = 5'bx;
			Rd = 5'bx;
			Rn = 5'bx;
			Reg2Loc = 1'bx;
			UncondBranch = 1;
			Branch = 1;
			MemRead = 0;
			MemWrite = 0;
			MemToReg = 1'bx;
			ALUSrc = 1'bx;
			RegWrite = 1'bx;
			ALUControl = 4'bXXXX; // ALU is not used for straight branch instruction
		
            signExtendOut = {{32{inst[26]}}, inst[26:0]};
                    
	
		
			if(opcode[10] == 1) begin
				 //$display("%h :This  is an BL.", inst); 
			end
			else	begin
				 //$display("%h :This  is an B.", inst);
			end
		end
		
		else begin
		
			 //$display("Instruction: %h , opcode: %b ERROR.", inst,opcode);
		
		end
	   WriteReg = Rd;
	end
	
endmodule


//registers and sign extenstion 
module registers(
input RegWrite,
input [4:0] ReadRegister1,
input [4:0] ReadRegister2,
input [4:0] WriteRegister, 
input [31:0] WriteData,
output reg [31:0] ReadData1,
output reg [31:0] ReadData2,
output reg [31:0] RegData0,
output reg [31:0] RegData1,
output reg [31:0] RegData2,
output reg [31:0] RegData3,
output reg [31:0] RegData4,
output reg [31:0] RegData5,
output reg [31:0] RegData6,
output reg [31:0] RegData7,
output reg [31:0] RegData8,
output reg [31:0] RegData9,
output reg [31:0] RegData10,
output reg [31:0] RegData11,
output reg [31:0] RegData12,
output reg [31:0] RegData13,
output reg [31:0] RegData14,
output reg [31:0] RegData15,
output reg [31:0] RegData16,
output reg [31:0] RegData17,
output reg [31:0] RegData18,
output reg [31:0] RegData19,
output reg [31:0] RegData20,
output reg [31:0] RegData21,
output reg [31:0] RegData22,
output reg [31:0] RegData23,
output reg [31:0] RegData24,
output reg [31:0] RegData25,
output reg [31:0] RegData26,
output reg [31:0] RegData27,
output reg [31:0] RegData28,
output reg [31:0] RegData29,
output reg [31:0] RegData30,
output reg [31:0] RegData31

);


	//input RegWrite;
	//input [4:0] ReadRegister1, ReadRegister2,WriteRegister;
	//input [31:0] WriteData;
	//output reg [31:0] ReadData1,ReadData2;
	//output [31:0]RegData[0:31];
	
	reg [4:0]RegisterAddress[0:31];
    reg [31:0]RegisterData[0:31]; 
    reg [6:0]i;
    reg [31:0]RegData[0:31];
    
    
   
always@(*)begin
    
     //just so I could display the registers in my testbench
    RegData0 = RegisterData[0];
    RegData1 = RegisterData[1];
    RegData2 = RegisterData[2];
    RegData3 = RegisterData[3];
                               
    RegData4 = RegisterData[4];
    RegData5 = RegisterData[5];
    RegData6 = RegisterData[6];
    RegData7 = RegisterData[7];
    RegData8 = RegisterData[8];
    RegData9 = RegisterData[9];
    RegData10 = RegisterData[10];
    RegData11 = RegisterData[11];
    RegData12 = RegisterData[12];
    RegData13 = RegisterData[13];
    RegData14 = RegisterData[14];
    RegData15 = RegisterData[15];
    RegData16 = RegisterData[16];
    RegData17 = RegisterData[17];
    RegData18 = RegisterData[18];
    RegData19 = RegisterData[19];
    RegData20 = RegisterData[20];
    RegData21 = RegisterData[21];
    RegData22 = RegisterData[22];
    RegData23 = RegisterData[23];
    RegData24 = RegisterData[24];
    RegData25 = RegisterData[25];
    RegData26 = RegisterData[26];
    RegData27 = RegisterData[27];
    RegData28 = RegisterData[28];
    RegData29 = RegisterData[29];
    RegData30 = RegisterData[30];
    RegData31 = RegisterData[31];
    
end
    

   always@(ReadRegister1, ReadRegister2, WriteRegister) begin
        //for(i = 0; i < 32; i=i+1) begin //how many lines there are
               //if(ReadRegister1 == RegisterAddress[i]) begin
                   ReadData1 = RegisterData[ReadRegister1];
              // end
              // if(ReadRegister2 == RegisterAddress[i])begin
                    ReadData2 = RegisterData[ReadRegister2];
              // end
        //end
   end  
   
   always@(WriteData)begin
        if(RegWrite)begin
  
                 RegisterData[WriteRegister] = WriteData;
        end
   end  
 
   
    
    initial begin
        	RegisterAddress[0] = 5'd0; RegisterData[0] = 32'd0;
        	RegisterAddress[1] = 5'd1; RegisterData[1] = 32'd1;
        	RegisterAddress[2] = 5'd2; RegisterData[2] = 32'd2;
        	RegisterAddress[3] = 5'd3; RegisterData[3] = 32'd3;
        	RegisterAddress[4] = 5'd4; RegisterData[4] = 32'd4;                                             
        	RegisterAddress[5] = 5'd5; RegisterData[5] = 32'd5;
        	RegisterAddress[6] = 5'd6; RegisterData[6] = 32'd6;
        	RegisterAddress[7] = 5'd7; RegisterData[7] = 32'd7;
        	RegisterAddress[8] = 5'd8; RegisterData[8] = 32'd8;
        	RegisterAddress[9] = 5'd9; RegisterData[9] = 32'd9;
        	RegisterAddress[10] = 5'd10; RegisterData[10] = 32'd10;
        	RegisterAddress[11] = 5'd11; RegisterData[11] = 32'd11;
        	RegisterAddress[12] = 5'd12; RegisterData[12] = 32'd12;
        	RegisterAddress[13] = 5'd13; RegisterData[13] = 32'd13;
        	RegisterAddress[14] = 5'd14; RegisterData[14] = 32'd14;
        	RegisterAddress[15] = 5'd15; RegisterData[15] = 32'd15;
        	RegisterAddress[16] = 5'd16; RegisterData[16] = 32'd16;
        	RegisterAddress[17] = 5'd17; RegisterData[17] = 32'd17;
        	RegisterAddress[18] = 5'd18; RegisterData[18] = 32'd18;
        	RegisterAddress[19] = 5'd19; RegisterData[19] = 32'd19;
        	RegisterAddress[20] = 5'd20; RegisterData[20] = 32'd20;
        	RegisterAddress[21] = 5'd21; RegisterData[21] = 32'd21;
        	RegisterAddress[22] = 5'd22; RegisterData[22] = 32'd22;
        	RegisterAddress[23] = 5'd23; RegisterData[23] = 32'd23;
        	RegisterAddress[24] = 5'd24; RegisterData[24] = 32'd24;
        	RegisterAddress[25] = 5'd25; RegisterData[25] = 32'd25;
        	RegisterAddress[26] = 5'd26; RegisterData[26] = 32'd26;
        	RegisterAddress[27] = 5'd27; RegisterData[27] = 32'd27;
        	RegisterAddress[28] = 5'd28; RegisterData[28] = 32'd28;
        	RegisterAddress[29] = 5'd29; RegisterData[29] = 32'd29;
        	RegisterAddress[30] = 5'd30; RegisterData[30] = 32'd30;
        	RegisterAddress[31] = 5'd31; RegisterData[31] = 32'd0;
        	

    end
	
	

endmodule

module ALU(input1,input2, result, zero,ALUControl);

	input[31:0] input1, input2;
	input[3:0] ALUControl; 
	output reg [31:0] result;
	output reg zero;
	
	always@(input1,input2)begin
        
            case(ALUControl)
                
                //CBZ 
                4'b0111: begin
                    if(input1 == 0)begin
                        result = 32'bx; 
                        zero = 1'b1;
                    end
                    else begin
                        result = 32'bx; 
                        zero = 1'b0;
                    end
                end
                 
                //ADD, LDUR, STUR
                4'b0010: begin
                
                    result = input1 + input2;
                    zero = (result == 0) ? 1'b1 : 1'b0;
                   
                end
                
                //SUB
                4'b1010: begin
                    result = input1 - input2;
                    zero = (result == 0) ? 1'b1 : 1'b0;
                end
                
                //AND
                4'b0110: begin
                    result = input1 & input2;
                    zero = (result == 0) ? 1'b1 : 1'b0;
                end
                
                //ORR
                4'b0100: begin
                
                    result = input1 | input2;
                   zero = (result == 0) ? 1'b1 : 1'b0;
                end
                
                //EOR 
                4'b1001: begin 
                    result = input1 ^ input2;
                    zero = (result == 0) ? 1'b1 : 1'b0;
                end
                
                //NOR
                4'b0101: begin 
                
                    result = ~(input1 | input2);
                    zero = (result == 0) ? 1'b1 : 1'b0;
            
                end
                
                //NAND 
                4'b1100: begin 
                    
                    result = ~(input1 & input2);
                    zero = (result == 0) ? 1'b1 : 1'b0;
                end 
                
                //MOV
                4'b1101: begin
                
                    result = input1; //only one input to ALU, dont care about input2, pass along input 1 to data memory
                    zero = (result == 0) ? 1'b1 : 1'b0;
                    
                end
            endcase
            
        end

endmodule

module dataCache(output reg [31:0]data, 
						output reg missFlag, 
						output [28:0] blockAddress,
						input [31:0]addr, 
						input [31:0]inputData,
						input writeData); //inputData and writeData would be used if we had an imperfect cache that would go to a lower level and get more data to load into this iCache.  Here we include but ignore it.

	reg [28:0]set0Address[0:7];
	reg [31:0]set0Data[0:7]; //could be expanded, but here we just use 16
	reg [28:0]set1Address[0:7];
	reg [31:0]set1Data[0:7]; //could be expanded, but here we just use 16
	
	reg [6:0]i;
	 assign blockAddress = addr[31:3];
	wire setID = blockAddress % 2;

	always@(blockAddress, inputData, setID) begin : search
		if(setID) begin //set 1
				for(i = 0; i < 8; i=i+1) begin //how many lines there are
					if(blockAddress == set1Address[i]) begin
						if(!writeData) begin
							data = set1Data[i];
						end //if
						else begin
							set1Data[i] = inputData;
							data = set1Data[i];
						end //else
					
						missFlag = 0;
						disable search;
					end //if PC==
					else begin
						missFlag = 1;
						data = 32'bx;
						//don't bother with updates/writes for now
					end
				end //for
		end //if

		else begin
				for(i = 0; i < 8; i=i+1) begin //how many lines there are
					if(blockAddress == set0Address[i]) begin
						if(!writeData) begin
							data = set0Data[i];
						end //if
						else begin
							set0Data[i] = inputData;
							data = set0Data[i];
						end //else
					
						missFlag = 0;
						disable search;
					end //if PC==
					else begin
						missFlag = 1;
						data = 32'bx;
						//don't bother with updates/writes for now
					end //else
				end //for
			end //else
	end //always


	initial begin
		set0Address[0] = 29'h00000002; set0Data[0] = 32'h01010101;
		set0Address[1] = 29'h00000004; set0Data[1] = 32'h02020202;
		set0Address[2] = 29'h00000006; set0Data[2] = 32'h03030303;
		set0Address[3] = 29'h00000008; set0Data[3] = 32'h04040404;
		set0Address[4] = 29'h0000000A; set0Data[4] = 32'h05050505;
		set0Address[5] = 29'h0000000C; set0Data[5] = 32'h06060606;
		set0Address[6] = 29'h0000000E; set0Data[6] = 32'h07070707;
		set0Address[7] = 29'h00000010; set0Data[7] = 32'h08080808;

		set1Address[0] = 29'h00000001; set1Data[0] = 32'h99999999;
		set1Address[1] = 29'h00000003; set1Data[1] = 32'hAAAAAAAA;
		set1Address[2] = 29'h00000005; set1Data[2] = 32'hBBBBBBBB;
		set1Address[3] = 29'h00000007; set1Data[3] = 32'hCCCCCCCC;
		set1Address[4] = 29'h00000009; set1Data[4] = 32'hDDDDDDDD;
		set1Address[5] = 29'h0000000B; set1Data[5] = 32'hEEEEEEEE;
		set1Address[6] = 29'h0000000D; set1Data[6] = 32'hFFFFFFFF;
		set1Address[7] = 29'h0000000F; set1Data[7] = 32'h01234567;
	end


endmodule


module multiplexer64_32(option1, option2, out, select);

input [31:0] option1, option2;
input select;
output reg [31:0] out;

	always@(select, option1, option2)begin
		if(select)begin
			 out = option2;
		end
		else begin
			 out = option1;
		end
	end

endmodule

module multiplexer10_5(option1, option2, out, select);

input [4:0] option1, option2;
input select;
output reg [4:0] out;

	always@(select, option1, option2)begin
		if(select)begin
			 out = option2;
		end
		else begin
			 out = option1;
		end
	end

endmodule

module fetchDecodeReg(input enable,input [31:0] thisPCAddressIN, input [31:0] in ,output reg [31:0] out , output reg [31:0] thisPCAddressOUT);
   always@(enable)begin
        if(enable)begin
            out = in;
        end 
    end
endmodule
module DecodeExecuteReg(input enable,input [31:0] i1, input [31:0] i2, input[31:0]  i3, input i4 , input i5, input i6, input i7, input [4:0] i8,output reg [31:0] o1, output reg [31:0] o2, output reg[31:0] o3, output reg o4, output reg o5, output reg o6, output reg o7, output reg [4:0] o8);
   always@(enable)begin
        if(enable)begin
            o1 = i1;
            o2 = i2;
            o3 = i3;
            o4 = i4;
            o5 = i5;
            o6 = i6;
            o7 = i7;
            o8 = i8;
        end 
    end
endmodule

module ExecuteMemoryReg(input enable, input [31:0] ALUresultIN,input [31:0] readdata2IN,  output reg [31:0] ALUresultOUT, output reg [31:0] readdata2OUT, input memWriteIN, output reg memWriteOUT , input memToRegIN, output reg memToRegOUT, input [4:0] writeRegIN, output reg [4:0] writeRegOUT);
    always@(enable)begin
        if(enable)begin
            ALUresultOUT = ALUresultIN;
            readdata2OUT = readdata2IN;
            memWriteOUT = memWriteIN;
            memToRegOUT = memToRegIN;
            writeRegOUT = writeRegIN;
        end 
        
    end
endmodule

module MemoryWriteBackReg(input enable, input [31:0] readDataIN, output reg [31:0] readDataOUT, input [31:0] aluResultIN, output reg [31:0] aluResultOUT, input [4:0] destinationRegIN, output reg [4:0] destinationRegOUT,  input memToRegIN, output reg memToRegOUT, input [4:0] writeRegIN, output reg [4:0] writeRegOUT);
    always@(enable)begin
        if(enable)begin
            readDataOUT = readDataIN;
            aluResultOUT = aluResultIN;
            destinationRegOUT = destinationRegIN;
            memToRegOUT = memToRegIN;
            writeRegOUT = writeRegIN;
        end
      end
endmodule