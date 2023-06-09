 module signextmux(
	 input [23:0] in,
	 input [1:0] select,
	 output reg[31:0] extVal);
 
	integer i;
	always @ (*) begin
	case(select)
	0 : begin
		extVal[7:0]=in[7:0];
		for(i=8;i<32;i=i+1) extVal[i]=in[7];
		end
	1 : begin
		extVal[11:0]=in[11:0];
		for(i=12;i<32;i=i+1) extVal[i]='b0;
		end
	2 : begin
		extVal[1:0]=2'b00;
		extVal[25:2]=in[23:0];
		for(i=26;i<32;i=i+1) extVal[i]=in[23];
		end
	default : extVal='h00000000;
	endcase
	end
endmodule
module register_1bit(
	input regin,
	input clk,
	input write,
	input reset,
	output reg regout);
	
	always @ (posedge clk,posedge reset)
	if(reset) begin
		regout<=0;
	end
	else if(write) begin
		regout<=regin;
	end
	
endmodule
module register(
	input[31:0] regin,
	input clk,
	input write,
	input reset,
	output reg[31:0] regout);
	
	always @ (posedge clk)
	if(reset)
		regout<='h0000;
	else if(write)
		regout<=regin;
endmodule
module decoder_4to16(
	input[3:0] in,
	output reg[15:0] out);
	
	always @ (*)
	case(in)
	0 : out='h0001;
	1 : out='h0002;
	2 : out='h0004;
	3 : out='h0008;
	4 : out='h0010;
	5 : out='h0020;
	6 : out='h0040;
	7 : out='h0080;
	8 : out='h0100;
	9 : out='h0200;
   10 : out='h0400;
   11 : out='h0800;
   12 : out='h1000;
   13 : out='h2000;
   14 : out='h4000;
   15 : out='h8000;
    endcase
endmodule

module registerfile(
	input[3:0] reg1,
	input[3:0] reg2,
	input[3:0] regdst,
	input[31:0] regsrc,
	input[31:0] r15,
	input clk,
	input reset,
	input we,
	input PCWrite,
	output[31:0] out1,
	output[31:0] out2,
	output[31:0] pc);
	
	wire[15:0] write;
	wire[31:0] registers [15:0];
	
	decoder_4to16 selecter(.in (regdst), .out (write));
	
	register register0 (.regin (regsrc), .write (write[0]&we), .clk (clk), .reset (reset), .regout (registers[0]));
	register register1 (.regin (regsrc), .write (write[1]&we), .clk (clk), .reset (reset), .regout (registers[1]));
	register register2 (.regin (regsrc), .write (write[2]&we), .clk (clk), .reset (reset), .regout (registers[2]));
	register register3 (.regin (regsrc), .write (write[3]&we), .clk (clk), .reset (reset), .regout (registers[3]));
	register register4 (.regin (regsrc), .write (write[4]&we), .clk (clk), .reset (reset), .regout (registers[4]));
	register register5 (.regin (regsrc), .write (write[5]&we), .clk (clk), .reset (reset), .regout (registers[5]));
	register register6 (.regin (regsrc), .write (write[6]&we), .clk (clk), .reset (reset), .regout (registers[6]));
	register register7 (.regin (regsrc), .write (write[7]&we), .clk (clk), .reset (reset), .regout (registers[7]));
	register register8 (.regin (regsrc), .write (write[8]&we), .clk (clk), .reset (reset), .regout (registers[8]));
	register register9 (.regin (regsrc), .write (write[9]&we), .clk (clk), .reset (reset), .regout (registers[9]));
	register register10 (.regin (regsrc), .write (write[10]&we), .clk (clk), .reset (reset), .regout (registers[10]));
	register register11 (.regin (regsrc), .write (write[11]&we), .clk (clk), .reset (reset), .regout (registers[11]));
	register register12 (.regin (regsrc), .write (write[12]&we), .clk (clk), .reset (reset), .regout (registers[12]));
	register register13 (.regin (regsrc), .write (write[13]&we), .clk (clk), .reset (reset), .regout (registers[13]));
	register register14 (.regin (regsrc), .write (write[14]&we), .clk (clk), .reset (reset), .regout (registers[14]));
	// r15는 regsrc로 값이 저장될 일이 없다고 가정.
	register register15 (.regin (r15), .write (PCWrite), .clk (clk), .reset (reset), .regout (registers[15]));

	assign out1=registers[reg1];
	assign out2=registers[reg2];
	assign pc=registers[15];

endmodule

module SignalBEQ(
	input[31:0] inst,
	input zero,
	output PCWrite
)
	always @(*) begin
		if (inst[31:24] == 8'b00001010 && zero) PCWrite = 1;
		else if (inst[31:24] == 8'b11101010) PCWrite = 1;
		else PCWrite = 0;
	end
	
endmodule

module Adder4(
	input[31:0] in,
	output[31:0] result
)
	assign result = in + 'b100;
endmodule


module IFIDRegister(
	input[31:0] pcIn,
	input[31:0] instIn,
	input clk,
	input write,
	input reset,
	output reg[31:0] pcOut
	output reg[31:0] instOut);
	
	always @ (posedge clk)
	if(reset)
		pcOut<='h0000;
		instOut<='h0000;
	else if(write)
		pcOut<=pcIn;
		instOut<=instIn;
endmodule

module IDEXRegister(
	input[31:0] pcIn,
	input[31:0] instIn,
	input[31:0] reg1In,
	input[31:0] reg2In,
	input[31:0] immIn,
	input clk,
	input write,
	input reset,
	output reg[31:0] pcOut
	output reg[31:0] instOut
	output reg[31:0] reg1Out
	output reg[31:0] reg2Out
	output reg[31:0] immOut);
	
	always @ (posedge clk)
	if(reset)
		pcOUt <= 'h0000;
		instOut <= 'h0000;
	else if(write)
		pcOut <= pcIn;
		instOut <= instIn;
		reg1Out <= reg1In;
		reg2Out <= reg2In;
		immOut <= immIn;
endmodule

module EXMEMRegister(
	input[31:0] pcIn,
	input[31:0] instIn,
	input[31:0] ALUResultIn,
	input[31:0] reg2In,
	input clk,
	input write,
	input reset,
	output reg[31:0] pcOut,
	output reg[31:0] instOut,
	output reg[31:0] ALUResultOut,
	output reg[31:0] reg2Out);
	
	always @ (posedge clk)
	if(reset)
		pcOUt <= 'h0000;
		instOut <= 'h0000;
		ALUResultOut <= 'h0000;
		reg2Out <= 'h0000;
	else if(write)
		pcOut <= pcIn;
		instOut <= instIn;
		ALUResultOut <= ALUResultIn;
		reg2Out <= reg2In;
endmodule

module MEMWBRegister(
	input[31:0] instIn,
	input[31:0] mdrIn,
	input[31:0] ALUResultIn,
	input[31:0] reg2In,
	input clk,
	input write,
	input reset,
	output reg[31:0] instOut,
	output reg[31:0] mdrOut,
	output reg[31:0] ALUResultOut
	output reg[31:0] reg2Out);

	always @ (posedge clk)
	if(reset)
		pcOUt <= 'h0000;
		instOut <= 'h0000;
		ALUResultOut <= 'h0000;
		reg2Out <= 'h0000;
	else if(write)
		pcOut <= pcIn;
		instOut <= instIn;
		ALUResultOut <= ALUResultIn;
		reg2Out <= reg2In;
endmodule

module armreduced(
	input clk,
	input reset,
	output[31:0] pc,
	input[31:0] inst,
	input nIRQ,
	output [3:0] be,
	output[31:0] memaddr,
	output memwrite,
	output memread,
	output[31:0] writedata,
	input[31:0] readdata);
	
	//signals
	wire IRwrite,regwrite,NZCVwrite;
	wire [1:0] regdst,regsrc,ALUsrcA,immsrc,ALUsrcB;
	wire [2:0] ALUop;
	wire [3:0] instop;
	wire regBdst;
	wire IFIDWrite, IDEXWrite, EXMEMWrite, MEMWBWrite;
	wire IFIDReset, IDEXReset, EXMEMReset, MEMWBReset;

	
	//wires_out
	wire[31:0] imm;
	
	//wires_in
	wire [3:0] ALUflags;
	wire [31:0] ALUresult,readA,readB;
	
	wire [3:0] RFdst [2:0];
	wire [31:0] RFsrc [3:0];
	wire [31:0] ALUnum1 [2:0];
	wire [31:0] ALUnum2 [3:0];
	
	wire [3:0] regBread [1:0];
	
	//registers
	assign be = 4'b1111;
	wire[31:0] A,B,instructions,mdr,ALUout, ALUoutWB, BWB;
	reg n,v;
	wire z,c;
	wire[31:0] pcIFIDtoIDEX, pcIDEXtoEXMEM, pcEXMEMtoMEMWB;
	wire[31:0] instIFIDtoIDEX, instIDEXtoEXMEM, instEXMEMtoMEMWB;
	wire[31:0] immIDEXtoEXMEM, BEXMEMtoMEMWB;
  
  //register MDR(.regin (readdata), .write ('b1), .clk (clk), .reset (reset), .regout (mdr));
  //register ALUoutRegister(.regin (ALUresult), .write ('b1), .clk (clk), .reset (reset), .regout (ALUout));
  //register A_Register(.regin (readA), .write ('b1), .clk (clk), .reset (reset), .regout (A));
  //register B_Register(.regin (readB), .write ('b1), .clk (clk), .reset (reset), .regout (B));
  //register InstructionRegister(.regin (inst), .write (IRwrite), .clk (clk), .reset (reset), .regout (instructions));
  //NZCVregister NZCV(.regin (ALUflags), .write(NZCVwrite), .clk (clk), .reset (reset), .regout({n,z,c,v}));
	IFIDRegister IFIDStage(
		.pcIn(pc),
		.instIn(inst),
		.clk(clk), 
		.write(IFIDWrite), 
		.reset(reset), 
		.pcOut(pcIFIDtoIDEX),
		.instOut(instIFIDtoIDEX)
		);
	IDEXRegister IDEXStage(
    	.pcIn(pcIFIDtoIDEX), 
    	.instIn(instIFIDtoIDEX),
	  	.reg1In(readA),
		.reg2In(readB),
		.immIn(imm),
		.clk(clk), 
		.write(IDEXWrite), 
		.reset(IDEXReset), 
		.pcOut(pcIDEXtoEXMEM),
		.instOut(instIDEXtoEXMEM),
		.reg1Out(A),
		.reg2Out(B),
		.immOut(immToALU)
  		);
	EXMEMRegister EXMEMStage(
	    .pcIn(pcIDEXtoEXMEM),
	    .instIn(instIDEXtoEXMEM),
	    .ALUResultIn(ALUresult),
	    .reg2In(B),
	    .clk(clk),
	    .write(EXMEMWrite),
	    .reset(EXMEMWrite),
	    .pcOut(pcEXMEMtoMEMWB),
	    .instOut(instEXMEMtoMEMWB),
	    .ALUResultOut(ALUOut),
	    .reg2Out(BEXMEMtoMEMWB));
	MEMWBRegister MEMWBStage(
	    .instIn(instEXMEMtoMEMWB),
	    .mdrIn(readdata),
	    .ALUResultIn(ALUOut),
		.reg2In(BEXMEMtoMEMWB)
	    .clk(clk),
	    .write(MEMWBWrite),
	    .reset(MEMWBReset),
	    .instOut(instEXMEMtoMEMWB),
	    .mdrOut(mdr),
	    .ALUResultOut(ALUoutWB),
			.reg2Out(BWB)
	);
  register_1bit Z(.regin (ALUflags[2]), .reset (reset), .clk (clk), .write (NZCVwrite), .regout (z)); // EXMEM Stage register에 포함
  register_1bit C(.regin (ALUflags[1]), .reset (reset), .clk (clk), .write (NZCVwrite), .regout (c));
	
	//mux
	assign RFdst[0]=instIFIDtoIDEX[15:12];
	assign RFdst[1]=4'b1111;
	assign RFdst[2]=4'b1110;

	assign RFsrc[0]=mdr;
	assign RFsrc[1]=ALUoutWB;
	assign RFsrc[2]='h00000000; // ALUResult
	assign RFsrc[3]=BWB;

	assign regBread[0]=instIFIDtoIDEX[3:0];
	assign regBread[1]=instIFIDtoIDEX[15:12];

	assign ALUnum1[0]=pcIDEXtoEXMEM;
	assign ALUnum1[1]=A;
	assign ALUnum1[2]='h00000000;

	assign ALUnum2[0]='h00000004;
	assign ALUnum2[1]='h00000008;
	assign ALUnum2[2]=immToALU;
	assign ALUnum2[3]=B<<instIDEXtoEXMEM[11:7];

	assign writedata=BEXMEMtoMEMWB;
	assign memaddr=ALUOut;

	signalunit SignalControl(
		.clk (clk),
		.reset (reset),
		.flags (instructions[31:20]),
		.zero (z),
		.Mwrite (memwrite),
		.IRwrite (IRwrite),
		.Mread (memread),
		.regwrite (regwrite),
		.regdst (regdst),
		.regsrc (regsrc),
		.ALUsrcA (ALUsrcA),
		.ALUsrcB (ALUsrcB),
		.ALUop (instop),
		.NZCVwrite (NZCVwrite),
		.immsrc (immsrc),
		.regbdst (regBdst));
	
	// registerFile의 r15 갱신 여부를 결정. IDEX에 저장된 BEQ signal과 Z값이 모두 1일 때 enable.
	wire pcwrite;

	SignalBEQ signalBEQ(.inst (instIDEXtoEXMEM), .zero (z), .PCWrite (pcwrite))
	registerfile RegisterFile(
		.reg1 (instructions[19:16]),
		.reg2 (regBread[regBdst]),
		.regdst (RFdst[regdst]),
		.regsrc (RFsrc[regsrc]),
		.r15 (ALUresult),
		.clk (clk),
		.reset (reset),
		.we (regwrite),
		.PCWrite (pcwrite),
		.out1 (readA),
		.out2 (readB),
		.pc (pc));
 
	signextmux Immidiate(instructions[23:0],immsrc,imm);

	ALUopdecoder ALUopDecoder(
		.instop (instop),
		.aluop (ALUop));
	
	ALU32bit ALU(
		.inpa (ALUnum1[ALUsrcA]),
		.inpb (ALUnum2[ALUsrcB]),
		.cin (c),
		.aluop (ALUop),
		.result (ALUresult),
		.negative (ALUflags[3]),
		.zero (ALUflags[2]),
		.cout (ALUflags[1]),
		.overflow (ALUflags[0]));
	
endmodule
