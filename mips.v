// Data Memory Module
module dataMemory(clk,w_mem,address,write_data,read_data);
  input wire w_mem,clk;
  input wire [31:0] address  ;
  input wire [31:0] write_data;
  output wire [31:0] read_data;

  reg [31:0] memory[4095:0];
  
  assign read_data = memory[address>>2];
  
  always @(posedge clk)
    begin
      if(w_mem)
        memory[address>>2] <= write_data;
    end
endmodule

// Instruction Memory Module
module instMemory(clk,pc,instruction);
  input wire clk;
  input wire [31:0] pc;
  output wire [31:0] instruction;
  
  reg [31:0] memory[1023:0];

  assign instruction = memory[pc>>2];
endmodule

// Register file Module
module regFile(clk,w_reg, write_add, write_data, read_add1, read_add2, read_data1, read_data2);
  input wire w_reg,clk;
  input wire [4:0] write_add, read_add1 ,read_add2;
  input wire [31:0] write_data;
  output wire [31:0] read_data1 , read_data2;
  
  reg [31:0] registers[31:0];
  
  always @(posedge clk)
    begin
      if(w_reg && write_add!=0)
        registers[write_add] <= write_data;
    end
  
  assign read_data1 = registers[read_add1];
  assign read_data2 = registers[read_add2];
endmodule

// MIPs processor Module
module processor(clk,IR,j_flag,z_flag,jump_address);
  input wire clk;
  input wire [31:0] IR;
  output reg j_flag, z_flag;
  output reg [31:0] jump_address;
  
  //decoding
  wire [5:0] op , funct;
  wire [4:0] rs , rt , rd;
  wire [15:0] immediate;
  wire [25:0] j_address;
  assign op= IR[31:26];
  assign rs= IR[25:21];
  assign rt= IR[20:16];
  assign rd= IR[15:11];
  assign funct= IR[5:0];
  assign immediate= IR[15:0];
  assign j_address= IR[25:0];
  
  //Register File
  reg w_reg;
  reg [4:0] write_add;
  reg [31:0] reg_write_data;
  wire [31:0] read_data1 , read_data2;
  regFile RF(clk,w_reg, write_add, reg_write_data, rs, rt, read_data1, read_data2);
  
  //data memeory
  reg w_mem;
  wire [31:0] address;
  wire [31:0] mem_read_data;
  wire [31:0] mem_write_data;
  dataMemory D_memory (clk,w_mem,address,mem_write_data,mem_read_data);
  
  assign mem_write_data = read_data2;
  
  reg [31:0] alu_result;
  
  assign address = alu_result;
  
  //write_add for RF mux
  always @(*) begin
    if(op == 6'b000000)
      write_add = rd;
    else 
      write_add = rt;
  end
  //ALU
  always @(*) begin
    if(op == 6'b000000)
      begin
        if( funct == 6'b100000 )
          alu_result = read_data1 + read_data2;
        else
          alu_result = read_data1 - read_data2;
      end
    else if( op == 6'b100011 || op == 6'b101011)
      begin
        alu_result = read_data1 + {{16{immediate[15]}},immediate};
      end
    
  end
  
  reg memToReg;
  //control unit
  always @(*) begin
    case(op)
      6'b000000: //R_format
        begin
          w_reg = 1;
          w_mem = 0;
          memToReg = 0;
        end
      6'b100011: //lw
        begin
          w_reg = 1;
          w_mem = 0;
          memToReg = 1;
        end
      6'b101011: //sw
        begin
          w_reg = 0;
          w_mem = 1;
        end
      default:
        begin
          w_reg = 0;
          w_mem = 0;
          memToReg = 0;
        end
    endcase
  end
  //j
  always @(*) begin
    if(op == 6'b000010)
      begin
        j_flag = 1;
        jump_address = j_address<<2;
      end
    else 
      j_flag = 0;
  end
  // zero flag
  always @(*) begin
    if(alu_result == 0)
      z_flag = 1;
    else
      z_flag =0;
  end
  //memory to register mux
  always @(*) begin
    if(memToReg)
      reg_write_data = mem_read_data;
    else if (memToReg == 0)
      reg_write_data = alu_result;
      
  end
      
endmodule

// Program Counter Module
module programCounter(clk,j_flag,jump_address,pc);
  input wire clk ,j_flag;
  input wire [31:0] jump_address;
  output reg [31:0] pc;

  always @(posedge clk) begin
    if(j_flag == 1)
      pc <= {pc[31:28],jump_address[27:0]};
    else
      pc <= pc+4;
  end
endmodule

// Mips Module (Top Module)
module MIPS(input wire clk);
  wire [31:0] pc;
  wire [31:0] IR,jump_address;
  wire j_flag,z_flag;

  // instantiate instruction memory , mips and program counter modules
  instMemory Imemory(clk,pc,IR);
  processor p(clk,IR,j_flag,z_flag,jump_address);
  programCounter PC(clk,j_flag,jump_address,pc);
 
endmodule



// MIPs Test bench
module mips_tb;
  reg clk;

  //produce clk signal and inialize program counter
  initial begin
      clk = 0;
      mips.PC.pc = 0;
      forever
        #5 clk=~clk;
    end
   // instantiate the Top Module
   MIPS mips(clk);

    // initialize the instruction memory to test the processor
  initial begin
    mips.Imemory.memory[0] = {6'b000000,5'b01000,5'b01001,5'b01100,5'b00000,6'b100000} ;  // add R12,R8,R9  #R12=17
    mips.Imemory.memory[1] = {6'b101011,5'b00000,5'b01100,16'b0000000000000100} ;         // sw R12, 4[R0]  #mem[1]=17
    mips.Imemory.memory[2] = {6'b100011,5'b00000,5'b01111,16'b0000000000000100} ;         // lw R15, 4[R0]  #R15=mem[1]=17
    mips.Imemory.memory[3] = {6'b000010,26'b00000000000000000000001010} ;                 // j 40           #jump to add 10
    mips.Imemory.memory[10]= {6'b000000,5'b01111,5'b00111,5'b10000,5'b00000,6'b100010} ;  // sub R16,R15,R7 #R16=17-7=10
    #50;
    mips.Imemory.memory[11] = {6'b000000,5'd16,5'd10,5'd20,5'b00000,6'b100010} ;    // sub R20,R16,R10  #R20=0
    mips.Imemory.memory[12] = {6'b101011,5'd4,5'd20,16'd8} ;                        // sw R20, 8[R4]  #mem[3]=0
    mips.Imemory.memory[13] = {6'b100011,5'd4,5'd19,16'd8} ;                        // lw R19, 8[R4]  #R19=mem[3]=0
    mips.Imemory.memory[14] = {6'b000010,26'd20} ;                                  // j 80          #jump to add 20
    mips.Imemory.memory[20]= {6'b000000,5'd19,5'd24,5'd21,5'b00000,6'b100000} ;     // add R21,R19,R24 #R21=0+24=24
    #50;
    mips.Imemory.memory[21] = {6'b100011,5'd0,5'd25,16'd4} ;                        // lw R25, 4[R0]   #R25=mem[1]=17
    mips.Imemory.memory[22] = {6'b000010,26'd30} ;                                  // j 120           #jump to add 30
    mips.Imemory.memory[30] = {6'b000000,5'd25,5'd5,5'd26,5'b00000,6'b100000} ;     // add R26,R25,R5  #R26=17+5=22
    mips.Imemory.memory[31] = {6'b000000,5'd26,5'd24,5'd26,5'b00000,6'b100010} ;    // sub R26,R26,R24 #R26=22-24=-2
    mips.Imemory.memory[32]=  {6'b101011,5'd0,5'd26,16'd40} ;                       // sw R26, 40[R0]  #mem[10]=-2
    #50;
    mips.Imemory.memory[33] = {6'b100011,5'd8,5'd28,16'hfffc} ;                        // lw R28, -4[R8]   #R28=mem[1]=17
  end
  
  
  initial begin : RF_Dmemory
    //initialize the register file for testing
    integer i;
    for (i = 0; i < 32; i = i + 1) begin
      mips.p.RF.registers[i] = i;
    end
    
    #50;
    //after 50 sim steps display R12,R15,R16 ,memory[1] (should be R12=17=0x11,R15=17=0x11,R16=10=0xa,mem[1]=17=0x11) 
    $display("R12=0x%h R15=0x%h R16=0x%h",mips.p.RF.registers[12],mips.p.RF.registers[15],mips.p.RF.registers[16]);
    $display("mem[1]=0x%h",mips.p.D_memory.memory[1]);
    #50;
    //after 50 sim steps display R20,R19,R21 ,memory[3] (should be R20=0,R19=0,R21=24=0x18,mem[3]=0) 
    $display("R20=0x%h R19=0x%h R21=0x%h",mips.p.RF.registers[20],mips.p.RF.registers[19],mips.p.RF.registers[24]);
    $display("mem[3]=0x%h",mips.p.D_memory.memory[3]);
    #50;
    //after 50 sim steps display R25,R26,memory[10] (should be R25=17=0x11,R26=-2=0xfffffffe,mem[10]=-2=0xfffffffe) 
    $display("R25=0x%h R26=0x%h",mips.p.RF.registers[25],mips.p.RF.registers[26]);
    $display("mem[10]=0x%h",mips.p.D_memory.memory[10]);
    #10;
    //after 10 sim steps display R28 = mem[1]=17=0x11
    $display("R28=0x%h",mips.p.RF.registers[28]);
    $stop();
    
  end

  initial begin
    $monitor("time=%0d pc=0x%h IR=0x%h op=%0d w_reg=%0d reg_write_data=0x%h read_data1=%0d read_data2=%0d  w_mem=%0d  \n       address=0x%h mem_write_data =0x%h mem_read_data=0x%h alu_result=0x%h z_flag=%0d memToReg=%0d j_flag=%0d jump_address=0x%h",$time, mips.pc, mips.IR, mips.p.op, mips.p.w_reg, mips.p.reg_write_data, mips.p.read_data1, mips.p.read_data2, mips.p.w_mem, mips.p.address, mips.p.mem_write_data , mips.p.mem_read_data, mips.p.alu_result ,mips.z_flag, mips.p.memToReg ,mips.j_flag ,mips.jump_address);
    
  end
endmodule
