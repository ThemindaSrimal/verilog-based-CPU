
`timescale 1ns/100ps

module test_bench;                   //testbench for testing
    wire [31:0] PC;                  //32 bit PC as a wire
    wire busywait;                   //busywait as a wire
    wire ICbusywait;                 //busywait of instruction cache as a wire
    wire DCbusywait;                 //busywait of data cache as a wire
    wire memRead;                    //memRead as a wire
    wire memWrite;                   //memWrite as a wire
    wire [7:0] RESULT;               //8 bit RESULT as a wire
    wire [7:0] writedata;            //8 bit writedata as a wire
    wire [7:0] readdata;             //8 bit readdata as a wire
    
    wire dmRead;                     //dmRead as a wire
    wire dmWrite;                    //dmWrite as a wire
    wire [31:0] dmWritedata;         //32 bit dmWritedata as a wire
    wire [31:0] dmReaddata;          //32 bit dmReaddata as a wire
    wire [5:0] dmAddress;            //6 bit dmAddress as a wire
    wire dmBusywait;                 //dmBusywait as a wire
    wire [9:0] ICaddress;            //10 bit address which is used by instruction cache 
    wire ICread;                     //read sinal for instruction cache
    wire IMread;                     //read signal for instruction memory
    wire [5:0] IMaddress;            //6 bit address which is used by instruction memory
    wire [127:0] IMreaddata;         //128 bit instruction memory read data
    wire IMbusywait;                 //busywait siganl of instruction memory
    
    wire [31:0] INSTRUCTION;         //32 bit INSTRUCTION as a register
    reg CLK;                         //CLK as a register
    reg RESET;                       //RESET as a register
    
    
    reg [7:0] num[1023:0];           //array to hold instructions,1024 bytes
    
    cpu mycpu(PC,INSTRUCTION,CLK,RESET,busywait,memRead,memWrite,RESULT,writedata,readdata,ICaddress,ICread,ICbusywait);      //cpu module
    
    data_cache my_data_cache(CLK,RESET,memRead,memWrite,RESULT,writedata,readdata,DCbusywait,dmRead,dmWrite,dmWritedata,dmReaddata,dmAddress,dmBusywait);     //data cache module
    
    data_memory my_data_memory(CLK,RESET,dmRead,dmWrite,dmAddress,dmWritedata,dmReaddata,dmBusywait);     //data memory module
    
    instruction_cache my_instruction_cache(CLK,ICread,ICaddress,INSTRUCTION,ICbusywait,IMread,IMaddress,IMreaddata,IMbusywait,RESET);    //instruction cache module
    
    instruction_memory my_instruction_memory(CLK,IMread,IMaddress,IMreaddata,IMbusywait);           //instruction memory module
    
    initial begin
  
       //needed files for ploting the waveform 
       $dumpfile("dump.vcd");                     
	   $dumpvars(0,mycpu); 
	   $dumpvars(0,my_data_cache);
	   $dumpvars(0,my_data_memory);
	   $dumpvars(0,my_instruction_cache);
	   $dumpvars(0,my_instruction_memory);
	   
       
       //give values with time 
       CLK=1'b0;
       RESET=1'b1;
           
       #3;
       RESET=1'b0;
       
       
       #5000;
       $finish;      
    end
    
    assign busywait=DCbusywait || ICbusywait;       //set the value for busywait according DCbusywait and ICbusywait
  
    always 
        #4 CLK=~CLK;           //inverting the CLK in every 4 time units
     
    
endmodule
 
 
 //opcodes     loadi-00001000     mov-00000000     add-00000001     sub-00001001     and-00000010      or-00000011     j-00000100    beq-00000101
 //lwd-00000110     lwi-00001110     swd-00000111      swi-00001111
 //OUT_DATA-output of the twosComp
 //CHOOSED1-output of the mux which chooses between add and sub
 //CHOOSED2-output of the mux which chooses whether there is an immediate value to get
 
 
 
//cpu module
module cpu(PC,INSTRUCTION,CLK,RESET,busywait,memRead,memWrite,RESULT,writedata,readdata,ICaddress,ICread,ICbusywait);
     output [31:0] PC;                    //output port for 32 bit PC
     input busywait;                      //input port for busywait
     input [31:0] INSTRUCTION;            //input port for 32 bit INSTRUCTION
     input CLK;                           //input port for CLK
     input RESET;                         //input port for RESET
     input ICbusywait;
     
     output [7:0] RESULT;                  //8 bit RESULT as a wire
     output [9:0] ICaddress;
     output ICread;
     
     wire [2:0] SELECT;                   //3 bit SELECT as a wire
     wire [2:0] INADDRESS;                //3 bit INADDRESS as a wire
     wire [2:0] OUT1ADDRESS;              //3 bit OUT1ADDRESS as a wire
     wire [2:0] OUT2ADDRESS;              //3 bit OUT2ADDRESS as a wire
     wire [7:0] OUT1;                     //8 bit OUT1 as a wire
     wire [7:0] OUT2;                     //8 bit OUT2 as a wire 
     wire [7:0] IMMEDIATE;                //8 bit IMMEDIATE as a wire
     wire [7:0] OUT_DATA;                 //8 bit OUT_DATA as a wire
     wire [7:0] CHOOSED1;                 //8 bit CHOOSED1 as a wire
     wire [7:0] CHOOSED2;                 //8 bit CHOOSED2 as a wire
     wire AddSubChoose;                   //AddSubChoose as a wire
     wire ImmediateChoose;                //ImmdiateCoose as a wire
     wire WRITE;                          //WRITE as a wire
     wire [31:0] OFFSET;                  //32  bit OFFSET as a wire
     wire JumpIndicator;                  //JumpIndicator which indicates whether a branch or jump instruction, as a wire
     wire ZERO;                           //ZERO which indicates the whether the vlaue of (DATA1-DATA2) is zero or not, as a wire
     wire [31:0] initialPC;               //32 bit initialPC which is the value after adding 4 to current PC value, as a wire
     wire [31:0] offsetPC;                //32 bit offsetPC which is the value after adding OFFSET to initialPC value, as a wire
     wire [31:0] jumpPC;                  //32 bit jumpPC which is choosed according to the value of JumpIndicator, as a wire
     wire [31:0] finalPC;                 //32 bit finalPC as a wire
     wire BranchIndicator;                //BranchIndicator which indicates whether a branch, as a wire
     output memRead;                      //memRead as a wire
     output memWrite;                     //memWrite as a wire
     output [7:0] writedata;              //writedata as a wire
     output [7:0] readdata;               //readdata as a wire
     wire readDataChoose;                 //readDataChoose as a wire
     wire [7:0] finalResult;              //finalResult as a wire
     
     
     //instantiation of modules with in top-level cpu module 
     
     //control_unit module instantiation
     control_unit myctrl(INSTRUCTION,SELECT,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,IMMEDIATE,AddSubChoose,ImmediateChoose,WRITE,OFFSET,JumpIndicator,RESET,BranchIndicator,memRead,memWrite,readDataChoose,busywait);
     
     //reg_file module instantiation
     reg_file myregfile(finalResult,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,WRITE,CLK,RESET,JumpIndicator,BranchIndicator,writedata,readdata,readDataChoose); 
     
     //twosComp module instantiation
     twosComp mytwosComp(OUT2,OUT_DATA); 
     
     //instantiation of mux module which chooses between add and sub
     mux addsubchoose(OUT2,OUT_DATA,AddSubChoose,CHOOSED1);
     
     //instantiation of mux module which chooses whether there is an immediate value to get
     mux immediatechoose(IMMEDIATE,CHOOSED1,ImmediateChoose,CHOOSED2);   
     
     //alu module instantiation
     alu myalu(OUT1,CHOOSED2,SELECT,RESULT,ZERO);
     
     //programCounter module instantiation
     programCounter myPC(CLK,initialPC,RESET,finalPC,PC,busywait,ICaddress,ICread,ICbusywait);
     
     //instantiation adder which adds the OFFSET value to initialPC
     adder_offset_PC myAdderOffset(initialPC,OFFSET,offsetPC);
     
     //instantiation of mux module which chooses between initialPC and jumpPC
     muxZeroJumpChoose myzeroChoose(initialPC,offsetPC,finalPC,BranchIndicator,JumpIndicator,ZERO);
     
     //instantiation of mux module which chooses between alu result and readdata
     mux readDataMux(RESULT,readdata,readDataChoose,finalResult);
     
endmodule



//control_unit module
module control_unit(INSTRUCTION,SELECT,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,IMMEDIATE,AddSubChoose,ImmediateChoose,WRITE,OFFSET,JumpIndicator,RESET,BranchIndicator,memRead,memWrite,readDataChoose,busywait);
    input [31:0] INSTRUCTION;                           //input port for 32 bit INSTRUCTION
    input RESET;                                        //input port for RESET
    input busywait;                                     //input port for busywait
    output [2:0] SELECT;                                //output port for 3 bit SELECT
    output [2:0] INADDRESS;                             //output port for 3 bit INADDRESS
    output [2:0] OUT1ADDRESS;                           //output port for 3 bit OUT1ADDRESS
    output [2:0] OUT2ADDRESS;                           //output port for 3 bit OUT2ADDRESS
    output [7:0] IMMEDIATE;                             //output port for 8 bit IMMEDIATE
    output AddSubChoose;                                //output port for AddSubChoose
    output ImmediateChoose;                             //output port for ImmediateChoose
    output WRITE;                                       //output port for WRITE
    output [31:0] OFFSET;                               //output port for 32 bit OFFSET
    output JumpIndicator;                               //output port for JumpIndicator
    output BranchIndicator;                             //output port for BranchIndicator
    output memRead;                                     //output port for memRead
    output memWrite;                                    //output port for memWRite
    output readDataChoose;                              //output port for readDataChoose
    
    reg [2:0] SELECT;                                   //3 bit SELECT as a register
    reg [2:0] INADDRESS;                                //3 bit INADDRESS  as a register
    reg [2:0] OUT1ADDRESS;                              //3 bit OUT1ADDRESS as a register
    reg [2:0] OUT2ADDRESS;                              //3 bit OUT2ADDRESS as a register              
    reg [7:0] IMMEDIATE;                                //8 bit IMMEDIATE as a register
    reg AddSubChoose;                                   //AddSubChoose as a register
    reg ImmediateChoose;                                //ImmediateChoose as a register
    reg WRITE;                                          //WRITE as a register
    reg [31:0] OFFSET;                                  //32 bit OFFSET as a register
    reg JumpIndicator;                                  //JumpIndicator as a register
    reg BranchIndicator;                                //BranchIndicator as a register
    reg [31:0] temp;                                    //32 bit temp register for temporary purposes
    reg memRead;                                        //memRead as a register
    reg memWrite;                                       //memWrite as a register
    reg readDataChoose;                                 //readDataChoose as a register
    
    always @(busywait) begin
        if(busywait==1'b0) begin                        //when busywait becomes zero memRead and memWrite signals also become zero
              memRead=1'b0;
              memWrite=1'b0;
        end
    end
    
    always @(INSTRUCTION)                               //instruction decoding using INSTRUCTION
        begin                                         
           INADDRESS=INSTRUCTION[18:16];                //get the INADDRESS from the INSTRUCTION
           OUT1ADDRESS=INSTRUCTION[10:8];               //get the OU1ADDRESS from the INSTRUCTION
           OUT2ADDRESS=INSTRUCTION[2:0];                //get the OUT2ADDRESS from the INSTRUCTION
           IMMEDIATE=INSTRUCTION[7:0];                  //get the IMMEDIATE from the INSTRUCTION
           temp=INSTRUCTION[31:0];                      //get the temp value
           OFFSET={{22{temp[23]}},temp[23:16],2'b00};   //setting the OFFSET by sign extending and shifing left by two
           
           #1;                                          //decode delay is added here
           
           SELECT=INSTRUCTION[26:24];                   //get the SELECT value from INSTRUCTION   
           
           if((INSTRUCTION[31:24]==8'b00001000)|(INSTRUCTION[31:24]==8'b00001110)|(INSTRUCTION[31:24]==8'b00001111)) begin    //check the opcode value
              ImmediateChoose=1'b0;                     //set ImmediateChoose to low
           end else begin
              ImmediateChoose=1'b1;                     //set ImmediateChoose to high
           end
           
           
           if(INSTRUCTION[31:24]==8'b00001001) begin    //check the opcode value
              AddSubChoose=1'b1;                        //set the AddSubChoose to high
           end else begin 
              AddSubChoose=1'b0;                        //set the AddSubChoose to low
           end
           
           
           if(INSTRUCTION[31:24]==8'b00000100) begin    //check the opcode value
              JumpIndicator=1'b1;                       //set the JumpIndicator to high
           end else begin 
              JumpIndicator=1'b0;                       //set the JumpIndicator to low
           end
           
           
           if(INSTRUCTION[31:24]==8'b00000101) begin    //check the opcode value
              BranchIndicator=1'b1;                     //set the BranchIndicator to high
           end else begin
              BranchIndicator=1'b0;                     //set the BranchIndicator to low
           end
           
            //check the opcode value
           if((INSTRUCTION[26:24]==3'b000) | (INSTRUCTION[26:24]==3'b001) | (INSTRUCTION[26:24]==3'b010) | (INSTRUCTION[26:24]==3'b011) | (INSTRUCTION[26:24]==3'b100) | (INSTRUCTION[26:24]==3'b101)) begin  
               WRITE=1'b1;                              //enable the WRITE
           end else begin
               WRITE=1'b0;                              //disable the WRITE
           end
           
           
            //for data memory by chcking opcode value
           if(INSTRUCTION[26:24]==3'b110) begin         //check the opcode value
              memRead=1'b1;                             //set the memRead to high
              readDataChoose=1'b1;                      //set the readdataChoose to high
           end else begin
              memRead=1'b0;                             //set the memRead to low
              readDataChoose=1'b0;                      //set the readDataChoose to low
           end
           
           if(INSTRUCTION[26:24]==3'b111) begin         //check the opcode value
              memWrite=1'b1;                            //set the memWrite to high
           end else begin
              memWrite=1'b0;                            //set the memWrite to low
           end
    end
    
    always @(RESET) begin                               //sensitive to RESET value
         if(RESET) begin                                //if RESET is enabled
            JumpIndicator=1'b0;                         //make JumpIndicator as 0
             OFFSET=32'd0;                              //make OFFSET as 32 bit zero
         end
    end
    
endmodule



//alu module
module alu(DATA1,DATA2,SELECT,RESULT,ZERO);  //module instantiation
    input [7:0] DATA1;                       //8 bit input port for DATA1
    input [7:0] DATA2;                       //8 bit input port for DATA2
    input [2:0] SELECT;                      //3 bit input port for SELECT
    output [7:0] RESULT;                     //8 bit output port for RESULT
    reg [7:0] RESULT;                        //declare RESULT as a register
    output ZERO;                             //output port for ZERO
    wire ZERO;                               //ZERO as a wire
    
    always @ (DATA1,DATA2,SELECT) begin      //executing conditions
       case(SELECT)                          //use of case structure for SELECT 
         3'b000: #1 RESULT=DATA2;            //FORWARD function  
         3'b001: #2 RESULT=DATA1+DATA2;      //ADD function 
         3'b010: #1 RESULT=DATA1&DATA2;      //AND function
         3'b011: #1 RESULT=DATA1|DATA2;      //OR function
         3'b110: #2 RESULT=DATA2;            //LW function
         3'b111: #2 RESULT=DATA2;            //SW function
         default: #1 RESULT=0;               //dealing with unused bit combinations of SELECT
       endcase
    end
    
    assign #3 ZERO=((DATA1-DATA2)==0)? 1'b1:1'b0;   //check the value of (DATA1-DATA2) and set the value of ZERO according to that
    
endmodule                                   //end module



//reg_file module
module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,WRITE,CLK,RESET,JumpIndicator,BranchIndicator,writedata,readdata,readDataChoose);     //reg_file module instantiation
     input [7:0] IN;                    //8 bit input port for IN
     output [7:0] OUT1;                 //8 bit output port for OUT1
     output [7:0] OUT2;                 //8 bit output port for OUT2	
     output [7:0] writedata;            //8 bit output port for writedata
     input [2:0] INADDRESS;             //3 bit input port for INADDRESS
     input [2:0] OUT1ADDRESS;           //3 bit input port for OUT1ADDRESS
     input [2:0] OUT2ADDRESS;           //3 bit input port for OUT1ADDRESS
     input WRITE;                       //input port for WRITE
     input CLK;                         //input port for CLK
     input RESET;                       //input port for RESET
     input BranchIndicator;             //input port for BranchIndicator
     input JumpIndicator;               //input port for JumpIndicator
     input [7:0] readdata;              //input port for readdata
     input readDataChoose;              //input port for readDataChoose
     reg [7:0] regNum[7:0];             //represent registers as an array of words
     wire [7:0] OUT1;                   //OUT1 as a wire
     wire [7:0] OUT2;                   //OUT2 as a wire
     wire [7:0] writedata;              //writedata as a wire
        
     integer i;                         //declare an integer variable
     always @(posedge CLK) begin        //at the positive edge of the CLK
       if (!RESET && WRITE && !JumpIndicator && !BranchIndicator && !readDataChoose) #1 begin   //check whether WRITE is enabled and RESET,JumpIndicator,BranchIndicator,readDataChoose are disabled
	     regNum[INADDRESS] = IN;        //write data present on IN port to input register specified by the INADDRESS
	   end 
	   if (!RESET && !WRITE && !JumpIndicator && !BranchIndicator && readDataChoose) #1 begin   //check whether readDataChoose is enabled and RESET,JumpIndicator,BranchIndicator,WRITE are disabled
	     regNum[INADDRESS] = IN;      //write data present on readdata port to input register specified by the INADDRESS
	   end 
     end
     
	 assign #2 OUT1=regNum[OUT1ADDRESS];    //register identified by OUT1ADDRESS is read and value is loaded onto OUT1
     assign #2 OUT2=regNum[OUT2ADDRESS];    //register identified by OUT2ADDRESS is read and value is loaded onto OUT2 
     
     assign #2 writedata=regNum[OUT1ADDRESS];    //register identified by OUT1ADDRESS is read and value is loaded onto writedata
     
     always @(RESET) begin                 //at the RESET
        if (RESET) #2 begin                //check whether RESET is enabled
        for(i=0;i<8;i=i+1) begin
           regNum[i]<=8'b00000000;         //make the values of registers as zeros
        end
        end
     end 
endmodule                                  //end module



//twosComp module
module twosComp(IN_DATA,OUT_DATA);        //twosComp module instantiation
    input [7:0] IN_DATA;                  //8 bit input port
	output [7:0] OUT_DATA;                //8 bit output port
	reg [7:0] OUT_DATA;                   //OUT as a register
	
	always @(IN_DATA) begin
      #1 OUT_DATA=~IN_DATA+1;             //get the two's complement value of IN
    end
endmodule                                 //end module



//programCounter module
module programCounter(CLK,initialPC,RESET,finalPC,PC,busywait,ICaddress,ICread,ICbusywait);
     input CLK;                           //input port for CLK
     input RESET;                         //input port for PC
     input [31:0] finalPC;                //32 bit input port for finalPC
     input busywait;                      //input port for busywait
     input ICbusywait;
     output [31:0] initialPC;             //output port for initialPC
     reg [31:0] initialPC;                //initialPC as a register
     output [31:0] PC;                    //32 bit output port for PC
     reg [31:0] PC;                       //PC as a register
     output reg [9:0] ICaddress;          //output port for ICaddress
     output reg ICread;                   //output port for ICread
     
     always @(PC) begin
         if(PC!=-4) begin
            ICaddress=PC[9:0];               //provide the address for instruction cache
            ICread=1'b1;                     //read signal for instruction cache
         end
     end         
     
     always @(ICbusywait) begin
       if(!ICbusywait) begin
		    ICread=1'b0;                  //make ICread as zero 
       end
     end
     
     always @(posedge CLK)  begin         //at the positive edge of the CLK
        #1 if(!RESET && !busywait)  begin      //delay of 2 time units is added here, PC update delay-#1,adder delay-#1
            PC=finalPC[31:0];             //write to PC register
            #1;                           //latency of adder which increment the value of PC
            initialPC=PC+32'd4;           //increment the PC by 4
        end
     end
     
     always @(RESET) begin               //sensitive to RESET
        if(RESET) #1 begin               //if RESET is enabled
           initialPC=-4;                 //make PC as -4
        end
     end 
endmodule



//adder_offset_PC module
module adder_offset_PC(initialPC,OFFSET,offsetPC);
     input [31:0] initialPC;            //32 bit input port for initialPC
     input [31:0] OFFSET;               //32 bit input port for OFFSET
     output [31:0] offsetPC;            //32 bit output port for offsetPC
     reg [31:0] offsetPC;               //32 bit offsetPC as a register
     
     always @(OFFSET) begin
        #2 offsetPC=initialPC+OFFSET;    //add OFFSET to initialPC with a delay of 1 time unit
     end
endmodule


//mux module
module mux(IN0,IN1,SEL,OUT);        //module instantiation
     input [7:0] IN0;               //8 bit input port
     input [7:0] IN1;               //8 bit input port
     input SEL;                     //input port SEL
     output [7:0] OUT;              //8 bit output port
     reg [7:0] OUT;                 //OUT as a register
     
     always @(IN0,IN1,SEL)      
     begin
        if (SEL==1'b0) begin        //check whether SEL=0
           OUT=IN0;                 //make OUT as IN0 
        end else begin
           OUT=IN1;                 //make OUT as IN1
        end    
     end
endmodule                           //end module


//advanced mux for jump and branch instructions
module muxZeroJumpChoose(IN0,IN1,OUT,BranchIndicator,JumpIndicator,ZERO);   //module instantiation
     input [31:0] IN0;               //8 bit input port
     input [31:0] IN1;               //8 bit input port
     input SEL;                      //input port SEL
     input BranchIndicator;          //input port for BranchIndicator
     input JumpIndicator;            //input port for JumpIndicator
     input ZERO;                     //input port for ZERO
     output [31:0] OUT;              //8 bit output port
     reg [31:0] OUT;                 //OUT as a register
     reg temp;                       //temp as a register
     
     always @(IN0,IN1,ZERO)      
     begin
        temp=(JumpIndicator | (BranchIndicator & ZERO));     //set a condition for mux  
        if (temp) begin             //check the value of temp
           OUT=IN1;                 //make OUT as IN0 
        end else begin
           OUT=IN0;                 //make OUT as IN1
        end    
     end
endmodule                           //end module




//module for data memory
module data_memory(clock,reset,read,write,address,writedata,readdata,busywait);
input clock;
input reset;
input read;
input write;
input [5:0] address;
input [31:0] writedata;
output reg [31:0] readdata;
output reg busywait;
//Declare memory array 256x8-bits 
reg [7:0] memory_array [255:0];

//Detecting an incoming memory access
reg readaccess, writeaccess;
always @(read, write,address)
begin
	busywait = (read || write)? 1 : 0;
	readaccess = (read && !write)? 1 : 0;
	writeaccess = (!read && write)? 1 : 0;
end

//Reading & writing
always @(posedge clock)
begin
	if(readaccess)
	begin
		readdata[7:0]   = #40 memory_array[{address,2'b00}];
		readdata[15:8]  = #40 memory_array[{address,2'b01}];
		readdata[23:16] = #40 memory_array[{address,2'b10}];
		readdata[31:24] = #40 memory_array[{address,2'b11}];
		busywait = 0;
		readaccess = 0;
	end
	if(writeaccess)
	begin
		memory_array[{address,2'b00}] = #40 writedata[7:0];
		memory_array[{address,2'b01}] = #40 writedata[15:8];
		memory_array[{address,2'b10}] = #40 writedata[23:16];
		memory_array[{address,2'b11}] = #40 writedata[31:24];
		busywait = 0;
		writeaccess = 0;
	end
end

integer i;

//Reset memory
always @(posedge reset)
begin
    if (reset)
    begin
        for (i=0;i<256; i=i+1)
            memory_array[i] = 0;
        
        busywait = 0;
		readaccess = 0;
		writeaccess = 0;
    end
end

endmodule



//module for data cache
module data_cache(clock,reset,read,write,address,writedata,readdata,busywait,dmRead,dmWrite,dmWritedata,dmReaddata,dmAddress,dmBusywait);

    input clock;                      //input port for clock
    input reset;                      //input port for reset
    input read;                       //input port for read signal from control unit
    input write;                      //input port for write signal from control unit
    input [7:0] address;              //3 bit address as a input
    input [7:0] writedata;            //8 bit writedata as a input
    output reg [7:0] readdata;        //8 bit output port for readdata
    output reg busywait;              //output port for busywait
    
    output reg dmRead;                //output port dmRead for data memory to read data from data memory
    output reg dmWrite;               //output port dmWrite for data memory to write data to data memory
    output reg [31:0] dmWritedata;    //32 bit output port dmWritedata to provide writing data to data memory
    input [31:0] dmReaddata;          //32 bit input port dmReaddata to get reading data from data memory
    output reg [5:0] dmAddress;       //6 bit output port for data memory accessing address
    input dmBusywait;                 //input port for getting dmBusywait signal from data memory
    
    always @(dmBusywait) begin
        busywait=dmBusywait;         //busywait signal vaires according to dmBusywait signal in data memory
    end
    
    always @(read,write) begin
        if(read || write) begin
           busywait=1'b1;           //whenever a read or write signal comes from control unit busywait becomes 1
        end
    end
    
    integer i;
    
    reg [7:0] cache_mem [31:0];     //cache memorty of 32 bytes
    reg valid [7:0];                //valid array
    reg dirty [7:0];                //dirty array
    reg [2:0] cacheTag [7:0];       //cacheTag array
    
    reg [2:0] tag;                  //tag,index and offset given by address
    reg [2:0] index;
    reg [1:0] offset;
    reg hit;                        //indicates whether a hit or not
    reg [7:0] needDataBlock;        //requested data word
    reg needValid;                  //valid bit value corresponding to requested data word
    reg needDirty;                  //dirty bit value corresponding to requested data word
    reg [2:0] needTag;              //cache tag value corresponding to requested data word
    reg [2:0] flag;                 //this was added just to see whether correct operation is executing
    reg [31:0] temp;                //register for holding the read data from data memory

    always @(address) begin
    
       offset=address[1:0];         //after address is received offset,index and tag values are splitted
       index=address[4:2];
       tag=address[7:5];
       #1;                          //delay for extracting values for requested data word 
       needDataBlock=cache_mem[4*index+offset];
       needValid=valid[index];
       needDirty=dirty[index];
       needTag[2:0]=cacheTag[index];
       #0.9;                         //tag comparison and validation latency
       
       if((tag[2:0]==needTag[2:0]) && (needValid==1)) begin
           hit=1'b1;
       end else begin 
           hit=1'b0;
       end
       
       //read hit operation
       if(!write && read) begin                       
               if(hit) begin                            //check whether a hit or not
                   readdata=needDataBlock[7:0];         //read the needed data
                   flag=3'b010;                         
                   busywait=1'b0;                       //in a read hit had to make busywait zero before positive clock edge for avoding confusions in changing same signal with no time
               end
       end
       
    end
   
    always @(dmBusywait) begin
       if((dmBusywait==1'b0) && !hit && needDirty && !read && !write) begin         //in (read miss,dirty) or (write miss,dirty) instructions
          busywait=1'b1;
          #1 {cache_mem[4*index],cache_mem[4*index+1],cache_mem[4*index+2],cache_mem[4*index+3]}=temp;    //write to cache with a latency of #1
          #2 hit=1'b1;                                                              //change the value of hit signal after #2 
       end
    end
    
    always @(posedge clock) begin
         if(hit && read && !write) begin
            busywait=1'b0;                             //in a hit,busywait is deasserted in next positve clock edge
            hit=1'b0;                                  //making hit as zero
         end
         if(hit && !read && write) begin
            busywait=1'b0;
            #1 hit=1'b0;
         end
         if(hit && !read && !write) begin
            busywait=1'b0;
            hit=1'b0;
         end
    
    end
    
    always @(posedge reset) begin                      //at the positive edge of reset
        if(reset) begin
            for(i=0;i<8;i=i+1) begin
               valid[i]=0;                            //making valid bits to zero
               dirty[i]=0;                            //making dirty bits to zero 
               tag[i]=3'bxxx;                         //tag values are don't care
            end
            for(i=0;i<32;i=i+1) begin
               cache_mem[i]=0;                        //making cache memory zero
            end
            dmWrite=1'b0;                             //making dmWrite to zero
            dmRead=1'b0;                              //making dmRead to zero
        end
    end
    
    //write hit operation
    always @(posedge clock) begin
         if(write && !read) begin                    //if write signal is present
              if(hit) begin                          //if a hit
                   flag=3'b001;
                   busywait=1'b0;                    //deasserted busywait
                   dirty[index]=1'b1;                                   //update dirty bit
                   valid[index]=1'b1;                                   //update valid bit
                   #1 cache_mem[4*index+offset]=writedata[7:0];         //write data provided by cpu to correct data word
              end 
              
         end
    end
    
    always @(posedge clock) begin
       if(!hit) begin                 
            if(write && !read && !needDirty) begin                      //write miss and not dirty
   
                  flag=3'b011;
                  dmWrite=1'b0;                                         
                  dmRead=1'b1;                                //dmRead signal is enabled                            
                  dmAddress=address[7:2];                     //dmAddress is provided
                  temp=dmReaddata[31:0];                      //read from data memory
                  busywait=1'b1;
                  
                  #1;                                         //cache writing latency
                  if(dmBusywait==1'b0) begin
                     {cache_mem[4*index],cache_mem[4*index+1],cache_mem[4*index+2],cache_mem[4*index+3]}=temp;      //write to cache
                     dirty[index]=1'b1;                          //update dirty bit
                     valid[index]=1'b1;                          //update valid bit
                     cacheTag[index]=tag;                        //update tag
                     cache_mem[4*index+offset]=writedata[7:0];   //write to correct data word
                     #2 hit=1'b1;
                  end
                  
            end else if(write && !read && needDirty) begin              //write miss and dirty  
             
                  flag=3'b100;
                  dmRead=1'b0;                                  
                  dmWrite=1'b1;                               //dmWrite signal is enabled
                  dmAddress=address[7:2];                     //dmAddress is provided
                  dmWritedata={cache_mem[4*index],cache_mem[4*index+1],cache_mem[4*index+2],cache_mem[4*index+3]};      //write back is happening
                  
                  if(dmBusywait==0) begin                     //change values of dmWrite and dmRead for data reading from data memory
                     dmWrite=1'b0;
                     dmRead=1'b1;
                  end
                  temp=dmReaddata[31:0];                      //read from data memory
                  
                  if(dmBusywait==1'b0) begin
                     dirty[index]=1'b1;                          //update dirty bit
                     valid[index]=1'b1;                          //update valid bit
                     cacheTag[index]=tag;                        //update tag
                     cache_mem[4*index+offset]=writedata[7:0];   //write to correct data word
                  end
                 
            end else if(!write && read && !needDirty) begin             //read miss and not dirty
            
                 flag=3'b101;
                 dmWrite=1'b0;
                 dmRead=1'b1;                                 //dmRead signal is enabled
                 dmAddress=address[7:2];                      //dmAddress is provided
                 temp=dmReaddata[31:0];                       //read to data memory
                 busywait=1'b1;
                 #1;                                          //cache writing latency
                 
                 if(dmBusywait==1'b0) begin
                    {cache_mem[4*index],cache_mem[4*index+1],cache_mem[4*index+2],cache_mem[4*index+3]}=temp;     //write to cache
                    cacheTag[index]=tag;                         //update tag
                    valid[index]=1'b1;                           //update valid bit
                    readdata=cache_mem[4*index+offset];          //read the needed data
                    #2 hit=1'b1;
                 end
              
            end else if(!write && read && needDirty) begin              //read miss and dirty
            
                  flag=3'b110;
                  dmRead=1'b0;
                  dmWrite=1'b1;                               //dmWrite signal is enabled
                  dmAddress=address[7:2];                     //dmAddress is provided
                  dmWritedata={cache_mem[4*index],cache_mem[4*index+1],cache_mem[4*index+2],cache_mem[4*index+3]};      //write back is happening
                  
                  if(dmBusywait==0) begin                     //change values of dmWrite and dmRead for data reading from data memory
                     dmWrite=1'b0;
                     dmRead=1'b1;
                  end
                  temp=dmReaddata[31:0];                      //read from data memory
                  
                  if(dmBusywait==1'b0) begin
                      cacheTag[index]=tag;                        //update tag
                      dirty[index]=1'b0;                          //update dirty
                      readdata=cache_mem[4*index+offset];         //read the needed data
                  end
            end
       end
    end
endmodule




//module for instruction memory
module instruction_memory(clock,read,address,readdata,busywait);

    input clock;
    input read;
    input[5:0] address;
    output reg [127:0] readdata;
    output reg busywait;

    reg readaccess;

   //Declare memory array 1024x8-bits 
   reg [7:0] memory_array [1023:0];

//Initialize instruction memory
initial
begin
    busywait = 0;
    readaccess = 0;

    // Sample program given below. You may hardcode your software program here, or load it from a file:
    {memory_array[10'd3],  memory_array[10'd2],  memory_array[10'd1],  memory_array[10'd0]}  = 32'b00001000000000000000000000010010;   //loadi 0 0x12
    {memory_array[10'd7],  memory_array[10'd6],  memory_array[10'd5],  memory_array[10'd4]}  = 32'b00001000000000010000000000100001;   //loadi 1 0x21
    {memory_array[10'd11], memory_array[10'd10], memory_array[10'd9],  memory_array[10'd8]}  = 32'b00000001000000100000000000000001;   //add 2 0 1
    {memory_array[10'd15], memory_array[10'd14], memory_array[10'd13], memory_array[10'd12]} = 32'b00000011000000110000001000000000;   //or 3 2 0
    {memory_array[10'd19], memory_array[10'd18], memory_array[10'd17], memory_array[10'd16]} = 32'b00000000000001000000000000000001;   //mov 4 1
    {memory_array[10'd23], memory_array[10'd22], memory_array[10'd21], memory_array[10'd20]} = 32'b00001001000001010000001000000011;   //sub 5 2 3
    {memory_array[10'd27], memory_array[10'd26], memory_array[10'd25], memory_array[10'd24]} = 32'b00000010000001100000000000000101;   //and 6 0 5
    {memory_array[10'd31], memory_array[10'd30], memory_array[10'd29], memory_array[10'd28]} = 32'b00000100000000010000000000000000;   //j 0x01
    {memory_array[10'd35], memory_array[10'd34], memory_array[10'd33], memory_array[10'd32]} = 32'b00001000000001110000000001010100;   //loadi 7 0x54
    {memory_array[10'd39], memory_array[10'd38], memory_array[10'd37], memory_array[10'd36]} = 32'b00001000000000100000000000110010;   //loadi 2 0x32
    {memory_array[10'd43], memory_array[10'd42], memory_array[10'd41], memory_array[10'd40]} = 32'b00001000000000110000000001000101;   //loadi 3 0x45
    {memory_array[10'd47], memory_array[10'd46], memory_array[10'd45], memory_array[10'd44]} = 32'b00001000000001000000000001100001;   //loadi 4 0x61
    {memory_array[10'd51], memory_array[10'd50], memory_array[10'd49], memory_array[10'd48]} = 32'b00001111000000000000000100010100;   //swi 1 0x14
    {memory_array[10'd55], memory_array[10'd54], memory_array[10'd53], memory_array[10'd52]} = 32'b00000111000000000000001000000011;   //swd 2 3
    {memory_array[10'd59], memory_array[10'd58], memory_array[10'd57], memory_array[10'd56]} = 32'b00001110000001010000000001100001;   //lwi 5 0x61
    {memory_array[10'd63], memory_array[10'd62], memory_array[10'd61], memory_array[10'd60]} = 32'b00001111000000000000010001100010;   //swi 4 0x62
    {memory_array[10'd67], memory_array[10'd66], memory_array[10'd65], memory_array[10'd64]} = 32'b00001110000001100000000001100111;   //lwi 6 0x67
    {memory_array[10'd71], memory_array[10'd70], memory_array[10'd69], memory_array[10'd68]} = 32'b00001111000000000000001000110101;   //swi 2 0x35
    {memory_array[10'd75], memory_array[10'd74], memory_array[10'd73], memory_array[10'd72]} = 32'b00001000000001110000000000010010;   //loadi 7 0x12
    {memory_array[10'd79], memory_array[10'd78], memory_array[10'd77], memory_array[10'd76]} = 32'b00000101111101010000000000000111;   //beq 0xF5 0 7
    
end

//Detecting an incoming memory access
always @(read)
begin
    busywait = (read)? 1 : 0;
    readaccess = (read)? 1 : 0;
end

//Reading
always @(posedge clock)
begin
    if(readaccess)
    begin
        readdata[7:0]     = #40 memory_array[{address,4'b0000}];
        readdata[15:8]    = #40 memory_array[{address,4'b0001}];
        readdata[23:16]   = #40 memory_array[{address,4'b0010}];
        readdata[31:24]   = #40 memory_array[{address,4'b0011}];
        readdata[39:32]   = #40 memory_array[{address,4'b0100}];
        readdata[47:40]   = #40 memory_array[{address,4'b0101}];
        readdata[55:48]   = #40 memory_array[{address,4'b0110}];
        readdata[63:56]   = #40 memory_array[{address,4'b0111}];
        readdata[71:64]   = #40 memory_array[{address,4'b1000}];
        readdata[79:72]   = #40 memory_array[{address,4'b1001}];
        readdata[87:80]   = #40 memory_array[{address,4'b1010}];
        readdata[95:88]   = #40 memory_array[{address,4'b1011}];
        readdata[103:96]  = #40 memory_array[{address,4'b1100}];
        readdata[111:104] = #40 memory_array[{address,4'b1101}];
        readdata[119:112] = #40 memory_array[{address,4'b1110}];
        readdata[127:120] = #40 memory_array[{address,4'b1111}];
        busywait = 0;
        readaccess = 0;
    end
end
endmodule


//module for instruction cache
module instruction_cache(clock,ICread,ICaddress,ICreaddata,ICbusywait,IMread,IMaddress,IMreaddata,IMbusywait,reset);                   //IC=instruction cache,IM=instruction memory
    
    input clock;                          //input port for clock
    input ICread;                         //input port for ICread
    input [9:0] ICaddress;                //input port for instruction cache address
    output reg [31:0] ICreaddata;         //output port for provide relevant instruction
    output reg ICbusywait;                //output port for instruction cache busywait
    output reg IMread;                    //output port for instruction memory read signal
    output reg [5:0] IMaddress;           //output port for provide instruction memory address
    input [127:0] IMreaddata;             //input port for get the instruction memory read data
    input IMbusywait;                     //input port for instruction memory busywait signal
    input reset;                          //input port for reset
    
    reg [31:0] cache_mem [31:0];          //cache memory
    reg valid [7:0];                      //array for indicating valid bit
    reg [1:0] offset;                     
    reg [2:0] index;
    reg[2:0] tag;
    reg hit;                              //indicates whether a hit or not
    reg [31:0] needInsWord;               //required instruction word
    reg [2:0] addressTag [7:0];           //for indicate address tag
    reg [2:0] needTag;                    //required tag
    reg [127:0] temp;                     //temporary variable
    reg needValid;                        //required valid bit
    
    always @(IMbusywait) begin
        ICbusywait=IMbusywait;            //set the instruction cache busywait signal according to instruction memory busywait signal
    end
    
    always @(ICread) begin
        if(ICread) begin
           ICbusywait=1'b1;               //set the ICbusywait value according to ICread value
        end 
    end
    
    always @(ICaddress) begin
    
       if(ICaddress>=0) begin             //check the validity of ICaddress
           offset=ICaddress[3:2];         //splitting of instruction cache address
           index=ICaddress[6:4];
           tag=ICaddress[9:7];
           #1;                            //delay for selecting required instruction word from the block
           needInsWord=cache_mem[4*index+offset];        //select required instruction word
           needValid=valid[index];                       //select required valid bit value
           needTag[2:0]=addressTag[index];               //select requried address tag
           #1;                                           //tag comparision latency
       
       if((tag[2:0]==needTag[2:0]) && (needValid==1)) begin      //check whether a hit or not
           hit=1'b1;
       end else begin 
           hit=1'b0;
       end
    
       if(ICread) begin                       
               if(hit) begin                            
                   ICreaddata=needInsWord[31:0];         //provide the instruction
               end
       end
       end
    end
    
    
    always @(posedge clock) begin
       if(!hit && ICread) begin 
           IMread=1'b1;                 //in a miss, enable IMread
           IMaddress=ICaddress[9:4];    //in a miss, provide the relevant IMaddress
       end
       if(hit && ICread) begin          //in a hit
           ICbusywait=1'b0;             //disable ICbusywait
           IMread=1'b0;                 //disable IMread
           hit=1'b0;                    //disable hit
       end
    end
    
    
    always @(IMbusywait) begin
    
       if((IMbusywait==1'b0) && !hit && IMread) begin         
          ICbusywait=1'b1;
          temp=IMreaddata[127:0];              //get the read data of instruction memory 
          #1 {cache_mem[4*index+3],cache_mem[4*index+2],cache_mem[4*index+1],cache_mem[4*index]}=temp;    //write to cache with a latency of #1
          valid[index]=1'b1;                   //set the valid bit
          addressTag[index]=tag[2:0];          //update address tag
          #2 hit=1'b1;                         //change the value of hit signal after #2 
          ICreaddata=cache_mem[4*index+offset];     //provide the instruction for cpu
       end
    end                    
    
    integer i;
    always @(posedge reset) begin                      //at the positive edge of reset
    
        if(reset) begin
            for(i=0;i<8;i=i+1) begin
               valid[i]=0;                            //making valid bits to zero
               addressTag[i]=3'bxxx;                  //making address tag values to zero
            end
            for(i=0;i<32;i=i+1) begin
               cache_mem[i]=0;                        //reseting of cache memory
            end
            IMread=1'b0;                              //disabling IMread signal
        end
    end

endmodule
