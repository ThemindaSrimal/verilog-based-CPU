
module test_bench;                   //testbench for testing
    wire [31:0] PC;                  //32 bit PC as a wire
    reg [31:0] INSTRUCTION;          //32 bit INSTRUCTION as a register
    reg CLK;                         //CLK as a register
    reg RESET;                       //RESET as a register
    
    reg [7:0] num[1023:0];           //array to hold instructions,1024 bytes
    
    cpu mycpu(PC,INSTRUCTION,CLK,RESET);
    
    initial begin
      
       //needed files for ploting the waveform 
       $dumpfile("dump.vcd");                     
	   $dumpvars(0,mycpu); 
	   
       
       //give values to registers with time 
       CLK=1'b0;
       RESET=1'b1;
       
       //instructions are stored here
       {num[0],num[1],num[2],num[3]}=32'b00001000000000000000000000010010;       //loadi 0 0x12
       {num[4],num[5],num[6],num[7]}=32'b00001000000000010000000000010101;       //loadi 1 0x15
       {num[8],num[9],num[10],num[11]}=32'b00000001000000100000000000000001;     //add 2 0 1
       {num[12],num[13],num[14],num[15]}=32'b00000011000000110000001000000000;   //or 3 2 0
       {num[16],num[17],num[18],num[19]}=32'b00001001000001000000001100000001;   //sub 4 3 1
       {num[20],num[21],num[22],num[23]}=32'b00000000000001010000000000000010;   //mov 5 2
       {num[24],num[25],num[26],num[27]}=32'b00000010000001100000000100000100;   //and 6 1 4
       {num[28],num[29],num[30],num[31]}=32'b00000100000000100000000000000000;   //j 0x02
       {num[32],num[33],num[34],num[35]}=32'b00001000000001110000000000100001;   //loadi 7 0x21
       {num[36],num[37],num[38],num[39]}=32'b00000101111101110000001000000011;   //beq 0xF7 2 3
       {num[40],num[41],num[42],num[43]}=32'b00000000000001010000000000000001;   //mov 5 1
       {num[44],num[45],num[46],num[47]}=32'b00000101111111100000000100000011;   //beq 0xFE 1 3
       {num[48],num[49],num[50],num[51]}=32'b00001000000001000000000000011000;   //loadi 4 0x18
       {num[52],num[53],num[54],num[55]}=32'b00000000000001100000000000000100;   //mov 6 4
       {num[56],num[57],num[58],num[59]}=32'b00000101000000010000011000000100;   //beq 0x01 6 4
       {num[60],num[61],num[62],num[63]}=32'b00000011000000110000010100000111;   //or 3 5 7
       {num[64],num[65],num[66],num[67]}=32'b00001000000000100000000000110111;   //loadi 2 0x37
       {num[68],num[69],num[70],num[71]}=32'b01000110000001000000011000000010;   //sll 4 6 0x02
       {num[72],num[73],num[74],num[75]}=32'b01100110000001000000000100000010;   //srl 4 1 0x02
       {num[76],num[77],num[78],num[79]}=32'b10000110000001000000011000000010;   //sra 4 6 0x02
       {num[80],num[81],num[82],num[83]}=32'b10100110000001000000000100000010;   //ror 5 1 0x02
       {num[84],num[85],num[86],num[87]}=32'b11000110111100110000000100000010;   //bne 0xF3 1 2
      
       #3;
       RESET=1'b0;
       
       
       #240;
       $finish;      
    end
    always @(PC) begin
        #2 INSTRUCTION={num[PC],num[PC+1],num[PC+2],num[PC+3]};             //set the INSTRUCTION
        #8;
    end 
     
    always 
        #5 CLK=~CLK;           //inverting the CLK in every 5 time units
endmodule
 
 
 //opcodes     loadi-00001000     mov-00000000     add-00000001     sub-00001001     and-00000010      or-00000011     j-00000100    beq-00000101
 //mult-00100110    sll-01000110    srl-01100110     sra-10000110     ror-10100110    bne-11000110
 //OUT_DATA-output of the twosComp
 //CHOOSED1-output of the mux which chooses between add and sub
 //CHOOSED2-output of the mux which chooses whether there is an immediate value to get
 
 
 
//cpu module
module cpu(PC,INSTRUCTION,CLK,RESET);
     output [31:0] PC;                    //output port for 32 bit PC
     input [31:0] INSTRUCTION;            //input port for 32 bit INSTRUCTION
     input CLK;                           //input port for CLK
     input RESET;                         //input port for RESET
     
     wire [7:0] RESULT;                   //8 bit RESULT as a wire
     wire [2:0] SELECT;                   //3 bit SELECT as a wire
     wire [2:0] EXTRA;                    //3 bit EXTRA as a wire
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
     wire bneIndicator;                   //bneIndicator as a wire
     
     //instantiation of modules with in top-level cpu module 
     
     //control_unit module instantiation
     control_unit myctrl(INSTRUCTION,SELECT,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,IMMEDIATE,AddSubChoose,ImmediateChoose,WRITE,OFFSET,JumpIndicator,RESET,BranchIndicator,EXTRA,bneIndicator);
     
     //reg_file module instantiation
     reg_file myregfile(RESULT,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,WRITE,CLK,RESET,JumpIndicator,BranchIndicator,bneIndicator); 
     
     //twosComp module instantiation
     twosComp mytwosComp(OUT2,OUT_DATA); 
     
     //instantiation of mux module which chooses between add and sub
     mux addsubchoose(OUT2,OUT_DATA,AddSubChoose,CHOOSED1);
     
     //instantiation of mux module which chooses whether there is an immediate value to get
     mux immediatechoose(IMMEDIATE,CHOOSED1,ImmediateChoose,CHOOSED2);   
     
     //alu module instantiation
     alu myalu(OUT1,CHOOSED2,SELECT,RESULT,ZERO,EXTRA);
     
     //programCounter module instantiation
     programCounter myPC(CLK,initialPC,RESET,finalPC,PC);
     
     //instantiation adder which adds the OFFSET value to initialPC
     adder_offset_PC myAdderOffset(initialPC,OFFSET,offsetPC,SELECT,ImmediateChoose,JumpIndicator,EXTRA);
     
     //instantiation of mux module which chooses between initialPC and jumpPC
     muxZeroJumpChoose myzeroChoose(initialPC,offsetPC,finalPC,BranchIndicator,JumpIndicator,ZERO,bneIndicator);
     
endmodule



//control_unit module
module control_unit(INSTRUCTION,SELECT,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,IMMEDIATE,AddSubChoose,ImmediateChoose,WRITE,OFFSET,JumpIndicator,RESET,BranchIndicator,EXTRA,bneIndicator);
    input [31:0] INSTRUCTION;                           //input port for 32 bit INSTRUCTION
    input RESET;                                        //input port for RESET
    output [2:0] SELECT;                                //output port for 3 bit SELECT
    output [2:0] EXTRA;                                 //output port for 3 bit EXTRA which is used for extra functions in lab 5 bonus part
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
    output bneIndicator;                                 //output port for bneIndicator
    
    reg [2:0] SELECT;                                   //3 bit SELECT as a register
    reg [2:0] EXTRA;                                    //3 bit EXTRA as a register
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
    reg bneIndicator;                                   //bneIndicator as a register
    reg [31:0] temp;                                    //32 bit temp register for temporary purposes
    
    always @(INSTRUCTION)                               //instruction decoding using INSTRUCTION
        begin                                         
           INADDRESS=INSTRUCTION[18:16];                //get the INADDRESS from the INSTRUCTION
           OUT1ADDRESS=INSTRUCTION[10:8];               //get the OU1ADDRESS from the INSTRUCTION
           OUT2ADDRESS=INSTRUCTION[2:0];                //get the OUT2ADDRESS from the INSTRUCTIONs
           IMMEDIATE=INSTRUCTION[7:0];                  //get the IMMEDIATE from the INSTRUCTION
           temp=INSTRUCTION[31:0];                      //get the temp value
           OFFSET={{22{temp[23]}},temp[23:16],2'b00};   //setting the OFFSET by sign extending and shifing left by two
           
           #1;                                          //decode delay is added here
           
           SELECT=INSTRUCTION[26:24];                   //get the SELECT value from INSTRUCTION   
           EXTRA=INSTRUCTION[31:29];                    //get the EXTRA value from INSTRUCTION
           
           if(INSTRUCTION[31:24]==8'b00001000 | INSTRUCTION[31:24]==8'b01000110 | INSTRUCTION[31:24]==8'b01100110 | INSTRUCTION[31:24]==8'b10000110 | INSTRUCTION[31:24]==8'b10100110) begin    //check the opcode value
              ImmediateChoose=1'b0;                     //set ImmediateChoose to low
           end else begin
              ImmediateChoose=1'b1;                     //set ImmediateChoose to high
           end
           
           
           if(INSTRUCTION[31:24]==8'b00001001) begin    //check the opcode value
              AddSubChoose=1'b1;                        //set the AddSubChoose to high
           end else begin 
              AddSubChoose=1'b0;                        //set the AddSubChoose to low
           end
           
           
           if(INSTRUCTION[31:24]==8'b00000100) begin     //check the opcode value
              JumpIndicator=1'b1;                        //set the JumpIndicator to high
           end else begin 
              JumpIndicator=1'b0;                        //set the JumpIndicator to low
           end
           
           
           if(INSTRUCTION[31:24]==8'b00000101) begin     //check the opcode value
              BranchIndicator=1'b1;                      //set the BranchIndicator to high
           end else begin
              BranchIndicator=1'b0;                      //set the BranchIndicator to low
           end
           
           
           if(INSTRUCTION[31:24]==8'b11000110) begin     //check the opcode value
              bneIndicator=1'b1;                         //set the bneIndicator to high
           end else begin
              bneIndicator=1'b0;                         //set the bneIndicator to low
           end
           
           
            //check the opcode value
           if((INSTRUCTION[26:24]==3'b000) |  (INSTRUCTION[26:24]==3'b001) | (INSTRUCTION[26:24]==3'b010) | (INSTRUCTION[26:24]==3'b011) | (INSTRUCTION[26:24]==3'b110)) begin  
               WRITE=1'b1;                             //enable the WRITE
           end else begin
               WRITE=1'b0;                             //disable the WRITE
           end
    end
        
    
    always @(RESET) begin                      //sensitive to RESET value
         if(RESET) begin                       //if RESET is enabled
            JumpIndicator=1'b0;                //make JumpIndicator as 0
             OFFSET=32'd0;                     //make OFFSET as 32 bit zero
         end
    end
    
endmodule



//alu module
module alu(DATA1,DATA2,SELECT,RESULT,ZERO,EXTRA);  //module instantiation
    input [7:0] DATA1;                       //8 bit input port for DATA1
    input [7:0] DATA2;                       //8 bit input port for DATA2
    input [2:0] SELECT;                      //3 bit input port for SELECT
    input [2:0] EXTRA;                       //3 bit input port for EXTRA
    output [7:0] RESULT;                     //8 bit output port for RESULT
    reg [7:0] RESULT;                        //declare RESULT as a register
    output ZERO;                             //output port for ZERO
    wire ZERO;                               //ZERO as a wire
    integer i;                               //integer variable 
    reg [7:0] tempData1;                     //8 bit temporary register
    reg [7:0] tempData2;                     //8 bit temporary register
    reg [7:0] tempResult;                    //8 bit temporary register

    always @ (DATA1,DATA2,SELECT,EXTRA) begin      //executing conditions
       i=0;                                  //give an initial value to i
       tempData1=DATA1[7:0];                 //give an initial value to tempData1
       tempData2=DATA2[7:0];                 //give an initial value to tempData2
       case(SELECT)                          //use of case structure for SELECT 
         3'b000: #1 RESULT=DATA2;            //FORWARD function  
         3'b001: #2 RESULT=DATA1+DATA2;      //ADD function 
         3'b010: #1 RESULT=DATA1&DATA2;      //AND function
         3'b011: #1 RESULT=DATA1|DATA2;      //OR function
         3'b100: #1 RESULT=0;                //j function
         3'b101: #1 RESULT=0;                //beq function
         3'b110:  case(EXTRA)                 //for extra functions in lab 5 bonus part 
                     //3'b001:#2 RESULT=DATA1 * DATA2;                    //mult fucntion was not implemented
                     3'b010:#2 begin                                      //Logical shift left
                                   tempData1=DATA1[7:0];                  //vlaue for tempData1
                                   tempData2=DATA2[7:0];                  //value for tempData2
                                   for(i=0;i<tempData2;i=i+1) begin
                                      tempResult={tempData1[6:0],1'b0};   //get the last 7 bits of tempData1 to start and add a 0 to end
                                      tempData1=tempResult[7:0];
                                   end
                                   RESULT=tempResult[7:0];                //RESULT of function
                               end
                     3'b011:#2 begin                                      //Logical shift right
                                   tempData1=DATA1[7:0];                  //vlaue for tempData1
                                   tempData2=DATA2[7:0];                  //vlaue for tempData2
                                   for(i=0;i<tempData2;i=i+1) begin
                                      tempResult={1'b0,tempData1[7:1]};   //get the 0 to start and add the last 7 bits of tempData1 to end
                                      tempData1=tempResult[7:0];
                                   end
                                   RESULT=tempResult[7:0];                //RESULT of function
                               end
                     3'b100:#2 begin                                      //arithmatic shift right
                                   tempData1=DATA1[7:0];                  //vlaue for tempData1
                                   tempData2=DATA2[7:0];                  //vlaue for tempData1
                                   for(i=0;i<tempData2;i=i+1) begin
                                      tempResult={tempData1[7],tempData1[7:1]};     //first bit is same and first 7 bits of tempData is taken
                                      tempData1=tempResult[7:0];
                                   end
                                   RESULT=tempResult[7:0];
                               end
                     3'b101:#2 begin                                      //rotate right
                                   tempData1=DATA1[7:0];                  //vlaue for tempData1 
                                   tempData2=DATA2[7:0];                  //vlaue for tempData2
                                   for(i=0;i<tempData2;i=i+1) begin
                                      tempResult={tempData1[0],tempData1[7:1]};   //get the last bit of tempData1 and add it to the start
                                      tempData1=tempResult[7:0];
                                   end
                                   RESULT=tempResult[7:0];                //RESULT of function
                               end
                     3'b110:#1 RESULT=0;                                  //bne
                     default: #1 RESULT=0;
                  endcase
         default: #1 RESULT=0;
       endcase
    end
    
    assign #2 ZERO=((DATA1-DATA2)==0)? 1'b1:1'b0;   //check the value of (DATA1-DATA2) and set the value of ZERO according to that
    
endmodule                                   //end module



//reg_file module
module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,WRITE,CLK,RESET,JumpIndicator,BranchIndicator,bneIndicator);     //reg_file module instantiation
     input [7:0] IN;                    //8 bit input port for IN
     output [7:0] OUT1;                 //8 bit output port for OUT1
     output [7:0] OUT2;                 //8 bit output port for OUT2	
     input [2:0] INADDRESS;             //3 bit input port for INADDRESS
     input [2:0] OUT1ADDRESS;           //3 bit input port for OUT1ADDRESS
     input [2:0] OUT2ADDRESS;           //3 bit input port for OUT1ADDRESS
     input WRITE;                       //input port for WRITE
     input CLK;                         //input port for CLK
     input RESET;                       //input port for RESET
     input BranchIndicator;             //input port for BranchIndicator
     input JumpIndicator;               //input port for JumpIndicator
     input bneIndicator;                //input port for bneIndicator
     reg [7:0] regNum[7:0];             //represent registers as an array of words
     wire [7:0] OUT1;                   //OUT1 as a wire
     wire [7:0] OUT2;                   //OUT2 as a wire
        
     integer i;                         //declare an integer variable
     always @(posedge CLK) begin        //at the positive edge of the CLK
       if ((((~RESET) & WRITE)&((~JumpIndicator) & (~BranchIndicator))) & (~bneIndicator)) #2 begin   //check whether WRITE is enabled and RESET,JumpIndicator,BranchIndicator,bneIndicator are disabled
	     regNum[INADDRESS] = IN;        //write data present on IN port to input register specified by the INADDRESS
	   end
	   //following line was added just to see the values in registers 
	   $display(" reg0:%h reg1:%h reg2:%h reg3:%h reg4:%h reg5:%h reg6:%h reg7:%h ",regNum[0],regNum[1],regNum[2],regNum[3],regNum[4],regNum[5],regNum[6],regNum[7]);
     end
     
     assign #2 OUT1=regNum[OUT1ADDRESS];    //register identified by OUT1ADDRESS is read and value is loaded onto OUT1
     assign #2 OUT2=regNum[OUT2ADDRESS];    //register identified by OUT2ADDRESS is read and value is loaded onto OUT2 
     
     always @(RESET) begin              //at the RESET
        if (RESET) #2 begin             //check whether RESET is enabled
        for(i=0;i<8;i=i+1) begin
           regNum[i]<=8'b00000000;      //make the values of registers as zeros
        end
        end
     end 
endmodule                               //end module



//twosComp module
module twosComp(IN_DATA,OUT_DATA);        //twosComp module instantiation
    input [7:0] IN_DATA;                  //8 bit input port
	output [7:0] OUT_DATA;                //8 bit output port
	reg [7:0] OUT_DATA;                   //OUT as a register
	
	always @(IN_DATA) begin
      OUT_DATA=~IN_DATA+1;                //get the two's complement value of IN
    end
endmodule                                 //end module



//programCounter module
module programCounter(CLK,initialPC,RESET,finalPC,PC);
     input CLK;                           //input port for CLK
     input RESET;                         //input port for PC
     input [31:0] finalPC;                //32 bit input port for PC
     output [31:0] initialPC;             //output port for PC
     reg [31:0] initialPC;                //PC as a register
     output [31:0] PC;
     reg [31:0] PC;
     
     
     always @(posedge CLK)  begin         //at the positive edge of the CLK
        if(~RESET) #1 begin               //delay of 3 time units is added here, PC update delay-#1,adder delay-#2
            PC=finalPC[31:0];             //write to PC register
            #2;                           //latency of adder which increment the value of PC
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
module adder_offset_PC(initialPC,OFFSET,offsetPC,SELECT,ImmediateChoose,JumpIndicator,EXTRA);
     input [31:0] initialPC;            //32 bit input port for initialPC
     input [31:0] OFFSET;               //32 bit input port for OFFSET
     output [31:0] offsetPC;            //32 bit output port for offsetPC
     reg [31:0] offsetPC;               //32 bit offsetPC as a register
     input [2:0] SELECT;                //3 bit input port for SELECT
     input ImmediateChoose;             //ImmediateChoose as a input
     input JumpIndicator;               //JumpIndicator as a input
     input [2:0] EXTRA;                 //3 bit input port for EXTRA
     always @(SELECT,EXTRA)  begin 
        if(JumpIndicator | (~ImmediateChoose)) begin
             #2 offsetPC=initialPC+OFFSET;   //add OFFSET to initialPC with a latency of 2 time units
        end else #1 begin                    //extra 1 time unit delay, becaue it should wait 1 time unit after decoding to start parallel with alu in functions except jump and loadi
             #2 offsetPC=initialPC+OFFSET;   //add OFFSET to initialPC with a latency of 2 time units
        end
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



module muxZeroJumpChoose(IN0,IN1,OUT,BranchIndicator,JumpIndicator,ZERO,bneIndicator);   //module instantiation
     input [31:0] IN0;               //8 bit input port
     input [31:0] IN1;               //8 bit input port
     input SEL;                      //input port SEL
     input BranchIndicator;          //input port for BranchIndicator
     input JumpIndicator;            //input port for JumpIndicator
     input bneIndicator;             //input port for bneIndicator
     input ZERO;                     //input port for ZERO
     output [31:0] OUT;              //8 bit output port
     reg [31:0] OUT;                 //OUT as a register
     reg temp;                       //temp as a register
     
     always @(IN0,IN1)      
     begin
        temp=((JumpIndicator | (BranchIndicator & ZERO)) | (bneIndicator & (~ZERO)));     //set a condition for mux  
        if (temp) begin             //check the value of temp 
           OUT=IN1;                 //make OUT as IN0 
        end else begin
           OUT=IN0;                 //make OUT as IN1
        end    
     end
endmodule                           //end module
