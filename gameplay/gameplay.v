module gameplay(
	 input CLOCK_50,
	 input PS2_DAT,
	 input PS2_CLK,
	 input [9:0] SW,
	 
	 output [9:0] LEDR,
	 output VGA_CLK,  			//	VGA Clock
	 output VGA_HS,				//	VGA H_SYNC
	 output VGA_VS,				//	VGA V_SYNC
	 output VGA_BLANK_N,			//	VGA BLANK
	 output VGA_SYNC_N,			//	VGA SYNC
	 output [9:0]	VGA_R,   	//	VGA Red[9:0]
	 output [9:0]	VGA_G,	 	//	VGA Green[9:0]
	 output [9:0]	VGA_B   		//	VGA Blue[9:0]
	 );
	 
	 wire valid;
	 wire makeBreak;
	 wire [7:0] keyboard_out;
	 reg p1 = 0;
	 reg p2 = 0;
	 wire writeEn;
	 wire FSMplot;
	 wire [7:0] x;
	 wire [6:0] y;
	 wire color;
	 wire [5:0] datapath_scene;
	 wire resetn = SW[9];
	 
	 assign LEDR[5:0] = datapath_scene[5:0];
	 
	 assign writeEn = ((x == 8'b10011111) && (y == 7'b1110111)) ? 0 : FSMplot;
	 
	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(color),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "TRUE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "NEW_GAME.mif";
		
		
	 // Instansiate FSM
	 gameplay_FSM game0(CLOCK_50, SW[9], p1, p2, FSMplot, datapath_scene[5:0]);
	 
	 // Instansiate datapath
	 datapath d0(
	  	 .clk(CLOCK_50),
		 .resetn(resetn),
		 .scene(datapath_scene),
		 .run(writeEn),
		 .x_out(x),
		 .y_out(y),
		 .color_out(color)
		 );
	 
	 keyboard_press_driver keyboard(
		.clk(CLOCK_50), 
		.valid(valid), 
		.makeBreak(makeBreak),
		.outCode(keyboard_out),
		.PS2_DAT(PS2_DAT), // PS2 data line
		.PS2_CLK(PS2_CLK), // PS2 clock line
		.reset(~SW[9])
		);
	 
	 always@(negedge valid)
		begin
			if (keyboard_out == 8'b00010110)  		// If 1 is being pressed/released
				p2 <= makeBreak;
			else if (keyboard_out == 8'b01000101)	// If 0 is being pressed/released
				p1 <= makeBreak;
		end
	 
endmodule

module gameplay_FSM(
    input clk,
    input resetn,
	 input p1,
	 input p2,
	 
	 output reg writeEn,
    output reg [5:0] state_out
	 );
	 
	 wire [27:0] randVal;
	 wire enable;
	 wire animEnable;
	 wire randEnable;
	 reg run = 0;
	 reg runAnim = 0;
	 reg runRand = 0;
	 
	 oneSecond timer0(enable, clk, run);
	 animCounter aTimer(animEnable, clk, runAnim);
	 randomCounter rCount(randVal, clk, runRand);
	 randomTimer rTimer(randEnable, clk, runRand, randVal);

    reg [5:0] current_state, next_state; 
    
    localparam  S_NEW_GAME			= 6'd0,
					 S_NEW_GAME_WAIT  = 6'd1,
					 
					 S_READY_WAIT		= 6'd2,
					 S_READY				= 6'd3,
					 S_STEADY_WAIT		= 6'd4,
					 S_STEADY			= 6'd5,
					 S_SHOOT_WAIT		= 6'd6,
					 
					 S_P1_EARLY			= 6'd7,
					 S_P1_EARLY_2		= 6'd8,
					 S_P1_EARLY_3		= 6'd9,
					 S_P1_EARLY_4		= 6'd10,
					 S_P1_EARLY_5		= 6'd11,
					 S_P1_EARLY_6		= 6'd12,
					 S_P1_EARLY_7		= 6'd13,
					 S_P1_EARLY_8		= 6'd14,
					 
					 S_P2_EARLY			= 6'd15,
					 S_P2_EARLY_2		= 6'd16,
					 S_P2_EARLY_3		= 6'd17,
					 S_P2_EARLY_4		= 6'd18,
					 S_P2_EARLY_5		= 6'd19,
					 S_P2_EARLY_6		= 6'd20,
					 S_P2_EARLY_7		= 6'd21,
					 S_P2_EARLY_8		= 6'd22,
					 
					 S_SHOOT				= 6'd23,
					 
					 S_P1_WIN			= 6'd24,
					 S_P1_WIN_2			= 6'd25,
					 S_P1_WIN_3			= 6'd26,
					 S_P1_WIN_4			= 6'd27,
					 S_P1_WIN_5			= 6'd28,
					 S_P1_WIN_6			= 6'd29,
					 S_P1_WIN_7			= 6'd30,
					 S_P1_WIN_8			= 6'd31,
					 
					 S_P2_WIN			= 6'd32,
					 S_P2_WIN_2			= 6'd33,
					 S_P2_WIN_3			= 6'd34,
					 S_P2_WIN_4			= 6'd35,
					 S_P2_WIN_5			= 6'd36,
					 S_P2_WIN_6			= 6'd37,
					 S_P2_WIN_7			= 6'd38,
					 S_P2_WIN_8			= 6'd39;
    
    // Next state logic aka our state table
    always@(*)
    begin: state_table 
            case (current_state)
                
					 S_NEW_GAME: next_state = p1 || p2 ? S_NEW_GAME_WAIT : S_NEW_GAME;
					 
                S_NEW_GAME_WAIT: next_state = ~p1 && ~p2 ? S_READY_WAIT : S_NEW_GAME_WAIT;
					 
					 
					 S_READY_WAIT: begin
											if (p1 == 1'b1)
												next_state = S_P1_EARLY;
											else if (p2 == 1'b1)
												next_state = S_P2_EARLY;
											else if (enable == 1'b1)
												next_state = S_READY;
											else
												next_state = S_READY_WAIT;
										end
					 
					 S_READY: 		begin
											if (p1 == 1'b1)
												next_state = S_P1_EARLY;
											else if (p2 == 1'b1)
												next_state = S_P2_EARLY;
											else
												next_state = S_STEADY_WAIT;
										end
					 
					 S_STEADY_WAIT: begin
											if (p1 == 1'b1)
												next_state = S_P1_EARLY;
											else if (p2 == 1'b1)
												next_state = S_P2_EARLY;
											else if (enable == 1'b1)
												next_state = S_STEADY;
											else
												next_state = S_STEADY_WAIT;
										end
					 
					 S_STEADY: 		begin
											if (p1 == 1'b1)
												next_state = S_P1_EARLY;
											else if (p2 == 1'b1)
												next_state = S_P2_EARLY;
											else
												next_state = S_SHOOT_WAIT;
										end
					 
					 S_SHOOT_WAIT: begin
											if (p1 == 1'b1)
												next_state = S_P1_EARLY;
											else if (p2 == 1'b1)
												next_state = S_P2_EARLY;
											else if (randEnable == 1'b1)
												next_state = S_SHOOT;
											else
												next_state = S_SHOOT_WAIT;
										end
										
					 // P1_EARLY Animation States
					 
					 S_P1_EARLY: next_state = animEnable ? S_P1_EARLY_2 : S_P1_EARLY;
					 
					 S_P1_EARLY_2: next_state = animEnable ? S_P1_EARLY_3 : S_P1_EARLY_2;
					 
					 S_P1_EARLY_3: next_state = animEnable ? S_P1_EARLY_4 : S_P1_EARLY_3;
					 
					 S_P1_EARLY_4: next_state = animEnable ? S_P1_EARLY_5 : S_P1_EARLY_4;
					 
					 S_P1_EARLY_5: next_state = animEnable ? S_P1_EARLY_6 : S_P1_EARLY_5;
					 
					 S_P1_EARLY_6: next_state = animEnable ? S_P1_EARLY_7 : S_P1_EARLY_6;
					 
					 S_P1_EARLY_7: next_state = animEnable ? S_P1_EARLY_8 : S_P1_EARLY_7;
					 
					 S_P1_EARLY_8: next_state = enable ? S_NEW_GAME : S_P1_EARLY_8;
					 
					 
					 // P2_EARLY Animation States
					 
					 S_P2_EARLY: next_state = animEnable ? S_P2_EARLY_2 : S_P2_EARLY;
					 
					 S_P2_EARLY_2: next_state = animEnable ? S_P2_EARLY_3 : S_P2_EARLY_2;
					 
					 S_P2_EARLY_3: next_state = animEnable ? S_P2_EARLY_4 : S_P2_EARLY_3;
					 
					 S_P2_EARLY_4: next_state = animEnable ? S_P2_EARLY_5 : S_P2_EARLY_4;
					 
					 S_P2_EARLY_5: next_state = animEnable ? S_P2_EARLY_6 : S_P2_EARLY_5;
					 
					 S_P2_EARLY_6: next_state = animEnable ? S_P2_EARLY_7 : S_P2_EARLY_6;
					 
					 S_P2_EARLY_7: next_state = animEnable ? S_P2_EARLY_8 : S_P2_EARLY_7;
					 
					 S_P2_EARLY_8: next_state = enable ? S_NEW_GAME : S_P2_EARLY_8;
					 
					 S_SHOOT: 		begin
											if (p2 == 1'b1)
												next_state = S_P1_WIN;
											else if (p1 == 1'b1)
												next_state = S_P2_WIN;
											else
												next_state = S_SHOOT;
										end
										
					 // P1_WIN Animation States
					 
					 S_P1_WIN: next_state = animEnable ? S_P1_WIN_2 : S_P1_WIN;
					 
					 S_P1_WIN_2: next_state = animEnable ? S_P1_WIN_3 : S_P1_WIN_2;
					 
					 S_P1_WIN_3: next_state = animEnable ? S_P1_WIN_4 : S_P1_WIN_3;
					 
					 S_P1_WIN_4: next_state = animEnable ? S_P1_WIN_5 : S_P1_WIN_4;
					 
					 S_P1_WIN_5: next_state = animEnable ? S_P1_WIN_6 : S_P1_WIN_5;
					 
					 S_P1_WIN_6: next_state = animEnable ? S_P1_WIN_7 : S_P1_WIN_6;
					 
					 S_P1_WIN_7: next_state = animEnable ? S_P1_WIN_8 : S_P1_WIN_7;
					 
					 S_P1_WIN_8: next_state = enable ? S_NEW_GAME : S_P1_WIN_8;
					 
					 
					 // P2_WIN Animation States
					 
					 S_P2_WIN: next_state = animEnable ? S_P2_WIN_2 : S_P2_WIN;
					 
					 S_P2_WIN_2: next_state = animEnable ? S_P2_WIN_3 : S_P2_WIN_2;
					 
					 S_P2_WIN_3: next_state = animEnable ? S_P2_WIN_4 : S_P2_WIN_3;
					 
					 S_P2_WIN_4: next_state = animEnable ? S_P2_WIN_5 : S_P2_WIN_4;
					 
					 S_P2_WIN_5: next_state = animEnable ? S_P2_WIN_6 : S_P2_WIN_5;
					 
					 S_P2_WIN_6: next_state = animEnable ? S_P2_WIN_7 : S_P2_WIN_6;
					 
					 S_P2_WIN_7: next_state = animEnable ? S_P2_WIN_8 : S_P2_WIN_7;
					 
					 S_P2_WIN_8: next_state = enable ? S_NEW_GAME : S_P2_WIN_8;
					 
            default:     next_state = S_NEW_GAME;
        endcase
    end // state_table
   

    // Output logic aka all of our datapath control signals
    always @(*)
    begin: enable_signals
        // By default make all our signals 0
        state_out = 6'b000000;
		  run = 1'b0; 		// To restart timers used for the countdown
		  runAnim = 1'b0; // To restart timers used for animating scenes
		  runRand = 1'b0; // To restart the random counter/timer
		  writeEn = 1'b0;
		  
        case (current_state)
					 
				S_NEW_GAME: begin
					 writeEn = 1'b1;
					 state_out = 6'd0;
					 end
				
				S_NEW_GAME_WAIT: begin 	// NO DRAWING HERE
					 state_out = 6'd1;
					 end
					 
				S_READY_WAIT: begin
					 writeEn = 1'b1;
					 state_out = 6'd2;
					 run = 1'b1;
					 end
				
				S_READY: begin				// NO DRAWING HERE
					 state_out = 6'd3;
					 end
				
				S_STEADY_WAIT: begin
					 writeEn = 1'b1;
					 state_out = 6'd4;
					 run = 1'b1;
					 end
				
				S_STEADY: begin			// NO DRAWING HERE
					 state_out = 6'd5;
					 end
				
				S_SHOOT_WAIT: begin		// NO DRAWING HERE
					 state_out = 6'd6;
					 runRand = 1'b1;
					 end
					 
				// P1_EARLY Animation States	 
					 
				S_P1_EARLY: begin
					 writeEn = 1'b1;
					 state_out = 6'd7;
					 runAnim = 1'b1;
					 end
					 
				S_P1_EARLY_2: begin
					 writeEn = 1'b1;
					 state_out = S_P1_EARLY_2;
					 runAnim = 1'b1;
					 end
					 
				S_P1_EARLY_3: begin
					 writeEn = 1'b1;
					 state_out = S_P1_EARLY_3;
					 runAnim = 1'b1;
					 end
					 
				S_P1_EARLY_4: begin
					 writeEn = 1'b1;
					 state_out = S_P1_EARLY_4;
					 runAnim = 1'b1;
					 end
					 
				S_P1_EARLY_5: begin
					 writeEn = 1'b1;
					 state_out = S_P1_EARLY_5;
					 runAnim = 1'b1;
					 end
					 
				S_P1_EARLY_6: begin
					 writeEn = 1'b1;
					 state_out = S_P1_EARLY_6;
					 runAnim = 1'b1;
					 end
					 
				S_P1_EARLY_7: begin
					 writeEn = 1'b1;
					 state_out = S_P1_EARLY_7;
					 runAnim = 1'b1;
					 end
					 
				S_P1_EARLY_8: begin
					 writeEn = 1'b1;
					 state_out = S_P1_EARLY_8;
					 run = 1'b1;
					 end
					 
				// P2_EARLY Animation States
				
				S_P2_EARLY: begin
					 writeEn = 1'b1;
					 state_out = 6'd15;
					 runAnim = 1'b1;
					 end
					 
				S_P2_EARLY_2: begin
					 writeEn = 1'b1;
					 state_out = S_P2_EARLY_2;
					 runAnim = 1'b1;
					 end
					 
				S_P2_EARLY_3: begin
					 writeEn = 1'b1;
					 state_out = S_P2_EARLY_3;
					 runAnim = 1'b1;
					 end
					 
				S_P2_EARLY_4: begin
					 writeEn = 1'b1;
					 state_out = S_P2_EARLY_4;
					 runAnim = 1'b1;
					 end
					 
				S_P2_EARLY_5: begin
					 writeEn = 1'b1;
					 state_out = S_P2_EARLY_5;
					 runAnim = 1'b1;
					 end
					 
				S_P2_EARLY_6: begin
					 writeEn = 1'b1;
					 state_out = S_P2_EARLY_6;
					 runAnim = 1'b1;
					 end
					 
				S_P2_EARLY_7: begin
					 writeEn = 1'b1;
					 state_out = S_P2_EARLY_7;
					 runAnim = 1'b1;
					 end
					 
				S_P2_EARLY_8: begin
					 writeEn = 1'b1;
					 state_out = S_P2_EARLY_8;
					 run = 1'b1;
					 end
					 
				////////////////////////////////
					 
				S_SHOOT: begin
					 writeEn = 1'b1;
					 state_out = 6'd23;
					 end
					 
				// P1_WIN Animation States
				
				S_P1_WIN: begin
					 writeEn = 1'b1;
					 state_out = 6'd24;
					 runAnim = 1'b1;
					 end
					 
				S_P1_WIN_2: begin
					 writeEn = 1'b1;
					 state_out = S_P1_WIN_2;
					 runAnim = 1'b1;
					 end
					 
				S_P1_WIN_3: begin
					 writeEn = 1'b1;
					 state_out = S_P1_WIN_3;
					 runAnim = 1'b1;
					 end
					 
				S_P1_WIN_4: begin
					 writeEn = 1'b1;
					 state_out = S_P1_WIN_4;
					 runAnim = 1'b1;
					 end
					 
				S_P1_WIN_5: begin
					 writeEn = 1'b1;
					 state_out = S_P1_WIN_5;
					 runAnim = 1'b1;
					 end
					 
				S_P1_WIN_6: begin
					 writeEn = 1'b1;
					 state_out = S_P1_WIN_6;
					 runAnim = 1'b1;
					 end
					 
				S_P1_WIN_7: begin
					 writeEn = 1'b1;
					 state_out = S_P1_WIN_7;
					 runAnim = 1'b1;
					 end
					 
				S_P1_WIN_8: begin
					 writeEn = 1'b1;
					 state_out = S_P1_WIN_8;
					 run = 1'b1;
					 end
					 
				// P2_WIN Animation States
				
				S_P2_WIN: begin
					 writeEn = 1'b1;
					 state_out = 6'd32;
					 runAnim = 1'b1;
					 end
					 
				S_P2_WIN_2: begin
					 writeEn = 1'b1;
					 state_out = S_P2_WIN_2;
					 runAnim = 1'b1;
					 end
					 
				S_P2_WIN_3: begin
					 writeEn = 1'b1;
					 state_out = S_P2_WIN_3;
					 runAnim = 1'b1;
					 end
					 
				S_P2_WIN_4: begin
					 writeEn = 1'b1;
					 state_out = S_P2_WIN_4;
					 runAnim = 1'b1;
					 end
					 
				S_P2_WIN_5: begin
					 writeEn = 1'b1;
					 state_out = S_P2_WIN_5;
					 runAnim = 1'b1;
					 end
					 
				S_P2_WIN_6: begin
					 writeEn = 1'b1;
					 state_out = S_P2_WIN_6;
					 runAnim = 1'b1;
					 end
					 
				S_P2_WIN_7: begin
					 writeEn = 1'b1;
					 state_out = S_P2_WIN_7;
					 runAnim = 1'b1;
					 end
					 
				S_P2_WIN_8: begin
					 writeEn = 1'b1;
					 state_out = S_P2_WIN_8;
					 run = 1'b1;
					 end
        endcase
    end // enable_signals
   
    // current_state registers
    always@(posedge clk)
    begin: state_FFs
        if(!resetn)
            current_state <= S_NEW_GAME;
        else
            current_state <= next_state;
    end // state_FFS
endmodule

////////////////////////////////////////////////////////////////////

// Outputs an enable signal once a second
module oneSecond(enable, clock, reset);
	output enable;
	input clock;
	input reset;
	
	reg [25:0] q = 26'b0;
	
	always @(posedge clock)
		begin
			if (reset == 0)
				q <= 26'b10111110101111000010000000;
			else if (q == 26'b000000000000000000000000)
				q <= 26'b10111110101111000010000000;  		//For 1/sec
			else
				q <= q - 26'b000000000000000000000001;
		end
		
	assign enable = (q == 26'b0) ? 1 : 0;
endmodule

// Outputs an enable signal 4(?) times a second
module animCounter(enable, clock, reset);
	output enable;
	input clock;
	input reset;
	
	reg [25:0] q = 26'b0;
	
	always @(posedge clock)
		begin
			if (reset == 0)
				q <= 26'b00101111101011110000100000;
			else if (q == 26'b000000000000000000000000)
				q <= 26'b00101111101011110000100000;  		//For 4 enables/second
			else
				q <= q - 26'b000000000000000000000001;
		end
		
	assign enable = (q == 26'b0) ? 1 : 0;
endmodule

// Outputs a random number that will provide a countdown time
// between 1 and 3 seconds
module randomCounter(randVal, clock, reset);
	output reg [27:0] randVal;
	input clock;
	input reset;
	
	always @(posedge clock)
		begin
			if (reset == 0)
				randVal <= 28'b1000111100001101000110000000;
			else if (randVal < 28'b0010111110101111000010000000)	// For 1 sec
				randVal <= 28'b1111111111111111111111111111;  		// For 5 sec
			else
				randVal <= randVal - 28'b0001010011000100101101000000; // Minus large amount
		end
endmodule

// Outputs an enable signal once every 1-3 seconds (Counts UP from 0)
module randomTimer(enable, clock, reset, randVal);
	output enable;
	input clock;
	input reset;
	input [27:0] randVal;
	
	reg [27:0] q = 0;
	
	always @(posedge clock)
		begin
			if (reset == 0)
				q <= 0;
			else if (q == randVal)
				q <= 28'b00000000000000000000000000;  									// For 1-3 sec
			else
				q <= q + 28'b00000000000000000000000001;
		end
				
	assign enable = (q == randVal) ? 1 : 0;
endmodule

////////////////////////////////////////////////////////////////////

module datapath(
    input clk,
    input resetn,
	 input [5:0] scene,
	 input run,
	 
	 output [7:0] x_out,
	 output [6:0] y_out,
	 output reg color_out
    );
	 
	 wire [14:0] Current_Address;
	 memoryCounter count0(Current_Address[14:0], x_out[7:0], y_out[6:0], clk, run);
	 
	 wire ram0out;
	 wire ram2out;
	 wire ram4out;
	 wire ram7out;
	 wire ram8out;
	 wire ram9out;
	 wire ram10out;
	 wire ram11out;
	 wire ram12out;
	 wire ram13out;
	 wire ram14out;
	 wire ram15out;
	 wire ram16out;
	 wire ram17out;
	 wire ram18out;
	 wire ram19out;
	 wire ram20out;
	 wire ram21out;
	 wire ram22out;
	 wire ram23out;
	 wire ram24out;
	 wire ram25out;
	 wire ram26out;
	 wire ram27out;
	 wire ram28out;
	 wire ram29out;
	 wire ram30out;
	 wire ram31out;
	 wire ram32out;
	 wire ram33out;
	 wire ram34out;
	 wire ram35out;
	 wire ram36out;
	 wire ram37out;
	 wire ram38out;
	 wire ram39out;
    
    // Registers color with respective input logic
    always @(posedge clk) begin												// color = 0 for black, 1 for white
        if (!resetn || scene == 6'd0) color_out <= ram0out;
		  
        else if (scene == 6'd2) color_out <= ram2out;
		  
		  else if (scene == 6'd4) color_out <= ram4out;
		  
		  else if (scene == 6'd7) color_out <= ram7out;
		  
		  else if (scene == 6'd8) color_out <= ram8out;
		  
		  else if (scene == 6'd9) color_out <= ram9out;
		  
		  else if (scene == 6'd10) color_out <= ram10out;
		  
		  else if (scene == 6'd11) color_out <= ram11out;
		  
		  else if (scene == 6'd12) color_out <= ram12out;
		  
		  else if (scene == 6'd13) color_out <= ram13out;
		  
		  else if (scene == 6'd14) color_out <= ram14out;
		  
		  else if (scene == 6'd15) color_out <= ram15out;
		  
		  else if (scene == 6'd16) color_out <= ram16out;
		  
		  else if (scene == 6'd17) color_out <= ram17out;
		  
		  else if (scene == 6'd18) color_out <= ram18out;
		  
		  else if (scene == 6'd19) color_out <= ram19out;
		  
		  else if (scene == 6'd20) color_out <= ram20out;
		  
		  else if (scene == 6'd21) color_out <= ram21out;
		  
		  else if (scene == 6'd22) color_out <= ram22out;
		  
		  else if (scene == 6'd23) color_out <= ram23out;
		  
		  else if (scene == 6'd24) color_out <= ram24out;
		  
		  else if (scene == 6'd25) color_out <= ram25out;
		  
		  else if (scene == 6'd26) color_out <= ram26out;
		  
		  else if (scene == 6'd27) color_out <= ram27out;
		  
		  else if (scene == 6'd28) color_out <= ram28out;
		  
		  else if (scene == 6'd29) color_out <= ram29out;
		  
		  else if (scene == 6'd30) color_out <= ram30out;
		  
		  else if (scene == 6'd31) color_out <= ram31out;
		  
		  else if (scene == 6'd32) color_out <= ram32out;
		  
		  else if (scene == 6'd33) color_out <= ram33out;
		  
		  else if (scene == 6'd34) color_out <= ram34out;
		  
		  else if (scene == 6'd35) color_out <= ram35out;
		  
		  else if (scene == 6'd36) color_out <= ram36out;
		  
		  else if (scene == 6'd37) color_out <= ram37out;
		  
		  else if (scene == 6'd38) color_out <= ram38out;
		  
		  else if (scene == 6'd39) color_out <= ram39out;
    end
	 
	 ramNEW_GAME ram0(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram0out));
		
	 ramREADY ram2(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram2out));
		
	 ramSTEADY ram4(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram4out));
		
	 ramSHOOT ram23(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram23out));
		
	 ////////////////////////////////////
	
	 ramP1_EARLY ram7(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram7out));
		
	 ramP1_EARLY_2 ram8(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram8out));
		
	 ramP1_EARLY_3 ram9(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram9out));
		
	 ramP1_EARLY_4 ram10(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram10out));
		
	 ramP1_EARLY_5 ram11(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram11out));
		
	 ramP1_EARLY_6 ram12(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram12out));
		
	 ramP1_EARLY_7 ram13(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram13out));
		
	 ramP1_EARLY_8 ram14(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram14out));
		
	 ////////////////////////////////////
		
	 ramP2_EARLY ram15(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram15out));
		
	 ramP2_EARLY_2 ram16(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram16out));
		
	 ramP2_EARLY_3 ram17(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram17out));
		
	 ramP2_EARLY_4 ram18(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram18out));
		
	 ramP2_EARLY_5 ram19(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram19out));
		
	 ramP2_EARLY_6 ram20(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram20out));
		
	 ramP2_EARLY_7 ram21(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram21out));
		
	 ramP2_EARLY_8 ram22(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram22out));
		
	 ////////////////////////////////////
		
	 ramP1_WIN ram24(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram24out));
		
	 ramP1_WIN_2 ram25(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram25out));
		
	 ramP1_WIN_3 ram26(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram26out));
		
	 ramP1_WIN_4 ram27(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram27out));
		
	 ramP1_WIN_5 ram28(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram28out));
		
	 ramP1_WIN_6 ram29(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram29out));
		
	 ramP1_WIN_7 ram30(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram30out));
		
	 ramP1_WIN_8 ram31(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram31out));
		
	 ////////////////////////////////////
		
	 ramP2_WIN ram32(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram32out));
		
	 ramP2_WIN_2 ram33(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram33out));
		
	 ramP2_WIN_3 ram34(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram34out));
		
	 ramP2_WIN_4 ram35(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram35out));
		
	 ramP2_WIN_5 ram36(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram36out));
		
	 ramP2_WIN_6 ram37(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram37out));
		
	 ramP2_WIN_7 ram38(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram38out));
		
	 ramP2_WIN_8 ram39(
		.address(Current_Address),
		.clock(clk),
		.data(0),
		.wren(0),
		.q(ram39out));
	
endmodule

////////////////////////////////////////////////////////////////////

// Must manually reset before each use

// Counts through each memory address & x,y coordinates

module memoryCounter(q, x_count, y_count, clock, resetn);
	output reg [14:0] q = 15'b0;
	output reg [7:0] x_count = 8'b0;
	output reg [6:0] y_count = 7'b0;
	input clock;
	input resetn;
	
	always @(posedge clock)
		begin
			if (resetn == 0) begin
				q <= 15'b0;
				x_count <= 8'b0;
				y_count <= 7'b0;
			end
			else begin

				if (x_count < 8'b10011111)					// Counts through x coords
					x_count <= x_count + 8'b00000001;
					
				else if ((x_count == 8'b10011111) && (y_count < 7'b1110111)) begin
					x_count <= 8'b00000000;
					y_count <= y_count + 7'b0000001;		// Increments y when a row has been scanned
				end
				
				if (q < 15'b100101011111110)				// Counts through memory addresses
					q <= q + 15'b000000000000001;
			end
		end
endmodule