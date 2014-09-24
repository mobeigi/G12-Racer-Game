/*	 ; COMP2121 Project - G12 Racer
 *
 *	 ; Documentation: See user manual for game instructions and documentation.
 *	 ; Version: 1.2
 *
 *   ; Connections:
 *   ; LCD D0-D7 -> PC0-PC7
 *   ; LCD BE-RS -> PB0-PB3
 *   ; KEYPAD R0-R3 -> PA0-PA3
 *   ; KEYPAD C0-C3 -> PA4-PA7
 *	 ; LED0 - LED3 -> PF0-PF3 
 *	 ; LED4 - LED9 -> PE2-PE7 
 *	 ; PB0 -> PD0
 *   ; PB1 -> PD1
 *	 ; MOT -> PB4
 *    
 *	 ; Project Group:
 *   ; Mohammad Ghasembeigi (z3464208)
 *	 ; Phillip Khorn (z3463638)
 *	 ; Pearlie Zhang (z3460347)
 *	 ; Savinka Wijeyeratne (z3433816)
 *
 */

; Includes
.include "m64def.inc"

;Definitions
.def data_lo =r16	; must be register pair, used to store paired temporary data
.def data_hi =r17
.def temp = r18		; general temp register

.def level = r19	; stores level
.def lives = r28	; stores lives

.def score_lo = r20 ; stores game score
.def score_hi = r21

.def car_top = r22	; stores care position
.def car_bot = r23

.def obs_top = r24	; stores obstacle position
.def obs_bot = r25

.def powerup = r26  ; stores powerup position
.def shield = r27	; stores shield powerup position

.def mask = r29		; keypad masks
.def col = r30
.def row = r31

; Setting up the interrupt vector
jmp RESET
.org INT0addr ; INT0addr is the address of EXT_INT0
jmp EXT_INT0
.org INT1addr ; INT1addr is the address of EXT_INT1
jmp EXT_INT1
jmp Default ; IRQ2 Handler
jmp Default ; IRQ3 Handler
jmp Default ; IRQ4 Handler
jmp Default ; IRQ5 Handler
jmp Default ; IRQ6 Handler
jmp Default ; IRQ7 Handler
jmp Default ; Timer2 Compare Handler
jmp Timer2 ; Timer2 Overflow Handler
jmp Default ; Timer1 Capture Handler
jmp Default ; Timer1 CompareA Handler
jmp Default ; Timer1 CompareB Handler
jmp Default ; Timer1 Overflow Handler
jmp Default ; Timer0 Compare Handler
jmp Timer0  ; Timer0 Overflow Handler
Default: reti

; Constant Game LCD Characters
.equ FIELD_SEPERATOR = ':'
.equ VIEW_SEPERATOR = '|'
.equ SPACE = ' '
.equ CAR_SYMBOL = 'c'					; ascii character for car (representing player)
.equ CAR_WITH_SHIELD_SYMBOL = 236		; ascii character for car with shield buff
.equ SHIELD_SYMBOL = 243				; ascii character for shield powerup
.equ OBSTACLE_SYMBOL = 252				; ascii character for obstacle (or monster)
.equ POWERUP_SYMBOL = 'S'
.equ EXPLOSION_SYMBOL = '*'				; symbol shown at crash position if car crashes

.equ LINE_POS_1 = 0b10000000			; Car position constants
.equ LINE_POS_2 = 0b01000000
.equ LINE_POS_3 = 0b00100000
.equ LINE_POS_4 = 0b00010000
.equ LINE_POS_5 = 0b00001000
.equ LINE_POS_6 = 0b00000100
.equ LINE_POS_7 = 0b00000010
.equ LINE_POS_8 = 0b00000001

.equ NUMBER_FLASHES_INTRO = 3
.equ INTRO_TOP_STRING_SIZE = 14	; string constant character sizes
.equ INTRO_BOT_STRING_SIZE = 16
.equ GAME_OVER_STRING_SIZE = 10
.equ SCORE_STRING_SIZE = 7

; Constant Time Step values (rounded up to nearest whole)
.equ OVERFLOW_FOR_DOUBLE_PRESS = 5
.equ OVERFLOW_PER_SEC = 36		; ~= 1 second for next level timer
.equ TIME_STEP_LVL_1 = 36		; 3.6 overflows per 0.1 sec
.equ TIME_STEP_LVL_2 = 33
.equ TIME_STEP_LVL_3 = 29
.equ TIME_STEP_LVL_4 = 26
.equ TIME_STEP_LVL_5 = 22
.equ TIME_STEP_LVL_6 = 18
.equ TIME_STEP_LVL_7 = 10
.equ TIME_STEP_LVL_8 = 4
; Levels 9 and above are half duration than the previous level

; Game logic constants
.equ START_LEVEL = 1
.equ START_NUM_LIVES = 3
.equ POWERUP_MULTIPLIER = 10
.equ LEVEL_DURATION = 30		    ; each level lasts 30 seconds
.equ SECOND_PRESS_DURATION = 12 	; 0 prescaling on timer2 means roughly 0.1 seconds to register double press

; Random num generator constants
.equ RAND_A = 214013
.equ RAND_C = 2531011

; New element percentage chance from 0-255 (rounded up to nearest whole)
.equ PERCENT_2_TOP = 3			; top 1% of numbers
.equ PERCENT_2_BOT = 6			; top 2% of numbers
.equ PERCENT_3_TOP = 15			; top 6% of numbers
.equ PERCENT_3_BOT = 26			; top 10% of numbers
.equ PERCENT_25_TOP = 90		; top 35% of numbers
.equ PERCENT_25_BOT = 154		; top 60% of numbers
.equ PERCENT_40 = 255			; 100% of numbers

; Motor constants
.equ MOTOR_ON_TIME = 80	; duration motor is on, 80 ~= 2 seconds
.equ MOTOR_SPEED = 0x92	; will result in motor speed being 70 rpm, PWM value obtained via lab04 measurement

; Keypad constants
.equ PORTBDIR = 0xF0
.equ INITCOLMASK = 0xEF
.equ INITROWMASK = 0x01
.equ ROWINPUTMASK = 0x0F

;LCD protocol control bits
.equ LCD_RS = 3
.equ LCD_RW = 1
.equ LCD_E = 2

;LCD functions
.equ LCD_FUNC_SET = 0b00110000
.equ LCD_DISP_OFF = 0b00001000
.equ LCD_DISP_CLR = 0b00000001
.equ LCD_DISP_ON = 0b00001100
.equ LCD_ENTRY_SET = 0b00000100
.equ LCD_ADDR_SET = 0b10000000
;LCD function bits and constants
.equ LCD_BF = 7
.equ LCD_N = 3
.equ LCD_F = 2
.equ LCD_ID = 1
.equ LCD_S = 0
.equ LCD_C = 1
.equ LCD_B = 0
.equ LCD_LINE1 = 0
.equ LCD_LINE2 = 0x40

; ******** Macros ********

; REPEAT_DELAY macro will repeat a delay of given time 'n' number of times
; del_lo:del_hi are used and altered in this macro
; @0 = low number for delay, @1 = high number for delay and @2 = register with number of times to repeat (n)
.MACRO REPEAT_DELAY
	REPEAT_DELAY_AGAIN:
	rcall delay
	ldi data_lo, @0
	ldi data_hi, @1

	dec @2
	cpi @2, 0
	brne REPEAT_DELAY_AGAIN  
.ENDMACRO


; ******** Code Segment ********
.cseg

;Initialise required variables and then jump to main
RESET:
	;Initialise stack pointer
	ldi temp, LOW(RAMEND)
	out SPL, temp 
	ldi temp, HIGH(RAMEND) 
	out SPH, temp

	; Show introduction screen (flashing then still)
	rcall lcd_init
	rcall flashIntroMessage

SOFT_RESET:	; Soft reset used to restart game without showing initial message

	; Set up Timer0
	; Set the Timer0 to Phase Correct PWM mode.
	; Also set Timer0 to have no prescaling
	ldi temp, (1<< WGM00)|(1<<COM01)|(1<<CS00)
	out TCCR0, temp          ; no prescaling

	; Set up Timer2
	ldi temp, 0b00000001    
	out TCCR2, temp          ; no prescaling

	; Enable timer 0 and timer 2
	ldi temp, (1<<TOIE2)  | (1<<TOIE0)
	out TIMSK, temp

	; Initialise time step timer counters in data memory
	clr temp		; clear counter
	sts Timer_Counter, temp				; store in data memory
	sts Timer_Overflow_Count, temp

	ldi temp, TIME_STEP_LVL_1 ; set correct number of overflows for start level
	sts Timer_Overflow_Per_Sec, temp ; store in data memory
	
	; Initialise next level timer counters in data memory
	clr temp	;clear counter
	sts Timer_Level_Counter, temp
	sts Timer_Level_Overflow_Count, temp
	sts Timer_Level_Sec_Count, temp

	; Initialise second INT1 press variables in data memory
	sts TIMER_INT1_Second_Press, temp	; 0 is disabled, ie not on second press
	sts TIMER_INT1_Overflow_Count, temp

	; Enable INT0 and INT1
	clr temp                   			  ; clear temp
	out DDRD,temp              			  ; making PORTD as input
	ldi temp, (2 << ISC10) | (2 << ISC00) ; setting the interrupts for falling edge
	sts EICRA, temp                       ; storing them into EICRA 
	in temp, EIMSK                        ; taking the values inside the EIMSK  
	ori temp, (1<<INT0) | (1<<INT1)       ; oring the values with INT0 and INT1  
	out EIMSK, temp                       ; enabling interrput0 and interrupt1

	; Set up Keypad
	ldi temp, PORTBDIR ; columns are outputs, rows are inputs
	out DDRA, temp
	ldi mask, INITCOLMASK ; initial column mask
	
	; Set up LED
	; We use PORTF (non IO port) and PORTE to use all 10 LEDs available
	ser temp
	sts DDRF, temp	;	non io port
	out DDRE, temp	; enable port e

	clr temp
	sts PORTF, temp ; Initially no LED's on
	out PORTE, temp

	;Initialise game variables
	ldi level, START_LEVEL
	ldi lives, START_NUM_LIVES
	clr score_lo
	clr score_hi

	ldi car_top, LINE_POS_1		;car starts top line at most left position (LINE_POS_1)
	clr car_bot					;empty car bot line

	clr obs_top					; no obstacles initially
	clr obs_bot

	clr powerup					; no powerups initially

	clr shield					; no shields initially and player not shielded
	sts IS_PLAYER_SHIELDED, shield

	; Show initial gameview
	rcall lcd_init
	rcall printGameView

	; Add slight delay so gameview initial state is visible
	ldi data_lo, low(500)
	ldi data_hi, high(500)
	rcall delay

	sei  ; Now enable global interrupts

end: rjmp end	;infinite loop after reset, interrupts control flow of program

; ******** Timer Interrupts ********

; Timer 0, timer with highest priority will be used to poll the keypad each interupt
; This ensures a reactive move of the player object that is independant of the time step
Timer0:
	;push conflict registers
	push data_lo
	push data_hi
	push temp

	sts MASK_STORE, mask	; store the current mask in data memory

	; Poll the keypad, temp and data_hi will be used as temp registers as required
	ldi mask, INITCOLMASK ; restore initial column mask
	clr col ; clear column number
	
	Timer0_colloop:
		out PORTA, mask ;set ground for current column

		ldi data_hi, high(40)
		ldi data_lo, low(40)
		rcall delay	; delay to allow stabilisation

		in temp, PINA ; read PORTA
		andi temp, ROWINPUTMASK ; read only the row bits
		cpi temp, 0x0F ; check if any rows are grounded
		breq Timer0_nextcol ; if not go to the next column

		ldi mask, INITROWMASK ; initialise mask for row check
		clr row ; clear row number
	
	Timer0_rowloop:
		mov data_hi, temp
		and data_hi, mask ; check masked bit
		brne Timer0_nextrow ; next row if not found

		ldi data_hi, high(40)
		ldi data_lo, low(40)
		rcall delay ; delay after each loop (debouncing)

		lds data_lo, MASK_STORE		; Check if this mask is same as previously stored mask
		cp data_lo, temp
		breq Timer0_rowloop_delay			; If so, don't perform keypad action

		rcall keypad_pressed		; row found, button pressed so we perform keypad action

		mov mask, temp			; store current mask for next keypad poll
		sts MASK_STORE, temp

		rjmp Timer0_end_keypad_scan

		Timer0_rowloop_delay:	; row not found (same mask), delay for slight duration for stabilisation
		ldi data_lo, LOW(15)
		ldi data_hi, HIGH(15)
		rcall delay
	
		mov mask, temp	; store current mask for next keypad poll
		sts MASK_STORE, temp

		rjmp Timer0_end_keypad_scan ; keypad scan done
	
	Timer0_nextrow:
		inc row ; next row
		lsl mask ; shift the mask to the next bit
		rjmp Timer0_rowloop

	Timer0_nextcol:
		cpi col, 3 
		breq Timer0_end_keypad_scan ; start again (on last column)

		lsl mask ; and then rotate column mask left
		inc col ; next column
		jmp Timer0_colloop

	Timer0_end_keypad_scan:

	;pop conflicts
	pop temp
	pop data_hi
	pop data_lo
reti

; Perform required task based on key pressed
; Helper subroutine used by Timer0
keypad_pressed:
	;push conflicts
	push temp

	; Check cols and rows to determine which key was pressed

	cpi row, 0	;check if row is zero
	brne keypad_pressed_not_row_0

	keypad_pressed_row_0:
	cpi col, 1	;check if col is 1
	brne keypad_pressed_not_row_0
	
	;Key 2 was pressed
	rcall moveUp
	rjmp keypad_pressed_end

	keypad_pressed_not_row_0:
	cpi row, 1 ; check if row is row 1 (second row)
	brne keypad_pressed_not_row_1

	cpi col, 0	;check if col is 0
	brne keypad_pressed_not_row_0_col_not_0

	; Key 4 was pressed
	rcall moveLeft
	rjmp keypad_pressed_end

	keypad_pressed_not_row_0_col_not_0:
	cpi col, 2	;check if col is 2
	brne keypad_pressed_not_row_1

	; Key 6 was pressed
	rcall moveRight
	rjmp keypad_pressed_end

	keypad_pressed_not_row_1:
	cpi row, 2	;check if row is zero
	brne keypad_pressed_end
	cpi col, 1
	brne keypad_pressed_end

	; Key 8 was pressed
	rcall moveDown

	keypad_pressed_end:

	; pop conflicts
	pop temp
ret

; Timer 2 will control the timing of the time step in addition to the timing of the level duration
; This timer will also call the game update subroutine each time step
Timer2:
	;push conflict registers
	push data_lo
	push data_hi
	push temp

	; *** Timer Overflow Checks ***
	; Check if this is an overflow for level counter 
	lds temp, Timer_Level_Overflow_Count	; Load overflow count
	inc temp	; increment counter
	sts Timer_Level_Overflow_Count, temp

	brne Timer_Level_end	; ignore if not 0, ie not increment after overflow

	; Check INT1 second press
	lds temp, TIMER_INT1_Second_Press
	cpi temp, 0
	breq Timer2_Level_Checks

	lds temp, TIMER_INT1_Overflow_Count	;load overflow
	inc temp
	sts TIMER_INT1_Overflow_Count, temp

	cpi temp, OVERFLOW_FOR_DOUBLE_PRESS	; Check if enough time has passed
	brne Timer2_Level_Checks		; if not enough time yet, ignore

	; Otherwise, its been enough time, disable second press opportunity, reset overflow counter
	clr temp
	sts TIMER_INT1_Second_Press, temp
	sts TIMER_INT1_Overflow_Count, temp

	; Load timer counters from data memory
	Timer2_Level_Checks:

	lds temp, Timer_Level_Counter
	inc temp					 ; inc Timer_Level_Counter
	sts Timer_Level_Counter, temp

	cpi temp, OVERFLOW_PER_SEC   ; compare Timer_Level_Counter with overflows in one second
	brne Timer_Level_end		 ; bridge if not one second

	; It has been one second, check if level is over
	lds temp, Timer_Level_Sec_Count
	inc temp
	sts Timer_Level_Sec_Count, temp

	; Reset overflow counter as its reached its theoretical max
	clr data_lo
	sts Timer_Level_Counter, data_lo

	cpi temp, LEVEL_DURATION ; check if it has been enough time for this game
	brne Timer_Level_end

	; Level has now been beaten, go to next level
	rcall gotoNextLevel
	rjmp Timer2_end		;exit interupt

	
	Timer_Level_end:

	; Check if this is an overflow for time step
	lds temp, Timer_Overflow_Count	; Load overflow count
	inc temp	; increment counter
	sts Timer_Overflow_Count, temp

	brne Timer2_end		; ignore if not 0, ie not increment after overflow

	; Load timer counters from data memory
	lds temp, Timer_Counter
	lds data_lo, Timer_Overflow_Per_Sec

	; Check for duration of time passed
	inc temp					; inc Timer_Counter
	sts Timer_Counter, temp		; Write increment back to data memory

	cp temp, data_lo	;compare Timer_Counter with Timer_Overflow_Count
	brne Timer2_end

	; It has been the required duration of time (time step)
	; Update game state by moving everything left
	rcall gameUpdate

	; Reset required time step counters
	clr temp ; clear counter
	sts Timer_Counter, temp

	Timer2_end:

	;pop conflicts
	pop temp
	pop data_hi
	pop data_lo
reti

; ******** External Interrupts ********

; Interrupt invoked on PB0 press
; Action completed: Reset Game to default mode, that is restart from level 1 without showing initial welcome message
EXT_INT0:
	; push conflicts
	push temp
	in temp, SREG
	push temp
	push data_lo
	push data_hi

	; Debouncing
	; If button pressed, wait
	ldi data_lo, 0b10000000
	in temp, PIND
	and temp, data_lo
	cpi temp, 0
	breq EXT_INT0_end

	; Delay for significant duration to ensure press
	ldi temp, 4						
	REPEAT_DELAY low(65000), high(65000), temp	; delay macro

	; Check again for press
	ldi data_lo, 0b10000000
	in temp, PIND
	and temp, data_lo
	cpi temp, 0
	breq EXT_INT0_end

	; Button pressed, restore registers (SREG must be preserved)
	pop data_hi
	pop data_lo
	pop temp
	out SREG, temp
	pop temp

	cli					; disable all interrupts to avoid them disturbing reset process
	rjmp SOFT_RESET		; perform soft reset

	EXT_INT0_end:
	; pop conflicts
	pop data_hi
	pop data_lo
	pop temp
	out SREG, temp
	pop temp
reti


; Interrupt invoked on PB1 press
; Action completed: Go to next level
EXT_INT1:
	; push conflicts
	push temp
	in temp, SREG
	push temp
	push data_lo
	push data_hi

	; Debouncing
	; If button pressed, wait
	ldi data_lo, 0b01000000
	in temp, PIND
	and temp, data_lo
	cpi temp, 0
	breq EXT_INT1_end

	; Delay for short duration
	ldi temp, 2									; call 2 times
	REPEAT_DELAY low(65000), high(65000), temp	; delay macro

	; Check again for press
	ldi data_lo, 0b01000000
	in temp, PIND
	and temp, data_lo
	cpi temp, 0
	breq EXT_INT1_end

	; Button pressed, check to see if this is 1 press or 2 presses
	lds temp, TIMER_INT1_Second_Press
	cpi temp, 1
	breq EXT_INT1_Next_Level	; if second press, goto next level

	EXT_INT1_Restart_Current_LeveL:
	; Restart current level
	rcall restartCurrentLevel

	; Set up second press flag
	ldi temp, 1
	sts TIMER_INT1_Second_Press, temp

	; Delay for short duration
	ldi temp, 2									; call 2 times
	REPEAT_DELAY low(65000), high(65000), temp	; delay macro

	rjmp EXT_INT1_Action_Performed

	EXT_INT1_Next_Level:
	; Disable second press flag/counter
	clr temp
	sts TIMER_INT1_Second_Press, temp
	sts TIMER_INT1_Overflow_Count, temp

	; Go to next level
	rcall gotoNextLevel

	; Delay for short duration
	ldi temp, 2									; call 2 times
	REPEAT_DELAY low(65000), high(65000), temp	; delay macro

	EXT_INT1_Action_Performed:

	; Print screen again
	rcall lcd_init
	rcall printGameView

	EXT_INT1_end:

	; pop conflicts
	pop data_hi
	pop data_lo
	pop temp
	out SREG, temp
	pop temp
reti


; ******** LCD Initialisation functions ********

;Function lcd_write_com: Write a command to the LCD. The data_lo reg stores the value to be written.
lcd_write_com:
	push temp	;push conflicts

	out PORTC, data_lo ; set the data_lo port's value up
	clr temp
	out PORTB, temp ; RS = 0, RW = 0 for a command write
	nop ; delay to meet timing (Set up time)
	sbi PORTB, LCD_E ; turn on the enable pin
	nop ; delay to meet timing (Enable pulse width)
	nop
	nop
	cbi PORTB, LCD_E ; turn off the enable pin
	nop ; delay to meet timing (Enable cycle time)
	nop
	nop

	pop temp ; pop conflicts
ret

;Function lcd_write_data_lo: Write a character to the LCD. The data_lo reg stores the value to be written.
lcd_write_data_lo:
	push temp	;push conflicts

	out PORTC, data_lo ; set the data_lo port's value up
	ldi temp, 1 << LCD_RS
	out PORTB, temp ; RS = 1, RW = 0 for a data_lo write
	nop ; delay to meet timing (Set up time)
	sbi PORTB, LCD_E ; turn on the enable pin
	nop ; delay to meet timing (Enable pulse width)
	nop
	nop
	cbi PORTB, LCD_E ; turn off the enable pin
	nop ; delay to meet timing (Enable cycle time)
	nop
	nop

	pop temp ; pop conflicts
ret

;Function lcd_wait_busy: Read the LCD busy flag until it reads as not busy.
lcd_wait_busy:
	push temp	;push conflicts

	clr temp
	out DDRC, temp ; Make PORTC be an input port for now
	out PORTC, temp
	ldi temp, 1 << LCD_RW
	out PORTB, temp ; RS = 0, RW = 1 for a command port read
busy_loop:
	nop ; delay to meet timing (Set up time / Enable cycle time)
	sbi PORTB, LCD_E ; turn on the enable pin
	nop ; delay to meet timing (data_lo delay time)
	nop
	nop
	in temp, PINC ; read value from LCD
	cbi PORTB, LCD_E ; turn off the enable pin
	sbrc temp, LCD_BF ; if the busy flag is set
	rjmp busy_loop ; repeat command read
	clr temp ; else
	out PORTB, temp ; turn off read mode,
	ser temp
	out DDRC, temp ; make PORTC an output port again

	pop temp ; pop conflicts
ret ; and return

; Function lcd_init Initialisation function for LCD.
lcd_init:
	push temp	;push conflicts

	ser temp
	out DDRC, temp ; PORTC, the data_lo port is usually all otuputs
	out DDRB, temp ; PORTB, the control port is always all outputs
	ldi data_lo, low(15000)
	ldi data_hi, high(15000)
	rcall delay ; delay for > 15ms

	; Function set command with N = 1 and F = 0
	ldi data_lo, LCD_FUNC_SET | (1 << LCD_N)
	rcall lcd_write_com ; 1st Function set command with 2 lines and 5*7 font
	ldi data_lo, low(4100)
	ldi data_hi, high(4100)
	rcall delay ; delay for > 4.1ms
	rcall lcd_write_com ; 2nd Function set command with 2 lines and 5*7 font
	ldi data_lo, low(100)
	ldi data_hi, high(100)
	rcall delay ; delay for > 100us
	rcall lcd_write_com ; 3rd Function set command with 2 lines and 5*7 font
	rcall lcd_write_com ; Final Function set command with 2 lines and 5*7 font
	rcall lcd_wait_busy ; Wait until the LCD is ready
	ldi data_lo, LCD_DISP_OFF
	rcall lcd_write_com ; Turn Display off
	rcall lcd_wait_busy ; Wait until the LCD is ready
	ldi data_lo, LCD_DISP_CLR
	rcall lcd_write_com ; Clear Display
	rcall lcd_wait_busy ; Wait until the LCD is ready
	; Entry set command with I/D = 1 and S = 0
	ldi data_lo, LCD_ENTRY_SET | (1 << LCD_ID)
	rcall lcd_write_com ; Set Entry mode: Increment = yes and Shift = no
	rcall lcd_wait_busy ; Wait until the LCD is ready
	; Display on command with C = 0 and B = 1
	ldi data_lo, LCD_DISP_ON | (1 << LCD_C)
	rcall lcd_write_com ; Trun Display on with a cursor that doesn't blink

	pop temp ; pop conflicts
ret

; ******** Special LCD Display Functions ********

; Display flashing intro message as an introduction to the game
; Title and welcome message are flashed a few times before being displayed for a few seconds
flashIntroMessage:
	; push conflicts
	push r27
	push temp
	push data_lo
	push data_hi

	; Show introduction screen (flashing then still)
	rcall lcd_init
	rcall printIntroMessage

	ldi r27, NUMBER_FLASHES_INTRO	; show this many flashes

	RESET_intro_screen_repeat:
		; Delay for short duration
		ldi temp, 10							
		REPEAT_DELAY low(65000), high(65000), temp	; delay macro

		rcall lcd_init
		rcall lcd_wait_busy
		ldi data_lo, LCD_DISP_ON | (0 << LCD_C)	; Turn Display on without a cursor (so you have blank screen) 
		rcall lcd_write_com 

		; Delay for short duration
		ldi temp, 10							
		REPEAT_DELAY low(65000), high(65000), temp	; delay macro

		rcall printIntroMessage

		dec r27
		cpi r27, 0
		brne RESET_intro_screen_repeat

	; Show title for a few seconds
	; Delay for longer duration so title is clearly visible
	ldi temp, 50							
	REPEAT_DELAY low(65000), high(65000), temp	; delay macro

	; pop conflicts
	pop data_hi
	pop data_lo
	pop temp
	pop r27
ret

; Display intro message onto screen on game restart
printIntroMessage:
	;Push conflict registers
	push data_lo
	push data_hi
	push temp
	push ZH
	push ZL

	; Print Intro message

	; *** Print Line 1 ***
	; 1 space on top start
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo

	; Print game title
	ldi ZL, LOW(INTRO_TOP_STRING << 1)		; set up stack
	ldi ZH, HIGH(INTRO_TOP_STRING << 1)

	ldi temp, INTRO_TOP_STRING_SIZE         ; initialise counter 

	printIntroMessage_top: 
        lpm data_lo, Z+                    ; read a character from the string 
        rcall lcd_wait_busy
        rcall lcd_write_data_lo            ; write the character to the screen
        dec temp                           ; decrement character counter
        brne printIntroMessage_top      

	; 1 space on top end
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo
	
	; *** Print Line 2 ***

	; Move insertion point to line 2
	rcall lcd_wait_busy
	ldi data_lo, LCD_ADDR_SET | LCD_LINE2
	rcall lcd_write_com                     ; move the insertion point to start of line 2

	; Print Welcoming Message
	ldi ZL, LOW(INTRO_BOT_STRING << 1)		; set up stack
	ldi ZH, HIGH(INTRO_BOT_STRING << 1)

	ldi temp, INTRO_BOT_STRING_SIZE         ; initialise counter 

	printIntroMessage_bot: 
        lpm data_lo, Z+                    ; read a character from the string 
        rcall lcd_wait_busy
        rcall lcd_write_data_lo            ; write the character to the screen
        dec temp                           ; decrement character counter
        brne printIntroMessage_bot

	;Pop conflict registers
	pop ZL
	pop ZH
	pop temp
	pop data_hi
	pop data_lo
ret

; Display message onto screen
printGameOverMessage:
	;push conflict registers
	push data_lo
	push data_hi
	push ZH
	push ZL

	; Print Game Over message

	; *** Print Line 1 ***
	; 3 Spaces on top start
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo

	; Print Game over text
	ldi ZL, LOW(GAME_OVER_STRING << 1)		; set up stack
	ldi ZH, HIGH(GAME_OVER_STRING << 1)

	ldi temp, GAME_OVER_STRING_SIZE         ; initialise counter 

	printMessage_loop_gameover: 
        lpm data_lo, Z+                    ; read a character from the string 
        rcall lcd_wait_busy
        rcall lcd_write_data_lo            ; write the character to the screen
        dec temp                           ; decrement character counter
        brne printMessage_loop_gameover      

	; 3 Spaces on top end
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo

	; *** Print Line 2 ***

	; Move insertion point to line 2
	rcall lcd_wait_busy
	ldi data_lo, LCD_ADDR_SET | LCD_LINE2
	rcall lcd_write_com                     ; move the insertion point to start of line 2

	; 2 Spaces on top start
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo

	; Print Score text
	ldi ZL, LOW(SCORE_STRING << 1)		; set up stack
	ldi ZH, HIGH(SCORE_STRING << 1)

	ldi temp, SCORE_STRING_SIZE    ; initialise counter 

	printMessage_loop_score: 
        lpm data_lo, Z+                    ; read a character from the string 
        rcall lcd_wait_busy
        rcall lcd_write_data_lo            ; write the character to the screen
        dec temp                           ; decrement character counter
        brne printMessage_loop_score      

	; Print score to screen (5 char long)
	mov data_lo, score_lo
	mov data_hi, score_hi
	ldi temp, 5
	rcall printNumToLCD

	; 2 Spaces on top end
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo
	ldi data_lo, SPACE
    rcall lcd_wait_busy
    rcall lcd_write_data_lo

	;pop conflict registers
	pop ZL
	pop ZH
	pop data_hi
	pop data_lo
ret

; Display ingame view for active game in correct format using latest game state
printGameView:
	;push conflict registers
	push data_lo
	push data_hi

	; *** Print Line 1 ***
	
	;Print lives (L:)
	ldi data_lo, 'L'
    rcall lcd_wait_busy
    rcall lcd_write_data_lo
    ldi data_lo, FIELD_SEPERATOR
	rcall lcd_wait_busy
    rcall lcd_write_data_lo            ; write the character to the screen
	
	; Print level (1 or 2 char long)
	ldi temp, 1
	cpi level, 10
	brlo single_level
	ldi temp, 2

	single_level:
	;For levels higher than 10, print 2 digits
	mov data_lo, level
	ldi data_hi, HIGH(0)	;empty high byte
	rcall printNumToLCD

	cpi level, 10
	brsh skip_level_space

	; Print single space seperator (1 char long)
	ldi data_lo, SPACE
	rcall lcd_wait_busy
    rcall lcd_write_data_lo            ; write the character to the screen

	skip_level_space:

	;Print cars label (2 char long)
	ldi data_lo, 'C'
    rcall lcd_wait_busy
    rcall lcd_write_data_lo
    ldi data_lo, FIELD_SEPERATOR
	rcall lcd_wait_busy
    rcall lcd_write_data_lo            ; write the character to the screen
	
	; Print num of lives (1 char long)
	mov data_lo, lives
	ldi data_hi, HIGH(0)	;empty high byte
	ldi temp, 1
	rcall printNumToLCD

	; Print Half way seperator (1 char long)
	ldi data_lo, VIEW_SEPERATOR
	rcall lcd_wait_busy
    rcall lcd_write_data_lo 

	; Print TOP game area (8 chars long)
	ldi temp, 0b10000000	;check from the MSB to LSB

	print_top:
		; Check Car
		mov data_lo, temp		; store temp in data_lo
		and data_lo, car_top	;logical and to see if this bit is set or not in car_top

		cpi data_lo, 0	; compare with 0
		breq obs_top_check	;continue if 0

		; Check Exploision (car/obstacle overlap)
		and data_lo, obs_top
		
		cpi data_lo, 0 ; compare with 0
		breq car_top_symbol	; if nothing, print car
		ldi data_lo, EXPLOSION_SYMBOL
		rjmp print_top_symbol

		car_top_symbol:
		; Check if car has shield
		lds data_lo, IS_PLAYER_SHIELDED
		cpi data_lo, 0
		breq car_top_symbol_no_shield

		ldi data_lo, CAR_WITH_SHIELD_SYMBOL
		rjmp print_top_symbol

		car_top_symbol_no_shield:
		ldi data_lo, CAR_SYMBOL
		rjmp print_top_symbol

		; Check obss
		obs_top_check:
		mov data_lo, temp		; store temp in data_lo
		and data_lo, obs_top	;logical and to see if this bit is set or not in obs_top

		cpi data_lo, 0	;	compare with 0
		breq shield_top_check	;continue if 0
		ldi data_lo, OBSTACLE_SYMBOL
		rjmp print_top_symbol

		; Check shield power up
		shield_top_check:
		mov data_lo, temp			; store temp in data_lo
		cpi data_lo, 0b00010000		; do not check 4 MSB as they are not top powerups
		brsh powerup_top_check
		and data_lo, shield    	;logical and to see if this bit is set or not in powerup bot

		; If checking 4 LSB, shift top powerups right
		mov data_hi, shield
		lsr data_hi
		lsr data_hi
		lsr data_hi
		lsr data_hi

		and data_lo, data_hi    	; logical and to see if this bit is set or not in powerup top

		cpi data_lo, 0	;	compare with 0
		breq powerup_top_check	; continue if 0
		ldi data_lo, SHIELD_SYMBOL
		rjmp print_top_symbol

		; Check power ups
		powerup_top_check:
		mov data_lo, temp			; store temp in data_lo
		cpi data_lo, 0b00010000		; do not check 4 MSB as they are not top powerups
		brsh print_top_space

		; If checking 4 LSB, shift top powerups right
		mov data_hi, powerup
		lsr data_hi
		lsr data_hi
		lsr data_hi
		lsr data_hi

		and data_lo, data_hi    	; logical and to see if this bit is set or not in powerup top

		cpi data_lo, 0	;	compare with 0
		breq print_top_space	; write space if 0
		ldi data_lo, POWERUP_SYMBOL

		; Print symbol for this position
		print_top_symbol:
		rcall lcd_wait_busy
		rcall lcd_write_data_lo
		rjmp print_top_check

		print_top_space:
		ldi data_lo, SPACE
		rcall lcd_wait_busy
		rcall lcd_write_data_lo            ; write the character to the screen

		print_top_check:
		lsr temp
		cpi temp, 0		; leave if last bit to check
		breq print_top_end
		rjmp print_top

	print_top_end:

	; *** Print Line 2 ***

	; Move insertion point to line 2
	rcall lcd_wait_busy
	ldi data_lo, LCD_ADDR_SET | LCD_LINE2
	rcall lcd_write_com                     ; move the insertion point to start of line 2

	;Print Score label (2 char long)
	ldi data_lo, 'S'
    rcall lcd_wait_busy
    rcall lcd_write_data_lo
    ldi data_lo, FIELD_SEPERATOR
	rcall lcd_wait_busy
    rcall lcd_write_data_lo            ; write the character to the screen
	
	; Print score to screen (5 char long)
	mov data_lo, score_lo
	mov data_hi, score_hi
	ldi temp, 5
	rcall printNumToLCD

	; Whenever Score is printed onto screen, ensure LED's display LSB of score
	rcall updateLEDScore

	; Print Half way seperator
	ldi data_lo, VIEW_SEPERATOR
	rcall lcd_wait_busy
    rcall lcd_write_data_lo 

	; Print BOT game area (8 chars long)
	ldi temp, 0b10000000	;check from the MSB to LSB

	print_bot:
		; Check Car
		mov data_lo, temp		; store temp in data_lo
		and data_lo, car_bot	;logical and to see if this bit is set or not in car_bot

		cpi data_lo, 0	; compare with 0
		breq obs_bot_check	;continue if 0

		; Check Exploision (car/obstacle overlap)
		and data_lo, obs_bot
		
		cpi data_lo, 0 ; compare with 0
		breq car_bot_symbol	; if nothing, print car
		ldi data_lo, EXPLOSION_SYMBOL
		rjmp print_bot_symbol

		car_bot_symbol:
		; Check if car has shield
		lds data_lo, IS_PLAYER_SHIELDED
		cpi data_lo, 0
		breq car_bot_symbol_no_shield

		ldi data_lo, CAR_WITH_SHIELD_SYMBOL
		rjmp print_bot_symbol

		car_bot_symbol_no_shield:
		ldi data_lo, CAR_SYMBOL
		rjmp print_bot_symbol

		; Check obss
		obs_bot_check:
		mov data_lo, temp		; store temp in data_lo
		and data_lo, obs_bot	;logical and to see if this bit is set or not in obs_bot

		cpi data_lo, 0	;	compare with 0
		breq shield_bot_check	;continue if 0
		ldi data_lo, OBSTACLE_SYMBOL
		rjmp print_bot_symbol

		; Check shield power up
		shield_bot_check:
		mov data_lo, temp			; store temp in data_lo
		cpi data_lo, 0b00010000		; do not check 4 MSB as they are not bot shields
		brsh powerup_bot_check
		and data_lo, shield    	;logical and to see if this bit is set or not in powerup bot

		cpi data_lo, 0	;	compare with 0
		breq powerup_bot_check	; continue if 0
		ldi data_lo, SHIELD_SYMBOL
		rjmp print_bot_symbol

		; Check power ups
		powerup_bot_check:
		mov data_lo, temp			; store temp in data_lo
		cpi data_lo, 0b00010000		; do not check 4 MSB as they are not bot powerups
		brsh print_bot_space
		and data_lo, powerup    	;logical and to see if this bit is set or not in powerup bot

		cpi data_lo, 0	;	compare with 0
		breq print_bot_space	; write space if 0
		ldi data_lo, POWERUP_SYMBOL

		; Print symbol for this position
		print_bot_symbol:
		rcall lcd_wait_busy
		rcall lcd_write_data_lo
		rjmp print_bot_check

		print_bot_space:
		ldi data_lo, SPACE
		rcall lcd_wait_busy
		rcall lcd_write_data_lo            ; write the character to the screen

		print_bot_check:
		lsr temp
		cpi temp, 0		; leave if last bit to check
		breq print_bot_end
		rjmp print_bot

	print_bot_end:

	;pop conflict registers
	pop data_hi
	pop data_lo
ret

; Split value in data_lo into its units and convert to ascii characters.
; Print these ascii characters to the LCD on a previously specified line (ie at current blinker position)
;
; Precondition: Number must be stored in data_lo:data_hi
;				Number must be unsigned (ie 0 or positive)
;				Number must be equal to or lower than than 65535
;				temp stores the number of digits to print, must be 0 < temp <= 5

printNumToLCD:
	;push conflict registers
	push r18	; r18:r19 will hold our compare values
	push r19
	push r20	; r20:r21 will contain our number to print
	push r21
	push r22	; 5 others to hold our units for a 5 digit number
	push r23
	push r24
	push r25
	push r26
	push r27	; stores number of digits to print

	;Move 16-bit unsigned int to r20:r21 from data_lo:data_hi
	movw r20, data_lo

	; Store number of digits to print in r27
	mov r27, temp

    clr r22		;stores number of 10000's
    clr r23		;stores number of 1000's
    clr r24		;stores number of 100's
	clr r25		;stores number of 10's
	clr r26		;stores number of 1's

	start10000s:
		ldi r18, low(10000)
		ldi r19, high(10000)

		start10000s_loop:
		;compare with 10000
		cp r20, r18; compare lower bytes
		cpc r21, r19 ; compare upper bytes w/ carry

		brcs start1000s	;if carry set, r20:r21 is smaller
		inc r22

		;subtract 10000 from our number
		sub r20, r18 ; first the low-byte
		sbc r21, r19 ; then the high-byte

		rjmp start10000s_loop	;repeat until no more 10000's

	start1000s:
		ldi r18, low(1000)
		ldi r19, high(1000)

		start1000s_loop:
		;compare with 1000
		cp r20, r18; compare lower bytes
		cpc r21, r19 ; compare upper bytes w/ carry

		brcs start100s	;if carry set, r20:r21 is smaller
		inc r23

		;subtract 1000 from our number
		sub r20, r18 ; first the low-byte
		sbc r21, r19 ; then the high-byte

		rjmp start1000s_loop	;repeat until no more 1000's

	start100s:
		ldi r18, low(100)
		ldi r19, high(100)

		start100s_loop:
		;compare with 100
		cp r20, r18; compare lower bytes
		cpc r21, r19 ; compare upper bytes w/ carry

		brcs start10s	;if carry set, r20:r21 is smaller
		inc r24

		;subtract 100 from our number
		sub r20, r18 ; first the low-byte
		sbc r21, r19 ; then the high-byte

		rjmp start100s_loop	;repeat until no more 100's

	start10s:
		;compare with 10
		cpi r20, 10	;compare lower byte

		brlo start1s	;if r20 is smaller
		inc r25

		;subtract 10 from our number
		subi r20, 10

		rjmp start10s	;repeat until no more 10's

	start1s:
		;compare with 1
		cpi r20, 1	;compare lower byte

		brlo ascii_transform	;if carry set, r20:r21 is smaller
		inc r26

		;subtract 1 from our number
		dec r20

		rjmp start1s	;repeat until no more 10000's

	ascii_transform:
	;Convert integers to ascii by adding the value of '0' (first ascii number zero)
    subi r22, -'0'
    subi r23, -'0'
	subi r24, -'0'
	subi r25, -'0'
	subi r26, -'0'
		      
	; Write ascii to LCD
	; Write certain about of spaces, r20 reg free at this point

	mov r20, r27	;move number of total digits to write to r20

	cpi r22, '0'
	breq not_5_digits
	subi r20, 5
	rjmp update_digits_to_write

	not_5_digits:	; must be 3 digits or less
	cpi r23, '0'
	breq not_4_digits
	subi r20, 4
	rjmp update_digits_to_write

	not_4_digits:	; must be 3 digits or less
	cpi r24, '0'
	breq not_3_digits
	subi r20, 3
	rjmp update_digits_to_write

	not_3_digits:	; must be 2 digits or less
	cpi r25, '0'
	breq not_2_digits
	subi r20, 2
	rjmp update_digits_to_write

	not_2_digits:
	subi r20, 1

	update_digits_to_write:	;number of digits to write is total digits (r27) - num of spaces (r20)
	sub r27, r20

	;Write r20 number of spaces
	write_space:
		cpi r20, 0
		breq write_digits

		ldi data_lo, SPACE		;write space
		rcall lcd_wait_busy
		rcall lcd_write_data_lo

		dec r20		;dec number of spaces left to write
		rjmp write_space

	write_digits:
	; Now write r27 number of digits 
	cpi r27, 4
	breq write_4
	cpi r27, 3
	breq write_3
	cpi r27, 2
	breq write_2
	cpi r27, 1
	breq write_1

	write_5:
	mov data_lo, r22		;write 10000's digit
    rcall lcd_wait_busy
    rcall lcd_write_data_lo

	write_4:
	mov data_lo, r23		;write 1000's digit
    rcall lcd_wait_busy
    rcall lcd_write_data_lo

	write_3:
	mov data_lo, r24		;write 1000's digit
    rcall lcd_wait_busy
    rcall lcd_write_data_lo
	
	write_2:
	mov data_lo, r25		;write 1000's digit
    rcall lcd_wait_busy
    rcall lcd_write_data_lo

	write_1:
	mov data_lo, r26			;write last digit, even if '0'
    rcall lcd_wait_busy
    rcall lcd_write_data_lo

	;pop conflict registers
	pop r27
	pop r26
	pop r25
    pop r24
    pop r23
    pop r22
    pop r21
	pop r20
	pop r19
	pop r18
ret

; ******** Game Logic Functions ********

; This subroutine will update the game state, moving all obstacles to their next location
; It also handles the scoring as well as collisions of the player with obstacles or powerups or shields
; New elements to be added on LSB are determined by chance and a random number
; A request to reprint the LCD is made at the end of this function once game state has been updated.
gameUpdate:
	;push conflicts
	push temp
	push data_lo

	; Check MSB (most sig. bits) about to be removed in obstacle
	clr data_lo ;will hold number of obstacles destoryed

	ldi temp, 0b10000000
	and temp, obs_top
	cpi temp, 0		; check if 0, if so no obstacle destroyed
	breq gameUpdate_obstacle_bot_check
	inc data_lo

	gameUpdate_obstacle_bot_check:
	ldi temp, 0b10000000
	and temp, obs_bot
	cpi temp, 0		; check if 0, if so no obstacle destroyed
	breq gameUpdate_obstacle_end
	inc data_lo

	gameUpdate_obstacle_end:
	cpi data_lo, 0					;if no obstacles
	breq gameUpdate_obstacle_no_score

	;Otherwise update the score
	gameUpdate_obstacle_score:
		mov temp, level
		add score_lo, temp		; score added is 'level' points per obstacle destroyed
		ldi temp, HIGH(0)
		adc	score_hi, temp		; add 0 to high

		dec data_lo
		cpi data_lo, 0
		brne gameUpdate_obstacle_score

	gameUpdate_obstacle_no_score:

	; Shift the obstacles left now
	lsl obs_top
	lsl obs_bot

	rcall carCrashed	;check if player has crashes as result of obstacle movement

	; Shift the powerups left now
	lsl powerup

	; We must clear bit 4 as expired bot powerup may have just moved here
	andi powerup, 0b11101111	; logical and to remove bit 4

	rcall powerupCollected	;check if player now has powerup as result of power up movement

	; Shift the shield powerup left now
	lsl shield

	; We must clear bit 4 as expired bot shield powerup may have just moved here
	andi shield, 0b11101111	; logical and to remove bit 4

	rcall shieldCollected	;check if player now has shield as result of shield movement

	; Generate new elements for newest column

	; Check if there was obstacle in last column
	ldi temp, LINE_POS_7	; after shift, this would be line pos 7
	mov data_lo, obs_top
	and data_lo, temp		; logical between and data_lo and temp

	cpi data_lo, 0			; check if there was obstacle
	brne gameUpdate_new_column_end	; if there was, jump to end

	;Otherwise there was an obstacle in top
	; Check bot

	mov data_lo, obs_bot
	and data_lo, temp		; logical between and data_lo and temp

	cpi data_lo, 0			; check if there was obstacle
	brne gameUpdate_new_column_end	; if there was, jump to end

	; There was no obstacle in last column
	gameUpdate_last_column_no_obstacle:

	; Generate random number
	rcall generateRandomNum		;result stored in data_lo

	; Determine element based on number
	; Progresively check random number using lower than check to cover all percentages
	gameUpdate_new_column_shield_top:
	cpi data_lo, PERCENT_2_TOP		; 1%, shield in top row
	brsh gameUpdate_new_column_shield_bot

	ldi temp, LINE_POS_8			; set 8th position
	eor shield, temp			    ; logical or to add powerup at top

	rjmp gameUpdate_new_column_end

	gameUpdate_new_column_shield_bot:
	cpi data_lo, PERCENT_2_BOT		; 1% shield in bot row
	brsh gameUpdate_new_column_powerup_top

	ldi temp, LINE_POS_4			; set 4th position
	eor shield, temp				; logical or to add powerup at bot

	rjmp gameUpdate_new_column_end

	gameUpdate_new_column_powerup_top:
	cpi data_lo, PERCENT_3_TOP		; 1.5% powerup in top row
	brsh gameUpdate_new_column_powerup_bot

	ldi temp, LINE_POS_8			; set 8th position
	eor powerup, temp			    ; logical or to add powerup at top

	rjmp gameUpdate_new_column_end

	gameUpdate_new_column_powerup_bot:
	cpi data_lo, PERCENT_3_BOT		; 1.5%, powerup in bot row
	brsh gameUpdate_new_column_obstacle_top

	ldi temp, LINE_POS_4			; set 4th position
	eor powerup, temp				; logical or to add powerup at bot

	rjmp gameUpdate_new_column_end

	gameUpdate_new_column_obstacle_top:
	cpi data_lo, PERCENT_25_TOP		; 25%, obstacle in top row
	brsh gameUpdate_new_column_obstacle_bot
	
	ldi temp, LINE_POS_8			; set 8th position
	eor obs_top, temp				; logical or to add obstacle

	rjmp gameUpdate_new_column_end

	gameUpdate_new_column_obstacle_bot:
	cpi data_lo, PERCENT_25_BOT		; 25%, obstacle in bot row
	brsh gameUpdate_new_column_end

	ldi temp, LINE_POS_8			; set 8th position
	eor obs_bot, temp				; logical or to add obstacle

	; At this stage, remaining percentage means no new elements
	; Taken care of by logical shift left

	; There was an obstacle, new column must be blank and 
	; that is satisfied already by logical shift left
	gameUpdate_new_column_end:


	;Check which screen to display
	cpi lives, 0
	brne gameUpdate_print_game

	; Print gameover
	rcall lcd_init					; init lcd
	rcall printGameOverMessage		; print to display
	rjmp gameUpdate_end

	; Print game
	gameUpdate_print_game:
	rcall lcd_init					; init lcd
	rcall printGameView				; print to display

	gameUpdate_end:

	;pop conflicts
	pop temp
	pop data_lo
ret

; Functions to move the car left, right, up and down if legal move

; Move the car left
moveLeft:
	cpi car_top, 0
	breq moveLBot		; If car isnt top, check bot

	cpi car_top, LINE_POS_1
	breq move_exit		;If car_top is at 0, we can't move left
	lsl car_top
	rjmp move_updateGame

moveLBot:
	cpi car_bot, LINE_POS_1
	breq move_exit
	lsl car_bot
	rjmp move_updateGame

; Move the car right
moveRight:
	cpi car_top, 0
	breq moveRBot		; If car isnt top, check bot

	cpi car_top, LINE_POS_8	
	breq move_exit		;If car_top is at 128, we can't move right
	lsr car_top
	rjmp move_updateGame

moveRBot:
	cpi car_bot, LINE_POS_8
	breq move_exit
	lsr car_bot
	rjmp move_updateGame

; Move the car up
moveUp:
	cpi car_top, 0
	brne move_exit		;If car_top is not 0, we can't move up
	mov car_top, car_bot
	clr car_bot
	rjmp move_updateGame

; Move the car down
moveDown:
	cpi car_bot, 0
	brne move_exit		;If car_bot is not 0, we can't move down
	mov car_bot, car_top
	clr car_top
	rjmp move_updateGame

move_updateGame:
	; We have just moved, check to see if we collided with obstacle or got a powerup or shield
	rcall carCrashed
	rcall shieldCollected
	rcall powerupCollected

	;Check which screen to display
	cpi lives, 0
	brne move_updateGame_print_game

	; Print gameover
	rcall lcd_init					; init lcd
	rcall printGameOverMessage		; print to display
	rjmp move_exit

	; Print game
	move_updateGame_print_game:
	rcall lcd_init					; init lcd
	rcall printGameView				; print to display

	move_exit:
ret

; Called to check if car has crashed with an obstacle and performs the appropriate action
carCrashed:
	;push conflicts
	push temp
	push data_lo

	; Check if player has now crashed with an obstacle
	cpi car_top, 0
	breq carCrashed_bot_check

	; player is at top
	mov temp, obs_top
	and temp, car_top

	cpi temp, 0	;check if zero
	breq carCrashed_end	; no crashes
	eor temp, obs_top	; exclusive or, remove obstacle but dont write yet
	mov data_lo, temp	; store this for now
	rjmp carCrashed_occured

	carCrashed_bot_check:
	mov temp, obs_bot
	and temp, car_bot

	cpi temp, 0	;check if zero
	breq carCrashed_end	; no crashes
	eor temp, obs_bot	; exclusive or, remove obstacle but dont write yet
	mov data_lo, temp	; store this for now

	carCrashed_occured:
	; Check if player has shield available
	lds temp, IS_PLAYER_SHIELDED
	cpi temp, 0
	breq carCrashed_occured_no_shield

	; We have shield, use it to destory obstacle
	clr temp
	sts IS_PLAYER_SHIELDED, temp	; remove shield

	cpi car_top, 0						; check if car top
	breq carCrashed_occured_shield_bot
	; Car is top, remove obstacle there
	mov obs_top, data_lo
	rjmp carCrashed_end				; continue game

	carCrashed_occured_shield_bot:		; check if car bot
	mov obs_bot, data_lo
	rjmp carCrashed_end				; continue game

	 carCrashed_occured_no_shield:
	; Crash has occured
	dec lives	; remove 1 car life

	; Check if out of lives
	cpi lives, 0
	brne carCrashed_restart
	
	; Out of lives now, freeze game
	clr temp
	out TIMSK, temp		; disable both timers

	; Disable INT1
	in temp, EIMSK                        ; taking the values inside the EIMSK  
	andi temp, (1<<INT0)				  ; and with INT0 to keep INT0 enabled
	out EIMSK, temp                       ; disable INT1

	;Game state updates, print new gameview
	rcall lcd_init					; init lcd
	rcall printGameView				; print to display

	; Display above while running motor
	rcall runMotor

	; Now display game over screen while frozen
	rcall lcd_init					; init lcd
	rcall printGameOverMessage

	; Delay to allow LCD to update
	ldi temp, 80
	REPEAT_DELAY low(50000), high(50000), temp	; delay macro

	rjmp carCrashed_end			; return

	carCrashed_restart:
	; Visually represent crash for short while by print new gameview
	rcall lcd_init					; init lcd
	rcall printGameView				; print to display
	
	; Then restart this level
	rcall restartCurrentLevel

	carCrashOccured_end:

	; Run motor at 70rpm
	rcall runMotor

	carCrashed_end:

	;pop conflicts
	pop data_lo
	pop temp
ret

; Run motor at specified speed for a specified duration using PWM
runMotor:
	;push conflicts
	push temp

	; Run motor at 70rpm
	ldi temp, MOTOR_SPEED
	out OCR0, temp

	; Delay to represent vibration both on LCD and with motor
	ldi temp, MOTOR_ON_TIME						; delay for MOTOR_ON_TIME (time motor will remain on)
	REPEAT_DELAY low(50000), high(50000), temp	; delay macro

	; Turn off motor
	clr temp
	out OCR0, temp

	;pop conflicts
	pop temp
ret

; Called to check if power up has been collected
; If so, powerup is removed and bonus points are added
powerupCollected:
	;push conflicts
	push temp

	; Check if player has now collected a powerup
	cpi car_top, 0
	breq powerupCollected_bot_check

	; player is at top

	; Store correct bits in temp
	mov temp, powerup
	andi temp, 0b11110000	; ignore 4 LSB, they are not for top
	lsr temp				; shift top powerups into 4 LSB positions
	lsr temp
	lsr temp
	lsr temp

	and temp, car_top		; check if car and powerup at same spot

	cpi temp, 0	; check if zero
	breq powerupCollected_end	; no powerups colliding
	eor temp, powerup	; exclusive or, if power up collected, powerup now removed
	lsl temp			; move top powerups back into 4 MSB positions
	lsl temp
	lsl temp
	lsl temp
	andi powerup, 0b00001111 ; clear top powerups
	and powerup, temp		 ; add powerups back

	rjmp powerupCollected_score

	powerupCollected_bot_check:
	mov temp, powerup

	andi temp, 0b00001111	; ignore 4 MSB, they are not for bot
	and temp, car_bot		; check if car and powerup at same spot

	cpi temp, 0	;check if zero
	breq powerupCollected_end	; no powerups colliding
	eor powerup, temp			; exclusive or, if power up collected, powerup now removed

	powerupCollected_score:
	; Add bonus points for powerup
	ldi temp, POWERUP_MULTIPLIER
	mul level, temp		; score added is 'level' * 10 points
	add score_lo, r0	; add low part
	adc score_hi, r1	; add high part with carry

	powerupCollected_end:

	;pop conflicts
	pop temp
ret

; Called to check if shield has been collected
; If so, shield is removed and shield buff is applied
shieldCollected:
	;push conflicts
	push temp

	; Check if player has now collected a shield
	cpi car_top, 0
	breq shieldCollected_bot_check

	; player is at top
	; Store correct bits in temp
	mov temp, shield
	andi temp, 0b11110000	; ignore 4 LSB, they are not for top
	lsr temp				; shift top shields into 4 LSB positions
	lsr temp
	lsr temp
	lsr temp

	and temp, car_top		; check if car and powerup at same spot

	cpi temp, 0	; check if zero
	breq shieldCollected_end	; no shields colliding
	eor temp, shield	; exclusive or, if shield power up collected, shield now removed
	lsl temp			; move top shields back into 4 MSB positions
	lsl temp
	lsl temp
	lsl temp
	andi shield, 0b00001111 ; clear top shields
	and shield, temp		 ; add shields back

	rjmp shieldCollected_add_buff

	shieldCollected_bot_check:
	mov temp, shield

	andi temp, 0b00001111	; ignore 4 MSB, they are not for bot
	and temp, car_bot		; check if car and shield powerup at same spot

	cpi temp, 0	;check if zero
	breq shieldCollected_end	; no shields colliding
	eor shield, temp			; exclusive or, if shield collected, shield now removed

	shieldCollected_add_buff:
	; Check if player already has shield
	lds temp, IS_PLAYER_SHIELDED
	cpi temp, 1
	breq shieldCollected_add_buff_add_points

	; Shield not collect, add shield buff
	ldi temp, 1
	sts IS_PLAYER_SHIELDED, temp
	rjmp shieldCollected_end

	shieldCollected_add_buff_add_points:

	; Shield already collected, collecting more shields gives you bonus points equal to 1 obstacle leaving playing area
	mov temp, level
	add score_lo, temp		; score added is 'level' points per extra shield obtained
	ldi temp, HIGH(0)
	adc	score_hi, temp		; add 0 to high

	shieldCollected_end:

	;pop conflicts
	pop temp
ret

; ******** Program Flow Functions ********

; Restarts the current level without restoring previous score
restartCurrentLevel:
	;push conflicts
	push temp

	; Clear all fields
	clr obs_top
	clr obs_bot
	clr powerup
	clr shield
	clr car_top
	clr car_bot

	;Set car to original position
	ldi car_top, LINE_POS_1

	; Reset all other timer counters in data memory except for variable time step for level
	clr temp
	sts Timer_Counter, temp
	sts Timer_Overflow_Count, temp

	sts Timer_Level_Counter, temp
	sts Timer_Level_Overflow_Count, temp
	sts Timer_Level_Sec_Count, temp

	;pop conflicts
	pop temp
ret

; Subroutine updates all variables required to prepare the next level of the game
gotoNextLevel:
	;push conflicts
	push r0
	push r1
	push temp
	push data_lo
	push data_hi
	push ZL
	push ZH

	; Increase level register
	inc level

	; Set up correct timestep phase for new level
	mov temp, level
	cpi temp, 9
	brlo gotoNextLevel_lower_9

	; if higher or equal to 9, we always want to go to 9th table address
	ldi temp, 9

	gotoNextLevel_lower_9:
	ldi data_lo, 2
	mul temp, data_lo	; multiply temp by 2 (as 2 instructions in table)
	mov temp, r0		; copy result to temp	

	; Lookup correct table location
	ldi ZL, LOW(TIME_STEP_TABLE)
	ldi ZH, HIGH(TIME_STEP_TABLE)

	clr data_lo

	add ZL, temp
	adc ZH, data_lo		;add 0 to ZH
	ijmp				;indirect jump to address pointed to by Z
	
	gotoNextLevel_return:
	sts Timer_Overflow_Per_Sec, temp		; set timer overflow based on selection in table

	; Restart this level with new level and time-step
	rcall restartCurrentLevel

	; Display new level initial screen
	rcall lcd_init					; init lcd
	rcall printGameView				; print to display

	; Delay for small duration so that initial screen is shown
	ldi temp, 12								; ~= 400ms delay
	REPEAT_DELAY low(65000), high(65000), temp	; delay macro

	;pop conflicts
	pop ZH
	pop ZL
	pop data_hi
	pop data_lo
	pop temp
	pop r1
	pop r0
ret

; Sets temp to a particular time step based on level
; Uses and alters temp and data_lo
; Must be accessed only via gotoNextLevel subroutine 
TIME_STEP_TABLE:
	nop									; level 0 (placeholder)
	nop
	ldi temp, TIME_STEP_LVL_1			; level 1
	rjmp gotoNextLevel_return
	ldi temp, TIME_STEP_LVL_2			; level 2
	rjmp gotoNextLevel_return
	ldi temp, TIME_STEP_LVL_3			; level 3
	rjmp gotoNextLevel_return
	ldi temp, TIME_STEP_LVL_4			; level 4
	rjmp gotoNextLevel_return
	ldi temp, TIME_STEP_LVL_5			; level 5
	rjmp gotoNextLevel_return
	ldi temp, TIME_STEP_LVL_6			; level 6
	rjmp gotoNextLevel_return
	ldi temp, TIME_STEP_LVL_7			; level 7
	rjmp gotoNextLevel_return
	ldi temp, TIME_STEP_LVL_8			; level 8
	rjmp gotoNextLevel_return

	; From here onwards (level 9+), half the time step
	lds temp, Timer_Overflow_Per_Sec	; read current timestep
	cpi temp, 1
	breq gotoNextLevel_return	; if timestep is 1, we will not half and keep it at 1

	; Otherwise, half timestep
	lsr temp	; lsr will divide the time step by 2
	rjmp gotoNextLevel_return	; return back to function

; ******** LED functions ********
; Subroutine updates LED score based on current score in: score_lo:score_hi
; Will display binary score from 0-1023 and then overflow for numbers after that
updateLEDScore:
	; Set PORTF bits, 0-3
	mov data_lo, score_lo
	ldi temp, 0b00001111	; mask for 4 LSB
	and data_lo, temp
	sts PORTF, data_lo

	; PORTE is bits 4-9
	; First set bits 4-7 (16-511)
	mov data_lo, score_lo
	ldi temp, 0b11110000	; mask for 4 MSB of low part
	and data_lo, temp

	; Bit reverse this result as we want to display bits as continuation from PORTF
	mov temp, data_lo
	rcall bitReverse

	lsl data_lo				; Bit shift left 4 times into MSB position
	lsl data_lo
	lsl data_lo
	lsl data_lo

	mov data_hi, data_lo	; Use data_hi to preserve data_lo's value

	; Now set bits 8 and 9 (512-1023)
	ldi temp, 0b00000011	; mask for 2 LSB of high part
	and temp, score_hi
	rcall bitReverse

	lsr data_lo				; Bit shift from bits 7-6 to 2-3
	lsr data_lo
	lsr data_lo
	lsr data_lo

	; Bits 1-2 are off LED and are ignored
	; Or is used to add bits 4-7 and 8-9
	or data_lo, data_hi
	out PORTE, data_lo	; print to LED
ret

; Subroutine takes register in temp and reverses bit positions in register, storing result in data_lo
; Uses and changed values of data_lo:data_hi and temp
bitReverse:
	; push conflicts
	push r20
	push r21

	clr data_lo ; clear data lo
	ldi r21, 0b10000000	;check from the MSB to LSB

	bitReverse_loop:
		mov r20, temp
		and r20, r21

		lsr data_lo	; bit shift left destination

		cpi r20, 0	; check if bit set in source
		breq bitReverse_not_set	; if not
		; if set
		subi data_lo, -128	;add current number of bits to data_lo

		bitReverse_not_set:

		lsr r21 ; bit shift right to check for next bit
		cpi r21, 0
		brne bitReverse_loop

	;pop conflicts
	pop r21
	pop r20
ret

; ******** Misc / Helper Functions ********

; Function delay: Pass a number in registers data_lo:data_hi to indicate how many microseconds
; must be delayed. Actual delay will be slightly greater (~1.08us*data_lo:data_hi).
; data_lo:data_hi are altered in this function.
delay:
	; Subtract 1 from data pair until they are 0
	delay_loop:
	subi data_lo, low(1)
	sbci data_hi, high(0)

	cpi data_lo, 0
	brne delay_loop
	cpi data_hi, 0
	brne delay_loop
ret

; Generate a random 8bit unsigned number and store result in data_lo register
generateRandomNum:
	rcall InitRandom
	rcall GetRandom
ret

; Stores a time seed based off timer 0 and our Timer_Level_Counter at RAND addresses
InitRandom:
push temp ; save conflict register

in temp, TCNT0 ; Create random seed from time of timer 0
sts RAND, temp
sts RAND+2, temp
lds temp, Timer_Level_Counter	; Also seed from interupt count for levels
sts RAND+1, temp
sts RAND+3, temp

pop temp ; restore conflict register
ret

; Generate a random number
; Requires time seed to have been made and stored in RAND addresses by InitRandom
GetRandom:
	; push conflict registers
	push r0
	push r1
	push r17
	push r18
	push r19
	push r20
	push r21
	push r22

	clr r22 ; remains zero throughout

	ldi data_lo, low(RAND_C) ; set original value to be equal to C
	ldi r17, BYTE2(RAND_C)
	ldi r18, BYTE3(RAND_C)
	ldi r19, BYTE4(RAND_C)

	; calculate A*X + C where X is previous random number.  A is 3 bytes.
	lds r20, RAND
	ldi r21, low(RAND_A)
	mul r20, r21 ; low byte of X * low byte of A
	add data_lo, r0
	adc r17, r1
	adc r18, r22

	ldi r21, byte2(RAND_A)
	mul r20, r21  ; low byte of X * middle byte of A
	add r17, r0
	adc r18, r1
	adc r19, r22

	ldi r21, byte3(RAND_A)
	mul r20, r21  ; low byte of X * high byte of A
	add r18, r0
	adc r19, r1

	lds r20, RAND+1
	ldi r21, low(RAND_A)
	mul r20, r21  ; byte 2 of X * low byte of A
	add r17, r0
	adc r18, r1
	adc r19, r22

	ldi r21, byte2(RAND_A)
	mul r20, r21  ; byte 2 of X * middle byte of A
	add r18, r0
	adc r19, r1

	ldi r21, byte3(RAND_A)
	mul r20, r21  ; byte 2 of X * high byte of A
	add r19, r0

	lds r20, RAND+2
	ldi r21, low(RAND_A)
	mul r20, r21  ; byte 3 of X * low byte of A
	add r18, r0
	adc r19, r1

	ldi r21, byte2(RAND_A)
	mul r20, r21  ; byte 2 of X * middle byte of A
	add r19, r0

	lds r20, RAND+3
	ldi r21, low(RAND_A)	
	mul r20, r21  ; byte 3 of X * low byte of A
	add r19, r0

	sts RAND, data_lo ; store random number
	sts RAND+1, r17
	sts RAND+2, r18
	sts RAND+3, r19

	mov data_lo, r19  ; prepare result (bits 30-23 of random number X)
	lsl r18
	rol data_lo

	; restore conflict registers
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r1
	pop r0
ret

; ******** Code Segment Constants ********
GAME_OVER_STRING: .db "GAME OVER!"
SCORE_STRING: .db "SCORE: "

INTRO_TOP_STRING: .db "G12 RACER V1.2"
INTRO_BOT_STRING: .db "PREPARE TO PLAY!"

; ******** Data Segment Storage ********
.dseg
.org 0x100 ; set SRAM address
RAND: .byte 4					; reserve byte for random number

; Counters for variable rate time step game updates
Timer_Counter: .byte 1			; reserve for 8bit unsigned int
Timer_Overflow_Count: .byte 1		; reserve for 8bit unsigned int
Timer_Overflow_Per_Sec: .byte 1	; reserve for 8bit unsigned int

; Counters for 30 second next level trigger
Timer_Level_Counter: .byte 1	; reserve for 8bit unsigned int
Timer_Level_Overflow_Count: .byte 1	; reserve for 8bit unsigned int
Timer_Level_Sec_Count: .byte 1	; reserve for 8bit unsigned int

; Counters for INT_1 second button press trigger
TIMER_INT1_Second_Press: .byte 1 ; reserve for int determining true/false
TIMER_INT1_Overflow_Count: .byte 1 ; reserve for 8bit unsigned int

; Shield powerup boolean
IS_PLAYER_SHIELDED: .byte 1 ; reserve for 'boolean', zero = false, non-zero = true

; Used to store keypad mask to check debounce keypad presses
MASK_STORE: .byte 1