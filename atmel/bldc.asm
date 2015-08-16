; -*- mode: avr -*- 



.include "m8def.inc"

.include "sense.inc"



.equ		F_CPU		= 16
; ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
; Register Definitions
.def	zero			= r0		; stays at 0
.def	i_sreg			= r1		; SREG save in interrupts
.def	state_on		= r2		; motor output on
.def	state_off		= r3		; motor output off
.def	pwm_duty		= r4
.def	tcnt0_h			= r5		; t0 high byte, extend t0 to 16bit
.def	rcp_error_cnt	= r6		; rcp error count
.def	state_index		= r7		; current state
.def	edge_pwm		= r8		; useless
.def	edge_cnt		= r9		; useless
.def	rcp_l			= r10
.def	rcp_h			= r11
;.def	correct_zc		= r12

.def	i_temp1			= r20		; interrupt temporary
.def	i_temp2			= r21		; interrupt temporary
.def	i_temp3			= r22		; interrupt temporary
.def	i_temp4			= r23		; interrupt temporary

.def	flag1			= r25
	.equ	startup		= 0
; status of pwm
	.equ	stable		= 1
	.equ	rcp_done	= 2

	.equ	full_power	= 7


;	.equ	debug_flag	= 3
;	.equ	debug_zc_timeout = 4

;.equ	STARTUP_STEPS		= 12

;.equ	ZERO_CROSS			= 5
;.equ	ZC_TIMEOUT_LIMIT	= 3
;.equ	BRAKE_ENABLE		= 0



.equ	RCP_ERROR_COUNT			= 100
.equ	RCP_HIGH_X				= 20000 * (F_CPU/8)
.equ	RCP_LOW_X				= 7500 * (F_CPU/8)


;.equ	RCP_HIGH				= 14260 * (F_CPU/8)		; 1700 ns
.equ	RCP_HIGH				= 14000 * (F_CPU/8)		; 1700 ns
.equ	RCP_LOW					= 9227 * (F_CPU/8)		; 1100 ns
.equ	RCP_STOP_AREA			= 500 * (F_CPU/8)

;.equ	PWM_EDGE_STARTUP		= 10
.equ	PWM_EDGE				= 10

;.equ	CALCULATE_COM			= 1


.equ	STARTUP_PWM				= 20
.equ	STARTUP_PWM_LMT			= 120


.equ	RCP_MID					= (RCP_HIGH + RCP_LOW) / 2

;.equ	STARTUP_STATE_ELAPSED		= 3495 * (F_CPU/8)		;3495 ~= 3333ns  3000 rpm/ clk/8
;.equ	STARTUP_STATE_ELAPSED		= 1747 * (F_CPU/8)		;6000 rpm/ clk/8
;.equ	STARTUP_STATE_ELAPSED		= 5242 * (F_CPU/8)		;5242 ~= 5000ns  2000 rpm/ clk/8
;.equ	STARTUP_STATE_ELAPSED		= 10484 * (F_CPU/8)		;10484 ~= 10000ns  1000 rpm/ clk/8
;.equ	STARTUP_STATE_ELAPSED		= 20972 * (F_CPU/8)		;500 rpm/ clk/8
.equ	STARTUP_COMM		= 10000 * (F_CPU/8)

.macro	motor_stop
	andi	flag2,0b11111100
.endm
.equ		STOP	= 0
.macro	motor_forward
	sbr		flag2,0b00000001
	cbr		flag2,0b00000010
.endm
.equ		FORWARD	= 1
.macro	motor_brake
	ori		flag2,0b00000011
.endm
.equ		BRAKE	= 3
.macro	motor_reverse
	sbr		flag2,0b00000010
	cbr		flag2,0b00000001
.endm
.equ		REVERSE	= 2
.macro motor_dir
	ldi		@0,0b00000011
	and		@0,flag2
.endm

; 选择模拟比较器负端选择
; @0 register
; @1 immedate
.macro ac_select
; SFIOR [-|-|-|-|ACME|PUD|PSR2|PSR10]
; ACME - 1
	ldi		@0,0b00001000
	out		SFIOR,@0
; ADCSRA [ADEN|ADSC|ADFR|ADIF|ADIE|ADPS2|ADPS1|ADPS0]
	cbi		ADCSRA,ADEN
; ADMUX [REFS1|REFS0|ADLAR|-|MUX3|MUX2|MUX1|MUX0]
	ldi		@0,0b11100000+@1
	out		ADMUX,@0
.endm

;.macro current_tcnt2
;	mov		@1,tcnt2_h
;	in		@0,TCNT2
;	cpi		@0,15
;	brcc	ct2
;	mov		@1,tcnt2_h
;ct2:
;.endm

;.macro set_current_tcnt2
;	mov		@1,tcnt2_h
;	in		@0,TCNT2
;	cpi		@0,15
;	brcc	sct2
;	mov		@1,tcnt2_h
;sct2:
;	sts		mem_last_tcnt2,@0
;	sts		mem_last_tcnt2+1,@1
;.endm

.macro resume_t2

	ldi		@0,(0<<CS22)+(1<<CS21)+(0<<CS20)+(1<<WGM21)+(1<<WGM20) +(1<<COM21)+(0<<COM20)
	out		TCCR2,@0
.endm
.macro pause_t2
	ldi		@0,(1<<WGM21)+(1<<WGM20) +(1<<COM21)+(0<<COM20)
	out		TCCR2,@0
.endm
.macro all_output_off
	out		PORTD,zero
.endm

;**** **** **** **** ****
; ATmega8 interrupts

;.equ	INT0addr=$001	; External Interrupt0 Vector Address
;.equ	INT1addr=$002	; External Interrupt1 Vector Address
;.equ	OC2addr =$003	; Output Compare2 Interrupt Vector Address
;.equ	OVF2addr=$004	; Overflow2 Interrupt Vector Address
;.equ	ICP1addr=$005	; Input Capture1 Interrupt Vector Address
;.equ	OC1Aaddr=$006	; Output Compare1A Interrupt Vector Address
;.equ	OC1Baddr=$007	; Output Compare1B Interrupt Vector Address
;.equ	OVF1addr=$008	; Overflow1 Interrupt Vector Address
;.equ	OVF0addr=$009	; Overflow0 Interrupt Vector Address
;.equ	SPIaddr =$00a	; SPI Interrupt Vector Address
;.equ	URXCaddr=$00b	; USART Receive Complete Interrupt Vector Address
;.equ	UDREaddr=$00c	; USART Data Register Empty Interrupt Vector Address
;.equ	UTXCaddr=$00d	; USART Transmit Complete Interrupt Vector Address
;.equ	ADCCaddr=$00e	; ADC Interrupt Vector Address
;.equ	ERDYaddr=$00f	; EEPROM Interrupt Vector Address
;.equ	ACIaddr =$010	; Analog Comparator Interrupt Vector Address
;.equ	TWIaddr =$011	; Irq. vector address for Two-Wire Interface
;.equ	SPMaddr =$012	; SPM complete Interrupt Vector Address
;.equ	SPMRaddr =$012	; SPM complete Interrupt Vector Address
;-----bko-----------------------------------------------------------------
.dseg				; DATA segment

mem_t0_temp_l:	.byte	1	;t0 temp address
mem_t0_temp_h:	.byte	1
; for x_function used
mem_a_l:	.byte	1
mem_a_h:	.byte	1
mem_b_l:	.byte	1
mem_b_h:	.byte	1
mem_rcp_low:	.byte	2
mem_rcp_high:	.byte	2

mem_startup_cnt: .byte 1

; bugs tracing use
mem_reset_fg:	.byte 2


mem_comm: .byte 2
mem_timeout_cnt: .byte 1

;debug *******************************
debug_mem_seq: .byte 1
debug_mem_idx: .byte 1
debug_mem_data:	.byte 512
;debug end ***************************

mem_states:		.byte	12

;**** **** **** **** ****
.cseg
.org 0
;**** **** **** **** ****

;-----bko-----------------------------------------------------------------
; Reset and interrupt jump table
; When multiple interrupts are pending, the vectors are executed from top
; (ext_int0) to bottom.
	rjmp	reset
	rjmp	ext_int0
	nop		; ext_int1
	rjmp	t2oc_int
	rjmp	t2ovfl_int
	nop		; icp1_int
	nop		;t1oca_int
	nop		; t1ocb_int
	nop		;t1ovfl_int
	rjmp	t0ovfl_int
	nop		; spi_int
	nop		; urxc
	nop		; udre
	nop		; utxc
;------------------------------------------------------------------------------------
; state feedback change
sw_state:
	ldi		ZL,low (case_jmp)
	clr		ZH
	add		ZL,state_index
	ijmp
case_jmp:
	rjmp	state_fb_1
	rjmp	state_fb_2
	rjmp	state_fb_3
	rjmp	state_fb_4
	rjmp	state_fb_5
	rjmp	state_fb_6
state_fb_1:
state_fb_4:
	ac_select r17,fb_b
	rjmp	state_change
state_fb_2:
state_fb_5:
	ac_select r17,fb_c
	rjmp	state_change
state_fb_3:
state_fb_6:
	ac_select r17,fb_a

state_change:
	ldi		yl,low(mem_states)
	ldi		yh,high(mem_states)
	mov		r16,state_index
;*********************************
	sbrs	r16,0
	sbi		PORTB,2
	sbrc	r16,0
	cbi		PORTB,2
;*********************************
	add		r16,r16
	add		yl,r16
	adc		yh,zero
	ld		state_on,Y+
	ld		state_off,Y
	sbrc	flag1,full_power	; full power?
	rjmp	instant_full_pwm	; ... yes
	sbis	PORTB,PB3			; ... no
	out		PORTD,state_off
	sbic	PORTB,PB3
	out		PORTD,state_on
	resume_t2 r16
	ret
instant_full_pwm:
	pause_t2 r16
sch_1:
	out		PORTD,state_on
	ret
;------------------------------------------------------------------------------------

reset:
	clr	zero
	out	SREG, zero		; Clear interrupts and flags
	; Set up stack
	ldi	ZH, high(RAMEND)
	ldi	ZL, low (RAMEND)
	out	SPH, ZH
	out	SPL, ZL

; initialize PORTD
	ldi		r16,(1<<anFET)+(1<<apFET)+(1<<bnFET)+(1<<bpFET)+(1<<cnFET)+(1<<cpFET)
	out		DDRD,r16

; initialize PORTB
.ifdef green_led
	ldi		r16,(1<<red_led)+(1<<green_led)
	out		DDRB,r16
.endif
; state initial
	ldi		YL,low(mem_states)
	ldi		YH,high(mem_states)
	ldi		r16,STATE_1
	st		Y+,r16
	ldi		r16,STATE_2
	st		Y+,r16
	ldi		r16,STATE_3
	st		Y+,r16
	ldi		r16,STATE_4
	st		Y+,r16
	ldi		r16,STATE_5
	st		Y+,r16
	ldi		r16,STATE_6
	st		Y+,r16
	ldi		r16,STATE_7
	st		Y+,r16
	ldi		r16,STATE_8
	st		Y+,r16
	ldi		r16,STATE_9
	st		Y+,r16
	ldi		r16,STATE_A
	st		Y+,r16
	ldi		r16,STATE_B
	st		Y+,r16
	ldi		r16,STATE_C
	st		Y+,r16


; TIMER0 extern R6, CLK/1
; TCCR0[-|-|-|-|-|CS02|CS01|CS00]
; CS02:0=001 CLK/1
	ldi		r16, (1<<CS00)
	out		TCCR0,r16


; PWM setting
; TIMER2  CLK/8
; TCCR2[FOC2|WGM20|COM21|COM20|WGM21|CS22|CS21|CS20]
; CS22:0=010	CLK/8
; WGM2[1:0]=11 MODE 3 Fast PWM
;	ldi		r16,(0<<CS22)+(1<<CS21)+(0<<CS20)+(1<<WGM21)+(1<<WGM20)
;	out		TCCR2,r16

; TIMSK[OCIE2|TOIE2|TICIE1|OCIE1A|OCIE1B|TOIE1|-|TOIE0]
; TOIE0=1 T0 overflow interrupt enable
; TOIE1=1 T1 overflow interrupt enable
; TOIE2=1 T2 overflow interrupt enable
; OCIE1A=1 T1 output compare interrupt enable
	ldi		r16,(1<<OCIE2)+(1<<TOIE2)+(1<<TOIE0)
	out		TIMSK,r16

;	pause_t1 r16
	all_output_off
;	clr		state_on
;	clr		state_off

; 用于相位长度计算
; TCCR1A [COM1A1|COM1A0|COM1B1|COM1B0|FOC1A|FOC1B|WGM11|WGM10]
; TCCR1B [ICNC1 |ICES1 |-     |WGM13 |WGM12|CS12 |CS11 |CS10 ]
; COM1A1:0 = 00	
; WGM[4:0] = 0000 (Fast PWM, TOP = ICR1)
	clr		r16
	out		TCCR1A,r16
; clk/8
	ldi		r16,(0<<CS12)+(1<<CS11)+(0<<CS10)
	out		TCCR1B,r16

; int0 enable
; MCUCR [SE|SM2|SM1|SM0|ISC11|ISC10|ISC01|ISC00]
; ISC01:00 = 1:1 rising edge trigger interrupt
	ldi		r16,0b00000011
	out		MCUCR,r16
; GICR [INT1|INT0|-|-|-|-|IVSEL|IVCE]
; INT0 int0 enable
	ldi		r16,0b01000000
	out		GICR,r16


; comparator initializing
; ACSR [ACD|ACBG|ACO|ACI|ACIE|ACIC|ACIS1|ACIS0]
; ACD 1 disabled comparator
	out		ACSR,zero
	pause_t2 r16

	lds		r16,mem_reset_fg
	cpi		r16,0xff
	brne	reset_1
	lds		r16,mem_reset_fg+1
	cpi		r16,0xff
	brne	reset_1

; debug -------------------------------------------------------
;	clr		r16
;	clr		r17
;	rcall	to_ram
	
;	ldi		r16,2
;	sts		debug_mem_seq,r16
; debug end ---------------------------------------------------
	sei
	rjmp	stop_motor
reset_1:
	rcall	sound_a
	rcall	short_delay
	rcall	sound_b
	rcall	short_delay
	rcall	sound_c
	rcall	short_delay
	rcall	sound_d
	rcall	short_delay
	rcall	sound_e
	rcall	short_delay
	rcall	sound_f
	rcall	short_delay
	rcall	sound_g
	rcall	short_delay

.ifdef green_led
	sbi		PORTB,red_led
	rcall	short_delay
	cbi		PORTB,red_led
.endif
;	resume_t2 r16
	sei

;rjmp	test_fet

request_hi:
	rcall	rcp50
	sbrs	r16,0
	rjmp	request_hi
	sts		mem_rcp_high,rcp_l
	sts		mem_rcp_high+1,rcp_h
	cli
	rcall	sound_f
	sei
.ifdef green_led
	sbi		PORTB,green_led
	rcall	short_delay
	cbi		PORTB,green_led
.endif
request_lo:
	rcall	rcp50
	sbrc	r16,0
	rjmp	request_lo
	movw	r16,rcp_l

	ldi		r18,low(RCP_STOP_AREA)
	add		r16,r18
	ldi		r18,high(RCP_STOP_AREA)
	adc		r17,r18
	sts		mem_rcp_low,r16
	sts		mem_rcp_low+1,r17

	cli
	rcall	sound_f
	sei
.ifdef green_led
	sbi		PORTB,green_led
	rcall	short_delay
	cbi		PORTB,green_led
.endif
; debug --------------------------------------------------
	sts		debug_mem_seq,zero
; debug end ----------------------------------------------

	rcall	clear_data



;	resume_t2 r16
stop_motor:
	pause_t2 r16
	all_output_off
	ldi		r16,0x55
	sts		mem_reset_fg,r16
	sts		mem_reset_fg+1,r16

;	ldi		r16,20
;	rcall	brake_on_pwm
.ifdef green_led
	out		PORTB,zero
.endif
main_loop:
	rcall	wait_rcp_ready
	lds		r16,mem_rcp_low
	lds		r17,mem_rcp_low+1
	cp		r16,rcp_l
	cpc		r17,rcp_h
	brcc	main_loop


; debug --------------------------------------------------
	lds		r16,debug_mem_seq
	cpi		r16,2
	breq	xxx_end
	ldi		r16,1
;	sts		debug_mem_seq,r16
	sts		debug_mem_idx,zero

xxx_end:
	rcall	block_to_eep
; debug end ----------------------------------------------

; output full power commuation instead of pwm , when pwm duty is over 250, 
	cbr		flag1,1<<full_power
	ldi		r16,250
	mov		edge_pwm,r16
high_speed_timeout:
	rcall	no_brake
	rcall	glide_speed
	ser		r16
	sts		mem_reset_fg,r16
	sts		mem_reset_fg+1,r16
	ldi		r16,STARTUP_PWM
	mov		pwm_duty,r16
;static running
static_running:


	sbr		flag1,1<<startup
	cbr		flag1,1<<stable
;	rcall	open_loop
	rcall	one_loop

	sts		mem_timeout_cnt,zero
	sts		mem_startup_cnt,zero

	rjmp	wait_rest
state_start:
	sbrs	flag1,rcp_done
	rjmp	before_zc
	cbr		flag1,1<<rcp_done
	rcall	RCP2PWM
	and		r16,r16
	breq	stop_motor
	rjmp wait_rest
	sbrc	flag1,startup
	rcall	startup_accel
	sbrs	flag1,startup
	rcall	accel_pwm

	mov		pwm_duty,r16
ssbz_1:
	out		OCR2,pwm_duty
before_zc:
; delay for before zero cross
	in		r16,TIFR
	sbrs	r16,OCF1A
	rjmp	before_zc

;	rcall	scan_zero_cross


	rcall	scan_zc_new
; note: the function will return r18:r19 as timing of ZC, keep it
; until call calc_comm
; debug --------------------------------------------------
	movw	r16,r18
	rcall	data_save
; debug end ----------------------------------------------

	sts		mem_timeout_cnt,zero

;	sbrs	flag1,startup
	rcall	calc_comm
;	sbrc	flag1,startup
;	rcall	st_calc_comm

	sbrs	flag1,startup
	rjmp	set_comm
	lds		r16,mem_startup_cnt
	inc		r16
	sts		mem_startup_cnt,r16
	cpi		r16,60
	brcs	set_comm
	cbr		flag1,1<<startup
.ifdef green_led
	sbi		PORTB,red_led
.endif



set_comm:
	lds		r16,mem_comm
	lds		r17,mem_comm+1
	rcall	set_t1
;	rcall	study_speed
wait_rest:
	inc		state_index
	ldi		r16,6
	cp		state_index,r16
	brcs	wtr_1
	clr		state_index
wtr_1:
; debug --------------------------------------------------
sbis	ACSR,ACO
rjmp xdd_1
sbi		PORTB,0
rjmp xdd_2
xdd_1:
cbi		PORTB,0
xdd_2:
; debug end ----------------------------------------------
	in		r16,TIFR
	sbrs	r16,TOV1
	rjmp	wtr_1
	rcall	sw_state
; bellow is unneccesary
;	out		TCNT1L,zero
;	out		TCNT1H,zero

	rcall	scan225				; scan zc from 7.5 to 22.5
	rjmp	state_start

main_timeout:
; debug --------------------------------------------------
	ser		r16
	ser		r17
	rcall	data_save
;	sbrc	flag1,startup
;	rjmp	dbg_end
;	clr		r16
;	clr		r17
;	rcall	data_save
;	ldi		r16,2
;	sts		debug_mem_seq,r16
;dbg_end:
; debug end ----------------------------------------------

; Motor is blocked by something, go to start motor again
	sbrs	flag1,startup
	rjmp	high_speed_timeout
;	inc		pwm_duty
;	ldi		r16,STARTUP_PWM_LMT
;	cp		r16,pwm_duty
;	brcc	xxx
;	mov		pwm_duty,r16
;	sts		mem_startup_cnt,zero
;xxx:
.ifdef green_led
	sbi		PORTB,green_led
.endif

	lds		r16,mem_timeout_cnt
	inc		r16
	sts		mem_timeout_cnt,r16
	cpi		r16,12
	brcs	set_comm
	rjmp	static_running


one_loop:
	ldi		r16,high(STARTUP_COMM/2) ; 	setting 30 degree
	sts		mem_comm+1,r16

	clr		state_index
	rcall	sw_state
	lds		r16,mem_comm
	lds		r17,mem_comm+1
	; Do not scan zc at the first commutation
	lsl		r16
	rol		r17
	rcall	set_t1
;	ldi		r16,STARTUP_PWM
	out		OCR2,pwm_duty
;	mov		pwm_duty,r16
	resume_t2 r16
	ret
open_loop:
	ldi		r16,high(STARTUP_COMM) ; setting 30 degree
	sts		mem_comm+1,r16
	clr		state_index
	sts		mem_a_l,zero
	ldi		r16,STARTUP_PWM
	out		OCR2,r16
	resume_t2 r16
ol_1:
	rcall	sw_state
	lds		r16,mem_comm
	lds		r17,mem_comm+1
	rcall	set_t1
	lds		r16,mem_comm
	lds		r17,mem_comm+1
	movw	r18,r16
	lsr		r17
	ror		r16
	lsr		r17
	ror		r16
	lsr		r17
	ror		r16
	lsr		r17
	ror		r16
	sub		r18,r16
	sbc		r19,r17
	sts		mem_comm,r18
	sts		mem_comm+1,r19
ol_wt:
	in		r16,TIFR
	sbrs	r16,TOV1
	rjmp	ol_wt
	inc		state_index
	lds		r16,6
	cp		state_index,r16
	brne	ol_2
	clr		state_index
ol_2:
	
	
	lds		r16,mem_a_l
	inc		r16
	sts		mem_a_l,r16
	cpi		r16,24
	brcs	ol_1
	lds		r16,mem_comm
	lds		r17,mem_comm+1
	movw	r18,r16
	lsr		r18
	ror		r17
	sts		mem_comm,r17
	sts		mem_comm+1,r18
	rcall	set_t1

	ret
		;; motor start at gliding status
glide_speed:
; debug-----------------------------------------------------------
;	rcall	block_to_eep
; debug end-----------------------------------------------------------
	ac_select r16,fb_b
	ldi		r16,low(65536-30000)
	ldi		r17,high(65536-30000)
	out		TCNT1H,r17
	out		TCNT1L,r16
	ldi		r16,1<<TOV1
	out		TIFR,r16
; first rising edge
gs_1:
	in		r16,TIFR
	sbrc	r16,TOV1
	rjmp	motor_no_move
	sbic	ACSR,ACO			; 0
	rjmp	gs_1
gs_2:
	in		r16,TIFR
	sbrc	r16,TOV1
	rjmp	motor_no_move
	sbis	ACSR,ACO			; 1
	rjmp	gs_2
	in		yl,TCNT1L
	in		yh,TCNT1H
	mov		state_index,zero
gs_3:
	in		r16,TCNT1L
	in		r17,TCNT1H
	movw	r18,r16
	sub		r18,yl
	sbc		r19,yh
	cpi		r19,high(10000*F_CPU/8)
	brcc	motor_no_move

	mov		r18,state_index
	sbrc	r18,0
	rjmp	taggle_aco
	sbic	ACSR,ACO
	rjmp	gs_3
	rjmp	gs_4
taggle_aco:
	sbis	ACSR,ACO
	rjmp	gs_3
gs_4:
	movw	r18,yl
	movw	yl,r16
	sub		r16,r18
	sbc		r17,r19
	movw	r18,r16

	subi	r18,low(250*F_CPU/8)
	sbci	r19,high(250*F_CPU/8)
	brcs	glide_speed		; too small, try again


	inc		state_index
	ldi		r18,6
	cp		state_index,r18
	brcc	gs_5
	rjmp	gs_3
gs_5:
	sts		mem_a_l,r16
	sts		mem_a_h,r17
	rcall	divide
	ldi		r16,3			; 0 or 3
	mov		state_index,r16
	lds		r16,mem_b_l
	lds		r17,mem_b_h
	lsr		r17
	ror		r16
	sts		mem_comm,r16
	sts		mem_comm+1,r17
	rcall	set_t1
	pop		r16
	pop		r16
	cbr		flag1,1<<startup
	ldi		r16,STARTUP_PWM
	out		OCR2,r16
	rjmp	wait_rest
motor_no_move:
; debug --------------------------------------------------
;lds r16,debug_mem_seq
;and r16,r16
;brne dbg_end
;ldi r16,1
;sts debug_mem_seq,r16
;sts debug_mem_idx,zero
;dbg_end:
; debug end ----------------------------------------------
	ret
scan225:
; scan zc from 15 to 37.5  
	lds		r16,mem_comm
	lds		r17,mem_comm+1
	lsr		r17
	ror		r16
	; 开始扫描zc
	out		OCR1AH,r17
	out		OCR1AL,r16
	sbrs	flag1,startup
	rjmp	xscan_1
	lds		r16,mem_comm
	lds		r17,mem_comm+1
	lsl		r16
	rol		r17
	rjmp	xscan_exit
xscan_1:
	lds		r18,mem_comm
	add		r16,r18
	lds		r18,mem_comm+1
	adc		r17,r18
xscan_exit:
	out		OCR1BH,r17
	out		OCR1BL,r16

	ldi		r16,(1<<OCF1A)+(1<<OCF1B)+(1<<TOV1)
	out		TIFR,r16
	ret
scan15:
; scan zc from 15 to 30
	lds		r16,mem_comm
	lds		r17,mem_comm+1
	out		OCR1BH,r17
	out		OCR1BL,r16
	lsr		r17
	ror		r16
	out		OCR1AH,r17
	out		OCR1AL,r16
;	sts		mem_last_tcnt1,r16
;	sts		mem_last_tcnt1+1,r17
	ldi		r16,(1<<OCF1A)+(1<<OCF1B)+(1<<TOV1)
	out		TIFR,r16
	ret
; ********************************************************
; calculate next commutation
; ********************************************************
calc_comm:

	lds		r16,mem_comm
	lds		r17,mem_comm+1
	add		r18,r16
	adc		r19,r17
	lsr		r19
	ror		r18
	add		r18,r16
	adc		r19,r17
	lsr		r19
	ror		r18
	sts		mem_comm,r18
	sts		mem_comm+1,r19
	ret
zc_delay:
	ldi		r16,6
;zcd_1:
	dec		r16
	brne	PC-2
	ret

;;  deprecited, no longer uses
scan_zero_cross:
; detect pwm edge spark
	in		r17,TIFR
	sbrc	r17,OCF1B
	rjmp	zc_timeout
;	ldi		r18,PWM_EDGE_STARTUP
;	sbrs	flag1,startup
	ldi		r18,PWM_EDGE

; 
	in		r16,TCNT2
	cp		r16,r18
	brcs	scan_zero_cross
	in		r17,OCR2

	sub		r16,r17
	brcs	oszc_3
	
	cp		r16,r18
	brcs	scan_zero_cross

oszc_3:
	ldi		r16,10
	
oszc_2:
	in		r17,TIFR
	sbrc	r17,OCF1B
	rjmp	zc_timeout


	mov		r17,state_index
	sbrc	r17,0
	rjmp	oszc_high
	sbic	ACSR,ACO
	rjmp	scan_zero_cross
	dec		r16
	brne	oszc_2
	rjmp	oszc_x
oszc_high:
	sbis	ACSR,ACO
	rjmp	scan_zero_cross
	dec		r16
	brne	oszc_2

oszc_x:
; 
	in		r16,TCNT2
	cp		r16,r18
	brcs	scan_zero_cross
	in		r17,OCR2

	sub		r16,r17
	brcs	oszc_5
	
	cp		r16,r18
	brcs	scan_zero_cross

oszc_5:

	rjmp	zc_happen

	
	
scan_zc_new:
; 检测连续的过零状态
	in		r16,TIFR
	sbrc	r16,OCF1B
	rjmp	zc_timeout
szc_1:
	clr		r18
szc_2:
	mov		r17,state_index
	sbrc	r17,0
	rjmp	szc_high
szc_low:
	in		r16,TIFR
	sbrc	r16,OCF1B
	rjmp	zc_timeout

	sbic	ACSR,ACO
	rjmp	scan_zc_new
;*********************************
	sbis	PORTB,0
	rjmp	ddd_1
	cbi		PORTB,0
	rjmp	ddd_2
ddd_1:
	sbi		PORTB,0
ddd_2:
;*********************************
	sbrc	flag1,full_power
	rjmp	zc_happen

	inc		r18
	sbrs	flag1,startup
	inc		r18
	cpi		r18,20
	brcs	szc_low
	rjmp	zc_happen
szc_high:
	in		r16,TIFR
	sbrc	r16,OCF1B
	rjmp	zc_timeout

	sbis	ACSR,ACO
	rjmp	scan_zc_new
;*********************************
	sbis	PORTB,0
	rjmp	ddd_3
	cbi		PORTB,0
	rjmp	ddd_4
ddd_3:
	sbi		PORTB,0
ddd_4:
;*********************************

	sbrc	flag1,full_power
	rjmp	zc_happen

	inc		r18
	sbrs	flag1,startup
	inc		r18
	cpi		r18,20
	brcs	szc_high

zc_happen:
	in		r18,TCNT1L
	in		r19,TCNT1H
	ret
zc_timeout:
	pop		r16
	pop		r16
	rjmp	main_timeout


startup_accel:
	ldi		r17,10
	add		pwm_duty,r17
	cp		pwm_duty,r16
	brcs	check_limit
	mov		pwm_duty,r16
check_limit:
	ldi		r17,STARTUP_PWM_LMT
	cp		pwm_duty,r17
	brcs	sacc_1
	mov		pwm_duty,r17
sacc_1:
	mov		r16,pwm_duty
	ret
; =======================================================================================
; 包含了满功率模式的延时切换函数
; =======================================================================================
accel_pwm:
	sbrs	flag1,stable
	rjmp	evaluate_stable

	cp		r16,edge_pwm
	brcs	ap_5

	sbrs	flag1,full_power
	rjmp	enter_ful_pwr
	clr		edge_cnt
	ret
ap_5:
; 小于满功率边界
	sbrc	flag1,full_power
	rjmp	exit_ful_pwr
	clr		edge_cnt
	
	cp		r16,pwm_duty
	brcc	ap_2
	;减功率，直接输出
	mov		pwm_duty,r16
	ret
ap_2:
	; 加功率
	ldi		r17,20
	add		r17,pwm_duty
	brcs	ap_3
	cp		r16,r17
	brcs	ap_3
	mov		r16,r17
ap_3:
	mov		pwm_duty,r16
	ret
enter_ful_pwr:
	inc		edge_cnt
	ldi		r17,5
	cp		edge_cnt,r17
	brcs	efp_exit
	ldi		r16,236
	mov		pwm_duty,r16
	mov		edge_pwm,r16
	sbr		flag1,1<<full_power
	clr		edge_cnt
efp_exit:
	ret
exit_ful_pwr:
	inc		edge_cnt
	ldi		r17,5
	cp		edge_cnt,r17
	brcs	efp_exit
	cbr		flag1,1<<full_power
	ldi		r17,245
	mov		edge_pwm,r17
	mov		r16,r17
	mov		pwm_duty,r17
	clr		edge_cnt
	ret

evaluate_stable:
	cp		r16,pwm_duty
	brcs	es_exit
	inc		pwm_duty
	inc		pwm_duty
	inc		pwm_duty
	mov		r16,pwm_duty
	cpi		r16,STARTUP_PWM_LMT
	brcs	es_exit
	sbr		flag1,1<<stable
es_exit:
	ret
; end of accel_pwm ===============================================================================

set_t1:
	com		r16
	com		r17
	out		TCNT1H,r17
	out		TCNT1L,r16
	ldi		r16,(1<<OCF1A)+(1<<OCF1B)+(1<<TOV1)
	out		TIFR,r16
	ret
clear_data:
	clr		flag1
;	ldi		xl,low(mem_sdy_spd*2)
;	ldi		xh,low(mem_sdy_spd*2)
;	ldi		r16,26
;cd_1:
;	st		X+,zero
;	dec		r16
;	and		r16,r16
;	brne	cd_1
	ret

; r16.0 1 50个连续高
; r16.0 0 50个连续低
rcp50:
	ldi		r18,50
	mov		r19,r18
rcp50_1:
	rcall	wait_rcp_ready
	movw	r16,rcp_l
	subi	r16,low((RCP_HIGH+RCP_LOW)/2)
	sbci	r17,high((RCP_HIGH+RCP_LOW)/2)
	brcs	rcp50_2
	ldi		r19,50
	dec		r18
	breq	return_hi
	rjmp	rcp50_1
rcp50_2:
	ldi		r18,50
	dec		r19
	brne	rcp50_1
	clr		r16		; return_low
	ret
return_hi:
	ldi		r16,1
	ret
;	rjmp	PC
; interrupt routine ====================================================================
t0ovfl_int:
	in		i_sreg,SREG
	inc		tcnt0_h
	breq	t0ovfl_1
	rjmp	t0ovfl_exit
t0ovfl_1:
	dec		rcp_error_cnt
;	breq	lost_rcp
;	sbr		flag1,1<<tcnt0_h_ovf
t0ovfl_exit:
	out		SREG,i_sreg
	reti
lost_rcp:
	pop		r16
	pop		r16
	rjmp	reset
;------------------------------------------------------------------------------------
t2oc_int:
	out		PORTD,state_off
;	clt
;	cbi		PORTB,red_led
	reti
t2ovfl_int:
	out		PORTD,state_on
;	set
;	sbi		PORTB,red_led
	reti
;------------------------------------------------------------------------------------
;	RCP in uses extend INT0
ext_int0:
	in		i_sreg,SREG
	in		i_temp1,TCNT0
	mov		i_temp2,tcnt0_h

	in		i_temp3,TIFR
	sbrs	i_temp3,TOV0
	rjmp	int0_1
	in		i_temp1,TCNT0
	inc		i_temp2
int0_1:
	lds		i_temp3,mem_t0_temp_l
	lds		i_temp4,mem_t0_temp_h
	sts		mem_t0_temp_l,i_temp1
	sts		mem_t0_temp_h,i_temp2
	
	sub		i_temp1,i_temp3
	sbc		i_temp2,i_temp4

; RCP signal start at rising edge 
	in		i_temp3,MCUCR
	sbrs	i_temp3,ISC00
	rjmp	int0_2
	
	; triggered by falling edge at next time
	ldi		i_temp3,(1<<ISC01)
	out		MCUCR,i_temp3
	rjmp	int0_exit

int0_2:
	; RCP signal completed at falling edge

	ldi		i_temp3,(1<<ISC01)+(1<<ISC00)
	out		MCUCR,i_temp3

	movw	rcp_l,i_temp1
	; if(r20:r21 > RCP_LOW && r20:r21 < RCP_HIGH) goto int0_rcp_ready
	subi	i_temp1,low(RCP_LOW_X)
	sbci	i_temp2,high(RCP_LOW_X)
	subi	i_temp1,low(RCP_HIGH_X-RCP_LOW_X)
	sbci	i_temp2,high(RCP_HIGH_X-RCP_LOW_X)
	brcs	int0_rcp_ready
	; rcp signal is incorrect
	cbr		flag1,1<<rcp_done
	rjmp	int0_exit
int0_rcp_ready:
	ldi		i_temp3,RCP_ERROR_COUNT
	mov		rcp_error_cnt,i_temp3

	sbr		flag1,1<<rcp_done
int0_exit:
	out		SREG,i_sreg
	reti
; interrupt routine end ================================================================
.include "sound.inc"

; r18:r19 EEPROM address
; r16 value will be wroten to EEPROM
eep_write:
	; Wait for completion of previous write
	sbic	EECR,EEWE
	rjmp	eep_write
	; Set up address (r19:r18) in address register
	out		EEARH, r19
	out		EEARL, r18
	; Write data (r16) to Data Register
	out		EEDR,r16
	; Write logical one to EEMPE
	sbi		EECR,EEMWE
	; Start eeprom write by setting EEPE
	sbi		EECR,EEWE
	ret

; r18:r19 EEPROM address
; r16 return value
eep_read:
	; Wait for completion of previous write
	sbic	EECR,EEWE
	rjmp	eep_read
	; Set up address (r18:r19) in address register
	out		EEARH, r19
	out		EEARL, r18
	; Start eeprom read by writing EERE
	sbi		EECR,EERE
	; Read data from Data Register
	in		r16,EEDR
	ret
wait_rcp_ready:
	sbrs	flag1,rcp_done
	rjmp	wait_rcp_ready
	cbr		flag1,1<<rcp_done
	ret
; in mem_a
; out mem_b
divide:
	sts		mem_b_l,zero
	sts		mem_b_h,zero
	ldi		r19,16
	clr		r18
d_0:
	lds		r16,mem_a_l
	lds		r17,mem_a_h
	lsl		r16
	rol		r17
	sts		mem_a_l,r16
	sts		mem_a_h,r17
	rol		r18
	
	cpi		r18,3			; divided 3
	brcc	d_1
	rjmp	d_2
d_1:
	subi	r18,3			; divided 3
	lds		r16,mem_b_l
	ori		r16,0b00000001
	sts		mem_b_l,r16
d_2:
	dec		r19
	breq	d_exit

	lds		r16,mem_b_l
	lds		r17,mem_b_h
	lsl		r16
	rol		r17
	sts		mem_b_l,r16
	sts		mem_b_h,r17
	
	rjmp	d_0

d_exit:
	ret


RCP2PWM:
; M ~= 10000 
	lds		r16,mem_rcp_low
	Lds		r17,mem_rcp_low+1
	lds		r18,mem_rcp_high
	lds		r19,mem_rcp_high+1
	cp		rcp_l,r18
	cpc		rcp_h,r19
	brcs	rcp2pwm_1
	ser		r16
	ret
rcp2pwm_1:

	sub		r18,r16
	sbc		r19,r17

	Lsr		r19
	Ror		r18
	Lsr		r19
	Ror		r18
	Lsr		r19
	Ror		r18
	Lsr		r19
	Ror		r18
	Lsr		r19
	Ror		r18

.if F_CPU == 16
	Lsr		r19
	Ror		r18
.endif

	mov		r19,rcp_l
	sub		r19,r16
	mov		r16,r19
	mov		r19,rcp_h
	sbc		r19,r17
	mov		r17,r19
	brcs	return_zero

	Lsl		r16						; << 3
	Rol		r17
	Lsl		r16
	Rol		r17
.if F_CPU == 8
	Lsl		r16
	Rol		r17
.endif
	Ldi		r19, 9
Convert_RCP2PWM_Loop:
	Dec		r19
	Breq	CR2P_4
	Lsl		r16
	Rol		R17
	Brcs	CR2P_3
; C=0
	Cp		R17, r18				; H-M
	Brcs	Convert_RCP2PWM_Loop	; H-M<0 
CR2P_3:
; C=1
	Sub		r17, r18				; H=H-M
	Inc		r16						; L++
	Rjmp	Convert_RCP2PWM_Loop
CR2P_4:
;	Mov		r19, r16
	Ret
return_zero:
	clr		r16
	ret

		;;  For testing all fets work fine
test_fet:
	sbr		flag1,1<<full_power

tst_low:
.ifdef	red_led
	cbi		PORTB,red_led
	cbi		PORTB,green_led
.endif
	out		PORTD,zero
tstl_1:

	rcall	wait_rcp_ready
	movw	r16,rcp_l
	subi	r16,low((RCP_HIGH+RCP_LOW)/2)
	sbci	r17,high((RCP_HIGH+RCP_LOW)/2)
	brcc	tst_confirm
	rjmp	tstl_1
tst_confirm:
	rcall	rcp50
	sbrs	r16,0
	rjmp	tst_low
	inc		state_index
	mov		r16,state_index
	cpi		r16,6
	brcs	tsth_1
	clr		state_index
tsth_1:
	rcall	sw_state
.ifdef	red_led
	sbi		PORTB,red_led
.endif
tsth_2:
	rcall	wait_rcp_ready
	movw	r16,rcp_l
	subi	r16,low((RCP_HIGH+RCP_LOW)/2)
	sbci	r17,high((RCP_HIGH+RCP_LOW)/2)
	brcs	tst_confirm

	rjmp	tsth_2





.ifdef	red_led
check_point:
	sbis	PORTB,red_led
	rjmp	r_off
	rjmp	r_on
r_off:
	cbi		PORTB,red_led
	rjmp	r_e
r_on:
	sbi		PORTB,red_led
r_e:
	ret
.endif



brake_on_pwm:
	ldi		r17,BRAKE_ON
	mov		state_on,r17
	ldi		r17,BRAKE_OFF
	mov		state_off,r17
	out		OCR2,r16
	resume_t2 r16
	ret
no_brake:
	pause_t2 r16
	ldi		r16,BRAKE_OFF
	out		PORTD,r16
	ret
; debug -------------------------------------------------------
data_save:
	push	r16
	lds		r16,debug_mem_seq
	cpi		r16,1
	pop		r16
	brne	ds_exit
	rcall	to_ram

;	rjmp	save_at_timeout
;	rjmp	ds_exit

	lds		r16,debug_mem_idx
	and		r16,r16
	brne	ds_exit
	ldi		r16,2
	sts		debug_mem_seq,r16
ds_exit:
	ret

save_at_timeout:
	lds		r16,mem_timeout_cnt
	cpi		r16,10
	brcs	ds_exit
	clr		r16
	clr		r17
	rcall	to_ram

	ret

	ldi		r16,2
	sts		debug_mem_seq,r16
;	rjmp	block_to_eep
;	sbi		PORTB,red_led
	ret

to_ram:
	push	r16
	ldi		xl,low(debug_mem_data)
	ldi		xh,high(debug_mem_data)
	lds		r16,debug_mem_idx
	lsl		r16
	adc		xh,zero
	add		xl,r16
	adc		xh,zero
	lds		r16,debug_mem_idx
	inc		r16
	sts		debug_mem_idx,r16
	pop		r16
	st		X+,r16
	st		X,r17
	ret

block_to_eep:
	lds		r16,debug_mem_seq
	cpi		r16,2
	brne	bte_exit
	cli
	all_output_off
	clr		r18
	clr		r19
	sts		debug_mem_idx,r18
bte_1:
	ldi		xl,low(debug_mem_data)
	ldi		xh,high(debug_mem_data)
	lds		r17,debug_mem_idx
	lsl		r17
	brcc	bte_2
	inc		xh
	
bte_2:
	add		xl,r17
	adc		xh,zero
	ld		r16,X+
	rcall	eep_write
	ldi		r17,1
	add		r18,r17
	adc		r19,zero
	ld		r16,X
	rcall	eep_write
	add		r18,r17
	adc		r19,zero

	lds		r17,debug_mem_idx
	inc		r17
	sts		debug_mem_idx,r17
	brne	bte_1
	
	; write done
.ifdef	green_led
	sbi		PORTB,red_led
.endif
	rcall	sound_a
	rjmp	PC			; infinity loop
bte_exit:
	ret
; debug end ---------------------------------------------------
.exit


; vim: filetype=avr
