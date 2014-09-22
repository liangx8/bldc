;25 A
;26 B
;27 C

; ML4425 commutation sequence
;    AN BN CN AP BP CP
; S1 0  0  1  1  0  0
; S2 0  0  1  0  1  0
; S3 1  0  0  0  1  0
; S4 1  0  0  0  0  1
; S5 0  1  0  0  0  1
; S6 0  1  0  1  0  0


.equ	anFET = 7	; 11
.equ	bnFET = 4	; 2
.equ	cnFET = 1	; 31
.equ	apFET = 5	; 9
.equ	bpFET = 3	; 1
.equ	cpFET = 0	; 30





; feedback phase
.equ	fb_a	= 2			;pin 25	adc2
.equ	fb_b	= 3			;pin 26 adc3
.equ	fb_c	= 4			;pin 27 adc4

.equ	STATE_1 = (1<<cpFET)+(1<<anFET)
.equ	STATE_2 = (1<<cpFET)
.equ	STATE_3 = (1<<bpFET)+(1<<anFET)
.equ	STATE_4 = (1<<bpFET)
.equ	STATE_5 = (1<<bpFET)+(1<<cnFET)
.equ	STATE_6 = (1<<bpFET)
.equ	STATE_7 = (1<<apFET)+(1<<cnFET)
.equ	STATE_8 = (1<<apFET)
.equ	STATE_9 = (1<<apFET)+(1<<bnFET)
.equ	STATE_A = (1<<apFET)
.equ	STATE_B = (1<<cpFET)+(1<<bnFET)
.equ	STATE_C = (1<<cpFET)



.equ	SNDA1_STATE	= STATE_9
.equ	SNDA2_STATE	= STATE_7
.equ	SNDB1_STATE	= STATE_3
.equ	SNDB2_STATE	= STATE_5
.equ	SNDC1_STATE	= STATE_1
.equ	SNDC2_STATE	= STATE_B



.equ	SOUND_ON1	= STATE_1
.equ	SOUND_OFF1	= STATE_2
.equ	SOUND_ON2	= STATE_3
.equ	SOUND_OFF2	= STATE_4
.equ	BRAKE_ON	= (1<<anFET)+(1<<bnFET)+(1<<cnFET)
.equ	BRAKE_OFF	= 0

; this is a parameter when motor beep in some driven system.
; no PFET charge if P_CHARGE is 0 
.equ	P_CHARGE1	= 0


; PORTB
.equ	ha		= 2		; 14
.equ	hb		= 1		; 13
.equ	hc		= 0		; 12
.equ	red_led = 0	
.equ	green_led = 2

; vim: filetype=avr
