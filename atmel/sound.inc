.equ	SOUND_VOL =	 100	;SOUND_VOL/255
;.equ 	SOUND_CAP = 9000
.equ	SOUND_LEN = 300

.dseg				; DATA segment
mem_len1:	.byte 1
mem_len2:	.byte 1
mem_snd1:	.byte 1
mem_snd2:	.byte 1

.cseg

beep_c:
	ldi		r20,SOUND_ON1
	ldi		r21,SOUND_OFF1
	ldi		r16,18
	ldi		r17,14
	rjmp	beep_cycle
beep_b:
	ldi		r20,SOUND_ON1
	ldi		r21,SOUND_OFF1
	ldi		r16,24
	ldi		r17,10
	rjmp	beep_cycle
beep_a:
	ldi		r20,SOUND_ON1
	ldi		r21,SOUND_OFF1
	ldi		r16,30
	ldi		r17,8
	rjmp	beep_cycle

beep_cycle:
; P capacity charge
.if	P_CHARGE1>0
	ldi		r19,P_CHARGE1
	out		PORTD,r19
	ldi		r19,50
p_charge_1:
	dec		r19
	brne	p_charge_1
	out		PORTD,r19		; r19 = 0
.endif
beep_cycle_1:

	ldi		r27,40				; cycle times
beep_s:
	mov		r26,r16
beep_0:
	ldi		r18,200			; off period
beep_1:
	dec		r18				; 1 * 200 * temp1
	brne	beep_1				; 2 * 200 * temp1
	dec		r26					; 1 * temp1
	brne	beep_0				; 2 * temp1
	ldi		r26,70				; on period
	out		PORTD,r20
beep_2:
	dec		r26					; 1
	brne	beep_2				; 2
	out		PORTD,r21
;	out		PORTD,zero
	dec		r27
	brne	beep_s
	dec		r17
	brne	beep_cycle_1
	ret

short_delay:
	ldi		r17,255
d1:
	ldi		r16,255
d2:
	dec		r16
	brne	d2
	dec		r17
	brne	d1
	ret


set_state:
	cpi		r16,0
	brne	ss0
	ldi		r16,SNDA1_STATE
	rjmp	ss_end
ss0:
	cpi		r16,1
	brne	ss1
	ldi		r16,SNDA2_STATE
	rjmp	ss_end
ss1:
	cpi		r16,2
	brne	ss2
	ldi		r16,SNDB1_STATE
	rjmp	ss_end
ss2:
	cpi		r16,3
	brne	ss3
	ldi		r16,SNDB2_STATE
	rjmp	ss_end
ss3:
	cpi		r16,4
	brne	ss4
	ldi		r16,SNDC1_STATE
	rjmp	ss_end
ss4:
; expected r16 is 5 here.
;	cpi		r16,5
;	brne	ss_exit
	ldi		r16,SNDC2_STATE
ss_end:
	mov		state_on,r16
	clr		r16
	mov		state_off,r16
ss_exit:
; p fet charge
	sbrc	state_on,apFET
	ldi		r16,1<<anFET
	sbrc	state_on,bpFET
	ldi		r16,1<<bnFET
	sbrc	state_on,cpFET
	ldi		r16,1<<cnFET
	out		PORTD,r16
	rcall	sound_delay
	clr		r16
	out		PORTD,r16
	ret


sound_delay:

	push	r16
	ldi		r16,16
sd1:
	dec		r16
	brne	sd1
	pop		r16
	ret

sound_a:
	ldi		r16,1
	rcall	set_state
	ldi		r16,low(9000)
	sts		mem_snd1,r16
	ldi		r16,high(9000)
	sts		mem_snd2,r16
	rjmp	so_cycle

sound_b:
	ldi		r16,3
	rcall	set_state
	ldi		r16,low(8100)
	sts		mem_snd1,r16
	ldi		r16,high(8100)
	sts		mem_snd2,r16
	rjmp	so_cycle
sound_c:
	ldi		r16,5
	rcall	set_state
	ldi		r16,low(7300)
	sts		mem_snd1,r16
	ldi		r16,high(7300)
	sts		mem_snd2,r16
	rjmp	so_cycle

sound_d:
	ldi		r16,0
	rcall	set_state
	ldi		r16,low(6500)
	sts		mem_snd1,r16
	ldi		r16,high(6500)
	sts		mem_snd2,r16
	rjmp	so_cycle

sound_e:
	ldi		r16,2
	rcall	set_state
	ldi		r16,low(5900)
	sts		mem_snd1,r16
	ldi		r16,high(5900)
	sts		mem_snd2,r16
	rjmp	so_cycle

sound_f:
	ldi		r16,4
	rcall	set_state
	ldi		r16,low(5400)
	sts		mem_snd1,r16
	ldi		r16,high(5400)
	sts		mem_snd2,r16
	rjmp	so_cycle

sound_g:
	ldi		r16,4
	rcall	set_state
	ldi		r16,low(5000)
	sts		mem_snd1,r16
	ldi		r16,high(5000)
	sts		mem_snd2,r16

so_cycle:

	ldi		r16,low(SOUND_LEN)
	sts		mem_len1,r16
	ldi		r16,high(SOUND_LEN)
	sts		mem_len2,r16

;--------------------- all
soc_1:
	lds		r16,mem_snd1
	lds		r17,mem_snd2
	out		PORTD,state_off
;-------------------- off 
soc_2:
;	rcall	sound_delay
	dec		r16
	brne	soc_2
	and		r17,r17
	breq	soc_5
	dec		r17
	rjmp	soc_2
soc_5:
;-------------------- off end
	out		PORTD,state_on

	ldi		r16,SOUND_VOL

;--------------------- on
soc_3:
;	rcall	sound_delay
	dec		r16
	brne	soc_3
	lds		r16,mem_len1
	dec		r16
	sts		mem_len1,r16
	brne	soc_1

	lds		r16,mem_len2
	and		r16,r16
	breq	soc_4
	dec		r16
	sts		mem_len2,r16
	rjmp	soc_1
soc_4:
;---------------------- on end
;---------------------- all end
;	clr		r16
	clr		state_on
	out		PORTD,state_on
	ret
; vim: filetype=avr
