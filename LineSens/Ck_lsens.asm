;   project CARkar
; subsystem linesensor
; Ck_lsens.asm
; acquire 8 pixels ~240 usec 		
; interface I2C address 0xA
; clock_config  1MHz intern    (use Window/PIC MemoryViews/Configbits)
;
; servo functions are omitted in this application
;
;
;
;
;*----------------------------		
;
;TMR0 not used
;TMR1 not used
 
		
    		LIST p=16F15323
		INCLUDE  "P16F15323.INC"

 __CONFIG _CONFIG1, _FEXTOSC_OFF & _RSTOSC_HFINTPLL & _CLKOUTEN_OFF & _CSWEN_OFF & _FCMEN_ON 		
 __CONFIG _CONFIG2, _MCLRE_OFF & _PWRTE_OFF & _LPBOREN_OFF & _BOREN_ON & _BORV_LO & _ZCD_OFF & _PPS1WAY_OFF & _STVREN_ON
 __CONFIG _CONFIG3, _WDTCPS_WDTCPS_31 & _WDTE_OFF & _WDTCWS_WDTCWS_7 & _WDTCCS_SC

    errorlevel	-302

#define I2C_AD	0xA	; i2c chip address
#define BUF	0x40	; buffer[0x40..0x47]
#define	_bank	BANKSEL
	
	
;--- symbols ---
DUT_1W	equ 0xAB    ; servo1 dutycycle wide   scannner in/out speed adjust
DUT_1N	equ 0x42    ; servo1 dutycycle narrow	 _1x  speed adjust
PWA_1	equ 0xE3    ; pulsewidth adjust servo1	
	
DUT_2W	equ 0x44    ; servo2 dutycycle wide   Z-axis  CCW speed adjust
DUT_2N	equ 0x18    ; 0x18 servo2 dutycycle narrow    CW speed adjust
PWA_2	equ 0xAB    ; pulsewidth adjust servo2

		
variables udata
TMP		res 1
IDX		res 1
c_SMP	res 1
b_I2C	res 1
	
	
;----- ISR -------	ref dspl.asm
	ORG 0x4
	_bank	PIR3
	bcf	PIR3,SSP1IF
	_bank	SSP1STAT
	btfsc	SSP1STAT,R_W	; ? remote r/w
	bra	is_02r		; => remote read
	btfss	SSP1STAT,D_A	; remote:write ?data /address
	bra	is_02n		; neglect
	_bank	SSP1BUF		; second SSP1IF load_buf -> slave:rcv
	movfw	SSP1BUF		
	_bank	SSP1CON1
	bsf	SSP1CON1,CKP
	_bank	0
	retfie

is_02r	nop
	_bank	0
	_bank	SSP1STAT
	btfsc	SSP1STAT,D_A	; remote:read ?data /address
	bra	is_02x
	_bank	0		
	movlw	0x3
	movwf	b_I2C		; <1> skip first byteread <0> start acquisition
	movlw	BUF		; address: reset buffer_pointer
	movwf	FSR1
	clrf	INDF1
	clrf	IDX
is_02x	nop
	_bank	0
	movfw	IDX		; in simulatie wel in firmware niet
	addwf	FSR1L,F		; IDX enige mogelijkheid
	movfw	INDF1
	_bank	SSP1BUF
	movwf	SSP1BUF
	_bank	SSP1CON1
	bsf	SSP1CON1,CKP
	_bank	0
	btfsc	b_I2C,1
	bra	$+3
	incf	IDX,F
	retfie
	bcf	b_I2C,1
	retfie
	
is_02n	nop			; done 
	_bank	SSP1BUF
	movfw	SSP1BUF	
	_bank	SSP1CON1
	bsf	SSP1CON1,CKP
	_bank	0
	retfie	
		
;----- code -----		

	ORG 0
	goto	initz
	

	ORG 0x40
	
intvar	clrf	IDX
	clrf	b_I2C
	bcf	LATA,2	; testpin
	bcf	LATA,4	; testpin
	_bank	INTCON
	bsf	INTCON,GIE
	_bank	0

codez	nop
	_bank	0
	btfss	b_I2C,0	
	bra	srvo
	bcf	LATA,0	    ; disable DC_restore
	movlw	BUF	    ; I2C priority
	movwf	FSR0L	
	movlw	0x8
	movwf	c_SMP
	movlw	0xC7	    ; b'1100 0111'
	andwf	LATC,F
	_bank	PIE3
	bcf	PIE3,SSP1IE	    ; MSSP1 interupt off
	_bank	0
acq1	bsf	LATC,2	    ; led ON
	_bank	ADCON0
	bsf	ADCON0,1    ; adc go
	btfsc	ADCON0,1
	bra	$-1
	_bank	0
	bcf	LATC,2	    ; led OFF
	_bank	PIE3
	bsf	PIE3,SSP1IE	    ; MSSP1 interupt on	
	_bank	ADRESH
	movfw	ADRESH
	_bank	0
	movwi	INDF0++
	movlw	0x8
	addwf	LATC,F	    ; next led
	decfsz	c_SMP,F
	bra	acq1
	bcf	b_I2C,0
	bsf	LATA,0	    ; DC_restore
	bra	codez
	

; TMR0 10 msec pulse repetition     TMR1 250 msec settling time
;------ servo control -------	

srvo	bra	codez

;    servo functions eliminated	

	
	
;============ initialize ==========
	ORG 0x200
initz	nop	    
	_bank	PPSLOCK
	movlw	0x55
	movwf	PPSLOCK
	movlw	0xAA
	movwf	PPSLOCK
	clrf	PPSLOCK
	_bank	RA4PPS
	movlw	0x15
	movwf	RC0PPS
	movlw	0x16
	movwf	RC1PPS
	_bank	SSP1CLKPPS
	bsf	SSP1CLKPPS,4	;INPUT page 187
	_bank	SSP1DATPPS
	movlw	b'00010001'
	iorwf   SSP1DATPPS,F	;INPUT page 187
	_bank	PPSLOCK
	movlw	0x55
	movwf	PPSLOCK
	movlw	0xAA
	movwf	PPSLOCK
	movlw	0x1
	movwf	PPSLOCK		; END pin swap
	_bank	0
	clrf	LATA
	movlw	b'11000011'	; LATC[3..5] mux RC<2> test
	andwf	TRISC,F
	movlw	b'11101010'	; (up to 10.02.25 b'11101011' RA0 output DC restore)  (obso RA2 servo1  RA4 servo2)
	andwf	TRISA,F
	_bank	ADCON1		; == analog setup
	movlw	b'01110000'	; left justified
	movwf	ADCON1
	_bank	ANSELA
	movlw	b'00100000'	;RA<5> analog
	movwf	ANSELA
	_bank	ADCON0
	movlw	b'00010101'	;analog mux RA5
	movwf	ADCON0	
	_bank	ANSELC
	clrf	ANSELC
	_bank	PIE3		    ;==== I2C interrupt ====
	bsf	PIE3,SSP1IE	    ; MSSP1 interupt on
	_bank	INTCON
	bsf	INTCON,PEIE	    ; Peripheral Interrupt Enable on
	_bank	SSP1CON1	    ;==== I2C ====
	movlw	b'00110110'	    ; slave  7 bitaddr
	movwf	SSP1CON1
	_bank	0
	movlw	I2C_AD		; i2c address
	movwf	TMP	
	bcf	STATUS,C
	rlf	TMP,F		; TMP holds i2c address
	movfw	TMP
	_bank	SSP1ADD
	movwf	SSP1ADD
	_bank	0
	movlw	BUF
	movwf	FSR0L
	movwf	FSR1L
	clrf	FSR0H
	clrf	FSR1H	
	movlw	8		; clear BUF[0x40..0x47]
	movwf	TMP
	clrw
	movwi	FSR0++
	decfsz	TMP,F
	bra	$-2
	movlw	BUF
	movwf	FSR0L
	goto	intvar	
	
	END