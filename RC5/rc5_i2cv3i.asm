; rc5_i2cv3i.asm
; interface I2C address 0xB
; clock_config  1MHz intern    (use Window/PIC MemoryViews/Configbits)
; 05-12-2020
; 17-12-2020
; 16-01-2021  20:30 rc5_i2c interrupt based
;---------------------------------------------------------		
;
;   PIC16F5313 VDD 5V		
;		
; RC5 serial -> PORTA4	'obsolate'
; b'00010110'  slave address 0xB   refer 'initz'		
;
; 26-10-22
; in udata i2c_add moet zijn i2c_adr
; 'call spi'  entry spi ontbreekt in 'xmit'.		
 
		
    		LIST p=16F15313
		INCLUDE  "P16F15313.INC"

	 __CONFIG _CONFIG1, _FEXTOSC_OFF & _RSTOSC_HFINTPLL & _CLKOUTEN_OFF & _CSWEN_OFF & _FCMEN_ON
	 __CONFIG _CONFIG2, _MCLRE_OFF & _PWRTE_OFF & _LPBOREN_OFF & _BOREN_ON & _BORV_LO & _ZCD_OFF & _PPS1WAY_OFF & _STVREN_ON
	 __CONFIG _CONFIG3, _WDTCPS_WDTCPS_31 & _WDTE_OFF & _WDTCWS_WDTCWS_7 & _WDTCCS_SC

		errorlevel	-302

#define I2C_AD	0xB	; i2c chip address
#define BCNT	0xD	; bitcount 13 bits
#define BUF	0x4C	; buffer[0x4C..0x4E]
#define	_bank	BANKSEL
		
variables udata
i2c_adr	res 1	    
TMP	res 1
SRI_L	res 1
SRI_H	res 1
b_PRCS	res 1
b_NEGL	res 1
c_BIT	res 1
M_SQ	res 1
b_EDGE	res 1
c_DIG	res 1
MM1	res 1
DUM	res 1
SDAT	res 1
XMTBY	res 1	    ; xmit busy
	
	
;---- symbols ----
PULSEW	equ 0x2F
PRC5	equ 0x10	; port RA5 sensor



	ORG 0
	call	initz
	bra	intvar
	
	ORG 0x4
	
;----- ISR -------
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
	movwf	DUM		; slave receive not used
	bcf	LATA,5		; end /request
	retfie
	
is_02r	btfsc	SSP1STAT,D_A	; remote:read ?data /address
	bra	is_02n		; neglect
	_bank	0		; first SSP1IF load_buf -> slave:xmt
	movfw	SDAT
	_bank	SSP1BUF
	movwf	SSP1BUF
	_bank	SSP1CON1
	bsf	SSP1CON1,CKP
	_bank	0
	bcf	LATA,5		; end /request	
	retfie
	
is_02n	nop			; done 
	_bank	SSP1BUF
	movfw	SSP1BUF	
	_bank	SSP1CON1
	bsf	SSP1CON1,CKP
	_bank	0
	bcf	XMTBY,0		; end xmit busy
	retfie		
	
	
	ORG 0x40
	
	
dec2bn	addwf	PCL,F
	retlw	0x00	; 0 x 10
	retlw	0x0A	; 1 x 10
	retlw	0x14	; 2 x 10
	retlw	0x1E	; 3 x 10
	retlw	0x28	; 4 x 10
	retlw	0x32	; 5 x 10
	retlw	0x3C	; 6 x 10
	retlw	0x46	; 7 x 10
	retlw	0x50	; 8 x 10
	retlw	0x5A	; 9 x 10
		
hun2bn	addwf	PCL,F
	retlw	0x00	; 0 x 100
	retlw	0x64	; 1 x 100
	retlw	0xC8	; 2 x 100
	
	

intvar	nop
	_bank	INTCON
	bsf	INTCON,GIE
	_bank	0
	
codez	movfw	PORTA
	andlw	PRC5
	subwf	b_EDGE,W
	btfss	STATUS,Z
	goto	cz_11
	btfsc	b_PRCS,7    ; ?process
	goto	prcss
	goto	codez	    ; no change
cz_11	movlw	PRC5	    ; port RC5 sensor	
	xorwf	b_EDGE,F
	movfw	M_SQ
	addwf	PCL,F
	goto	m_S1
	goto	m_S2
	
;--------- S1 ---------	
m_S1	nop
	_bank	TMR0L	    ;
	movlw	PULSEW
	movwf	TMR0L
	_bank	0
	clrf	SRI_H
	bsf	b_NEGL,0
	incf	M_SQ,F
	bra	codez		

;--------- S2 ---------	
m_S2	nop
	_bank	TMR0L
	btfsc	TMR0L,7		
	bra	m2s_01		; -- FULL period
	_bank	0
	btfss	b_NEGL,0	; ? flush  -- half period --
	bra	m2s_01		; process
	bcf	b_NEGL,0	; T
	_bank	TMR0L
	movlw	PULSEW
	movwf	TMR0L
	_bank	0
	bra	codez
m2s_01	nop
	_bank	0
	bsf	b_NEGL,0	; -- acquire --
	bcf	STATUS,C
	btfss	b_EDGE,4	; 
	bsf	STATUS,C
	rlf	SRI_L,F
	rlf	SRI_H,F
	_bank	TMR0L
	movlw	PULSEW
	movwf	TMR0L
	_bank	0
	decfsz	c_BIT,F
	bra	codez
	_bank	TMR0L
	movlw	PULSEW
	movwf	TMR0L
	_bank	0
	movlw	BCNT
	movwf	c_BIT	; restore for next acquisition
	clrf	M_SQ
	_bank	TMR1L
	clrf	TMR1H
	clrf	TMR1L
	btfss	TMR1H,6
	bra	$-1
	_bank	0
	movfw	PORTA		; bank0
	andlw	PRC5
	movwf	b_EDGE
	bsf	b_PRCS,7	; start processing
	bra	codez	
	
;---- process acuired data ---
prcss	bcf	b_PRCS,7
	btfss	b_PRCS,0	; ?first
	goto	pcs_01		; F
	movfw	SRI_H		; T copy SRI_H<3> => b_PRCS<3>
	andlw	0x8
	iorwf	b_PRCS,F
	bcf	b_PRCS,0
	goto	pcs_02		; process
pcs_01	movfw	SRI_H		; ?change
	andlw	0x8
	movwf	TMP
	movfw	b_PRCS
	andlw	0x8
	subwf	TMP,F
	btfsc	STATUS,Z
	goto	codez		; no change 
	movlw	0x8		; change
	xorwf	b_PRCS,F
pcs_02	movlw	0x2B		; colourbuttons
	subwf	SRI_L,W
	movwf	TMP
	movlw	0xFC
	addwf	TMP,F
	btfss	STATUS,C
	goto	pcs_03
	movlw	0x10
	subwf	SRI_L,W
	btfsc	STATUS,Z	; ?crunch and send 
	goto	pcs_05		; T
	movlw	0xFD
	addwf	c_DIG,W
	btfsc	STATUS,C
	goto	codez		; reject input
	movfw	SRI_L
	movwf	INDF0
	_bank	FSR0L
	incf	FSR0L,F
	_bank	0
	incf	c_DIG,F
	goto	codez
		
pcs_03	comf	TMP,F		; four colour codes
	movlw	0xFC
	addwf	TMP,W
	movwf	SRI_L
	bra	xmit
	goto	codez		; en colour codes
pcs_05	movf	c_DIG,F		; --- crunch and send
	btfsc	STATUS,Z	; ? buffer empty
	goto	codez		; T
	movfw	c_DIG
	movwf	MM1
	movlw	BUF			; test out of range input
	_bank	FSR0L
	movwf	FSR0L
	_bank	0
pcs_07	movfw	INDF0
	_bank	FSR0L
	incf	FSR0L,F
	_bank	0
	movwf	TMP
	movlw	0xF6
	addwf	TMP,W
	btfsc	STATUS,C
	goto	pcs_06		;	digits only
	decfsz	MM1,F
	goto	pcs_07
	decf	c_DIG,F		; T  n..0
	movfw	c_DIG
	addwf	PCL,F
	goto	ass_S1
	goto	ass_S2
	goto	ass_S3

ass_S1	movlw	BUF
	_bank	FSR0L	
	movwf	FSR0L
	_bank	0
	movfw	INDF0
	movwf	SRI_L
	bra	xmit
	goto	pcs_06

ass_S2	movlw	BUF
	_bank	FSR0L
	movwf	FSR0L
	_bank	0
	movfw	INDF0
	call	dec2bn
	movwf	SRI_L
	_bank	FSR0L
	incf	FSR0L,F
	_bank	0
	movfw	INDF0
	addwf	SRI_L,F
	bra	xmit
	goto	pcs_06

ass_S3	movlw	BUF
	_bank	FSR0L
	movwf	FSR0L
	_bank	0
	movfw	INDF0
	movwf	TMP
	movlw	0xFD
	addwf	TMP,W
	btfss	STATUS,C
	goto	ss3_01
	clrf	SRI_L		; out of range
	bra	xmit
	goto	pcs_06
ss3_01	movfw	INDF0
	call	hun2bn
	movwf	SRI_L
	_bank	FSR0L
	incf	FSR0L,F
	_bank	0
	movfw	INDF0
	call	dec2bn
	addwf	SRI_L,F
	_bank	FSR0L
	incf	FSR0L,F
	_bank	0
	movfw	INDF0
	addwf	SRI_L,F
	bra	xmit
	goto	pcs_06	
	
;--- serial output clockperiod 2 msec

xmit	call	i2ci
	goto	pcs_06


;---------- i2c ---------
i2ci	movfw	SRI_L
	movwf	SDAT
	_bank	PIR3
	bcf	PIR3,SSP1IF
	_bank	0
	bsf	LATA,5		; request
	bsf	XMTBY,0
	btfsc	XMTBY,0
	bra	$-1
	_bank	TMR1L
	movlw	0xFE
	movwf	TMR1H
	clrf	TMR1L
	movwf	TMR1L
	_bank	PIR4
	bcf	PIR4,TMR1IF	; 200 usec
	btfss	PIR4,TMR1IF
	bra	$-1
	_bank	0
	retlw	0
	
pcs_06	clrf	c_DIG
	movlw	BUF
	_bank	FSR0L
	movwf	FSR0L
	_bank	0
	goto	codez
	
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
	movwf	RA4PPS
	movlw	0x16
	movwf	RA2PPS
	_bank	SSP1CLKPPS
	bsf	SSP1CLKPPS,0
	_bank	SSP1DATPPS
	bsf	SSP1DATPPS,1
	_bank	PPSLOCK
	movlw	0x55
	movwf	PPSLOCK
	movlw	0xAA
	movwf	PPSLOCK
	movlw	0x1
	movwf	PPSLOCK
	_bank	0
	movlw	b'11011111'
	andwf	TRISA,F
	_bank	ODCONA
	bsf	ODCONA,5    ; REQ open drain
	_bank	SSP1CON1
	movlw	b'00110110'
	movwf	SSP1CON1
	_bank	SSP1CON3
	bsf	SSP1CON3,3  ; hold time no effect
	_bank	0
	movlw	I2C_AD		; i2c address
	movwf	i2c_adr
	bcf	STATUS,C
	rlf	i2c_adr
	movfw	i2c_adr
	_bank	SSP1ADD
	movwf	SSP1ADD		; store i2c address
	_bank	PIE3		    ;==== I2C interrupt ====
	bsf	PIE3,SSP1IE	    ; MSSP1 interupt on
	_bank	INTCON
	bsf	INTCON,PEIE	    ; Peripheral Interrupt Enable on
	_bank	0
	clrf	LATA
	_bank	ANSELA
	clrf	ANSELA
	_bank	T0CON0
	movlw	b'10001111'	    ; enable	div16
	movwf	T0CON0
	movlw	b'01000111'	    ; Fosc/4 div128
	movwf	T0CON1
	_bank	T1CON
	movlw	b'00110001'	    ; timer1
	movwf	T1CON
	movlw	b'00000001'	    ; timer1
	movwf	T1CLK		    ; 
	_bank	0	
	movlw	BUF
	_bank	FSR0L
	movwf	FSR0L
	clrf	FSR0H
	movlw	4		; clear buffer
	movwf	TMP		;
	clrf	INDF0		;
	incf	FSR0L,F		;
	decfsz	TMP,F		;
	bra	$-3		;
	movlw	BUF		; ?
	_bank	FSR0L		; ?
	movwf	FSR0L		; ?
	_bank	0
	movlw	0x1
	movwf	b_PRCS		; check new/repetition
	movwf	b_NEGL
	movlw	BCNT
	movwf	c_BIT
	clrf	c_DIG
	clrf	M_SQ
	bcf	LATA,5		; /request
	_bank	TMR0L
	clrf	TMR0L
	btfss	TMR0L,7
	bra	$-1
	_bank	0	
	movfw	PORTA
	andlw	PRC5		;0x10
	movwf	b_EDGE
	retlw	0	
	
		
	END