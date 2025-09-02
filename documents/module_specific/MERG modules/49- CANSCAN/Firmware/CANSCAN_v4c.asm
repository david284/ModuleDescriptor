;   	TITLE		"Code for switch scan  FLiM node for CBUS"
; filename CANSCAN_v4a_beta102.asm	 	13/04/20
; This code is for a PIC  18F25K80
; Uses  "..\cbuslib\evhndler_1c_25k80"
; uses  "..\cbuslib\cbusdefs8r"

;adapted from CANPAN.

; 

; Uses 16 MHz resonator and PLL for 64 MHz clock
; The setup timer is TMR0. Used during self enumeration.
; CAN bit rate of 125 Kbits/sec
; Standard frame only except for bootloader





;node number release frame <0x51><NN hi><NN lo>
;keep alive frame  <0x52><NN hi><NN lo>
;set learn mode <0x53><NN hi><NN lo>
;out of learn mode <0x54><NN hi><NN lo>
;clear all events <0x55><NN hi><NN lo>  Valid only if in learn mode
;read no. of events left <0x56><NN hi><NN lo>
;set event in learn mode  <0xD2><EN1><EN2><EN3><EN4><EVI><EV>  uses EV indexing
;The EV sent will overwrite the existing one
;read event in learn mode <0xB2><EN1><EN2><EN3><EN4><EVI>
;unset event in learn mode <0x95><EN1><EN2><EN3><EN4>
;reply to 0xB2. <0xD3><EN1><EN2><EN3><EN4><EVI><EV>
;Also sent if attempt to read / write too many EVs. Returns with EVI = 0 in that case

;read node parameters <0x10> Only works in setup mode. Sends string of 7 bytes as 
;<0xEF><para1><para2><para3><para4><para5><para6><para7>

;Implements ENUM and CANID OpCodes

;Teaches switch events 

;EV1	If SV = 1 is a switch event. (0 not used)
;		If 2 it is a SoD event only. 3 if self SoD.
;EV2	Switch number if used. Base 1.Top bit is set by firmware if a taught switch event.
;EV3	Switch action byte (SV)

;  EV3  Switch states for toggles etc  (128 switches). 
;		bit 0 is set for on/off  Default is set
;		bit 1 is polarity
;		bit 2 is on only (off only if pol set)
;		bit 3 is toggle on / off mode
;		bit 4 is send to self as well. 
;		bit 5 is short event
;		bit 6 is toggle state (used in RAM only)
;		bit 7 set for a learned event. Used in startup routine.
;		all zero is do nothing



;EV4	not used at present. May need it for startup options












;Rev v4a		Starting version


;Rev 4a Beta101 Based on CANPAN2
;				Extra startup with NV1. If NV1 = 2 then it sets to present switch positions. 
;				NV1 =3  is now all off instead of all on. Matches JMRI 'closed' for TOs.
;				Added a lot more comments.  Added 'busy' and Tx error routine in LPINT.
;				If NV1, top bit is set, never sends default events

;end of comments for CANSCAN. This is FLiM only at present but still needs putting into FLiM and giving a NN.


; This is the bootloader section

;*	Filename Boot2.asm  30/10/09

;*************************************************************** * * * * * * * * * * * * * * ;*
;*	CBUS bootloader

;*	Based on the Microchip botloader 'canio.asm' tho which full acknowledgement is made.
;*	Relevant information is contained in the Microchip Application note AN247

;*
;* Basic Operation:
;* The following is a CAN bootloader designed for PIC18F microcontrollers
;* with built-in CAN such as the PIC18F458. The bootloader is designed to
;* be simple, small, flexible, and portable.
;*
;
;
;*
;* Commands:
;* Put commands received from source (Master --> Slave)
;* The count (DLC) can vary.
;* XXXXXXXXXXX 0 0 8 XXXXXXXX XXXXXX00 ADDRL ADDRH ADDRU RESVD CTLBT SPCMD CPDTL CPDTH
;* XXXXXXXXXXX 0 0 8 XXXXXXXX XXXXXX01 DATA0 DATA1 DATA2 DATA3 DATA4 DATA5 DATA6 DATA7
;*


;*
;* ADDRL - Bits 0 to 7 of the memory pointer.
;* ADDRH - Bits 8 - 15 of the memory pointer.
;* ADDRU - Bits 16 - 23 of the memory pointer.
;* RESVD - Reserved for future use.
;* CTLBT - Control bits.
;* SPCMD - Special command.
;* CPDTL - Bits 0 - 7 of 2s complement checksum
;* CPDTH - Bits 8 - 15 of 2s complement checksum
;* DATAX - General data.
;*
;* Control bits:
;* MODE_WRT_UNLCK-Set this to allow write and erase operations to memory.
;* MODE_ERASE_ONLY-Set this to only erase Program Memory on a put command. Must be on 64-byte
;*	boundary.
;* MODE_AUTO_ERASE-Set this to automatically erase Program Memory while writing data.
;* MODE_AUTO_INC-Set this to automatically increment the pointer after writing.
;* MODE_ACK-Set this to generate an acknowledge after a 'put' (PG Mode only)
;*
;* Special Commands:
;* CMD_NOP			0x00	Do nothing
;* CMD_RESET		0x01	Issue a soft reset after setting last EEPROM data to 0x00
;* CMD_RST_CHKSM 	0x02	Reset the checksum counter and verify
;* CMD_CHK_RUN		0x03	Add checksum to special data, if verify and zero checksum
;* CMD_BOOT_TEST 	0x04	Just sends a message frame back to verify boot mode.

;*	Modified version of the Microchip code by M Bolton  30/10/09
;
;	The user program must have the folowing vectors

;	User code reset vector  0x0800
;	User code HPINT vector	0x0808
;	user code LPINT vector	0x0818

;	Checksum is 16 bit addition of all programmable bytes.
;	User sends 2s complement of addition at end of program in command 0x03 (16 bits only)

;**********************************************************************************



;	
; Assembly options
	LIST	P=18F25K80,r=hex,N=75,C=120,T=ON

	include		"p18f25K80.inc"
	include		"../cbuslib/cbusdefs8r.inc"

	
; Must define FLIM_ONLY for "evhndler_1c_25k80"




#define FLIM_ONLY
#define EVBLK_SZ	.16

BLK_SZ 		equ	EVBLK_SZ
EN_NUM		equ .128
EVENT_SZ	equ	BLK_SZ-8
ERASE_SZ	equ	.64
HASH_SZ 	equ .32
HASH_MASK	equ	HASH_SZ-1
EVSPER_BLK	equ ERASE_SZ/BLK_SZ

S_PORT 		equ	PORTA	;setup switch  Change as needed
S_BIT		equ	5		;so 18F25K80 can be used with CANACE3 PCB	


LED_PORT 	equ	PORTB  ;change as needed. 

;	


YELLOW		equ		6	;RB6 is the yellow LED on the PCB
GREEN		equ		7	;RB7 is the green LED on the PCB


;OPCs now defined in cbusdefs.



OLD_EN_NUM  equ	.32		;old number of allowed events
EN_NUM		equ	.128
EV_NUM  	equ 	.4		;number of allowed EVs per event



Modstat 	equ 1		;address in EEPROM

MAN_NO      equ MANU_MERG    ;manufacturer number
MAJOR_VER   equ 4
MINOR_VER   equ 43                  ; 43 is ascii C
MODULE_ID   equ MTYP_CANSCAN 		;for CANSCAN  id to identify this type of module
EVT_NUM     equ EN_NUM           	; Number of events
EVperEVT    equ EV_NUM           	; Event variables per event
NV_NUM      equ 1          			; Number of node variables
NODEFLGS    equ PF_CONSUMER + PF_PRODUCER + PF_BOOT
CPU_TYPE    equ P18F25K80
BETA_VER	equ	0				;for now





; definitions used by bootloader

#define	MODE_SELF_VERIFY	;Enable self verification of written data (undefine if not wanted)

#define	HIGH_INT_VECT	0x0808		;HP interrupt vector redirect. Change if target is different
#define	LOW_INT_VECT	0x0818		;LP interrupt vector redirect. Change if target is different.
#define	RESET_VECT	0x0800			;start of target
#define	CAN_CD_BIT	RXB0EIDL,0		;Received control / data select bit
#define	CAN_PG_BIT	RXB0EIDL,1		;Received PUT / GET bit
#define	CANTX_CD_BIT	TXB0EIDL,0	;Transmit control/data select bit
#define	CAN_TXB0SIDH	B'10000000'	;Transmitted ID for target node
#define	CAN_TXB0SIDL	B'00001000'
#define	CAN_TXB0EIDH	B'00000000'	;
#define	CAN_TXB0EIDL	B'00000100'
#define	CAN_RXF0SIDH	B'00000000'	;Receive filter for target node
#define	CAN_RXF0SIDL	B'00001000'
#define	CAN_RXF0EIDH	B'00000000'
#define	CAN_RXF0EIDL	B'00000111'
#define	CAN_RXM0SIDH	B'11111111'	;Receive masks for target node
#define	CAN_RXM0SIDL	B'11101011'
#define	CAN_RXM0EIDH	B'11111111'
#define	CAN_RXM0EIDL	B'11111000'
;#define	CAN_BRGCON1		B'00001111'	;CAN bit rate controls. 16 MHz resonator
#define	CANBIT_RATE		B'00001111'		;CAN bit rate while runnung. For 16 MHz clock
#define	CANBIT_BL		B'00000011'		;CAN bit rate for bootloader		
#define	CAN_BRGCON2		B'10011110'
#define	CAN_BRGCON3		B'00000011'
#define	CAN_CIOCON		B'00100000'	;CAN I/O control	
;	************************************************************ ** * * * * * * * * * * * * * * *
;	************************************************************ ** * * * * * * * * * * * * * * *
#ifndef	EEADRH		
#define	EEADRH	EEADR+ 1	
#endif			
#define	TRUE	1	
#define	FALSE	0	
#define	WREG1	PRODH	; Alternate working register
#define	WREG2	PRODL	
#define	MODE_WRT_UNLCK	_bootCtlBits, 0	; Unlock write and erase
#define	MODE_ERASE_ONLY	_bootCtlBits, 1	; Erase without write
#define	MODE_AUTO_ERASE	_bootCtlBits, 2	; Enable auto erase before write
#define	MODE_AUTO_INC	_bootCtlBits, 3	; Enable auto inc the address
#define	MODE_ACK		_bootCtlBits, 4	; Acknowledge mode
#define	ERR_VERIFY		_bootErrStat, 0	; Failed to verify if set
#define	CMD_NOP			0x00	
#define	CMD_RESET		0x01	
#define	CMD_RST_CHKSM	0x02	
#define	CMD_CHK_RUN		0x03
#define CMD_BOOT_TEST 	0x04
#define	CANTX			0x02			;port B RB2	



;set config registers. Note. CANSCAN uses Port B for the CAN. 



	CONFIG	FCMEN = OFF, FOSC = HS2, IESO = OFF, PLLCFG = OFF
	CONFIG	PWRTEN = ON, BOREN = SBORDIS, BORV=0, SOSCSEL = DIG
	CONFIG	WDTEN=OFF
	CONFIG	MCLRE = ON, CANMX = PORTB
	CONFIG	BBSIZ = BB1K 
	
	CONFIG	XINST = OFF,STVREN = ON,CP0 = OFF
	CONFIG	CP1 = OFF, CPB = OFF, CPD = OFF,WRT0 = OFF,WRT1 = OFF, WRTB = OFF
	CONFIG 	WRTC = OFF,WRTD = OFF, EBTR0 = OFF, EBTR1 = OFF, EBTRB = OFF
	


;	processor uses  16 MHz. Resonator with HSPLL to give a clock of 64MHz  (V4 onwards)

;********************************************************************************
;	RAM addresses used by boot. can also be used by application.

	CBLOCK 0
	_bootCtlMem
	_bootAddrL		; Address info
	_bootAddrH		
	_bootAddrU		
	_unused0		;(Reserved)
	_bootCtlBits	; Boot Mode Control bits
	_bootSpcCmd		; Special boot commands
	_bootChkL		; Chksum low byte fromPC
	_bootChkH		; Chksum hi byte from PC		
	_bootCount		
	_bootChksmL		; 16 bit checksum
	_bootChksmH		
	_bootErrStat	;Error Status flags
	ENDC
	
	; end of bootloader RAM


;****************************************************************
;	define RAM storage for CANSCAN. Not all may be used in v4c.
	
	CBLOCK	0		;file registers - access bank
					;interrupt stack for low priority
					;hpint uses fast stack
	W_tempL
	St_tempL
	Bsr_tempL
	PCH_tempH		;save PCH in hpint
	PCH_tempL		;save PCH in lpint
	Fsr_temp0L
	Fsr_temp0H 
	Fsr_temp1L
	Fsr_temp1H 
	Fsr_temp2L
	Fsr_temp2H 
	Fsr_scan1L
	Fsr_scan1H 
	Fsr_scan2L
	Fsr_scan2H 
	TempCANCON
	TempCANSTAT
	TempINTCON
	TempECAN
	Saved_Fsr0H
	Saved_Fsr0L
	Saved_Fsr1H
	Saved_Fsr1L
	CanID_tmp		;temp for CAN Node ID
	IDtemph			;used in ID shuffle
	IDtempl
	NN_temph		;node number in RAM
	NN_templ
	ENtemp1			;number of events
	NVtemp
	NVtemp1			;flag for no defaults
	Pb_temp

	
	IDcount			;used in self allocation of CAN ID.
	

	Datmode			;flag for data waiting 
	Count			;counter for loading
	Count1
	Count2
	Count3
	CountFb0		; used when reading/writing fb data from/to flash
	CountFb1		;  ditto
	Lcount2			;used in Lpint
	Lcount3
	Scount			;counter for startup sequence
	Keepcnt			;keep alive counter
	Latcount		;latency counter

	Roll			;used in CAN_ID

	Temp			;temps
	Temp1
	Temp_er			;error temp
	Dlc				;data length.  Needs to be set before sending TX1. 
	
	evaddrh			; event data ptr
	evaddrl
	prevadrh		; previous event data ptr
	prevadrl
	nextadrh		; next event data ptr
	nextadrl
	htaddrh			; current hash table ptr
	htaddrl
	htidx			; index of current hash table entry in EEPROM
	hnum			; actual hash number
	freadrh			; current free chain address
	freadrl
	initFlags
	Set_flg


	Switch1			;holds curent switch states 1 to 8
	Switch2			;switches 9 to 16
	Switch3			;switches 17 to 24
	Switch4			;switches 25 to 32
	Switch5			;switches 33 to 40
	Switch6			;switches 41 to 48
	Switch7			;switches 49 to 56
	Switch8			;switches 57 to 64
	Switch9			;switches 65 to 72
	Switch10		;switches 73 to 80
	Switch11		;switches 81 to 88
	Switch12		;switches 89 to 96
	Switch13		;switches 97 to 104
	Switch14		;switches 105 to 112
	Switch15		;switches 113 to 120
	Switch16		;switches 121 to 128

	Swtemp1			;temps for switches
	Swtemp2
	Swtemp3
	Swtemp4
	Swtemp5
	Swtemp6
	Swtemp7
	Swtemp8
	Swtemp9
	Swtemp10
	Swtemp11
	Swtemp12
	Swtemp13
	Swtemp14
	Swtemp15
	Swtemp16

	Switchno		;switch number
	Switch_t		;temp for set_loop
	Switch_s		;save for loop

	
	Swbit			;rolling bit for switch test
	Swtemp			;temp for switch scan
	Colcnt			;column count for switches
	Colsave
	Sflag			;flag byte for switches
	Scancnt			;counter for scan interval
	Svtemp			;temp for Sv. Used in set_sw.
	


	Tx1con			;Main transmit buffer. TX1 is used. 
	Tx1sidh
	Tx1sidl
	Tx1eidh
	Tx1eidl
	Tx1dlc
	Tx1d0
	Tx1d1
	Tx1d2
	Tx1d3
	Tx1d4
	Tx1d5
	Tx1d6
	Tx1d7
	
	Match		;match flag
	ENcount		;which EN matched
	ENcount1	;temp for count offset

	EVtemp		;holds current EV
	EVtemp1
	EVtemp2
	
	EVidx		; EV index from learn cmd
	EVdata		; EV data from cmd
	ENidx		; event index from commands which access events by index
	
	EVflags

	
	
	

	


	
	
	TkCount		; counts number of timer ticks
	T1count		; used in setup
	Debcnt		;debounce counter
	
	Rolle		;rolling bit for enum
	
	Fsr_tmp1Le	;temp store for FSR1
	Fsr_tmp1He 
	Enum0		;bits for new enum scheme.
	Enum1
	Enum2
	Enum3
	Enum4
	Enum5
	Enum6
	Enum7
	Enum8
	Enum9
	Enum10
	Enum11
	Enum12
	Enum13
	
	;add variables to suit

;data area to store data for event learning and event matching

	ev_opc	;event OpCode
	ev0		;event data from learn command and from received event (4 bytes )
	ev1
	ev2
	ev3
	Ss0		; storage for event used in self send
	Ss1
	Ss2
	Ss3
	Ss4



	flags
	
	setevt	; temp variable. holds next event number
	
	endc

; data area for storing event data while flash is being updated	
; Structure of each 16 bytes is as follows
; Bytes 0-3 - the event number
; Bytes 4-5 - the next event pointer
; Bytes 6-7 - the previous event pointer
; Bytes 8-11- the switch control bits


	CBLOCK 0x100		;bank 1
	; 64 bytes of event data - the quanta size for updating Flash
	; uses generic event handler
	; allows for 16 bytes total per event (8 EVs)


	evt00
	evt01
	evt02
	evt03
	next0h
	next0l
	prev0h
	prev0l
	ev00
	ev01
	ev02
	ev03
	ev04
	ev05
	ev06
	ev07

	evt10				; Event number - 4 bytes
	evt11
	evt12
	evt13
	next1h				; next entry in list
	next1l
	prev1h				; previous entry in list
	prev1l
	ev10				; event variables - upto 8
	ev11
	ev12
	ev13
	ev14
	ev15
	ev16
	ev17
	
	evt20				; Event number - 4 bytes
	evt21
	evt22
	evt23
	next2h				; next entry in list
	next2l
	prev2h				; previous entry in list
	prev2l
	ev20				; event variables - upto 8
	ev21
	ev22
	ev23
	ev24
	ev25
	ev26
	ev27
	
	evt30				; Event number - 4 bytes
	evt31
	evt32
	evt33
	next3h				; next entry in list
	next3l
	prev3h				; previous entry in list
	prev3l
	ev30				; event variables - upto 8
	ev31
	ev32
	ev33
	ev34
	ev35
	ev36
	ev37
	


	
	
	ENDC


	CBLOCK	0x180		;bank 1 upper half. Used to hold switch parameters
	
	sv0
	sv1
	sv2
	sv3
	sv4
	sv5
	sv6
	sv7
	sv8
	sv9
	sv10
	sv11
	sv12
	sv13
	sv14
	sv15
	sv16
	sv17
	sv18
	sv19
	sv20
	sv21
	sv22
	sv23
	sv24
	sv25
	sv26
	sv27
	sv28
	sv29
	sv30
	sv31
	sv32
	sv33
	sv34
	sv35
	sv36
	sv37
	sv38
	sv39
	sv40
	sv41
	sv42
	sv43
	sv44
	sv45
	sv46
	sv47
	sv48
	sv49
	sv50
	sv51
	sv52
	sv53
	sv54
	sv55
	sv56
	sv57
	sv58
	sv59
	sv60
	sv61
	sv62
	sv63
	sv64
	sv65
	sv66
	sv67
	sv68
	sv69
	sv70
	sv71
	sv72
	sv73
	sv74
	sv75
	sv76
	sv77
	sv78
	sv79
	sv80
	sv81
	sv82
	sv83
	sv84
	sv85
	sv86
	sv87
	sv88
	sv89
	sv90
	sv91
	sv92
	sv93
	sv94
	sv95
	sv96
	sv97
	sv98
	sv99
	sv100
	sv101
	sv102
	sv103
	sv104
	sv105
	sv106
	sv107
	sv108
	sv109
	sv110
	sv111
	sv112
	sv113
	sv114
	sv115
	sv116
	sv117
	sv118
	sv119
	sv120
	sv121
	sv122
	sv123
	sv124
	sv125
	sv126
	sv127

	ENDC
	
			
	CBLOCK	0x200		;bank 2 lower half
	EN1					;only used during copying to flash
						
	ENDC
	
	CBLOCK	0x280		;bank 2	upper half	
						;holds EVs 
	EV1					;only used during copying to flash
						
	ENDC


;****************************************************************
;	This is the bootloader
; ***************************************************************************** 
;_STARTUPCODE	0x00
	ORG 0x0000
; *****************************************************************************
	bra	_CANInit
	bra	_StartWrite
; ***************************************************************************** 
;_INTV_H CODE	0x08
	ORG 0x0008
; *****************************************************************************

	goto	HIGH_INT_VECT

; ***************************************************************************** 
;_INTV_L CODE	0x18
	ORG 0x0018
; *****************************************************************************

	goto	LOW_INT_VECT 

; ************************************************************** 
;	Bootloader Code start
; **************************************************************
	ORG 0x0020
;_CAN_IO_MODULE CODE
; ************************************************************ ** * * * * * * * * * * * * * * * 
; Function: VOID _StartWrite(WREG _eecon_data)
;PreCondition: Nothing
;Input: _eecon_data
;Output: Nothing. Self write timing started.
;Side Effects: EECON1 is corrupted; WREG is corrupted.
;Stack Requirements: 1 level.
;Overview: Unlock and start the write or erase sequence to protected
;	memory. Function will wait until write is finished.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_StartWrite
	movwf 	EECON1
	btfss 	MODE_WRT_UNLCK	; Stop if write locked
	return
	movlw 	0x55	; Unlock
	movwf 	 EECON2 
	movlw	 0xAA 
	movwf 	 EECON2
	bsf	 EECON1, WR	; Start the write
	nop
	btfsc 	EECON1, WR	; Wait (depends on mem type)
	bra	$ - 2
 	return
; ************************************************************ ** * * * * * * * * * * * * * * *

; Function: _bootChksm _UpdateChksum(WREG _bootChksmL)
;
; PreCondition: Nothing
; Input: _bootChksmL
; Output: _bootChksm. This is a static 16 bit value stored in the Access Bank.
; Side Effects: STATUS register is corrupted.
; Stack Requirements: 1 level.
; Overview: This function adds a byte to the current 16 bit checksum
;	count. WREG should contain the byte before being called.
;
;	The _bootChksm value is considered a part of the special
;	register set for bootloading. Thus it is not visible. ;
;*************************************************************** * * * * * * * * * * * *
_UpdateChksum:
	addwf	_bootChksmL,	F ; Keep a checksum
	btfsc	STATUS,	C
	incf	_bootChksmH,	F
	return
;************************************************************ ** * * * * * * * * * * * * * * *
;
;	Function:	VOID _CANInit(CAN,	BOOT)
;
;	PreCondition: Enter only after a reset has occurred.
; Input: CAN control information, bootloader control information ; Output: None.
; Side Effects: N/A. Only run immediately after reset.
; Stack Requirements: N/A
; Overview: This routine is technically not a function since it will not
;	return when called. It has been written in a linear form to
;	save space.Thus 'call' and 'return' instructions are not
;	included, but rather they are implied. ;
;	This routine tests the boot flags to determine if boot mode is
;	desired or normal operation is desired. If boot mode then the
;	routine initializes the CAN module defined by user input. It
;	also resets some registers associated to bootloading.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_CANInit:
	clrf	EECON1
	setf	EEADR	; Point to last location of EEDATA
	setf	EEADRH
	bsf	EECON1, RD	; Read the control code
	nop
	incfsz EEDATA, W

	goto	RESET_VECT


	clrf	_bootSpcCmd 	; Reset the special command register
	movlw 	0x1C		; Reset the boot control bits
	movwf 	_bootCtlBits 
	movlb	d'14'		; Set Bank 14 for K series
	bcf 	TRISB, CANTX 	; Set the TX pin to output 
	movlw 	CAN_RXF0SIDH 	; Set filter 0
	movwf 	RXF0SIDH
	movlw 	CAN_RXF0SIDL 
	movwf 	RXF0SIDL
	comf	WREG		; Prevent filter 1 from causing a receive event





	movwf	RXF1SIDL	;		
	movlw	CAN_RXF0EIDH	
	movwf	RXF0EIDH	
	movlw	CAN_RXF0EIDL	
	movwf	RXF0EIDL	
	movlw	CAN_RXM0SIDH	;	Set mask
	movwf	RXM0SIDH	
	movlw	CAN_RXM0SIDL	
	movwf	RXM0SIDL	
	movlw	CAN_RXM0EIDH	
	movwf	RXM0EIDH	
	movlw	CAN_RXM0EIDL	
	movwf	RXM0EIDL

	movlw	CANBIT_BL

	movwf	BRGCON1	
	movlw	CAN_BRGCON2	
	movwf	BRGCON2	
	movlw	CAN_BRGCON3	
	movwf	BRGCON3	
	movlb	.15
	clrf	ANCON0
	clrf	ANCON1
;	movlb	0
	movlw	CAN_CIOCON	;	Set IO
	movwf	CIOCON	
	
	clrf	CANCON	; Enter Normal mode

	bcf	TRISB,7
	bcf	TRISB,6
	bsf	LED_PORT,GREEN		;green LED on
	bsf	LED_PORT,YELLOW		;yellow LED on


; ************************************************************ ** * * * * * * * * * * * * * * * 
; This routine is essentially a polling loop that waits for a
; receive event from RXB0 of the CAN module. When data is
; received, FSR0 is set to point to the TX or RX buffer depending
; upon whether the request was a 'put' or a 'get'.
; ************************************************************ ** * * * * * * * * * * * * * * * 
_CANMain
	
	bcf	RXB0CON, RXFUL	; Clear the receive flag
_wait	clrwdt			; Clear WDT while waiting		
	btfss 	RXB0CON, RXFUL	; Wait for a message	
	bra	_wait



_CANMainJp1
	lfsr	0, RXB0D0
	movf	RXB0DLC, W 
	andlw 	0x0F
	movwf 	_bootCount 
	movwf 	WREG1
	bz	_CANMain 
_CANMainJp2				;?
	


; ************************************************************** * * * * * * * * * * * * * * * 
; Function: VOID _ReadWriteMemory()
;
; PreCondition:Enter only after _CANMain().
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: This routine is technically not a function since it will not
;	return when called. It has been written in a linear form to
;	save space.Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;This is the memory I/O engine. A total of eight data bytes are received and decoded. In addition two control bits are received, put/get and control/data.
;A pointer to the buffer is passed via FSR0 for reading or writing. 
;The control register set contains a pointer, some control bits and special command registers.
;Control
;<PG><CD><ADDRL><ADDRH><ADDRU><_RES_><CTLBT>< SPCMD><CPDTL><CPDTH>
;Data
;<PG>< CD>< DATA0>< DATA1>< DATA2>< DATA3>< DATA4>< DATA5>< DATA6>< DATA7>
;PG bit:	Put = 0, Get = 1
;CD bit:	Control = 0, Data = 1

; ************************************************************ ** * * * * * * * * * * * * * * *
_ReadWriteMemory:
	btfsc	CAN_CD_BIT	; Write/read data or control registers
	bra	_DataReg
; ************************************************************ ** * * * * * * * * * * * * * * * ; This routine reads or writes the bootloader control registers,
; then executes any immediate command received.
_ControlReg
	lfsr	1, _bootAddrL		;_bootCtlMem
_ControlRegLp1

	movff 	POSTINC0, POSTINC1 
	decfsz 	WREG1, F
	bra	_ControlRegLp1

; ********************************************************* 
; This is a no operation command.
	movf	_bootSpcCmd, W		; NOP Command
	bz	_CANMain
;	bz	_SpecialCmdJp2		; or send an acknowledge

; ********************************************************* 
; This is the reset command.
	xorlw 	CMD_RESET		; RESET Command 
	btfss 	STATUS, Z
	bra		_SpecialCmdJp4
	setf	EEADR		; Point to last location of EEDATA
	setf	EEADRH
	clrf	EEDATA		; and clear the data (at 0x3FF for now)
	movlw 	b'00000100'	; Setup for EEData
	rcall 	_StartWrite
	bcf		LED_PORT,YELLOW		;yellow LED off
	reset
; *********************************************************
; This is the Selfcheck reset command. This routine 
; resets the internal check registers, i.e. checksum and 
; self verify.
_SpecialCmdJp4
	movf	_bootSpcCmd, W 
	xorlw 	CMD_RST_CHKSM
	bnz		_SpecialCmdJp1
	clrf	_bootChksmH
	clrf	_bootChksmL
	bcf		ERR_VERIFY		
	clrf	_bootErrStat
	bra		_CANMain
; RESET_CHKSM Command
; Reset chksum
; Clear the error verify flag

;This is the Test and Run command. The checksum is
; verified, and the self-write verification bit is checked. 
; If both pass, then the boot flag is cleared.
_SpecialCmdJp1
	movf	_bootSpcCmd, W		; RUN_CHKSM Command
	xorlw 	CMD_CHK_RUN 
	bnz	_SpecialCmdJp3
	movf	_bootChkL, W	; Add the control byte
	addwf	 _bootChksmL, F
	bnz	_SpecialCmdJp2
	movf	_bootChkH, W 
	addwfc	_bootChksmH, F
	bnz	_SpecialCmdJp2
	btfsc 	ERR_VERIFY		; Look for verify errors
	bra	_SpecialCmdJp2

	bra		_CANSendOK	;send OK message


_SpecialCmdJp2

	bra	_CANSendNOK	; or send an error acknowledge


_SpecialCmdJp3
	movf	_bootSpcCmd, W		; RUN_CHKSM Command
	xorlw 	CMD_BOOT_TEST 
	bnz	_CANMain
	bra	_CANSendBoot

; ************************************************************** * * * * * * * * * * * * * * * 
; This is a jump routine to branch to the appropriate memory access function.
; The high byte of the 24-bit pointer is used to determine which memory to access. 
; All program memories (including Config and User IDs) are directly mapped. 
; EEDATA is remapped.
_DataReg
; *********************************************************
_SetPointers
	movf	_bootAddrU, W	; Copy upper pointer
	movwf 	TBLPTRU
	andlw 	0xF0	; Filter
	movwf 	WREG2
	movf	_bootAddrH, W	; Copy the high pointer
	movwf 	TBLPTRH
	movwf 	EEADRH
	movf	_bootAddrL, W	;   the low pointer
	movwf 	TBLPTRL
	movwf	 EEADR
	btfss 	MODE_AUTO_INC	; Adjust the pointer if auto inc is enabled
	bra	_SetPointersJp1
	movf	_bootCount, W	; add the count to the pointer
	addwf	 _bootAddrL, F 
	clrf	WREG
	addwfc	 _bootAddrH, F 
	addwfc	 _bootAddrU, F 

_SetPointersJp1			;?

_Decode
	movlw 	0x30
	cpfslt 	WREG2
	bra	_DecodeJp1



	bra	_PMEraseWrite

_DecodeJp1
	movf	WREG2,W
	xorlw 	0x30
	bnz	_DecodeJp2



	bra	_CFGWrite 
_DecodeJp2
	movf	WREG2,W 
	xorlw 0xF0
	bnz	_CANMain
	bra	_EEWrite

	

; Program memory < 0x300000
; Config memory = 0x300000
; EEPROM data = 0xF00000
	
; ************************************************************ ** * 
; ************************************************************** * 
; Function: VOID _PMRead()
;	VOID _PMEraseWrite ()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of
; the source data.
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space.Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;These are the program memory read/write functions. Erase is available through control flags. An automatic erase option is also available.
; A write lock indicator is in place to ensure intentional write operations.
;Note: write operations must be on 8-byte boundaries and must be 8 bytes long. Also erase operations can only occur on 64-byte boundaries.
; ************************************************************ ** * * * * * * * * * * * * * * *



_PMEraseWrite:
	btfss 	MODE_AUTO_ERASE
	bra	_PMWrite
_PMErase:
	movf	TBLPTRL, W
	andlw	b'00111111'
	bnz	_PMWrite
_PMEraseJp1
	movlw	b'10010100' 
	rcall 	_StartWrite 
_PMWrite:
	btfsc 	MODE_ERASE_ONLY


	bra	_CANMain 

	movf	TBLPTRL, W
	andlw	b'00000111'
	bnz	_CANMain 
	movlw 	0x08
	movwf WREG1

_PMWriteLp1					; Load the holding registers
	movf	POSTINC0, W 
	movwf 	TABLAT
	rcall	 _UpdateChksum 	; Adjust the checksum
	tblwt*+
	decfsz	 WREG1, F
	bra	_PMWriteLp1

#ifdef MODE_SELF_VERIFY 
	movlw	 0x08
	movwf 	WREG1 
_PMWriteLp2
	tblrd*-			; Point back into the block
	movf	POSTDEC0, W 
	decfsz	 WREG1, F
	bra	_PMWriteLp2
	movlw	 b'10000100' 	; Setup writes
	rcall	_StartWrite 	; Write the data
	movlw 	0x08
	movwf 	WREG1
_PMReadBackLp1
	tblrd*+			; Test the data
	movf	TABLAT, W 
	xorwf 	POSTINC0, W
	btfss	STATUS, Z
	bsf	ERR_VERIFY 
	decfsz 	WREG1, F
	bra	_PMReadBackLp1	; Not finished then repeat
#else
	tblrd*-			; Point back into the block
				 ; Setup writes
	movlw 	b'10000100' 	; Write the data
	rcall 	_StartWrite 	; Return the pointer position
	tblrd*+
#endif

	bra	_CANMain


; ************************************************************** * * * * * * * * * * * * * * *
 ; Function: VOID _CFGWrite()
;	VOID _CFGRead()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of the source data. 
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space. Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;
;	These are the Config memory read/write functions. Read is
;	actually the same for standard program memory, so any read
;	request is passed directly to _PMRead.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_CFGWrite

#ifdef MODE_SELF_VERIFY		; Write to config area
	movf	INDF0, W		; Load data
#else
	movf	POSTINC0, W
#endif
	movwf 	TABLAT
	rcall 	_UpdateChksum	; Adjust the checksum
	tblwt*			; Write the data
	movlw	b'11000100' 
	rcall 	_StartWrite
	tblrd*+			; Move the pointers and verify
#ifdef MODE_SELF_VERIFY 
	movf	TABLAT, W 
	xorwf 	POSTINC0, W

#endif
	decfsz 	WREG1, F
	bra	_CFGWrite	; Not finished then repeat

	bra	_CANMain 



; ************************************************************** * * * * * * * * * * * * * * * 
; Function: VOID _EERead()
;	VOID _EEWrite()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of
 ;	the source data.
; Input:	None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space. Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;
;	This is the EEDATA memory read/write functions.
;
; ************************************************************ ** * * * * * * * * * * * * * * *


_EEWrite:

#ifdef MODE_SELF_VERIFY
	movf	INDF0, W
#else
	movf	POSTINC0, W 
#endif

	movwf 	EEDATA
	rcall 	_UpdateChksum 
	movlw	b'00000100' 
	rcall	 _StartWrite

#ifdef MODE_SELF_VERIFY 
	clrf	EECON1
	bsf	EECON1, RD
	nop
	movf	EEDATA, W 
	xorwf 	POSTINC0, W
	btfss	STATUS, Z
	bsf	ERR_VERIFY
#endif

	infsnz	 EEADR, F 
	incf 	EEADRH, F 
	decfsz 	WREG1, F
	bra	_EEWrite


	bra	_CANMain 
	

; Read the data

; Adjust EEDATA pointer
; Not finished then repeat
; Load data
; Adjust the checksum 
; Setup for EEData
; and write
; Read back the data ; verify the data ; and adjust pointer
; Adjust EEDATA pointer
; Not finished then repeat

; ************************************************************** * * * * * * * * * * * * * * *
; Function: VOID _CANSendAck()
;	VOID _CANSendResponce ()
;
; PreCondition:TXB0 must be preloaded with the data.
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space. Thus 'call' and 'return' instructions are not
;	included, but rather they are implied. ;
;	These routines are used for 'talking back' to the source. The
;	_CANSendAck routine sends an empty message to indicate
;	acknowledgement of a memory write operation. The
;	_CANSendResponce is used to send data back to the source. ;
; ************************************************************ ** * * * * * * * * * * * * * * *



_CANSendMessage
	btfsc 	TXB0CON,TXREQ 
	bra	$ - 2
	movlw 	CAN_TXB0SIDH 
	movwf 	TXB0SIDH
	movlw 	CAN_TXB0SIDL 
	movwf 	TXB0SIDL
	movlw 	CAN_TXB0EIDH 
	movwf 	TXB0EIDH	

	movlw	CAN_TXB0EIDL
	movwf	TXB0EIDL
	bsf	CANTX_CD_BIT
	btfss	CAN_CD_BIT 
	bcf	CANTX_CD_BIT
	bsf	TXB0CON, TXREQ
    	bra	 _CANMain	; Setup the command bit

_CANSendOK				;send OK message 
	movlw	1			;a 1 is OK
	movwf	TXB0D0
	movwf	TXB0DLC
	bra		_CANSendMessage
	
_CANSendNOK				;send not OK message
	clrf	TXB0D0		;a 0 is not OK
	movlw	1
	movwf	TXB0DLC
	bra		_CANSendMessage

_CANSendBoot
	movlw	2			;2 is confirm boot mode
	movwf	TXB0D0
	movlw	1
	movwf	TXB0DLC
	bra		_CANSendMessage
    
; Start the transmission

; 	End of bootloader

;************************************************************************************************************
;
;		start of  CANPAN program code

		ORG		0800h
loadadr
		nop						;for debug
		goto	setup

		ORG		0808h
		goto	hpint			;high priority interrupt
		
		ORG		0810h			;node type parameters
myName	db	"SCAN   "

		ORG		0818h	
		goto	lpint			;low priority interrupt
		
		org		0820h

nodeprm     db  MAN_NO, MINOR_VER, MODULE_ID, EVT_NUM, EVperEVT, NV_NUM 
			db	MAJOR_VER,NODEFLGS,CPU_TYPE,PB_CAN    ; Main parameters
            dw  RESET_VECT     ; Load address for module code above bootloader
            dw  0           ; Top 2 bytes of 32 bit address not used
			dw	0			;CPU Maufacturers ID low
			dw	0			;CPU manufacturers ID hi
			db	CPUM_MICROCHIP,BETA_VER

sparprm     fill 0,prmcnt-$ ; Unused parameter space set to zero

PRMCOUNT    equ sparprm-nodeprm ; Number of parameter bytes implemented

             ORG 0838h

prmcnt      dw  PRMCOUNT    ; Number of parameters implemented
nodenam     dw  myName      ; Pointer to module type name
            dw  0 			; Top 2 bytes of 32 bit address not used


PRCKSUM     equ MAN_NO+MINOR_VER+MODULE_ID+EVT_NUM+EVperEVT+NV_NUM+MAJOR_VER+NODEFLGS+CPU_TYPE+PB_CAN+HIGH myName+LOW myName+HIGH loadadr+LOW loadadr+PRMCOUNT+CPUM_MICROCHIP+BETA_VER

cksum       dw  PRCKSUM     ; Checksum of parameters


;*******************************************************************

		ORG	0x0840
;	

;	
;		high priority interrupt. Not used

hpint	
		retfie	1

		

		
	


	
	


;**************************************************************
;
;		low priority interrupt. Used for CAN transmit error / latency and 'busy' frame.
;		Busy frame is a max priority, zero data frame, preloaded in TXB0.
;		Latency count (number of tries to transmit) is preset in code to .10
	
				ORG 0A00h

lpint	movwf	W_tempL					;save critical variables
		movff	STATUS,St_tempL
		movff	CANCON,TempCANCON
		movff	CANSTAT,TempCANSTAT
	
		movff	FSR0L,Fsr_temp0L		;save FSR0
		movff	FSR0H,Fsr_temp0H
		movff	FSR1L,Fsr_temp1L		;save FSR1
		movff	FSR1H,Fsr_temp1H

		btfsc	PIR5,ERRIF
		bra		txerr				;transmit error?
		btfss	PIR5,FIFOWMIF		;FIFO error?
		bra		no_fifo
		bcf		PIR5,FIFOWMIF		;clear FIFO flag
		bcf		PIR5,TXB0IF			;clear busy frame flag
		movlb	.15
		bsf		TXB0CON,TXREQ		;send busy frame
		bcf		PIE5,FIFOWMIE		;disable FIFO interrupt 
		bsf		PIE5,TXB0IE			;enable IRQ for busy frame sent
		movlb	0
		bra		back1	
	
no_fifo	bcf		PIR5,TXB0IF			;clear busy frame flag
		bcf		PIE5,TXB0IE			;no busy frame IRQ
		bsf		PIE5,FIFOWMIE		;wait for next FIFO IRQ
		bra		back1


		
		;Transmit error routine here. Only acts on lost arbitration	

txerr	movlb	.15					;change bank			
		btfss	TXB1CON,TXLARB
		bra		errbak				;not lost arb.
	
		movf	Latcount,F			;is it already at zero?
		bz		errbak
		decfsz	Latcount,F
		bra		errbak
		bcf		TXB1CON,TXREQ
		movlw	B'00111111'
		andwf	TXB1SIDH,F			;change priority
txagain bsf		TXB1CON,TXREQ		;try again
					
errbak	bcf		RXB1CON,RXFUL
		movlb	0
		bcf		RXB0CON,RXFUL		;ready for next
		
		bcf		COMSTAT,RXB0OVFL	;clear overflow flags if set
		bcf		COMSTAT,RXB1OVFL		
		bra		back1


		
back	bcf		RXB0CON,RXFUL		;ready for next
	
	
back1	clrf	PIR5				;clear all flags
		movf	CANCON,W			;recover variables
		andlw	B'11110001'
		iorwf	TempCANCON,W
		
		movwf	CANCON
		movff	PCH_tempH,PCLATH
		movff	Fsr_temp0L,FSR0L		;recover FSR0
		movff	Fsr_temp0H,FSR0H

		movff	Fsr_temp1L,FSR1L		;recover FSR1
		movff	Fsr_temp1H,FSR1H
		movf	W_tempL,W
		movff	St_tempL,STATUS	
		
		retfie	

;	
;**************************************************************************************************

	

;		Note. SLiM mode only used for initial setup. This is not a SLiM module

;		main running loop. Looks for incoming CAN frame. Acts on it if received.
;		Clears CAN when done.
;		Checks small PB for reversion to SLiM. If held in, then go back to virgin state.
;		Then checks keys for any change and sends message if required.

main	btfsc	COMSTAT,7			;any CAN frame?
		call 	getcan
		btfsc	Datmode,0
		call	packet

				;do the CAN message.

		bcf		Datmode,0
		bcf		RXB0CON,RXFUL		;clear CAN RX buffer for next
		btfsc	S_PORT,S_BIT		;PB in?
		bra		main1
		call	set1				;yes so cancel
		sublw	0
		bz		main1
		goto	setup1				;out of main and back to SLiM
		
main1	call	do_keys				;check keys for any change and do it.
		bra		main
		


;***************************************************************************
;		main setup routine
;*************************************************************************


		
setup	call 	setsub		;set hardware first. 
		
;		test for setup mode

		call	setup1		;checks for SLiM / FLiM and sets accordingly
							;waits till in FLiM
	

		


;		specific to CANSCAN
		
setid	

		call	chkevdata		;set up flash ram if not already done
		call	readsw			;switch parameters to RAM

		clrf	EEADR
		call	newid			;put ID into Tx1buf, TXB2 and ID number store
		btfsc	Datmode,2
		bra		seten_f

	



;		for switch scan

		call	sw_set			;set switch variables to current switch state.
								;will be all off if PBs are used
		clrf	Colcnt
		clrf	Sflag
		movff	Colcnt,LATA		;set first scan column
		clrf	Scancnt
	


		
seten_f	
		bcf		LED_PORT,GREEN		;green off
		btfss	Datmode,2			;flashing?
		
		bsf		LED_PORT,YELLOW		;Yellow LED on.
		bcf		RXB0CON,RXFUL
		bcf		Datmode,0

;		get NV1 to see what to do
		
		movlw	LOW	NVstart			;layout setup routine 
		movwf	EEADR
		call	eeread				;get NV
		
		btfss	WREG,7
		bra		getNV1
		movwf	NVtemp
		movlw	1
		movwf	NVtemp1				;flag for no defaults sent	
			
getNV1	movf	NVtemp,W
		andlw 	B'01111111'			;mask top bit
		movwf	NVtemp				;save it
		
	

;		check for what to do on startup

		movf	NVtemp,W
		bz		lay0		;set to last (default) NV1 = 0
		movlw	1
		subwf	NVtemp,W
		bz		set_end		;NV1 = 1 so do nothing
		movlw	2
		subwf	NVtemp,W
		bz		sw_scan		;NV1 = 2 look at switches and set accordingly (change from earlier versions)
		movlw	3
		subwf	NVtemp,W
		bz		lay_off		;NV1 = 3 set all off  (change from earlier versions)
		
		bra		set_end		;no other options
		
;		do layout startup		
	
lay_off call	sub_off		;all to off.
		bra		lay0


;events are the last saved  (the default state)

lay0	movlw	.128			;128 possible switch events
		movwf	Scount
		clrf	Switchno	;start at first switch
		bra		setlay		;set layout

;scan switches if NV1 = 2

sw_scan	call	rd_sw		;read all switches if not a toggle. Only valid with on/off switches
		call	readsw		;put in RAM
		bra		lay0

setlay	call	sub_set		;set layout

							;end of setup.

set_end		goto	main	;run main loop






		



;***************************************************************************			

#include  "../cbuslib/evhndler_1c_25k80.asm"		;this is event handler. Uses hash method for lookup
											;written by Roger Healey. See the file header for useage.
											;file is in cbuslib.
	
;****************************************************************************

;		start of subroutines



getcan	movf	CANCON,W			;look for a CAN frame. Used in main outine
		andlw	B'00001111'
		movwf	TempCANCON
		movf	ECANCON,W			;uses the ECAN in mode 2
		andlw	B'11110000'
		iorwf	TempCANCON,W
		movwf	ECANCON
		btfsc	RXB0SIDL,EXID		;ignore extended frames here
		bra		no_can
		btfss	RXB0DLC,RXRTR		;is it RTR?
		bra		get_3
		call	isRTR
		bra		no_can
		

	
get_3	movf	RXB0DLC,F
		bnz		get_2			;ignore zero length frames
		bra		no_can 
get_2	bsf		Datmode,0		;valid message frame. Datmode bit 0 indicates a CAN frame waiting.
		call	copyev			;save to buffer. Saves OpCode and 4 bytes.	
		return

no_can	bcf		RXB0CON,RXFUL	;no valid CAN frame so clear for next.
		return

;**********************************************************************


;		Send contents of Tx1 buffer via CAN TXB1

sendTX1	movff	FSR1L,Fsr_temp1L		;save FSRs
		movff	FSR1H,Fsr_temp1H
		movff	FSR0L,Fsr_temp0L
		movff	FSR0H,Fsr_temp0H


		lfsr	FSR0,Tx1con			;shift buffer to TXB1 registers
		lfsr	FSR1,TXB1CON
		
		movlb	.15				;check for buffer access
ldTX2	btfsc	TXB1CON,TXREQ	; Tx buffer available...?
		bra		ldTX2			;... not yet  so loop
		movlb	0
		
ldTX1	movf	POSTINC0,W
		movwf	POSTINC1		;load TXB1 registers
		movlw	Tx1d7+1
		cpfseq	FSR0L
		bra		ldTX1

		
		movlb	.15				;bank 15
tx1test	btfsc	TXB1CON,TXREQ	;test if clear to send
		bra		tx1test
		bsf		TXB1CON,TXREQ	;OK so send
		
tx1done	movlb	0				;bank 0
		movff	Fsr_temp1L,FSR1L		;recover FSRs
		movff	Fsr_temp1H,FSR1H
		movff	Fsr_temp0L,FSR0L
		movff	Fsr_temp0H,FSR0H
		return					;successful send

		
;*********************************************************************
;		put in NN from command. 

putNN	movff	RXB0D1,NN_temph	;get new Node Number
		movff	RXB0D2,NN_templ
		movlw	LOW NodeID		;put in EEPROM
		movwf	EEADR
		movf	RXB0D1,W
		call	eewrite
		incf	EEADR
		movf	RXB0D2,W
		call	eewrite
		movlw	Modstat			;set module status to running mode and store it.
		movwf	EEADR
		movlw	B'00001000'		;Module status has NN set  (bit 3)
		movwf	Datmode			;running mode
		call	eewrite			;save mode
		
		return

;***************************************************************************	

newid	movlw	LOW CANid		;put new CAN_ID etc in EEPROM
		movwf	EEADR
		call	eeread
		movwf	CanID_tmp			
		call	shuffle			;rearrange bits so there is a single CAN_ID byte
		movlw	B'11110000'
		andwf	Tx1sidh
		movf	IDtemph,W		;set current ID into CAN buffer
		iorwf	Tx1sidh			;leave priority bits alone
		movf	IDtempl,W
		movwf	Tx1sidl			;only top three bits used
		movlw	LOW NodeID
		movwf	EEADR
		call	eeread
		movwf	NN_temph		;get stored NN
		incf	EEADR
		call	eeread
		movwf	NN_templ	
		
		movlb	.15				;put CAN_ID into TXB2 for enumeration response to RTR
new_1	btfsc	TXB2CON,TXREQ
		bra		new_1
		clrf	TXB2SIDH
		movf	IDtemph,W
		movwf	TXB2SIDH
		movf	IDtempl,W
		movwf	TXB2SIDL
		movlw	0xB0
		iorwf	TXB2SIDH		;set priority
		clrf	TXB2DLC			;no data, no RTR
		movlb	0

		return

;*********************************************************************		

;		request frame for new NN or ack if not virgin
		
nnreq	movlw	OPC_RQNN		
nnrel	movwf	Tx1d0
		movff	NN_temph,Tx1d1
		movff	NN_templ,Tx1d2
		movlw	3
		movwf	Dlc
		call	sendTX
		return



		
;*****************************************************************************
;
;		shuffle for standard ID. Puts 7 bit ID into IDtemph and IDtempl for CAN frame

shuffle	movff	CanID_tmp,IDtempl		;get 7 bit ID
		swapf	IDtempl,F
		rlncf	IDtempl,W
		andlw	B'11100000'
		movwf	IDtempl					;has sidl
		movff	CanID_tmp,IDtemph
		rrncf	IDtemph,F
		rrncf	IDtemph,F
		rrncf	IDtemph,W
		andlw	B'00001111'
		movwf	IDtemph					;has sidh
		return

;*********************************************************************************

;		reverse shuffle for incoming ID. sidh and sidl into one byte.

shuffin	movff	RXB0SIDL,IDtempl
		swapf	IDtempl,F
		rrncf	IDtempl,W
		andlw	B'00000111'
		movwf	IDtempl
		movff	RXB0SIDH,IDtemph
		rlncf	IDtemph,F
		rlncf	IDtemph,F
		rlncf	IDtemph,W
		andlw	B'01111000'
		iorwf	IDtempl,W			;returns with ID in W
		return
;************************************************************************************

;read a EEPROM byte, EEADR  must be set before this sub.
;		
eeread	bcf		EECON1,EEPGD	
		bcf		EECON1,CFGS		
		bsf		EECON1,RD
		nop						;needed for K series PICs
		movf	EEDATA,W		;returns with data in W
		return

;**************************************************************************

;	write to EEPROM, EEADR must be set before this sub.
;	data to write in W

eewrite	movwf	EEDATA		
		bcf		EECON1,EEPGD	
		bcf		EECON1,CFGS
		bsf		EECON1,WREN
		movff	INTCON,TempINTCON
		
		clrf	INTCON	;disable interrupts
		movlw	0x55
		movwf	EECON2
		movlw	0xAA
		movwf	EECON2
		bsf		EECON1,WR

eetest	btfsc	EECON1,WR				;check it has written
		bra		eetest
		bcf		PIR2,EEIF
		bcf		EECON1,WREN
		movff	TempINTCON,INTCON		;reenable interrupts
		
		return	
		

;*********************************************************************
;		send a CAN frame
;		entry at sendTX puts the current NN in the frame - for producer events
;		entry at sendTXa needs Tx1d1 and Tx1d2 setting first
;		Latcount is the number of CAN send retries before priority is increased
;		the CAN_ID is pre-loaded in the Tx1 buffer 
;		Dlc must be loaded by calling source to the data length value
		
sendTX	movff	NN_temph,Tx1d1		;get NN
		movff	NN_templ,Tx1d2

sendTXa	movf	Dlc,W				;get data length
		movwf	Tx1dlc
		movlw	B'00001111'		;clear old priority
		andwf	Tx1sidh,F
		movlw	B'10110000'
		iorwf	Tx1sidh			;low priority
		movlw	.10
		movwf	Latcount
		call	sendTX1			;send frame
		return			

;**************************************************************************

;	main key scanning routine. Used to teach the key number 

do_keys	btfsc	Datmode,7		;dont'scan. Already got a key.
		return
		
		decfsz	Scancnt 		;only scan every 256 loops. Used for debounce.
		return
		decfsz	Debcnt			;debounce is now divide by 4 as processor is running at 16 MIPS.
		return
		movlw	4				;reset debcount
		movwf	Debcnt
		call	keyscan			;this actually scans the key matrix
		tstfsz	WREG			;WREG set if a switch change while learning
		bra		is_sw
		return
is_sw	btfss	Datmode,4		;in learn mode?
		return					;no
		bsf		Datmode,7		;prevent multiple sends.
		movlw	OPC_ARON1		;ARON1 response. Sends switch number to FCU
								;without risking doing anything
		movwf	Tx1d0
		movf	Switchno,W
		bcf		WREG,7			;clear pol bit if set
		movwf	Tx1d5
		incf	Tx1d5			;switch numbers start at 1
		clrf	Tx1d3
		clrf	Tx1d4
		movlw	6
		movwf	Dlc
		call	sendTX			;send event with NN
		clrf	Colcnt
		clrf	Sflag

		return

;*******************************************************************

;		check if command is for this node

thisNN	movf	NN_temph,W
		subwf	RXB0D1,W
		bnz		not_NN
		movf	NN_templ,W
		subwf	RXB0D2,W
		bnz		not_NN
		retlw 	0			;returns 0 if match
not_NN	retlw	1
							


		
;**************************************************************************
;		send node parameter bytes (7 maximum)

parasend	
		movlw	OPC_PARAMS
		movwf	Tx1d0
		movlw	LOW nodeprm
		movwf	TBLPTRL
		movlw	8
		movwf	TBLPTRH			;relocated code
		lfsr	FSR0,Tx1d1
		movlw	7
		movwf	Count
		bsf		EECON1,EEPGD
		
para1	tblrd*+
		movff	TABLAT,POSTINC0
		decfsz	Count
		bra		para1
		bcf		EECON1,EEPGD	
		movlw	8
		movwf	Dlc
		call	sendTXa			;send CBUS message
		return

;**************************************************************************

;		send module name - 7 bytes

rqmn	btfss	Datmode,2		;setup mode only
		return
		movlw	OPC_NAME
		movwf	Tx1d0
		movlw	LOW myName
		movwf	TBLPTRL
		movlw	HIGH myName
		movwf	TBLPTRH			;relocated code
		lfsr	FSR0,Tx1d1
		movlw	7
		movwf	Count
		bsf		EECON1,EEPGD
		
name1	tblrd*+
		movff	TABLAT,POSTINC0
		decfsz	Count
		bra		name1
		bcf		EECON1,EEPGD	
		movlw	8
		movwf	Dlc
		call	sendTXa
		return
		

;**********************************************************

;		send individual parameter

;		Index 0 sends no of parameters

para1rd	movf	RXB0D3,W
		sublw	0
		bz		numParams
		movlw	PRMCOUNT
		movff	RXB0D3, Temp
		decf	Temp
		cpfslt	Temp
		bra		pidxerr
		movlw	OPC_PARAN
		movwf	Tx1d0
		movlw	7			;FLAGS index in nodeprm
		cpfseq	Temp
		bra		notFlags			
		call	getflags
		movwf	Tx1d4
		bra		addflags
notFlags		
		movlw	LOW nodeprm
		movwf	TBLPTRL
		movlw	HIGH nodeprm
		movwf	TBLPTRH		;relocated code
		clrf	TBLPTRU
		decf	RXB0D3,W
		addwf	TBLPTRL
		bsf		EECON1,EEPGD
		tblrd*
		movff	TABLAT,Tx1d4
addflags						
		movff	RXB0D3,Tx1d3
		movlw	5
		movwf	Dlc
		call	sendTX
		return	
		
numParams
		movlw	OPC_PARAN
		movwf	Tx1d0
		movlw	PRMCOUNT
		movwf	Tx1d4
		movff	RXB0D3,Tx1d3
		movlw	5
		movwf	Dlc
		call	sendTX
		return
		
pidxerr
		movlw	.10				;error
		call	errsub
		return
		
getflags						; create flags byte
		movlw	PF_PRODUCER
		btfsc	Datmode,3
		iorlw	4				; set bit 2
		movwf	Temp
		bsf		Temp,3			;set bit 3, we are bootable
		movf	Temp,W
		return
		
		
;**********************************************************

; returns Node Number, Manufacturer Id, Module Id and Flags

whoami
		call	ldely		;wait for other nodes
		movlw	OPC_PNN
		movwf	Tx1d0
		movlw	MAN_NO		;Manufacturer Id
		movwf	Tx1d3
		movlw	MODULE_ID		; Module Id
		movwf	Tx1d4
		call	getflags
		movwf	Tx1d5
		movlw	6
		movwf	Dlc
		call	sendTX
		return

	
;**************************************************************

		; copy event data to safe buffer

copyev	movff	RXB0D0, ev_opc
		movff	RXB0D1, ev0
		movff	RXB0D2, ev1
		movff	RXB0D3, ev2
		movff	RXB0D4, ev3
		movff	RXB0D5, EVidx      	; only used by learn and some read cmds
		movff	RXB0D6, EVdata		; only used by learn cmd

		movlw	OPC_ASON
		subwf	RXB0D0,W
		bz		short				;is a short event
		movlw	OPC_ASOF
		subwf	RXB0D0,W
		bz		short
		movlw	OPC_ASRQ
		subwf	RXB0D0,W
		bz		short
		return

short	clrf	ev0					;here if a short event
		clrf	ev1
		return

		
;****************************************************************

;		longer delay

ldely	movlw	.100
		movwf	Count2
ldely1	call	dely
		decfsz	Count2
		bra		ldely1
		
		return

;******************************************************************

;		set switch info in FLASH, EEPROM and RAM

set_sw	
		movf	PREINC0,W		;Get switch number
		decf	WREG			;switch numbers start at 1 so decrement
		bcf		WREG,7			;pol bit
		movwf	Switchno
		
		movlw	LOW swstate		;put in EEPROM
		movwf	EEADR
		movf	Switchno,W
		addwf	EEADR

		movf	PREINC0,W		;get SV
		bsf		WREG,7			;flag for taught switch event
		movf	RXB0D1,F		;check for short event
		bnz		write
		movf	RXB0D2,F		;
		bnz		write
		bsf		WREG,5			;set fo short event
write
		call	eewrite
		call	readsw			;put back in RAM
		
		call	putsw_ev		;put event in FLASH

		return
								


;***********************************************************************

;	unset switch info in FLASH, EEPROM and RAM. restore to default

unset_sw	movf	PREINC0,W		;Get switch number
		decf	WREG			;switch numbers start at 1 so decrement
		bcf		WREG,7			;pol bit
		movwf	Switchno
		movlw	LOW swstate		;put in EEPROM
		movwf	EEADR
		movf	Switchno,W
		addwf	EEADR

		movlw	1				;get default SV
		call	eewrite
		call	readsw			;put back in RAM
		call	defsw_ev		;put in FLASH


		return


		

;**********************************************************************

;			read switch variables to RAM
;
readsw		lfsr	FSR0,sv0
			movlw	.128
			movwf	Count
			movlw	LOW swstate
			movwf	EEADR

readsw1		call	eeread
			movwf	POSTINC0
			incf	EEADR
			decfsz	Count
			bra		readsw1
			
			return
			
;*********************************************************


sendlog
		movlw	OPC_ARDAT
		movwf	Tx1d0
		movff	FSR0H, Tx1d3
		movff	FSR0L, Tx1d4
		movff	EVtemp, Tx1d5
		movff	EVtemp2, Tx1d6
		clrf	Tx1d7
		movlw	8
		movwf	Dlc
		call	sendTX
		call	ldely
		return

;**************************************************************
;
;		main key scan routine

keyscan	movlw	B'11000000'		;don't scan while sending layout setup info
		andwf	Datmode,W
		bz		key1
		retlw	0
key1	movff	FSR1L,Fsr_scan1L		;save FSRs
		movff	FSR1H,Fsr_scan1H
		movff	FSR2L,Fsr_scan2L
		movff	FSR2H,Fsr_scan2H

		btfsc	Sflag,1			;has changed so do debounce
		bra		debnce

		movf	PORTC,W			;get rows
		comf	WREG			;for 74HC238
		
		movwf	Swtemp			;hold value

		lfsr	FSR1,Switch1	;load FSRs
		lfsr	FSR2,Swtemp1
		movlw	B'00001111'
		andwf	Colcnt,W
		addwf	FSR1L
		addwf	FSR2L

		movf	Swtemp,W		;get switch byte
		xorwf	INDF1,W			;has it changed?
		bz		sec1			;no
		bsf		Sflag,0			;set for debounce
sec1	movff	Swtemp,INDF2	;save current input
		incf	Colcnt
		movff	Colcnt,LATA		;next column
		movlw	B'00001111'
		andwf	Colcnt,W		;end of scan?
		bnz		sec2
		btfsc	Sflag,0			;any change?
		bsf		Sflag,1			;do debounce
sec2	retlw	0

debnce	bcf		Sflag,0			;clear match
		
		movf	PORTC,W			;get rows
		comf	WREG
	
		movwf	Swtemp			;hold value


sec_b	
		lfsr	FSR2,Swtemp1
		movlw	B'00001111'
		andwf	Colcnt,W
		addwf	FSR1L
		addwf	FSR2L
		
		movf	Swtemp,W		;get switch byte
		xorwf	INDF2,W			;has it changed again?
		bz		sec1_b			;no
		bsf		Sflag,0			;set for change
sec1_b	
		incf	Colcnt
		movff	Colcnt,LATA		;next column
		movlw	B'00001111'
		andwf	Colcnt,W		;end of scan?
		bz		swchng
sec2_b	retlw	0

		


swchng	btfss	Sflag,0			;change so is a bounce
		bra		deb1
		bcf		Sflag,1			;clear debounce flag
		bcf		Sflag,0			;clear change flag
		bra		swret			;scan again

swret	movff	Fsr_scan1L,FSR1L		;unsave FSRs
		movff	Fsr_scan1H,FSR1H
		movff	Fsr_scan2L,FSR2L
		movff	Fsr_scan2H,FSR2H
		retlw	0

swret1	movff	Fsr_scan1L,FSR1L		;unsave FSRs
		movff	Fsr_scan1H,FSR1H
		movff	Fsr_scan2L,FSR2L
		movff	Fsr_scan2H,FSR2H
		retlw	1

;	now sort out which has changed

deb1	bcf		Sflag,1			;clear debounce
		lfsr	FSR1,Switch1	;here on valid change
		lfsr	FSR2,Swtemp1
		clrf	Switchno		;switch number counter
bits_1	movlw	B'00000001'		;set rolling bit
		movwf	Swbit
		movlw	8
		movwf	Count			;8 bits per byte
		movf	INDF1,W
		xorwf	INDF2,W			;has this byte changed?
		bz		nxt_sw			;no
		movwf	Swtemp			;save bit that has changed
bit_tst	movf	Swbit,W			;get rolling bit
		andwf	Swtemp,W		;this switch?
		bnz		this_sw			;yes
		rlncf	Swbit,F			;no, so roll one
		incf	Switchno,F
		decfsz	Count			;last bit?
		bra		bit_tst
nxt_sw	incf	FSR1L
		incf	FSR2L
		movf	FSR2L,W			;last of set?
		sublw	Swtemp1+.16
		bz		sw_done			;finished
		movlw	8
		addwf	Switchno
		bra		bits_1			;next comparison
sw_done	bra		swret			;should never get here

this_sw	movff	INDF2,INDF1		;new pattern to old
		btfss	Datmode,4		;is it switch learn?
		bra		chng2			;no
		bra		swret1			;return for switch learn
chng2	andwf	INDF2,W			;which polarity?
		bz		sw_off
		bsf		Switchno,7		;bit 7 used as pol flag. Set if on.
		bra		chngsw			;do switch change
sw_off	bcf		Switchno,7		;is an off
chngsw	lfsr	FSR1,sv0		;look up switch variable. What should this switch do?
		movf	Switchno,W		;get switch number
		bcf		WREG,7			;clear POL bit if set
		addwf	FSR1L
		movf	INDF1,W			;get variable
		bnz		chng1	
		bra		swret			;abort. A bounce or SW action not set.

chng1	movwf	Switch_t		;save sw variable
		btfsc	WREG,0			;is it on / off
		bra		on_off
		btfsc	WREG,2			;is it ON only?
		bra		on_only
		btfsc	WREG,3			;is it toggle?
		bra		tog_sw
		bra		swret			;must be one of these

on_off	btfsc	Switch_t,5		;is it a short event
		bra		on_off1
		movlw	OPC_ACON
		movwf	Tx1d0			;set OPC
		bra		on_off2

on_off1	movlw	OPC_ASON
		movwf	Tx1d0
on_off2	btfsc	Switchno,7		;on or off
		bsf		Tx1d0,0
		btfsc	Datmode,7		;no off event if teaching button
		bra		swret
		btfsc	Switch_t,1		;Swtemp,1
		btg		Tx1d0,0			;reverse polarity if pol bit set in sv.
		bra		snd_sw			;send the switch change

on_only	btfsc	Switchno,7		;ignore an off
		retlw	0
		
		btfsc	Switch_t,5
		bra		only1
		movlw	OPC_ACON
		movwf	Tx1d0
		bra		only2
only1	movlw	OPC_ASON
		movwf	Tx1d0
only2	btfsc	Switch_t,1		;Swtemp,1		;pol bit
		bsf		Tx1d0,0
		btfsc	Switch_t,4		;is it a sod
		bra		sodsw
		bra		snd_sw			;send the switch change

tog_sw	btfsc	Switchno,7		;ignore off state
		retlw	0
		lfsr	FSR1,sv0		;ready for bit change
		btfsc	Switch_t,6		;Swtemp,6		;old toggle bit
		bra		tog_on
		movf	Switchno,W
		addwf	FSR1L
		bsf		INDF1,6			;set toggle bit	
		bsf		Switchno,7		;set for off
		bra		on_off			;send OFF
tog_on  movf	Switchno,W
		addwf	FSR1L
		bcf		INDF1,6			;clear toggle
		bra		on_off			;send ON

snd_sw	call	load_sw			;get event for that switch into Tx buffer
		movlw	LOW	swstate		;get SV from EEPROM to Switch_t
		movwf	EEADR
		movf	Switchno,W
		bcf		WREG,7			;clear pol bit
		addwf	EEADR
		call	eeread
		movwf	Switch_t		;save it
		btfsc	Switch_t,7		;is it taught
		bra		snd_sw5
		btfsc	NVtemp1,0		;no defaults ?
		goto	swret			;don't do
snd_sw5		movff	Tx1d0,Ss0		;put OPC for self send
		movlw	5
		movwf	Dlc
		btfsc	Switch_t,5		;Swtemp,5		;is	it short?
		bra		snd_sw1
		movf	Tx1d1,F			;check for initial switch long events
		bnz		snd_sw3			;if NN = 00 00, then add current NN
		movf	Tx1d2,F			;else leave alone
		bnz		snd_sw3
		bra		snd_sw1			;add NN
snd_sw3	call	sendTXa			;Don't change N bytes
		bra		snd_sw2
snd_sw1	call	sendTX			;Add NN bytes
snd_sw2	bcf		Switch_t,6		;Swtemp,6		;clear polarity bit
		btfsc	Tx1d0,0			;what polarity was sent?
		bsf		Switch_t,6		;Swtemp,6		;was off so set bit 6
		movlw	LOW swstate		;now save in EEPROM
		movwf	EEADR
		movf	Switchno,W		;get switch number
		bcf		WREG,7
		addwf	EEADR
		movf	Switch_t,W		;Swtemp,W		;save new SV
		call	eewrite
		goto	swret			;done


sodsw	call	set_loop
		bcf		Tx1d0,0			;an ON event
		bra		snd_sw			
					
		



load_sw movlw	0x29			;get switch event to send. Held in FLASH by switch number
		movwf	TBLPTRH
		clrf	TBLPTRL
		movf	Switchno,W
		bcf		WREG,7			;mask POL bit
		rlncf	WREG			;four bytes per entry
		bcf		STATUS,C		;clear carry
		rlcf	WREG
		btfsc	STATUS,C
		incf	TBLPTRH
		addwf	TBLPTRL
		
		tblrd*+
		movf	TABLAT,W
		movwf	Tx1d1
		movwf	Ss1
		tblrd*+
		movf	TABLAT,W
		movwf	Tx1d2
		movwf	Ss2
		tblrd*+
		movf	TABLAT,W
		movwf	Tx1d3
		movwf	Ss3
		tblrd*+
		movf	TABLAT,W
		movwf	Tx1d4
		movwf	Ss4
		goto	swret			;got it





;***********************************************************
;
;		put switch event into flash

putsw_ev	movlw	0x29 		;switch events start at 0x2900
		movwf	evaddrh	
		movf	Switchno,W		;switch events indexed by switch number
		bcf		WREG,7			;clear pol bit
		movwf	Swtemp
put_loop movlw	.16	
		subwf	Swtemp,F			;which bank is switch in?
		bn		putsw
		movlw	.64
		addwf	evaddrl		;next block
		bnc		put_loop
		incf	evaddrh
		bra		put_loop		
		
		

				

putsw	call	rdfbev			; block
		lfsr	FSR0,evt00
		lfsr	FSR1,ev0
		movlw	.16
		addwf	Swtemp
		movf	Swtemp,W		;Switchno,W
		rlncf	WREG
		rlncf	WREG
		addwf	FSR0L			;4 bytes per event
		movff	POSTINC1,POSTINC0
		movff	POSTINC1,POSTINC0
		movff	POSTINC1,POSTINC0
		movff	POSTINC1,POSTINC0

		call	wrfbev
		return			


	
		

;*********************************************************************

;		put default switch event into flash

defsw_ev movlw	0x29 		;switch events start at 0x2900
		movwf	evaddrh	
		movf	Switchno,W
		movwf	Swtemp
def_loop movlw	.16	
		subwf	Swtemp,F			;which bank is switch in?
		bn		defsw
		movlw	.64
		addwf	evaddrl		;next block
		bnc		def_loop
		incf	evaddrh
		bra		def_loop		
		
				

defsw	call	rdfbev			; block
		lfsr	FSR0,evt00
		lfsr	FSR1,ev0
		movlw	.16
		addwf	Swtemp
		movf	Swtemp,W		;Switchno,W
		rlncf	WREG
		rlncf	WREG
		addwf	FSR0L			;4 bytes per event
		clrf	POSTINC0
		clrf	POSTINC0
		clrf	POSTINC0
		incf	Switchno,W
		movwf	POSTINC0
		
		call	wrfbev			;write back
		return				




		
;********************************************************************

;		do_it as a subroutine. Does whatever the CBUS message is - from Packet.
;
do_it	movff	FSR1H,Saved_Fsr1H
		movff	FSR1L,Saved_Fsr1L
		call	enmatch				;is it an event match?
		sublw	0
		bz		do_it1
		return						;no

		
sod		call 	set_loop	;if a sod, do it
		return

do_rsp1	goto	do_rsp		;branch too long

do_it1	call	rdfbev		; read relevant event data
		movff	FSR0L,Saved_Fsr0L ;save pointer
		movff	FSR0H,Saved_Fsr0H
		movff	FSR0H,FSR1H	;point to correct set of 16 (16 bytes per switch event)
		movlw	OPC_AREQ	;is it a request?
		subwf	ev_opc,W
		bz		do_rsp1		;do a response
		movlw	OPC_ASRQ
		subwf	ev_opc,W
		bz		do_rsp1
		btfsc	INDF0,1		;for  SoD
		
		bra		sod
	
		
		
do_it2	
		btfsc	ev_opc,0		; is it ON cmnd
		bra		offcmnd			; j if OFF
		btfss	EVflags, 1
		bra		switch			; j if On event not actioned

	
		


switch	btfsc	Datmode,0		;is it an incoming event?
		bra		sw_in			;yes
		movff	Saved_Fsr0L,FSR0L
		movlw	2
		addwf	FSR0L			;get switch control byte
		btfss	INDF0,3			;not a toggle event?
		return

		decf	FSR0L			;
		movf	INDF0,W			;get switch number
		lfsr	FSR1,sv0		;switch status table
		addwf	FSR1L			;point to switch
		decf	FSR1L			;switchno to base 0
		btfsc	ev_opc,0		;on or off
		bra		sw_off1
		btfss	INDF1,6
		bra		sw_ret			;do nothing
		bcf		INDF1,6
		bra		sw_ret
sw_off1	btfsc  	INDF1,6
		bra		sw_ret
		bsf		INDF1,6	
sw_ret	
		movff	Saved_Fsr1H,FSR1H
		movff	Saved_Fsr1L,FSR1L	
		return	

 

sw_in	movff	Saved_Fsr0L,FSR0L
		incf	FSR0L,F			;get switch number
		movf	INDF0,W			;get switch number
		movwf	Switchno
		lfsr	FSR1,sv0		;switch status table
		addwf	FSR1L			;point to switch
		decf	FSR1L			;switchno to base 0
		btfsc	ev_opc,0		;on or off
		bra		sw_off2
		bcf		INDF1,6			;set on/off bit
		bra		sw_ret1			
		
sw_off2	bsf  	INDF1,6
		bra		sw_ret1

sw_ret1 
		movlw	LOW swstate		;now save in EEPROM
		movwf	EEADR
		movf	Switchno,W		;get switch number
		decf	WREG			;base 0
		bcf		WREG,7
		addwf	EEADR
		movf	INDF1,W			;get new state
		
		call	eewrite

		bcf		Datmode,0		;done incoming event
		bra		sw_ret

offcmnd	;btfss	EVflags, 0
		bra		switch			; j if Off event not actioned
		
		


do_rsp	movff	Saved_Fsr0L,FSR1L
		btfss	INDF1,0		;is it a switch event?
		return				;no
		
resp1	movlw 	1
		movf	PLUSW1,W		;get switch number
		decf	WREG
		movwf	Swtemp5			;base 0
	
read	movlw	LOW swstate		;get switch state
		addwf	Swtemp5,W		;add switch number
		movwf	EEADR
		call	eeread
		movwf	Swtemp5			;save SV
		movlw	OPC_AREQ		;is it a long?
		subwf	ev_opc,W
		bnz		shrt_on
resp3	movlw	OPC_ARON
		movwf	Tx1d0
		btfsc	Swtemp5,6	;on or off?
		bra		off
snd_lng		movff	ev0,Tx1d1
		movff	ev1,Tx1d2
		movff	ev2,Tx1d3
		movff	ev3,Tx1d4
		movlw	5
		movwf	Dlc
		call	sendTXa
		bra		sw_ret

shrt_on	movlw	OPC_ASRQ
		subwf	ev_opc,W
		bnz		sw_retz		;not a short request
		movlw	OPC_ARSON
		movwf	Tx1d0
		btfsc	Swtemp5,6
		bra		shrt_of
snd_sh	movff	ev2,Tx1d3
		movff	ev3,Tx1d4
		movlw	5
		movwf	Dlc
		call	sendTX
	
		bra		sw_retz

off		movlw	OPC_AREQ		;is it a long?
		subwf	ev_opc,W
		bnz		shrt_of
		movlw	OPC_AROF		;off
		movwf	Tx1d0
		bra		snd_lng		;send it

shrt_of	movlw	OPC_ARSOF
		movwf	Tx1d0
		bra		snd_sh

sw_retz	goto	sw_ret		;too long branch

;******************************************************************

;		routine to set layout to last state on startup
;		called if NV1 = 0 (default)or on SoD



set_loop movff	Switchno,Switch_s	;save current switch number
		movlw	B'00110010'		
		movwf	T1CON
		clrf	TMR1H			;reuse timer 1  65 mSec delay between sends
		clrf	TMR1L			;avoids flooding the bus
		bcf		PIR1,TMR1IF
		bsf		T1CON,TMR1ON

		movlw	.128			;128 switches
		movwf	Scount
		clrf	Switchno
set_nxt 
		movlw	LOW swstate		;get saved SVs
		addwf	Switchno,W
		movwf	EEADR
		call	eeread
		btfss	WREG,7			;is it a taught event?
		bra		nxt_lay
		movwf	Switch_t
		btfsc	Switch_t,4		;miss out a SoD send
		bra		nxt_lay
		movlw	OPC_ACON		;set OPC
		movwf	Tx1d0
		btfsc	Switch_t,5		;is it a short?
		bsf		Tx1d0,3			;yes
		btfsc	Switch_t,6		;is it on or off?
		bsf		Tx1d0,0			;off
		call	snd_sw			;send event
set_nxt1	btfss	PIR1,TMR1IF		;wait for timer
		bra		set_nxt1
		bcf		PIR1,TMR1IF

nxt_lay	incf	Switchno		;next switch
		decfsz	Scount,F		;last?
		bra		set_nxt			;more?
set_bak	movff	Switch_s,Switchno	;recover switch number
		clrf	Switch_t
		return


;**********************************************************************	

;		restores FLASH and EEPROM to default state. Includes SVs		

clr_sub	call	initevdata	;clear event FLASH
		movlw	.128
		movwf	Count
		movlw	LOW swstate	;reset EEPROM
		movwf	EEADR
clr1	movlw	1			;default SV state
		movwf	EEDATA
		call	eewrite
		incf	EEADR
		decfsz	Count
		bra		clr1
		call	readsw		;restore RAM table
		lfsr	FSR0,evt00	;set default event table
		movlw	.64
		movwf	Count
clr2	clrf	POSTINC0
		decfsz	Count
		bra		clr2
		clrf	evaddrl		;starting FLASH page
		movlw	HIGH swdata
		movwf	evaddrh
		movlw	1			;start switch number
		movwf	Temp
		movlw	8
		movwf	Count1		;four pages
	
clr2a	movlw	.16
		movwf	Count	
		lfsr	FSR0,evt00
		decf	FSR0L		;first sw no at evt3
clr3	movlw	4
		addwf	FSR0L		;only fourth byte to change
		movff	Temp,INDF0
		incf	Temp
		decfsz	Count
		bra		clr3
	
		call	wrfbev		;write 64 byte block
		movlw	.64
		addwf	evaddrl		;next block
		btfsc	STATUS,C	;carry to next page?
		incf	evaddrh	
		decfsz	Count1		;last page?
		bra		clr2a

	
		return		
		




;*************************************************************

;	Request for read of node variable	

nvrd		call 	thisNN		;is it here?
			sublw	0
			bnz		nvrd1		;no
			movlw	LOW	NVstart
			addwf	ev2,W		;add index
			movwf	EEADR
			decf	EEADR,F		;index starts at 1, buffer at 0
			call	eeread
			movwf	Tx1d4		;NV val to transmit buffer
			movff	ev2,Tx1d3	;transfer index
			movlw	OPC_NVANS	;NVANS
			movwf	Tx1d0
			movlw	5
			movwf	Dlc
			call	sendTX		;send answer
nvrd1		return

;***********************************************************

;	A new NV so put it in EEPROM

putNV	call	thisNN  		;is it here?
		sublw	0
		bnz		no_NV
		movlw	NV_NUM + 1		;put new NV in EEPROM 
		cpfslt	ev2
		return
		movf	ev2,W
		bz		no_NV
		decf	WREG			;NVI starts at 1
		addlw	LOW NVstart
		movwf	EEADR
		movf	ev3,W
	
		call	eewrite	
		
no_NV	return
		

;******************************************************************

;		self enumeration as separate subroutine

self_en	movff	FSR1L,Fsr_tmp1Le	;save FSR1 just in case
		movff	FSR1H,Fsr_tmp1He 
		bsf		Datmode,1		;set to 'setup' mode
		movlw	.14
		movwf	Count
		lfsr	FSR0, Enum0
clr_en
		clrf	POSTINC0
		decfsz	Count
		bra		clr_en

		movlw	0x00			;set T0 to 128 mSec (may need more?)
		movwf	TMR0H			;this waits till all other nodes have answered with their CAN_ID
		movlw	0x00
		movwf	TMR0L
		movlw	B'00000100'		;clock div  32 (0.5 uSec clock)									
		movwf	T0CON			;enable timer 0
		bsf		T0CON,TMR0ON
		bcf		INTCON,TMR0IF
		
								;now send an RTR frame so all other nodes will send their CAN_IDs
		movlb	.15
		movlw	B'10111111'		;fixed node, default ID  
		movwf	TXB1SIDH
		movlw	B'11100000'
		movwf	TXB1SIDL
		movlw	B'01000000'		;RTR frame
		movwf	TXB1DLC
rtr_snd	btfsc	TXB1CON,TXREQ
		bra		rtr_snd
		bsf		TXB1CON,TXREQ
rtr_go	btfsc	TXB1CON,TXREQ		;wait till sent
		bra		rtr_go
		clrf	TXB1DLC				;no more RTR frames
		movlb	0
				
;	wait for answers	

self_en1	btfsc	INTCON,TMR0IF		;setup timer out?
		bra		en_done
		btfsc	COMSTAT,7		;look for CAN input. 
		bra		getcan1
		bra		self_en1		;no CAN
	

getcan1	movf	CANCON,W		;process answers
		andlw	B'00001111'
		movwf	TempCANCON
		movf	ECANCON,W
		andlw	B'11110000'
		iorwf	TempCANCON,W
		movwf	ECANCON
		btfsc	RXB0SIDL,EXID		;ignore extended frames here
		bra		no_can1
		
		
en_1	btfss	Datmode,1			;setup mode?
		bra		no_can1				;must be in setup mode
		movf	RXB0DLC,F
		bnz		no_can1				;only zero length frames
		call	setmode
		bra		no_can1	

no_can1	bcf		RXB0CON,RXFUL
		bra		self_en1			;loop till timer out 

en_done	bcf		T0CON,TMR0ON		;timer off
		bcf		INTCON,TMR0IF		;clear flag

; now sort out the new CAN_ID

		clrf	IDcount
		incf	IDcount,F			;ID starts at 1
		clrf	Roll
		bsf		Roll,0
		lfsr	FSR1,Enum0			;set FSR to start
here1	incf	INDF1,W				;find a space
		bnz		here
		movlw	8
		addwf	IDcount,F
		incf	FSR1L
		bra		here1
here	movf	Roll,W
		andwf	INDF1,W
		bz		here2
		rlcf	Roll,F
		incf	IDcount,F
		bra		here
here2	movlw	.100				;limit to ID
		cpfslt	IDcount
		bra		segful				;segment full
		
here3	movlw	LOW CANid		;put new ID in EEPROM
		movwf	EEADR
		movf	IDcount,W
		call	eewrite
		movf	IDcount,W
		call	newid			;put new ID in various buffers

			
		movff	Fsr_tmp1Le,FSR1L	;
		movff	Fsr_tmp1He,FSR1H 
		return					;finished	

segful	movlw	7		;segment full, no CAN_ID allocated
		call	errsub	;error
		setf	IDcount
		bcf		IDcount,7
		bra		here3

;********************************************************

isRTR	btfsc	Datmode,1		;setup mode?
		return					;back
		btfss	Datmode,3		;FLiM?
		return
		movlb	.15
isRTR1	btfsc	TXB2CON,TXREQ	
		bra		isRTR1		
		bsf		TXB2CON,TXREQ	;send ID frame - preloaded in TXB2

		movlb	0
		return

;****************************************************************
;
setmode	tstfsz	RXB0DLC
		return				;only zero length frames for setup
		
		swapf	RXB0SIDH,W			;get ID into one byte
		rrcf	WREG
		andlw	B'01111000'			;mask
		movwf	Temp
		swapf	RXB0SIDL,W
		rrncf	WREG
		andlw	B'00000111'
		iorwf	Temp,W
		movwf	IDcount				;has current incoming CAN_ID

		lfsr	FSR1,Enum0			;set enum to table
enum_st	clrf	Roll				;start of enum sequence
		bsf		Roll,0
		movlw	8
enum_1	cpfsgt	IDcount
		bra		enum_2
		subwf	IDcount,F			;subtract 8
		incf	FSR1L				;next table byte
		bra		enum_1
enum_2	dcfsnz	IDcount,F
		bra		enum_3
		rlncf	Roll,F
		bra		enum_2
enum_3	movf	Roll,W
		iorwf	INDF1,F
		bcf		RXB0CON,RXFUL		;clear read


		return

;***************************************************************

;		read all switches except PBs on startup
;		Colcnt starts at 0
;		EEADR set to start of swstate

rd_sw	movlw	LOW swstate		;start of switch states in EEPROM
		movwf	EEADR
		lfsr	FSR0,Switch1	;start of current switch state bytes
		movlw	.16				;number of switch state bytes
		movwf	Count			;counter
rd_sw1	movff	POSTINC0,Swtemp	;get switch byte
		movlw	8
		movwf	Count1			;bits per byte
rd_sw4	rrcf	Swtemp,F		;what is bit?
		btfss	STATUS,C
		bra		rd_sw2			;is a 0
		call	eeread			;get byte
		bcf		WREG,3			;no toggle
		iorlw	B'11000000'
		call	eewrite			;put in EEPROM
		bra		rd_sw3
rd_sw2	call	eeread			;get byte
		bcf		WREG,3			;no toggle
		bcf		WREG,6
		iorlw	B'10000000'
		call	eewrite
rd_sw3	incf	EEADR			;next EEPROM byte
		decfsz	Count1			;next switch byte?
		bra		rd_sw4
		decfsz	Count			;last Switch byte?
		bra		rd_sw1
		return


;********************************************************************
								;main packet handling is here
								;add more commands for incoming frames as needed
								;test OPC and branch if match to 'opc_x'
								;at opc_x, goto opc routine which is a sub and returns to 
								;code that called 'packet'.
		
packet	movlw	OPC_ACON	  ;only ON, OFF and REQ  events supported
		subwf	ev_opc,W	
		bz		go_on_x
		movlw	OPC_ACOF
		subwf	ev_opc,W
		bz		go_on_x
		movlw	OPC_AREQ
		subwf	ev_opc,W
		bz		go_on_x
		
		movlw	OPC_ASON			
		subwf	ev_opc,W
		bz		go_on_x
		movlw	OPC_ASOF
		subwf	ev_opc,W
		bz		go_on_x
		movlw	OPC_ASRQ
		subwf	ev_opc,W
		bz		go_on_x

;		now all other OPCs
		
		movlw	OPC_BOOT		;reboot
		subwf	ev_opc,W
		bz		boot_x
		movlw	OPC_ENUM		;re-enumerate
		subwf	ev_opc,W
		bz		enum_x
		movlw	OPC_RQNPN
		subwf	ev_opc,W
		bz		rqnpn_x			;read individual parameters

		movlw	OPC_CANID		;force new CAN_ID
		subwf	ev_opc,W
		bz		canid_x
		movlw	OPC_SNN			;set NN 
		subwf	ev_opc,W
		bz		snn_x
		movlw	OPC_QNN			; QNN
		subwf	ev_opc,w
		bz		qnn_x
		movlw	OPC_RQNP					
		subwf	ev_opc,W
		bz		rqnp_x			;read node parameters
		movlw	OPC_RQMN		
		subwf	ev_opc,w
		bz		rqmn_x			;read module name		
		movlw	OPC_NNLRN		;set to learn mode 
		subwf	ev_opc,W
		bz		nnlrn_x		
		movlw	OPC_NNULN		;clear learn mode 
		subwf	ev_opc,W
		bz		nnuln_x
		movlw	OPC_NNCLR		;clear all events on 0x55
		subwf	ev_opc,W
		bz		nnclr_x
		movlw	OPC_NNEVN		;read number of events left
		subwf	ev_opc,W
		bz		nnevn_x
		movlw	OPC_EVLRN		;is it set event?
		subwf	ev_opc,W
		bz		evlrn_x			;do learn
		movlw	OPC_REVAL
		subwf	ev_opc,W
		bz		reval_x
		movlw	OPC_EVULN		;is it unset event
		subwf	ev_opc,W			
		bz		evuln_x
		movlw	OPC_REQEV		;read event variables
		subwf	ev_opc,W
		bz		reqev_x
		movlw	OPC_NVSET		;set NV
		subwf	ev_opc,W
		bz		nvset_x
		movlw	OPC_NVRD		;read NVs
		subwf	ev_opc,W
		bz		nvrd_x
	
		movlw	OPC_NERD		;is it read events
		subwf	ev_opc,W
		bz		nerd_x

		movlw	OPC_RQEVN
		subwf	ev_opc,W
		bz		rqevn_x

		return					;no match of OPC

		; Too long for a branch so use gotos

go_on_x goto	go_on
snn_x	goto	snn
boot_x	goto 	boot
enum_x	goto 	enum

rqnpn_x goto	rqnpn
canid_x goto	canid
rqnp_x	goto	rqnp
rqmn_x	goto	rqmn
nnlrn_x goto	nnlrn
nnuln_x	goto	nnuln
evlrn_x	goto	evlrn

reval_x	goto	reval
rqevn_x	goto	rqevn
nnclr_x	goto	nnclr
nvset_x	goto 	nvset
nvrd_x	goto	nvrd
nnevn_x	goto	nnevn


qnn_x	goto	qnn


evuln_x	goto	evuln
reqev_x	goto	reqev


nerd_x	goto	nerd


;**************************************************************

go_on	call	do_it		;do command
		return	
		
;*******************************************************

; force self enumeration

enum	call	thisNN
		sublw	0
		bnz		notNN1			;not there
		call	self_en			;do self enum
		movlw	OPC_NNACK
		call	nnrel			;send confirm frame
		bcf		RXB0CON,RXFUL
		movlw	B'00001000'		;back to normal running
		movwf	Datmode
notNN1	return

;**********************************************************************

nnevn	call	thisNN
		sublw	0
		bnz		nnnret
		call	rdFreeSp		;in event handler
nnnret	return

;********************************************************************

qnn		movf	NN_temph,W		;respond if NN is not zero
		addwf	NN_templ,W
		btfss	STATUS,Z
		call	whoami
		return	

;***********************************************************************

;		Start bootloader

boot	
		call	thisNN
		sublw	0
		bnz		retboot			;not there
		
reboot1	movlw	0xFF
		movwf	EEADR
		movlw	0x3F
		movwf	EEADRH
		movlw	0xFF
		call	eewrite			;set last EEPROM byte to 0xFF
		reset					;software reset to bootloader
								;should clear return stack

retboot	return	

;***********************************************************

rqnpn		
		call	thisNN			;read parameter by index
		sublw	0
		bnz		npnret
		call	para1rd
npnret	return

;*************************************************************

;		Set a CAN_ID

canid	call	thisNN
		sublw	0
		bnz		canret				;abort
		movff	RXB0D3,IDcount
		call	here2				;put in as if it was enumerated
		movlw	OPC_NNACK
		call	nnrel				;acknowledge new CAN_ID
canret	return



;
;*********************************************************************

;		Set a NV

nvset	call	thisNN
		sublw	0
		bnz		notnv			;not this node
		call	putNV
		movlw	OPC_WRACK
		call	nnrel		;send WRACK
notnv	return

;*********************************************************************

;		Unlearn a NN

nnuln	call	thisNN
		sublw	0
		bnz		notret
		bcf		Datmode,4
		bcf		Datmode,7		;OK to scan
		
notln1									;leave in learn mode
		bcf		Datmode,5
	
notret	return
	
;***********************************************************************

;		Learn a NN

nnlrn	call	thisNN
		sublw	0
		bnz		nnlret			;abort
		bsf		Datmode,4

nnlret	return

;************************************************************************

reval	call	thisNN
		sublw	0
		bnz		revret			;abort
		movff	RXB0D3, ENidx
		movff	RXB0D4, EVidx
		call	evsend
revret	return

;***********************************************************************

;		Learn an event

evlrn	btfss	Datmode,4		;is in learn mode?
		return					;return if not
		movf	EVidx,w			;check EV index
		bz		noEV1
		movlw	EV_NUM+1
		cpfslt	EVidx
		bra		noEV1
		bra		learn2

noEV	movlw	6				;invalid EV#
		call	errmsg2
		return

noEV1	movlw	6
		call	errmsg
		return
	
			

learn2	btfss	Datmode,3		;FLiM?
		return
		
		call	enmatch			;is it there already?
		sublw 	0
		bz		isthere

learn3	btfsc	Datmode,6		;read EV?
		bra		rdbak1			;not here
		btfsc	Datmode,5		;if unset and not here
		bra		lrnend			;do nothing else 
		
	
learn5	decf	EVidx,F			;base 0
		call	learn			;put EN into flash
		
		sublw	0
		bz		lrnend
		
		movlw	4
		call	errmsg2	
		bra		l_out1
		
rdbak1	movlw	5				;no match
		call	errmsg2	
		bra		l_out1	

isthere	
		btfsc	Datmode, 6		;is it read back
		bra		rdbak					
		btfss	Datmode,5		;FLiM unlearn?
		bra		do_learn
		call	rdfbev			;get EVs
		movf	INDF0,W			;FSR0 points to start of EVs in RAM
		btfss	WREG,0			;is it switch event?
		bra		uln1
		call	unset_sw		;put switch info back to default
uln1	call	unlearn
		movlw	OPC_WRACK
		call	nnrel
		bra		l_out1
		
		
do_learn
		movf	EVidx,W			; indexes start at 1
		bnz		do_lrn1
		bra		rdeverr			;error
do_lrn1
		decf	EVidx,F			;base 0
		call	learn
		sublw	0				; check if no space left
		bz		do_lrn2
		movlw	4
		call	errmsg2
		return	
		
do_lrn2	call	enmatch
		call	rdfbev
		movf	INDF0,F			;FSR0 points to start of EVs in RAM
		bz		lrnend			;not a switch event
		movlw	2				;is it a sod?
		subwf	INDF0,W
		bz		lrnend			;don't set a switch if ev0 = 2
		movlw	2
		subwf	EVidx,W			;is it SV event?
		bnz		lrnend			;no
		call	set_sw			;sort out switch event
								;put in FLASH and ack.
;		
	
				
lrnend	bra	l_out2
		
		
do_unlearn
		call	unlearn
		movlw	OPC_WRACK		; send WRACK
		call	nnrel

		return
		
do_rdev
		tstfsz	EVidx
		bra		do_rdev1
rdeverr

		movlw	6
		call	errsub
		return
		
do_rdev1

		movlw	EV_NUM+1
		cpfslt	EVidx
		bra		rdeverr
		call	readev
		return
		
rdbak
		call	rdfbev			; read event info
		movff	EVidx,Tx1d5		;Index for readout	
		incf	Tx1d5,F			;add one back
		movf	EVidx,w
		movff	PLUSW0,Tx1d6
		movlw	OPC_EVANS		;readback of EVs
		movwf	Tx1d0
		movff	ev0,Tx1d1
		movff	ev1,Tx1d2
		movff	ev2,Tx1d3
		movff	ev3,Tx1d4
		movlw	7
		movwf	Dlc
		call	sendTXa	
		bra		l_out1



l_out	bcf		Datmode,4
l_out1	bcf		Datmode,6
l_out2	bcf		RXB0CON,RXFUL
		bcf		Datmode,0
		clrf	PCLATH
		return

		


		




;***************************************************************

;		Unlearn an event

evuln	
		btfss	Datmode,4
		return			;prevent error message
		bsf		Datmode,5
		bra		learn2

;*******************************************************************

;		Request an EV

reqev	btfss	Datmode,4
		return					;prevent error message
		movf	EVidx,w			;check EV index
		bz		rdev1
		movlw	EV_NUM+1
		cpfslt	EVidx

rdev1	bra		noEV1
		bsf		Datmode,6
		bra		learn2

;********************************************

;		Clear a node.  Must be in learn mode for safety

nnclr	call	thisNN
		sublw	0
		bnz		clrret
		btfss	Datmode,4
		bra		clrerr
		call	clr_sub
		
		movlw	OPC_WRACK
		call	nnrel		;send WRACK
		bra		notln1
clrret	return

clrerr	movlw	2			;not in learn mode
		call	errmsg
		return

;**********************************************************************

;	error message send

errmsg	call	errsub
		bra		errret
errmsg1	call	errsub
		bcf		Datmode,6
		bra		errret
errmsg2	call	errsub
		
errret	clrf	PCLATH
		return		

errsub	movwf	Tx1d3		;main error message send. Error no. in WREG
		movlw	OPC_CMDERR
		movwf	Tx1d0
		movlw	4
		movwf	Dlc
		call	sendTX
		return
;**************************************************************

rqevn	call	thisNN				;read event numbers
		sublw	0
		bnz		rqevret
		call	evnsend				;send event number
rqevret return

;*****************************************************************

nerd	call	thisNN
		sublw	0
		bnz		nrdret			;abort
		call	enread
nrdret	return

;******************************************

snn		btfss	Datmode,2		;in NN set mode?
		return			;no
		call	putNN			;put in NN
		movlw	Modstat
		movwf	EEADR
		movlw	B'00001000'
		call	eewrite			;set to normal status
		bcf		Datmode,1		;out of setup
		bcf		Datmode,2
		bsf		Datmode,3		;run mode

	
		movlw	.10
		movwf	Keepcnt			;for keep alive
		movlw	OPC_NNACK
		call	nnrel 			;confirm NN set
startNN	bsf		LED_PORT,YELLOW	;LED ON
		bcf		LED_PORT,GREEN	;LED off
		return

;******************************************************************

rqnp	btfss	Datmode,2		;only in setup mode
		return
		call	parasend
		return

;*******************************************************************

;		main setup subroutine
;		will depend on the hardware used
;		sets all I/O, timers. CAN etc.

setsub 	movlb	.15
		bsf		OSCTUNE,PLLEN	;put PLL on
		clrf	ANCON0			;disable A/D
		clrf	ANCON1
		clrf	CM1CON			;disable comparator
		clrf	CM2CON
		clrf	INTCON2			
		bsf		INTCON2,7		;weak pullups off
		clrf	WPUB			;pullups clear
		movlb	0	

		;port settings will be hardware dependent. RB2 and RB3 are for CAN.
		;set S_PORT and S_BIT to correspond to port used for setup.
		;rest are hardware options

		clrf	LATB			;outputs low at start
		movlw	B'00001000'		;Port B  set to outputs except RB3 which is CANRX 
								;RB2 is CANTX
		movwf	TRISB

		setf	TRISC			;port C is all inputs fo rows 


		lfsr	FSR0, 0			; clear page 1
		
nextram	clrf	POSTINC0
		tstfsz	FSR0L
		bra		nextram	
		
		clrf	INTCON			;no interrupts yet
				
	
		movlw	B'00100000'		;Port A   PA5 is setup PB
		movwf	TRISA			;RA0,1,2 and 3 are switch column select
								;RA4 is the capacitor
									

		movlw	B'00000000'		;set unused outputs off
		movwf	LATB			;set LED row drivers off
		movlw	B'00001000'		;RB0,1 go nowhere
								;RB2 is CAN TX
								;RB3 is CAN RX
								;RB4, 5 go nowhere 
								;RB6,7 for debug and ICSP and LED drivers
								;PORTB has pullups disabled on inputs
								;
		movwf	TRISB
		bcf		LED_PORT,GREEN
		bcf		LED_PORT,YELLOW
		bsf		PORTB,2			;CAN recessive

;	
	

	
		
;	next segment is essential.
		
		bsf		RCON,IPEN		;enable interrupt priority levels
		clrf	BSR				;set to bank 0
		clrf	EECON1			;no accesses to program memory	
		clrf	Datmode
		clrf	Latcount
		bsf		CANCON,7		;CAN to config mode
		movlw	B'10110000'
		movwf	ECANCON	
		bsf		ECANCON,5		;CAN mode 2 
		movf	ECANCON,W
		movwf	TempECAN 

		movlb	.14
		clrf	BSEL0			;8 frame FIFO
		clrf	RXB0CON
		clrf	RXB1CON
		clrf	B0CON
		clrf	B1CON
		clrf	B2CON
		clrf	B3CON
		clrf	B4CON
		clrf	B5CON
		
			
		movlw	CANBIT_RATE		;set CAN bit rate at 125000 
		movwf	BRGCON1
		movlw	B'10011110'		;set phase 1 etc
		movwf	BRGCON2
		movlw	B'00000011'		;set phase 2 etc
		movwf	BRGCON3
		movlb	0

		movlw	B'00100000'
		movwf	CIOCON			;CAN to high when off


		
mskload	lfsr	FSR0,RXM0SIDH		;Clear masks, point to start
mskloop	clrf	POSTINC0		

		cpfseq	FSR0L			;0xEFF is last mask address
		bra		mskloop
		
		clrf	CANCON			;out of CAN setup mode
		clrf	CCP1CON

		
		clrf	IPR5			;low priority CAN RX and Tx error interrupts
		clrf	IPR1			;all peripheral interrupts are low priority
		clrf	IPR2
		clrf	PIE2
		clrf	T1GCON	
		movlw	B'00110010'		;Timer 1 set Timer 1 for LED flash
		movwf	T1CON
		movlw	0x00
		movwf	TMR1H			;Timer 1 is a 16 bit timer
		movwf	TMR1L
		bsf		T1CON,TMR1ON	;run timer
		movlw	.16
		movwf	T1count			;flash delay counter
		movlw	4
		movwf	Debcnt			;debounce multiplier		
	
		bsf		PIE5,ERRIE		;Tx error enable
		bsf		PIE5,FIFOWMIE	;FIFO full interrupt
		
		clrf	Tx1con
	


;next segment required
		
		movlw	B'00000001'
		movwf	IDcount			;set at lowest value for starters
		
	
		clrf	INTCON3			;
		clrf	T3GCON
			
		clrf	PIR1
	
		bcf		RXB0CON,RXFUL		;ready for next
		bcf		COMSTAT,RXB0OVFL	;clear overflow flags if set
		bcf		COMSTAT,RXB1OVFL
		clrf	PIR5			;clear all ECAN flags

		clrf	EEADRH			;upper EEPROM page to 0

		movlb	.15				;set busy frame
		clrf	TXB0SIDH
		clrf	TXB0SIDL
		clrf	TXB0DLC
		movlb 0
		
		movlw	B'11000000'
		movwf	INTCON

		return

;*************************************************************************

;		Set switches to current values

sw_set	movlw	.16				;column count
		movwf	Count2
		lfsr	FSR0,Switch1	;switch variables in RAM
		clrf	Colcnt
sw_set1	movf	Colcnt,W
		movwf	LATA			;set column
		call	dely			;wait for column to settle
		movf	PORTC,W			;get  switches
		comf	WREG
		movwf	POSTINC0		;save it
	
		incf	Colcnt
		decfsz	Count2
		bra		sw_set1
		setf	Swtemp
		return

;*********************************************************************

;	Flashes yellow LED in setup

flash	btfss	PIR1,TMR1IF		;Timer 1 out?
		return					;no
		bcf		PIR1,TMR1IF		;reset timer
		decfsz	T1count			;loop counter
		bra		flash
		movlw	.16
		movwf	T1count			;reset counter
		
flash1	btg		LED_PORT,YELLOW	;flash yellow

		return

;*************************************************************************

unflash	bsf		LED_PORT,YELLOW	; yellow steady
		
		return

;*************************************************************************

;		Pushbutton debounce
	
pb_deb	btfss	PIR1,TMR1IF
		bra		pb_deb			;loop
		bcf		PIR1,TMR1IF
		btfss	S_PORT,S_BIT	;test PB again
		bra		pb_ok			;still on
		retlw 1					;bounce
pb_ok
		retlw 0					;is OK

;***********************************************************

		;Time for PB debounce

pb_time	call	pb_deb				;PB debounce
		movf	WREG
		bnz		bounce				;was bounce
		movlw	3					;loops for delay
		movwf	Count
		clrf	TMR0H
		clrf	TMR0L
		movlw	B'00000111'			;set timer 0
		movwf	T0CON
		bsf		T0CON,TMR0ON		;start timer	1 second
pb_1a	btfsc	S_PORT,S_BIT		;PB released?
		bra		pb_2				;yes. Is too short 
		btfss	INTCON,TMR0IF		;timer out?
		bra		pb_1a
		bcf		INTCON,TMR0IF
		decfsz	Count
		bra		pb_1a				;keep looking
		retlw	0
pb_2	retlw 	1
bounce	retlw	2

;****************************************************************

;initial set to running mode. Gets NN etc and sets to run

slimset	clrf	NN_temph
		clrf	NN_templ
		call	clr_sub				;set defaults for events and SVs
		setf	EVflags				;no flash till taught
		bcf		LED_PORT,YELLOW		;yellow LED off
		bsf		LED_PORT,GREEN		;green LED on. Green for SLiM
		
set1	btfsc	S_PORT,S_BIT		;look for PB on 
		bra		set1				;loop till on
		call	pb_time				;time the on state
		btfsc	Datmode,3
		goto	set2				;is pb in main loop
		sublw	0					;is it held in long enough?
		bnz		abort				;bounce or too short
		bcf		LED_PORT,GREEN		;green off
set1a	btfss	S_PORT,S_BIT		;button released?
		bra		set1a				;loop
		call	self_en				;do an enumerate
		call	nnreq				;request node number RQNN
set1b	call	flash				;flash yellow
		btfsc	S_PORT,S_BIT		;PB again?
		bra		set1g				;go on
		bcf		LED_PORT,YELLOW		;in case it was on.
		call	pb_time				;how long is it in?
		sublw	0
		bnz		set1g				;too short
		bra		slimset				;try again
set1g	btfsc	COMSTAT,7			;look for CAN input. 
		call	getcan				;wait for answer
set1c	btfss	Datmode,0			;got an answer
		bra		set1b				;loop till		(maybe needs an abort mechanism?)
		movlw	OPC_RQNP			;request for parameters
		subwf	ev_opc,W
		bz		set1e
		movlw	OPC_SNN				;set new NN
		subwf	ev_opc,W
		bz		set1f
set1d	bcf		RXB0CON,RXFUL
		bcf		Datmode,0
		bra		set1c				;look again

set1e	call	parasend			;send parameters to FCU
		bra		set1d				;wait for SNN

set1f	call	putNN				;put in new NN. Sets Datmode to 8
		
		call	newid				;move all ID to EEPROM etc
		call	unflash				;yellow on 
		movlw	OPC_NNACK
		call	nnrel				;send NNACK
		bcf		RXB0CON,RXFUL		;clear CAN 
		bcf		Datmode,0
		retlw	0					;continue setup as if running mode
abort	bcf		LED_PORT,YELLOW
		retlw	1					;too short or a bounce


set2	nop							;here if in running mode
		
			
		sublw	1
		bz		set2a 
		bra		set_off				;is	held so  cancel run mode 
set2a	
		call	self_en				;do an enumerate
		call	nnreq				;request node number RQNN
set2b	call	flash				;flash yellow
		btfss	S_PORT,S_BIT		;pb in again?
		bra		set_bk1				;set back to main
		btfsc	COMSTAT,7			;look for CAN input. 
		call	getcan				;wait for answer
set2c	btfss	Datmode,0			;got an answer
		bra		set2b				;loop till		(maybe needs an abort mechanism?)
		movlw	OPC_SNN				;set new NN
		subwf	ev_opc,W
		bz		set2f
set2d	bcf		RXB0CON,RXFUL
		bcf		Datmode,0
		bra		set2b				;look again



set2f	call	putNN				;put in new NN. Sets Datmode to 8
		call	newid				;move all ID to EEPROM etc
		call	unflash				;yellow on 
		bcf		Datmode,1			;out of NN waiting
		bcf		Datmode,2
		movlw	OPC_NNACK
		movwf	Tx1d0
		call	nnrel				;send NNACK
		retlw	0					;back to main

set_bk1	btfss	S_PORT,S_BIT		;released 
		bra		set_bk1
		bra		set2f				;back
		
		




set_off	bcf		LED_PORT,YELLOW
		bsf		LED_PORT,GREEN
		btfss	S_PORT,S_BIT		;released
		bra		set_off
		clrf	Datmode
		movlw	LOW Modstat
		movwf	EEADR
		movf	Datmode,W
		call	eewrite
		movlw	OPC_NNREL
		movwf	Tx1d0				;send release
		call	nnrel
		movlw	0
		incf	EEADR
		call	eewrite				;clear old NN
		movlw	0
		incf	EEADR
		call	eewrite
		
		retlw	1
		
;*****************************************************************		

setup1	bcf		LED_PORT,YELLOW	;clear if on
		movlw	Modstat			;get setup status
		movwf	EEADR
		call	eeread
		movwf	Datmode
		sublw	8				;is it in run mode?
		bz		setup2			;yes	
		movlw	0				;else set it
		movwf	Datmode			;not set yet
		call	eewrite
		call	slimset			;wait for setup PB
		movf	WREG			;is it long enough
		bnz		setup1			;no
setup2	return

;**********************************************************

sub_off movlw	.128		;sets all taught events to off.
		movwf	Scount
		clrf	Switchno
of_loop	movlw	LOW swstate		;get saved SVs
		addwf	Switchno,W
		movwf	EEADR
		call	eeread
		btfss	WREG,7			;is it a taught event?
		bra		nxt_off
		bsf		WREG,6			;set to off.
		call	eewrite

nxt_off	incf	Switchno	;next switch
		decfsz	Scount
		bra		of_loop
		call	readsw		;put all EEPROM SVs in RAM
		return

;********************************************************

;set layout to saved switches

sub_set
		clrf	TMR0H
		clrf	TMR0L		;set for one second
		movlw	B'10000111'
		movwf	T0CON		;start it
setdly	btfss	INTCON,TMR0IF  	;allow nodes to settle
		bra		setdly
		bcf		INTCON,TMR0IF
		call	set_loop			
		return

;********************************************************************



;		a delay routine
			
dely	movlw	.10
		movwf	Count1
dely2	clrf	Count
dely1	decfsz	Count,F
		goto	dely1
		decfsz	Count1
		bra		dely2
		return	
		
	
; Switch event. Indexed by switch number. 128 switches of 4 bytes / switch = 512 Bytes

	ORG 0x2900

;default table

swdata
	
		


; Events data, room for 128 events at 32 bytes/event = 4KB	
		
	ORG	0x3000

evdata



	

;************************************************************************		
	ORG 0xF00000			;EEPROM data. Defaults
	
CANid	de	B'01111111',0	;CAN id default and module status
NodeID	de	0,0			;Node ID
ENindex	de	0,0		; hi byte contains free space
					; lo byte contains number of events
					; hi byte + lo byte = EN_NUM
FreeCh	de 0,0

	ORG 0xF00010

hashtab	de 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
		de 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
		de 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0		
		de 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
	
		
; number of events in each hash table entry


hashnum	de	0,0,0,0,0,0,0,0
		de	0,0,0,0,0,0,0,0
		de	0,0,0,0,0,0,0,0
		de	0,0,0,0,0,0,0,0


swstate	de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1


sod_sw	de	0,0
NVstart	de	0x80,0							;one NV, initialised to 0x80

		ORG	0xF003FE
		de		0,0		;for boot load
		end


