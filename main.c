/**
 * \file main.c
 *
 *  Created on: 02.10.2014
 *      Author: Jacek
 *
 *
 *  Opis:
 *  Procesor
 *  - zegar
 *  	zegar wewn 16/1 MHZ
 *  	zegar zewn
 *
 *  - Porty
 *  	Port PWM sterowany sprzetowo zegarem PWM OUT
 *  	Port blink
 *  	Port wejscia
 *
 *  - wyjscie PIN
 *  - Zegar PWM port ->
 *
 *
 *  Logika:
 *  Uruchomienie / Inicjalizacja
 *  - inicjalizacja portow
 *  - po wlaczeniu ustawienie domyslnego wyjscia
 *  - przejscie w tryb Pracy
 *
 *
 *  Praca:
 *  - Ustawienie PWM
 *  - Ustawienie Przerwania do odbioru komend
 *
 *  - nowa komenda
 *  	logika stan akt, krok, cel, -> nowy (przyklad)
 *  			70		2		>=72	72
 *  			70		2		71		70
 *  			70		2		69		70
 *  			70		2		<=68	68
 *
 *  - Fade IN/OUT
 *
 *  przerwanie z portu:
 *  	ustawienie timera na 100us
 *  	zmienna odbior komendy
 *
 *  odbior komendy
 *  	rozpoznanie bit start / stop
 *  		rozpoznanie dlugosci
 *
 *  	rozpoznanie bitu komendy
 *  		stanu 1/0
 *  		zapamietanie bitu
 *
 *  	przerwanie komendy
 *  		licznik przekroczony gdy stan portu wysoki
 *
 *  	przetworzenie komendy (porownanie ze znanymi komendami)
 *  		zapamietanie
 *  		zmiana stanu wyjscia / PWM
 *  		blink
 *
 *
 *  Nauka komend pilota:
 *  - kolejno progi +,-, PWM_1/2/3
 *  	nowa komenda? -> zapamietanie
 *  	blink
 *
 *  - kontrolka Blink
 *
 *  Uspienie / Pamiec:
 *  - wylaczenie przerywania portu IR
 *  - wlaczenie monitorowanie stanu pracy lampy (wl / wyl)
 *  - wlaczenie trybu niskiego poboru mocy
 *  procesor nie ma pamieci. Tryb poboru niskiej mocy zeby 1 dzien pamietal w ram.
 *
 *
 *
 * Tablica Stanow	Przejscie	nastepny	Timer	CCR0	CCR1		Skala	INT	LED	Watchdog
	eInit,			-			OP			SM		-							OFF	1x
	eOperation,		INT						A		128		PWMwyjscie			ON	-
	eInterrupt,		-			ODBIOR		SM1MHz	1000	-					ON	-
	eOdbior,		INT			OP
	eNauka,			INT
	eUspienie,		INT			OP


	Timery
	- CCR0 dziala na P1.1

 */



//#include "msp430g2231.h"
#include "msp430g2452.h"

//#include "lib_rc.h"

#define PORT_CLR	(a,b) a &= ~(b)
#define PORT_SET 	(a,b) a |= (b)
#define PORT(a,b) 	((b=0)?(P1OUT &= ~(a)):(P1OUT |= (a)))
#define LED_TOG
#define LED__ON
#define LED_OFF

// wejscia wyjscia
#define LED			BIT0	// LED czerwony port 1.0

#define WYJSCIE		BIT6	// PWM port 1.6
#define IR_DATA		BIT4	// wejscei IR port 1.4

// Predef Mocy
#define PWM_MAX		0xFFFF
#define PWM_MIN		2*SKALA
#define SKALA		(PWM_MAX/100)

#define PWM_1		10*SKALA
#define PWM_2		40*SKALA
#define PWM_3		70*SKALA

#define	PWM_PLUS	2*SKALA
#define PWM_MINU	-4*SKALA


// definicje do zegara
#define PWM_TAKTA	128		// Cykl w trybie LPM3 przy zegarze ACLK = 32768Hz
#define PWM_TAKTS	512		// Cykl w trybie normalnym / LPM0 przy zegarze SMCLK = 1MHz
#define CLK_100U	100		// 100us (10kHz) Cykl w trybie normalnym przy zegarze SMCLK = 1MHz

// Tryby pracy
enum eOPmodeT
{
	eInit,					// co po wlaczeniu
	eOperation,				// oczekiwanie na IR kod lub przycisk
	eInterrupt,				// przerywanie IR kod
	eOdbior,				// interpretacja komendy
	eNauka,					// nauka / logika
	eSave,					// zapis nowej komendy
	eUspienie,				// pauza
}eOPmode;


enum eIR_rcvStateT
{
	eStart,
	eAdress,
	eKomenda,
	eStop,
	eKoniec,
} eIR_rcvState;

// Zmienne globalne
int CommandCode[4];			// bufor do odbioru komendy
int CommandBit;				// ktory bit komendy
int CommandNew;				// informacja o odebranej nowej komendzie

int PWMwyjscie;				// stan wyjscia 16bit
int PWMwyjscieN;			// nowy (docelowy) stan wyjscia po odebraniu komendy

int LED_Kod;

int rcvLicznikC;			// Licznik calkowity mierzacy ciagle od poczatku komendy
int rcvLicznikO;			// ostatni stan licznika calkowitego
int rcvLicznik;				// Licznik do mierzenia dlugosci bitu
int rcvPortOst;				// Ostatni stan portu do wykrycia zmiany stanu

int TablicaKomendSzukaj[]={0,0,1,2,3,4};	// Tablica do poszukiwania komendy
int ii,jj;					// zmienne pomocnicze

//! czasy w komendzie w jednostce 100us
static const int dlugosc_start = 30;		// dlugosc_1 - start
static const int dlugosc_1 = 9;		 		// dlugosc_2 - dlugi bit = 1
static const int dlugosc_0 = 5;				// dlugosc_3 - krotki bit = 0
static const int dlugosc_pauza = 10;		// dlugosc_4 - pauza
static const int dlugosc_koniec = 10;		// dlugosc_5 - koniec komendy
static const int dlugosc_blad = 300;		// dlugosc_6 - blad, przerwanie odbioru
static const int dlugosc_komenda = 48;		// dlugosc_7 - ilosc bitow komendy musi byc mniejsza niz 64

//! blok modyfikacji wyjscia
static const int TablicaPWM[] = {
		PWM_1,
		PWM_2,
		PWM_3,
		PWM_PLUS,
		PWM_MINU
};

//! blok w pamieci ktory bedzie nadpisany przy zapamietaniu nowej komendy
static const int TablicaKomend[][]={
	{ 0x0000, 0x0000, 0x0000, 0x0000}, // PWM_1
	{ 0x0000, 0x0000, 0x0000, 0x0000}, // PWM_2
	{ 0x0000, 0x0000, 0x0000, 0x0000}, // PWM_3
	{ 0x0000, 0x0000, 0x0000, 0x0000}, // PWM_PLUS
	{ 0x0000, 0x0000, 0x0000, 0x0000}, // PWM_MINUS
};





int main(void) {

/*
	 * IR Decoder
	 * Decode and respond to RC5 IR remote control commands, using a TSOP38238SS1V receiver IC
	 * Copyright: Jerry Pommer <jerry@jerrypommer.com> 2012-12-04
	 * License: GNU General Public License v3
*/
	WDTCTL = WDTPW | WDTHOLD;    			// disable watchdog


	for(;;) {
		volatile unsigned int i;	// volatile to prevent optimization

		switch (eOPmode)
		{
		case eInit:
/*
 *  Uruchomienie / Inicjalizacja
 *  - inicjalizacja portow
 *  - po wlaczeniu ustawienie domyslnego wyjscia
 *  - przejscie w tryb Pracy
 */
			DCOCTL = CALDCO_1MHZ;     				// select DCO approx. 1MHz
			BCSCTL1 |= CALBC1_1MHZ;
			BCSCTL2 &= ~SELS;		  				// SMCLK source

			PORT_CLR(P1DIR,IR_DATA);				// IRDATA is an input
			P1IE = IR_DATA;							// enable interrupts, watching IRDATA for a change
			P1IES = IR_DATA;						// watch for falling edge change on IRDATA

			P1OUT |= LED | WYJSCIE;					// Set all these HIGH
			P1DIR |= LED | WYJSCIE;					// Set all these as outputs
													// P1.0,1 oraz P1.4 maja wyjscie sterowane prosto z zegara
			PORT_CLR(P1DIR,WYJSCIE);                // Zerowanie wyjscia
			PORT_SET(P1SEL,6);                      // P1.6 funkcja (TA0.1 output) aktywna (SET)

			// Ustawienie Zegara
			//TACTL = TASSEL_1 + MC_1				// ACLK 32768 Hz, up mode
			TACTL = TASSEL_2 + MC_1 + TAIE;			// SMCLK 1MHz, up mode
			CCTL0 = CCIE;                          	// CCR0 interrupt enabled
			CCR0 = PWM_TAKT;						// Ustalenie Taktu; licznik liczy od 0 do PWM_TAKT po tym interrupt
			// koniec inicjalizacji

			PWMwyjscie = PWM_1;
			eOPmode = eOperation;
			CCR1 = PWMwyjscie;						// Ustawienie wyjscia

			__bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupt SMCLK
			// __bis_SR_register(LPM3_bits + GIE);  // Enter LPM3 w/ interrupt ACLK

			break;

		case eInterrupt:
/*
 *  przerwanie z portu:
 *  - ustawienie timera na 100us
 *  - zmienna odbior komendy
 *
 *	kasowanie zmiennych: Liczniki, bufor odbioru
 *
 */
			rcvLicznikC = 0;
			rcvLicznikO = rcvLicznikC;
			rcvLicznik = 0;
			rcvPortOst = 1;

			eIR_rcvState = eStart;

			CommandNew = 0;
			CommandCode [0]=0;
			CommandCode [1]=0;
			CommandCode [2]=0;
			CommandCode [3]=0;

			eOPmode = eOdbior;
			break;

		case eOdbior:
/*
 *  odbior komendy:
 *
 *  w petli:
 *  nowy stan licznika? tak, inkrementowac licznik i dalej : nie, czekac,
 *  rozpoznanie bitu, nowy stan? tak, dalej : nie, czekac
 *  	zapamietanie nowego stanu portu
 *  	ocena dlugosci stanu (rozpoznanie bitu)
 *  	dlugosc_1 - start
 *  	dlugosc_2 - dlugi bit = 1
 *  	dlugosc_3 - krotki bit = 0
 *  	dlugosc_4 - pauza
 *  	dlugosc_5 - koniec komendy
 *  	dlugosc_6 - blad, przerwanie odbioru
 *  kasowanie licznika bitu
 *
 *  zapamietanie bitu komendy
 *
 *  przetworzenie komendy (porownanie ze znanymi komendami)
 *
 *  zapamietanie
 *  zmiana stanu wyjscia / PWM
 *  blink
 *
 *
 */
			if(rcvLicznikO != rcvLicznikC)
			{	// nowy stan licznika,
				rcvLicznik++;
				if ((rcvPortOst != 0) && !(P1IN & IR_DATA))
				{// rozpoznanie bitu, nowy stan = 0? tak, dalej : nie, czekac na 0

					// nowa komenda / pierwszy bit ? tak, czekaj na nastepny : nie, odbior komendy
					if (eIR_rcvState == eStart)
					{
						eIR_rcvState = eKomenda;
					}
					else
					{  // zapamietanie bitu komendy

						if ((rcvLicznik > dlugosc_0) && (rcvLicznik < dlugosc_1))
						{ // jedynka
							CommandCode[CommandBit%16] <<= 1;
							CommandCode[CommandBit%16]++;
						}
						else
						{ // zero
						}
						CommandBit++;

						if (CommandBit == 63)
						{	// bufor pelny / blad
							eIR_rcvState = eStop;
						}
					}
					rcvLicznik=0;				// kasowanie licznika
				}

				if (rcvLicznik >= dlugosc_blad)
					//eOPmode = eOperation;		// za dlugo / blad
					eIR_rcvState = eStop;		// za dlugo koniec odbioru / blad

				rcvPortOst = P1IN & IR_DATA;	// zapamietanie aktualnego stanu portu

				if (eIR_rcvState == eStop)
				{ // nowa komenda; porownanie ze znanymi komendami

					TablicaKomendSzukaj[0]=CommandNew;

					for(ii=0; ii < sizeof(TablicaKomendSzukaj); ii++)
					{
						for(jj=0; jj<4; jj++)
						{
							if (CommandCode[jj]!=TablicaKomend[ii][jj])
								break;
						}
						if (jj=4)
						{	// komenda znalezina (ii)
							eIR_rcvState = eKoniec;
							ii = sizeof(TablicaKomendSzukaj);

							if (ii<3)
							{	// wartosci predefiniowane
								PWMwyjscieN = TablicaPWM[ii];
							}
							else
							{	// wartosci modyfikowane + / -
								PWMwyjscieN = PWMwyjscie+TablicaPWM[ii];
							}
						}
					}

					eOPmode = eOperation;		// nowa komenda przetworzona
				}
			}
			break;

		case eOperation:
/*
 *  Praca:
 *  - Ustawienie PWM
 *  - Ustawienie Przerwania do odbioru komend
 *
 *  - nowa komenda
 *  	logika stan akt, krok, cel, -> nowy (przyklad)
 *  			70		2		>=72	72
 *  			70		2		71		70
 *  			70		2		69		70
 *  			70		2		<=68	68
 *
 *  - Fade IN/OUT
 *
 *	- skalowanie wyjscia
 */


			break;

		case eNauka:
/*
 *  Nauka komend pilota:
 *  - kolejno progi +,-, PWM_1/2/3
 *  	nowa komenda? -> zapamietanie
 *  	blink
 *
 */

			break;

		case eUspienie:
			__bis_SR_register(CPUOFF + GIE);	// Low Power Mode 0 with interrupts enabled
			break;

		default:
			break;

		}

/*
		switch (blink)
		{
		case ONE:
			break;
		case ONE_SEC:
			break;
		default:
			break;
		}
*/
	}


	return 0;
}


/*
 *	przerwanie uzywane do sterowania PWM;
 *	przerwanie wywolane jesli stan licznika TAR jest rowny Rejestrowi CCR0=TAR (koniec cyklu)
 *	licznik bignie dalej od 0, port przelaczyc na stan wysoki
 *
 */

// Timer A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A0 (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMERA0_VECTOR))) Timer_A0 (void)
#else
#error Compiler not supported!
#endif
{
											// eOPmode = eOperation;
	PORT(WYJSCIE,1);						// przerzucanie stanu portu 0 -> 1
	rcvLicznikC++;							// inkrementacja glownego licznika
}

/*
 *	przerwanie uzywane do sterowania PWM;
 *	przerwanie wywolane jesli stan licznika TAR jest rowny Rejestrowi CCR1=TAR (regulacja wypelnienia PWM)
 *	licznik bignie dalej
 *
 */

// Timer_A2 Interrupt Vector (TAIV) handler
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMERA1_VECTOR
__interrupt void Timer_A1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMERA1_VECTOR))) Timer_A1 (void)
#else
#error Compiler not supported!
#endif
{
	switch( TAIV )
	{
	case  2:
		PORT(WYJSCIE,0);						// przerzucanie stanu portu 1 -> 0
		break;
	case 10:
		break;
	}
}

/*
 *  przerwanie z portu:
 *  wylaczenie przerywania z portu
 *  ustawienie timera na 100us
 *
 *  kasowanie zmiennych
 *  przejscie w tryb
 */

#pragma vector = PORT1_VECTOR;
void __interrupt Port_1(void)
{
	P1IE &= ~IR_DATA;				// Turn off P1 interrupts while we work
	P1IFG &= ~IR_DATA;				// clear the P1 interrupt flag for IRDATA

	//TODO: wait for second start bit within T_INTERVAL; abandon if it doesn't come.

	/*
	// start timer
    TACCTL0 |= CCIE;				// enable timer interrupts
	TACCR0 &= ~CCIFG;				// clear any pending timerA interrrupt flags
	TACTL |= MC_1;					// start the timer in UP mode
*/
	eOPmode = eInterrupt;

}


/*
 *
 * TACTL - Timer A Control Register

The Timer A clock source (TASSELx)
– TACLK (TASSEL_0)
– ACLK  (TASSEL_1)
– SMCLK (TASSEL_2)
– INCLK (TASSEL_3)

These bits select the divider for the input clock (IDx)
– (ID_0) is divide by 1
– (ID_1) is divide by 2
– (ID_2) is divide by 4
– (ID_3) is divide by 8

Clock mode control (MC_1)
– (MC_0) Stop mode: the timer halts
– (MC_1) Up mode: the timer counts up CCR0
– (MC_2) Continuous mode: the timer counts up to the max value of CCR0
– (MC_3) Up down mode: the timer counts up CCR0 and then down to 0x0000h


6 Types of Low Power Modes
- AM   – All clocks are active
- LPM0 - The CPU is disabled and MCLK is disabled.
- LPM1 - The DCO’s generator is disabled.
- LPM2 – SMCLK is also disabled.
- LPM3 – Same as LPM2. Don’t know why this was added.
- LPM4 – The ACLK and the crystal oscillator are also disabled.
• NOTE: As Low Power mode rises, all previous low power modes are also included – (Ex. LPM3 includes LPM0-LPM2)


PWM Code: Out Modes
– OUTMOD_0
– OUTMOD_1
– OUTMOD_2
– OUTMOD_3: The output is set when the timer counts to the TACCRx value. It is reset when the timer counts to the TACCR0 value
– OUTMOD_4
– OUTMOD_5
– OUTMOD_6
– OUTMOD_7: The output is reset when the timer counts to the TACCRx value. It is set when the timer counts to the TACCR0 value.
• Only worry about OUTMOD_3 and OUTMOD_7


 *
 */
