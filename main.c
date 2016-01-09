/**
 * \file main.c
 *
 *  Created on: 1.01.2016
 *      Author: Jacek
 *
 *
 *  Opis:
 *  Procesor
 *  - zegar
 *  	zegar wewn 21/1 MHZ
 *  	zegar zewn
 *
 *  - Porty
 *  	Port OUT 1.6 PWM sterowany programowo zegarem PWM OUT (TODO sprzetowo przez Timer A1)
 *  	Port OUT 1.7 Kopia PWM -> DEBUG
 *  	Port OUT 1.0 blink (LOW -> LED1 swieci)
 *  	Port  IN 1.3 wejscie SW NAUKA
 *  	Port  IN 1.4 wejscie IR_DATA
 *
 *
 *  Logika:
 *  Uruchomienie / Inicjalizacja
 *  - inicjalizacja portow
 *  - po wlaczeniu ustawienie domyslnego wyjscia
 *  - przejscie w tryb uspienia / Pracy i oczekiwania na przerwanie (odbior komendy / przycisku nauka)
 *  - Po przerwaniu z portu IR -> odbior komandy
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
 *  - Fade IN/OUT -> zbyteczne
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
 * Historia
 * 	1.2.2015 -> fix dla odbioru komend
 * 	1.4.2015 -> fix dla unsigned int przy PWM, zapamietanie ostatniej wartosci w infoD, + zapamietanie ostatniej wartosci
 *
 * + sprawdzic dlugosc petli systemowej ~800 tikow
 * + debug -> do pamieci ile trwaly kolejne sygnaly
 * + nieliniowa skala (kwadratowa)
 * + min-max: brak przerywan ponizej progu minimalnego (dodatkowy tryb) oraz powyzej maksymalnego
 * + zapamietanie ostatniej wartosci
 *
 * 	1.1.2016 -> optymalizacja kompilatora (99% -> 87%), jednoczesne wysterowanie portu 1.7 w celych testowych,
 *
 */

/* todo preflash
 * - fade in / out -> zbyteczne ?
 * - polowa (1/3?) czestotliwosci dla krotkich PWM
 * - Praca z kwarca ???
 * - tryb uspienia LPM ???
*/


#include "msp430g2211.h"
//#include "msp430g2231.h"
//#include "msp430g2452.h"

//#include "lib_rc.h"

#define PORT_CLR(a,b) 	a &= ~(b)
#define PORT_SET(a,b) 	a |= (b)
#define PORT(a,b) 	((b>0)?(P1OUT |= a):(P1OUT &= ~a))
#define LED_TOG
#define LED__ON
#define LED_OFF

// wejscia wyjscia
#define LED			BIT0	// LED czerwony port 1.0
#define WYJSCIE		BIT6	// PWM port 1.6

#define IR_DATA		BIT4	// wejscie IR port 1.4
#define SW_NAUKA	BIT3	// wejscie Przyciska nauka port 1.3

// Predef Mocy max = 250 / 1000
#define PWM_MAX		250
#define SKALA		PWM_MAX/100
#define PWM_MIN		2

// Predefinicje "mocy"
#define PWM_1		20*SKALA
#define PWM_2		60*SKALA
#define PWM_3		90*SKALA

#define	PWM_PLUS	4*SKALA
#define PWM_MINU	-2*SKALA




// definicje do zegara
#define MHZ21		0   //  MHZ21		1

#define MHZ			1000000u
#define CLK_A		32768			// Cykl w trybie LPM3 przy zegarze ACLK = 32768Hz

#if MHZ21
#define CLK_S		21				// Cykl w trybie normalnym / LPM0 przy zegarze SMCLK = 21MHz
#else
#define CLK_S		16*1123			// Cykl w trybie normalnym / LPM0 przy zegarze SMCLK = 16MHz / DCO=6, RSEL=F 17968
#endif

//#define CLK_MAIN	CLK_S*MHZ			// glowny zegar

#define CLK_NAUKA	 100			// zegar us (skorygowany) (zegarek oraz 100 blink przez )
#define CLK_LOOP	1000			// wybrany zegar w mikrosek

#define CLK_LOOP_SET(a) (a==CLK_NAUKA)?(CLK_S/10):(CLK_S)

// liniowa skala
// #define PWM(a)		(a>PWM_MAX)?(PWM_MAX*CLK_S):((a*a)*CLK_S)
// #define PWM_NAUKA(a)	(a>>3)*CLK_S 					/* przeliczanie dla trybu nauki (timer 100us */

// nieliniowa skala
// wejscie 0-255; wyjscie
#if MHZ21
#define PWM(a)			(a>PWM_MAX)?(((PWM_MAX*PWM_MAX)/64)*CLK_S):(((a*a)>>6)*CLK_S) //
#define PWM_NAUKA(a)	(a>PWM_MAX)?(PWM_MAX*PWM_MAX):((a*a)>>5)	/* przeliczanie dla trybu nauki (timer 100us 21MHz */
#else
#define PWM(a)			(a>PWM_MAX)?(((PWM_MAX*PWM_MAX)/4)):(((a*a)>>2)) //
#define PWM_NAUKA(a)	(a>PWM_MAX)?(PWM_MAX*PWM_MAX):((a*a)>>5)	/* przeliczanie dla trybu nauki (timer 100us 16MHz */
#endif
//#define PWM(a)		((a*CLK_LOOP*CLK_S)/PWM_MAX)
//#define PWM_NAUKA(a) 	((a*CLK_NAUKA*CLK_S)/PWM_MAX)

#define KALIBRACJA_PILOTA 0
#define TEST 0

#if TEST == 0
#define USPIENIE 0
#define DEBUG_BLINK_ON
#define DEBUG_BLINK_OFF
#define DEBUG_KOMENDA 0
#else
#define USPIENIE 0
#define DEBUG_BLINK_ON(a)   P1OUT |= a										// debug blink Port 1.1
#define DEBUG_BLINK_OFF(a)  P1OUT &= ~a										// debug blink
#define DEBUG_KOMENDA 1
// tymczasowe
//static int powtorka;
static int temp1;

#endif


// Tryby pracy
typedef enum
{
	eInit,					// co po wlaczeniu
	eOperation,				// oczekiwanie na IR kod lub przycisk
	eOperationLoop,				// oczekiwanie na IR kod lub przycisk
	eInterrupt,				// przerywanie IR kod
	eOdbior,				// interpretacja komendy
	eNauka,					// nauka / logika
	eSave,					// zapis nowej komendy
	eUspienie,				// pauza
} eOPmodeT;
volatile eOPmodeT eOPmode;

typedef enum
{
	eStart0,
	eStart1,
	eStart2,
	eStart3,
	eStart4,
	eStart5,
	eStartEnd,
	eAdress,
	eKomenda,
	eStop,
	eKoniec,
	eNowa,
	eBlad,
} eIR_rcvStateT;
volatile eIR_rcvStateT eIR_rcvState;


// Zmienne globalne
#define COMMANDBITMAX 	64			// dlugosc_7 - ilosc bitow komendy musi byc mniejsza niz 64

// zmienne modyfikowane w przerywaniu
volatile static char Nauka;			// etapy nauki
//static int LED_Kod;
volatile static int blinkToggle;			// etapy nauki
volatile static int blink;
volatile static int blinkCnt;
volatile static int timer;

static volatile int rcvLicznikC;	// Licznik calkowity mierzacy ciagle od poczatku komendy
static volatile int rcvPortCur;

//----
static int CommandCode[COMMANDBITMAX/16];	// bufor do odbioru komendy
static int CommandBit;				// ktory bit komendy

volatile int rcvLicznik;				// Licznik do mierzenia dlugosci bitu
static int rcvLicznikO;				// ostatni stan licznika calkowitego

static unsigned int PWMwyjscie;				// stan wyjscia 16bit
static unsigned int PWMwyjscieN;				// nowy (docelowy) stan wyjscia po odebraniu komendy

int TablicaKomendNauka[5][4];

//! czasy w komendzie w jednostce 1us
//static const int dlugosc_start0 = 3440/CLK_NAUKA;		// dlugosc_start0 - nowa transmisja -stan 0
//static const int dlugosc_start1 = 1720/CLK_NAUKA;		// dlugosc_start1 - start stan 1
//static const int dlugosc_bit =  430/CLK_NAUKA;		// dlugosc_bit - marker bitu, stan 0
static const int dlugosc_1 = 1720/CLK_NAUKA;		// dlugosc_2 - dlugi bit = 1, stan 1; (430+1290)
static const int dlugosc_0 = 860/CLK_NAUKA;		// dlugosc_3 - krotki bit = 0, stan 1+0; (430+ 430)
//static const int dlugosc_pauza = 1290/CLK_NAUKA;		// dlugosc_4 - pauza, stan 1+0
static const int dlugosc_blad = 3000/CLK_NAUKA;		// dlugosc_6 - blad, przerwanie odbioru
static const int dlugosc_korekcja = 400/CLK_NAUKA;		// dlugosc_korekcja blad rozpoznania komendy

//static volatile int dlugosc_start0;		// dlugosc_start0 - nowa transmisja -stan 0
//static volatile int dlugosc_start1;		// dlugosc_start1 - start stan 1
//static volatile int dlugosc_bit;			// dlugosc_bit - marker bitu, stan 0
//static volatile int dlugosc_1;				// dlugosc_2 - dlugi bit = 1, stan 1
//static volatile int dlugosc_0;				// dlugosc_3 - krotki bit = 0, stan 1
//static const int dlugosc_pauza 	= 1290/CLK_NAUKA;		// dlugosc_4 - pauza, stan 1
//static const int dlugosc_blad 	= 60000/CLK_NAUKA;		// dlugosc_6 - blad, przerwanie odbioru
//static const int dlugosc_korekcja =100/CLK_NAUKA;		// dlugosc_korekcja blad rozpoznania komendy

//static const int dlugosc_koniec = 3000/CLK_NAUKA;		// dlugosc_5 - koniec komendy


//! definicje dla modyfikacji PWM wyjscia; Kolejnosc odpowiada kolejnosci komend z "TablicaKomend"
// Logika: jesli odebrany Kod odpowiada ktoremus z "TablicaKomend" to wyjscie PWM bedzie wysterowane odpowiednio do
// lini w "tablicaPWM"

static const int TablicaPWM[] = {
		PWM_1,
		PWM_2,
		PWM_3,
		PWM_PLUS,
		PWM_MINU,
};

#pragma DATA_SECTION(TablicaKomend, ".infoD")
//! blok w pamieci ktory bedzie nadpisany przy zapamietaniu nowej komendy; adres 0x1000
static const int TablicaKomend[5][4]={
	{ 0x0010, 0x3402, 0x4A7C, 0x0000}, // PWM_1 	<<
	{ 0x0010, 0x3400, 0x4174, 0x0000}, // PWM_2 	>
	{ 0x0010, 0x3400, 0x497C, 0x0000}, // PWM_3 	>>
	{ 0x0010, 0x3400, 0xB084, 0x0000}, // PWM_PLUS 	^
	{ 0x0010, 0x3402, 0xB284, 0x0000}  // PWM_MINUS .
};

#pragma DATA_SECTION(pamiecStala, ".infoC")
//! blok w pamieci ktory bedzie nadpisany przy zapamietaniu nowej komendy; adres 0x1040
// cecha charakterystyczna pamieci .info -> mozna kasowac oddzielnie
static const int pamiecStala[4]= { PWM_3, 0, 0, 0}; // dodatkowa pamiec

static const int *PamiecPWM = &pamiecStala[1];

// --- pamiec
#pragma PERSISTENT(powerstate)
volatile char powerstate;

// ----
void write_FlashSegD (int *Flash_ptr,int *Data, char Length);

//! Initialize the MSP430 clock.
void msp430_init_dco() {

	if(CALBC1_1MHZ!=0xFF){
		//Clear DCL for BCL12
		DCOCTL = 0x00;
		//Info is intact, use it.
		BCSCTL1 = CALBC1_1MHZ;
		DCOCTL = CALDCO_1MHZ;
	}else{
		//Info is missing, guess at a good value.
//		BCSCTL1 = 0x8f;   //CALBC1_16MHZ at 0x10f9
//		DCOCTL = 0x7f;    //CALDCO_16MHZ at 0x10f8

	}
#if MHZ21
	DCOCTL |= 0xE0;							// 21MHz
	BCSCTL1|= 0x0F;
#else
	//DCOCTL |= 0x60;							// ~16,9MHz
	BCSCTL1|= 0x0F;

#endif
	BCSCTL2 &= ~SELS;		  				// SMCLK source


}

int main(void) {

//	//static int TablicaKomendSzukaj[]={0,0,1,2,3,4};	// Tablica do poszukiwania komendy
	int ii,jj;							// zmienne pomocnicze
	char * addess_temp;


#if	DEBUG_KOMENDA
	int temp3,temp2;
#endif
	/*
	 * IR Decoder
	 * Decode and respond to RC5 IR remote control commands, using a TSOP38238SS1V receiver IC
	 * Copyright: Jerry Pommer <jerry@jerrypommer.com> 2012-12-04
	 * License: GNU General Public License v3
*/
	WDTCTL = WDTPW | WDTHOLD;    			// disable watchdog
	Nauka = timer = 0;

	// todo Wartosc PWM po pierwszym wlaczeniu
	if (powerstate == 0)
		PWMwyjscie = *PamiecPWM;
	else
		PWMwyjscie = TablicaPWM[0];

	powerstate=1;
	blink=1000;                				// P1.0 LED =1 zapalic LED 1sek
	//blinkToggle=500;						// blink LED do kalibracji timera
	eOPmode = eInit;
	msp430_init_dco();

	// memset(TablicaKomendNauka,0,sizeof(TablicaKomendNauka));
	addess_temp = (char *)(&TablicaKomendNauka);
	for (ii=0; ii< sizeof(TablicaKomendNauka);ii++)
		*addess_temp++=0;


#if	DEBUG_KOMENDA
	temp1=0;
	temp3 = TAR;
#endif

	for(;;) {
		volatile unsigned int i;			// volatile to prevent optimization

#if DEBUG_KOMENDA
		temp2 = TAR-temp3;
		if(temp2>temp1)
			temp1=temp2;
		temp3 = TAR;
#endif
		switch (eOPmode)
		{
		case eInit:
/*
 *  Uruchomienie / Inicjalizacja
 *  - inicjalizacja portow
 *  - po wlaczeniu ustawienie domyslnego wyjscia
 *  - przejscie w tryb Pracy
 */

			PORT_CLR(P1DIR,IR_DATA|SW_NAUKA);	    // IRDATA|SW_NAUKA is an input
			PORT_CLR(P1OUT,LED|WYJSCIE|BIT1|BIT2);	// Set all these LOW
			P1DIR |= LED | WYJSCIE;					// Set all these as outputs
			P1DIR |= BIT7;							// Debug -> port 1.7 = 1.6
//			PORT_SET(P1SEL,WYJSCIE);                // P1.6 funkcja, wyjscie zegara, (TA0.1 output) aktywna (SET)
			PORT_CLR(P1SEL,WYJSCIE);                // P1.6 funkcja, wyjscie zegara, (TA0.1 output) nieaktywna (CLEAR)
													// Interrupt
			P1IES |= IR_DATA;						// watch for falling edge change on IRDATA
			P1IFG 	= 0;							// clear the P1 interrupt flags
			P1IE = IR_DATA|SW_NAUKA;				// enable interrupts, watching IRDATA and SW_NAUKA for a change

			// Ustawienie Zegara
			TACTL = TACLR;
			//TACTL = TASSEL_1 + MC_1				// ACLK 32768 Hz, up mode
			TACTL = TASSEL_2 + MC_1 + TAIE;			// SMCLK 1MHz, up mode
			CCTL0 = CCIE;                          	// CCR0 interrupt enabled
			CCTL1 = CCIE;                         	// CCR1 interrupt enabled
			CCR0 = CLK_LOOP_SET(CLK_LOOP);			// Ustalenie Taktu; licznik liczy od 0 do PWM_TAKT po tym interrupt
			CCR1 = PWM(PWMwyjscie);					// Ustawienie wyjscia

// manual / automatycznie
/*
 			P1.0,1 oraz P1.4 maja wyjscie sterowane prosto z zegara jesli SEL = 1
			PORT_CLR(P1DIR,WYJSCIE);                // Zerowanie wyjscia
			PORT_SET(P1SEL,WYJSCIE);                // P1.6 funkcja, wyjscie zegara, (TA0.1 output) aktywna (SET)
			CCTL1 = OUTMOD_7;                       // CCR1 reset/set
			CCTL0 &= ~CCIE;                         // CCR0 interrupt enabled
			CCTL1 &= ~CCIE;                         // CCR1 interrupt enabled
*/

			__bis_SR_register(GIE);					// interrupts enabled

			// koniec inicjalizacji
			eOPmode = eUspienie;
			eIR_rcvState = eStart0;

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
			rcvLicznikO = 0;
			rcvLicznik = 0;
			rcvPortCur = 1;

			CommandCode [0]=0;
			CommandCode [1]=0;
			CommandCode [2]=0;
			CommandCode [3]=0;
			CommandBit = 0;

			eOPmode = eOdbior;

			// Ustawienie Zegara
			TACTL = TACLR;
			TACTL = TASSEL_2 + MC_1 + TAIE;		// SMCLK 1MHz, up mode

			CCR0 = CLK_LOOP_SET(CLK_NAUKA);			// Ustalenie Taktu; licznik liczy od 0 do CLK_NAUKA po tym interrupt
			CCR1 = PWM_NAUKA(PWMwyjscie);
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
 */
//			rcvLicznikO -= rcvLicznikC;
			if((rcvLicznikC - rcvLicznikO) != 0)
			{	// nowy stan licznika calkowitego -> znaczy minelo X razy CLK_NAUKA us,
				rcvLicznik += rcvLicznikC - rcvLicznikO;							// inkrementacja licznika czasu odbioru
				// rcvPortCur zerowane po przerwaniu

				switch (eIR_rcvState)
				{
/*
				case eStart0:
					if (rcvPortOst == 0) {
					if (rcvPortCur)
					{// rozpoznanie bitu startu, nowy stan = 1? tak, dalej : nie, czekac na 1
						DEBUG_BLINK_ON(BIT0);
//						if ((rcvLicznik > dlugosc_start0-dlugosc_korekcja) && (rcvLicznik < dlugosc_start0+dlugosc_korekcja))
						{
							dlugosc_start0 = rcvLicznik;
							eIR_rcvState = eStart1; 				// Bit Startu odebrany; stan niski OK
							rcvLicznik=0;							// kasowanie licznika
						}
					}}
					break;
				case eStart1:
					if ((rcvPortOst != 0) && !(rcvPortCur))
					{// rozpoznanie bitu startu, nowy stan = 0? tak, dalej : nie, czekac na 0
//						if ((rcvLicznik > dlugosc_start1-dlugosc_korekcja) && (rcvLicznik < dlugosc_start1+dlugosc_korekcja))
						{
							dlugosc_start1 = rcvLicznik;
							eIR_rcvState = eKomenda; 				// Bit Startu odebrany; stan wysoki OK
							rcvLicznik=0;							// kasowanie licznika
						}
					}
					break;
*/
				case eStartEnd:
					eIR_rcvState = eKomenda; 				// Bit Startu odebrany; stan wysoki OK
					rcvLicznik=1;							// kasowanie licznika
					break;

				case eKomenda:
					if (!(rcvPortCur)) {
						// rozpoznanie bitu, nowy stan = 0? tak, dalej : nie, czekac na 0
						// zapamietanie bitu komendy
//						DEBUG_BLINK_ON(BIT0);

						if ((rcvLicznik >= dlugosc_0-dlugosc_korekcja) && (rcvLicznik < dlugosc_0+dlugosc_korekcja))
						{ // zero
							CommandCode[CommandBit>>4] <<= 1;
							CommandBit++;
						}
						else if ((rcvLicznik >= dlugosc_1-dlugosc_korekcja) && (rcvLicznik < dlugosc_1+dlugosc_korekcja))
						{ // jedynka
							CommandCode[CommandBit>>4] = CommandCode[CommandBit>>4] << 1;
							CommandBit++;
							CommandCode[CommandBit>>4]++;
						}
						else
						{ // pauza
							// powtorka++;
						}

						if (CommandBit == COMMANDBITMAX-1)
						{	// bufor pelny / blad
							eIR_rcvState = eStop;
						}

						// tylko w celu debugowania
#if DEBUG_KOMENDA
						CommandBit--;
						TablicaKomendNauka[CommandBit>>4][0x0003&(CommandBit>>2)] = TablicaKomendNauka[CommandBit>>4][0x0003&(CommandBit>>2)] << 4;
						TablicaKomendNauka[CommandBit>>4][0x0003&(CommandBit>>2)] += rcvLicznik>>3;
						CommandBit++;
#endif

						rcvLicznik=1;						// kasowanie licznika impulsow
						rcvPortCur = 1;						// zapamietanie aktualnego stanu portu
						P1IE |= IR_DATA;					// Turn on P1 interrupts while we work
					}
					else{	// odbierany jest stan niski
						if (rcvLicznik >= dlugosc_blad)
							//eOPmode = eOperation;		// za dlugo / blad
							eIR_rcvState = eStop;		// za dlugo koniec odbioru / blad
//							DEBUG_BLINK_OFF(BIT0);
						}
					break;

				case eStop:
				case eKoniec:
					// nowa komenda; porownanie ze znanymi komendami

					// przesowanie ostatnich bitow w lewo
					CommandCode[CommandBit>>4] <<=  16 * (1+ (CommandBit>>4))- CommandBit;
#if DEBUG_KOMENDA
					TablicaKomendNauka[CommandBit>>4][0x0003&(CommandBit>>2)] <<= 4 * (16 * (1+ (CommandBit>>4))- CommandBit);
#endif
					if (Nauka != 0)
					{ // nauka
						eIR_rcvState = eKoniec;									// tymczasowa zmienna, jesli zostanie zmienione na stop to znaczy nie znana komenda
						for(ii=0; ii < sizeof(CommandCode)/sizeof(CommandCode[0]); ii++)
						{	// zapamietanie Komendy
							if ((Nauka>1) && (CommandCode[ii] != TablicaKomend[Nauka-2][ii]))
								eIR_rcvState = eStop;							// nie pasuje do znanego musteru -> nowa komenda

							TablicaKomendNauka[Nauka-1][ii]=CommandCode[ii];
						}
						if ((Nauka>1) && (eIR_rcvState == eKoniec))				// komenda taka sama co ostatnio, zignorowac
						{
							Nauka--;
						}
						Nauka++;
						eOPmode = eNauka;
					}
					else {
						// tryb normalny
						PWMwyjscieN = PWMwyjscie;									// bez zmian prosze jesli komenda nierozpoznana

						for(ii=0; ii < sizeof(TablicaKomend)/sizeof(TablicaKomend[0]); ii++)
						{
							eIR_rcvState = eKoniec;									// tymczasowa zmienna, jesli zostanie zmienione na stop to znaczy nie znana komenda
							for(jj=0; jj<sizeof(CommandCode)/sizeof(CommandCode[0]); jj++)
							{
								if (CommandCode[jj]!=TablicaKomend[ii][jj])
									eIR_rcvState = eStop;							// nie pasuje do znanego musteru
							}
							if (eIR_rcvState == eKoniec)
							{	// komenda znalezina (ii)
								// nowe wartosci predefiniowane
								if (ii<3)
								{	// wartosci predefiniowane
									PWMwyjscieN = TablicaPWM[ii];
								}
								else
								{	// wartosci modyfikowane '+' (ii=3)/ '-' (ii=4)
									PWMwyjscieN = PWMwyjscie+TablicaPWM[ii];
									// ograniczenie do MIN / MAX
									if (PWMwyjscieN > PWM_MAX) {
										if (ii == 3) PWMwyjscieN = PWM_MAX;
										else
											PWMwyjscieN = PWM_MIN;
									}
								}
								ii = sizeof(TablicaKomend)/sizeof(TablicaKomend[0]); // koniec poszukiwania
							}
						}
						eOPmode = eOperation;				// nowa komenda przetworzona
					}

					blink = 100;							// potwierdzenie na LED
#if DEBUG_KOMENDA
					if(CommandBit<46)
						blink = 1000;						// bledna komenda
#endif
					CCR0 = CLK_LOOP_SET(CLK_LOOP);			// Ustalenie Taktu; licznik liczy od 0 do PWM_TAKT po tym interrupt
					P1IE = IR_DATA|SW_NAUKA;				// enable interrupts, watching IRDATA and SW_NAUKA for a change
					break;

				default:
					break;
				}
				rcvLicznikO = rcvLicznikC;
			}
			break;

		case eOperation:
/*
 *  komenda odebrana jesli ii<TablicaKomend, PWMwyjscieN != PWMwyjscie
 *  :
 *  - Ustawienie PWM
 *  - Ustawienie Przerwania do odbioru komend
 *  - nowa komenda ze skalowaniem; logika, przyklad
 *  	  stan akt,     krok,   cel, -> nowy
 *  			70		2		>=72	72
 *  			70		2		71		70
 *  			70		2		69		70
 *  			70		2		<=68	68
 *
 *  - Fade IN/OUT
 *	- skalowanie wyjscia
 */

			// \todo fade in/ out
			PWMwyjscie = PWMwyjscieN;

			// min-max: brak przerywan ponizej progu minimalnego (dodatkowy tryb) oraz powyzej maksymalnego
			if (PWM_MIN >= PWMwyjscie)
				CCTL1 &= ~CCIE;             // CCR1 interrupt disabled
			else
			{
				CCTL1 |= CCIE;              // CCR1 interrupt enabled
				if (PWM_MAX <= PWMwyjscie)
					CCR1 = -1;
				else
					CCR1 = PWM(PWMwyjscie);			// Ustawienie wyjscia
			}

			eOPmode = eUspienie;
			eIR_rcvState = eStart0;
			timer = 3000;					// opoznienie czasowe do zapisania ostatniego stanu
			break;

		case eUspienie:
#if USPIENIE == 1
			if (blink == 0)
				__bis_SR_register(CPUOFF + GIE);		// Low Power Mode 0 with interrupts enabled

#endif
			if (timer == 0)
			{ 	// opoznienie czasowe; zapisanie tylko jesli nie zero lub inne niz poprzednie
				if ((PWMwyjscieN != 0) &&
						(PWMwyjscie != 0) && (*PamiecPWM != PWMwyjscie))
				{ // zapamietanie ostatniej wartosci do pamieci stalej "pamiecStala"
					PORT(WYJSCIE,0);					// podczas zapisu nastepuje wylaczenie przerywan;
														// zeby zapobiec blyskowi, wylaczam PWM -> przerzucanie stanu portu 1 -> 0
					write_FlashSegD((int *)PamiecPWM, (int *)&PWMwyjscie, sizeof(PWMwyjscie)/sizeof(int));
					blink = 100;
					PWMwyjscieN = 0;
				}
			}
			break;

		case eNauka:
/*
 *  Nauka komend pilota:
 *  - kolejno progi +,-, PWM_1/2/3
 *  	nowa komenda? -> zapamietanie
 *  	blink
 *
 */
			eOPmode = eOperation;					// nowa komenda przetworzona

			if (Nauka == 1+(sizeof(TablicaPWM)/sizeof(TablicaPWM[0])))
			{	// nowe kody odebrane
				write_FlashSegD((int *)TablicaKomend, (int *)TablicaKomendNauka, sizeof(TablicaKomendNauka)/sizeof(int));

				// nowa tablica zapamietana
				// memset(TablicaKomendNauka,0,sizeof(TablicaKomendNauka));
				addess_temp = (char *)(&TablicaKomendNauka);
				for (ii=0; ii< sizeof(TablicaKomendNauka);ii++)
					*addess_temp++=0;

				// skasowanie licznika nauki
				blink = 1000;
				Nauka = 0;
				eOPmode = eOperation;				// nowa komenda przetworzona
			}
			else
			{
//				blink = 100;

				Nauka--;
				// nowe wartosci predefiniowane
				if ((Nauka)<3)
				{	// wartosci predefiniowane
					PWMwyjscieN = TablicaPWM[Nauka];
				}
				else
				{	// wartosci modyfikowane + / -; mnoznik 2* zeby lepiej uwidocznic efekt
					PWMwyjscieN = TablicaPWM[1]+(2*TablicaPWM[Nauka]);
				}
				Nauka++;
			}
			break;

		default:
			break;

		}

		if (blinkCnt != 0) {
			if (blinkCnt > blink+1)
			{
				PORT(LED,1);                // P1.6 LED =1 zapalic
			}
			else
			{
				PORT(LED,0);                // P1.6 LED =0 zgasic
			}
		} else {
			if (blinkToggle) {
				blinkCnt = blinkToggle<<1;
				blink=blinkToggle;
			} else if (blink)
			{
				blinkCnt = blink;
				blink = 0;
			}
		}
	}


	return 0;
}


void write_FlashSegD (int *Flash_ptr, int *Data, char Length)
{
//  char *Flash_ptr;       					  // Flash pointer
	unsigned int i;
	unsigned char temp1,temp2;

    temp1 = BCSCTL1;
    temp2 = DCOCTL;

    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
	BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
	DCOCTL = CALDCO_1MHZ;

	FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator
	FCTL1 = FWKEY + ERASE;                    // Set Erase bit
	FCTL3 = FWKEY;                            // Clear Lock bit

	*Flash_ptr = 0;                           // Dummy write to erase Flash segment

	FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

	for (i=0; i<Length; i++)
	{
		*Flash_ptr++ = *Data++;				  // Write value to flash
	}

	FCTL1 = FWKEY;                            // Clear WRT bit
	FCTL3 = FWKEY + LOCK;                     // Set LOCK bit

    BCSCTL1 = temp1;
    DCOCTL = temp2;

}

/*
 *	przerwanie uzywane do sterowania PWM;
 *	przerwanie wywolane jesli stan licznika TAR jest rowny Rejestrowi CCR0=TAR (koniec cyklu)
 *	licznik bignie dalej od 0, port przelaczyc na stan wysoki
 *
 */

// Timer A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0 (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMERA0_VECTOR))) Timer_A0 (void)
#else
#error Compiler not supported!
#endif
{
											// eOPmode = eOperation;
	rcvLicznikC++;							// inkrementacja glownego licznika
	if (blinkCnt)
		blinkCnt--;
	if (timer)
		timer--;
	if( CCTL1 & CCIE)             			// CCR1 interrupt enabled
	{
		PORT(WYJSCIE,1);						// przerzucanie stanu portu 0 -> 1
		PORT(BIT7,1);						// Debug -> przerzucanie stanu portu 0 -> 1
	}

}

/*
 *	przerwanie uzywane do sterowania PWM;
 *	przerwanie wywolane jesli stan licznika TAR jest rowny Rejestrowi CCR1=TAR (regulacja wypelnienia PWM)
 *	licznik bignie dalej
 *
 */

// Timer_A2 Interrupt Vector (TAIV) handler
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
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
		PORT(BIT7,0);						// przerzucanie stanu portu 1 -> 0
		break;
	case 10:
		break;
	}
}

/*!
 *  przerwanie z portu:
 *  1) PORT IR (zbocze opadajace)
 * *
 *	2) przycisk
 */

#pragma vector = PORT1_VECTOR;
void __interrupt Port_1(void)
{

#if KALIBRACJA_PILOTA
	if (IR_DATA & P1IFG)
	{	// IR port -> przerwanie
		P1IFG &= ~IR_DATA;							// clear the P1 interrupt flag for IRDATA
		if (eOPmode == eUspienie)
			eOPmode = eInterrupt;

		switch (eIR_rcvState)
		{
		case eStart0:
			eIR_rcvState = eStart1; 				// Bit Startu odebrany; stan niski OK
  			P1IES &= ~IR_DATA;						// now watch for Rising edge change on IRDATA
  			P1IE |= IR_DATA;						// Turn on P1 interrupts while we work
  			break;
		case eStart1:
  			dlugosc_start0 = rcvLicznikC;
			eIR_rcvState = eStart2; 				// Bit Startu odebrany; stan niski OK
  			P1IES |= IR_DATA;						// now watch for falling edge change on IRDATA
  			P1IE |= IR_DATA;						// Turn on P1 interrupts while we work
			break;
		case eStart2:
  			dlugosc_start1 = rcvLicznikC;
			eIR_rcvState = eStart3; 				// Bit Startu odebrany; stan wysoki OK
  			P1IES &= ~IR_DATA;						// now watch for Rising edge change on IRDATA
  			P1IE |= IR_DATA;						// Turn on P1 interrupts while we work
			break;
		case eStart3:
  			dlugosc_bit = rcvLicznikC;
			eIR_rcvState = eStart4; 				// Znacznik bitu odebrany (niski)
  			P1IES |= IR_DATA;						// now watch for falling edge change on IRDATA
  			P1IE |= IR_DATA;						// Turn on P1 interrupts while we work
  			break;
		case eStart4:
  			dlugosc_0 = rcvLicznikC; 				// Bit 0 odebrany (wysoki)
			eIR_rcvState = eStart5; 				//
  			P1IE |= IR_DATA;						// Turn on P1 interrupts while we work
			break;
		case eStart5:
  			dlugosc_1 = rcvLicznikC-dlugosc_0; 		// Bit 1 odebrany (wysoki)
  			dlugosc_0 = dlugosc_0-dlugosc_start1; 		// Bit 0 odebrany (wysoki)
  			dlugosc_bit = dlugosc_bit-dlugosc_start1;
  			dlugosc_start1 = dlugosc_start1-dlugosc_start0;
			eIR_rcvState = eStartEnd; 				//
  			P1IE &= ~IR_DATA;						// Turn off P1 interrupts while we work
			break;
#else
			if (IR_DATA & P1IFG)
			{	// IR port -> przerwanie
				P1IFG &= ~IR_DATA;							// clear the P1 interrupt flag for IRDATA
				if (eOPmode == eUspienie)
					eOPmode = eInterrupt;

				switch (eIR_rcvState)
				{
				case eStart0:
					eIR_rcvState = eStart1; 				// Bit Startu odebrany; stan niski OK
//					break;									// no break
				case eStart1:
					eIR_rcvState = eStart2; 				// Bit Startu odebrany; stan niski OK
					break;
				case eStart2:
					eIR_rcvState = eStart3; 				// Bit Startu odebrany; stan wysoki OK
					break;
				case eStart3:
					eIR_rcvState = eStart4; 				// Znacznik bitu odebrany (niski)
//					break;									// no break
				case eStart4:
					eIR_rcvState = eStart5; 				//
					break;
				case eStart5:
					eIR_rcvState = eStartEnd; 				//
					break;
#endif
/*
					if (IR_DATA & P1IFG)
					{	// IR port -> przerwanie
						P1IFG &= ~IR_DATA;							// clear the P1 interrupt flag for IRDATA
						if (eOPmode == eUspienie)
							eOPmode = eInterrupt;

						switch (eIR_rcvState)
						{
						case eStart0:
							eIR_rcvState = eStart1; 				// Bit Startu odebrany; stan niski OK
				//			P1IES &= ~IR_DATA;						// now watch for Rising edge change on IRDATA
				//			P1IE |= IR_DATA;						// Turn on P1 interrupts while we work
				//			break;
						case eStart1:
				//			dlugosc_start0 = rcvLicznikC;
							eIR_rcvState = eStart2; 				// Bit Startu odebrany; stan niski OK
				//			P1IES |= IR_DATA;						// now watch for falling edge change on IRDATA
				//			P1IE |= IR_DATA;						// Turn on P1 interrupts while we work
							break;
						case eStart2:
				//			dlugosc_start1 = rcvLicznikC;
							eIR_rcvState = eStart3; 				// Bit Startu odebrany; stan wysoki OK
				//			P1IES &= ~IR_DATA;						// now watch for Rising edge change on IRDATA
				//			P1IE |= IR_DATA;						// Turn on P1 interrupts while we work
							break;
						case eStart3:
				//			dlugosc_bit = rcvLicznikC;
							eIR_rcvState = eStart4; 				// Znacznik bitu odebrany (niski)
				//			P1IES |= IR_DATA;						// now watch for falling edge change on IRDATA
				//			P1IE |= IR_DATA;						// Turn on P1 interrupts while we work
				//			break;
						case eStart4:
				//			dlugosc_0 = rcvLicznikC; 				// Bit 0 odebrany (wysoki)
							eIR_rcvState = eStart5; 				//
				//			P1IE |= IR_DATA;						// Turn on P1 interrupts while we work
							break;
						case eStart5:
				//			dlugosc_1 = rcvLicznikC-dlugosc_0; 		// Bit 1 odebrany (wysoki)
				//			dlugosc_0 = dlugosc_0-dlugosc_start1; 		// Bit 0 odebrany (wysoki)
				//			dlugosc_bit = dlugosc_bit-dlugosc_start1;
				//			dlugosc_start1 = dlugosc_start1-dlugosc_start0;
							eIR_rcvState = eStartEnd; 				//
				//			P1IE &= ~IR_DATA;						// Turn off P1 interrupts while we work
							break;
*/
		default:
			//										debug, command processiny by interrupt
			P1IE &= ~IR_DATA;						// Turn off P1 interrupts while we work
			rcvPortCur = 0;
			break;
		}
	}
	else if (SW_NAUKA & P1IFG)
	{	// przycisk -> Nauka
		P1IFG &= ~SW_NAUKA;							// clear the P1 interrupt flag for SW_NAUKA
		if (eOPmode == eUspienie)
			eOPmode = eNauka;
		Nauka = 1;
	}

	//TODO: wait for second start bit within T_INTERVAL; abandon if it doesn't come.


}

/**
\dot
digraph G {"
eInit	->	eOperation	;
eOperation	->	eUspienie	;
eUspienie	->	eUspienie	;
eOdbior	->	eOperation	;
eNauka	->	eOdbior	;
eInterrupt Timer0	->	PWM_ON	;
eInterrupt Timer1	->	PWM_OFF	;
eInterrupt Port IR	->	eOdbior	;
eInterrupt BUTTON	->	eNauka	;
"}
\enddot
*/


/**
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


http://www.egr.msu.edu/classes/ece480/capstone/fall10/group04/Dong,%20Roy%20-%20Application%20Note.pdf


For example, from pg. 6 of the MSP430G2231
datasheet, pin 4 is “P1.2/TA0.1/A2”. We want to use this pin as “TA0.1”. To do this, we have to set second
bit of both the P1SEL and P1DIR registers accordingly. Generally, to set the Px.y pin, we must set the
PxSEL and PxDIR registers accordingly at bit y. A 0 in PxDIR is input; a 1 is output. A 0 in PxSEL means
general purpose input/output, while a 1 in PxSEL reflects a special purpose based on the pin. For
example, to use the timer, we would set the appropriate bits in PxSEL and PxDIR to 1. To use the ADC
converter (A2), we would set PxSEL to 1 and PxDIR to 0. The code to set the pin as a timer is as follows:
P1DIR |= BIT2;
P1SEL |= BIT2;


 *
 */
