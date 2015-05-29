#include<xc.h> // processor SFR definitions
#include<sys/attribs.h> // __ISR macro
#define VOLTS_PER_COUNT (3.3/1024)
#define CYCLES 10000000
// DEVCFGs here
// DEVCFG0
#pragma config DEBUG = OFF// no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // not boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 40MHz
#pragma config FPLLIDIV = DIV_4 // divide input clock to be in range 4-5MHz  DIVID BY 4
#pragma config FPLLMUL = MUL_20 // multiply clock after FPLLIDIV   times 20
#pragma config UPLLIDIV = DIV_1 // divide clock after FPLLMUL     divid by 1
#pragma config UPLLEN = ON // USB clock on
#pragma config FPLLODIV = DIV_1 // divide clock by 1 to output on pin

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid
#pragma config PMDL1WAY = 1 // not multiple reconfiguration, check this
#pragma config IOL1WAY = 1 // not multimple reconfiguration, check this
#pragma config FUSBIDIO = 1 // USB pins controlled by USB module
#pragma config FVBUSONIO = 1 // controlled by USB module


int readADC(void);

int main() {

    // startup
	__builtin_disable_interrupts();

	// set the CP0 CONFIG register to indicate that
	// kseg0 is cacheable (0x3) or uncacheable (0x2)
	// see Chapter 2 "CPU for Devices with M4K Core"
	// of the PIC32 reference manual
	__builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

	// no cache on this chip!

	// 0 data RAM access wait states
	BMXCONbits.BMXWSDRM = 0x0;

	// enable multi vector interrupts
	INTCONbits.MVEC = 0x1;

	// disable JTAG to be able to use TDI, TDO, TCK, TMS as digital
	DDPCONbits.JTAGEN = 0;

	__builtin_enable_interrupts();
    // set up USER pin as input
        ANSELBbits.ANSB13=0;
	TRISBbits.TRISB13=1;
    // set up LED1 pin as a digital output
	TRISBbits.TRISB7=0;
       	LATBbits.LATB7=1;

    // set up LED2 as OC1 using Timer2 at 1kHz
        ANSELBbits.ANSB15=0;
	RPB15Rbits.RPB15R=0b0101;
	T2CONbits.TCKPS = 0;
	PR2=39999;
	TMR2=0;
	T2CONbits.ON=1;
        T2CONbits.TCS=0;
	OC1CONbits.OCTSEL=0;
	OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
	OC1CONbits.ON = 1;
	OC1RS=20000;
	OC1R=20000;
    // set up A0 as AN0
    ANSELAbits.ANSA0 = 1;
    AD1CON3bits.ADCS = 3;
    AD1CHSbits.CH0SA = 0;
    AD1CON1bits.ADON = 1;

    

    int voltage_count;
    float volts;
    while (1) {
        voltage_count=readADC();
        volts=(float)voltage_count/(float)1024;
        OC1RS=volts*(PR2+1);
        if (PORTBbits.RB13){
            _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT() < CYCLES){
                ;
            }
            LATBINV=1<<7;
        }
        else{
            LATBINV=1<<7;
        }

        // invert pin every 0.5s, set PWM duty cycle % to the pot voltage output %
    }
}

int readADC(void) {
    int elapsed = 0;
    int finishtime = 0;
    int sampletime = 20;
    int a = 0;

    AD1CON1bits.SAMP = 1;
    elapsed = _CP0_GET_COUNT();
    finishtime = elapsed + sampletime;
    while (_CP0_GET_COUNT() < finishtime) {
    }
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE) {
    }
    a = ADC1BUF0;
    return a;
}