/* dspic30f2010 Base Pure Sine Wave,Owner, Myanmar Smart Power Co.,Ltd, ( M.S.P ), Zaw Win Hlaing */
#define ROLE_MASTER   1
// #define ROLE_SLAVE_B  1    // -120° (i.e., +240°)
// #define ROLE_SLAVE_C  1    // +120°

#include <p30f2010.h>
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

/* ====== CLOCK & DELAY DEFINES ======= */
/* 6.144 MHz crystal, XT + PLL×16 => Fosc = 98.304 MHz, Fcy = 24.576 MHz */
#define FOSC_HZ  (98304000UL)
#define FCY      (FOSC_HZ/4UL)
#define FCY_HZ   FCY
#include <libpic30.h>

/* === CONFIGURATION BITS (XC16 v2.10 friendly) ======= */
// FOSC
#pragma config FPR = XT_PLL16           // XT w/PLL 16x
#pragma config FOS = PRI                // Primary Oscillator
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock switch OFF, Fail-safe OFF
// FWDT
#pragma config FWPSB = WDTPSB_16
#pragma config FWPSA = WDTPSA_512
#pragma config WDT   = WDT_OFF
// FBORPOR
#pragma config FPWRT = PWRT_64
#pragma config BODENV= BORV_27
#pragma config BOREN = PBOR_ON
#pragma config LPOL  = PWMxL_ACT_HI
#pragma config HPOL  = PWMxH_ACT_HI
#pragma config PWMPIN= RST_IOPIN
#pragma config MCLRE = MCLR_EN
// FGS
#pragma config GWRP  = GWRP_OFF
#pragma config GCP   = CODE_PROT_OFF
// FICD
#pragma config ICS   = PGD

/* ====== USER SETTINGS ====== */
#define FPWM_HZ        (20000UL)         /* ~20 kHz center-aligned */
#define MAINS_HZ       (50UL)            /* 50 Hz fundamental */
#define DEADTIME_NS    (1000UL)          /* 1.0 us dead-time */
#define SYNC_PULSE_US  (100UL)           /* 100 us */
#define MODE_THRESH_V  (3.5f)            /* AN5 > 3.5V => parallel/same-phase */

/* ===== PINS ====== */
#if defined(ROLE_MASTER)                 /* SYNC OUT on RB0 (pin 2) */
  #define TRIS_SYNC     TRISBbits.TRISB0
  #define LAT_SYNC      LATBbits.LATB0
#else                                    /* SYNC IN on INT0 = RE8 (pin 16) */
  #define TRIS_SYNCIN   TRISEbits.TRISE8
  #define PORT_SYNCIN   PORTEbits.RE8    /* not read; INT0 edge used */
#endif

/* Mode select: RB5 / AN5 (pin 7) */
#define MODE_ANCH       5
#define MODE_TRIS       TRISBbits.TRISB5

/* ======================== NCO/PHASE ======================== */
static volatile uint16_t phase_acc  = 0;   /* 0..65535 => 0..360° */
static volatile uint16_t phase_step = 0;   /* add each PWM ISR for 50 Hz */

#define DEG_TO_ACC(d)   ((uint16_t)((uint32_t)(d)*65536UL/360UL))
#define OFFS_MASTER     DEG_TO_ACC(0)
#define OFFS_SLAVE_B    DEG_TO_ACC(240)    /* -120° == +240° */
#define OFFS_SLAVE_C    DEG_TO_ACC(120)

/* ======================== SINE LUT (RAM) ======================== */
/* 128 entries (256 bytes) fits dsPIC30F2010 .bss easily */
#define LUT_BITS   7
#define LUT_SIZE   (1u<<LUT_BITS)
static uint16_t sinLUT[LUT_SIZE];

/* ======================== PWM PERIOD & DEAD-TIME ======================== */
/* Center-aligned: Fpwm = Fcy / (2*(PTPER+1))  =>  PTPER = Fcy/(2*Fpwm) - 1 */
#define PTPER_VAL       ((uint16_t)((FCY_HZ/(2UL*FPWM_HZ)) - 1UL))   /* ~614 -> ~19.98 kHz */
#define DT_COUNTS_RAW   ((uint32_t)((FCY_HZ/1000000000UL)*DEADTIME_NS)) /* ~24.6 for 1000ns */
#define DT_COUNTS       ((uint16_t)((DT_COUNTS_RAW>31)?31:DT_COUNTS_RAW)) /* 5-bit cap */

/* ======================== GLOBALS ======================== */
static volatile uint16_t g_an5 = 0;
static volatile uint16_t phase_offset = OFFS_MASTER;
/* Soft-start */
static volatile uint16_t mod_q15 = 0;            /* 0..0x7FFF */
#define MOD_TARGET_Q15   (0x6AAB)                /* ~0.833 */
#define SOFTSTART_MS     (500U)
#define MOD_STEP_PER_ISR ((uint16_t)(( (uint32_t)MOD_TARGET_Q15 ) / ((uint32_t)SOFTSTART_MS * (uint32_t)FPWM_HZ / 1000UL) + 1U))

/* ======================== HELPERS ======================== */
static inline uint16_t q15_from_float(float x){
    if (x <= 0.0f) return 0;
    if (x >= 0.999969f) return 0x7FFF;
    return (uint16_t)(x*32767.0f + 0.5f);
}

/* ======================== ADC (AN5) ======================== */
static void ADC_Init_AN5(void){
    ADPCFG = 0xFFFF;                 /* all digital */
    ADPCFGbits.PCFG5 = 0;            /* AN5 analog */
    MODE_TRIS = 1;                   /* RB5 input */
    ADCON1bits.ADSIDL = 0;
    ADCON1bits.FORM   = 0;
    ADCON1bits.SSRC   = 7;           /* auto-convert */
    ADCON1bits.SIMSAM = 0;
    ADCON1bits.ASAM   = 0;           /* manual sample */
    ADCON2 = 0;
    ADCON3bits.SAMC = 16;            /* sample time */
    ADCON3bits.ADCS = 10;            /* Tad = 11*Tcy */
    ADCHS = 0;
    ADCON1bits.ADON = 1;
}

static uint16_t ADC_Read_AN5(void){
    ADCHSbits.CH0SA = 5;             /* AN5 */
    ADCON1bits.SAMP = 1;
    __delay32( (FCY_HZ/1000000UL)*5 ); /* ~5 us */
    ADCON1bits.SAMP = 0;
    while(!ADCON1bits.DONE);
    ADCON1bits.DONE = 0;
    return ADCBUF0;
}

/* ======================== SYNC ======================== */
#if defined(ROLE_MASTER)
static volatile uint16_t sync_pulse_ticks = 0;
#endif

void __attribute__((__interrupt__, auto_psv)) _INT0Interrupt(void){
#ifndef ROLE_MASTER
    IFS0bits.INT0IF = 0;             /* clear */
    phase_acc = 0;                   /* hard-align to ?=0 on slaves */
#endif
}

/* ======================== PWM CORE ======================== */
/* Two-leg duties: A = +sin, B = -sin (bipolar full-bridge SPWM) */
static inline void PWM_SetDuties(uint16_t dutyA, uint16_t dutyB){
    if(dutyA == 0) dutyA = 1; if(dutyA >= PTPER_VAL) dutyA = PTPER_VAL-1;
    if(dutyB == 0) dutyB = 1; if(dutyB >= PTPER_VAL) dutyB = PTPER_VAL-1;
    PDC1 = dutyA << 1;   /* Leg A (PWM1H/L) */
    PDC2 = dutyB << 1;   /* Leg B (PWM2H/L) */
}

static void PWM_Init(void){
    PTCONbits.PTEN = 0;              /* disable during config */

    PTCONbits.PTCKPS = 0;            /* 1:1 prescale */
    PTCONbits.PTOPS  = 0;            /* 1:1 postscale */
    PTPER = PTPER_VAL;               /* ? 614 -> ~19.98 kHz */

    PWMCON1bits.PMOD1 = 0;           /* complementary */
    PWMCON1bits.PMOD2 = 0;
    PWMCON1bits.PMOD3 = 0;

    PWMCON1bits.PEN1H = 1;
    PWMCON1bits.PEN1L = 1;
    PWMCON1bits.PEN2H = 1;
    PWMCON1bits.PEN2L = 1;
    PWMCON1bits.PEN3H = 0;
    PWMCON1bits.PEN3L = 0;

    DTCON1bits.DTAPS = 0;            /* 1:1 */
    DTCON1bits.DTA   = DT_COUNTS;    /* ~25 counts -> ~1.0 us */

    PTCONbits.PTMOD = 2;             /* center-aligned mode */
    SEVTCMP = PTPER;                  /* reserved for ADC sync if needed */

    PDC1 = 0;
    PDC2 = 0;

    PTCONbits.PTEN = 1;              /* enable PWM */
}

/* ======================== BUILD SINE LUT (once) ======================== */
static void BuildSinLUT(void){
    for (uint16_t i=0; i<LUT_SIZE; i++){
        float x = ((float)i * 6.28318530718f) / (float)LUT_SIZE;
        float y = 0.5f + 0.5f * __builtin_sin(x);   /* 0..1 */
        sinLUT[i] = q15_from_float(y);              /* 0..32767 */
    }
}

/* ======================== ANGLE INIT ======================== */
static void Angle_Init(void){
    /* phase_step = 65536 * (50 / Fpwm) */
    uint32_t step = (uint32_t)65536UL * (uint32_t)MAINS_HZ / (uint32_t)FPWM_HZ; /* 163.84 -> 164 */
    phase_step = (uint16_t)step;
    phase_acc  = 0;
}

/* ======================== SYNC GPIO INIT ======================== */
static void SYNC_Init(void){
#if defined(ROLE_MASTER)
    TRIS_SYNC = 0;  LAT_SYNC = 0;                 /* RB0 as output low */
#else
    TRIS_SYNCIN = 1;
    INTCON2bits.INT0EP = 0;                       /* rising edge on RE8 */
    IFS0bits.INT0IF = 0;
    IEC0bits.INT0IE = 1;
#endif
}

/* ======================== PWM ISR ======================== */
void __attribute__((__interrupt__, auto_psv)) _PWMInterrupt(void){
    IFS2bits.PWMIF = 0;

    uint16_t old = phase_acc;
    phase_acc += phase_step;

#if defined(ROLE_MASTER)
    if (phase_acc < old){             /* wrap -> ?=0 */
        LAT_SYNC = 1;
        sync_pulse_ticks = (uint16_t)(((uint32_t)SYNC_PULSE_US * FPWM_HZ)/1000000UL);
        if (sync_pulse_ticks == 0) sync_pulse_ticks = 1;
    }
    if (sync_pulse_ticks){
        if (--sync_pulse_ticks == 0) LAT_SYNC = 0;
    }
#endif

    /* Soft-start ramp */
    if (mod_q15 < MOD_TARGET_Q15){
        uint16_t next = mod_q15 + MOD_STEP_PER_ISR;
        mod_q15 = (next > MOD_TARGET_Q15) ? MOD_TARGET_Q15 : next;
    }

    /* Effective phase with offset */
    uint16_t theta = phase_acc + phase_offset;
    uint16_t idx = theta >> (16 - LUT_BITS);
    uint16_t s = sinLUT[idx];                 /* 0..32767 (Q15 0..1) */
    int32_t sb = (int32_t)s - 16384;          /* -16384..+16383 */

    /* +sin for leg A, -sin for leg B */
    int32_t vA = ((int32_t)mod_q15 *  sb) >> 15;
    int32_t vB = ((int32_t)mod_q15 * -sb) >> 15;

    int32_t dutyA = ((vA + 16384) * (int32_t)PTPER_VAL) >> 15;
    int32_t dutyB = ((vB + 16384) * (int32_t)PTPER_VAL) >> 15;

    if (dutyA < 0) dutyA = 0; if (dutyA > PTPER_VAL) dutyA = PTPER_VAL;
    if (dutyB < 0) dutyB = 0; if (dutyB > PTPER_VAL) dutyB = PTPER_VAL;

    PWM_SetDuties((uint16_t)dutyA, (uint16_t)dutyB);
}

/* ======================== MAIN ======================== */
int main(void){
    BuildSinLUT();
    Angle_Init();
    ADC_Init_AN5();
    SYNC_Init();
    PWM_Init();

    /* Enable PWM interrupt */
    IFS2bits.PWMIF = 0;
    IEC2bits.PWMIE = 1;

    /* Initial role offset */
#if defined(ROLE_MASTER)
    phase_offset = OFFS_MASTER;
#elif defined(ROLE_SLAVE_B)
    phase_offset = OFFS_SLAVE_B;
#elif defined(ROLE_SLAVE_C)
    phase_offset = OFFS_SLAVE_C;
#else
  #error "Select exactly one ROLE_*"
#endif

#if defined(ROLE_MASTER)
    LAT_SYNC = 0;
#endif

    /* Soft-start begins from mod_q15=0 in ISR */

    while (1){
        /* Mode: AN5 <= 3.5 V => true 3-phase; > 3.5 V => parallel/same-phase */
        g_an5 = ADC_Read_AN5();
        bool parallel = (g_an5 >= (uint16_t)((MODE_THRESH_V/5.0f)*1023.0f + 0.5f));

        phase_offset =
            parallel ? OFFS_MASTER :
        #if defined(ROLE_MASTER)
            OFFS_MASTER
        #elif defined(ROLE_SLAVE_B)
            OFFS_SLAVE_B
        #elif defined(ROLE_SLAVE_C)
            OFFS_SLAVE_C
        #endif
        ;

        __delay32( FCY_HZ/200 );        /* ~5 ms */
    }
    return 0;
}
