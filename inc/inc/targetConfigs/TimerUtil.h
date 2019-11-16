#ifndef TIMER_UTIL_HEADER
#define TIMER_UTIL_HEADER

/*  TAxCCRy is the constant value
 *
 *  TAxR    is the counter value
 *
 *  SMCLK = BUSCLK / 4 = 48MHz / 4 = 12 MHz, 83.33 ns
 *      Is this always? At first he makes it sound like the SMCLK is always
 *      going to be this value, but then he says in this module???
 *
 *      It does seem like SMCLK is constantly 12MHz
 *
 *      System Modle Register
 *  RSVD    TASSEL  ID      MC      RSVD    TACLR   TAIE    TAIFG
 *  15-10   9-8     7-6     5-4     3       2       1       0
 *      TAIFG
 *      TAIE
 *      TACLR
 *      MC      Mode Control
 *      ID      Prescale Value
 *      TASSEL  Clock Select
 *
 *      Submodule Registers TAxCCTLy
 *  CM      CCIS    SCS SCCI    RSVD    CAP OUTMOD  CCIE    CCI OUT COV CCIFG
 *  15-14   13-12   11  10      9       8   7-5     4       3   2   1   0
 *
 *      CCIFG   Trigger Bit
 *      COV
 *      OUT
 *      CCI
 *      CCIE    Arm bit for the interrupt?
 *      OUTMOD  
 *      CAP
 *      SCCI
 *      SCS
 *      CCIS
 *      CM
 *
 *
 */
enum TASSEL_VALUES {
    TAxCLK,
    ACLK,
    SMCLK,
    INCLK
};

enum PRESCALE {
    PRESCALE_ONE,
    PRESCALE_TWO,
    PRESCALE_FOUR,
    PRESCALE_EIGHT 
};

enum MODE_CONTROL {
    MODE_STOP,
    MODE_UP,            //TImer counts up to TAxCCR0
    MODE_CONTINUOUS,    //Timer counts up to 0xFFFF
    MODE_UP_DOWN        //Timer counts up to TAxCCR0 then down to 0x0000
};

enum TA_CLR_BIT {
    TA_NO_CLR,
    TA_CLR
};

enum TA_I_E {
    TA_INT_NO_EN,
    TA_INT_EN
};

#define TIMER_CTL_MASK(TASSEL, ID, MC, TACLR, TAIE, TAIFG)              \
    (((TASSEL & 0x3) << 8) | ((ID & 0x3) << 6) | ((MC & 0x3) << 4) |    \
     ((TACLR & 0x1) << 2) | ((TAIE & 0x1) << 1) | (TAIFG & 0x1))


#define TIMER_CCTL_TOGGLE 0x80
#define TIMER_CCTL_TOGGLE_RESET 0x40

#define TIMER_CCTL_MASK(CM, CCIS, SCS, SCCI, CAP, OUTMOD, CCIE, CCI, OUT, COV, CCIFG)\
    (((CM & 0x3) << 14) | ((CCIS & 0x3) << 12) | ((SCS & 1) << 11) |        \
     ((SCCI & 1) << 10) | ((CAP & 1) << 8) | ((OUTMOD & 0x7) << 5) |        \
     ((CCIE & 0x1) << 4) | ((CCI & 1) << 3) | ((OUT & 0x1) << 2) |          \
     ((COV & 1) << 1) | (CCIFG & 1))

#endif

