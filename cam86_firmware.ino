/*  Copyright © 2017 Gilmanov Rim, Vakulenko Sergiy and Luka Pravica
   
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * CAM86 DIY Astronomy Camera Firmware v3.2
 * Arduino IDE — ATmega328P @ 8MHz internal RC oscillator
 * Fuses: Low=0xE2, High=0xD9, Extended=0xFF
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

// ─── Version ──────────────────────────────────────────────────────────────────
#define VERSION 32

// ─── PORTD idle state ─────────────────────────────────────────────────────────
#define PORTD_IDLE 0xCF

// ─── PORTC values ─────────────────────────────────────────────────────────────
#define K_ACT_0  0x14
#define K_ACT_1  0x2a
#define K_BIN_S  0x14
#define K_BIN_A  0x1a
#define K_BIN_B  0x2a
#define K_DUM_0  0x14
#define K_DUM_1  0x28 //0x20
#define K_TRAIL  0x28
#define K_CLR_A  0x14
#define K_CLR_B  0x20

// ─── PORTB bit masks ──────────────────────────────────────────────────────────
#define PB_MOSI (1<<PB3)
#define PB_MISO (1<<PB4)
#define PB_SCK  (1<<PB5)
#define PB_TEC  (1<<PB0) //green LED
#define PB_DHT  (1<<PB1) //red LED
#define PB_DSB  (1<<PB2) //yellow LED

// ─── Commands ─────────────────────────────────────────────────────────────────
#define CMD_READFRAME          0x1b
#define CMD_CLEAR_SUBSTRATE    0x2b
#define CMD_OFF15V             0x3b
#define CMD_SET_ROISTARTY      0x4b
#define CMD_SET_ROINUMY        0x5b
#define CMD_SET_EXPOSURE       0x6b
#define CMD_SET_BINNING        0x8b
#define CMD_CLEARFRAME         0xcb
#define CMD_INITMCU            0xdb
#define CMD_SET_DELAYPERLINE   0xeb
#define CMD_GET_VERSION        0xbb
#define CMD_GET_CCDTEMP        0xbf
#define CMD_GET_TARGETTEMP     0xbe
#define CMD_GET_COOLERSTATUS   0xbd
#define CMD_GET_COOLERPOWER    0xbc

#define TRUE_INV_PROT  0xaa55
#define FALSE_INV_PROT 0x55aa
#define HIGH_MASK_PROT 0xaa00

#define SPI_TIMEOUT  100

#define TEMP_OFFSET  1280
#define TEMP_TARGET  1380

// ─── State ────────────────────────────────────────────────────────────────────
static volatile uint16_t g_roiStartY  = 0;
static volatile uint16_t g_roiNumY    = 1000;
static volatile uint16_t g_exposure   = 0;
static volatile uint8_t  g_binning    = 0;
static volatile uint16_t g_lineDelay  = 100;
static volatile bool     g_doReadFrame      = false;
static volatile bool     g_doClearSubstrate = false;
static volatile bool     g_doOff15V         = false;
static volatile bool     g_doClearFrame     = false;
static volatile bool     g_doInitCamera     = false;
static uint16_t          g_diagTemp   = 1530;


// ═══════════════════════════════════════════════════════════════════════════════
// Pixel clock loops
// ═══════════════════════════════════════════════════════════════════════════════

static void pixelLoop(uint8_t k_even, uint8_t k_odd, uint16_t count) {
    __asm__ __volatile__(
        "   cp  %A[cnt], __zero_reg__     \n"
        "   cpc %B[cnt], __zero_reg__     \n"
        "   breq 2f                       \n"
        "1: out %[port], %[ke]            \n"
        "   out %[port], %[ko]            \n"
        "   out %[port], %[ke]            \n"
        "   out %[port], %[ko]            \n"
        "   out %[port], %[ke]            \n"
        "   out %[port], %[ko]            \n"
        "   out %[port], %[ke]            \n"
        "   out %[port], %[ko]            \n"
        "   sbiw %[cnt], 1               \n"
        "   brne 1b                       \n"
        "2:                               \n"
        :
        : [port] "I" (_SFR_IO_ADDR(PORTC)),
          [ke]   "r" (k_even),
          [ko]   "r" (k_odd),
          [cnt]  "w" (count)
        :
    );
}

static void __attribute__((noinline)) pixel1000(uint8_t ks, uint8_t ke, uint8_t ko) {
#define P4  "out %[port],%[ks]\n out %[port],%[ko]\n" \
            "out %[port],%[ke]\n out %[port],%[ko]\n" \
            "out %[port],%[ke]\n out %[port],%[ko]\n" \
            "out %[port],%[ke]\n out %[port],%[ko]\n"
//#define P4  "out %[port],%[ks]\n nop\n out %[port],%[ko]\n" \
            "out %[port],%[ke]\n nop\n out %[port],%[ko]\n" \
            "out %[port],%[ke]\n nop\n out %[port],%[ko]\n" \
            "out %[port],%[ke]\n nop\n out %[port],%[ko]\n"
#define P8   P4 P4
#define P40  P8 P8 P8 P8 P8 
#define P200 P40 P40 P40 P40 P40
    __asm__ __volatile__(
        P200 P200 P200 P200 P200
        : : [port]"I"(_SFR_IO_ADDR(PORTC)), [ks]"r"(ks), [ke]"r"(ke), [ko]"r"(ko) :
    );
#undef P4
#undef P8
#undef P40
#undef P200
}

static inline void dummyPixels(uint16_t count) {
    pixelLoop(K_DUM_0, K_DUM_1, count);
}

static inline void activePixels(uint16_t count) {
    pixelLoop(K_ACT_0, K_ACT_1, count);
}

static inline void __attribute__((always_inline)) fixedDelay(void) {
    __asm__ __volatile__(
        "ldi r24, 10     \n"  // 1 cycle
        "1: dec r24      \n"  // 1 cycle  } × 10 = 30 cycles
        "brne 1b         \n"  // 2/1 cycle}
        "nop             \n"  // 1 cycle
        "nop             \n"  // 1 cycle
        ::: "r24"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// CXD1267 vertical driver
// ═══════════════════════════════════════════════════════════════════════════════

static void rowTransfer(void) {
    PORTD=0xcb; delayMicroseconds(5);
    PORTD=0xdb; delayMicroseconds(5);
    PORTD=0xda; delayMicroseconds(5);
    PORTD=0xfa; delayMicroseconds(5);
    PORTD=0xea; delayMicroseconds(5);
    PORTD=0xee; delayMicroseconds(5);
    PORTD=0xce; delayMicroseconds(5);
    PORTD=0xcf; delayMicroseconds(5);
}

static void shiftToVertical(void) {
    rowTransfer();

    PORTD=0xc7; delayMicroseconds(20);
    //PORTD=0xc7; delayMicroseconds(5);
    //PORTD=0xc7; delayMicroseconds(5);
    //PORTD=0xc7; delayMicroseconds(5);

    PORTD=0xcb; delayMicroseconds(5);

    PORTD=0xd9; delayMicroseconds(20); 
    //PORTD=0xd9; delayMicroseconds(5);
    //PORTD=0xd9; delayMicroseconds(5); 
    //PORTD=0xd9; delayMicroseconds(5);

    PORTD=0xdb; delayMicroseconds(5);

    PORTD=0xfa; delayMicroseconds(5); 
    PORTD=0xea; delayMicroseconds(5);
    PORTD=0xee; delayMicroseconds(5); 
    PORTD=0xce; delayMicroseconds(5);
    PORTD=0xcf; delayMicroseconds(5);
}

static void clearSubstrate(void) {
    PORTD=0xcb; delayMicroseconds(5);
    PORTD=0xdb; delayMicroseconds(5);
    PORTD=0x9a; delayMicroseconds(5);
    PORTD=0xba; delayMicroseconds(5);
    PORTD=0xaa; delayMicroseconds(5);
    PORTD=0xee; delayMicroseconds(5);
    PORTD=0xce; delayMicroseconds(5);
    PORTD=0xcf; delayMicroseconds(5);
}

static void ccdClearLine(void) {
    pixelLoop(K_CLR_A, K_CLR_B, 1600);
}

static void ccdClearFrame(void) {
    for (uint16_t x = 0; x < 1012; x++) {
        rowTransfer();
    }
    ccdClearLine();
}


// ═══════════════════════════════════════════════════════════════════════════════
// Frame readout — called from loop() so delay() works correctly
// ═══════════════════════════════════════════════════════════════════════════════

static void ccdReadFrame(void) {
    PORTB |= PB_DSB;

    if (g_exposure > 55) {
        if (g_exposure <= 1000) {
            clearSubstrate();
        }
        delay(g_exposure - 55);
        ccdClearLine();
        ccdClearFrame();
    } else {
        ccdClearLine();
        ccdClearFrame();
        clearSubstrate();
        delay(g_exposure);
    }

    shiftToVertical();

    uint16_t skip = 10 + g_roiStartY;
    for (uint16_t y = 0; y < skip; y++) {
        rowTransfer();
    }
    ccdClearLine();

    rowTransfer();
    ccdClearLine();

    for (uint16_t y = 0; y < g_roiNumY; y++) {
        rowTransfer();
        dummyPixels(12);
        activePixels(1);

        if (!g_binning) {
            pixel1000(K_ACT_0, K_ACT_0, K_ACT_1);
            pixel1000(K_ACT_0, K_ACT_0, K_ACT_1);
            pixel1000(K_ACT_0, K_ACT_0, K_ACT_1);
            pixel1000(K_ACT_0, K_ACT_0, K_ACT_1);
            pixel1000(K_ACT_0, K_ACT_0, K_ACT_1);
            pixel1000(K_ACT_0, K_ACT_0, K_ACT_1);
        } else {
            pixel1000(K_BIN_S, K_BIN_A, K_BIN_B);
            pixel1000(K_BIN_S, K_BIN_A, K_BIN_B);
            pixel1000(K_BIN_S, K_BIN_A, K_BIN_B);
            pixel1000(K_BIN_S, K_BIN_A, K_BIN_B);
            pixel1000(K_BIN_S, K_BIN_A, K_BIN_B);
            pixel1000(K_BIN_S, K_BIN_A, K_BIN_B);
        }

        PORTC = K_TRAIL;
        dummyPixels(21);
        //delayMicroseconds(g_lineDelay);
    }

    PORTB &= ~PB_DSB;
}


// ═══════════════════════════════════════════════════════════════════════════════
// SPI 
// ═══════════════════════════════════════════════════════════════════════════════

static uint8_t g_spiCmd = 0;

static uint16_t resi(void) {
    uint8_t  x, count;
    uint16_t buf  = 0;  // Accumulates incoming bits from MOSI
    uint16_t buf2 = 0;  // Holds the response value to send back on MISO

    // ── Phase 1: Read 8-bit command byte from master ──────────────────────────
    // The master clocks out 8 bits MSB-first. We sample MOSI on each rising
    // edge of SCK and shift the bit into buf.
    for (x = 0; x < 8; x++) {

        // Wait for SCK to go high (rising edge), timing out if it takes too long
        count = 0;
        while ((PINB & PB_SCK) == 0) {
            if (++count > SPI_TIMEOUT) {
                return 0xFFFF;
            }
        }

        // Shift buf left to make room for the new bit, then sample MOSI
        buf = (buf << 1);
        if (PINB & PB_MOSI) {
            buf |= 0x0001;
        }

        // Wait for SCK to go low again before sampling the next bit
        count = 0;
        while ((PINB & PB_SCK) != 0) {
            if (++count > SPI_TIMEOUT) {
                return 0xFFFF;
            }
        }
    }

    // ── Phase 2: Decode command and prepare response ──────────────────────────
    // buf now holds the 8-bit command byte. Store it in the global so the ISR
    // can read it after we return, then reset buf to accumulate the parameter.
    g_spiCmd = (uint8_t)buf;
    buf = 0;

    // Look up the response value for read commands. Write commands don't send
    // a meaningful response so buf2 stays 0 for those.
    switch (g_spiCmd) {
        case CMD_GET_VERSION:
            // Return the firmware version number
            buf2 = VERSION;
            break;
        case CMD_GET_CCDTEMP:
            // Return the current CCD temperature and advance the diagnostic counter
            buf2 = g_diagTemp;
            g_diagTemp++;
            break;
        case CMD_GET_TARGETTEMP:
            // Return the fixed target temperature setpoint
            buf2 = TEMP_TARGET;
            break;
        case CMD_GET_COOLERSTATUS:
            // Cooler not active — return the false sentinel value
            buf2 = FALSE_INV_PROT;
            break;
        case CMD_GET_COOLERPOWER:
            // Return zero power with the high-byte mask applied
            buf2 = (0 | HIGH_MASK_PROT);
            break;
        default:
            buf2 = 0;
            break;
    }

    // ── Phase 3: Simultaneously receive 16-bit parameter and send response ────
    // While the master clocks out its 16-bit parameter, we clock our response
    // back. On each rising edge we sample MOSI into buf and output the next
    // bit of buf2 onto MISO, MSB first.
    for (x = 0; x < 16; x++) {

        // Wait for SCK rising edge
        count = 0;
        while ((PINB & PB_SCK) == 0) {
            if (++count > SPI_TIMEOUT) {
                return 0xFFFF;
            }
        }

        // Shift buf left and sample the incoming parameter bit from MOSI
        buf = (buf << 1);
        if (PINB & PB_MOSI) {
            buf |= 0x0001;
        }

        // Output the MSB of our response onto MISO, then shift buf2 left
        // ready for the next bit
        if (buf2 & 0x8000) {
            PORTB |= PB_MISO;
        } else {
            PORTB &= ~PB_MISO;
        }
        buf2 <<= 1;

        // Wait for SCK to go low before the next bit
        count = 0;
        while ((PINB & PB_SCK) != 0) {
            if (++count > SPI_TIMEOUT) {
                return 0xFFFF;
            }
        }
    }

    // Return the 16-bit parameter received from the master
    return buf;
}


// ═══════════════════════════════════════════════════════════════════════════════
// ISR — handles all commands except CMD_READFRAME which is flagged to loop()
// ═══════════════════════════════════════════════════════════════════════════════

ISR(PCINT0_vect) {
    PORTB |= PB_TEC;
    PCMSK0 = 0x00;

    uint16_t param = resi();
    if (param == 0xFFFF) {
        PORTB &= ~PB_TEC;
        PCMSK0 = (1 << PCINT5);
        return;
    }
    uint8_t  cmd   = g_spiCmd;

    switch (cmd) {
        case CMD_READFRAME:
            g_doReadFrame = true;
            PORTB &= ~PB_TEC;
            PCMSK0 = (1 << PCINT5);
            return;
        case CMD_CLEAR_SUBSTRATE:
            g_doClearSubstrate = true;
            PORTB &= ~PB_TEC;
            PCMSK0 = (1 << PCINT5);
            return;
        case CMD_OFF15V:
            g_doOff15V = true;
            PORTB &= ~PB_TEC;
            PCMSK0 = (1 << PCINT5);
            return;
        case CMD_SET_ROISTARTY:
            g_roiStartY = param;
            break;
        case CMD_SET_ROINUMY:
            g_roiNumY = param;
            break;
        case CMD_SET_EXPOSURE:
            g_exposure = param;
            break;
        case CMD_SET_BINNING:
            if (param == 0 || param == 1) {
                g_binning = (uint8_t)param;
            }
            break;
        case CMD_CLEARFRAME:
            g_doClearFrame = true;
            PORTB &= ~PB_TEC;
            PCMSK0 = (1 << PCINT5);
            return;
        case CMD_INITMCU:
            g_doInitCamera = true;
            PORTB &= ~PB_TEC;
            PCMSK0 = (1 << PCINT5);
            return;
        case CMD_SET_DELAYPERLINE:
            g_lineDelay = param;
            break;
        default:
            break;
    }

    PORTB &= ~PB_TEC;
    PCMSK0 = (1 << PCINT5);
}


// ═══════════════════════════════════════════════════════════════════════════════
// PORTB helpers
// ═══════════════════════════════════════════════════════════════════════════════

static inline void toggleDHT(void) {
    PORTB ^= PB_DHT;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Init + Arduino entry points
// ═══════════════════════════════════════════════════════════════════════════════

void initCamera(void) {
    CLKPR = 0x80; CLKPR = 0x00;
    DDRD  = 0xFF; PORTD = PORTD_IDLE;
    DDRC  = 0x3E; PORTC = 0x00;
    DDRB  = PB_MISO | PB_TEC | PB_DHT | PB_DSB; PORTB = 0x00;
    clearSubstrate();
    ccdClearFrame();
}

void setup(void) {
    initCamera();
    PCICR  = (1 << PCIE0);
    PCMSK0 = (1 << PCINT5);
    sei();
}

void loop(void) {
    static uint32_t lastToggle = 0;
    if (millis() - lastToggle >= 1000) {
        lastToggle = millis();
        toggleDHT();
    }

    if (g_doInitCamera) {
        g_doInitCamera = false;
        initCamera();
    } else if (g_doClearFrame) {
        g_doClearFrame = false;
        ccdClearFrame();
    } else if (g_doClearSubstrate) {
        g_doClearSubstrate = false;
        clearSubstrate();
    } else if (g_doOff15V) {
        g_doOff15V = false;
        PORTD = 0x4f;
        delay(1);
        PORTD = PORTD_IDLE;
    } else if (g_doReadFrame) {
        g_doReadFrame = false;
        ccdReadFrame();
    }
}