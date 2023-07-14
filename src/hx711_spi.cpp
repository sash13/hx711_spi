/**
 *
 * HX711_SPI library for Arduino
 * https://github.com/bogde/HX711_SPI
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/
#include <Arduino.h>
#include "HX711_spi.h"

#define GAIN128      0b10000000
#define GAIN64       0b10101000
#define GAIN32       0b10100000
#define CLOCK        0b10101010
#define SIGNAL_LOW   0x0
#define RESET_SIGNAL 0xFF

// from https://gist.github.com/amotl/94c83f3ecc3dbf181da708fdd8ef9f45
static byte lookup[] = {
    0x00, 0x01, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x05, 0x00, 0x00, 0x06, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x08, 0x09, 0x00, 0x00, 0x0a, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0c, 0x0d, 0x00, 0x00, 0x0e, 0x0f
};

// struct fields_tag {
//    // LSB
//    unsigned int f4:6;
//    unsigned int f3:4;
//    unsigned int f3:2;
//    unsigned int f3:0;
//    // MSB
   
//    // LSB
//    unsigned int f4:6;
//    unsigned int f3:4;
//    unsigned int f3:2;
//    unsigned int f3:0;
//    // MSB
// };


// typedef union some_reg_tag {
//     uint8_t raw;
//     struct fields_tag fields;
// } some_reg_t;

// TEENSYDUINO has a port of Dean Camera's ATOMIC_BLOCK macros for AVR to ARM Cortex M3.
#define HAS_ATOMIC_BLOCK (defined(ARDUINO_ARCH_AVR) || defined(TEENSYDUINO))

// Whether we are running on either the ESP8266 or the ESP32.
#define ARCH_ESPRESSIF (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))

// Whether we are actually running on FreeRTOS.
#define IS_FREE_RTOS defined(ARDUINO_ARCH_ESP32)

// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU \
    ( \
    ARCH_ESPRESSIF || \
    defined(ARDUINO_ARCH_SAM)     || defined(ARDUINO_ARCH_SAMD) || \
    defined(ARDUINO_ARCH_STM32)   || defined(TEENSYDUINO) \
    )

#if HAS_ATOMIC_BLOCK
// Acquire AVR-specific ATOMIC_BLOCK(ATOMIC_RESTORESTATE) macro.
#include <util/atomic.h>
#endif


#if ARCH_ESPRESSIF
// ESP8266 doesn't read values between 0x20000 and 0x30000 when DOUT is pulled up.
#define DOUT_MODE INPUT
#else
#define DOUT_MODE INPUT_PULLUP
#endif

SPIClass *spi = NULL;

// HX711_SPI::HX711_SPI() {
// }

HX711_SPI::~HX711_SPI() {
}

void HX711_SPI::begin(byte dout, byte pd_sck, byte gain, int speed) {
    PD_SCK = pd_sck;
    DOUT = dout;

    pinMode(PD_SCK, OUTPUT);
    pinMode(DOUT, DOUT_MODE);

    spi = new SPIClass(VSPI);
    spi->begin();
    spi->beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE1));
    set_gain(gain);

    digitalWrite(PD_SCK, LOW);
}

bool HX711_SPI::is_ready() {
    return digitalRead(DOUT) == LOW;
}

void HX711_SPI::set_gain(byte gain) {
    switch (gain) {
        case 128:       // channel A, gain factor 128
            GAIN = GAIN128;
            break;
        case 64:        // channel A, gain factor 64
            GAIN = GAIN64;
            break;
        case 32:        // channel B, gain factor 32
            GAIN = GAIN32;
            break;
    }

}

long HX711_SPI::read() {

    // Wait for the chip to become ready.
    wait_ready();

    // Define structures for reading data into.
    unsigned long value = 0;
    uint8_t data[7] = { 0 };
    uint8_t filler = 0x00;

    // Protect the read sequence from system interrupts.  If an interrupt occurs during
    // the time the PD_SCK signal is high it will stretch the length of the clock pulse.
    // If the total pulse time exceeds 60 uSec this will cause the HX711_SPI to enter
    // power down mode during the middle of the read sequence.  While the device will
    // wake up when PD_SCK goes low again, the reset starts a new conversion cycle which
    // forces DOUT high until that cycle is completed.
    //
    // The result is that all subsequent bits read by shiftIn() will read back as 1,
    // corrupting the value returned by read().  The ATOMIC_BLOCK macro disables
    // interrupts during the sequence and then restores the interrupt mask to its previous
    // state after the sequence completes, insuring that the entire read-and-gain-set
    // sequence is not interrupted.  The macro has a few minor advantages over bracketing
    // the sequence between `noInterrupts()` and `interrupts()` calls.
    // #if HAS_ATOMIC_BLOCK
    // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

    // #elif IS_FREE_RTOS
    // // Begin of critical section.
    // // Critical sections are used as a valid protection method
    // // against simultaneous access in vanilla FreeRTOS.
    // // Disable the scheduler and call portDISABLE_INTERRUPTS. This prevents
    // // context switches and servicing of ISRs during a critical section.
    // portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    // portENTER_CRITICAL(&mux);

    // #else
    // // Disable interrupts.
    // noInterrupts();
    // #endif

    data[0] = spi->transfer(SIGNAL_LOW);
    if (data[0] & 0x01 == 1) {
        return static_cast<long>(0);
    }

    // Pulse the clock pin 24 times to read the data.
    data[0] = CLOCK;
    data[1] = CLOCK;
    data[2] = CLOCK;
    data[3] = CLOCK;
    data[4] = CLOCK;
    data[5] = CLOCK;
    data[6] = GAIN;

    spi->transfer(data, 7);

    // Set the channel and the gain factor for the next reading using the clock pin.
    // for (unsigned int i = 0; i < GAIN; i++) {
    //     digitalWrite(PD_SCK, HIGH);
    //     #if ARCH_ESPRESSIF
    //     delayMicroseconds(1);
    //     #endif
    //     digitalWrite(PD_SCK, LOW);
    //     #if ARCH_ESPRESSIF
    //     delayMicroseconds(1);
    //     #endif
    // }

    // #if IS_FREE_RTOS
    // // End of critical section.
    // portEXIT_CRITICAL(&mux);

    // #elif HAS_ATOMIC_BLOCK
    // }

    // #else
    // // Enable interrupts again.
    // interrupts();
    // #endif

    // Replicate the most significant bit to pad out a 32-bit signed integer
    // if (data[2] & 0x80) {
    //     filler = 0xFF;
    // } else {
    //     filler = 0x00;
    // }

    // Construct a 32-bit signed integer
    // value = ( static_cast<unsigned long>(filler) << 24
    //         | static_cast<unsigned long>(data[2]) << 16
    //         | static_cast<unsigned long>(data[1]) << 8
    //         | static_cast<unsigned long>(data[0]) );

    for (int i = 0; i < 6; ++i)
    {
        value = (value << 4) + lookup[data[i] & 0x55];
    }

    return static_cast<long>(value - ((value & 0x800000) << 1));
}

void HX711_SPI::wait_ready(unsigned long delay_ms) {
    // Wait for the chip to become ready.
    // This is a blocking implementation and will
    // halt the sketch until a load cell is connected.
    while (!is_ready()) {
        // Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
        // https://github.com/bogde/HX711_SPI/issues/73
        delay(delay_ms);
    }
}

bool HX711_SPI::wait_ready_retry(int retries, unsigned long delay_ms) {
    // Wait for the chip to become ready by
    // retrying for a specified amount of attempts.
    // https://github.com/bogde/HX711_SPI/issues/76
    int count = 0;
    while (count < retries) {
        if (is_ready()) {
            return true;
        }
        delay(delay_ms);
        count++;
    }
    return false;
}

bool HX711_SPI::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
    // Wait for the chip to become ready until timeout.
    // https://github.com/bogde/HX711_SPI/pull/96
    unsigned long millisStarted = millis();
    while (millis() - millisStarted < timeout) {
        if (is_ready()) {
            return true;
        }
        delay(delay_ms);
    }
    return false;
}

long HX711_SPI::read_average(byte times) {
    long sum = 0;
    for (byte i = 0; i < times; i++) {
        sum += read();
        // Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
        // https://github.com/bogde/HX711_SPI/issues/73
        delay(0);
    }
    return sum / times;
}

double HX711_SPI::get_value(byte times) {
    return read_average(times) - OFFSET;
}

float HX711_SPI::get_units(byte times) {
    return get_value(times) / SCALE;
}

void HX711_SPI::tare(byte times) {
    double sum = read_average(times);
    set_offset(sum);
}

void HX711_SPI::set_scale(float scale) {
    SCALE = scale;
}

float HX711_SPI::get_scale() {
    return SCALE;
}

void HX711_SPI::set_offset(long offset) {
    OFFSET = offset;
}

long HX711_SPI::get_offset() {
    return OFFSET;
}

void HX711_SPI::power_down() {
    digitalWrite(PD_SCK, LOW);
    digitalWrite(PD_SCK, HIGH);
}

void HX711_SPI::power_up() {
    digitalWrite(PD_SCK, LOW);
}
