#include <Arduino.h>
#include "config.h"

int serial_putc( char c, struct __file * )
{
#ifdef DEBUG_PRINT_USE_SERIAL3
    Serial3.write(c);
#elif defined DEBUG_PRINT_USE_SERIAL
    Serial.write(c);
#endif
    return c;
}

void log_init(void)
{
    fdevopen(&serial_putc, 0);
}
