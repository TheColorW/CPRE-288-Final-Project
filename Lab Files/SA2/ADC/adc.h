#include "Timer.h"
#include "lcd.h"
#include "adc.h"
#include "driverlib/interrupt.h"


int main(void)
{

    int rawReading;
    timer_init();
    IntMasterEnable(); // for interrrupts
    lcd_init();
    lcd_printf("hello");
    adc_init();

    while (1)
    {
        lcd_printf("69");
        rawReading = adc_read();
// print 4 characters
        lcd_printf("IR raw: %4d", rawReading);
        lcd_printf("67");
        timer_waitMillis(200); // delay
        lcd_printf("hello again");

    }
}
