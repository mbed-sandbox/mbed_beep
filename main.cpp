
#include "mbed.h"
#include "beep/beep.h"
#include "Buzzer/Buzzer.h"

Buzzer buzz(P3_26);

int main()
{
buzz.long_beep();
	while(1)
	{

	}
}
