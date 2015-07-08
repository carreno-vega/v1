#ifndef act_off_H
#define act_off_H
 
#include <Arduino.h> //It is very important to remember this! note that if you are using Arduino 1.0 IDE, change "WProgram.h" to "Arduino.h"
 
class act //actuadores
{
public:
        act();
        ~act();
        void off();
};
 
#endif

