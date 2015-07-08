#ifndef manual_control_H
#define manual_control_H
#include <Arduino.h> //It is very important to remember this! note that if you are using Arduino 1.0 IDE, change "WProgram.h" to "Arduino.h"
 
class control //actuadores
{
public:
        control();
        ~control();
        void manual(byte flag_control_m);
private:
        byte _flag_control_m;
};
 
#endif

