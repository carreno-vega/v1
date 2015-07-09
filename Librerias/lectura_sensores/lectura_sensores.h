#ifndef lectura_sensores_H
#define lectura_sensores_H
#include <Arduino.h> //It is very important to remember this! note that if you are using Arduino 1.0 IDE, change "WProgram.h" to "Arduino.h"
 
class lectura
{
public:
        lectura();
        ~lectura();
        int sensor(byte sens_read);
private:
        byte _sens_read;
        int _sensor_value;
       
};
 
#endif

