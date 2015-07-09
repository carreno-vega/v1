#include "lectura_sensores.h" //include the declaration for this class
 
const byte sensor_1  = A0; 
const byte sensor_2  = A1;
const byte sensor_3  = A2;
const byte sensor_4  = A3;
const byte sensor_5  = A4;
const byte sensor_6  = A5;
int sensor_value;

//<<constructor>> setup the LED, make pines an OUTPUT

lectura::lectura()
{
  
}
 
//<<destructor>>
lectura::~lectura()
{
/*nothing to destruct*/
}
 
int lectura::sensor(byte sens_read)
{
  switch(sens_read) //lectura de un sensor cada 0.1 (s), equivale a un tiempo de lectura = 0.6 (s) x sensor,  fs= (1/0.6)
  {
    case(1):
    {
      sensor_value = analogRead(sensor_1); 
      break;
    }
    case(2):
    {
      sensor_value = analogRead(sensor_2); 
      break;
    }
    case(3):
    {
      sensor_value = analogRead(sensor_3); 
      break;
    }
    case(4):
    {
      sensor_value = analogRead(sensor_4); 
      break;
    }
    case(5):
    {
      sensor_value = analogRead(sensor_5); 
      break;
    }
    case(6):
    {
      sensor_value = analogRead(sensor_6); 
      break;
    }
  default:break;
  }
  
return sensor_value;
}

