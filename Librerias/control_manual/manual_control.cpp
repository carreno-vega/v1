#include "manual_control.h" //include the declaration for this class
 
const byte rele_D8  = 8; //use the LED @ Arduino pin 13, this should not change so make it const (constant)
const byte rele_D9  = 9;
const byte rele_D10 = 10;
const byte rele_D11 = 11;

//<<constructor>> setup the LED, make pines an OUTPUT
control::control()
{
    pinMode(rele_D8,  OUTPUT); //make that pin an OUTPUT
    pinMode(rele_D9,  OUTPUT); //make that pin an OUTPUT
    pinMode(rele_D10, OUTPUT); //make that pin an OUTPUT
    pinMode(rele_D11, OUTPUT); //make that pin an OUTPUT
}
 
//<<destructor>>
control::~control()
{
/*nothing to destruct*/
}
 
//control manual
void control::manual(byte flag_control_m)
{
  switch(flag_control_m)
  {
    case(0):  // Calentador OFF
    { 
      digitalWrite(rele_D8,LOW);
      break;
    }
    case(1):  // Calentador ON
    { 
      digitalWrite(rele_D8,HIGH);
      break;
    }
    case(2):  // Auxiliar OFF
    { 
      break;
    }
    case(3):  // Auxiliar ON
    { 
      break;
    }
    case(4):  // Agitador OFF
    { 
      digitalWrite(rele_D9,LOW);
      break;
    }
    case(5):  // Agitador ON
    { 
      digitalWrite(rele_D9,HIGH);
      break;
    }
    case(6):  // Compresor OFF
    { 
      digitalWrite(rele_D10,LOW);
      break;
    }
    case(7):  // Compresor ON
    { 
      digitalWrite(rele_D10,HIGH);
      break;
    }
    case(8):  // Bomba OFF
    { 
      digitalWrite(rele_D11,LOW);
      break;
    }
    case(9):  // Bomba ON
    { 
      digitalWrite(rele_D11,HIGH);
      break;
    }
    default:break;
  }

}
 

