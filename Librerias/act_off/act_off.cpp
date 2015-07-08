#include "act_off.h" //include the declaration for this class
 
const byte rele_D8  = 8; //use the LED @ Arduino pin 13, this should not change so make it const (constant)
const byte rele_D9  = 9;
const byte rele_D10 = 10;
const byte rele_D11 = 11;

//<<constructor>> setup the LED, make pines an OUTPUT
act::act()
{
    pinMode(rele_D8,  OUTPUT); //make that pin an OUTPUT
    pinMode(rele_D9,  OUTPUT); //make that pin an OUTPUT
    pinMode(rele_D10, OUTPUT); //make that pin an OUTPUT
    pinMode(rele_D11, OUTPUT); //make that pin an OUTPUT
}
 
//<<destructor>>
act::~act()
{
/*nothing to destruct*/
}
 
//apaga RELE'S
void act::off()
{
        digitalWrite(rele_D8,LOW); //set the pin HIGH and thus turn LED on
        digitalWrite(rele_D9,LOW);
        digitalWrite(rele_D10,LOW);
        digitalWrite(rele_D11,LOW);

}
 

