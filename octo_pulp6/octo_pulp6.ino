#include <avr/io.h>
#include <avr/interrupt.h>
// comentario prueba
// comentario 2
/*Varibles de prueba*/
int  led = 8;
int  led1 = 9;
int  led2 = 10;
int  led3 = 11;
byte timer_lectura;
/*id_trama enviada incrementa cada 1 seg*/
byte incrementador_tx; // incrementador de timer enviado hacia java
/*Timer1*/
int  aux_timer1; 
int  seconds;
/*Buffer de trama recibida*/
int  arreglo[10];
int  contador;
/*Identificadores de trama*/
byte  id_trama;
byte  estado_rx;
/*Variables Set point*/
byte  ph_sp;
byte  od_sp;  
byte  temp_sp;
byte  rpm_sp;  
byte  aux1_sp;
byte  aux2_sp;
/* Variables Control Manual*/
byte  inst1_man;
byte  inst2_man;  
byte  inst3_man;  
byte  aux1_man;
byte  aux2_man;  
byte  aux3_man;
/* Variables de calibracion*/
byte  flag_cal; 
byte  val_cal1; 
byte  val_cal2;
byte  aux1_cal;
byte  aux2_cal;
byte  aux3_cal; 
/*Variables enviadas*/
byte  sof_tx;
byte  id_trama_tx;                // normal = 1, confirm = 2
byte  estado_tx;               // envia estado q se recibiò
byte  sens1_tx;
byte  sens2_tx;
byte  sens3_tx;
byte  sens4_tx;
byte  sens5_tx;
byte  aux_tx;
byte  eof_tx;
/*Declaracion entradas analogas para sensores*/
byte controlador_ph   = A0;  //Lectura controlador pH 4-20 mA a 1- 5   Volts en entrada analoga A0
byte controlador_aux  = A1;  //Lectura controlador auxiliar 4-20 mA a 1- 5   Volts en entrada analoga A1
byte PT100            = A2;  //Lectura controlador auxiliar 4-20 mA a 1- 5   Volts en entrada analoga A1
byte sensor_ph        = A4; //Lectura sensor PH
byte sensor_aux       = A3; //Lectura sensor auxiliar
byte ADC_input        = A5; //Lectura ADC
float voltaje_ref_ADC;  
/**/
float sens1_read;
float sens2_read;
float sens3_read;
float sens4_read;
float sens5_read;
float sens6_read;
/*Variables propias de los sensores*/
float sensor_ph_value;
byte flag_ph1;
byte flag_ph2;
float sensor_ph_7_cal;
float sensor_ph_4_cal;
float paso_ph_cal;
/*variables controlador pH*/
float controlador_ph_value;
byte flag_ph1_cont;
byte flag_ph2_cont;
float controlador_ph7_cal;
float controlador_ph4_cal;
float paso_ph_cont;
/*Variables Filtro Butter para sensor pH*/
double ACoef[5];
double BCoef[5];
int NCoef;

double y[5]; //output samples
double x[5]; //input samples
int n;

float promedio_ph = 0;
int suma_ph = 0;
byte arreglo_ph[150];

void setup()
{
  for (int h = 0; h < 150; h++)
  {
    arreglo_ph[h] = 0;
  }
  
   NCoef = 4;
    /* 
   ACoef[0] = 0.000000374;
   ACoef[1] = 0.0000001496;
   ACoef[2] = 0.0000002244;
   ACoef[3] = 0.0000001496;
   ACoef[4] = 0.0000000374;
   
   BCoef[0] = 1;
   BCoef[1] = -3.8687;
   BCoef[2] = 5.6145;
   BCoef[3] = -3.6228;
   BCoef[4] = 0.8769;
   */
   ACoef[0] = 0.0466 ;
   ACoef[1] = 0.1863;
   ACoef[2] = 0.2795 ;
   ACoef[3] = 0.1863;
   ACoef[4] = 0.0466;
   
   BCoef[0] =  1.0000;
   BCoef[1] = -0.7821;
   BCoef[2] =  0.6800;
   BCoef[3] = -0.1827;
   BCoef[4] =  0.0301; 
   
   x[0] = 0;
   x[1] = 0;
   x[2] = 0;
   x[3] = 0;
   x[4] = 0;
   
   y[0] = 0;
   y[1] = 0;
   y[2] = 0;
   y[3] = 0;
   y[4] = 0;
   
   
   
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);    
  digitalWrite(led,LOW);  
  digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
  digitalWrite(led3,LOW);
  
  /*Variables a medir e implementar en progtama antes de instalacion*/
  voltaje_ref_ADC = 4.85;   //Voltaje medido
  
  flag_ph1 = 0;
  flag_ph2 = 0;
  paso_ph_cal = 20;  //paso inicial referencial, luego en funcion calibracion se toma el valor real del paso de pH 
  sensor_ph_7_cal = 1; //Valor inicial referencial para el voltaje en ADC para el sensor de pH igual a 7
  /*
  50mv/pH referencia
  1 V offset del sumador
  el sumador invierte señal sensor pH y suma offset
  ph7 = 1V, pH4 = 0.85 refencial
  paso inicial = (7-4)/(1-0.85)
  paso = 20 pH/volt
  */
  
flag_ph1_cont = 0;
flag_ph2_cont = 0;
controlador_ph7_cal = 2.641;
paso_ph_cont = 3.94;
/*
controlador pH
pH7 - 12 mA - 2.64 V (simulador)
pH4 - 8.54 mA - 1.88 V (simulador)
*/
  incrementador_tx  = 0;
  id_trama    = 0;
  estado_tx   = 0;
  contador    = 0;
  aux_timer1  = 0;
  seconds     = 0;
  sens1_tx    = 0;
  sens2_tx    = 0;
  sens3_tx    = 0;
  /* Inicio del timer1*/
  //Since Timer1 is 16 bits, it can hold a maximum value of (2^16 – 1) or 65535 a 16MHz.
  cli();          // Desaciva las interrupciones globales
  TCCR1A  = 0;     // pone el regitro TCCR1A entero a 0
  TCCR1B  = 0;     // pone el registro TCCR1B entero a 0
  OCR1A   = 124;     // configurado para 0.008 seg (125 Hz) OCR1A = 124, Comparación cada un seg 15624,   www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
  /*
  we divide our clock source by 1024. This gives us a timer resolution of 1/(16*10^6 / 1024), or 6.4e-5 seconds
  (# timer counts + 1) = (target time) / (timer resolution)
  (# timer counts + 1) = (1 s) / (6.4e-5 s)
  (# timer counts + 1) = 15625
  (# timer counts) = 15625 - 1 = 15624  El timer1 iguala este valor para activar la interrupecion, para 1 seg es 15624
  */
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);  //datasheet 32U4 
  sei();
}

double iir(float NewSample) 
{
  
  for(n=NCoef; n>0; n--) 
  {
      x[n] = x[n-1];
      y[n] = y[n-1];
   }
   
   //Calculate the new output
   x[0] = NewSample;
   y[0] = ACoef[0] * x[0];
   
   for(n=1; n<=NCoef; n++)
   {
     y[0] += (ACoef[n] * x[n]) - (BCoef[n] * y[n]);
   }
   
   Serial.print(NewSample);
   Serial.print(",");
   Serial.println(y[0],7);
   return y[0];
}

int deco_trama()
{
/* ID TRAMA
1	Trama de setpoint
2	Trama de control manual
3	Trama de calibraciòn*/

  id_trama    = arreglo[1];
  estado_rx   = arreglo[2];   // Incrementador
  
  switch(id_trama)
  {
    case(1): //trama = setpoint
    { 
      ph_sp       = arreglo[3];  
      od_sp       = arreglo[4];  
      temp_sp     = arreglo[5];  
      rpm_sp      = arreglo[6];  
      aux1_sp     = arreglo[7]; 
      aux2_sp     = arreglo[8];       
      break;
    }
    case(2): //trama = manual
    { 
      inst1_man     = arreglo[3];  //Si es 0=off calentar, 1= on calentar, 2=off auxiliar 3=on auxiliar, 4=off agitador 5= on agitador, 6= off compresor 7=on compresor 8= off bomba 9= on bomba
      inst2_man     = arreglo[4];  
      inst3_man     = arreglo[5];  
      aux1_man      = arreglo[6];  
      aux2_man      = arreglo[7];  
      aux3_man      = arreglo[8];
      break;
    }
    case(3):  // trama = calibracion
    { 
      flag_cal   = arreglo[3];  //Indica el sensor a calibrar
      val_cal1   = arreglo[4];  
      val_cal2   = arreglo[5];  
      aux1_cal   = arreglo[6];  //Para ph, aux1_cal=1 -> pH=4, aux1_cal=2 ->pH=7
      aux2_cal   = arreglo[7];  
      aux3_cal   = arreglo[8]; 
      break;
    }
    default:break;  
  }
}

void set_point()
{
  

}
void set_manual()
{
  switch(inst1_man)
  {
    case(0):  // Calentador OFF
    { 
      digitalWrite(led,LOW);
      break;
    }
    case(1):  // Calentador ON
    { 
      digitalWrite(led,HIGH);
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
      digitalWrite(led1,LOW);
      break;
    }
    case(5):  // Agitador ON
    { 
      digitalWrite(led1,HIGH);
      break;
    }
    case(6):  // Compresor OFF
    { 
      digitalWrite(led2,LOW);
      break;
    }
    case(7):  // Compresor ON
    { 
      digitalWrite(led2,HIGH);
      break;
    }
    case(8):  // Bomba OFF
    { 
      digitalWrite(led3,LOW);
      break;
    }
    case(9):  // Bomba ON
    { 
      digitalWrite(led3,HIGH);
      break;
    }
    default:break;
  }
}

byte set_calibracion(byte flag_c)
{
  switch(flag_c)
  {
    case(1):  // Calibracion sensor de pH
    { 
      if(aux1_cal == 1) //calibracion pH4
      {
        sensor_ph_4_cal = analogRead(sensor_ph) * (voltaje_ref_ADC / 1024); //lectura pH4 de bit a milivolts
        flag_ph1 = 1;
        digitalWrite(led1,HIGH);
      }
      else if(aux1_cal == 2) //calibracion con pH7
      {
        sensor_ph_7_cal = analogRead(sensor_ph) * (voltaje_ref_ADC / 1024); //lectura pH4 de bit a milivolts
        flag_ph2 = 1;
        digitalWrite(led2,HIGH);
      }
      if((flag_ph1 == 1) && (flag_ph2 == 1))
      {
        paso_ph_cal = ((7 - 4) / (sensor_ph_7_cal - sensor_ph_4_cal)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph1 = 0;
        flag_ph2 = 0;
        digitalWrite(led3,HIGH);
      }
      
      break;
    }
    case(2):  // Calibracion Oxigeno Disuelto
    { 
      break;
    }
    case(3):  // Calibracion Controlador pH 420 (1)
    { 
      if(aux1_cal == 3) //calibracion pH4
      {
        controlador_ph4_cal = analogRead(controlador_ph) * (voltaje_ref_ADC / 1024); //lectura pH4 de bit a milivolts
        flag_ph1_cont = 1;
        digitalWrite(led1,HIGH);
      }
      else if(aux1_cal == 4) //calibracion con pH7
      {
        controlador_ph7_cal = analogRead(controlador_ph) * (voltaje_ref_ADC / 1024); //lectura pH4 de bit a milivolts
        flag_ph2_cont = 1;
        digitalWrite(led2,HIGH);
      }
      if((flag_ph1_cont == 1) && (flag_ph2_cont == 1))
      {
        paso_ph_cont = ((7 - 4) / (controlador_ph7_cal - controlador_ph4_cal)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph1_cont = 0;
        flag_ph2_cont = 0;
        digitalWrite(led3,HIGH);
      }
      break;     
    }
    case(4):  // Calibracion Controlador auxiliar 420 (2)
    { 
      break;
    }
    case(5):  // Calibracion PT100
    { 
      break;
    }
     case(6):  // Calibracion auxiliar
    { 
      break;
    }
    default:break;
  }
}  

void lectura_sensores()
{
  switch(timer_lectura) //lectura de un sensor cada 0.008 (s), equivale a un tiempo de lectura = 0.048 (s) x sensor.
  {
    case(1):
    {
      sens1_read = analogRead(sensor_ph);
      break;
    }
    case(2):
    {
      sens2_read = analogRead(sensor_aux);
      break;
    }
    case(3):
    {
      sens3_read = analogRead(controlador_ph);
      break;
    }
    case(4):
    {
      sens4_read = analogRead(controlador_aux);               //Sensor 4-20
      break;
    }
    case(5):
    {
      sens5_read = analogRead(PT100);                        //Sensor auxiliar 2
      break;
    }
    case(6):
    {
      sens6_read = analogRead(ADC_input);                    //Sensor auxiliar 3
      timer_lectura = 0;
      break;
    }
  default:break;
  }
  
}
void procesar_datos()
{
  sensor_ph_value         = sens1_read * (voltaje_ref_ADC / 1024);
  sensor_ph_value         = (paso_ph_cal * sensor_ph_value) - (paso_ph_cal * sensor_ph_7_cal) + 7;    //voltaje a pH                                                   // decimas a entero para enviar como (byte)
  controlador_ph_value    = sens3_read * (voltaje_ref_ADC / 1024); 
  controlador_ph_value    = (paso_ph_cont * controlador_ph_value) - (paso_ph_cont * controlador_ph7_cal) + 7;  
  
    //shift the old samples
    arreglo_ph[0] = sens1_read;
    
    for(int g = 150; g > 0; g--)   //a 0.048 seg por muestra el arreglo 7.2 (s)//rellenar arreglo utilizando incrementador seconds (sugerencia no validada)
    {
       arreglo_ph[g] = arreglo_ph[g-1];
    }

    for (int g = 0; g < 150; g++)
    {
        suma_ph = suma_ph + arreglo_ph[g];
    }
    
    promedio_ph = suma_ph / 150;
    
    Serial.print(sens1_read,4);
    Serial.print(",");
    Serial.println(promedio_ph,4);;
    suma_ph = 0;

}
byte make_trama(byte a,byte b)
{
  sof_tx      = '#';
  id_trama_tx = a;                          // normal = 1, confirm = 2
  estado_tx   = b;                          // envia estado q se recibiò o incrementador para trama normal
  sens1_tx    = sensor_ph_value * 10; 
  sens2_tx    = 0;
  sens3_tx    = controlador_ph_value * 10;  // decimas a entero para enviar como (byte)
  sens4_tx    = 0;
  sens5_tx    = 0;
  aux_tx      = 0;
  eof_tx      = '%';
}


void send_trama()
{
  Serial.write(sof_tx);
  Serial.write(id_trama_tx);
  Serial.write(estado_tx);
  Serial.write(sens1_tx);
  Serial.write(sens2_tx);
  Serial.write(sens3_tx);
  Serial.write(sens4_tx);
  Serial.write(sens5_tx);
  Serial.write(aux_tx);
  Serial.write(eof_tx); 
}

int read_trama()
{
  arreglo[contador] = Serial.read();   
  if(arreglo[contador] == '#')                                    
  {
    do                                                 
    {
      contador++;
      arreglo[contador] = Serial.read();                    
    }
    while(contador < 9);  // largo trama 10  
    if((arreglo[0] == '#') && (arreglo[9] == '%'))
    {
      contador = 0;  
      return(1);
    }
    else
    {
      contador = 0;
      return(0);      
    }
  } 
}
/*Interrupcion Timer*/
ISR(TIMER1_COMPA_vect)   //Flag correspondiente a timer1 comparacion
{                        
    aux_timer1 = 1;     
}

void loop()
{
  if (Serial.available() > 0)
  {   
    if(read_trama())
    {
      deco_trama();
      make_trama(2,estado_rx);  //ide_trama 2= trama de confirmacion, se envia el estado_rx recibido a java
      // Se envia la trama de confirmaciòn con valores, por èstos debieran no considerarse porque lo importante es la confirmaciòn
      send_trama();
    }
    else
    {
      make_trama(3,0);   // id_trama = 3 hacia java = error de trama
      send_trama(); 
    } 
  }
 
  switch(id_trama)
  {
    case(1): //trama = setpoint
    { 
      //set_point();  // Etapa de control, fija paràmetros mìnimos o màximos
       //  Control pH y temperatura
     if (promedio_ph <= ph_sp)
     {
       digitalWrite(led3,HIGH);
     }
     else if(promedio_ph >= (ph_sp+0.2))
     {
       digitalWrite(led3,LOW);
     }
      id_trama = 0;
      break;
    }
    case(2): //trama = manual
    { 
      set_manual();
      id_trama = 0; 
      break;
    }
    case(3): //trama = calibraciòn
    { 
      set_calibracion(flag_cal);
      id_trama = 0; 
      break;
    }
    default:break;
  } 
  

 
  
  if(aux_timer1 == 1) // 0.5seg * 2 = 1 seg
  { 
    seconds++;             //aumenta cada 0.008 (s)
    timer_lectura++;      //timer de lectura de sensores aumenta cada 8 (ms)
    lectura_sensores();
    procesar_datos();
  
  //Serial.println(sens1_read);
    
    if(seconds == 1250)  // 10seg / 0.008 seg = 1250, envio cada 10 seg
    {
      incrementador_tx++;  // Retorna a cero despues de 255
      make_trama(1,incrementador_tx);  // 1 trama normal hacia java cada un segundo 1 second, con incrementador_tx++
      send_trama();
      seconds = 0;
    }
    aux_timer1 = 0;
  }
}  



