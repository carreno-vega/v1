#include <avr/io.h>
#include <avr/interrupt.h>
// comentario prueba
/*Varibles de prueba*/
int  led = 8;
int  led1 = 9;
int  led2 = 10;
int  led3 = 11;

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

/*Variables propias de los sensores*/
int sensor_ph;
byte flag_ph1;
byte flag_ph2;
int sensor_ph_7_cal;
int sensor_ph_4_cal;
int paso_ph_cal;


void setup()
{
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);    
  digitalWrite(led,LOW);  
  digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
  digitalWrite(led3,LOW);
  flag_ph1 = 0;
  flag_ph2 = 0;
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
  cli();          // Desaciva las interrupciones globales
  TCCR1A  = 0;     // pone el regitro TCCR1A entero a 0
  TCCR1B  = 0;     // pone el registro TCCR1B entero a 0
  OCR1A   = 15624;     //  Comparación cada un seg 15624, 31249 para dos seg,   www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);  //datasheet 32U4 
  sei();
}
int deco_trama()
{
  id_trama    = arreglo[1];
  estado_rx   = arreglo[2]; 
  
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
    case(1):  // Calibracion pH
    { 
      if(aux1_cal == 1) //calibracion pH4
      {
        sensor_ph_4_cal = analogRead(sensor_ph); //Almacenar milivolts para ph4
        flag_ph1 = 1;
      }
      if else(aux1_cal == 2) //calibracion con pH7
      {
        sensor_ph_7_cal = analogRead(sensor_ph); //Almacenar milivolts para ph7
        flag_ph2 = 1;
      }
      if((flag_ph1 == 1) && (flag_ph2 == 1))
      {
        paso_ph_cal = ((sensor_ph_4_cal - sensor_ph_7_cal) / 3);  //pendiente negativa -3 paso de ph x milivolt
        flag_ph1 = 0;
        flag_ph2 = 0;
      }
      
      break;
    }
    case(2):  // Calibracion Oxigeno Disuelto
    { 
      break;
    }
    case(3):  // Calibracion Temperatura
    { 
      break;
    }
    case(4):  // Calibracion RPM
    { 
      break;
    }
    case(5):  // Calibracion Auxiliar
    { 
      break;
    }
    default:break;
  }
}  
  
}

byte make_trama(byte a,byte b)
{
  sof_tx      = '#';
  id_trama_tx = a;                // normal = 1, confirm = 2
  estado_tx   = b;               // envia estado q se recibiò
  sens1_tx    = 1 + sens1_tx;
  sens2_tx    = 2;
  sens3_tx    = 3 + sens3_tx;
  sens4_tx    = 4;
  sens5_tx    = 5;
  aux_tx      = 6;
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
    while(contador < 9);    
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
    aux_timer1=1;     
}

void loop()
{
  if (Serial.available() > 0)
  {   
    if(read_trama())
    {
      deco_trama();
      make_trama(2,estado_rx);  //ide_trama 2= trama de confirmacion, se envia el estado_rx recibido a java
      send_trama();
    }
    else
    {
      make_trama(3,0);   // id_trama = 3 hacia java = error de trama
      send_trama(); 
    } 
  }
 
  if(id_trama == 1)
  {
   set_point();
   id_trama = 0; 
  } 
  if else(id_trama == 2)
  {
   set_manual();
   id_trama = 0; 
  }
  if else(id_trama == 3)
  {
   set_calibracion(flag_cal);
   id_trama = 0; 
  }    
 
 
  if(aux_timer1 == 1)
  {  
    seconds++;
    if (seconds == 1)   
    {
      incrementador_tx++;
      make_trama(1,incrementador_tx);  // 1 trama normal hacia java cada un segundo 1 second, con incrementador_tx++
      send_trama();
      seconds = 0;
    }
    aux_timer1 = 0;
  }
  delay(100);
}  



