#include <avr/io.h>
#include <avr/interrupt.h>
#include <act_off.h>


// comentario prueba
// comentario 2
/*Instancia de Clases ara librerias*/
act actuadores; //clase actuadores de act_off
/*Varibles de relé*/
int  led = 8;    //Relé 1 D8  en arduino_xbee
int  led1 = 9;   //Relé 2 D9  en arduino_xbee
int  led2 = 10;  //Relé 3 D10 en arduino_xbee
int  led3 = 11;  //Relé 4 D11 en arduino_xbee
/*Variables timer*/
byte timer_lectura;
int  timer_control;
byte timer_muestreo;
byte milisegundos;
byte segundos;
byte minutos;

/*Variable inicio programa*/
byte flag_inicio;
/*id_trama enviada incrementa cada 1 seg*/
byte incrementador_tx; // incrementador de timer enviado hacia java
/*Timer1*/
int  aux_timer1; 
int  seconds;
int  timer_loop;
/*Buffer de trama recibida*/
float  arreglo[10];
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
byte controlador_ph   = A0;  //Lectura controlador pH       4-20 mA a 1- 5   Volts en entrada analoga A0
byte controlador_aux  = A1;  //Lectura controlador auxiliar 4-20 mA a 1- 5   Volts en entrada analoga A1
byte sensor_ptc       = A2;  //Lectura sensor temperatura PTC en entrada analoga A2
byte sensor_ph        = A4; //Lectura sensor PH   (Ver en que entrada analoga se conecta el sensor A3 o A4)
byte sensor_aux       = A3; //Lectura sensor auxiliar
byte adc_input        = A5; //Lectura ADC
float voltaje_ref_ADC;  
/**/
float sens1_read;
float sens2_read;
float sens3_read;
float sens4_read;
float sens5_read;
float sens6_read;
/*Variable estado salidas digitales*/
byte estado_led;
byte estado_led1;
byte estado_led2;
byte estado_led3;
byte output_state;
/*Variables propias del sensor pH*/
float sensor_ph_value;
byte flag_ph1;
byte flag_ph2;
byte flag_ph3;
float sensor_ph_4_cal;
float sensor_ph_7_cal;
float sensor_ph_10_cal;
float paso_ph_cal;
/*variables controlador pH*/
float controlador_ph_value;
byte flag_ph1_cont;
byte flag_ph2_cont;
byte flag_ph3_cont;
float controlador_ph4_cal;
float controlador_ph7_cal;
float controlador_ph10_cal;
float paso_ph_cont;
/*Variables propias del sensor de temperatura PTC*/
float sensor_ptc_value;

/*Variables Filtro Butter para sensor pH, sensor temperatura PTC*/
float ACoef_sph[5];
float BCoef_sph[5];
int   NCoef_sph;
float y_sph[5]; //output samples
float x_sph[5]; //input samples
int   n_sph;
/*Variables Filtro IIR Butter para  sensor temperatura PTC*/
float ACoef_temp[5];
float BCoef_temp[5];
int   NCoef_temp;
float y_temp[5]; //output samples
float x_temp[5]; //input samples
int n_temp;
/*Variables Filtro Butter para sensor pH, sensor temperatura PTC*/
float ACoef_con_ph[5];
float BCoef_con_ph[5];
int   NCoef_con_ph;
float y_con_ph[5]; //output samples
float x_con_ph[5]; //input samples
int n_con_ph;




/*--Media movil--
float promedio_ph;
float promedio_temp;
float promedio_cont_aux;
int suma_ph;
int suma_temp;
int suma_cont_aux;
int arreglo_ph[100];
int arreglo_temp[100];
int arreglo_cont_aux[100];
*/

/* Varibales identificador sensor o actuador*/
byte ph_sens_or_cont;    //aux2_sp primer BIT 0 para sensor pH y 1 para controlador pH 
byte OD_sens_or_cont;    //aux2_sp segund BIT 0 para sensor OD y 1 para controlador OD
byte temp_sens_or_cont;  //aux2_sp tercer BIT 0 para sensor Temperatura y 1 para controlador Temperatura
byte rpm_sens_or_cont;   //aux2_sp cuarto BIT 0 para sensor RPM y 1 para controlador RPM
byte aux1_sens_or_cont;  //aux2_sp quinto BIT 0 para sensor auxiliar_1 y 1 para controlador auxiliar_1.  

void setup()
{
  for (int h = 0; h < 5; h++)
  {
    x_sph[h] = 0; //Arreglo sensor pH para filtro IIR Butterworth
    y_sph[h] = 0;
    
    x_temp[h] = 0; //Arreglo sensor temperatura ptc para filtro IIR Butterworth
    y_temp[h] = 0;
    
    x_con_ph[h] = 0; //Arreglo controlador pH para filtro IIR Butterworth
    y_con_ph[h] = 0;
  }
 /*Constantes filtro IIR Butter sensor ph*/
  NCoef_sph = 4;
   
  ACoef_sph[0] = 0.000806359;
  ACoef_sph[1] = 0.003225439;
  ACoef_sph[2] = 0.004838159;
  ACoef_sph[3] = 0.003225439;
  ACoef_sph[4] = 0.000806359;
   
  BCoef_sph[0] =  1;
  BCoef_sph[1] = -3.01755;
  BCoef_sph[2] =  3.50719;
  BCoef_sph[3] = -1.84755;
  BCoef_sph[4] =  0.37081;
   
   /*Constantes filtro IIR Butter sensor temperatura*/
  NCoef_temp = 4;
   
  ACoef_temp[0] = 0.000806359;
  ACoef_temp[1] = 0.003225439;
  ACoef_temp[2] = 0.004838159;
  ACoef_temp[3] = 0.003225439;
  ACoef_temp[4] = 0.000806359;
   
  BCoef_temp[0] =  1;
  BCoef_temp[1] = -3.01755;
  BCoef_temp[2] =  3.50719;
  BCoef_temp[3] = -1.84755;
  BCoef_temp[4] =  0.37081;
   
   /*Constantes filtro IIR Butter sensor ph*/
  NCoef_con_ph = 4;
   
  ACoef_con_ph[0] = 0.000806359;
  ACoef_con_ph[1] = 0.003225439;
  ACoef_con_ph[2] = 0.004838159;
  ACoef_con_ph[3] = 0.003225439;
  ACoef_con_ph[4] = 0.000806359;
   
  BCoef_con_ph[0] =  1;
  BCoef_con_ph[1] = -3.01755;
  BCoef_con_ph[2] =  3.50719;
  BCoef_con_ph[3] = -1.84755;
  BCoef_con_ph[4] =  0.37081;
   
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);    
  digitalWrite(led,LOW);  
  digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
  digitalWrite(led3,LOW);
  
  /*Variable inicio programa*/
  flag_inicio = 0;
  
  estado_led   = 0;
  estado_led1  = 0;
  estado_led2  = 0;
  estado_led3  = 0;
  output_state = 0;
  /*Variables a medir e implementar en programa antes de instalacion*/
  voltaje_ref_ADC = 4.85;   //Voltaje medido en tarjet, utilizado para conversion ADC
  
  flag_ph1 = 0;
  flag_ph2 = 0;
  flag_ph3 = 0;
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
   flag_ph3_cont = 0;
   controlador_ph7_cal = 2.641; //Variable referencia arrojar valor en pantalla de controlador
   paso_ph_cont = 3.94;         //Variable referencia arrojar valor en pantalla de controlador
   /*Variables timer*/
   timer_control = 0;          //timer de control de variables
   timer_lectura = 0;          //timer de lectura de datos
   timer_loop = 0;             //timer loop
   milisegundos = 0;
   segundos = 0;               //segundos incrementandose hasta el minuto segundos == 60
   minutos = 0;                //minutos incrementandose hasta la hora minutos == 60, para comparar con tiempo de ejecucion de setpoint.
   /*
   controlador pH
   pH7 - 12 mA - 2.64 V (simulador)
   pH4 - 8.54 mA - 1.88 V (simulador)
   */
   
  /*Variables media movil 
  promedio_ph = 0;
  promedio_temp = 0;
  promedio_cont_aux = 0;
  suma_ph = 0;
  suma_temp = 0;
  suma_cont_aux = 0;
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
  sens4_tx    = 0;
  sens5_tx    = 0;
  /* Inicio del timer1*/
  //Since Timer1 is 16 bits, it can hold a maximum value of (2^16 – 1) or 65535 a 16MHz.
  cli();          // Desaciva las interrupciones globales
  TCCR1A  = 0;     // pone el regitro TCCR1A entero a 0
  TCCR1B  = 0;     // pone el registro TCCR1B entero a 0
  OCR1A   = 624;     // configurado para 0.008 seg (125 Hz) OCR1A = 124, Comparación cada un seg 15624,   www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
  /*
  we divide our clock source by 1024. This gives us a timer resolution of 1/(16*10^6 / preescaler), 
  (# timer counts + 1) = (target time) / (timer resolution)
  (# timer counts + 1) = (0.01) / (1.6e-5 s)
  (# timer counts + 1) = 625
  (# timer counts) = 625 - 1 = 624  El timer1 iguala este valor para activar la interrupecion, para 0.01 seg es 624
  */
  TCCR1B |= (1 << WGM12);
  //TCCR1B |= (1 << CS10);  //CS12 = 1 CS11 = 0 CS10 = 1, PREESCALER = 1024
  TCCR1B |= (1 << CS12);   //CS12 = 1 CS11 = 0 CS10 = 0, PREESCALER = 256
  TIMSK1 |= (1 << OCIE1A);  //datasheet 32U4 
  sei();
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
      ph_sp       = arreglo[3]/10;  //ph_sp ph set point para sensor y actuador
      od_sp       = arreglo[4];  
      temp_sp     = arreglo[5];  
      rpm_sp      = arreglo[6];  
      aux1_sp     = arreglo[7];     //tiempo en minutos para ejecucion de trama especial set point solo para case(1) y case(2).
      aux2_sp     = arreglo[8];     //Tipo de set poin, 1 = , 2 = , 3= Setpoint manual
   
      segundos  = 0;                 //Llega trama de set_point especifica
      minutos   = 0;                 //tiempo de reseteo en minutos.
      
/*    ph_sens_or_cont    = bitRead(aux2_sp, 0);   //aux2_sp primer BIT 0 para sensor pH y 1 para controlador pH 
      OD_sens_or_cont    = bitRead(aux2_sp, 1);   //aux2_sp segund BIT 0 para sensor OD y 1 para controlador OD
      temp_sens_or_cont  = bitRead(aux2_sp, 2);   //aux2_sp tercer BIT 0 para sensor Temperatura y 1 para controlador Temperatura
      rpm_sens_or_cont   = bitRead(aux2_sp, 3);   //aux2_sp cuarto BIT 0 para sensor RPM y 1 para controlador RPM
      aux1_sens_or_cont  = bitRead(aux2_sp, 4);   //aux2_sp quinto BIT 0 para sensor auxiliar_1 y 1 para controlador auxiliar_1.  
      */
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

void set_point()
{
  if (sensor_ph_value <= ph_sp)
  {
    digitalWrite(led3,HIGH);
  }
  else if(sensor_ph_value >= (ph_sp + 0.2))  //histeresis de 0.2 ph hacia arriba
  {
    digitalWrite(led3,LOW);
  }
  
  if (controlador_ph_value <= ph_sp)
  {
    digitalWrite(led2,HIGH);
  }
  else if(controlador_ph_value >= (ph_sp + 0.2))  //histeresis de 0.2 ph hacia arriba
  {
    digitalWrite(led2,LOW);
  }
  
  if (sensor_ptc_value <= (temp_sp - 1.5))  // histeresis entre set point -0.3 y set point - 0.1.
  {
    digitalWrite(led,HIGH);
  }
  else if(sensor_ptc_value >= (temp_sp - 0.5)) // por retardo de proceso, se apaga antes el calentador
  {
    digitalWrite(led,LOW);
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
        sensor_ph_4_cal = sensor_ph_value; //lectura pH4 de bit a milivolts
        flag_ph1 = 1;
       // digitalWrite(led1,HIGH);
      }
      else if(aux1_cal == 2) //calibracion con pH7
      {
        sensor_ph_7_cal = sensor_ph_value; //lectura pH4 de bit a milivolts
        flag_ph2 = 1;
       // digitalWrite(led2,HIGH);
      }
      else if(aux1_cal == 3) //calibracion con pH10
      {
        sensor_ph_10_cal = sensor_ph_value; //lectura pH4 de bit a milivolts
        flag_ph3 = 1;
      }
      
      if((flag_ph1 == 1) && (flag_ph2 == 1))
      {
        paso_ph_cal = ((7 - 4) / (sensor_ph_7_cal - sensor_ph_4_cal)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph1 = 0;
        flag_ph2 = 0;
      }
      if((flag_ph2 == 1) && (flag_ph3 == 1))
      {
        paso_ph_cal = ((10 - 7) / (sensor_ph_10_cal - sensor_ph_7_cal)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph2 = 0;
        flag_ph3 = 0;
      }
      
      break;
    }
    case(2):  // Calibracion Oxigeno Disuelto
    { 
      break;
    }
    case(3):  // Calibracion Controlador pH 420 (1)
    { 
      if(aux1_cal == 4) //calibracion pH4
      {
        controlador_ph4_cal = controlador_ph_value; //lectura pH4 de bit a milivolts
        flag_ph1_cont = 1;
       // digitalWrite(led1,HIGH);
      }
      else if(aux1_cal == 5) //calibracion con pH7
      {
        controlador_ph7_cal = controlador_ph_value; //lectura pH4 de bit a milivolts
        flag_ph2_cont = 1;
        //digitalWrite(led2,HIGH);
      }
      else if(aux1_cal == 6) //calibracion con pH7
      {
        controlador_ph10_cal = controlador_ph_value; //lectura pH4 de bit a milivolts
        flag_ph3_cont = 1;
        //digitalWrite(led2,HIGH);
      }
      
      if((flag_ph1_cont == 1) && (flag_ph2_cont == 1))
      {
        paso_ph_cont = ((7 - 4) / (controlador_ph7_cal - controlador_ph4_cal)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph1_cont = 0;
        flag_ph2_cont = 0;
        //digitalWrite(led3,HIGH);
      }
      if((flag_ph2_cont == 1) && (flag_ph3_cont == 1))
      {
        paso_ph_cont = ((10 - 7) / (controlador_ph10_cal - controlador_ph7_cal)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph2_cont = 0;
        flag_ph3_cont = 0;
        //digitalWrite(led3,HIGH);
      }
      break;     
    }
    case(4):  // Calibracion Controlador auxiliar 420 (2)
    { 
      break;
    }
    case(5):  // Calibracion sensor_ptc
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
  switch(timer_lectura) //lectura de un sensor cada 0.1 (s), equivale a un tiempo de lectura = 0.6 (s) x sensor,  fs= (1/0.6)
  {
    case(1):
    {
      sens1_read = analogRead(sensor_ph); 
      procesar_datos(1); // 60 ms    
      break;
    }
    case(2):
    {
      sens2_read = analogRead(sensor_aux);
      procesar_datos(2);
      break;
    }
    case(3):
    {
      sens3_read = analogRead(controlador_ph);
      procesar_datos(3);
      break;
    }
    case(4):
    {
      sens4_read = analogRead(controlador_aux);               //Sensor 4-20
      procesar_datos(4);
      break;
    }
    case(5):
    {
      sens5_read = analogRead(sensor_ptc);                        //Sensor auxiliar 2
      procesar_datos(5);
      break;
    }
    case(6):
    {
      sens6_read = analogRead(adc_input);                    //Sensor auxiliar 3
      timer_lectura = 0;
      break;
    }
  default:timer_lectura = 0;break;
  }
}



void procesar_datos(byte canal)
{
  switch (canal)
  {
    case(1):  // CANAL DE sensor PH
    {
      for(n_sph = NCoef_sph; n_sph > 0; n_sph--) //Desplazo de la muestra mas antigua
      {
        x_sph[n_sph] = x_sph[n_sph - 1];
        y_sph[n_sph] = y_sph[n_sph - 1];
      }
      
      x_sph[0] = sens1_read;                    //nuevo valor input
      y_sph[0] = ACoef_sph[0] * x_sph[0];       //nuevo valor output    
      //Filtro IIR 
      for(n_sph = 1; n_sph <= NCoef_sph; n_sph++)
      {
        y_sph[0] += (ACoef_sph[n_sph] * x_sph[n_sph]) - (BCoef_sph[n_sph] * y_sph[n_sph]);
      }
   
      sensor_ph_value = y_sph[0] * (voltaje_ref_ADC / 1024);
      sensor_ph_value = (paso_ph_cal * sensor_ph_value) - (paso_ph_cal * sensor_ph_7_cal) + 7;    //voltaje a pH     
      /*shift the old samples
      for(int g = 100; g > 0; g--)   //a 0.06 seg por muestra el arreglo para 100 muestras se llena en 6 (s)//rellenar arreglo utilizando incrementador seconds (sugerencia no validada)
      {
         arreglo_ph[g] = arreglo_ph[g-1];
      }
      arreglo_ph[0] = sens1_read;
      
      for (int g = 0; g < 100; g++)
      {
          suma_ph = suma_ph + arreglo_ph[g];
      }
      promedio_ph = suma_ph / 100;
      suma_ph = 0;
      
      sensor_ph_value         = promedio_ph * (voltaje_ref_ADC / 1024);
      sensor_ph_value         = (paso_ph_cal * sensor_ph_value) - (paso_ph_cal * sensor_ph_7_cal) + 7;    //voltaje a pH                                                   // decimas a entero para enviar como (byte)
      */
      break;
    }
    case(2):
    {
    
      //shift the old samples
      /*for(int g = 100; g > 0; g--)   //a 0.06 seg por muestra el arreglo para 100 muestras se llena en 6 (s)//rellenar arreglo utilizando incrementador seconds (sugerencia no validada)
      {
         arreglo_temp[g] = arreglo_temp[g-1];
      }
      arreglo_temp[0] = sens2_read;
      
      for (int g = 0; g < 100; g++)
      {
          suma_temp = suma_temp + arreglo_temp[g];
      }
      promedio_temp = suma_temp / 100;
      suma_temp = 0; */
      break;
    }
    case(3):
    {
      for(n_con_ph = NCoef_con_ph; n_con_ph > 0; n_con_ph--) //Desplazo de la muestra mas antigua
      {
        x_con_ph[n_con_ph] = x_con_ph[n_con_ph - 1];
        y_con_ph[n_con_ph] = y_con_ph[n_con_ph - 1];
      }
      //Calculate the new output
      x_con_ph[0] = sens3_read;
      y_con_ph[0] = ACoef_con_ph[0] * x_con_ph[0];
       
      for(n_con_ph = 1; n_con_ph <= NCoef_con_ph; n_con_ph++)
      {
        y_con_ph[0] += (ACoef_con_ph[n_con_ph] * x_con_ph[n_con_ph]) - (BCoef_con_ph[n_con_ph] * y_con_ph[n_con_ph]);
      }
   
      controlador_ph_value    =  y_con_ph[0] * (voltaje_ref_ADC / 1024);              //controlador pH conectado en adc 4 que corresponde a controlador auxiliar
      controlador_ph_value    = (paso_ph_cont * controlador_ph_value) - (paso_ph_cont * controlador_ph7_cal) + 7;  
      break;
    }
    case(4):
    {

      //shift the old samples
     /* for(int g = 100; g > 0; g--)   //a 0.06 seg por muestra el arreglo para 100 muestras se llena en 6 (s)//rellenar arreglo utilizando incrementador para no entrar en for (sugerencia no validada)
      {
         arreglo_cont_aux[g] = arreglo_cont_aux[g-1];    //arreglo controlador auxiliar
      }
      arreglo_cont_aux[0] = sens4_read;
      
      for (int g = 0; g < 100; g++)
      {
          suma_cont_aux = suma_cont_aux + arreglo_cont_aux[g];
      }
      promedio_cont_aux       = suma_cont_aux / 100;
      suma_cont_aux           = 0;   
      controlador_ph_value    = promedio_cont_aux * (voltaje_ref_ADC / 1024);              //controlador pH conectado en adc 4 que corresponde a controlador auxiliar
      controlador_ph_value    = (paso_ph_cont * controlador_ph_value) - (paso_ph_cont * controlador_ph7_cal) + 7;  */
      break;
    }
    case(5):
    {
      for(n_temp = NCoef_temp; n_temp > 0; n_temp--) //Desplazo de la muestra mas antigua
      {
        x_temp[n_temp] = x_temp[n_temp - 1];
        y_temp[n_temp] = y_temp[n_temp - 1];
      }
      //Calculate the new output
      x_temp[0] = sens5_read;
      y_temp[0] = ACoef_temp[0] * x_temp[0];
       
      for(n_temp = 1; n_temp <= NCoef_temp; n_temp++)
      {
        y_temp[0] += (ACoef_temp[n_temp] * x_temp[n_temp]) - (BCoef_temp[n_temp] * y_temp[n_temp]);
      }  
      sensor_ptc_value = y_temp[0] * (voltaje_ref_ADC / 1024);
      break;
    }
    case(6):
    {
      break;
    }
    default:break;
  }
}
/*-------------------------------------------------------------------Reseteo de Variables--*/

void variables_reset()
{
    for (int h = 0; h < 5; h++)
  {
    x_sph[h] = 0; //Arreglo sensor pH para filtro IIR Butterworth
    y_sph[h] = 0;
    
    x_temp[h] = 0; //Arreglo sensor temperatura ptc para filtro IIR Butterworth
    y_temp[h] = 0;
    
    x_con_ph[h] = 0; //Arreglo controlador pH para filtro IIR Butterworth
    y_con_ph[h] = 0;
  }
    /*Variable inicio programa*/
  flag_inicio  = 0;
  estado_led   = 0;
  estado_led1  = 0;
  estado_led2  = 0;
  estado_led3  = 0;
  /*Variables  calibracion*/
  flag_ph1 = 0;
  flag_ph2 = 0;
  flag_ph3 = 0;
  flag_ph1_cont = 0;
  flag_ph2_cont = 0;
  flag_ph3_cont = 0;
  /*Variables timer*/
  timer_control = 0;          //timer de control de variables
  timer_lectura = 0;          //timer de lectura de datos
  timer_loop = 0;             //timer loop
  milisegundos = 0;
  segundos = 0;               //segundos incrementandose hasta el minuto segundos == 60
  minutos = 0;                //minutos incrementandose hasta la hora minutos == 60, para comparar con tiempo de ejecucion de setpoint.
 
  incrementador_tx  = 0;
  id_trama    = 0;
  estado_tx   = 0;
  contador    = 0;
  aux_timer1  = 0;
  seconds     = 0;
  /*Valor sensores*/
  sensor_ph_value = 0;       // decimas a entero para enviar como (byte)
  controlador_ph_value = 0;  
}


void estado_output()
{
  estado_led   = digitalRead(led);
  bitWrite(output_state, 0, estado_led); 
  estado_led1 =digitalRead(led1);
  bitWrite(output_state, 1, estado_led1);
  estado_led2 =digitalRead(led2);
  bitWrite(output_state, 2, estado_led2);
  estado_led3 =digitalRead(led3);
  bitWrite(output_state, 3, estado_led3); 
}

/*-------------------------------------------------------------------Comunicacion--*/
byte make_trama(byte a,byte b)
{
  sof_tx      = '#';
  id_trama_tx = a;                          // normal = 1, confirm = 2
  estado_tx   = b;                          // envia estado q se recibiò o incrementador para trama normal
  sens1_tx    = sensor_ph_value * 10;       // decimas a entero para enviar como (byte)
  sens2_tx    = 0;
  sens3_tx    = controlador_ph_value * 10;  // decimas a entero para enviar como (byte)
  sens4_tx    = 0;
  sens5_tx    = 0;
  aux_tx      = output_state;               //Estado de los relay (Encendido = 1 / Apagado = 0)
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
/*-------------------------------------------------------------------LOOP--*/
void loop()
{
  if(timer_loop == 10)  //cada 100 (ms)
  {
    if (Serial.available() > 0)
    {   
      if(read_trama())
      {
        deco_trama();
        estado_output(); //estado de la salidas digitales HIGH = 1 o LOW = 0  valores almacenados en variable output_state.
      }
      else
      {
        make_trama(6,0);   // id_trama = 3 hacia java = error de trama
        send_trama(); 
      }       
    }
  /*
  if(id_trama == 5)
  {
    make_trama(5,0);
    send_trama();
    flag_inicio = 0;
    actuadores_off();
    goto fin_programa;
  }
  
  if((id_trama == 4) || (flag_inicio == 1))
  {
    make_trama(4,0);
    send_trama();
    flag_inicio = 1;
  }
  else
  {
    goto fin_programa;
  }
  */
  switch(id_trama)   //  Ingreso a la funciones control manual set_manual() y calibracion set_calibracion(flag_cal)
  {
    case(1):
    {
      make_trama(1,0);   // id_trama = 1 trama de set point
      send_trama(); 
      break;
    } 
    case(2): //trama = manual
    {
      make_trama(2,0);   // id_trama = 2 trama manual
      send_trama(); 
      set_manual();
      id_trama = 0; 
      break;
    }
    case(3): //trama = calibraciòn
    {
      make_trama(3,0);   // id_trama = 3 hacia java = calibracion
      send_trama();  
      set_calibracion(flag_cal);
      id_trama = 0; 
      break;
    }
    case(4):
    {
      make_trama(4,0);
      send_trama();
      flag_inicio = 1;
      break;
    }
    case(5):
    {
      make_trama(5,0);
      send_trama();
      flag_inicio = 0;
      actuadores.off();
      //goto fin_programa;
      break;
    }
    default:break;
  }
  timer_loop = 0; 
} 
 
  //Serial.println(sens1_read);
  if(seconds == 100) //Cada 1 segundo 
  {
    incrementador_tx++;  // byte Retorna a cero despues de 255
    estado_output();     //estado de la salidas digitales HIGH = 1 o LOW = 0  valores almacenados en variable output_state.
    make_trama(0,incrementador_tx);  // 0 trama normal hacia java cada 1 segundo, con incrementador_tx++
    send_trama();
    seconds = 0;
    
    /*---------------------------------------------------------------------Control ON/OFF de variables - Trama SETPOINT-----------------*/
    
    // Si id_trama(1) = control  && (1seg / 0.01 seg) = 100, condicion if cada 1 seg.
    /*-Solo se tendra estado manual o automatico, no se podra tener una variable con control automatico y otra con control manual*/
    
    if(id_trama == 1)   
    { 
      switch(aux2_sp)
      {
        case(1):   //Set_point Esterelizacion con tiempo limite
        {
          if(aux1_sp > minutos)
          {
            set_point();
          }
          else if(aux1_sp <= minutos)
          {
            minutos = aux1_sp;
            actuadores.off();
          }         
          break;
        }
        case(2):  //Set_point Enfriamiento con tiempo limite
        {
          if(aux1_sp > minutos)
          {
            set_point();
          } 
          else if(aux1_sp <= minutos)
          {
            minutos = aux1_sp;
            actuadores.off();
          }  
          break;
        }
        case(3):    //Set_point normal
        {
          set_point();
          break;
        }default:break;
      } 
    } 
  }
  
  if(timer_muestreo == 10)  // Tiempo de muestreo = 10 (ms) * 10=100 (ms), por cada uno de los sensores, lo que equivale a un tiempo de muestreo de 600 (ms) = (fs = (1/0.6))
  {
    timer_lectura++;        //timer de lectura de sensores aumenta cada 100 (ms)
    lectura_sensores();     // se llama procesar_datos() desde lectura_sensores() 
    timer_muestreo = 0;
  }
  
  if(milisegundos == 100)  // milisegundos == 100 * 10 (ms) = 1 seg
  {
    milisegundos = 0;
    segundos++;
  } 
  if(segundos == 60)      // segundos == 60 = 1 minuto
  {
    segundos = 0;
    minutos++;
  }
  
  fin_programa:;
  
    /*- Periodo de ejecucion funciones -*/
  if(aux_timer1 == 1) // cada 10 (ms) 
  { 
    seconds++;             //aumenta cada 10 (ms)
    milisegundos++;        //aumenta cada 10 (ms)  
    timer_muestreo++;
    timer_loop++;    
    aux_timer1 = 0;       
  }

} //<- final loop

  



