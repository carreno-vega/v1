#include <avr/io.h>
#include <avr/interrupt.h>
#include <act_off.h>
#include <manual_control.h>
#include <EEPROM.h>


//EEPROM
int aux_eeprom;
byte ok_calibration;
//Prueba filtro
float actual_ph,anterior_ph;
float actual_temp,anterior_temp;
float sensor_ph_sin_filtro;
float NewSamp;
float value_fst_sph;

//EEPROM datalogger
int j;
int contador_eeprom;
int eeprom_tasa;
int sector_init;
//float NewSample;
// comentario prueba
// comentario 2
/*Instancia Clase de librerias*/
act actuadores; //clase actuadores de act_off
control control; //clase control
/*Varibles de relé*/
int  actuador_D8  = 8;   //Relé 1 D8  en arduino_xbee
int  actuador_D9  = 9;   //Relé 2 D9  en arduino_xbee
int  actuador_D10 = 10;  //Relé 3 D10 en arduino_xbee
int  actuador_D11 = 11;  //Relé 4 D11 en arduino_xbee

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
float  arreglo[19];
int  contador;
/*Identificadores de trama*/
byte  id_trama;
byte  estado_rx;
byte flag_setpoint;
/*Variables Set point*/
byte  sens0_h_sp;  //ph_sp ph set point para sensor y actuador
byte  sens0_l_sp;  
byte  sens1_h_sp;  
byte  sens1_l_sp;
byte  sens2_h_sp;  //ph_sp ph set point para sensor y actuador
byte  sens2_l_sp;  
byte  sens3_h_sp;  
byte  sens3_l_sp;
byte  sens4_h_sp;  
byte  sens4_l_sp;
byte  sens5_h_sp;  
byte  sens5_l_sp;  
byte  aux_sp; 
byte  aux1_sp;
byte  aux2_sp;

/* Variables Control Manual*/
byte  inst1_man;
byte  inst2_man;  
byte  inst3_man;  
byte  inst4_man; 
byte  inst5_man;  
byte  inst6_man;
byte  inst7_man;  
byte  inst8_man;  
byte  inst9_man;
byte  inst10_man; 
byte  inst11_man; 
byte  inst12_man;
byte  aux1_man;
byte  aux2_man;  
byte  aux3_man;

/* Variables de calibracion*/
byte flag_cal;  //Indica el sensor a calibrar
byte val_cal1;  
byte val_cal2;  
byte val_cal3;  
byte val_cal4;  
byte val_cal5;
byte val_cal6;  
byte val_cal7;  
byte val_cal8; 
byte val_cal9;  
byte val_cal_10;  
byte val_cal_11; 
byte aux1_cal;  //Para ph, aux1_cal=1 -> pH=4, aux1_cal=2 ->pH=7
byte aux2_cal;  
byte aux3_cal; 

/*Variables de Configuración*/
byte sens0_act_noa;
byte sens1_act_noa;
byte sens2_act_noa;
byte sens3_act_noa;
byte sens4_act_noa;
byte sens5_act_noa;
byte actuador_h_or_l;
byte aux2_config;
byte aux3_config;
byte aux4_config;
byte aux5_config;
byte aux6_config;
byte aux7_config;
byte aux8_config;
byte aux9_config;

/* Variables funcion asignacciond e actuadores sensor_actuador() */
byte sens0_act_asi;   //sensor A0 - actuador - no  asignado
byte sens1_act_asi;
byte sens2_act_asi; 
byte sens3_act_asi;
byte sens4_act_asi;
byte sens5_act_asi;

/*Actuador disminuye variable proceso = 0 Actuador aumenta variable proceso = 1 , si sens_act_asi = 0 no hay actuador asignado*/
byte sens0_type_act;
byte sens1_type_act; 
byte sens2_type_act; 
byte sens3_type_act; 
byte sens4_type_act; 
byte sens5_type_act;

/*Variables enviadas*/
byte  sof_tx;
byte  id_trama_tx;                // normal = 1, confirm = 2
byte  estado_tx;               // envia estado q se recibiò

byte sens0_tx_h;
byte sens0_tx_l;
byte sens1_tx_h;
byte sens1_tx_l;
byte sens2_tx_h;
byte sens2_tx_l;
byte sens3_tx_h;
byte sens3_tx_l;
byte sens4_tx_h;
byte sens4_tx_l;
byte sens5_tx_h;
byte sens5_tx_l;
byte aux_tx;
byte aux_tx1;
byte aux_tx2;
byte eof_tx;
/*
byte  sens1_tx;
byte  sens2_tx;
byte  sens3_tx;
byte  sens4_tx;
byte  sens5_tx;
byte  aux_tx;
byte  eof_tx;
*/
/*Declaracion entradas analogas para sensores*/

byte ADC0        = A0;   //Lectura ADC A0
byte ADC1        = A1;   //Lectura sensor auxiliar
byte ADC2        = A2;   //Lectura sensor PH   (Ver en que entrada analoga se conecta el sensor A1 o A2)
byte ADC3        = A3;   //Lectura sensor temperatura PTC en entrada analoga A3
byte ADC4        = A4;   //Lectura controlador pH       4-20 mA a 1- 5   Volts en entrada analoga A4
byte ADC5        = A5;   //Lectura controlador auxiliar 4-20 mA a 1- 5   Volts en entrada analoga A5

float voltaje_ref_ADC;  
/**/
float sens0_read;
float sens1_read;
float sens2_read;
float sens3_read;
float sens4_read;
float sens5_read;

float sens0_actual;
float sens1_actual;
float sens2_actual;
float sens3_actual;
float sens4_actual;
float sens5_actual;

float sens0_anterior;
float sens1_anterior;
float sens2_anterior;
float sens3_anterior;
float sens4_anterior;
float sens5_anterior;

int sens0_pros;
int sens1_pros;
int sens2_pros;
int sens3_pros;
int sens4_pros;
int sens5_pros;

/*Valor de setpoint para aplicar el respectivo control a la variable*/
int sens0_sp;
int sens1_sp;
int sens2_sp;
int sens3_sp;
int sens4_sp;
int sens5_sp;

/*Variables control automaticon ON/OFF*/
byte sens0_time_control;
byte sens0_time_evaluation;
byte sens0_time_c_y_e;
byte sens0_incrementador_c_y_e;
byte sens1_time_control;
byte sens1_time_evaluation;
byte sens1_time_c_y_e;
byte sens1_incrementador_c_y_e;
byte sens2_time_control;
byte sens2_time_evaluation;
byte sens2_time_c_y_e;
byte sens2_incrementador_c_y_e;
byte sens3_time_control;
byte sens3_time_evaluation;
byte sens3_time_c_y_e;
byte sens3_incrementador_c_y_e;
byte sens4_time_control;
byte sens4_time_evaluation;
byte sens4_time_c_y_e;
byte sens4_incrementador_c_y_e;
byte sens5_time_control;
byte sens5_time_evaluation;
byte sens5_time_c_y_e;
byte sens5_incrementador_c_y_e;

/*Variable estado salidas digitales*/
byte estado_led;
byte estado_led1;
byte estado_led2;
byte estado_led3;
byte output_state;
/*Variables propias del sensor pH*/
float sensor_ph_value;
float sensor_ph_value_float;
float sensor_ph_value_volt;
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
float sensor_temp_value;
float sensor_temp_value_volt;
float sensor_temp_value_float;
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


void setup()
{
  //eeprom
  ok_calibration = 0;
  anterior_ph = actual_ph = 0;
  //eeprom datalogger
  sector_init = 10;     //sector inicial de escritura en eeprom
  contador_eeprom = 0;  // incrementador eeprom
  eeprom_tasa = 2;      //tasa de escritura en segundos
  aux_eeprom = 0;
  
  //PRueba sin filtro
  sensor_ph_sin_filtro = 0;
  
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
  pinMode(actuador_D8, OUTPUT);
  pinMode(actuador_D9, OUTPUT);
  pinMode(actuador_D10, OUTPUT);
  pinMode(actuador_D11, OUTPUT);    
  digitalWrite(actuador_D8,LOW);  
  digitalWrite(actuador_D9,LOW);
  digitalWrite(actuador_D10,LOW);
  digitalWrite(actuador_D11,LOW);
  
  
int  actuador_D8  = 8;   //Relé 1 D8  en arduino_xbee
int  actuador_D9  = 9;   //Relé 2 D9  en arduino_xbee
int  actuador_D10 = 10;  //Relé 3 D10 en arduino_xbee
int  actuador_D11 = 11;  //Relé 4 D11 en arduino_xbee
  
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
  paso_ph_cal = 4.1899;  //paso inicial referencial, luego en funcion calibracion se toma el valor real del paso de pH 
  sensor_ph_7_cal = 2.3; //Valor inicial referencial para el voltaje en ADC para el sensor de pH igual a 7
  /*
  50mv/pH referencia
  2.3 V offset del sumador

  ph7 = 2.327, pH4 = 3.043 refencial
  paso inicial = (7-4)/(2.327-3.043)
  paso =  pH/volt
*/
/*Valor inicial variables filtro*/
  sens0_actual = 0;
  sens1_actual = 0;
  sens2_actual = 0;
  sens3_actual = 0;
  sens4_actual = 0;
  sens5_actual = 0;
  
  sens0_anterior = 0;
  sens1_anterior = 490;
  sens2_anterior = 490;
  sens3_anterior = 0;
  sens4_anterior = 0;
  sens5_anterior = 0;
  /*valor de SETPOINT*/
  sens0_sp = 0;
  sens1_sp = 0;
  sens2_sp = 0;
  sens3_sp = 0;
  sens4_sp = 0;
  sens5_sp = 0;
  
    /*Valores configuración*/
  sens0_act_noa  = 0;   //sensor A0 - actuador - no  asignado
  sens1_act_noa  = 0;
  sens2_act_noa  = 0; 
  sens3_act_noa  = 0;
  sens4_act_noa  = 0;
  sens5_act_noa  = 0;
  actuador_h_or_l = 0;
  aux2_config    = 0;
  aux3_config    = 0;
  aux4_config    = 0;
  aux5_config    = 0;
  aux6_config    = 0;
  aux7_config    = 0;
  aux8_config    = 0;
  aux9_config    = 0;  

/*Actuador disminuye variable proceso = 0 Actuador aumenta variable proceso = 1 , si sens_act_asi = 0 no hay actuador asignado*/
sens0_type_act   = 0;
sens1_type_act   = 0; 
sens2_type_act   = 0; 
sens3_type_act   = 0; 
sens4_type_act   = 0; 
sens5_type_act   = 0;
  //Funcion asignación de actuador al respectivo sensor
  sens0_act_asi  = 0;   //sensor A0 - actuador - no  asignado
  sens1_act_asi  = 0;
  sens2_act_asi  = 0; 
  sens3_act_asi  = 0;
  sens4_act_asi  = 0;
  sens5_act_asi  = 0;
  
  
  
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
  flag_setpoint = 0;
    
  sens0_tx_h = 0;
  sens0_tx_l = 0;
  sens1_tx_h = 0;
  sens1_tx_l = 0;
  sens2_tx_h = 0;
  sens2_tx_l = 0;
  sens3_tx_h = 0;
  sens3_tx_l = 0;
  sens4_tx_h = 0;
  sens4_tx_l = 0;
  sens5_tx_h = 0;
  sens5_tx_l = 0;
  aux_tx     = 0;
  aux_tx1    = 0;
  aux_tx2    = 0;
  
  /*Variables setpoint*/
  sens0_time_control        = 2;  //tiempo en segundo de accionamiento del actuador
  sens0_time_evaluation     = 4;  //tiempo de evaluacion de la accion del actuador sobre el valor de proceso
  sens0_time_c_y_e          = sens0_time_control + sens0_time_evaluation;
  sens0_incrementador_c_y_e = 0;
  sens1_time_control        = 2;
  sens1_time_evaluation     = 4;
  sens1_time_c_y_e          = sens1_time_control + sens1_time_evaluation;
  sens1_incrementador_c_y_e = 0;
  sens2_time_control        = 2;
  sens2_time_evaluation     = 4;
  sens2_time_c_y_e          = sens2_time_control + sens2_time_evaluation;
  sens2_incrementador_c_y_e = 0;
  sens3_time_control        = 2;
  sens3_time_evaluation     = 4;
  sens3_time_c_y_e          = sens3_time_control + sens3_time_evaluation;
  sens3_incrementador_c_y_e = 0;
  sens4_time_control        = 2;
  sens4_time_evaluation     = 4;
  sens4_time_c_y_e          = sens4_time_control + sens4_time_evaluation;
  sens4_incrementador_c_y_e = 0;
  sens5_time_control        = 2;
  sens5_time_evaluation     = 4;
  sens5_time_c_y_e          = sens5_time_control + sens5_time_evaluation;
  sens5_incrementador_c_y_e = 0;

  /*incrementadores*/
  incrementador_tx  = 0;
  id_trama    = 0;
  estado_tx   = 0;
  contador    = 0;
  aux_timer1  = 0;
  seconds     = 0;
  
  
  /*sens1_tx    = 0;
  sens2_tx    = 0;
  sens3_tx    = 0;
  sens4_tx    = 0;
  sens5_tx    = 0;
  */
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
3	Trama de calibraciòn
4       Trama de inicio de programa
5       Trama de apagado de actuadores
6       Trama de error recepción de datos
7       Trama de lectura valores de calibración EEPROM
8       Trama de configuración
9       Trama de reinicio valor de variables
*/
  id_trama    = arreglo[1];
  estado_rx   = arreglo[2];   // Incrementador
  switch(id_trama)
  {
    case(1): //trama = setpoint
    { 
      flag_setpoint = arreglo[1];   //set point en ejecucion
      
      sens0_h_sp    = arreglo[3];  //ph_sp ph set point para sensor y actuador
      sens0_l_sp    = arreglo[4];  
      sens1_h_sp    = arreglo[5];  
      sens1_l_sp    = arreglo[6];
      sens2_h_sp    = arreglo[7];  //ph_sp ph set point para sensor y actuador
      sens2_l_sp    = arreglo[8];  
      sens3_h_sp    = arreglo[9];  
      sens3_l_sp    = arreglo[10];
      sens4_h_sp    = arreglo[11];  
      sens4_l_sp    = arreglo[12];
      sens5_h_sp    = arreglo[13];  
      sens5_l_sp    = arreglo[14];  
      aux_sp        = arreglo[15];
      aux1_sp       = arreglo[16];    
      aux2_sp       = arreglo[17];     
      
      //Union de byte alto con byte bajo para valor de set point  de 0 a 1023 
      sens0_sp = (sens0_h_sp << 8) + sens0_l_sp;
      sens1_sp = (sens1_h_sp << 8) + sens1_l_sp;
      sens2_sp = (sens2_h_sp << 8) + sens2_l_sp;
      sens3_sp = (sens3_h_sp << 8) + sens3_l_sp;
      sens4_sp = (sens4_h_sp << 8) + sens4_l_sp;
      sens5_sp = (sens5_h_sp << 8) + sens5_l_sp;
   
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
      inst4_man     = arreglo[6];  
      inst5_man     = arreglo[7];  
      inst6_man     = arreglo[8];
      inst7_man     = arreglo[9];  
      inst8_man     = arreglo[10];  
      inst9_man     = arreglo[11];
      inst10_man    = arreglo[12];  
      inst11_man    = arreglo[13];  
      inst12_man    = arreglo[14];
      aux1_man      = arreglo[15];  
      aux2_man      = arreglo[16];  
      aux3_man      = arreglo[17];
      break;
    }
    case(3):  // trama = calibracion
    { 
      flag_cal   = arreglo[3];  //Indica el sensor a calibrar
      val_cal1   = arreglo[4];  
      val_cal2   = arreglo[5];  
      val_cal3   = arreglo[6];  
      val_cal4   = arreglo[7];  
      val_cal5   = arreglo[8];
      val_cal6   = arreglo[9];  
      val_cal7   = arreglo[10];  
      val_cal8   = arreglo[11]; 
      val_cal9   = arreglo[12];  
      val_cal_10 = arreglo[13];  
      val_cal_11 = arreglo[14]; 
      aux1_cal   = arreglo[15];  //Para ph, aux1_cal=1 -> pH=4, aux1_cal=2 ->pH=7
      aux2_cal   = arreglo[16];  
      aux3_cal   = arreglo[17]; 
      break;
    }
    case(8):  //trama=configuración actuadores
    {
      sens0_act_noa  = arreglo[3];    //ADC el numero de la entrada (1 a 6) donde esta conectado el actuador
      sens1_act_noa  = arreglo[4];    
      sens2_act_noa  = arreglo[5]; 
      sens3_act_noa  = arreglo[6];
      sens4_act_noa  = arreglo[7];
      sens5_act_noa  = arreglo[8];

      actuador_h_or_l   = arreglo[9];  //Actuador disminuye variable proceso = 0 Actuador aumenta variable proceso = 1 , si sens_act_asi = 0 no hay actuador asignado  
      aux2_config       = arreglo[10]; //sensor del 0 al 5 con dos actuadores /actuador 1 y actuador 2 (Digital output 8 y 9 )
      aux3_config       = arreglo[11];
      aux4_config       = arreglo[12];
      aux5_config       = arreglo[13];
      aux6_config       = arreglo[14];
      aux7_config       = arreglo[15];
      aux8_config       = arreglo[16];
      aux9_config       = arreglo[17];
      
      sens0_type_act   = bitRead(actuador_h_or_l, 0); //Actuador disminuye variable proceso = 0 Actuador aumenta variable proceso = 1 , si sens_act_asi = 0 no hay actuador asignado
      sens1_type_act   = bitRead(actuador_h_or_l, 1); //Actuador disminuye variable proceso = 0 Actuador aumenta variable proceso = 1 , si sens_act_asi = 0 no hay actuador asignado
      sens2_type_act   = bitRead(actuador_h_or_l, 2); //Actuador disminuye variable proceso = 0 Actuador aumenta variable proceso = 1 , si sens_act_asi = 0 no hay actuador asignado
      sens3_type_act   = bitRead(actuador_h_or_l, 3); //Actuador disminuye variable proceso = 0 Actuador aumenta variable proceso = 1 , si sens_act_asi = 0 no hay actuador asignado
      sens4_type_act   = bitRead(actuador_h_or_l, 4); //Actuador disminuye variable proceso = 0 Actuador aumenta variable proceso = 1 , si sens_act_asi = 0 no hay actuador asignado
      sens5_type_act   = bitRead(actuador_h_or_l, 5); //Actuador disminuye variable proceso = 0 Actuador aumenta variable proceso = 1 , si sens_act_asi = 0 no hay actuador asignado
      break;
    }
    case(14):
    {
      eeprom_tasa  = arreglo[3];  //tiempo en minutos almacenamiento en datalogger
      break;
    }
    default:break;  
  }
}

void sensor_actuador() 
{      
  //Si sens_act_noa = 0, no se le asigna ningun actuador, si ya tiene un actuador asignado este vuelve a no tener asignación.
  if(sens0_act_noa > 0)
  {
    sens0_act_asi = sens0_act_noa + 7; 
  }
  else if(sens0_act_noa = 0)
  {
    sens0_act_asi = 0;
  }
  
  if(sens1_act_noa > 0)
  {
    sens1_act_asi  = sens1_act_noa + 7;
  }
  else if(sens1_act_noa = 0)
  {
    sens1_act_asi = 0;
  }
  
  if(sens2_act_noa > 0)
  {
    sens2_act_asi  = sens2_act_noa + 7;
  }
  else if(sens2_act_noa = 0)
  {
    sens2_act_asi = 0;
  }
  
  if(sens3_act_noa > 0)
  {
    sens3_act_asi  = sens3_act_noa + 7;
  }
  else if(sens3_act_noa = 0)
  {
    sens3_act_asi = 0;
  }
  
  if(sens4_act_noa > 0)
  {
    sens4_act_asi = sens4_act_noa  + 7;
  }
  else if(sens4_act_noa = 0)
  {
    sens4_act_asi = 0;
  }
  
  if(sens5_act_noa  > 0)
  {
    sens5_act_noa  = sens5_act_noa + 7;
  }
  else if(sens5_act_noa = 0)
  {
    sens5_act_asi = 0;
  }
}
//Lectura ADC A0
//A1 Lectura sensor auxiliar
//A2 Lectura sensor PH   (Ver en que entrada analoga se conecta el sensor A1 o A2)
//A3 Lectura sensor temperatura PTC en entrada analoga A3
//A4 Lectura controlador pH       4-20 mA a 1- 5   Volts en entrada analoga A4
//A5 Lectura controlador auxiliar 4-20 mA a 1- 5   Volts en entrada analoga A5


void set_point()
{
  
  if(sens0_act_asi =! 0)  //al sensor le ha sido asigando un salida digital ( 8 - 9 o 10 )
  //El sensor 0 Tiene un actuador asignado a  la variable
  {
    if(sens0_type_act = 0) //logica positiva
    {
      if(sens0_pros >= sens0_sp)
      {
        digitalWrite(sens0_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens0_act_asi,LOW);
      }  
    }
    else if(sens0_type_act = 1)  //logica negativa
    {
      if(sens0_pros <= sens0_sp)
      {
        digitalWrite(sens0_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens0_act_asi,LOW);
      }  
    }
  } 

  if((sens1_act_asi =! 0) && (aux2_config =! 1))
  {
    if(sens1_type_act = 0)
    {
      if(sens1_pros >= sens1_sp)
      {
        digitalWrite(sens1_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens1_act_asi,LOW);
      }  
    }
    else if(sens1_type_act = 1)
    {
      if(sens1_pros <= sens1_sp)
      {
        digitalWrite(sens1_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens1_act_asi,LOW);
      }  
    }    
  }
  else if((sens1_act_asi =! 0) && (aux2_config == 1))
  //Tiene dos controladores asignados a  la variable
  {
    
      if(sens1_pros >= sens1_sp)
      {
        digitalWrite(sens1_act_asi,HIGH);
        digitalWrite(sens1_act_asi+1,LOW);
      }
      else
      {
        digitalWrite(sens1_act_asi,LOW);
        digitalWrite(sens1_act_asi+1,HIGH);
      }  
  }  
  if((sens2_act_asi =! 0) && (aux2_config =! 2))
  //Tiene un controlador asignado a  la variable
  {
    if(sens2_type_act = 0)
    {
      if(sens2_pros >= sens2_sp)
      {
        digitalWrite(sens2_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens2_act_asi,LOW);
      }  
    }
    else if(sens2_type_act = 1)
    {
      if(sens2_pros <= sens2_sp)
      {
        digitalWrite(sens2_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens2_act_asi,LOW);
      }  
    }
  }
   else if((sens1_act_asi =! 0) && (aux2_config == 2))
  //Tiene un controlador asignado a  la variable
  {
    if(sens1_pros >= sens1_sp)
    {
      digitalWrite(sens1_act_asi,HIGH);
      digitalWrite(sens1_act_asi+1,LOW);
    }
    else
    {
      digitalWrite(sens1_act_asi,LOW);
      digitalWrite(sens1_act_asi+1,HIGH);
    }  
  }
  
  if(sens3_act_asi =! 0)
  //Tiene un controlador asignado a  la variable
  {
    if(sens3_type_act = 0)
    {
      if(sens3_pros >= sens3_sp)
      {
        digitalWrite(sens3_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens3_act_asi,LOW);
      }  
    }
    else if(sens3_type_act = 1)
    {
      if(sens3_pros <= sens3_sp)
      {
        digitalWrite(sens3_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens3_act_asi,LOW);
      }  
    }
  }

  if(sens4_act_asi =! 0)
  //Tiene un controlador asignado a  la variable
  {
    if(sens4_type_act = 0)
    {
      if(sens4_pros >= sens4_sp)
      {
        digitalWrite(sens4_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens4_act_asi,LOW);
      }  
    }
    else if(sens4_type_act = 1)
    {
      if(sens4_pros <= sens4_sp)
      {
        digitalWrite(sens4_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens4_act_asi,LOW);
      }  
    }
  }

  if(sens5_act_asi =! 0)
  //Tiene un controlador asignado a  la variable
  {
    if(sens5_type_act = 0)
    {
      if(sens5_pros >= sens5_sp)
      {
        digitalWrite(sens5_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens5_act_asi,LOW);
      }  
    }
    else if(sens5_type_act = 1)
    {
      if(sens5_pros <= sens5_sp)
      {
        digitalWrite(sens5_act_asi,HIGH);
      }
      else
      {
        digitalWrite(sens5_act_asi,LOW);
      }  
    }
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
        sensor_ph_4_cal = sensor_ph_value_volt; //lectura pH4 de bit a milivolts     
        int ph4_dec = actual_ph; //0 a 1023
        byte ph4_dec_lsb = 0xFF & ph4_dec;
        byte ph4_dec_msb = 0xFF & (ph4_dec >> 8);
        EEPROM.write(12,ph4_dec_lsb);
        EEPROM.write(14,ph4_dec_msb);
        // END SET CAL VAL ON EEMPROM
        flag_ph1 = 1;
       // digitalWrite(led1,HIGH);
      }
      else if(aux1_cal == 2) //calibracion con pH7
      {
        sensor_ph_7_cal = sensor_ph_value_volt; //lectura pH4 de bit a milivolts
        // SET CAL VAL ON EEMPROM
        int ph7_dec = actual_ph; //0 a 1023
        byte ph7_dec_lsb = 0xFF & ph7_dec;
        byte ph7_dec_msb = 0xFF & (ph7_dec >> 8);
        EEPROM.write(16,ph7_dec_lsb);
        EEPROM.write(18,ph7_dec_msb);
        flag_ph2 = 1;
       // digitalWrite(led2,HIGH);
      }
      
      if((flag_ph1 == 1) && (flag_ph2 == 1))
      {
        paso_ph_cal = ((7 - 4) / (sensor_ph_7_cal - sensor_ph_4_cal)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph1 = 0;
        flag_ph2 = 0;
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
      
      if((flag_ph1_cont == 1) && (flag_ph2_cont == 1))
      {
        paso_ph_cont = ((7 - 4) / (controlador_ph7_cal - controlador_ph4_cal)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph1_cont = 0;
        flag_ph2_cont = 0;
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
  /*  ADC0 = ADC auxiliar
      ADC1 Y ADC2 = BNC
      ADC3 = PT100
      ADC4 Y ADC5 = 4_20 mA
  */    
     
  switch(timer_lectura) //lectura de un sensor cada 0.1 (s), equivale a un tiempo de lectura = 0.6 (s) x sensor,  fs= (1/0.6)
  {
    case(1):
    {
      sens0_read = analogRead(ADC0); 
      sens0_pros = procesar_datos_sens0(sens0_read); // 60 ms    
      break;
    }
    case(2):
    {
      sens1_read = analogRead(ADC1);
      sens1_pros = procesar_datos_sens1(sens1_read); // 60 ms 
    //  procesar_datos(2);
      break;
    }
    case(3):
    {
      sens2_read = analogRead(ADC2);
      sens2_pros = procesar_datos_sens2(sens2_read); // 60 ms 
     // procesar_datos(3);
      break;
    }
    case(4):
    {
      sens3_read = analogRead(ADC3);               //Sensor 4-20
      sens3_pros = procesar_datos_sens3(sens3_read); // 60 ms 
     // procesar_datos(4);
      break;
    }
    case(5):
    {
      sens4_read = analogRead(ADC4);                //Sensor auxiliar 2
      sens4_pros = procesar_datos_sens4(sens4_read);
    //  procesar_datos(5);
      break;
    }
    case(6):
    {
      sens5_read = analogRead(ADC5);                    //Sensor auxiliar 3
      sens5_pros = procesar_datos_sens5(sens5_read);
      timer_lectura = 0;
      break;
    }
  default:timer_lectura = 0;break;
  }
}

float procesar_datos_sens0(float sens0_in)
{
  sens0_actual = 0.01 * sens0_in + sens0_anterior * 0.99;
  sens0_anterior = sens0_actual;  
  return sens0_actual; 
}

float procesar_datos_sens1(float sens1_in)
{
  sens1_actual = 0.05 * sens1_in + sens1_anterior * 0.95;
  sens1_anterior = sens1_actual;  
  return sens1_actual; 
}

float procesar_datos_sens2(float sens2_in)
{
  sens2_actual = 0.05 * sens2_in + sens2_anterior * 0.95;
  sens2_anterior = sens2_actual;  
  return sens2_actual; 
}

float procesar_datos_sens3(float sens3_in)
{
  sens3_actual = 0.01 * sens3_in + sens3_anterior * 0.99;
  sens3_anterior = sens3_actual;  
  return sens3_actual; 
}

float procesar_datos_sens4(float sens4_in)
{
  sens4_actual = 0.01 * sens4_in + sens4_anterior * 0.99;
  sens4_anterior = sens4_actual;  
  return sens4_actual; 
}

float procesar_datos_sens5(float sens5_in)
{
  sens5_actual = 0.01 * sens5_in + sens5_anterior * 0.99;
  sens5_anterior = sens5_actual;  
  return sens5_actual; 
}



float procesar_datos(byte canal, float NewSample)
{
  float output_ph = 0;
  switch (canal)
  {
    case(1):  // CANAL DE sensor PH
    {
      
      actual_ph = 0.1 * NewSample + anterior_ph * 0.9;

      NewSamp = NewSample;      
      sensor_ph_value_volt  =  actual_ph * (voltaje_ref_ADC / 1023);
      sensor_ph_value_float = (paso_ph_cal * sensor_ph_value_volt) - (paso_ph_cal * sensor_ph_7_cal) + 7;    //voltaje a pH                                                   // decimas a entero para enviar como (byte)
      output_ph = sensor_ph_value_float;
      
      sensor_ph_sin_filtro = (NewSamp) * (voltaje_ref_ADC / 1023);
      sensor_ph_sin_filtro  = (paso_ph_cal * sensor_ph_sin_filtro) - (paso_ph_cal * sensor_ph_7_cal) + 7;    //voltaje a pH     
     
      anterior_ph = actual_ph;
      return output_ph; 
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
   
      controlador_ph_value    =  y_con_ph[0] * (voltaje_ref_ADC / 1023);              //controlador pH conectado en adc 4 que corresponde a controlador auxiliar
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
    case(5):   //sensor de temperatura PT100
    {
      actual_temp = 0.1 * NewSample + anterior_temp * 0.9;
      
      sensor_temp_value_volt  =  actual_temp * (voltaje_ref_ADC / 1023);
      sensor_temp_value_float = 2,6012 * ( -15,559 * sensor_temp_value_volt + 150,41) - 269,39;
      
      /*for(n_temp = NCoef_temp; n_temp > 0; n_temp--) //Desplazo de la muestra mas antigua
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
      */
      anterior_temp = actual_temp;
      return sensor_temp_value_float;
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
  /*Variable lectura de sensores iniciales*/
  sens0_h_sp  = 0;  //ph_sp ph set point para sensor y actuador
  sens0_l_sp  = 0;  
  sens1_h_sp  = 0;  
  sens1_l_sp  = 0;
  sens2_h_sp  = 0;  //ph_sp ph set point para sensor y actuador
  sens2_l_sp  = 0;  
  sens3_h_sp  = 0;  
  sens3_l_sp  = 0;
  sens4_h_sp  = 0;  
  sens4_l_sp  = 0;
  sens5_h_sp  = 0;  
  sens5_l_sp  = 0;  
  aux_sp      = 0;
  aux1_sp     = 0;    
  aux2_sp     = 0; 

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
  /*Valor inicial variables filtro*/
  sens0_actual = 0;
  sens1_actual = 0;
  sens2_actual = 0;
  sens3_actual = 0;
  sens4_actual = 0;
  sens5_actual = 0;
  
  sens0_anterior = 0;
  sens1_anterior = 490;
  sens2_anterior = 490;
  sens3_anterior = 0;
  sens4_anterior = 0;
  sens5_anterior = 0;
  /*valor de SETPOINT*/
  sens0_sp = 0;
  sens1_sp = 0;
  sens2_sp = 0;
  sens3_sp = 0;
  sens4_sp = 0;
  sens5_sp = 0;
  
    /*Valores configuración*/
  sens0_act_noa    = 0;   //sensor A0 - actuador - no  asignado
  sens1_act_noa    = 0;
  sens2_act_noa    = 0; 
  sens3_act_noa    = 0;
  sens4_act_noa    = 0;
  sens5_act_noa    = 0;
  actuador_h_or_l  = 0;
  aux2_config      = 0;
  aux3_config      = 0;
  aux4_config      = 0;
  aux5_config      = 0;
  aux6_config      = 0;
  aux7_config      = 0;
  aux8_config      = 0;
  aux9_config      = 0;  
}


void estado_output()  //Estado de los Rele - salidas digitales
{
  estado_led   = digitalRead(actuador_D8);
  bitWrite(output_state, 0, estado_led); 
  estado_led1 =digitalRead(actuador_D9);
  bitWrite(output_state, 1, estado_led1);
  estado_led2 =digitalRead(actuador_D10);
  bitWrite(output_state, 2, estado_led2);
  estado_led3 =digitalRead(actuador_D11);
  bitWrite(output_state, 3, estado_led3); 
}


/*-------------------------------------------------------------------Comunicacion--*/
byte make_trama(byte a,byte b)
{
  sof_tx      = '#';
  id_trama_tx = a;                          // normal = 1, confirm = 2
  estado_tx   = b;                          // envia estado q se recibiò o incrementador para trama normal
        
  sens0_tx_h    = 0xFF & (sens0_pros >> 8);               //
  sens0_tx_l    = 0xFF & sens0_pros;
  sens1_tx_h    = 0xFF & (sens1_pros >> 8);               //
  sens1_tx_l    = 0xFF & sens1_pros;
  sens2_tx_h    = 0xFF & (sens2_pros >> 8);               //
  sens2_tx_l    = 0xFF & sens2_pros;
  sens3_tx_h    = 0xFF & (sens3_pros >> 8);               //
  sens3_tx_l    = 0xFF & sens3_pros;
  sens4_tx_h    = 0xFF & (sens4_pros >> 8);               //
  sens4_tx_l    = 0xFF & sens4_pros;
  sens5_tx_h    = 0xFF & (sens5_pros >> 8);               //
  sens5_tx_l    = 0xFF & sens5_pros;
  
  aux_tx        = output_state;               //Estado de los relay (Encendido = 1 / Apagado = 0)
  aux_tx1       = 0;
  aux_tx2       = 0;
  eof_tx        = '%';
}

void make_eeprom_trama(byte a,int b)  // a = id_trama = 11 (envio de datos eeprom) , b = i del bucle for
{ 
  sof_tx      = '#';
  id_trama_tx = a;                          // a = id_trama = 11 
  estado_tx   = b;                          // envia estado q se recibiò o incrementador para trama normal 
  sens0_tx_h    = 0xFF & EEPROM.read(10 + b * 12);               //
  sens0_tx_l    = 0xFF & EEPROM.read(11 + b * 12);
  sens1_tx_h    = 0xFF & EEPROM.read(12 + b * 12);               //
  sens1_tx_l    = 0xFF & EEPROM.read(13 + b * 12);
  sens2_tx_h    = 0xFF & EEPROM.read(14 + b * 12);               //
  sens2_tx_l    = 0xFF & EEPROM.read(15 + b * 12);
  sens3_tx_h    = 0xFF & EEPROM.read(16 + b * 12);              //
  sens3_tx_l    = 0xFF & EEPROM.read(17 + b * 12);
  sens4_tx_h    = 0xFF & EEPROM.read(18 + b * 12);               //
  sens4_tx_l    = 0xFF & EEPROM.read(19 + b * 12);
  sens5_tx_h    = 0xFF & EEPROM.read(20 + b * 12);              //
  sens5_tx_l    = 0xFF & EEPROM.read(21 + b * 12);
  
  aux_tx        = 0;               
  aux_tx1       = 0;
  aux_tx2       = 0;
  eof_tx        = '%'; 
}

void make_trama_actuadores(byte a, byte b, byte c)
{
    sof_tx      = '#';
  id_trama_tx   = a;                          // a = id_trama = 11 
  estado_tx     = b;                          // envia estado q se recibiò o incrementador para trama normal 
  sens0_tx_h    = c;              
  sens0_tx_l    = 0;
  sens1_tx_h    = 0;             
  sens1_tx_l    = 0;
  sens2_tx_h    = 0;              
  sens2_tx_l    = 0;
  sens3_tx_h    = 0;              
  sens3_tx_l    = 0;
  sens4_tx_h    = 0;           
  sens4_tx_l    = 0;
  sens5_tx_h    = 0;            
  sens5_tx_l    = 0;
  aux_tx        = 0;               
  aux_tx1       = 0;
  aux_tx2       = 0;
  eof_tx        = '%'; 
}

void send_trama()
{
  Serial.write(sof_tx);
  Serial.write(id_trama_tx);
  Serial.write(estado_tx);
  Serial.write(sens0_tx_h);
  Serial.write(sens0_tx_l);
  Serial.write(sens1_tx_h);
  Serial.write(sens1_tx_l);
  Serial.write(sens2_tx_h);
  Serial.write(sens2_tx_l);
  Serial.write(sens3_tx_h);
  Serial.write(sens3_tx_l);
  Serial.write(sens4_tx_h);
  Serial.write(sens4_tx_l);
  Serial.write(sens5_tx_h);
  Serial.write(sens5_tx_l);
  
  Serial.write(aux_tx);
  Serial.write(aux_tx1);
  Serial.write(aux_tx2);
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
    while(contador < 18);  // largo trama 19  
    if((arreglo[0] == '#') && (arreglo[18] == '%'))
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

    switch(id_trama)   //  Ingreso a la funciones control manual set_manual() y calibracion set_calibracion(flag_cal)
    {
      case(1):
      {
        make_trama(1,0);   // id_trama = 1 trama de set point
        send_trama(); 
        flag_setpoint = 1;
        id_trama = 0;
        break;
      } 
      case(2): //trama = manual
      {
        make_trama(2,0);   // id_trama = 2 trama manual
        send_trama(); 
        control.manual(inst1_man);
        id_trama = 0; 
        flag_setpoint = 0;
        break;
      }
      case(3): //trama = calibraciòn
      {
        make_trama(3,0);   // id_trama = 3 hacia java = calibracion
        send_trama();  
        set_calibracion(flag_cal);
        id_trama = 0;
        flag_setpoint = 0;
        break;
      }
      case(4):
      {
        make_trama(4,0);
        send_trama();
        flag_inicio = 1;
        flag_setpoint = 0;
        id_trama = 0;
        break;
      }
      case(5):  //apagado de actuadores
      {
        make_trama(5,0);
        send_trama();
        flag_inicio = 0;
        actuadores.off();
        flag_setpoint = 0;
        id_trama = 0;
        break;
      }
      case(7):
      {
        byte ph4_lsb = EEPROM.read(12);                 // Valor de calibración almacenado para pH 4 (* 100 / 2)
        byte ph4_msb = EEPROM.read(14); 
        sensor_ph_4_cal = ((ph4_msb << 8) + ph4_lsb) * (voltaje_ref_ADC / 1023);
        byte ph7_lsb = EEPROM.read(16);                 // Valor de calibración almacenado para pH 4 (* 100 / 2)
        byte ph7_msb = EEPROM.read(18); 
        sensor_ph_7_cal = ((ph7_msb << 8) + ph7_lsb) * (voltaje_ref_ADC / 1023);
        paso_ph_cal = (float)((7 - 4) / (sensor_ph_7_cal - sensor_ph_4_cal));  // Almacena el paso y los valores de calibración en variables globales.
        ok_calibration = 1; 
        flag_setpoint = 0;
        id_trama = 0;
        break;        
        //make_trama(7,0);                                               //Hay datos de calibracion en memoria id_trama = 7
        //send_trama();
      }
      case(8):  // trama configuracion actuadores
      {
        make_trama_actuadores(8,actuador_h_or_l,aux2_config );   
        send_trama();
        sensor_actuador();
        flag_setpoint = 0;
        id_trama = 0;
        break;      
      }
      case(9):     // trama reseteo variables y apagado de actuadores
      {
        make_trama(9,0);   
        send_trama();
        variables_reset();
        actuadores.off();
        flag_setpoint = 0;
        id_trama = 0;
        break;
      }
     case(10):   //trama de data logger
     {
       make_trama(10,0);   //Peticion de datos en memoria
       send_trama();
       
       for(j = 0; j <=((sector_init - 10) / 12); j++) 
       {
         make_eeprom_trama(11, j);   //envio de datos de memoria
         send_trama();
         delay(5);
       }
       sector_init = 10;
       aux_eeprom = 0;
       contador_eeprom = 0;
       make_trama(12,0);        //Fin del envio de datos de memoria
       send_trama();
       id_trama = 0;
       break;
     }
    case(14):     // trama con valor en minutos para variable eeprom_tasa, tiempo de escritura en data logger
    {
      make_trama(14,eeprom_tasa);   
      send_trama();
      id_trama = 0;
      break; 
    }
    case(15):     // recepcion de trama 15 y envio de trama =15
    {
      make_trama(15,0);   
      send_trama();
      id_trama = 0;
      break; 
    }
          
    default:break;
   }
    
    // CADA 100 ms LECTURA DE SENSORES
    timer_lectura++;        //timer de lectura de sensores aumenta cada 100 (ms)
    lectura_sensores();     // se llama procesar_datos() desde lectura_sensores() 
    timer_muestreo = 0;
    timer_loop = 0; 
  } 
 
  //Serial.println(sens1_read);
  if(seconds == 100) //Cada 1 segundo 
  {
    incrementador_tx++;  // byte Retorna a cero despues de 255
    estado_output();     //estado de la salidas digitales HIGH = 1 o LOW = 0  valores almacenados en variable output_state.
    make_trama(0,incrementador_tx);  // 0 trama normal hacia java cada 1 segundo, con incrementador_tx++
    send_trama();
    
    
    if((contador_eeprom == eeprom_tasa) && (aux_eeprom == 0))  // Cada x segundos y siempre que no estè llena la memoria
    {
      EEPROM.write(sector_init,sens0_tx_h);  // inicia en el byte 10, EEPROM
      EEPROM.write(sector_init+1,sens0_tx_l);
      EEPROM.write(sector_init+2,sens1_tx_h);
      EEPROM.write(sector_init+3,sens1_tx_l);
      EEPROM.write(sector_init+4,sens2_tx_h);
      EEPROM.write(sector_init+5,sens2_tx_l);
      EEPROM.write(sector_init+6,sens3_tx_h);
      EEPROM.write(sector_init+7,sens3_tx_l);
      EEPROM.write(sector_init+8,sens4_tx_h);
      EEPROM.write(sector_init+9,sens4_tx_l);
      EEPROM.write(sector_init+10,sens5_tx_h);
      EEPROM.write(sector_init+11,sens5_tx_l);
      sector_init = sector_init + 12;
      make_trama(13,sector_init);
      send_trama();
      if(sector_init >= 982)
      {
        sector_init = 982;
        aux_eeprom = 1;
      }
      else
      {
        aux_eeprom = 0;
      }
      
      contador_eeprom = 0;
    }
    /*---------------------------------------------------------------------Control ON/OFF de variables - Trama SETPOINT-----------------*/
    
    // Si id_trama(1) = control  && (1seg / 0.01 seg) = 100, condicion if cada 1 seg.
    /*-Solo se tendra estado manual o automatico, no se podra tener una variable con control automatico y otra con control manual*/

    if(flag_setpoint == 1)   
    { 
      set_point();
    }
    else
    {
      sens0_incrementador_c_y_e = 0;
      sens1_incrementador_c_y_e = 0;
      sens2_incrementador_c_y_e = 0;
      sens3_incrementador_c_y_e = 0;
      sens4_incrementador_c_y_e = 0;
      sens5_incrementador_c_y_e = 0;
    }       
    seconds = 0;
  }
  
    /*- Periodo de ejecucion funciones -*/
  if(aux_timer1 == 1) // cada 10 (ms) 
  { 
    seconds++;             //aumenta cada 10 (ms)
    milisegundos++;        //aumenta cada 10 (ms)  
    timer_muestreo++;
    timer_loop++;      
    // SOLO PARA MINUTOS
    if(milisegundos == 100)  // milisegundos == 100 * 10 (ms) = 1 seg
    {
      milisegundos = 0;
      segundos++;
    } 
    if(segundos == 60)      // segundos == 60 = 1 minuto
    {
      segundos = 0;
      //minutos++;
      contador_eeprom++;
    }
    // END SOLO PARA MINUTOS 
    aux_timer1 = 0;    
  }
} //<- final loop




