/*Codigo Motor-Generador Dc con Controladores:
   Autores: David Santiago Oyola Lozano
   Diego Fernando Blanco Rueda
   Heiner Camilo Varela
*/
#include <util/atomic.h>

//DEFINICION DE PINES DE LOS COMPONENTES:-----------------------------------------------------------------------------------------------------

//Encoder:
#define Encoder 2

//Puente H:
int ENA = 11;
int IN1 = 8;
int IN2 = 9;

//DEFINICION DE VARIABLES:--------------------------------------------------------------------------------------------------------------------
//Lectura Analogica del Encoder:

//Variable global de pulsos compartida con la interrupción
unsigned long timeold;
float resolution = 374.22;

//Variables del calculo de velocidad angular:
unsigned long pulsos, Time1, Time, TimeActual, TimeAnterior, DeltaT, TimeControl = 0;
double RPM, voltOut, frqVel, Wrad_s, k;
volatile int ISRCounter = 0;

//Variables para tiempo de muestro:
int PeriodoMuestreo = 15;

//Vector de suma promnedio:

//Parametros de control de PWM:
int  PWM = 0; //PWM que se controla.
float lecturaAnalogica = 0; //Lectura analogica motor.
float voltajeSalida = 0; //Voltaje Generador.

//Parametros controlador:
float sp; //Valor del setpoint
float pv; //Valor de la variable del proceso
float en;
float un = 0;
float Uc;
float K;           //Ganancia
float Umax = 100;         //Acción de control máxima
float Umin = 0;         //Acción de control mínimo
float un_1 = 0;     //Elemento de memoria del integrador

//Parametros Controlador PI y PID:
float L = 0.03;
float T = 1.395;
float Tm = 0.015; //Periodo de muestreo en segundos
float Ksta = 3.63 / 60; //Ganancia estática
float Lambda;
float KgPrima;
float TiPrima;
float TdPrima;
float Kcr = 800;
float Pcr = 0.375;

float en_1; //Error una muestra anterior
float I_error_1 = 0;  //Memoria Integral del error
float Kg; //Ganancia
float Kp; //Ganancia Proporcional
float Ki; //Ganancia Integral
float Kd; //Ganancia Derivativa
float Ti; //Constante de tiempo integral
float Td; //Constante de tiempo derivativo
float A_p;
float A_i;
float A_d;
float I_error;

//INICIO DEL CODIGO:-------------------------------------------------------------------------------------------------------------------------
//Funcion del encoder:
void doEncode() {
  if (millis() - Time > 3) {
    ISRCounter++;
    Time = millis();
  }
  if (ISRCounter == 2) {
    TimeAnterior = millis();
  }
  if (ISRCounter >= 4) {
    TimeActual = millis();
    DeltaT = TimeActual - TimeAnterior;
    k = 1;
    ISRCounter = 0;
  }
}

void monitorSerial(int seleccion){
  switch(seleccion){
    case 1: //IMPRIMIR VOLTAJE, SP, PWM y Tiempo:
      Serial.print(" "); Serial.print(voltajeSalida); Serial.print(" ");  //Voltaje de Salida en Voltios.
      Serial.print(" "); Serial.print(sp); Serial.print(" ");  //Setpoint del controlador.
      //Serial.print(" "); Serial.print(PWM * 5 / 100); Serial.print(" "); //Porcentaje de PWM aplicado y escalado.
      Serial.print(" "); Serial.print(PWM); Serial.print(" ");  //Porcentaje de PWM aplicado.
      //Serial.print(" "); Serial.print(timeold); Serial.print(" ");  //Tiempo
      Serial.println();
    break;
    case 2: //IMPRIMIR RPM, SP, PWM y Tiempo:
      Serial.print(" "); Serial.print(RPM); Serial.print(" ");  //Velocidad en RPM.
      Serial.print(" "); Serial.print(sp); Serial.print(" ");  //Setpoint del controlador.
      Serial.print(" "); Serial.print(PWM * 5 / 100); Serial.print(" "); //Porcentaje de PWM aplicado y escalado.
      //Serial.print(" "); Serial.print(PWM); Serial.print(" ");  //Porcentaje de PWM aplicado.
      //Serial.print(" "); Serial.print(timeold); Serial.print(" ");  //Tiempo
      Serial.println();
    break;
    case 3: //Imprimir Constantes Controles PI y PID (TODOS LOS METODOS):
      Serial.print("Kg:"); Serial.println(Kg); Serial.println(" ");
      Serial.print("Tm:"); Serial.println(Tm); Serial.println(" ");
      Serial.print("Ti:"); Serial.println(Ti); Serial.println(" ");
      Serial.print("Td:"); Serial.println(Td); Serial.println(" ");
      Serial.print("Lambda:"); Serial.println(Lambda); Serial.println(" ");
      Serial.print("Kg':"); Serial.println(KgPrima); Serial.println(" ");
      Serial.print("Ti':"); Serial.println(TiPrima); Serial.println(" ");
      Serial.print("Td':"); Serial.println(TdPrima); Serial.println(" ");
      Serial.print("Kp:"); Serial.println(Kp); Serial.println(" ");
      Serial.print("Ki:"); Serial.println(Ki); Serial.println(" ");
      Serial.print("Kd:"); Serial.println(Kd); Serial.println(" ");
      Serial.print("Kcr:"); Serial.println(Kcr); Serial.println(" ");
      Serial.print("Pcr:"); Serial.println(Pcr); Serial.println(" ");
    break;
  }
}
//Funcion de los controladores (ON-OFF, Proporcional, Integral y el por defecto):
void controladores(int tipoControl, int variableProcesoSeleccionada) {
  //Se selecciona la variable del proceso a controlar:
  switch (variableProcesoSeleccionada) {
    case 1:
      pv = voltajeSalida;
    break;
    case 2:
      pv = RPM;
    break;
  }

  //Controladores:
  switch (tipoControl) {
    case 1: //Control ON-OFF:
      if (Serial.available() > 0) {
        String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
        sp = entradaSerial.toFloat();
      }
      en = sp - pv;

      if (en > 0) {
        un = Umax;
      } else {
        un = Umin;
      }
      PWM = un;
    break;
    case 2: //Control Proporcional:
      if (Serial.available() > 0) {
        String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
        sp = entradaSerial.toFloat();
      }

      if (variableProcesoSeleccionada == 1) {
        //K = 28; //GANANCIA PARA VOLTAJE/PWM
        K = 800;  //GANANCIA PARA VOLVERLA OSCILATORIA
      }
      if (variableProcesoSeleccionada == 2) {
        K = 0.05; //GANANCIA PARA VELOCIDAD/PWM
      }

      en = sp - pv;
      Uc = K * en;

      un = Uc;

      if (Uc < Umin) {
        un = Umin;
      }
      if (Uc > Umax) {
        un = Umax;
      }
      PWM = un;
    break;
    case 3: //Control Integral:
      if (Serial.available() > 0) {
        String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
        sp = entradaSerial.toFloat();
      }

      if (variableProcesoSeleccionada == 1) {
        K = 0.35; //GANANCIA PARA VOLTAJE/PWM
      }
      if (variableProcesoSeleccionada == 2) {
        K = 0.0042; //GANANCIA PARA VELOCIDAD/PWM
      }

      en = sp - pv;
      Uc = K * en + un_1;

      un = Uc;

      if (Uc < Umin) {
        un = Umin;
      }
      if (Uc > Umax) {
        un = Umax;
      }
      PWM = un;
      un_1 = un;
      break;
    case 4: //Control de setpoint de PWM saltando de 10 en 10 segun tiempo
      if (millis() > 1000) {
        PWM = int(millis() / 1000);
      }
      if (millis() >= 101000) {
        PWM = 0;
      }
    break;
    case 5: //Control PI: (METODO DE ZIEGLER NICHOLS DE LAZO ABIERTO)
      //Kg = 0.9*(T/(L*Ksta));
      //Ti = L/0.3;
      Kg = 50;
      Ti = 0.5;
      Td = 0;
      Kp = Kg * (1 - (Tm / (2 * Ti)));
      Ki = (Kg * Tm) / Ti;
      Kd = Kg * Td / Tm;

      if (Serial.available() > 0) {
        String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
        sp = entradaSerial.toFloat();
      }

      en = sp - pv;

      A_p = Kp * en;
      A_d = Kd * (en - en_1);
      I_error = en + I_error_1;
      A_i = Ki * I_error;
      Uc = A_p + A_d + A_i;

      un = Uc;
      if (Uc < 0) {
        un = Umin;
      }
      if (Uc > 100) {
        un = Umax;
      }
      PWM = un;
      en_1 = en;
      I_error_1 = I_error;
    break;
    case 6: //Control PID: (METODO DE ZIEGLER NICHOLS DE LAZO ABIERTO)
      //Kg = 1.2*(T/(L*Ksta));
      //Ti = 2*L;
      //Td = 0.5*L;
      Kg = 75;
      Ti = 0.5;
      Td = 0.15;

      Kp = Kg * (1 - (Tm / (2 * Ti)));
      Ki = (Kg * Tm) / Ti;
      Kd = Kg * Td / Tm;

      if (Serial.available() > 0) {
        String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
        sp = entradaSerial.toFloat();
      }

      en = sp - pv;

      A_p = Kp * en;
      A_d = Kd * (en - en_1);
      I_error = en + I_error_1;
      A_i = Ki * I_error;
      Uc = A_p + A_d + A_i;

      un = Uc;
      if (Uc < 0) {
        un = Umin;
      }
      if (Uc > 100) {
        un = Umax;
      }
      PWM = un;
      en_1 = en;
      I_error_1 = I_error;
    break;
    case 7: //Control PI: (METODO LAMBDA)
      //Lambda = 3*T; //3T para una respuesta robusta, colocar T para una respuesta agresiva.
      Kg = T / (Ksta * (L + Lambda));
      Ti = T;
      Td = 0;
      Lambda = 0.007;

      //Kp = Kg*(1-(Tm/(2*Ti)));
      //Ki = (Kg*Tm)/Ti;
      Kp = 5;
      Ki = 0.1;
      Kd = Kg * Td / Tm;

      if (Serial.available() > 0) {
        String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
        sp = entradaSerial.toFloat();
      }

      en = sp - pv;

      A_p = Kp * en;
      A_d = Kd * (en - en_1);
      I_error = en + I_error_1;
      A_i = Ki * I_error;
      Uc = A_p + A_d + A_i;

      un = Uc;
      if (Uc < 0) {
        un = Umin;
      }
      if (Uc > 100) {
        un = Umax;
      }
      PWM = un;
      en_1 = en;
      I_error_1 = I_error;
    break;
    case 8: //Control PID: (METODO LAMBDA)
      //Lambda = 3 * T; //3T para una respuesta robusta, colocar T para una respuesta agresiva.
      Lambda = 0.007;
      KgPrima = T/(Ksta*((L/2)+Lambda));
      TiPrima = T;
      TdPrima = 0.5*L;
      
      Kg = KgPrima*((TiPrima+TdPrima)/TiPrima);
      Ti = TiPrima+TdPrima;
      Td = (TiPrima*TdPrima)/(TiPrima+TdPrima);

      //Kp = Kg * (1 - (Tm / (2 * Ti)));
      //Ki = (Kg * Tm) / Ti;
      //Kd = Kg * Td / Tm;
      Kp = 5;
      Ki = 0.01;
      Kd = 5;
      
      if (Serial.available() > 0) {
        String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
        sp = entradaSerial.toFloat();
      }

      en = sp - pv;

      A_p = Kp * en;
      A_d = Kd * (en - en_1);
      I_error = en + I_error_1;
      A_i = Ki * I_error;
      Uc = A_p + A_d + A_i;

      un = Uc;
      if (Uc < 0) {
        un = Umin;
      }
      if (Uc > 100) {
        un = Umax;
      }
      PWM = un;
      en_1 = en;
      I_error_1 = I_error;
    break;
    case 9: //Control PI: (METODO DE ZIEGLER NICHOLS DE LAZO CERRADO)
      Kg = 0.45*Kcr;
      Ti = (1/1.2)*Pcr;
      //Kg = 50;
      //Ti = 0.5;
      Td = 0;
      //Kp = Kg * (1 - (Tm / (2 * Ti)));
      //Ki = (Kg * Tm) / Ti;
      Kd = Kg * Td / Tm;
      Kp = 49.25;
      Ki = 1.5; 
      if (Serial.available() > 0) {
        String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
        sp = entradaSerial.toFloat();
      }

      en = sp - pv;

      A_p = Kp * en;
      A_d = Kd * (en - en_1);
      I_error = en + I_error_1;
      A_i = Ki * I_error;
      Uc = A_p + A_d + A_i;

      un = Uc;
      if (Uc < 0) {
        un = Umin;
      }
      if (Uc > 100) {
        un = Umax;
      }
      PWM = un;
      en_1 = en;
      I_error_1 = I_error;
    break;
    case 10:  //Control PID: (METODO DE ZIEGLER NICHOLS DE LAZO CERRADO)
      Kg = 0.6*Kcr;
      Ti = 0.5*Pcr;
      Td = 0.125*Pcr;
      //Kg = 50;
      //Ti = 0.5;
      //Td = 0;
      //Kp = Kg * (1 - (Tm / (2 * Ti)));
      //Ki = (Kg * Tm) / Ti;
      //Kd = Kg * Td / Tm;
      Kp = 73.87;
      Ki = 2.25;
      Kd = 50;
      if (Serial.available() > 0) {
        String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
        sp = entradaSerial.toFloat();
      }

      en = sp - pv;

      A_p = Kp * en;
      A_d = Kd * (en - en_1);
      I_error = en + I_error_1;
      A_i = Ki * I_error;
      Uc = A_p + A_d + A_i;

      un = Uc;
      if (Uc < 0) {
        un = Umin;
      }
      if (Uc > 100) {
        un = Umax;
      }
      PWM = un;
      en_1 = en;
      I_error_1 = I_error;
    break;
    default:  //Control por defecto (setpoint de PWM Manualmente):
      if (Serial.available() > 0) {
        String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el PWM a tomar.
        PWM = entradaSerial.toFloat();
      }
    break;
  }
}

void setup() {
  TCCR1B = TCCR1B & B11111000 | B00000101;
  //Activacion de la interrupcion en flanco de bajada del encoder:
  Serial.begin(9600); //Monitor Serial en 9600 baudios.

  //Se definen las entradas y salidas de los pines:
  pinMode(Encoder, INPUT);  //Se necesita el pin del encoder como entrada para la interrupcion.
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT); //Se define los pines del puente H como Salidas.
  pinMode(IN2, OUTPUT);

  //Señal Puente H L298N:
  digitalWrite(IN1, HIGH); //Se ajustan IN1 e IN2 para que gire en sentido horario el motor.
  digitalWrite(IN2, LOW); //Si se quiere girar antihorario definir IN1=LOW e IN2=HIGH.
  timeold = 0;

  attachInterrupt(digitalPinToInterrupt(Encoder), doEncode, RISING);
  Time1 = millis();
  DeltaT, RPM = 0;
}

void loop() {
  /*if(PWM < 30){
    w = 0;
    }*/
  //INSTRUCCIONES COMANDO CONTROLADORES:
  //Forma del comando: controladores(ControladorAUsar, VariableDelProcesoASeleccionar);
  
  //**Como "ControladorAUsar" se pueden seleccionar las siguientes opciones:
  //1:Control ON-OFF.
  //2:Control Proporcional.
  //3:Control Integral. 
  //4.PWM de Rampa.
  //5.Control PI Ziegler Nichols de Lazo Abierto.
  //6.Control PID Ziegler Nichols de Lazo Abierto.
  //7.Control PI Lambda.
  //8.Control PID Lambda.
  //9:Control PI Ziegler Nichols de Lazo Cerrado. 
  //10: Control PID Ziegler Nichols de Lazo Cerrado. 
  //OtroNumero: PWM definido por el usuario.
  
  //**Como "VariableDelProcesoASeleccionar" se pueden seleccionar las siguientes opciones:
  //1: Selecciona el voltaje como variable del proceso
  //2: Selecciona la velocidad como variable del proceso.

  //Comando:
  controladores(8,1);

  analogWrite(ENA, map(PWM, 0, 100, 0, 255)); //Enviamos el PWM al puente H.

  if (millis() - timeold >= PeriodoMuestreo) { //Imprimirá los valores cada 15 milisegundos asegurando el tiempo de muestreo.
    //Modifica las variables de la interrupción forma atómica
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      //LECTURA VELOCIDAD:
      if (DeltaT > 0) {
        frqVel = 1000 * k / DeltaT;
      } else {
        frqVel = 0;
      }
      RPM = 60 * frqVel * 2;
      Wrad_s = RPM * 3.14159265359 / 30;

      timeold = millis();

      //LECTURA VOLTAJE:
      lecturaAnalogica = analogRead(A5); //Se lee el voltaje del pin analogico A5 para mensurar el voltaje de salida del generador.
      voltajeSalida = lecturaAnalogica * (5.0 / 1023.0);
    }
    //Se obtendrá las muestras de velocidad, voltaje y PWM para los instantes de tiempo correspondientes al tiempo de muestreo.
    //Salida de monitor Serial para imprimir los datos:
    monitorSerial(1);
  }
}
