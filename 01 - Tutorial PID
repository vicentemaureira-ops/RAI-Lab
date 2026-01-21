/* ---------- LIBRERIAS ---------- */
#include <PID_v1.h>                   //Librería PID de Brett Beauregard. Para control PID.


/* ---------- VARIABLES ---------- */
double setpoint, input, output;
//SETPOINT: Valor al que quiero que mi sistema llegue. Puede ser una temperatura fija, una velocidad, un ángulo, etc.
//INPUT: Señal que envía el sensor y mide lo que estamos buscando; un sensor de temperatura, un encoder, un giroscopio, etc.
//OUPUT: Señal que se le enviará a un actuador para que corrija el estado en el que nos encontramos en función de a lo que queremos llegar:
  //Más o menos energia a un calentador para regular la temperatura, más o menos señal a un motor para regular la velocidad, etc.

double kp = 2, ki = 5, kd = 1;
//KP: Variable proporcional del PID. Se podría decir que es el acelerador; qué tan rápido se empuja el input hacía el setpoint.
//KI: Variable integral del PID. Corrije las mini desviaciones; hace oscilar al input hasta que se fije sobre el setpoint
//KD: Variable derivativa del PID.. Reduce las sobre oscilaciones
//Estas variables son números que deben ir calibrandose en función de la respuesta del sistema que estemos trabajando.


/* ---------- OBJETOS ---------- */
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
/*
El último argumento es la forma en la que actua el control:
Si estamos debajo del setpoint y necesitamos aumentar la potencia del actuador para que el input llegue al setpoint: DIRECT
Si necesitamos BAJAR la potencia del actuador para que el input suba hasta el valor del setpoint (aire acondicionado y temperatura): REVERSE
*/


/* ---------- CONFIGURACIONES INICIALES ---------- */
void setup() {
  setpoint = 100;                     //Primero: Indicamos nuestra meta (valor que debería medir el sensor).
  myPID.SetOutputLimits(0, 255);      //Segundo: Establecemos los limites a los que funciona el actuador (PWM).
  myPID.SetMode(AUTOMATIC);           //Inicializar el PID. (el argumento AUTOMATIC activa el PID, no hay que configurar nada).
}


/* ---------- BUCLE PRINCIPAL ---------- */
void loop() {
  input = analogRead(A0);  //Lo primero es leer la señal del actuador.
  //Si es necesario hay que hacer cálculos para que la entrada muestre valores en la misma unidad que el setpoint.
  //Por ejemplo, si tenemos un sensor de temperatura que lee los grados kelvin y nosotros tenemos un setpoint de 20°C.
  //Debemos hacer lo siguiente: input = analogRead() + 273.15. (así nuestro input estará en grados celsius).

  myPID.Compute();         //Comando que manda a la librería PID a hacer todos los calculos matemáticos correspondientes.
  
  analogWrite(9, output);  //Finalmente, toda la retroalimentación y valor que necesita el actuador para que el input se iguale al setpoint
                            //a pesar de cualquier variación que pueda haber en el sistema, es entregada mediante la variable output.
}
