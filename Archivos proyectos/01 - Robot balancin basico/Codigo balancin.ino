/* ---------- LIBRERÍAS ---------- */
//I2C:
#include "I2Cdev.h"                                                         //Libreria auxiliar para I2C.
#include "Wire.h"                                                           //Libreria estandar para comunicación I2C.

//MPU:
#include "MPU6050_6Axis_MotionApps20.h"                                     //Libreria particular de la libreria MPU6050 que activa al DMP.

//PID:
#include <PID_v1.h>


/* ---------- VARIABLES ---------- */
//MPU:
uint8_t estadoDMP;                                                          //Variable para guardar el estado. 
uint16_t TamanoPaqueteDMP;                                                  //Tamaño de cada paquete que enviará el DMP.
uint16_t ColaBytesDMP;                                                      //Bytes guardados esperando a ser leidos.
uint8_t DatosCrudosDMP[64];                                                 //Lista donde se guardarán los datos crudos del sensor.
// uint8_t es int de 8 bits (para ahorrar memoria).

Quaternion PaqueteDatosDMP;                                                 //Cuaternión: Paquete de datos de orientación.  
VectorFloat gravedad;                                                       //Dirección hacía donde está la gravedad.
float ypr[3];                                                               //Lista para 3 valores: Yaw, Pitch y Roll.
VectorInt16 aceleracionBruta;                                               //Contenedor para la aceleración bruta (con gravedad)
VectorInt16 aceleracionReal;                                                //Contenedor para la aceleración real (sin gravedad)
float angulo_equilibrio = 1.2;                                              //Debería ser 0, pero la IMU está chueca.
//Tipo de datos de la libreria. VectorInt16 guarda 3 valores. X,Y,Z.

//PID:
double setpoint, input, output;
double kp = 1.1, ki = 0.0, kd = 0.0;

//MOTOR1:
int A1A = 3;      //LOW para adelante.
int A1B = 5;      //LOW para reversa.
//MOTOR2:
int B1A = 6;      //LOW para adelante.
int B1B = 9;      //LOW para reversa.


/* ---------- OBJETOS ---------- */
//MPU:
MPU6050 mpu(0x68);                                                          //Objeto y su dirección.

//PID:
PID PIDmpu(&input, &output, &setpoint, kp, ki, kd, DIRECT);

/* ---------- CONFIGURACIONES INICIALES ---------- */
void setup() {
    Wire.begin();                                                           //Inicia el I2C.
    Wire.setClock(400000);                                                  //Aumenta la velocidad de la comunicación I2C porque el DMP envía demasiados datos.
    Serial.begin(115200);                                                   //Inicia la comunicación Serial.

  //MPU:
    mpu.initialize();                                                       //Inicia el mpu (pero el DMP aún no).
    
    estadoDMP = mpu.dmpInitialize();                                        //Carga el código al DMP. Si tuvo éxito devuelve 0.

    //Realizar Test de calibración para modificar los siguientes datos:
    mpu.setXGyroOffset(153);
    mpu.setYGyroOffset(-57);
    mpu.setZGyroOffset(6);

    mpu.setXAccelOffset(4928);
    mpu.setYAccelOffset(-4576);
    mpu.setZAccelOffset(9616);

    if (estadoDMP == 0) {
        mpu.setDMPEnabled(true);                                            //Si la inicialización resultó, el activa el DMP.
        TamanoPaqueteDMP = mpu.dmpGetFIFOPacketSize();                      //Pregunta cual será el tamaño de los paquetes que enviará el DMP (suelen ser de 42 bytes).
    } else {
        Serial.print("Error al iniciar el DMP");
    }

    //PID:
    setpoint = angulo_equilibrio;                         //grados en Pitch (y).
    PIDmpu.SetOutputLimits(100, 255);
    PIDmpu.SetMode(AUTOMATIC);

    //MOTORES:
    pinMode(A1A,OUTPUT);
    pinMode(A1B,OUTPUT);

    pinMode(B1A,OUTPUT);
    pinMode(B1B,OUTPUT);
}


/* ---------- BUCLE PRINCIPAL ---------- */
void loop() {
  //MPU:
  ColaBytesDMP = mpu.getFIFOCount();                                      //Leer cuantos bytes hay acumulados en el DMP.

  if (ColaBytesDMP < TamanoPaqueteDMP) {                                  //Si los bytes acumulados aún no llenan un paquete:
    return;                                                                 //Reiniciamos loop.
  }
  if (ColaBytesDMP >= 1024) {                                             //Si se llenó la memoria del DMP (1024 bytes):
    mpu.resetFIFO();                                                        //Borramos toda la memoria.
    Serial.println("FIFO Overflow!");
    return;                                                                 //Reiniciamos loop.
  }
  while(ColaBytesDMP >= TamanoPaqueteDMP) {                               //Mientras haya datos disponibles (paquetes completos):
    mpu.getFIFOBytes(DatosCrudosDMP, TamanoPaqueteDMP);                     //Arg1: Donde se guardarán los datos. Arg2: Cuantos leo? (tamaño del paquete).
    ColaBytesDMP -= TamanoPaqueteDMP;                                       //Restamos lo que leimos.
  }

  //PROCESAMIENTO DE LOS DATOS CRUDOS:
  mpu.dmpGetQuaternion(&PaqueteDatosDMP, DatosCrudosDMP);                                 //Tomar datos crudos y guardar en PaqueteDatosDMP.
  mpu.dmpGetGravity(&gravedad, &PaqueteDatosDMP);                                         //Usa los datos guardados en PaqueteDatosDMP para saber donde está la gravedad y guardarla en la variable gravedad.
  mpu.dmpGetYawPitchRoll(ypr, &PaqueteDatosDMP, &gravedad);                               //Usa la gravedad y PaqueteDatosDMP para calcular Yaw, Pitch, Roll y guardalos en ypr.
  mpu.dmpGetAccel(&aceleracionBruta, DatosCrudosDMP);                                     //Extrae los datos brutos del DMP y los guarda en aceleracionBruta.
  mpu.dmpGetLinearAccel(&aceleracionReal, &aceleracionBruta, &gravedad);                  //Toma los datos brutos, le resta la gravedad para obtener la aceleración real (estado base 0,0,0).

  
  //Serial.print("Y(z): "); Serial.print(ypr[0] * 180/M_PI); Serial.print(" \t");
  //Serial.print(" P(y): "); Serial.print(ypr[1] * 180/M_PI); Serial.print(" \t");
  //Serial.print(" R(x): "); Serial.print(ypr[2] * 180/M_PI); Serial.print(" \t\t");
  //Serial.print("A(x): "); Serial.print(aceleracionReal.x); Serial.print(" \t");
  //Serial.print(" A(y): "); Serial.print(aceleracionReal.y); Serial.print(" \t");
  //Serial.print(" A(z): "); Serial.println(aceleracionReal.z);



  //PID:
  float grados_y = (ypr[1] * 180/M_PI); 
  input = grados_y;       //Grados IMU.
  PIDmpu.Compute();
  Serial.println(grados_y);         //Serial Plotter

  
  if(grados_y < angulo_equilibrio){       //Si el balancín se inclina adelante, avanzar adelante.
    //MOTOR1:
    analogWrite(A1A,0);
    analogWrite(A1B,output);
    //MOTOR2:
    analogWrite(B1A,0);
    analogWrite(B1B,output);
  }else{                //Si el balancín se inclina atrás, retroceder.
        //MOTOR1:
    analogWrite(A1A,output);
    analogWrite(A1B,0);
    //MOTOR2:
    analogWrite(B1A,output);
    analogWrite(B1B,0);
  }
}
//Roll (x): Rota muñeca de derecha a izquierda.
//Pitch (y): Rota muñeca de arriba a abajo.
//Yaw (z): Mueve muñeca de derecha a izquierda.
