/* ---------- LIBRERÍAS ---------- */
#include "I2Cdev.h"                                                         //Libreria auxiliar para I2C.
#include "MPU6050_6Axis_MotionApps20.h"                                     //Libreria particular de la libreria MPU6050 que activa al DMP.
#include "Wire.h"                                                           //Libreria estandar para comunicación I2C.


/* ---------- OBJETOS ---------- */
MPU6050 mpu(0x68);                                                          //Objeto y su dirección.


/* ---------- VARIABLES ---------- */
uint8_t estadoDMP;                                                          //Variable para guardar el estado. 
uint16_t TamañoPaqueteDMP;                                                  //Tamaño de cada paquete que enviará el DMP.
uint16_t ColaBytesDMP;                                                      //Bytes guardados esperando a ser leidos.
uint8_t DatosCrudosDMP[64];                                                 //Lista donde se guardarán los datos crudos del sensor.
// uint8_t es int de 8 bits (para ahorrar memoria).

Quaternion PaqueteDatosDMP;                                                 //Cuaternión: Paquete de datos de orientación.  
VectorFloat gravedad;                                                       //Dirección hacía donde está la gravedad.
float ypr[3];                                                               //Lista para 3 valores: Yaw, Pitch y Roll.
VectorInt16 aceleracionBruta;                                               //Contenedor para la aceleración bruta (con gravedad)
VectorInt16 aceleracionReal;                                                //Contenedor para la aceleración real (sin gravedad)
//Tipo de datos de la libreria. VectorInt16 guarda 3 valores. X,Y,Z.


/* ---------- CONFIGURACIONES INICIALES ---------- */
void setup() {
    Wire.begin();                                                           //Inicia el I2C.
    Wire.setClock(400000);                                                  //Aumenta la velocidad de la comunicación I2C porque el DMP envía demasiados datos.
    Serial.begin(115200);                                                   //Inicia la comunicación Serial.

    mpu.initialize();                                                       //Inicia el mpu (pero el DMP aún no).
    
    estadoDMP = mpu.dmpInitialize();                                        //Carga el código al DMP. Si tuvo éxito devuelve 0.

    //Realizar Test de calibración para modificar los siguientes datos:
    mpu.setXGyroOffset(152);
    mpu.setYGyroOffset(-54);
    mpu.setZGyroOffset(19);

    mpu.setXAccelOffset(4843);
    mpu.setYAccelOffset(-4599);
    mpu.setZAccelOffset(9616);

    if (estadoDMP == 0) {
        mpu.setDMPEnabled(true);                                            //Si la inicialización resultó, el activa el DMP.
        TamañoPaqueteDMP = mpu.dmpGetFIFOPacketSize();                      //Pregunta cual será el tamaño de los paquetes que enviará el DMP (suelen ser de 42 bytes).
    } else {
        Serial.print("Error al iniciar el DMP");
    }
}


/* ---------- BUCLE PRINCIPAL ---------- */
void loop() {
    ColaBytesDMP = mpu.getFIFOCount();                                      //Leer cuantos bytes hay acumulados en el DMP.

    if (ColaBytesDMP < TamañoPaqueteDMP) {                                  //Si los bytes acumulados aún no llenan un paquete: esperar más datos (se reunicia el loop).
    } else {                                                                //Si hay paquetes completos:
        if (ColaBytesDMP == 1024) {                                             //Si se llenó la memoria del DMP (1024 bytes):
            mpu.resetFIFO();                                                        //Borramos toda la memoria.
            Serial.println("FIFO Overflow!");
        } else {                                                                //Si no se ha llenado la memoria del DMP:
            while (ColaBytesDMP >= TamañoPaqueteDMP) {                              //Mientras haya datos disponibles (paquetes completos):
                mpu.getFIFOBytes(DatosCrudosDMP, TamañoPaqueteDMP);                     //Arg1: Donde se guardarán los datos. Arg2: Cuantos leo? (tamaño del paquete).
                ColaBytesDMP -= TamañoPaqueteDMP;                                           //Restamos lo que leimos.
            }

            //PROCESAMIENTO DE LOS DATOS CRUDOS:
            mpu.dmpGetQuaternion(&PaqueteDatosDMP, DatosCrudosDMP);                                 //Tomar datos crudos y guardar en PaqueteDatosDMP.
            mpu.dmpGetGravity(&gravedad, &PaqueteDatosDMP);                                         //Usa los datos guardados en PaqueteDatosDMP para saber donde está la gravedad y guardarla en la variable gravedad.
            mpu.dmpGetYawPitchRoll(ypr, &PaqueteDatosDMP, &gravedad);                               //Usa la gravedad y PaqueteDatosDMP para calcular Yaw, Pitch, Roll y guardalos en ypr.
            mpu.dmpGetAccel(&aceleracionBruta, DatosCrudosDMP);                                     //Extrae los datos brutos del DMP y los guarda en aceleracionBruta.
            mpu.dmpGetLinearAccel(&aceleracionReal, &aceleracionBruta, &gravedad);                  //Toma los datos brutos, le resta la gravedad para obtener la aceleración real (estado base 0,0,0).

            Serial.print("Y(z): "); Serial.print(ypr[0] * 180/M_PI); Serial.print(" \t");
            Serial.print(" P(y): "); Serial.print(ypr[1] * 180/M_PI); Serial.print(" \t");
            Serial.print(" R(x): "); Serial.print(ypr[2] * 180/M_PI); Serial.print(" \t\t");
            Serial.print("A(x): "); Serial.print(aceleracionReal.x); Serial.print(" \t");
            Serial.print(" A(y): "); Serial.print(aceleracionReal.y); Serial.print(" \t");
            Serial.print(" A(z): "); Serial.println(aceleracionReal.z);
        }
    }
}
//Roll (x): Rota muñeca de derecha a izquierda.
//Pitch (y): Rota muñeca de arriba a abajo.
//Yaw (z): Mueve muñeca de derecha a izquierda.
