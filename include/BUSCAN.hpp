#ifndef BUSCAN_H
#define BUSCAN_H

#include <arpa/inet.h>
#include <sys/socket.h>
#include <semaphore.h>
#include <canlib.h>
#include <inttypes.h>
#include <stdio.h>
#include <pthread.h>
#include <math.h>
#include <string.h>

#include <unistd.h>
#include <fcntl.h>

//#include <signal.h>
//#include <errno.h>
//#include <time.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <termios.h>
//#include <sys/signal.h>
//#include <sys/types.h>
//#include <sys/ipc.h>
//#include <sys/shm.h>
//#include <sys/select.h>
//#include <sys/time.h>


#include <iostream>

#define NUM_IDS 29
#define STOP_THRESHOLD 5 		//Umbral en km/h por debajo del cual se considera STOP
#define ACC_MIN_THRESHOLD -7		//Umbral en m/s2 por debajo dle cual se considera FRENAZO
#define VEL_CTE_THRESHOLD 7		//Umbral en (+- %  de variacion de velocidad) por encima y por debajo del cual se empieza a contar el cambio de velocidad
#define VEL1_WIPER_WEIGHT 0.4           //Pesos para ponderar la activacion del limpiaparabrisas en funcion de la velocidad
#define VEL2_WIPER_WEIGHT 0.6           //Pesos para ponderar la activacion del limpiaparabrisas en funcion de la velocidad
#define VEL3_WIPER_WEIGHT 0.8		//Pesos para ponderar la activacion del limpiaparabrisas en funcion de la velocidad


enum estado_intermitente {intermitente_off, intermitente_izquierda, intermitente_derecha};
enum estado_luces_emergencia {emergencia_off, emergencia_on};
enum estado_luces {luces_off, luces_posicion, luces_cortas, luces_largas, luces_rafaga, luces_rafaga_posicion};
enum estado_luces_antiniebla {antiniebla_off, antiniebla_on};
enum estado_marchas {P, R, N, D1, D2, D3, D4, M1, M2, M3, M4};
enum estado_modo_cambio {normal, sport, invierno};

//Estructura de datos BUSCAN procesados
//_________________________________________________________________________________________________

//ID 520
typedef struct
{
    uint32_t timestamp;
    double rpm;
    double pedal_acelerador;

}BUSCAN_data_t0_t;

//ID 761
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t1_t;

//ID 773
typedef struct
{
    uint32_t timestamp;
    double volante_angulo;
    double volante_velocidad;
    int8_t volante_sentido;

}BUSCAN_data_t2_t;

//ID 781
typedef struct
{
    uint32_t timestamp;

    double vrdd;
    double vrdi;
    double vrtd;
    double vrti;

}BUSCAN_data_t3_t;

//ID 840
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t4_t;

//ID 841
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t5_t;

//ID 845
typedef struct
{
    uint32_t timestamp;
    uint8_t ESP_activado;
    uint8_t ESP_disparado;

}BUSCAN_data_t6_t;

//ID 909
typedef struct
{
    uint32_t timestamp;
    double vel;
    double distancia;
    double acc;

}BUSCAN_data_t7_t;

//ID 973
typedef struct
{
    uint32_t timestamp;
    uint8_t freno_servicio;
    double freno_servicio_intensidad;

}BUSCAN_data_t8_t;

//ID 1037
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t9_t;

//ID 1042
typedef struct
{
    uint32_t timestamp;
    uint8_t freno_estacionamiento;
    uint8_t puertas_abiertas;
    double temp_motor;

}BUSCAN_data_t10_t;

//ID 1074
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t11_t;

//ID 1101
typedef struct
{
    uint32_t timestamp;

    double vel_media_ruedas_delanteras;
    double vel_rueda_trasera_izq;
    double vel_rueda_trasera_dcha;
    double reservado;
}BUSCAN_data_t12_t;

//ID 1128
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t13_t;

//ID 1160
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t14_t;

//ID 1161
typedef struct
{
    uint32_t timestamp;
    enum estado_marchas marchas;
    enum estado_modo_cambio modo_cambio;

}BUSCAN_data_t15_t;

//ID 1293
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t16_t;

//ID 1294
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t17_t;

//ID 1362
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t18_t;

//ID 1416
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t19_t;

//ID 1426
typedef struct
{
    uint32_t timestamp;
    uint8_t limpiaparabrisas;

}BUSCAN_data_t20_t;

//ID 1544
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t21_t;

//ID 1554
typedef struct
{
    uint32_t timestamp;
    enum estado_luces luces;
    enum estado_luces_antiniebla luces_antiniebla;
    enum estado_intermitente intermitente;
    double nivel_combustible;

}BUSCAN_data_t22_t;

//ID 1928
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t23_t;

//ID 1929
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t24_t;

//ID 1933
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t25_t;

//ID 1938
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t26_t;

//ID 2018
typedef struct
{
    uint32_t timestamp;
    double temp_ext;

}BUSCAN_data_t27_t;

//ID 2034
typedef struct
{
    uint32_t timestamp;

}BUSCAN_data_t28_t;


typedef struct
{
    BUSCAN_data_t0_t data_t0;
    BUSCAN_data_t1_t data_t1;
    BUSCAN_data_t2_t data_t2;
    BUSCAN_data_t3_t data_t3;
    BUSCAN_data_t4_t data_t4;
    BUSCAN_data_t5_t data_t5;
    BUSCAN_data_t6_t data_t6;
    BUSCAN_data_t7_t data_t7;
    BUSCAN_data_t8_t data_t8;
    BUSCAN_data_t9_t data_t9;
    BUSCAN_data_t10_t data_t10;
    BUSCAN_data_t11_t data_t11;
    BUSCAN_data_t12_t data_t12;
    BUSCAN_data_t13_t data_t13;
    BUSCAN_data_t14_t data_t14;
    BUSCAN_data_t15_t data_t15;
    BUSCAN_data_t16_t data_t16;
    BUSCAN_data_t17_t data_t17;
    BUSCAN_data_t18_t data_t18;
    BUSCAN_data_t19_t data_t19;
    BUSCAN_data_t20_t data_t20;
    BUSCAN_data_t21_t data_t21;
    BUSCAN_data_t22_t data_t22;
    BUSCAN_data_t23_t data_t23;
    BUSCAN_data_t24_t data_t24;
    BUSCAN_data_t25_t data_t25;
    BUSCAN_data_t26_t data_t26;
    BUSCAN_data_t27_t data_t27;
    BUSCAN_data_t28_t data_t28;
}BUSCAN_crude_data_t;

typedef struct
{
    double vdiKPH;
    double vddKPH;
    double vtiKPH;
    double vtdKPH;

    double vdiMPS;
    double vddMPS;
    double vtiMPS;
    double vtdMPS;

    double vdKPH;
    double vtKPH;

    double vdMPS;
    double vtMPS;

    double vKPH;
    double vMPS;
    uint32_t timestamp;
}C4speed_t;

typedef struct
{
    double gradosVolante;
    double velocidadVolante;
    int8_t sentidoVolante;
    uint32_t timestamp;
}C4steer_t;

typedef struct
{
    uint32_t timestamp;
    uint8_t freno_servicio;
    double intensidad_freno;
}C4brake_t;

typedef struct
{
    uint32_t timestamp;
    double velKMH;
    double accMPS2;
    double distRecorridaCiclicaM;
}C4accelDist_t;

//Estructura de datos BUSCAN en Bruto
//_________________________________________________________________________________________________

struct _BUSCAN_raw_data_t
{
    uint32_t id;
    unsigned char D[8];
    uint8_t dlc;
    uint8_t packed1;
    uint8_t packed2;
    uint8_t packed3;
    uint32_t flag;
    uint32_t time;
} __attribute__((__packed__));
typedef struct _BUSCAN_raw_data_t BUSCAN_raw_data_t;

typedef struct
{
    // BUSCAN_crude_data_t        crudeData;
    BUSCAN_raw_data_t       data_raw[NUM_IDS]; //Estructuras con datos en bruto de cada ID
    canHandle               h;

    pthread_t               BUSCAN_thread;
    bool                    m_busCAN_thread_run = false;

    sem_t                   speed_control_thread_semaphore;
    sem_t                   steering_wheel_control_thread_semaphore;
    sem_t                   brake_control_thread_semaphore;

    pthread_mutex_t*        p_BUSCAN_semaphore;

    C4speed_t               *C4speeds;
    C4steer_t               *C4steer;
    C4brake_t               *C4brake;
    C4accelDist_t           *C4accelDist;
}BUSCAN_t;


class BUSCAN
{
private:
    static void* read_BUSCAN(void *p);
    void get_BUSCAN_ID(uint32_t *id, int8_t *id_orden);
    void parse_BUSCAN_ID(BUSCAN_crude_data_t *crudeDataBUSCAN, uint32_t *id, unsigned char *msg, uint8_t *dlc, uint32_t *flag, uint32_t *time);
    BUSCAN_t* m_dataBUSCAN;
    sem_t*      m_sem_HLC;

public:
    BUSCAN(sem_t *sem_HLC);
    ~BUSCAN();
    int init_BUSCAN(int bitrate);

    double getSpeedMPS_Unblocking(C4speed_t **C4speeds);
    double getSpeedMPS_Blocking(C4speed_t **C4speeds);

    double getBrakeForce_Unblocking(C4brake_t **C4brake);
    double getBrakeForce_Blocking(C4brake_t **C4brake);

    double getSteeringWheelPosition_Unblocking(C4steer_t **C4steer);
    double getSteeringWheelPosition_Blocking(C4steer_t **C4steer);

    bool m_BUSCAN_initialized;
};
#endif // BUSCAN_H
