#include "BUSCAN.hpp"


BUSCAN::BUSCAN(sem_t* sem_HLC)
{
    m_sem_HLC = sem_HLC;
    m_dataBUSCAN = (BUSCAN_t *) malloc(sizeof(BUSCAN_t));
    m_dataBUSCAN->p_BUSCAN_semaphore = (pthread_mutex_t *) malloc(sizeof(pthread_mutex_t));
    m_dataBUSCAN->C4speeds = (C4speed_t *) malloc(sizeof(C4speed_t));
    m_dataBUSCAN->C4steer = (C4steer_t *) malloc(sizeof(C4steer_t));
    m_dataBUSCAN->C4brake = (C4brake_t *) malloc(sizeof(C4brake_t));
    m_dataBUSCAN->C4gear = (BUSCAN_data_t15_t *) malloc(sizeof(BUSCAN_data_t15_t));
    m_dataBUSCAN->C4throttle = (BUSCAN_data_t0_t *) malloc(sizeof(BUSCAN_data_t0_t));
    m_dataBUSCAN->C4accelDist = (C4accelDist_t *) malloc(sizeof(C4accelDist_t));
    m_BUSCAN_initialized = false;
}

BUSCAN::~BUSCAN()
{
    if (m_BUSCAN_initialized)
    {
        m_dataBUSCAN->m_busCAN_thread_run = false;
        pthread_join(m_dataBUSCAN->BUSCAN_thread, NULL);
        canClose(m_dataBUSCAN->h);
        sem_destroy(&(m_dataBUSCAN->speed_control_thread_semaphore));
        sem_destroy(&(m_dataBUSCAN->steering_wheel_control_thread_semaphore));
        sem_destroy(&(m_dataBUSCAN->brake_control_thread_semaphore));
        pthread_mutex_destroy(m_dataBUSCAN->p_BUSCAN_semaphore);
        m_BUSCAN_initialized = false;
    }

    free(m_dataBUSCAN->C4speeds);
    free(m_dataBUSCAN->C4steer);
    free(m_dataBUSCAN->C4brake);
    free(m_dataBUSCAN->C4accelDist);
    free(m_dataBUSCAN->C4gear);
    free(m_dataBUSCAN->C4throttle);
    free(m_dataBUSCAN->p_BUSCAN_semaphore);
    free(m_dataBUSCAN);
}

int BUSCAN::init_BUSCAN(int bitrate)
{
    /* Localizar Canal KVASER */
    int chanCount = 0;
    //int stat;
    int channel = -1;

    //stat = canGetNumberOfChannels(&chanCount);
    canGetNumberOfChannels(&chanCount);

    if (chanCount < 0 || chanCount > 64)
    {
        printf("ChannelCount = %d but I don't believe it.\n", chanCount);
        exit(1);
    }
    else
    {
        channel=chanCount - 1; //Abre el ultimo canal que ha detectado

        /* Open channels, parameters and go on bus */
        m_dataBUSCAN->h = canOpenChannel(channel, canWANT_EXCLUSIVE | canWANT_EXTENDED);

        if (m_dataBUSCAN->h < 0)
        {
            printf("canOpenChannel %d failed\n", channel);
            return 0;
        }
        else
        {
            canSetBusParams(m_dataBUSCAN->h, bitrate, 4, 3, 1, 1, 0);
            canSetBusOutputControl(m_dataBUSCAN->h, canDRIVER_NORMAL);

            canBusOn(m_dataBUSCAN->h);

            sem_init(&(m_dataBUSCAN->speed_control_thread_semaphore),0, 0);
            sem_init(&(m_dataBUSCAN->steering_wheel_control_thread_semaphore),0, 0);
            sem_init(&(m_dataBUSCAN->brake_control_thread_semaphore),0, 0);

            //pthread_mutex_t mutex_lock;
            //m_dataBUSCAN->p_BUSCAN_semaphore = &mutex_lock;
            pthread_mutex_init(m_dataBUSCAN->p_BUSCAN_semaphore, NULL);
            pthread_create(&(m_dataBUSCAN->BUSCAN_thread), NULL, BUSCAN::read_BUSCAN, this);

            m_BUSCAN_initialized = true;
            return 1;
        }
    }
}

void* BUSCAN::read_BUSCAN(void *p)
{
    BUSCAN* p_this = (BUSCAN*) p;
    BUSCAN_t* dataBUSCAN = p_this->m_dataBUSCAN;

    BUSCAN_crude_data_t crudeDataBUSCAN;

    int ret = -1;
    long id;
    unsigned char msg[8];
    unsigned int dlc;
    unsigned int flag;
    unsigned long time;
    int8_t id_orden;
    //Compatibilidad con 64 bit
    uint32_t time32, id32;
    uint8_t dlc8;

    p_this->m_dataBUSCAN->m_busCAN_thread_run = true;

    // Inicializando estructura de datos BUSCAN
    memset(dataBUSCAN->data_raw,0,(NUM_IDS*sizeof(BUSCAN_raw_data_t)));

    while(p_this->m_dataBUSCAN->m_busCAN_thread_run)
    {
        //LEER DEL BUS CAN
        ret = canReadWait(dataBUSCAN->h, &id, &msg, &dlc, &flag, &time, -1);

        switch (ret)
        {
        case canOK: //comando valido (ret=canOk=0)

            //printf("lee_BUSCAN_while 2\n");


            sem_post(p_this->m_sem_HLC);
            pthread_mutex_unlock(dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */
            p_this->m_id = id;
            memcpy(&(p_this->m_msg), msg, sizeof(msg));
            pthread_mutex_unlock(dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */



            if(id == 773 || id == 973 || id == 909 || id==1101 || id==781 || id==1161 || id==520)
            {

                pthread_mutex_lock(dataBUSCAN->p_BUSCAN_semaphore); //Protect data from being read
                id32 = id;
                dlc8 = dlc;
                time32 = time;

                //OBTENER_un ID de BUSCAN______________________________________________
                p_this->get_BUSCAN_ID(&id32, &id_orden);
                if (id_orden == -1)
                {
                    printf("error\n");
                    exit(0);
                }

                //GUARDAR DATOS EN BRUTO______________________________________
                dataBUSCAN->data_raw[id_orden].id=id32;
                memcpy(dataBUSCAN->data_raw[id_orden].D, msg, sizeof(msg)); // Copia el msg a la estructura de datos en bruto

                dataBUSCAN->data_raw[id_orden].dlc=dlc8;
                dataBUSCAN->data_raw[id_orden].flag=flag;
                dataBUSCAN->data_raw[id_orden].time=time32;


                if(id == 520) //ID RPM, FRENO y PEDAL ACELERADOR
                {
                    p_this->parse_BUSCAN_ID(&crudeDataBUSCAN, &id32, msg, &dlc8, &flag, &time32);

                    dataBUSCAN->C4throttle->rpm = crudeDataBUSCAN.data_t0.rpm;
                    dataBUSCAN->C4throttle->pedal_acelerador = crudeDataBUSCAN.data_t0.pedal_acelerador;

                    dataBUSCAN->C4throttle->timestamp = crudeDataBUSCAN.data_t0.timestamp;

                }
                else if(id == 773)  // Msg del volante
                {
                    p_this->parse_BUSCAN_ID(&crudeDataBUSCAN, &id32, msg, &dlc8, &flag, &time32);

                    dataBUSCAN->C4steer->gradosVolante = crudeDataBUSCAN.data_t2.volante_angulo;
                    dataBUSCAN->C4steer->sentidoVolante = crudeDataBUSCAN.data_t2.volante_sentido;
                    dataBUSCAN->C4steer->velocidadVolante = crudeDataBUSCAN.data_t2.volante_velocidad;

                    dataBUSCAN->C4steer->timestamp = crudeDataBUSCAN.data_t2.timestamp;

                }
                else if(id == 781)   // Msg de la velocidad 4 ruedas a 10ms
                {
                    p_this->parse_BUSCAN_ID(&crudeDataBUSCAN, &id32, msg, &dlc8, &flag, &time32);
                    
                    dataBUSCAN->C4speeds->vddKPH = crudeDataBUSCAN.data_t3.vrdd;
                    dataBUSCAN->C4speeds->vdiKPH = crudeDataBUSCAN.data_t3.vrdi;
                    dataBUSCAN->C4speeds->vtdKPH = crudeDataBUSCAN.data_t3.vrtd;
                    dataBUSCAN->C4speeds->vtiKPH = crudeDataBUSCAN.data_t3.vrti;

                    dataBUSCAN->C4speeds->vddMPS = dataBUSCAN->C4speeds->vddKPH/3.6;
                    dataBUSCAN->C4speeds->vdiMPS = dataBUSCAN->C4speeds->vdiKPH/3.6;
                    dataBUSCAN->C4speeds->vtdMPS = dataBUSCAN->C4speeds->vtdKPH/3.6;
                    dataBUSCAN->C4speeds->vtiMPS = dataBUSCAN->C4speeds->vtiKPH/3.6;

                    dataBUSCAN->C4speeds->vdKPH = (crudeDataBUSCAN.data_t3.vrdd + crudeDataBUSCAN.data_t3.vrdi)/2.0;
                    dataBUSCAN->C4speeds->vtKPH = (crudeDataBUSCAN.data_t3.vrtd + crudeDataBUSCAN.data_t3.vrti)/2.0;

                    dataBUSCAN->C4speeds->vdMPS = dataBUSCAN->C4speeds->vdKPH/3.6;
                    dataBUSCAN->C4speeds->vtMPS = dataBUSCAN->C4speeds->vtKPH/3.6;

                    dataBUSCAN->C4speeds->vKPH = (dataBUSCAN->C4speeds->vdKPH + dataBUSCAN->C4speeds->vtKPH)/2.0;

                    dataBUSCAN->C4speeds->vMPS = dataBUSCAN->C4speeds->vKPH/3.6;

                    dataBUSCAN->C4speeds->timestamp = crudeDataBUSCAN.data_t3.timestamp;
                }
               else if (id == 909)
                {
                    p_this->parse_BUSCAN_ID(&crudeDataBUSCAN, &id32, msg, &dlc8, &flag, &time32);

                    dataBUSCAN->C4accelDist->accMPS2 = crudeDataBUSCAN.data_t7.acc;
                    dataBUSCAN->C4accelDist->velKMH = crudeDataBUSCAN.data_t7.vel;
                    dataBUSCAN->C4accelDist->distRecorridaCiclicaM = crudeDataBUSCAN.data_t7.distancia;

                    dataBUSCAN->C4accelDist->timestamp = crudeDataBUSCAN.data_t7.timestamp;
                }
                 else if (id == 973)
                {
                    p_this->parse_BUSCAN_ID(&crudeDataBUSCAN, &id32, msg, &dlc8, &flag, &time32);

                    dataBUSCAN->C4brake->intensidad_freno = crudeDataBUSCAN.data_t8.freno_servicio_intensidad;
                    dataBUSCAN->C4brake->freno_servicio = crudeDataBUSCAN.data_t8.freno_servicio;

                    dataBUSCAN->C4brake->timestamp = crudeDataBUSCAN.data_t8.freno_servicio;
                }
                else if(id == 1161) //ID MARCHAS
                {
                    p_this->parse_BUSCAN_ID(&crudeDataBUSCAN, &id32, msg, &dlc8, &flag, &time32);

                    dataBUSCAN->C4gear->marchas = crudeDataBUSCAN.data_t15.marchas;
                    dataBUSCAN->C4gear->modo_cambio = crudeDataBUSCAN.data_t15.modo_cambio;

                    dataBUSCAN->C4gear->timestamp = crudeDataBUSCAN.data_t15.timestamp;

                }

                pthread_mutex_unlock(dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */

                // Control con ciclo de 10ms y v4Ruedas
                if(id == 781)   // Velocidad
                {
                    sem_post(&(dataBUSCAN->speed_control_thread_semaphore));
                }

                if(id == 773)   // Volante
                    sem_post(&(dataBUSCAN->steering_wheel_control_thread_semaphore));

                if(id == 973)   // Freno
                    sem_post(&(dataBUSCAN->brake_control_thread_semaphore));
            }
            break;
        case canERR_NOMSG:
            perror("canERR_NOMSG error");
            break;
        default:
            perror("canReadBlock EError");
            break;
        } //End switch(ret)
    } //End while(1)
}

void BUSCAN::get_BUSCAN_ID(uint32_t *id, int8_t *id_orden)
{
    switch (*id) //Orden del ID para guardar los datos en bruto
    {
        case 520:  *id_orden=0;	 break;
        case 761:  *id_orden=1;	 break;
        case 773:  *id_orden=2;	 break;
        case 781:  *id_orden=3;  break;
        case 840:  *id_orden=4;  break;
        case 841:  *id_orden=5;  break;
        case 845:  *id_orden=6;  break;
        case 909:  *id_orden=7;  break;
        case 973:  *id_orden=8;  break;
        case 1037: *id_orden=9;  break;
        case 1042: *id_orden=10; break;
        case 1074: *id_orden=11; break;
        case 1101: *id_orden=12; break;
        case 1128: *id_orden=13; break;
        case 1160: *id_orden=14; break;
        case 1161: *id_orden=15; break;
        case 1293: *id_orden=16; break;
        case 1294: *id_orden=17; break;
        case 1362: *id_orden=18; break;
        case 1416: *id_orden=19; break;
        case 1426: *id_orden=20; break;
        case 1544: *id_orden=21; break;
        case 1554: *id_orden=22; break;
        case 1928: *id_orden=23; break;
        case 1929: *id_orden=24; break;
        case 1933: *id_orden=25; break;
        case 1938: *id_orden=26; break;
        case 2018: *id_orden=27; break;
        case 2034: *id_orden=28; break;
        default:   *id_orden=-1; break;
    } //End Switch (id)
}

void BUSCAN::parse_BUSCAN_ID(BUSCAN_crude_data_t *crudeDataBUSCAN, uint32_t *id, unsigned char *msg, uint8_t *dlc, uint32_t *flag, uint32_t *time)

{
    unsigned char intermitentes_aux;
    int freno_aux;

    if(0)
    {
        flag=flag;
        dlc=dlc;
    }

    switch (*id)
    {
    case 520: //ID RPM, FRENO y PEDAL ACELERADOR
        crudeDataBUSCAN->data_t0.timestamp = *time;
        //RPM
        crudeDataBUSCAN->data_t0.rpm=((double)(msg[0]*256+msg[1])/8);
        //PEDAL ACELERADOR
        crudeDataBUSCAN->data_t0.pedal_acelerador= (double)(msg[3]*0.5);
        break;
    case 761: //ID
        crudeDataBUSCAN->data_t1.timestamp = *time;
        break;
    case 773: //ID VOLANTE
        crudeDataBUSCAN->data_t2.timestamp = *time;
        //VOLANTE ANGULO
        /*			if (msg[0] > 0x15) //Angulo sentido horario
                    dataBUSCAN->data_t2.volante_angulo=(double)((65535-((msg[0]*256)+msg[1]))*0.1634);  //Grados
                else  //Angulo sentido antihorario
                    dataBUSCAN->data_t2.volante_angulo=(double)(((msg[0]*256)+msg[1])*-0.1634);  //Grados
    */
        crudeDataBUSCAN->data_t2.volante_angulo=(double)(((msg[0]*256)+msg[1]));
        if (crudeDataBUSCAN->data_t2.volante_angulo>7200.0)
            crudeDataBUSCAN->data_t2.volante_angulo -= 65535.0;
        crudeDataBUSCAN->data_t2.volante_angulo=(-crudeDataBUSCAN->data_t2.volante_angulo*0.1);

        /* d0 = xlEvent.tagData.msg.data[0];
         d1 = xlEvent.tagData.msg.data[1];
         angle = d0*256+d1;
         if (angle>7200)
         angle -= 65535;
         Steering_Angle=(-angle*0.1);
         //offset porque el sensor se ha soltado +35�
         *g_th->Result=Steering_Angle;*/

        //VOLANTE VELOCIDAD
        crudeDataBUSCAN->data_t2.volante_velocidad = (msg[2]*4); //grados/s
        //VOLANTE SENTIDO
        if (crudeDataBUSCAN->data_t2.volante_velocidad > 0)
        {
            if (msg[3] > 0x7F) //el bit msg[3].7 esta a 1
                crudeDataBUSCAN->data_t2.volante_sentido = 1; //sent horario
            else
                crudeDataBUSCAN->data_t2.volante_sentido = -1; //sent antihorario
        }
        else
            crudeDataBUSCAN->data_t2.volante_sentido = 0; //parado
        break;
    case 781: //ID 4 VELOCIDADES
        crudeDataBUSCAN->data_t3.timestamp = *time;
        crudeDataBUSCAN->data_t3.vrdi = ((double)(msg[0]*256+msg[1])*0.01);
        crudeDataBUSCAN->data_t3.vrdd = ((double)(msg[2]*256+msg[3])*0.01);
        crudeDataBUSCAN->data_t3.vrti = ((double)(msg[4]*256+msg[5])*0.01);
        crudeDataBUSCAN->data_t3.vrtd = ((double)(msg[6]*256+msg[7])*0.01);

        break;
    case 840: //ID Algun sensor entrada de aire??
        crudeDataBUSCAN->data_t4.timestamp = *time;
        break;
    case 841: //ID MARCHAS
        crudeDataBUSCAN->data_t5.timestamp = *time;
        break;
    case 845: //ID ESP Interruptor ON/OFF y Activado si salta
        crudeDataBUSCAN->data_t6.timestamp = *time;
        //ESP
        if(msg[0] > 0x7F) //el bit msg[0].7 esta a 1
            crudeDataBUSCAN->data_t6.ESP_activado = 0; // si msg[0]=0x8X: Interruptor ESP ON/OFF = OFF
        else
            crudeDataBUSCAN->data_t6.ESP_activado = 1; // si msg[0]=0x0X: Interruptor ESP ON/OFF = ON
        if(msg[0] == 0x45)
            crudeDataBUSCAN->data_t6.ESP_disparado = 1; // ESP Activado (Ha saltado)
        else
            crudeDataBUSCAN->data_t6.ESP_disparado = 0; // ESP Desactivado
        break;
    case 909: //ID ABS VEHICLE DINAMICS: Vel en delanteras, Distancia traseras y Aceleracion longitudinal
        crudeDataBUSCAN->data_t7.timestamp = *time;

        crudeDataBUSCAN->data_t7.vel=((double)(msg[0]*256+msg[1])*0.01); // km/h (0.01)
        crudeDataBUSCAN->data_t7.distancia=((double)(msg[2]*256+msg[3])*0.1); // m (0.1)
        crudeDataBUSCAN->data_t7.acc=((double)(msg[4]*0.08)-14); // m/s2 (0.08)
        break;
    case 973: //ID FRENO ACCION E INTENSIDAD
        crudeDataBUSCAN->data_t8.timestamp = *time;
        if (msg[4] == 0x40)
            crudeDataBUSCAN->data_t8.freno_servicio = 1;
        else
            crudeDataBUSCAN->data_t8.freno_servicio = 0;
        freno_aux = msg[6] >> 4; //quedarse con la parte alta de D6
        //crudeDataBUSCAN->data_t8.freno_servicio_intensidad = (double)((((msg[5]*16) + freno_aux)-552)/14.47);
        crudeDataBUSCAN->data_t8.freno_servicio_intensidad = (double)((((msg[5]*16) + freno_aux)-552));
        break;
    case 1037: //ID Distancia
        crudeDataBUSCAN->data_t9.timestamp = *time;
        break;
    case 1042: //ID FRENO SERVICIO Y DE ESTACIONAMIENTO, PUERTAS ABIERTAS , TEMP MOTOR
        crudeDataBUSCAN->data_t10.timestamp = *time;
        //FRENO ESTACIONAMIENTO
        if ((msg[0] & 0x0F) == 0x00) //freno estacionamiento suelto
            crudeDataBUSCAN->data_t10.freno_estacionamiento=0;
        else
            crudeDataBUSCAN->data_t10.freno_estacionamiento=1;
        //PUERTAS ABIERTAS
        crudeDataBUSCAN->data_t10.puertas_abiertas = msg[6] >> 3;
        //TEMPERATURA MOTOR
        crudeDataBUSCAN->data_t10.temp_motor = (msg[5]-45);
        break;
    case 1074: //ID
        crudeDataBUSCAN->data_t11.timestamp = *time;
        break;
    case 1101: //ID VELOCIDAD ABS MEDIA delanteras y traseras por separado
        crudeDataBUSCAN->data_t12.timestamp = *time;
        crudeDataBUSCAN->data_t12.vel_media_ruedas_delanteras = ((double)(msg[0]*256+msg[1])*0.01);
        crudeDataBUSCAN->data_t12.vel_rueda_trasera_izq = ((double)(msg[2]*256+msg[3])*0.01);
        crudeDataBUSCAN->data_t12.vel_rueda_trasera_dcha = ((double)(msg[4]*256+msg[5])*0.01);
        crudeDataBUSCAN->data_t12.reservado = ((double)(msg[6]*256+msg[7])*0.01);
        break;
    case 1128: //ID
        crudeDataBUSCAN->data_t13.timestamp = *time;
        break;
    case 1160: //ID TEMPERATURA MOTOR
        crudeDataBUSCAN->data_t14.timestamp = *time;
        break;
    case 1161: //ID MARCHAS
        crudeDataBUSCAN->data_t15.timestamp = *time;
        //Modo cambio marchas
        if(msg[1]==12)
        {crudeDataBUSCAN->data_t15.modo_cambio = invierno;}
        else if (msg[1]==4)
        {crudeDataBUSCAN->data_t15.modo_cambio = sport;}
        else
        {crudeDataBUSCAN->data_t15.modo_cambio = normal;}
        //Marchas


        switch (msg[0])
        {
        case 0xA0: crudeDataBUSCAN->data_t15.marchas = P;	break;
        case 0x91: crudeDataBUSCAN->data_t15.marchas = R;	break;
        case 0xA2: crudeDataBUSCAN->data_t15.marchas = N;	break;
        case 0x13: crudeDataBUSCAN->data_t15.marchas = D1;	break;
        case 0x23: crudeDataBUSCAN->data_t15.marchas = D2;	break;
        case 0x33: crudeDataBUSCAN->data_t15.marchas = D3;	break;
        case 0x43: crudeDataBUSCAN->data_t15.marchas = D4;	break;
        case 0x17: crudeDataBUSCAN->data_t15.marchas = M1;	break;
        case 0x26: crudeDataBUSCAN->data_t15.marchas = M2;	break;
        case 0x35: crudeDataBUSCAN->data_t15.marchas = M3;	break;
        case 0x44: crudeDataBUSCAN->data_t15.marchas = M4;	break;
        default:   crudeDataBUSCAN->data_t15.marchas = N;	break;
        }

        //printf("marcha=%x data_t15.marchas= %d\n", msg[0], crudeDataBUSCAN->data_t15.marchas);
        break;
    case 1293: //ID DISTANCIAS en D1D2 y D2D3
        crudeDataBUSCAN->data_t16.timestamp = *time;
        break;
    case 1294: //ID
        crudeDataBUSCAN->data_t17.timestamp = *time;
        break;
    case 1362: //ID TIMER 1 seg.
        crudeDataBUSCAN->data_t18.timestamp = *time;
        break;
    case 1416: //ID
        crudeDataBUSCAN->data_t19.timestamp = *time;
        break;
    case 1426: //ID Limpiaparabrisas delantero.
        crudeDataBUSCAN->data_t20.timestamp = *time;
        //LIMPIAPARABRISAS
        switch (msg[2])
        {
        case 0x00: crudeDataBUSCAN->data_t20.limpiaparabrisas = 0; break;
        case 0x10: crudeDataBUSCAN->data_t20.limpiaparabrisas = 1; break;
        case 0x20: crudeDataBUSCAN->data_t20.limpiaparabrisas = 2; break;
        case 0x40: crudeDataBUSCAN->data_t20.limpiaparabrisas = 3; break;
        case 0x08: crudeDataBUSCAN->data_t20.limpiaparabrisas = 8; break;
        default: 				     break;
        }
        break;
    case 1544: //ID 3 TRAMAS ??
        crudeDataBUSCAN->data_t21.timestamp = *time;
        break;
    case 1554: //ID LUCES, INTERMITENTES, NIVEL COMBUSTIBLE
        crudeDataBUSCAN->data_t22.timestamp = *time;
        //LUCES
        if ((msg[1]&0x04) > 0) //largas activadas
        {
            if ((msg[1]&0x02) > 0) //largas + cortas + posicion activadas
                crudeDataBUSCAN->data_t22.luces = luces_largas;
            else if ((msg[1]&0x01) > 0) //largas + posicion activadas
                crudeDataBUSCAN->data_t22.luces = luces_rafaga_posicion;
            else //solo largas activadas = rafaga
                crudeDataBUSCAN->data_t22.luces = luces_rafaga;
        }
        else if ((msg[1]&0x02) > 0) //cortas activadas
            crudeDataBUSCAN->data_t22.luces = luces_cortas;
        else if ((msg[1]&0x01) > 0) //posicion activadas
            crudeDataBUSCAN->data_t22.luces = luces_posicion;
        else
            crudeDataBUSCAN->data_t22.luces = luces_off;

        if ((msg[1]&0x08) > 0) //antinieblas activadas
            crudeDataBUSCAN->data_t22.luces_antiniebla = antiniebla_on;
        else
            crudeDataBUSCAN->data_t22.luces_antiniebla = antiniebla_off;
        //INTEMITENTES
        intermitentes_aux = msg[1]&0xF0;
        switch (intermitentes_aux)
        {
        case 0x20:
            crudeDataBUSCAN->data_t22.intermitente = intermitente_off;
            break;
        case 0xA0:
            crudeDataBUSCAN->data_t22.intermitente = intermitente_izquierda;
            break;
        case 0x60:
            crudeDataBUSCAN->data_t22.intermitente = intermitente_derecha;
            break;
        default:
            break;
        }
        //NIVEL COMBUSTIBLE
        crudeDataBUSCAN->data_t22.nivel_combustible = (msg[3]*0.5);

        /*//CONSUMO MEDIO E INSTANTANEO
                if (nivel_comb_inicial == 0.0)
                {
                  //printf("nivel_comb_inicial ==%f",crudeDataBUSCAN->data_t22.nivel_combustible );
                    nivel_comb_inicial = crudeDataBUSCAN->data_t22.nivel_combustible;
                    nivel_comb_anterior = crudeDataBUSCAN->data_t22.nivel_combustible;
                }
                if (nivel_comb_anterior != crudeDataBUSCAN->data_t22.nivel_combustible);
                {
                crudeDataBUSCAN->data_t22.consumo_medio = (nivel_comb_inicial - crudeDataBUSCAN->data_t22.nivel_combustible)/(crudeDataBUSCAN->distancia*100000);
                crudeDataBUSCAN->consumo_instantaneo = (nivel_comb_anterior - crudeDataBUSCAN->nivel_combustible) /(crudeDataBUSCAN->distancia - distancia_comb_anterior);
                distancia_comb_anterior = crudeDataBUSCAN->distancia; //Actualizar la distancia
                }*/

        break;
    case 1928: //ID
        crudeDataBUSCAN->data_t23.timestamp = *time;
        break;
    case 1929: //ID
        crudeDataBUSCAN->data_t24.timestamp = *time;
        break;
    case 1933: //ID
        crudeDataBUSCAN->data_t25.timestamp = *time;
        break;
    case 1938: //ID
        crudeDataBUSCAN->data_t26.timestamp = *time;
        break;
    case 2018: //ID TEMPERATURA EXTERIOR
        crudeDataBUSCAN->data_t27.timestamp = *time;
        //TEMPERATURA EXTERIOR
        if (msg[2]==0x8D)
            crudeDataBUSCAN->data_t27.temp_ext=(double)((msg[3]-80)*0.5);
        //else
        //	crudeDataBUSCAN->data_t27.temp_ext=-100; //Para evitar valores extraños al principio y evitar valores incorrectos cuando no hay lectura
        break;
    case 2034: //ID TEMPERATURA EXTERIOR
        crudeDataBUSCAN->data_t28.timestamp = *time;
        break;
    default:
        break;
        //break;
    } //End Switch (id)
}

double BUSCAN::getThrottle_Unblocking()
{
    double ret = 0.0;
    if(m_BUSCAN_initialized)
    {
        pthread_mutex_lock(m_dataBUSCAN->p_BUSCAN_semaphore); //Protect data from being read
        ret = (double)m_dataBUSCAN->C4throttle->pedal_acelerador;
        pthread_mutex_unlock(m_dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */
    }
    return  ret;
}


double BUSCAN::getGear_Unblocking()
{
    double ret = 1.0;
    if(m_BUSCAN_initialized)
    {
        pthread_mutex_lock(m_dataBUSCAN->p_BUSCAN_semaphore); //Protect data from being read
        ret = (double)m_dataBUSCAN->C4gear->marchas;
        pthread_mutex_unlock(m_dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */
    }
    return  ret;
}


double BUSCAN::getSpeedMPS_Unblocking(C4speed_t **C4speeds)
{
    double ret=-1000.0;
    if(m_BUSCAN_initialized)
    {
        pthread_mutex_lock(m_dataBUSCAN->p_BUSCAN_semaphore); //Protect data from being read
        ret = m_dataBUSCAN->C4speeds->vMPS;
        *C4speeds = m_dataBUSCAN->C4speeds;
        pthread_mutex_unlock(m_dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */
    }
    return  ret;
}

double BUSCAN::getSpeedMPS_Blocking(C4speed_t **C4speeds)
{
    sem_wait(&(m_dataBUSCAN->speed_control_thread_semaphore));

    double ret=-1000.0;

    if(m_BUSCAN_initialized)
    {
        pthread_mutex_lock(m_dataBUSCAN->p_BUSCAN_semaphore); //Protect data from being read
        ret = m_dataBUSCAN->C4speeds->vMPS;
        *C4speeds = m_dataBUSCAN->C4speeds;
        pthread_mutex_unlock(m_dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */
    }
    return  ret;
}

double BUSCAN::getBrakeForce_Unblocking(C4brake_t **C4brake)
{
    double ret=-1000.0;

    if(m_BUSCAN_initialized)
    {
        pthread_mutex_lock(m_dataBUSCAN->p_BUSCAN_semaphore); //Protect data from being read
        ret = m_dataBUSCAN->C4brake->intensidad_freno;
        *C4brake = m_dataBUSCAN->C4brake;
        pthread_mutex_unlock(m_dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */
    }

    return  ret;
}

double BUSCAN::getBrakeForce_Blocking(C4brake_t **C4brake)
{
    sem_wait(&(m_dataBUSCAN->brake_control_thread_semaphore));

    double ret=-1000.0;

    if(m_BUSCAN_initialized)
    {
        pthread_mutex_lock(m_dataBUSCAN->p_BUSCAN_semaphore); //Protect data from being read
        ret = m_dataBUSCAN->C4brake->intensidad_freno;
        *C4brake = m_dataBUSCAN->C4brake;
        pthread_mutex_unlock(m_dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */
    }

    return  ret;
}

double BUSCAN::getSteeringWheelPosition_Unblocking(C4steer_t **C4steer)
{
    double ret=-1000.0;

    if(m_BUSCAN_initialized)
    {
        pthread_mutex_lock(m_dataBUSCAN->p_BUSCAN_semaphore); //Protect data from being read
        ret = m_dataBUSCAN->C4steer->gradosVolante;
        *C4steer = m_dataBUSCAN->C4steer;
        pthread_mutex_unlock(m_dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */
    }

    return  ret;
}

double BUSCAN::getSteeringWheelPosition_Blocking(C4steer_t **C4steer)
{
    sem_wait(&(m_dataBUSCAN->steering_wheel_control_thread_semaphore));

    double ret=-1000.0;

    if(m_BUSCAN_initialized)
    {
        pthread_mutex_lock(m_dataBUSCAN->p_BUSCAN_semaphore); //Protect data from being read
        ret = m_dataBUSCAN->C4steer->gradosVolante;
        *C4steer = m_dataBUSCAN->C4steer;
        pthread_mutex_unlock(m_dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */
    }

    return  ret;
}

int BUSCAN::get_raw_msg_id()
{
    pthread_mutex_lock(m_dataBUSCAN->p_BUSCAN_semaphore); //Protect data from being read
    return m_id;
    pthread_mutex_unlock(m_dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */
}

void BUSCAN::get_raw_msg(unsigned char *p)
{
    pthread_mutex_lock(m_dataBUSCAN->p_BUSCAN_semaphore); //Protect data from being read
    memcpy(p, &m_msg, sizeof(m_msg));
    pthread_mutex_unlock(m_dataBUSCAN->p_BUSCAN_semaphore); /* Release data for reading */
}
