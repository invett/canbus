#include "BUSCAN.hpp"


// void BUSCAN::parse_BUSCAN_ID(BUSCAN_crude_data_t *crudeDataBUSCAN, uint32_t *id, unsigned char *msg, uint8_t *dlc, uint32_t *flag, uint32_t *time)

// {
//     unsigned char intermitentes_aux;
//     int freno_aux;

//     if(0)
//     {
//         flag=flag;
//         dlc=dlc;
//     }

//     switch (*id)
//     {
//     case 520: //ID RPM, FRENO y PEDAL ACELERADOR
//         crudeDataBUSCAN->data_t0.timestamp = *time;
//         //RPM
//         crudeDataBUSCAN->data_t0.rpm=((double)(msg[0]*256+msg[1])/8);
//         //PEDAL ACELERADOR
//         crudeDataBUSCAN->data_t0.pedal_acelerador= (double)(msg[3]*0.5);
//         break;
//     case 761: //ID
//         crudeDataBUSCAN->data_t1.timestamp = *time;
//         break;
//     case 773: //ID VOLANTE
//         crudeDataBUSCAN->data_t2.timestamp = *time;
//         //VOLANTE ANGULO
//         /*			if (msg[0] > 0x15) //Angulo sentido horario
//                     dataBUSCAN->data_t2.volante_angulo=(double)((65535-((msg[0]*256)+msg[1]))*0.1634);  //Grados
//                 else  //Angulo sentido antihorario
//                     dataBUSCAN->data_t2.volante_angulo=(double)(((msg[0]*256)+msg[1])*-0.1634);  //Grados
//     */
//         crudeDataBUSCAN->data_t2.volante_angulo=(double)(((msg[0]*256)+msg[1]));
//         if (crudeDataBUSCAN->data_t2.volante_angulo>7200.0)
//             crudeDataBUSCAN->data_t2.volante_angulo -= 65535.0;
//         crudeDataBUSCAN->data_t2.volante_angulo=(-crudeDataBUSCAN->data_t2.volante_angulo*0.1);

//         /* d0 = xlEvent.tagData.msg.data[0];
//          d1 = xlEvent.tagData.msg.data[1];
//          angle = d0*256+d1;
//          if (angle>7200)
//          angle -= 65535;
//          Steering_Angle=(-angle*0.1);
//          //offset porque el sensor se ha soltado +35�
//          *g_th->Result=Steering_Angle;*/

//         //VOLANTE VELOCIDAD
//         crudeDataBUSCAN->data_t2.volante_velocidad = (msg[2]*4); //grados/s
//         //VOLANTE SENTIDO
//         if (crudeDataBUSCAN->data_t2.volante_velocidad > 0)
//         {
//             if (msg[3] > 0x7F) //el bit msg[3].7 esta a 1
//                 crudeDataBUSCAN->data_t2.volante_sentido = 1; //sent horario
//             else
//                 crudeDataBUSCAN->data_t2.volante_sentido = -1; //sent antihorario
//         }
//         else
//             crudeDataBUSCAN->data_t2.volante_sentido = 0; //parado
//         break;
//     case 781: //ID 4 VELOCIDADES
//         crudeDataBUSCAN->data_t3.timestamp = *time;
//         crudeDataBUSCAN->data_t3.vrdi = ((double)(msg[0]*256+msg[1])*0.01);
//         crudeDataBUSCAN->data_t3.vrdd = ((double)(msg[2]*256+msg[3])*0.01);
//         crudeDataBUSCAN->data_t3.vrti = ((double)(msg[4]*256+msg[5])*0.01);
//         crudeDataBUSCAN->data_t3.vrtd = ((double)(msg[6]*256+msg[7])*0.01);

//         break;
//     case 840: //ID Algun sensor entrada de aire??
//         crudeDataBUSCAN->data_t4.timestamp = *time;
//         break;
//     case 841: //ID MARCHAS
//         crudeDataBUSCAN->data_t5.timestamp = *time;
//         break;
//     case 845: //ID ESP Interruptor ON/OFF y Activado si salta
//         crudeDataBUSCAN->data_t6.timestamp = *time;
//         //ESP
//         if(msg[0] > 0x7F) //el bit msg[0].7 esta a 1
//             crudeDataBUSCAN->data_t6.ESP_activado = 0; // si msg[0]=0x8X: Interruptor ESP ON/OFF = OFF
//         else
//             crudeDataBUSCAN->data_t6.ESP_activado = 1; // si msg[0]=0x0X: Interruptor ESP ON/OFF = ON
//         if(msg[0] == 0x45)
//             crudeDataBUSCAN->data_t6.ESP_disparado = 1; // ESP Activado (Ha saltado)
//         else
//             crudeDataBUSCAN->data_t6.ESP_disparado = 0; // ESP Desactivado
//         break;
//     case 909: //ID ABS VEHICLE DINAMICS: Vel en delanteras, Distancia traseras y Aceleracion longitudinal
//         crudeDataBUSCAN->data_t7.timestamp = *time;

//         crudeDataBUSCAN->data_t7.vel=((double)(msg[0]*256+msg[1])*0.01); // km/h (0.01)
//         crudeDataBUSCAN->data_t7.distancia=((double)(msg[2]*256+msg[3])*0.1); // m (0.1)
//         crudeDataBUSCAN->data_t7.acc=((double)(msg[4]*0.08)-14); // m/s2 (0.08)
//         break;
//     case 973: //ID FRENO ACCION E INTENSIDAD
//         crudeDataBUSCAN->data_t8.timestamp = *time;
//         if (msg[4] == 0x40)
//             crudeDataBUSCAN->data_t8.freno_servicio = 1;
//         else
//             crudeDataBUSCAN->data_t8.freno_servicio = 0;
//         freno_aux = msg[6] >> 4; //quedarse con la parte alta de D6
//         //crudeDataBUSCAN->data_t8.freno_servicio_intensidad = (double)((((msg[5]*16) + freno_aux)-552)/14.47);
//         crudeDataBUSCAN->data_t8.freno_servicio_intensidad = (double)((((msg[5]*16) + freno_aux)-552));
//         break;
//     case 1037: //ID Distancia
//         crudeDataBUSCAN->data_t9.timestamp = *time;
//         break;
//     case 1042: //ID FRENO SERVICIO Y DE ESTACIONAMIENTO, PUERTAS ABIERTAS , TEMP MOTOR
//         crudeDataBUSCAN->data_t10.timestamp = *time;
//         //FRENO ESTACIONAMIENTO
//         if ((msg[0] & 0x0F) == 0x00) //freno estacionamiento suelto
//             crudeDataBUSCAN->data_t10.freno_estacionamiento=0;
//         else
//             crudeDataBUSCAN->data_t10.freno_estacionamiento=1;
//         //PUERTAS ABIERTAS
//         crudeDataBUSCAN->data_t10.puertas_abiertas = msg[6] >> 3;
//         //TEMPERATURA MOTOR
//         crudeDataBUSCAN->data_t10.temp_motor = (msg[5]-45);
//         break;
//     case 1074: //ID
//         crudeDataBUSCAN->data_t11.timestamp = *time;
//         break;
//     case 1101: //ID VELOCIDAD ABS MEDIA delanteras y traseras por separado
//         crudeDataBUSCAN->data_t12.timestamp = *time;
//         crudeDataBUSCAN->data_t12.vel_media_ruedas_delanteras = ((double)(msg[0]*256+msg[1])*0.01);
//         crudeDataBUSCAN->data_t12.vel_rueda_trasera_izq = ((double)(msg[2]*256+msg[3])*0.01);
//         crudeDataBUSCAN->data_t12.vel_rueda_trasera_dcha = ((double)(msg[4]*256+msg[5])*0.01);
//         crudeDataBUSCAN->data_t12.reservado = ((double)(msg[6]*256+msg[7])*0.01);
//         break;
//     case 1128: //ID
//         crudeDataBUSCAN->data_t13.timestamp = *time;
//         break;
//     case 1160: //ID TEMPERATURA MOTOR
//         crudeDataBUSCAN->data_t14.timestamp = *time;
//         break;
//     case 1161: //ID MARCHAS
//         crudeDataBUSCAN->data_t15.timestamp = *time;
//         //Modo cambio marchas
//         if(msg[1]==12)
//         {crudeDataBUSCAN->data_t15.modo_cambio = invierno;}
//         else if (msg[1]==4)
//         {crudeDataBUSCAN->data_t15.modo_cambio = sport;}
//         else
//         {crudeDataBUSCAN->data_t15.modo_cambio = normal;}
//         //Marchas


//         switch (msg[0])
//         {
//         case 0xA0: crudeDataBUSCAN->data_t15.marchas = P;	break;
//         case 0x91: crudeDataBUSCAN->data_t15.marchas = R;	break;
//         case 0xA2: crudeDataBUSCAN->data_t15.marchas = N;	break;
//         case 0x13: crudeDataBUSCAN->data_t15.marchas = D1;	break;
//         case 0x23: crudeDataBUSCAN->data_t15.marchas = D2;	break;
//         case 0x33: crudeDataBUSCAN->data_t15.marchas = D3;	break;
//         case 0x43: crudeDataBUSCAN->data_t15.marchas = D4;	break;
//         case 0x17: crudeDataBUSCAN->data_t15.marchas = M1;	break;
//         case 0x26: crudeDataBUSCAN->data_t15.marchas = M2;	break;
//         case 0x35: crudeDataBUSCAN->data_t15.marchas = M3;	break;
//         case 0x44: crudeDataBUSCAN->data_t15.marchas = M4;	break;
//         default:   crudeDataBUSCAN->data_t15.marchas = N;	break;
//         }

//         //printf("marcha=%x data_t15.marchas= %d\n", msg[0], crudeDataBUSCAN->data_t15.marchas);
//         break;
//     case 1293: //ID DISTANCIAS en D1D2 y D2D3
//         crudeDataBUSCAN->data_t16.timestamp = *time;
//         break;
//     case 1294: //ID
//         crudeDataBUSCAN->data_t17.timestamp = *time;
//         break;
//     case 1362: //ID TIMER 1 seg.
//         crudeDataBUSCAN->data_t18.timestamp = *time;
//         break;
//     case 1416: //ID
//         crudeDataBUSCAN->data_t19.timestamp = *time;
//         break;
//     case 1426: //ID Limpiaparabrisas delantero.
//         crudeDataBUSCAN->data_t20.timestamp = *time;
//         //LIMPIAPARABRISAS
//         switch (msg[2])
//         {
//         case 0x00: crudeDataBUSCAN->data_t20.limpiaparabrisas = 0; break;
//         case 0x10: crudeDataBUSCAN->data_t20.limpiaparabrisas = 1; break;
//         case 0x20: crudeDataBUSCAN->data_t20.limpiaparabrisas = 2; break;
//         case 0x40: crudeDataBUSCAN->data_t20.limpiaparabrisas = 3; break;
//         case 0x08: crudeDataBUSCAN->data_t20.limpiaparabrisas = 8; break;
//         default: 				     break;
//         }
//         break;
//     case 1544: //ID 3 TRAMAS ??
//         crudeDataBUSCAN->data_t21.timestamp = *time;
//         break;
//     case 1554: //ID LUCES, INTERMITENTES, NIVEL COMBUSTIBLE
//         crudeDataBUSCAN->data_t22.timestamp = *time;
//         //LUCES
//         if ((msg[1]&0x04) > 0) //largas activadas
//         {
//             if ((msg[1]&0x02) > 0) //largas + cortas + posicion activadas
//                 crudeDataBUSCAN->data_t22.luces = luces_largas;
//             else if ((msg[1]&0x01) > 0) //largas + posicion activadas
//                 crudeDataBUSCAN->data_t22.luces = luces_rafaga_posicion;
//             else //solo largas activadas = rafaga
//                 crudeDataBUSCAN->data_t22.luces = luces_rafaga;
//         }
//         else if ((msg[1]&0x02) > 0) //cortas activadas
//             crudeDataBUSCAN->data_t22.luces = luces_cortas;
//         else if ((msg[1]&0x01) > 0) //posicion activadas
//             crudeDataBUSCAN->data_t22.luces = luces_posicion;
//         else
//             crudeDataBUSCAN->data_t22.luces = luces_off;

//         if ((msg[1]&0x08) > 0) //antinieblas activadas
//             crudeDataBUSCAN->data_t22.luces_antiniebla = antiniebla_on;
//         else
//             crudeDataBUSCAN->data_t22.luces_antiniebla = antiniebla_off;
//         //INTEMITENTES
//         intermitentes_aux = msg[1]&0xF0;
//         switch (intermitentes_aux)
//         {
//         case 0x20:
//             crudeDataBUSCAN->data_t22.intermitente = intermitente_off;
//             break;
//         case 0xA0:
//             crudeDataBUSCAN->data_t22.intermitente = intermitente_izquierda;
//             break;
//         case 0x60:
//             crudeDataBUSCAN->data_t22.intermitente = intermitente_derecha;
//             break;
//         default:
//             break;
//         }
//         //NIVEL COMBUSTIBLE
//         crudeDataBUSCAN->data_t22.nivel_combustible = (msg[3]*0.5);

//         /*//CONSUMO MEDIO E INSTANTANEO
//                 if (nivel_comb_inicial == 0.0)
//                 {
//                   //printf("nivel_comb_inicial ==%f",crudeDataBUSCAN->data_t22.nivel_combustible );
//                     nivel_comb_inicial = crudeDataBUSCAN->data_t22.nivel_combustible;
//                     nivel_comb_anterior = crudeDataBUSCAN->data_t22.nivel_combustible;
//                 }
//                 if (nivel_comb_anterior != crudeDataBUSCAN->data_t22.nivel_combustible);
//                 {
//                 crudeDataBUSCAN->data_t22.consumo_medio = (nivel_comb_inicial - crudeDataBUSCAN->data_t22.nivel_combustible)/(crudeDataBUSCAN->distancia*100000);
//                 crudeDataBUSCAN->consumo_instantaneo = (nivel_comb_anterior - crudeDataBUSCAN->nivel_combustible) /(crudeDataBUSCAN->distancia - distancia_comb_anterior);
//                 distancia_comb_anterior = crudeDataBUSCAN->distancia; //Actualizar la distancia
//                 }*/

//         break;
//     case 1928: //ID
//         crudeDataBUSCAN->data_t23.timestamp = *time;
//         break;
//     case 1929: //ID
//         crudeDataBUSCAN->data_t24.timestamp = *time;
//         break;
//     case 1933: //ID
//         crudeDataBUSCAN->data_t25.timestamp = *time;
//         break;
//     case 1938: //ID
//         crudeDataBUSCAN->data_t26.timestamp = *time;
//         break;
//     case 2018: //ID TEMPERATURA EXTERIOR
//         crudeDataBUSCAN->data_t27.timestamp = *time;
//         //TEMPERATURA EXTERIOR
//         if (msg[2]==0x8D)
//             crudeDataBUSCAN->data_t27.temp_ext=(double)((msg[3]-80)*0.5);
//         //else
//         //	crudeDataBUSCAN->data_t27.temp_ext=-100; //Para evitar valores extraños al principio y evitar valores incorrectos cuando no hay lectura
//         break;
//     case 2034: //ID TEMPERATURA EXTERIOR
//         crudeDataBUSCAN->data_t28.timestamp = *time;
//         break;
//     default:
//         break;
//         //break;
//     } //End Switch (id)
// }