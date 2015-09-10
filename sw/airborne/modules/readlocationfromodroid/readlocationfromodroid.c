/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/readlocationfromodroid/readlocationfromodroid.c"
 * @author Roland
 * reads from the odroid and sends information of the drone to the odroid using JSON messages
 */

#include "modules/readlocationfromodroid/readlocationfromodroid.h"
#include "subsystems/abi.h"
#include "modules/computer_vision/opticflow_module.h"
#include <fcntl.h>
#include <stropts.h>
#include <termios.h>
#include <serial_port.h>
#include <stdio.h>
#include <inttypes.h>
#include <errno.h>
#include <string.h>
#include "state.h"
#include "subsystems/gps.h"

#include "cJSON.h"
#include "navdata.h"
#include "subsystems/ins/ins_int.h"
#include "subsystems/datalink/telemetry.h"


#include "firmwares/rotorcraft/autopilot.h"

#include "mcu_periph/uart.h"
#include "subsystems/radio_control.h"
#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#include "subsystems/electrical.h"
#include "subsystems/settings.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_none.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#include "generated/settings.h"

float test1=23.0;
float test2 = 24.0;
int32_t vo_ecef_x=25, vo_ecef_y=26, vo_ecef_z=27;
float vo_xd=0.0,vo_yd=0.0;
int timesRest = 100;
static void send_odometry(struct transport_tx *trans, struct link_device *dev)
{
    pprz_msg_send_ODOMETRY(trans, dev, AC_ID, &test1,&test2,&vo_ecef_x,&vo_ecef_y,&vo_ecef_z,&optitrack_ecef_x,&optitrack_ecef_y,&optitrack_ecef_z,&optitrack_ecef_xd,&optitrack_ecef_yd,&vo_xd,&vo_yd);
    //DOWNLINK_SEND_ODOMETRY (DefaultChannel, DefaultDevice,3.0,4.0);
}

struct SerialPort *READING_port;
speed_t usbInputSpeed = B115200;
char *serialResponse;
int writeLocationInput=0;
int resetOdroid=0;

void odroid_loc_init() {
  register_periodic_telemetry(DefaultPeriodic, "ODOMETRY", send_odometry);
	// Open the serial port
	READING_port = serial_port_new();
	printf("start reading matrix\n");
		char namePort[50];
		for(int portExtension = 0; portExtension < 10; portExtension++){
			sprintf(namePort,"/dev/ttyUSB%d",portExtension);
			printf("Trying to open %s\n",namePort);
			int result=serial_port_open_raw(READING_port,namePort,usbInputSpeed);
			printf("Result: %d\n", result);
			if(result >=0){
				break;
			}
		}
	//int result=serial_port_open_raw(READING_port,"/dev/ttyUSB0",usbInputSpeed);
	int lengthBytesImage=50000;//COMPLETE_MATRIX_WIDTH*MATRIX_ROWS;
	serialResponse=malloc(lengthBytesImage*sizeof(char));
	memset(serialResponse, '0', lengthBytesImage);
	resetOdroid=0;
 }
 void odroid_loc_periodic() {
	printf("Loc periodic! %d", writeLocationInput);
	char justRead='a';

	if (writeLocationInput > 10000)
		writeLocationInput = 0;
	int n = read(  READING_port->fd, &serialResponse[writeLocationInput], 3000);
	//printf("Read %d bytes\n",n);
	if (n < 0)
	{
		printf(strerror(errno));
		printf("\n");
	}
	else{
		writeLocationInput+=n;
		serialResponse[writeLocationInput]='\0';
		int index=0;
		//printf("Now read bytes %d string: %s\n",strlen(serialResponse),serialResponse);
		int start=-1;
		for(index=0; index < writeLocationInput; index++){

			//printf("%c",serialResponse[index]);
			if(serialResponse[index]=='{'){
				start=index;
				printf("Encountered start");
			}

			if(serialResponse[index]=='#' && start >=0)
			{
				int lengthMessage=index-start;
				char subbuf[lengthMessage];
				printf("Read now actually:\n");
				int indexPrint = 0;
				int indexSubbuf=0;
				for (indexPrint = start;indexPrint < index; indexPrint++){
					printf("%c",serialResponse[indexPrint]);
					subbuf[indexSubbuf++]=serialResponse[indexPrint];
				}
				printf("\n\nDone, now subbuf is: %s\nsubbuf[0]=%c && subbuf[lengthMessage]=%c\n",subbuf,subbuf[0],subbuf[lengthMessage-1]);
				if(subbuf[0]=='{' && subbuf[lengthMessage-1]=='}'){
					printf("\naww yeah\n");
					cJSON * root = cJSON_Parse(subbuf);

					cJSON *array = cJSON_GetObjectItem(root,"array");

					int lengthArray = cJSON_GetArraySize(array);

					for (indexPrint = 0;indexPrint < lengthArray; indexPrint++){
						printf("%d, ",cJSON_GetArrayItem(array,indexPrint)->valueint);
					}
					printf("\n");
					printf(cJSON_PrintUnformatted(root));
					printf("\n");
//					int receivedReset = cJSON_GetObjectItem(root,"receivedReset")->valueint;
//
//					if (resetOdroid && receivedReset){
//							resetOdroid=0;
//					}
//					float xValue = cJSON_GetObjectItem(root,"x")->valuedouble;
//					float yValue = cJSON_GetObjectItem(root,"y")->valuedouble;
//                    vo_ecef_x = cJSON_GetObjectItem(root,"ecefposx")->valueint;
//                    vo_ecef_y = cJSON_GetObjectItem(root,"ecefposy")->valueint;
//                    vo_ecef_z = cJSON_GetObjectItem(root,"ecefposz")->valueint;
//                    vo_xd = cJSON_GetObjectItem(root,"vovx")->valuedouble;
//                    vo_yd = cJSON_GetObjectItem(root,"vovy")->valuedouble;
//                    gps.ecef_pos.x = vo_ecef_x;
//                    gps.ecef_pos.y = vo_ecef_y;
//                    gps.ecef_pos.z = vo_ecef_z;
//					printf("x: %d\n",xValue);
//					printf("y: %d\n",yValue);
//					printf("^^^^^\n");
//						test1 = xValue;
//				test2 = yValue;
            //    gps.fix = GPS_FIX_3D;
/*
                GpsFixValid();
            // publish new GPS data
              uint32_t now_ts = get_sys_time_usec();
              gps.last_msg_ticks = sys_time.nb_sec_rem;
              gps.last_msg_time = sys_time.nb_sec;
              if (gps.fix == GPS_FIX_3D) {
                gps.last_3dfix_ticks = sys_time.nb_sec_rem;
                gps.last_3dfix_time = sys_time.nb_sec;
              }
              AbiSendMsgGPS(GPS_DATALINK_ID, now_ts, &gps);
/*
				 gps.ecef_pos.x = cJSON_GetObjectItem(root,"ecefposx")->valueint;
					  gps.ecef_pos.y = cJSON_GetObjectItem(root,"ecefposy")->valueint;
					 gps.ecef_pos.z = cJSON_GetObjectItem(root,"ecefposz")->valueint;;

					  gps.lla_pos.lat =cJSON_GetObjectItem(root,"lat")->valueint;
					  gps.lla_pos.lon = cJSON_GetObjectItem(root,"lon")->valueint;
					  //test1 = cJSON_GetObjectItem(root,"lat")->valuedouble;
					  //test2 = cJSON_GetObjectItem(root,"lon")->valuedouble;
					  gps.lla_pos.alt = cJSON_GetObjectItem(root,"alt")->valueint;
					  gps.hmsl        = 125;
					  gps.ecef_pos.x = cJSON_GetObjectItem(root,"ecefposx")->valueint;
					  gps.ecef_pos.y = cJSON_GetObjectItem(root,"ecefposy")->valueint;
					  gps.ecef_pos.z = cJSON_GetObjectItem(root,"ecefposz")->valueint;

					  //gps.ecef_vel.x = vel_x;
					  //gps.ecef_vel.y = vel_y;
					  //gps.ecef_vel.z = 0;
/*
					  gps.course = 100;
					  gps.num_sv = 11;
					  gps.tow = 0;

*/
					/*
					IvySendMsg("0 REMOTE_GPS %d %d %d %d %d %d %d %d %d %d %d %d %d %d", 201,
					      1,                //uint8 Number of markers (sv_num)
					      (int)(xValue),                //int32 ECEF X in CM
					      (int)(yValue),                //int32 ECEF Y in CM
					      (int)(23),                //int32 ECEF Z in CM
					      (int)(0),       //int32 LLA latitude in deg*1e7
					      (int)(0),       //int32 LLA longitude in deg*1e7
					      (int)(123),              //int32 LLA altitude in mm above elipsoid
					      (int)(30),         //int32 HMSL height above mean sea level in mm
					      (int)(20), //int32 ECEF velocity X in cm/s
					      (int)(10), //int32 ECEF velocity Y in cm/s
					      (int)(0), //int32 ECEF velocity Z in cm/s
					      0,
					      (int)(20));             //int32 Course in rad*1e7
*/


					writeLocationInput=0;
				}
			}

		}
		printf("\n");
	}
	cJSON *root, *droneInformation;
	root = cJSON_CreateObject();
	struct Int32Eulers *euler = stateGetNedToBodyEulers_i();

	cJSON_AddItemToObject(root, "droneInformation", droneInformation = cJSON_CreateObject());
	cJSON_AddNumberToObject(droneInformation, "phi", euler->phi);
	cJSON_AddNumberToObject(droneInformation, "theta", euler->theta);
	cJSON_AddNumberToObject(droneInformation, "psi", euler->psi);
	cJSON_AddNumberToObject(droneInformation, "height", state.enu_pos_f.z);
	cJSON_AddNumberToObject(droneInformation, "accel_z", ins_int.ltp_accel.z);
	cJSON_AddNumberToObject(droneInformation, "accel_x", ins_int.ltp_accel.x);
	cJSON_AddNumberToObject(droneInformation, "accel_y", ins_int.ltp_accel.y);
	cJSON_AddNumberToObject(droneInformation, "ultrasound", navdata.measure.ultrasound);
	cJSON_AddNumberToObject(droneInformation, "baroz", ins_int.baro_z);
	cJSON_AddNumberToObject(droneInformation, "gpsx", gps.ecef_pos.x);
	cJSON_AddNumberToObject(droneInformation, "gpsy", gps.ecef_pos.y);
	cJSON_AddNumberToObject(droneInformation, "gpsz", gps.ecef_pos.z);
	cJSON_AddNumberToObject(droneInformation, "opticflowvelx", vel_x);
	cJSON_AddNumberToObject(droneInformation, "opticflowvely", vel_y);

	cJSON_AddNumberToObject(droneInformation, "optitrack_lat", optitrack_lat);
	cJSON_AddNumberToObject(droneInformation, "optitrack_lon", optitrack_lon);
	cJSON_AddNumberToObject(droneInformation, "optitrack_alt", optitrack_alt);
	cJSON_AddNumberToObject(droneInformation, "optitrack_hmsl", optitrack_hmsl);
	cJSON_AddNumberToObject(droneInformation, "optitrack_ecef_x", optitrack_ecef_x);
	cJSON_AddNumberToObject(droneInformation, "optitrack_ecef_y", optitrack_ecef_y);
	cJSON_AddNumberToObject(droneInformation, "optitrack_ecef_z", optitrack_ecef_z);
	cJSON_AddNumberToObject(droneInformation, "optitrack_ecef_xd", optitrack_ecef_xd);
	cJSON_AddNumberToObject(droneInformation, "optitrack_ecef_yd", optitrack_ecef_yd);
	cJSON_AddNumberToObject(droneInformation, "optitrack_ecef_zd", optitrack_ecef_zd);
	cJSON_AddNumberToObject(droneInformation, "optitrack_course", optitrack_course);
	cJSON_AddNumberToObject(droneInformation, "optitrack_numsv", optitrack_numsv);
	cJSON_AddNumberToObject(droneInformation, "optitrack_tow", optitrack_tow);
	gps.lla_pos.lat = optitrack_lat;
	  gps.lla_pos.lon = optitrack_lon;
	 gps.lla_pos.alt = optitrack_alt;

	  gps.hmsl        = optitrack_hmsl;

    /*  gps.ecef_pos.x = optitrack_ecef_x;
	  gps.ecef_pos.y = optitrack_ecef_y;
      gps.ecef_pos.z = optitrack_ecef_z;*/
	  gps.ecef_vel.x = optitrack_ecef_xd;
	  gps.ecef_vel.y = optitrack_ecef_yd;
	  gps.ecef_vel.z = optitrack_ecef_zd;

	  gps.course = optitrack_course;
	  gps.num_sv = optitrack_numsv;
	  gps.tow = optitrack_tow;
	  gps.fix = GPS_FIX_3D;
/*
	gps.ecef_vel.x = vel_x;
	gps.ecef_vel.y = vel_y;
*/

	cJSON_AddNumberToObject(root, "mustReset", resetOdroid);

	char* toWrite = cJSON_PrintUnformatted(root);
	int lengthToWrite= strlen(toWrite);
	printf("Writing to odroid: \n");
	printf(toWrite);
	printf("\n");
	write(READING_port->fd,toWrite,lengthToWrite);
	write(READING_port->fd,"\n",1);

 }


