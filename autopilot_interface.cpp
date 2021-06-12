//hey there
/****************************************************************************
 
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *           Tristan Sherman, <tristan.m.sherman@gmail.com>
 *	     Mitchell Caudle, Jimmy Lopez, Hana Haideri
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 * @author Tristan Sherman, <tristan.m.sherman@gmail.com>
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"

#include <iostream>
#include <string>
#include <fstream>
#include <chrono>
#include <cstring>

//ADDED FOR NEW CA_AVOID CODE
//also added these two lines to autopilot_interface.h
#include <cmath>
#include <tgmath.h>
#define TO_DEGREES 180 / PI
// END -- MAY BE SWITCHED TO OTHER FILE LATER


//ADDED	TO TEST CALLSIGN READING
#include <typeinfo>
using std::endl;
using std::cout;
//END CALLSIGN ADDS


using std::string;
using namespace std::chrono;



//overloaded simple string conversion methods
string convertToString(bool var)
{
	if (var)
		return "TRUE";
	else
		return "FALSE";
}
string convertToString(uint64_t var)
{
	return std::to_string(var);
}

string convertToString(double var)
{
	return std::to_string(var);
}
string convertToString(float var)
{
	return std::to_string(var);
}

string convertToString(int var)
{
	return std::to_string(var);
}
//returns system timestamp as string
string timeStamp()
{

	auto tp = std::chrono::high_resolution_clock::now();
	auto ttime_t = std::chrono::system_clock::to_time_t(tp);
	auto tp_sec = std::chrono::system_clock::from_time_t(ttime_t);
	milliseconds ms = duration_cast<milliseconds>(tp - tp_sec);

	std::tm * ttm = localtime(&ttime_t);

	char date_time_format[] = "%Y.%m.%d-%H.%M.%S";

	char time_str[] = "yyyy.mm.dd.HH-MM.SS.fff";

	strftime(time_str, strlen(time_str), date_time_format, ttm);

	string result(time_str);
	result.append(".");
	result.append(std::to_string(ms.count()));
	

	return result.append(":");
}

//Appends to txt file or creates txt file if none exist
static void addToFile(string line, string description)
{
	std::ofstream file;
	//modify string filepath based on folder you want to access
	//currently will output txt file to project cmake-debug folder
	//string fileName = localtime
	time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char filepath [80];
    strftime (filepath,80,"logging_file_%Y_%m_%d.txt",now);

	file.open(filepath, std::ios::out | std::ios::app);
	//make sure write fails with exception if something is wrong
	file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);
	file << timeStamp() + " " + description + " : " + line << std::endl;
	file.close();
}

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}



/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}







// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	write_count = 0;
	error_counter = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	reading_waypoint_status = 0;
	control_status = 0;      // whether the autopilot is in offboard control mode
	CA_status = 0;	 // whether the collision avoidance thread is running
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id
	CA_tid = 0;//Collision avoidance thread id

	system_id    = 1; // system id
	autopilot_id = 1; // autopilot component id
	companion_id = 1; // companion computer component id/dev/ttyUSB0

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{

	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		//Read message from serial
		mavlink_message_t message;
		success = serial_port->read_message(message);

		//Handle received message
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
				{
					//printf("MAVLINK_MSG_ID_RC_CHANNELS_RAW\n");
					mavlink_msg_rc_channels_raw_decode(&message, &(current_messages.rc_channels_raw_t));
					current_messages.time_stamps.rc_channels_raw_t = get_time_usec();
					this_timestamps.rc_channels_raw_t = current_messages.time_stamps.rc_channels_raw_t;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

				case MAVLINK_MSG_ID_MISSION_COUNT:
				{
					
					mavlink_msg_mission_count_decode(&message, &(current_messages.mavlink_mission_count));
					current_messages.time_stamps.mavlink_mission_count = get_time_usec();
					this_timestamps.mavlink_mission_count = current_messages.time_stamps.mavlink_mission_count;
					printf("MAVLINK_MSG_ID_MISSION_COUNT: %u\n", current_messages.mavlink_mission_count);
					break;
				}

				case MAVLINK_MSG_ID_MISSION_ITEM_INT:
				{
					printf("MAVLINK_MSG_ID_MISSION_ITEM\n");
					mavlink_msg_mission_item_int_decode(&message, &(current_messages.mavlink_mission_item));
					current_messages.time_stamps.mavlink_mission_item = get_time_usec();
					this_timestamps.mavlink_mission_item = current_messages.time_stamps.mavlink_mission_item;
					break;
				}

				case MAVLINK_MSG_ID_MISSION_CURRENT:
				{
					//printf("MAVLINK_MSG_ID_MISSION_CURRENT\n");
					mavlink_msg_mission_current_decode(&message, &(current_messages.mission_current_t));
					current_messages.time_stamps.mission_current_t = get_time_usec();
					this_timestamps.mission_current_t = current_messages.time_stamps.mission_current_t;

					currentWaypoint = current_messages.mission_current_t.seq;
					//printf("MAVLINK_MSG_ID_MISSION_CURRENT: %u\n", currentWaypoint);
					
					break;
				}

				case MAVLINK_MSG_ID_ADSB_VEHICLE:
				{
					//printf("\nMAVLINK_MSG_ID_ADS_B\n");
					//Test that this function will work

				
					// TEST CODE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
					//Get Callsign
					//string callsign = mavlink_msg_adsb_vehicle_get_callsign(&message, (current_messages.adsb_vehicle_t.callsign));
					//cout << typeid(mavlink_msg_adsb_vehicle_get_callsign(&message, (current_messages.adsb_vehicle_t.callsign))).name() << endl;
					//cout << mavlink_msg_adsb_vehicle_get_callsign(&message, (current_messages.adsb_vehicle_t.callsign)) << endl;
					//printf("Callsign: %c\n", callsign);


					//Get ICAO address
					uint32_t ICAO = mavlink_msg_adsb_vehicle_get_ICAO_address(&message);
					printf("ICAO %u\n", ICAO);

					// if (ICAO == 10700118 / 11393572) { //ADSB on test board
					if (ICAO == 3819648  ) { //ADSB on Piper Cub #Swip ICAO if this not working#
						printf("Quadcopter's ICAO %u\n", ICAO);			
						// Decode full message and store it
						mavlink_msg_adsb_vehicle_decode(&message, &(current_messages.adsb_vehicle_t));
						current_messages.time_stamps.adsb_vehicle_t = get_time_usec();
						this_timestamps.adsb_vehicle_t = current_messages.time_stamps.adsb_vehicle_t;

						printf("ICAO after decode: %u \n", current_messages.adsb_vehicle_t.ICAO_address);
						//printf("Time since last message: %u \n", current_messages.adsb_vehicle_t.tslc);

					}

					break;
				}

				case MAVLINK_MSG_ID_MISSION_ACK:
				{
					printf("MAVLINK_MSG_ID_MISSION_ACK\n");
					mavlink_msg_mission_ack_decode(&message, &(current_messages.mavlink_mission_ack));
					//if(current_messages.mavlink_mission_ack != 0)
						printf("ACK MSG: %u\n\n", current_messages.mavlink_mission_ack.type);
					//else	
						//printf("Success\n");
					break;
				}

				case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
				{
					printf("MAVLINK_MSG_ID_MISSION_REQUEST_INT\n");
					mavlink_msg_mission_request_int_decode(&message, &(current_messages.mission_request_int));
					current_messages.time_stamps.mission_request_int = get_time_usec();
					this_timestamps.mission_request_int = current_messages.time_stamps.mission_request_int;
					break;
				}

				case MAVLINK_MSG_ID_MISSION_REQUEST:
				{
					
					mavlink_msg_mission_request_decode(&message, &(current_messages.mission_request));
					current_messages.time_stamps.mission_request = get_time_usec();
					this_timestamps.mission_request = current_messages.time_stamps.mission_request;
					/*printf("MAVLINK_MSG_ID_MISSION_REQUEST -- Seq: %d\n",current_messages.mission_request.seq);
					printf("MAVLINK_MSG_ID_MISSION_REQUEST -- Mission: %d\n",current_messages.mission_request.mission_type);
					printf("MAVLINK_MSG_ID_MISSION_REQUEST -- TID: %u\n",current_messages.mission_request.target_system);
					printf("MAVLINK_MSG_ID_MISSION_REQUEST -- CID: %u\n",current_messages.mission_request.target_component);
					*/break;
				}

				case MAVLINK_MSG_ID_COMMAND_ACK:
				{
					
					mavlink_msg_command_ack_decode(&message, &(current_messages.mavlink_command_ack));
					current_messages.time_stamps.mavlink_command_ack = get_time_usec();
					this_timestamps.mavlink_command_ack = current_messages.time_stamps.mavlink_command_ack;
					printf("MAVLINK_MSG_ID_COMMAND_ACK -- result: %d\n",current_messages.mavlink_command_ack.result);
					break;
				}

				default:
				{
					//printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all = received_all = recieved_all_messages(this_timestamps);

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}


/*
 * Check if all desired messages have been recieved
 */
bool
Autopilot_Interface::recieved_all_messages(const Time_Stamps &time_stamps) {

    //Returns false only if you want to read something and haven't
    // If you want to read attitude (1) and haven't (0). 1 == 0 ==> false
    // If you don't want do read attitude(0) and haven't(0). 0 == 0 ==> true
    return
        (messages_to_read.read_attitude == time_stamps.attitude)
        //&&  (messages_to_read.read_autopilot_version == time_stamps.autopilot_version)
        &&  (messages_to_read.read_battery_status == time_stamps.battery_status)
        &&  (messages_to_read.read_global_position_int == time_stamps.global_position_int)
        &&  (messages_to_read.read_heartbeat == time_stamps.heartbeat)
        &&  (messages_to_read.read_highres_imu == time_stamps.highres_imu)
        &&  (messages_to_read.read_local_position_ned == time_stamps.local_position_ned)
        &&  (messages_to_read.read_position_target_global_int == time_stamps.position_target_global_int)
        &&  (messages_to_read.read_position_target_local_ned == time_stamps.position_target_local_ned)
        &&  (messages_to_read.read_radio_status == time_stamps.radio_status)
        &&  (messages_to_read.read_sys_status == time_stamps.sys_status);
        //&&  (messages_to_read.read_power_status == time_stamps.power_status);
}




// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	//uses pthread helper function &start_mavlink_interface_read_thread
   	//to call function start_read_thread in a new thread
	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		
		if ( time_to_exit )
		return;
		printf("Reboot Autopilot\n");

		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}



	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	
	printf("\n********START WRITE THREAD*************\n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;



}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);
	pthread_join(CA_tid,NULL);

	// now the predict, read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		//printf("Read message\n");
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}





// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	current_setpoint = sp;

	// write a message and signal writing
	write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		write_setpoint();
	}

	// signal end
	writing_status = false;

	return;

}



/*
 * Change PWM value of servo. (Moves Servo)
 */
void
Autopilot_Interface::
write_set_servo(const int &servo, const int &pwm)
{
    printf("Changing PWM value of servo %d to %d\n", servo, pwm);
    writing_status = true;
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------


    mavlink_command_long_t cmd;
    cmd.target_system = system_id;
    cmd.target_component = autopilot_id;
    cmd.command = MAV_CMD_DO_SET_SERVO;
    cmd.param1 = servo;
    cmd.param2 = pwm;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id,companion_id, &message, &cmd);


    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);

    // check the write
    if ( len <= 0 )
        fprintf(stderr,"WARNING: could not send MAV_CMD_DO_SET_SERVO \n");

    writing_status = false;
}


// End Autopilot_Interface

/*
 * Send waypoints to pixhawk
 */
void
Autopilot_Interface::
write_waypoints(std::vector<mavlink_mission_item_int_t> waypoints, uint16_t desiredSeq) {


	/*for (int count = 0; count < waypoints.size(); count++)
		printf("Waypoint %d\n Lat: %d, Lon %d\n",waypoints[count].seq,waypoints[count].x,waypoints[count].y);
    //printf("Autopilot_Interface::write_waypoints()\n");
	printf("Sending Waypoints\n");
    */writing_status = true;

	/*5-15-2021
	mavlink_message_t clear_message;
	mavlink_mission_clear_all_t clear_all_mission;
	clear_all_mission.target_system = system_id;
	clear_all_mission.target_component = autopilot_id;
	clear_all_mission.mission_type = MAV_MISSION_TYPE_ALL;
	mavlink_msg_mission_clear_all_encode(system_id, companion_id, &clear_message, &clear_all_mission);
	*/

    //Pixhawk needs to know how many waypoints it will receive
    mavlink_mission_count_t mission_count;
    mission_count.count = (uint16_t) waypoints.size();
	mission_count.target_component = autopilot_id;
	mission_count.target_system = system_id;
	mission_count.mission_type = MAV_MISSION_TYPE_MISSION;

	  printf("mission_count.count %i\n",mission_count.count);
    if(send_waypoint_count(mission_count) <= 0){
        fprintf(stderr,"WARNING: could not send waypoint count \n");
    }


	
	

    //Send all waypoints
    for(int i = 0; i < waypoints.size(); i++){
        waypoints[i].target_system = system_id;
        waypoints[i].target_component = autopilot_id;
        waypoints[i].frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        waypoints[i].autocontinue = true;

		  //if (i == desiredSeq) {waypoints[i].current = 1;}
		  //else          {waypoints[i].current = 0;}

		//WAIT FOR WAYPOINT REQUEST
		uint64_t time = current_messages.time_stamps.mission_request;

		while( time == current_messages.time_stamps.mission_request ) {
			//printf("waiting for request...\n");
			printf("AP Requesting SEQ: %d\n", current_messages.mission_request.seq);
			//sequence = current_messages.mission_request.seq;
			usleep(300000);
		}

		//printf("Give me number %u\n", current_messages.mission_request.seq);
		//printf("Waypoint seq: %u\n", waypoints[i].seq);

		// SEND WAYPOINT
		//printf("1Waypoint %d Lat: %d, Long: %d, Alt: %d, Seq: %u\n", i, waypoints[i].x, waypoints[i].y, waypoints[i].z, waypoints[i].seq);
		mavlink_message_t message;
        mavlink_msg_mission_item_int_encode(system_id, companion_id, &message, &waypoints[i]);
        //printf("2Waypoint %d Lat: %f, Long: %f, Alt: %f, Seq: %u\n", i, waypoints[i].x, waypoints[i].y, waypoints[i].z, waypoints[i].seq);
        if ( write_message(message) <= 0 )
            fprintf(stderr,"WARNING: could not send MAV_CMD_DO_SET_SERVO \n");
    }
	
	//printf("Write_waypoint:NewWaypoint: %f, %f\n", waypoints[waypoints.size()-1].x, waypoints[waypoints.size()-1].y);


	// Send ACK


    writing_status = false;
}


void
Autopilot_Interface::
write_waypoints_old(std::vector<mavlink_mission_item_t> waypoints, uint16_t desiredSeq) {


	for (int count = 0; count < waypoints.size(); count++)
		printf("Waypoint %d\n Lat: %d, Lon %d\n",waypoints[count].seq,waypoints[count].x,waypoints[count].y);
    //printf("Autopilot_Interface::write_waypoints()\n");
	printf("Sending Waypoints\n");
    writing_status = true;

	/*5-15-2021
	mavlink_message_t clear_message;
	mavlink_mission_clear_all_t clear_all_mission;
	clear_all_mission.target_system = system_id;
	clear_all_mission.target_component = autopilot_id;
	clear_all_mission.mission_type = MAV_MISSION_TYPE_ALL;
	mavlink_msg_mission_clear_all_encode(system_id, companion_id, &clear_message, &clear_all_mission);
	*/

    //Pixhawk needs to know how many waypoints it will receive
    mavlink_mission_count_t mission_count;
    mission_count.count = (uint16_t) waypoints.size();
	  printf("mission_count.count %i\n",mission_count.count);
    if(send_waypoint_count(mission_count) <= 0){
        fprintf(stderr,"WARNING: could not send waypoint count \n");
    }


	
	

    //Send all waypoints
    for(int i = 0; i < waypoints.size(); i++){
        waypoints[i].target_system = system_id;
        waypoints[i].target_component = autopilot_id;
        waypoints[i].frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        waypoints[i].autocontinue = true;

		  //if (i == desiredSeq) {waypoints[i].current = 1;}
		  //else          {waypoints[i].current = 0;}

		//WAIT FOR WAYPOINT REQUEST
		uint64_t time = current_messages.time_stamps.mission_request;

		while( time == current_messages.time_stamps.mission_request ) {
			//printf("waiting for request...\n");
			printf("AP Requesting SEQ: %d\n", current_messages.mission_request.seq);
			//sequence = current_messages.mission_request.seq;
			usleep(300000);
		}

		printf("Give me number %u\n", current_messages.mission_request.seq);
		printf("Waypoint seq: %u\n", waypoints[i].seq);

		// SEND WAYPOINT
		//printf("1Waypoint %d Lat: %d, Long: %d, Alt: %d, Seq: %u\n", i, waypoints[i].x, waypoints[i].y, waypoints[i].z, waypoints[i].seq);
		mavlink_message_t message;
        mavlink_msg_mission_item_encode(system_id, companion_id, &message, &waypoints[i]);
        printf("2Waypoint %d Lat: %f, Long: %f, Alt: %f, Seq: %u\n", i, waypoints[i].x, waypoints[i].y, waypoints[i].z, waypoints[i].seq);
        if ( write_message(message) <= 0 )
            fprintf(stderr,"WARNING: could not send MAV_CMD_DO_SET_SERVO \n");
    }
	
	printf("Write_waypoint:NewWaypoint: %f, %f\n", waypoints[waypoints.size()-1].x, waypoints[waypoints.size()-1].y);


    writing_status = false;
}

/*
 * Tells pixhawk how many waypoints it will recieve
 */
int
Autopilot_Interface::
send_waypoint_count(mavlink_mission_count_t mavlink_mission_count) {
    //mavlink_mission_count.target_system = system_id;
    //mavlink_mission_count.target_component = autopilot_id;
    mavlink_message_t message;
    mavlink_msg_mission_count_encode(system_id, autopilot_id, &message, &mavlink_mission_count);
    return write_message(message);
}






// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------


void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}

void*
start_collision_avoidance_thread (void *args)
{

	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->collision_avoidance_begin();

	// done!
	return NULL;

}
/*
 * Returns a mavlink acceptable waypoint
 */
mavlink_mission_item_int_t
Autopilot_Interface::create_waypoint(const int32_t &lat, const int32_t &lon, const float &alt, const uint16_t &wp_number, const float &radius) {
    mavlink_mission_item_int_t mission_item;
    mission_item.command = MAV_CMD_NAV_WAYPOINT;
	mission_item.mission_type = 0;
	//printf("COMMAND: %i\n", mission_item.command);
    mission_item.param1 = 0;//hold time in decimal second IGNORED by ArduPlane
    mission_item.param2 = radius;//Acceptance radius in meters
    mission_item.param3 = 0;//0 to pass through WP if >0 radius in meters to pass by WP
    mission_item.param4 = 0;//Desired yaw angle NaN for unchanged
    mission_item.x = lat;//latitude
    mission_item.y = lon;//longitude
    mission_item.z = alt;//altitude
    mission_item.seq = wp_number;//waypoint number
    return mission_item;
}

mavlink_mission_item_t
Autopilot_Interface::create_waypoint_old(const float &lat, const float &lon, const float &alt, const uint16_t &wp_number, const float &radius) {
    mavlink_mission_item_t mission_item;
    mission_item.command = MAV_CMD_NAV_WAYPOINT;
	mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	// This function creates a waypoint in the old mission_item format
	//printf("COMMAND: %i\n", mission_item.command);
    mission_item.param1 = 0;//hold time in decimal second IGNORED by ArduPlane
    mission_item.param2 = radius;//Acceptance radius in meters
    mission_item.param3 = 0;//0 to pass through WP if >0 radius in meters to pass by WP
    mission_item.param4 = 0;//Desired yaw angle NaN for unchanged
    mission_item.x = lat;//latitude
    mission_item.y = lon;//longitude
    mission_item.z = alt;//altitude
    mission_item.seq = wp_number;//waypoint number
    return mission_item;
}


/*
 * Returns a mavlink acceptable loiter waypoint
 */
mavlink_mission_item_int_t
Autopilot_Interface::create_loiter_point(const int32_t &lat, const int32_t &lon, const float &alt, const uint16_t &wp_number, const float &radius) {
    mavlink_mission_item_int_t mission_item;
    mission_item.command = MAV_CMD_NAV_LOITER_UNLIM;
    mission_item.param1 = 0;//hold time in decimal second IGNORED by ArduPlane
    mission_item.param2 = 0; //empty
    mission_item.param3 = radius;//0 to pass through WP if >0 radius in meters to pass by WP
    mission_item.param4 = 0;//Desired yaw angle NaN for unchanged
    mission_item.x = lat;//latitude
    mission_item.y = lon;//longitude
    mission_item.z = alt;//altitude
    mission_item.seq = wp_number;//waypoint number
    return mission_item;
}


/*
 * Returns a mavlink acceptable loiter waypoint
 */
mavlink_mission_item_int_t
Autopilot_Interface::create_takeoff_point(const float &lat, const float &lon, const int &alt, const int &wp_number) {
    mavlink_mission_item_int_t mission_item;
    mission_item.command = MAV_CMD_NAV_TAKEOFF;
    mission_item.param1 = 10;//desired pitch angle during takeoff
    mission_item.param2 = 0; //empty
    mission_item.param3 = 0;//empty
    mission_item.param4 = NAN;//Desired yaw angle NaN for unchanged
    mission_item.x = lat;//latitude
    mission_item.y = lon;//longitude
    mission_item.z = alt;//altitude
    mission_item.seq = wp_number;//waypoint number
    return mission_item;
}


/*
 * Returns a mavlink acceptable landing waypoint
 */
mavlink_mission_item_int_t
Autopilot_Interface::create_land_point(const float &lat, const float &lon, const int &wp_number) {
    mavlink_mission_item_int_t mission_item;
    mission_item.command = MAV_CMD_NAV_LAND;
    mission_item.param1 = 40;//Minimum target altitude if landing is aborted
    mission_item.param2 = 0; //empty
    mission_item.param3 = 0;//empty
    mission_item.param4 = NAN;//Desired yaw angle NaN for unchanged
    mission_item.x = lat;//latitude
    mission_item.y = lon;//longitude
    mission_item.seq = wp_number;//waypoint number
    return mission_item;
}



/*
 * Set aircraft mode
 */
void
Autopilot_Interface::setMode(std::string mode) {

    int MODE;

    if (mode == "AUTO")           { MODE = 92; } //MAV_MODE_AUTO_DISARMED
    else if (mode == "STABILIZE") { MODE = 80; } //MAV_MODE_STABILIZE_DISARMED
    else if (mode == "MANUAL")    { MODE = 64; } //MAV_MODE_MANUAL_DISARMED


    writing_status = true;
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    mavlink_command_long_t cmd;
    cmd.target_system = system_id;
    cmd.target_component = autopilot_id;
    cmd.command = MAV_CMD_DO_SET_MODE;
    cmd.param1 = MODE;
    
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id,companion_id, &message, &cmd);


    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);
    printf("MODE: %u\n", MODE);
    // check the write
    if ( len <= 0 )
        fprintf(stderr,"WARNING: could not change MODE \n");

    writing_status = false;

    return;
}





/*
 * Arm aircraft
 */
void
Autopilot_Interface::arm_disarm(int arm) {
//Value of 1 arms the autopilot, value of 0 disarms the autopilot

    if (arm == 1) printf("Arming... \n");
    if (arm == 0) printf("Disarming... \n");
    if (arm != 0 && arm != 1) {printf("Not a valid command!\n"); return; }

    writing_status = true;

    mavlink_command_long_t cmd;
    cmd.target_system = system_id;
    cmd.target_component = autopilot_id;
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.param1 = arm;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id,companion_id, &message, &cmd);

    // do the write
    int len = write_message(message);

    // check the write
    if ( len <= 0 )
        fprintf(stderr,"WARNING: could not ARM/DISARM \n");
    writing_status = false;

    return;
}





/*Return a new position given the x and y displacement in meters of a waypoint
  The waypoint is created off an existing mission item*/
mavlink_mission_item_int_t
Autopilot_Interface::
createNewDisplacedWaypoint(const double & deltaX, const double & deltaY, const mavlink_mission_item_int_t & b){

    //coordinate offset in Radians
    float deltaLat = (deltaY / RADIUS_EARTH);
    float deltaLong = deltaX / (RADIUS_EARTH * cos(b.x * PI / 180));

    mavlink_mission_item_int_t newPosition = b;
    newPosition.x = b.x + (deltaLat * (180 / PI));
    newPosition.y = b.y + deltaLong * ((180 / PI));
    //newPosition.seq = b.seq +1;    
    return newPosition;

}




















//Return a new position given the x and y displacement in meters of an avoid waypoint
//The waypoint is created off of the current position of the aircraft
mavlink_mission_item_int_t
Autopilot_Interface::NewAvoidWaypoint(const double & deltaX, const double & deltaY, aircraftInfo & pos){
 
    //INPUTS: deltaX and deltaY in meters
    //	      pos: current lat,lon position of the aircraft
    // NOTE: This function uses NED frame. Therefore, +x = North = dLat
    //                                                +y = East  = dLon    

	 //printf("deltax %f\n", deltaX);
    //Convert from cartesian to NED frame. For ease of understanding
    double deltaX_NED = deltaX;
    double deltaY_NED = deltaY;

    //coordinate offset in Radians
    double deltaLat = deltaX_NED  / RADIUS_EARTH;
    double deltaLon = deltaY_NED  / RADIUS_EARTH;

	 //printf("deltaLat(1) %f\n", deltaLat);

    mavlink_mission_item_int_t newPosition;
    
	 //printf("pos.lat[0] %f\n", pos.lat[0] );
    newPosition.x = pos.lat[0]  + (deltaLat / (3.14159265359/180.0) );
    newPosition.y = pos.lon[0]  + (deltaLon / (3.14159265359/180.0) );

	 //printf("deltaLat %f\n", deltaLat/(3.14159265359/180.0));
	 //printf("deltaLon %f\n", deltaLon/(3.14159265359/180.0));

	 //printf("newPosition.x %f\n", newPosition.x);

    return newPosition;

}



//Return x and y distance (NED frame) from current position to second position
mavlink_mission_item_int_t
Autopilot_Interface::
distanceVectors(const double &target_lat, const double &target_long, const double &current_lat, const double &current_long){
 
    //INPUTS: Current position and second position
    // NOTE: This function uses NED frame. Therefore, +x = North = dLat
    //                                                +y = East  = dLon    


	 //Find change in lattitude and longitude in radians
    double deltaLat = (target_lat - current_lat) * (PI/180);
	 double deltaLon = (target_long - current_long) * (PI/180);

	 //Find distance in meters
	 double distX = deltaLat * RADIUS_EARTH;
    double distY = deltaLon * RADIUS_EARTH;

    mavlink_mission_item_int_t newPosition;

	 //This must be changed later. For now we use mavlink_mission_item_int_t to get the job done. What we are really outputting is x and y distance
    newPosition.x = distX; //Distance N/S (meters)
    newPosition.y = distY; // Distance E/W (meters)

    return newPosition;

}



















/*
	Calculates the distance between the target's latitude and longitude coordinates and the plane's latitude and longitude coordinates
	Calculates in meters
*/
double
Autopilot_Interface::
gpsDistance(const double &target_lat, const double &target_long, const double &current_lat, const double &current_long)
{

    double tLatRad = target_lat * TO_RADIANS;
    double pLatRad = current_lat * TO_RADIANS;
    double deltaLat = (current_lat - target_lat) * TO_RADIANS;
    double deltaLong = (current_long - target_long) * TO_RADIANS;

    double var = sin(deltaLat / 2) * sin(deltaLat / 2) + (cos(tLatRad) * cos(pLatRad)) * (sin(deltaLong / 2) * sin(deltaLong / 2));
    double c = 2 * atan2(sqrt(var), sqrt(1 - var));

    return RADIUS_EARTH * c;

}


/*
double
Autopilot_Interface::
predictDistance( const float &ourPos, const predictedCollision &otherPos)
{
	double aLatRad = ourAircraft.lat[2]*TO_RADIANS;
	double bLatRad = otherAircraft.lat[2]*TO_RADIANS;
	double deltaLat = (otherAircraft.lat[2] - ourAircraft.lat[2])* TO_RADIANS;
	double deltaLong = (otherAircraft.lon[2] - otherAircraft.lon[2])* TO_RADIANS;

	double var = sin(deltaLat / 2) * sin(deltaLat / 2) + (cos(aLatRad) * cos(bLatRad)) * (sin(deltaLong / 2) * sin(deltaLong / 2));
	double c = 2 * atan2(sqrt(var), sqrt(1 - var));
	double distance = c * RADIUS_EARTH;

	
	double bubbleRadius = t*.5 + 5;
	// nested if to check the two collision points to check if they are close enough to tell if there is a collision 

	double rH = abs(ourAircraft.Hdg - otherAircraft.Hdg);
	cout << "Relative Heading " << t << ": " << rH << "\n" << "Distance " << t << ": " << distance << "\n\n";
		
}


*/


















//This function will tell the pixhawk to travel to this waypoint next
//This will override the previous next waypoint
void
Autopilot_Interface::
setCurrentWaypoint( const uint16_t &waypointNum )
{
	printf("Autopilot_Interface::setCurrentWaypoint()\n");
	mavlink_mission_set_current_t newWaypoint;

	//Set parameters for the message
	newWaypoint.seq = waypointNum;
	newWaypoint.target_system = system_id;
	newWaypoint.target_component = companion_id;


	//writing_status = true;

	mavlink_message_t message;
	mavlink_msg_mission_set_current_encode(system_id, companion_id, &message, &newWaypoint);
	//writing_status = false;	


        if ( write_message(message) <= 0 )
	fprintf(stderr,"WARNING: could not send MAV_CMD_DO_SET_SERVO \n");

	//printf("The pixhawk is now traveling to waypoint %u\n", newWaypoint.seq);
}


//mavlink_mission_item_int_t
bool
Autopilot_Interface::
Request_Waypoints()
{

//This function requests a mission according to this guide: http://qgroundcontrol.org/mavlink/waypoint_protocol

	if (reading_waypoint_status != 0) {
		printf("ERROR: WAYPOINTS ALREADY REQUESTED\n");
		return NULL;
	}

	printf("REQUESTING WAYPOINTS\n");
	reading_waypoint_status = true;

	//Request mission
	//mavlink_mission_request_list_t missionRequest;
	mavlink_message_t message; 

	mavlink_msg_mission_request_list_pack(system_id, companion_id, &message, system_id, companion_id, 0);
	if ( write_message(message) <= 0 )
	fprintf(stderr,"WARNING: could not send MAV_CMD_REQUEST_WAYPOINTS \n");

	//Receive waypoint count
	int counter = 0;
	mavlink_mission_item_int_t missionItem = current_messages.mavlink_mission_item;
	mavlink_mission_count_t missionCount = current_messages.mavlink_mission_count;
	missionCount.count = 0;
	
	
	bool waypointError = false;
	uint64_t count_time = current_messages.time_stamps.mavlink_mission_count;
	// This is a catch for errors
	//Give the autopilot some time to receive the message
	//while (missionCount.count < 0.1 && counter < 1000) {
	while (count_time == current_messages.time_stamps.mavlink_mission_count) {
		printf("error catch\n");
		missionCount = current_messages.mavlink_mission_count;
		counter++;
		usleep(500000);
	}

	// In the case that the waypoint was not received while waiting in the previous function, then return an error
	if (missionCount.count < 0.1 && !time_to_exit) {
		printf("TIMEOUT: MISSION COUNT NOT RECEIVED\n");
		waypointError = true;
	}


	//printf("MissionCount > 1000 problem here\n");
	printf("MissionCount: %u\n", missionCount.count);

	// if (missionCount.count > 1000) {

	// 	waypointError = true;
	// 	printf("WAYPOINT COUNT NOT RECEIVED: TRY AGAIN\n");
	// 	error_counter++;
	// }

	if (waypointError == true) {
		printf("ERROR: WAYPOINT COUNT NOT RECEIVED\n");
		reading_waypoint_status = false;
		return waypointError;
	}

	//For some reason this function needs to end if the waypoint message is to be read
	
	Receive_Waypoints();

	mavlink_message_t message_ack;
	mavlink_mission_ack_t ACK;
	ACK.type =0;

	mavlink_msg_mission_ack_encode(system_id, companion_id, &message_ack, &ACK);
	if ( write_message(message_ack) <= 0 )
		fprintf(stderr,"WARNING: could not send ACK \n");


	printf("Current mission size: %lu\n", currentMission.size());
	reading_waypoint_status = false;

	//printf("reading_waypoint_status: %c\n",reading_waypoint_status );
}



bool
Autopilot_Interface::
Receive_Waypoints() {

	printf("RECEIVING WAYPOINTS\n");
	//---------------------------------------------------------------------
	//Send request then receive each waypoint of the mission
	//---------------------------------------------------------------------

	int seqNum = 0; // The current waypoint we are requesting and receiving
 	mavlink_message_t message;
	mavlink_mission_item_int_t missionItem = current_messages.mavlink_mission_item;
	mavlink_mission_count_t missionCount = current_messages.mavlink_mission_count;

	printf("!!!!Number of waypoints: %i!!!!!!\n\n", missionCount.count);

	for (seqNum = 0;  seqNum < missionCount.count;  seqNum++) {

 
		//---------------------------------------------------------------------------------------
		//Request waypoint
		//---------------------------------------------------------------------------------------

		//mavlink_msg_mission_request_pack(system_id, companion_id, &message, system_id, companion_id, seqNum, 0);
		mavlink_msg_mission_request_int_pack(system_id, companion_id, &message, system_id, companion_id, seqNum, 0);

		if ( write_message(message) <= 0 )
			fprintf(stderr,"WARNING: could not send MAV_CMD_REQUEST_WAYPOINTS \n");
		
		usleep(30000);//Wait for response


		//--------------------------------------------------------------------------------------
		//Receive waypoint
		//--------------------------------------------------------------------------------------

		missionItem = current_messages.mavlink_mission_item;
		//printf("missionItem.seq: %u\n",missionItem.seq);

		//Account for whether the mission already has elements in it
		if (currentMission.size() > seqNum) { 
			currentMission[seqNum] = create_waypoint(missionItem.x, missionItem.y, missionItem.z, missionItem.seq, missionItem.param2); 
			}
		else { 
			currentMission.push_back(create_waypoint(missionItem.x, missionItem.y, missionItem.z, missionItem.seq, missionItem.param2)); 
			//printf("In else statement\n");
			}


		//Makes sure the waypoint is written to the right vector element and isn't skipped


		int i = 0;		
		
		while (seqNum != currentMission[seqNum].seq && !time_to_exit) {

			missionItem = current_messages.mavlink_mission_item;
			currentMission[seqNum] = missionItem;
			usleep(1000);		
			i++;
			/* Accounts for a frozen code. Needs improvement
			if (i=100000) { //If the autopilot does not receive the waypoint in this amount of time, an error has occured so start over.

				printf("hey %i\n", i);
				reading_waypoint_status = false;
				Request_Waypoints();
				break;
				
			}*/

		}

		printf("WP Params:\n Lat: %d\n Lon: %d\n seq: %d\n", currentMission[seqNum].x,currentMission[seqNum].y,currentMission[seqNum].seq);
		//printf("seqNum: %i\n", seqNum);
		//printf("current mission seq: %i\n", currentMission[seqNum].seq);

	}
	

}



//-------------------------------------------------------------------
//Collision avoidance starter functions
//-------------------------------------------------------------------


aircraftInfo::
aircraftInfo() {

	//Initializes all values to zero
	//Descriptions are in the aircraftInfo struct
	lat [3] = {0};
	lon [3] = {0};
	gpsTime [3] = {0};
	alt [3] = {0};
	velocityX [2] = {0};
	velocityY [2] = {0};
	xAcc = 0;
	yAcc = 0;
	futureDistx [3] = {0};
	futureDisty [3] = {0};
	Hdg [2] = {0};
	safetyBubble = 50;  //Roughly 300 ft
	priority = 0;

}

void
Autopilot_Interface::
start_collision_avoidance()
{

	int result;
	

	printf("\n Autopilot_Interface:: start_collision_avoidance() \n");
	//Start thread
	result = pthread_create( &CA_tid, NULL, &start_collision_avoidance_thread, this );
	if ( result ) throw result;

	//printf("\nSTART COLLISION AVOIDANCE THREAD\n");
}

void
Autopilot_Interface::
collision_avoidance_begin()
{

	if ( CA_status != 0 )
	{
		fprintf(stderr,"Collision Avoidance thread already running\n");
		return;
	}
	else
	{
		CA_predict_thread();
		return;
	}

}


void
Autopilot_Interface::
insert_waypoint ( mavlink_mission_item_int_t &newWaypoint, const uint16_t &desiredSeqNumber ) {

		//-----------------------------------------------------------------------
		//Insert waypoint
		//-----------------------------------------------------------------------

		printf("Autopilot_Interface::insert_waypoint()\n");
		//printf("Current mission size: %lu\n", currentMission.size());

		//5-13-2021
			currentMission.push_back(currentMission[0]); //Takes first element and copies it to last for dummy data
			int endVal = currentMission.size() - desiredSeqNumber;
			
			
			for (int i = 0; i < endVal; i++) {
				//printf("\nLatitude before %f\n", currentMission.rbegin()[i].x);
				currentMission.rbegin()[i] = currentMission.rbegin()[(i+1)]; //Shifts waypoints over until the inserted waypoint is reached
				currentMission.rbegin()[i].seq = currentMission.size() - (i+1);

			}
	
		newWaypoint.z = currentMission[endVal].z; //I chose endVal because it is guaranteed to always be there
		newWaypoint.seq = desiredSeqNumber;

		currentMission[desiredSeqNumber] = create_waypoint(newWaypoint.x, newWaypoint.y, newWaypoint.z, desiredSeqNumber, 15);
		
			//int endVal = currentMission.size() - desiredSeqNumber;	
			//desiredSeqNumber = 3;
			//for(int i=0; i<3; i++)
				//currentMission.pop_back(); //Remove the last item
			//newWaypoint.z = currentMission[endVal].z; //I chose endVal because it is guaranteed to always be there
			//newWaypoint.seq = desiredSeqNumber;
			//Put avoidWaypoint at the end as a current waypoint
			//currentMission.push_back(create_waypoint(newWaypoint.x, newWaypoint.y, newWaypoint.z, desiredSeqNumber, 15));
			
		//printf("LastOldWaypoint: %f, %f\n", currentMission[currentMission.size()-1].x, currentMission[currentMission.size()-1].y);
			//currentMission.push_back(create_waypoint(34.110088647446084, -117.76428455795262, 45.720001, desiredSeqNumber, 15));
			//currentMission.push_back(create_waypoint(34.110088, -117.764284, 45.720001, currentMission.size(), 15));
			//currentMission.push_back(currentMission[1]);
			//currentMission[currentMission.size()-1].seq = currentMission.size()-1;
		//printf("NewWaypoint: %f, %f\n", currentMission[currentMission.size()-1].x, currentMission[currentMission.size()-1].y);
	//Write avoid waypoint within mission
		printf("Seq: %d, %d, %d, %d, %d, %d\n", currentMission[0].seq, currentMission[1].seq, currentMission[2].seq, currentMission[3].seq, currentMission[4].seq, currentMission[5].seq);
	
	
	/*Display new mission
	for (int count = 0; count < currentMission.size(); count++)
		printf("Waypoint %d\n Lat: %d, Lon %d\n",currentMission[count].seq,currentMission[count].x,currentMission[count].y);
	*/write_waypoints(currentMission, desiredSeqNumber);

}


void
Autopilot_Interface::
updateAircraftInfo(aircraftInfo & aircraftObj, mavlink_global_position_int_t gpos, mavlink_adsb_vehicle_t adsb, int plane)
{

	float groundSpeed;

	//Shift previous data backwards
	aircraftObj.lat[2] = aircraftObj.lat[1];
	aircraftObj.lon[2] = aircraftObj.lon[1];
   	aircraftObj.gpsTime[2] = aircraftObj.gpsTime[1];
	aircraftObj.lat[1] = aircraftObj.lat[0];
	aircraftObj.lon[1] = aircraftObj.lon[0];
	aircraftObj.gpsTime[1] = aircraftObj.gpsTime[0];

	aircraftObj.Hdg[1] = aircraftObj.Hdg[0];

	aircraftObj.velocityX[1] = aircraftObj.velocityX[0];
	aircraftObj.velocityY[1] = aircraftObj.velocityY[0];
	aircraftObj.vTan[1]      = aircraftObj.vTan[0];

	//If working with the controlled plane, grab from internal
	if (plane == 1) 
	{ 
		aircraftObj.lat[0] = gpos.lat / 1E7;
		aircraftObj.lon[0] = gpos.lon / 1E7;

		aircraftObj.gpsTime[0] = gpos.time_boot_ms;

		// Derive velocity and heading from distance vectors
		mavlink_mission_item_int_t distVec = distanceVectors(aircraftObj.lat[0], aircraftObj.lon[0], aircraftObj.lat[1], aircraftObj.lon[1]);
		
		aircraftObj.velocityX[0] = distVec.x / ((aircraftObj.gpsTime[0] - aircraftObj.gpsTime[1]) / 1000.0);
		aircraftObj.velocityY[0] = distVec.y / ((aircraftObj.gpsTime[0] - aircraftObj.gpsTime[1]) / 1000.0);
		aircraftObj.Hdg[0] = atan2(distVec.y, distVec.x);

		//printf("Aircraft dist x: %f\n", distVec.x);
		//printf("Aircraft dist y: %f\n", distVec.y);
		
		//printf("Aircraft Velocity x: %f\n", aircraftObj.velocityX[0]);
		//printf("Aircraft Velocity y: %f\n", aircraftObj.velocityY[0]);
		//printf("Time diff: %f\n", (aircraftObj.gpsTime[0] - aircraftObj.gpsTime[1]) / 1000.0);
/* Doing heading and velocity seems to be giving errors
		aircraftObj.velocityX[0]  = gpos.vx  / 100.0; //NED Frame: x = North = Latitude
		aircraftObj.velocityY[0]  = gpos.vy  / 100.0; //           y = East  = longitude
		aircraftObj.Hdg[0]        = gpos.hdg / 100.0;
*/

		aircraftObj.vTan[0] = sqrt(pow(aircraftObj.velocityX[0],2) + pow(aircraftObj.velocityY[0], 2));

		//printf("update gpos.lat: %f\n", gpos.lat/1E7);
		//printf("update gpos.velx: %f\n", gpos.vx/100.0);
		//printf("update gpos.vely: %f\n", gpos.vy/100.0);

	}
	
	// else if working with other plane grab from ADS-B
	else 
	{ 
		aircraftObj.lat[0] = adsb.lat / 1E7;
		aircraftObj.lon[0] = adsb.lon / 1E7;

		// Derive velocity from heading (NED frame)
		aircraftObj.Hdg[0] = adsb.heading / 100.0;
		groundSpeed = adsb.hor_velocity / 100.0;

		aircraftObj.velocityX[0] = groundSpeed * cos(aircraftObj.Hdg[0] * TO_RADIANS);
		aircraftObj.velocityY[0] = groundSpeed * sin(aircraftObj.Hdg[0] * TO_RADIANS);
		aircraftObj.vTan[0] = sqrt(pow(aircraftObj.velocityX[0],2) + pow(aircraftObj.velocityY[0], 2));
		//printf("update adsb.lat: %f\n", adsb.lat/1E7);
	}


	//Update our aircraft acceleration (acc) A
	aircraftObj.xAcc = (aircraftObj.velocityX[0] - aircraftObj.velocityX[1]);//xAcc
	aircraftObj.yAcc = (aircraftObj.velocityY[0] - aircraftObj.velocityY[1]);//yAcc
	//printf("aircraftObj.xAcc: %f\n", aircraftObj.xAcc);
}


void
Autopilot_Interface::
CA_predict_thread()
{

	//printf("Autopilot_Interface::CA_predict_thread()\n");
	//------------------------------------------------
	//Start detect/predict loop
	//------------------------------------------------
	CA_status = true;
	int AVOID_DELAY = 0;
	predictedCollision collision;
	mavlink_global_position_int_t gpos = current_messages.global_position_int;
	mavlink_adsb_vehicle_t adsb = current_messages.adsb_vehicle_t;
	mavlink_adsb_vehicle_t dummydataADSB;
	mavlink_global_position_int_t dummydataGPOS;
	double vx;
	double vy;
	//Prediction will occur once both aircraft vectors are updated
	bool ourUpdated = false;
	bool otherUpdated = false;
	int fractionSinceUpdate = 0;

	while ( ! time_to_exit ) 
	{
		///---------------------------------
		///
      		/// Gather All Parameters for Our Plane, such as lat,lon,velocity,acceleration, Heading,
      		///
      		///----------------------------------

		//update stored messages
		gpos = current_messages.global_position_int;
		adsb = current_messages.adsb_vehicle_t;

		//If the current stored position is not equal to the gpos position i.e. the position message has been updated, then these vectors can be 			updated. Otherwise wait a little longer to update the array
		/*
		printf("\n!!!!!Here Our Plane!!!!!!!!\n");
		printf("Ouraircraft Update: %s\n", ourUpdated?"true":"false");
		printf("our lat %f\n", ourAircraft.lat[0]);
		printf("our lon %f\n", ourAircraft.lon[0]);
		printf("gpos lat %f\n", gpos.lat/1E7);
		printf("gpos lon %f\n", gpos.lon/1E7);
		*/
		//Update check for the controlled aircraft
		//If our position has changed then updateAircraftInfo(A)
		if ( fabs(ourAircraft.lat[0] - (double) gpos.lat/1E7) > 0.0000001 || fabs(ourAircraft.lon[0] - (double) gpos.lon/1E7) > 0.0000001 ) 
		{
			updateAircraftInfo(ourAircraft, gpos, adsb, 1);

			//Now that we have updated the position, lets inform the other functions
			ourUpdated = true;
		}

		//If other position has changed we received new info from adsb; updateAircraftInfo(B)
		if ( fabs(otherAircraft.lat[0] - (double) adsb.lat/1E7) > 0.000001 || fabs(otherAircraft.lon[0] - (double) adsb.lon/1E7) > 0.000001 )
		{
			//Update other aircraft
			updateAircraftInfo(otherAircraft, gpos, adsb, 2);
			
			//Now that we have updated the position, lets inform the other functions
			otherUpdated = true;
			fractionSinceUpdate = 0;
			//printf("logged!\n");
		}
		
		//If other position has not changed but speed = 0; It is hovering
		else if(adsb.hor_velocity == 0)
		{
			updateAircraftInfo(otherAircraft, gpos, adsb, 2);
			otherUpdated = true;
			fractionSinceUpdate = 0;
		}
		
		//Else we have not received new info from adsb; disregard sample data
		//If the position has not been updated in the current messages then start counting
		if ( fabs(otherAircraft.lat[0] - (double) adsb.lat/1E7) < 0.00001 && fabs(otherAircraft.lon[0] - (double) adsb.lon/1E7) < 0.00001)  
		{
			fractionSinceUpdate++; //Time = .3333*fractionSinceUpdate 
		}
		
		/*
		printf("\n???There Other Plane?????\n");
		printf("Otheraircraft Update: %s\n", otherUpdated?"true":"false");
		printf("other lat  %f\n",otherAircraft.lat[0]);
		printf("other lon  %f\n",otherAircraft.lon[0]);
		printf("adsb lat  %f\n",adsb.lat/1E7);
		printf("adsb lon  %f\n",adsb.lon/1E7);
		*/
		
		//printf("log criteria dist: %lf\n",(double) abs(otherAircraft.lat[0] - (double) adsb.lat/1E7));
		/*if ( fractionSinceUpdate > 9 ) 
		{ 
			//Get rid of zeros in otherAircraft vector[2]
			updateAircraftInfo(otherAircraft, dummydataGPOS, adsb, 2);
		}*/


		//printf("Time since update: %i\n",fractionSinceUpdate);
		//printf("Ours updated? %d\nOther updated? %d\n", ourUpdated, otherUpdated);
		//printf("First condition satisfied: %d\n Second condition satisfied: %d\n",(ourUpdated && otherUpdated), (fractionSinceUpdate > 9 && ourUpdated == true));

		//Predict now happens if both the ownships and the other aircraft have updated
		if ((ourUpdated == true && otherUpdated == true)) 
		{ /*9 is 3 seconds since update speed is 1/3 of a second*/
				

			//printf("1/3 seconds since ADS-B update: %i", fractionSinceUpdate);
		
			//log all the new position data
			addToFile("New point ~~~~~~~~~~","");
			addToFile(convertToString(ourAircraft.lat[2]),"last lattitude");
			addToFile(convertToString(ourAircraft.lon[2]),"last longitude");
			addToFile(convertToString(ourAircraft.lat[1]),"second to last Lattitude");
			addToFile(convertToString(ourAircraft.lon[1]),"second to last Longitude");
			addToFile(convertToString(ourAircraft.lat[0]),"Our Lattitude");
			addToFile(convertToString(ourAircraft.lon[0]),"Our Longitude");
			addToFile(convertToString(otherAircraft.lat[1]),"Other second Lattitude");
			addToFile(convertToString(otherAircraft.lon[1]),"Other second Longitude");
			addToFile(convertToString(otherAircraft.lat[0]),"Other Lattitude");
			addToFile(convertToString(otherAircraft.lon[0]),"Other Longitude");
			addToFile(convertToString(ourAircraft.velocityX[0]),"Our X Velocity");//NED
			addToFile(convertToString(ourAircraft.velocityY[0]),"Our Y Velocity");//NED
			addToFile(convertToString(otherAircraft.velocityX[0]),"Other X Velocity");//NED
			addToFile(convertToString(otherAircraft.velocityY[0]),"Other Y Velocity");//NED
			//printf("Done Logging\n");

			printf("\nPREDICT\n");			//Predict using the logged point
			collision = CA_Predict(ourAircraft, otherAircraft, ourAircraft.Hdg[0], ourAircraft.Hdg[1]);
			//collision.collisionDetected == true;
			
			printf("Collision predicted? %d\n", collision.collisionDetected);
		
			//Avoid if necessary
			if (collision.collisionDetected == true && AVOID_DELAY <=1 ) {
				printf("Avoid function\n");
				AVOID_DELAY = 90; //This is not a great way to keep multiple points from being written to the pixhawk 
				CA_Avoid(ourAircraft, otherAircraft, collision);
			}

			if(collision.collisionDetected)
				printf("Distance between aircraft: %f\n", gpsDistance(ourAircraft.lat[0], ourAircraft.lon[0], otherAircraft.lat[0], otherAircraft.lon[0]));

			if (AVOID_DELAY > 0) {
				AVOID_DELAY--;
				printf("Waiting for aircraft to avoid...\n");
				//addToFile("", "Waiting to avoid...");
			}
			
			//Now that we have predicted we can revert the update boolians
			ourUpdated = false;
			otherUpdated = false;
			if(collision.collisionDetected)
				printf("end predict with other updated = %d\n",otherUpdated);
			
		}//End predict


		//Wait a fourth of a second before updating the next position
		usleep(250000);
		//usleep(1000000);
	}

	CA_status = false;

}

/* Function to find the distance vector between aircraft in NED frame
vector<double>
Autopilot_Interface::
GPS2NEDvec(double currentLat, double currentLon, double otherLat, double otherLon)
{

// Math from http://www.movable-type.co.uk/scripts/latlong.html

// Find delta in positions
double deltaLat = (otherLat - currentLat) * TO_RADIANS;
double deltaLon = (otherLon - currentLon) * TO_RADIANS;

double latRad = currentLat * TO_RADIANS;


//Using the math from the website, assume dlat = 0 and solve dlon
double varDlon =  pow( cos(latRad), 2) * pow( sin(deltaLon/2), 2);
double dy = 2 * atan2( sqrt(varDlon), sqrt(1-varDlon)) * RADIUS_EARTH * 1000.0;

// Calculate Dlat assuming dlon = 0
double varDlat = pow( sin(deltaLat/2), 2);
double dy = 2 * atan2(sqrt(varDlat), sqrt(1-varDlat)) * RADIUS_EARTH * 1000.0;

if (deltaLon < 0) { dx = -dx; }
if (deltaLat < 0) { dy = -dy; }

vector<double> distVec = { dx, dy };

return distVec;

}This may be more accurate than distanceVectors function TODO: Test both for accuracy*/

predictedCollision
Autopilot_Interface::
CA_Predict(aircraftInfo & aircraftA, aircraftInfo & aircraftB, double current_HdgA, double last_HdgA)
{
	printf("Autopilot_Interface::CA_Predict()\n");
	float fps = 15.0; //fps meaning future points
	double rH; // for relative Heading of the planes
	float t;
	double accDirA;
	double accDirB;

	double accNUnitA [2];
	double accNUnitB [2];
	double accNMagA;
	double accNMagB;

	double accTUnitA [2];
	double accTUnitB [2];
	double accTMagA;
	double accTMagB;

	double RmagA;
	double RmagB;

	double omegaA;
	double omegaB;

	double RvCA [2];
	double RvCB [2];
	
	double collisionDist;

	//-----------------------------------------------------------------------------
	// Set up equations of predicted motion for both aircraft
	//-----------------------------------------------------------------------------
	
	//convert aircraft heading to degrees
	current_HdgA = current_HdgA * 180 / 3.1415;
	last_HdgA = last_HdgA * 180 / 3.1415;
	aircraftB.Hdg[0] = aircraftB.Hdg[0] * 180 / 3.1415;
	aircraftB.Hdg[1] = aircraftB.Hdg[1] * 180 / 3.1415;
	
   	/*aircraftA.Hdg[0] = atan2(aircraftA.velocityY[0], aircraftA.velocityX[0]) * 180.0/3.1415;
   	aircraftA.Hdg[1] = atan2(aircraftA.velocityY[1], aircraftA.velocityX[1]) * 180.0/3.1415;
   	aircraftB.Hdg[0] = atan2(aircraftB.velocityY[0], aircraftB.velocityX[0]) * 180.0/3.1415;
   	aircraftB.Hdg[1] = atan2(aircraftB.velocityY[1], aircraftB.velocityX[1]) * 180.0/3.1415;*/

	if (current_HdgA > last_HdgA)
		accDirA = current_HdgA + 90.0;
	if (current_HdgA < last_HdgA) 					
		accDirA = current_HdgA - 90.0;
	if ( fabs(current_HdgA - last_HdgA) > 180)  
		accDirA = accDirA + 180.0;
	if (aircraftB.Hdg[0] > aircraftB.Hdg[1]) 					
		accDirB = aircraftB.Hdg[0] + 90.0;
	if (aircraftB.Hdg[0] < aircraftB.Hdg[1]) 					
		accDirB = aircraftB.Hdg[0] - 90.0;
	if ( fabs(aircraftB.Hdg[0] - aircraftB.Hdg[1]) > 180)  
		accDirB = accDirB + 180.0;

	
	//printf("aircraftA velocityX: %f\n", aircraftA.velocityX[0]);
	//printf("aircraftA velocityY: %f\n", aircraftA.velocityY[0]);
	//printf("aircraftA heading 1: %f\n", aircraftA.Hdg[0]);
	//printf("aircraftA heading 2: %f\n", aircraftA.Hdg[1]);
	//printf("aircraftA accDirA: %f\n", accDirA);
	//printf("aircraftA vTan: %f\n", aircraftA.vTan[0]);
	

	//Unit vector in direction of theoretical acceleration
	accNUnitA[0] = cos(accDirA * TO_RADIANS); //X latitude
	accNUnitA[1] = sin(accDirA * TO_RADIANS); //Y longitude
	accNUnitB[0] = cos(accDirB * TO_RADIANS); //X latitude
	accNUnitB[1] = sin(accDirB * TO_RADIANS); //Y longitude

	//printf("update accNUnitA: %f\n", accNUnitA[0]);
	//printf("update gpos.lat: %f\n", gpos.lat/1E7);

	//printf("accNUnitA[0] %f\n", accNUnitA[0]);
	//printf("accNUnitA[1] %f\n", accNUnitA[1]);
	//printf("aircraftA.xAcc %f\n", aircraftA.xAcc);
	//printf("aircraftA.yAcc %f\n", aircraftA.yAcc);
	//Dot product of acceleration vector against theoretical normal acceleration vector
	accNMagA = accNUnitA[0]*aircraftA.xAcc + accNUnitA[1]*aircraftA.yAcc;
	accNMagB = accNUnitB[0]*aircraftB.xAcc + accNUnitB[1]*aircraftB.yAcc;


	//printf("accNMagA: %f\n", accNMagA);
	//printf("accNMagB: %f\n", accNMagB);
	//Unit vector of acceleration tangent to the direction of motion
	accTUnitA[0] = cos(current_HdgA * TO_RADIANS);
	accTUnitA[1] = sin(current_HdgA * TO_RADIANS);
	accTUnitB[0] = cos(aircraftB.Hdg[0] * TO_RADIANS);
	accTUnitB[1] = sin(aircraftB.Hdg[0] * TO_RADIANS);

	//Unit vector magnitude in direction of motion (achieved through dot product)
	accTMagA = accTUnitA[0]*aircraftA.xAcc + accTUnitA[1]*aircraftA.yAcc;
	accTMagB = accTUnitB[0]*aircraftB.xAcc + accTUnitB[1]*aircraftB.yAcc;
	
	//printf("accTUnitA[0] %f\n", accTUnitA[0]);
	//printf("accTUnitA[1] %f\n", accTUnitA[1]);
	//printf("accTMagA %f\n", accTMagA);
	//Dot product of acceleration vector against theore
	//printf("accNMagA %f\n",	accNMagA);

	//Find radius of theoretical circle of turning
	RmagA = pow(aircraftA.vTan[0], 2) / accNMagA;
	RmagB = pow(aircraftB.vTan[0], 2) / accNMagB;

	if (fabs(accNMagA) < 0.0001) 
		RmagA = 0.0;
	if (fabs(accNMagB) < 0.0001) 
		RmagB = 0.0;
	//printf("RmagA %f\n", RmagA);

	// Use acceleration vector to find vector from vehicle to center
	RvCA[0] = RmagA * accNUnitA[0];
	RvCA[1] = RmagA * accNUnitA[1];
	RvCB[0] = RmagB * accNUnitB[0];
	RvCB[1] = RmagB * accNUnitB[1];


	//printf("RvCA[0] %f\n", RvCA[0]);
	//printf("RvCA[1] %f\n", RvCA[1]);

	// Find magnitude of angular rotation (rad/s)
	omegaA = aircraftA.vTan[0] / RmagA;
	omegaB = aircraftB.vTan[0] / RmagB;
	if (fabs(RmagA) < 0.0001) 
		omegaA = 0.0;
	if (fabs(RmagB) < 0.0001) 
		omegaB = 0.0;


	//printf("aircraftA omega: %f\n", omegaA);
	//printf("aircraftB omega: %f\n", omegaB);
	// find direction of rotation. R_center_to_vehicle cross Velocity_Vec
	if (-1.0*(RvCA[1]*aircraftA.velocityX[0] - RvCA[0]*aircraftA.velocityY[0]) < 0.0) 
		omegaA = fabs(omegaA);
	else 
		omegaA = -fabs(omegaA);
	if (-1.0*(RvCB[1]*aircraftB.velocityX[0] - RvCB[0]*aircraftB.velocityY[0]) < 0.0) 
		omegaB = fabs(omegaB);
	else 
		omegaA = -fabs(omegaB);

	//printf("omegaA %f\n", omegaA);
	//--------------------------------------------------------
	// Predict along formulated path for fps seconds
	//--------------------------------------------------------
	float thetaA_i;
	float thetaB_i;
	float RvecA_i [2];
	float RvecB_i [2];
	float RpredictB [2];
	float RpredictA [2];
	int bubbleRadius;
	double predictedDistance;
	double futureHdgB;

	for (t = 0.0; t < fps; t=t+1.0)
	{
		//printf("\nTimestep %f\n", t);
		//---------------------------------------------------------------------------------------------------------
		//	Predict future positions
		//---------------------------------------------------------------------------------------------------------
		thetaA_i = (accDirA+180.0) * TO_RADIANS + omegaA * t;
		thetaB_i = (accDirB+180.0) * TO_RADIANS + omegaB * t;
		//printf("thetaA_i %f\n", thetaA_i);

		// Vector from center of rotation to new predicted position
		RvecA_i[0] = RmagA*cos(thetaA_i);
		RvecA_i[1] = RmagA*sin(thetaA_i);
		RvecB_i[0] = RmagB*cos(thetaB_i);
		RvecB_i[1] = RmagB*sin(thetaB_i);

		//printf("RvecA_i[0] %f\n", RvecA_i[0]);
		//printf("RvecA_i[1] %f\n", RvecA_i[1]);

		// Vector from vehicle to predicted position
		RpredictA[0] = RvCA[0] + RvecA_i[0];
		RpredictA[1] = RvCA[1] + RvecA_i[1];
		RpredictB[0] = RvCB[0] + RvecB_i[0];
		RpredictB[1] = RvCB[1] + RvecB_i[1];
		
		//printf("RpredictA[0] %f\n", RpredictA[0]);
		//printf("RpredictA[1] %f\n", RpredictA[1]);
		
		//determine the heading of B at the predicted location
		futureHdgB = 90 + (270 - (aircraftB.Hdg[0] + thetaB_i * 180 / 3.1415));

		//Creates a future position item based on the current position and future distance
		mavlink_mission_item_int_t ourFuturePos = NewAvoidWaypoint(RpredictA[0], RpredictA[1], aircraftA);
		mavlink_mission_item_int_t otherFuturePos = NewAvoidWaypoint(RpredictB[0], RpredictB[1], aircraftB);
		//printf("Predicted position: (%f, %f)\n", ourFuturePos.x, ourFuturePos.y);

		//printf("ourFuturePos.x %f\n", ourFuturePos.x);
		//printf("ourFuturePos.y %f\n", ourFuturePos.y);
		//Log A
		addToFile(convertToString(t), "Time interval");
		addToFile(convertToString(ourFuturePos.x), "ourFuturePos.x");
		addToFile(convertToString(ourFuturePos.y), "ourFuturePos.y");
		//Log B
		addToFile(convertToString(otherFuturePos.x), "otherFuturePos.x");
		addToFile(convertToString(otherFuturePos.y), "otherFuturePos.y");

		//----------------------------------------------------------------------------------------------
		//
		//	Compares the future positions of each plane to see if they are within the safety bubble
		//
		//----------------------------------------------------------------------------------------------

		bubbleRadius = aircraftA.safetyBubble;
		predictedDistance = gpsDistance(ourFuturePos.x, ourFuturePos.y, otherFuturePos.x, otherFuturePos.y);
		collisionDist = sqrt(pow((ourFuturePos.x - aircraftA.lat[0]), 2) + pow((ourFuturePos.y - aircraftA.lon[0]), 2)); //distance between current A loc and A loc at time of collision
		
		//Log
		addToFile(convertToString(predictedDistance), "Future predicted distance");
		//printf("Predicted distance between planes: %f m\n", predictedDistance);
		collisionPoint.collisionDetected = false;

		//	printf("NEED TO CHANGE: Collision detected if predicted distance <=1\n");
		if (predictedDistance <= bubbleRadius) 
		{ 
			//MODIFIED FOR LOGGING PURPOSES!!! NOT OFFICIAL CODE

			collisionPoint.collisionDetected = true;
			collisionPoint.timeToCollision = t;
			collisionPoint.distance = collisionDist;
			collisionPoint.angle = 90 - acos(collisionDist / (2*RmagA)); //angle between current A heading and collision point (theta)
			if (current_HdgA < last_HdgA) //error when crossing northern line (0 degrees = 360 degrees)
				collisionPoint.angle = -1 * collisionPoint.angle; //if plane turning left, theta is negative
			collisionPoint.relativeHeading = rH;
			collisionPoint.headingB = futureHdgB; //may change
			collisionPoint.locationA.x = ourFuturePos.x;
			collisionPoint.locationA.y = ourFuturePos.y;
			collisionPoint.locationB.x = otherFuturePos.x;
			collisionPoint.locationB.y = otherFuturePos.y;
			
			//printf("Collision detected\n");
			addToFile("COLLISION DETECTED", "");
			//printf("Predicted distance between planes at collision: %f m\n", predictedDistance);
			addToFile(convertToString(predictedDistance), "Distance at predicted collision");
			return collisionPoint;
		}
	}
	
	//printf("Autopilot_Interface::CA_Predict():Prediction function ended\n\n");
	return collisionPoint;
}



double
relHdg(double currentHdg, double otherHdg)
{

	double tmpHdg = otherHdg - currentHdg;

	double relativeHdg;
	if (tmpHdg > 180) 
		relativeHdg = tmpHdg - 360;
	else if (tmpHdg < -180) 
		relativeHdg = tmpHdg + 360;
	else 
		relativeHdg = tmpHdg; 

	return relativeHdg;
}

//currently only optimized for sidestep strategy
/*predictedCollision 
considerStrategy(predictedCollision collisionPoint, aircraftInfo & aircraftA, aircraftInfo & aircraftB, double headingA, double altA, double velA)
{
	predictedCollision newCollision;
	double distA = collisionPoint.distance;
	
	double futureLatA = asin(sin(aircraftA.lat[0]) * cos(distA / RADIUS_EARTH) +
				 cos(aircraftA.lat[0]) * sin(distA / RADIUS_EARTH) * cos(headingA * TO_RADIANS));
	double futureLonA = aircraftA.lon[0] + atan2(sin(headingA * TO_RADIANS) * sin(distA / RADIUS_EARTH) * cos(aircraftA.lat[0]), 
						     cos(distA / RADIUS_EARTH) * sin(aircraftA.lat[0]) * sin(futureLatA));
	double futureAltA = 
	//seconds it takes for A to get to new point
	//based off of current velocity(assuming no acceleration)
	float time = distA / sqrt(pow(aircraftA.velocityX[0], 2) + pow(aircraftA.velocityY[0], 2));
	
	//iterative distance of future A and B points based on travel time and velocity
	double distAIter;
	double distBIter;
	
	//needs to be adjusted to test points along predicted curve instead of straight line, depending on movement of each aircraft
	//run prediction algorithm on new avoid point
	//newCollision = Autopilot_Interface::CA_Predict(aircraftA, aircraftB, headingA, aircraftA.Hdg[0]);
	
	//iteratively determine if A and B intersect paths at the same time(creates new collision) at 1 sec intervals
	for(float i = time, i > 0, i = i - 1.0f)
	{	
		//predicted location of A at i seconds in the future
		distAIter = i * sqrt(pow(aircraftA.velocityX[0], 2) + pow(aircraftA.velocityY[0], 2));
		futureLatA = asin(sin(aircraftA.lat[0]) * cos(distAIter / RADIUS_EARTH) +
				 cos(aircraftA.lat[0]) * sin(distAIter / RADIUS_EARTH) * cos(headingA * TO_RADIANS));
		futureLonA = aircraftA.lon[0] + atan2(sin(headingA * TO_RADIANS) * sin(distAIter / RADIUS_EARTH) * cos(aircraftA.lat[0]), 
						     cos(distAIter / RADIUS_EARTH) * sin(aircraftA.lat[0]) * sin(futureLatA));
		//predicted location of B at i seconds in the future
		distBIter = i * sqrt(pow(aircraftB.velocityX[0], 2) + pow(aircraftB.velocityY[0], 2));
		futureLatB = asin(sin(aircraftB.lat[0]) * cos(distBIter / RADIUS_EARTH) + 
				  cos(aircraftB.lat[0]) * sin(distBIter / RADIUS_EARTH) * cos(collisionPoint.headingB * TO_RADIANS));
		futureLonB = aircraftB.lon[0] + atan2(sin(collisionPoint.headingB * TO_RADIANS) * sin(distBIter / RADIUS_EARTH) * cos(aircraftB.lat[0]),
						      cos(distBIter / RADIUS_EARTH) * sin(aircraftB.lat[0]) * sin(futureLatB));
		
		if(gpsDistance(futureLatA, futureLonA, futureLatB, futureLonB) <= aircraftA.safetyBubble)
		{
			newCollision.collisionDetected = true;
		}
	}
	return newCollision;
}*/

void 
Autopilot_Interface::
CA_Avoid(aircraftInfo & aircraftA, aircraftInfo & aircraftB, predictedCollision & collision)
{
	printf("Autopilot_Interface::CA_Avoid()\n");
	double missDist = 75; //Meters
	double turnRadius = 50; //Meters
	double avoidVec[2] = {0};
	//Stores waypoints in Current_Waypoints
    	//Request_Waypoints();  //5-10-2021
	mavlink_mission_item_int_t headingVector; //FOR TESTING PURPOSES. MUST BE CHANGED LATER ALONG WITH "distanceVectors" FUNCTION
	
	//get info for aircraft
    	double latA = aircraftA.lat[0];
    	double lonA = aircraftA.lon[0];
    	uint64_t buffA = aircraftA.safetyBubble;
    	double latB = aircraftB.lat[0];
    	double lonB = aircraftB.lon[0];
    	uint64_t buffB = aircraftB.safetyBubble;

	//Define more variables
	double distMag;
	double distHdg;
	double addHdg;
	double avdDist;
	double avdHdg; //target heading of aircraft A to avoid collision
	double futureLatA;
	double futureLonA;
	double futureAltA = aircraftA.alt[0];
	
	predictedCollision newCollision = collision;
	
	//double collisionDist = sqrt(pow((collision.locationA.x - aircraftA.lat[0]), 2) + pow((collision.locationA.y - aircraftA.lon[0]), 2));
	double avdAlt;
	double avdAng; //angle increment between collision point and avoid point (A)
	
	int count;
	bool right;

    double relativeHdg = relHdg(aircraftA.Hdg[0], aircraftB.Hdg[0]);
	//addToFile(convertToString(relativeHdg), "Relative Heading");
	
	//Sidestep
	right = true;
	count = 6;
	avdAng = 0;
	while(newCollision.collisionDetected && count > 0)
	{
		printf("In CA_Avoid()\n");
		if(right)
		{
			avdAng += 2 * asin(aircraftA.safetyBubble / collision.distance);
			avdHdg = aircraftA.Hdg[0] + (collision.angle + avdAng) / 15.0;
		}
		else
			avdHdg = aircraftA.Hdg[0] + (collision.angle - avdAng) / 15.0;
		
		newCollision = CA_Predict(aircraftA, aircraftB, avdHdg, aircraftA.Hdg[0]);
		
		//while collision is detected loop will continue
		//else loop will end and further strategies ignored
		
		right = !right;
		count--;
	}
	
	//TrackEarly
	
	//Vertical
	//consider safe altitude ranges(if plane is already too low or too high)
	right = true;
	count = 6;
	double avdLength = 0;
	while(newCollision.collisionDetected && count > 0)
	{
		if(right)
		{
			avdLength += (2 * buffA);
			avdAlt = aircraftA.alt[0] + avdLength;
		}
		else
			avdAlt = aircraftA.alt[0] - avdLength;
		
		//if new collision detected continue loop
		//else loop will end and current values committed to waypoint
		
		//up = !up;
		count--;
	}
	
	//Speed
	right = true;
	count = 6;
	/*while(newCollision.collisionDetected && count > 0)
	{
		collisionPoint = sqrt(pow((collision.locationA.x - aircraftA.lat[0]), 2) + pow((collision.locationA.y - aircraftA.lon[0]), 2));
		
		if(right)
		{
		}
	}*/

	//Avoid if other aircraft is approaching from the side
    	if (fabs(relativeHdg) > 30.0 && fabs(relativeHdg) < 150.0)
	{
		mavlink_mission_item_int_t distVec = distanceVectors(aircraftB.lat[0], aircraftB.lon[0], aircraftA.lat[0], aircraftA.lon[0]);
    		distMag = sqrt( pow((distVec.x),2) + pow(distVec.y, 2));
    		distHdg = atan2(distVec.y, distVec.x);

    		//Find avoid point locaion
      		addHdg = atan(missDist/distMag);
    		avdDist = sqrt(pow(missDist,2) + pow(distMag,2));

    		if (relativeHdg > 0.0)
			addHdg = -addHdg;

    		avdHdg = distHdg + addHdg;
    		avoidVec[0] = avdDist * cos(avdHdg);
		avoidVec[1] = avdDist * sin(avdHdg);

		printf("AvoidVec[0]: %f\n", avoidVec[0]);
		//Log for this kind of avoidance
		addToFile(convertToString(distMag), "distMag");
		addToFile(convertToString(distHdg), "distHdg");
		addToFile(convertToString(addHdg), "addHdg");
		addToFile(convertToString(avdDist), "avdDist");
		addToFile(convertToString(avdHdg), "avdHdg");
		addToFile(convertToString(avoidVec[0]), "avoidVec[x]");
		addToFile(convertToString(avoidVec[1]), "avoidVec[y]");
		printf("avoidVex x,y: %f, %f\n", avoidVec[0], avoidVec[1]);
	}
	
	//avoid if other aircraft is approaching from front or rear
    	else 
	{
    		//Find heading to the goal waypoint
		headingVector = distanceVectors(currentMission[currentWaypoint].x, currentMission[currentWaypoint].y, aircraftA.lat[0], aircraftA.lon[0]);
		double targetHdg = atan2(headingVector.y, headingVector.x);
		
		double addHdg  = atan(missDist/turnRadius);
		double avdDist = sqrt(pow(missDist,2) + pow(turnRadius,2));
		double avdHdg = targetHdg + addHdg;
		avoidVec[0] = avdDist * cos(avdHdg);
		avoidVec[1] = avdDist * sin(avdHdg);

 		//Log for this type of avoidance
		addToFile(convertToString(addHdg), "head or tail avoid");
		addToFile(convertToString(addHdg), "addHdg");
		addToFile(convertToString(avdDist), "avdDist");
		addToFile(convertToString(avdHdg), "avdHdg");
		addToFile(convertToString(avoidVec[0]), "avoidVec[x]");
		addToFile(convertToString(avoidVec[1]), "avoidVec[y]");
		printf("head to tail avoidVex x,y: %f, %f\n", avoidVec[0], avoidVec[1]);
   	}

	printf("avoidVex x,y: %f, %f\n", avoidVec[0], avoidVec[1]);
    	//generate the waypoint
    	mavlink_mission_item_int_t avoidWaypoint;
	futureLatA = asin(sin(aircraftA.lat[0]) * cos(collision.distance / RADIUS_EARTH) +
				 cos(aircraftA.lat[0]) * sin(collision.distance / RADIUS_EARTH) * cos(avdHdg * TO_RADIANS));
	futureLonA = aircraftA.lon[0] + atan2(sin(avdHdg * TO_RADIANS) * sin(collision.distance / RADIUS_EARTH) * cos(aircraftA.lat[0]), 
						     cos(collision.distance / RADIUS_EARTH) * sin(aircraftA.lat[0]) * sin(futureLatA));
	
	//Commented 5-13-2021
	//avoidWaypoint.x = futureLatA;
	//avoidWaypoint.y = futureLonA;
	//avoidWaypoint.z = futureAltA;

	//5-13-2021
	//Set a fix point as avoidWaypoint: Spadra Farm CPP
	avoidWaypoint.x = 340433920;
	avoidWaypoint.y = -1178129089;
	avoidWaypoint.z = 50;

	//printf("avoidVex x,y: %f, %f\n", avoidVec[0], avoidVec[1]);
	printf("avoidWaypoint.x after: %f\n", avoidWaypoint.x);
	printf("avoidWaypoint.x %f , y: %f\n", avoidWaypoint.x, avoidWaypoint.y);
	//printf("avoidVex x,y: %f, %f\n", avoidVec[0], avoidVec[1]);

    	//log its lat and lon
    	addToFile(convertToString(avoidWaypoint.x), "avoidWP lat (x)");
	addToFile(convertToString(avoidWaypoint.y), "avoidWP lon (y)");

	//See if distance is correct
	addToFile(convertToString(gpsDistance(avoidWaypoint.x, avoidWaypoint.y, aircraftA.lat[0], aircraftA.lon[0])), "Distance to avoid point");

		/*
    	//insert the waypoint
    	insert_waypoint( avoidWaypoint, currentWaypoint);
		*/

		std::vector<mavlink_mission_item_int_t> simpleMission;
		simpleMission.push_back(create_waypoint(avoidWaypoint.x, avoidWaypoint.y, 50, 0, 20));
		simpleMission.push_back(create_loiter_point(avoidWaypoint.x, avoidWaypoint.y, 50, 1, 20));

		write_waypoints(simpleMission,simpleMission[1].seq);


    	//Tell the aircraft to go to the waypoint
    	setCurrentWaypoint(simpleMission[1].seq); //May not need this now that the write waypoints function is fixed


	addToFile(convertToString(currentWaypoint), "Replaced waypoint and current waypoint");
    	printf("Collision point created\n");

}
