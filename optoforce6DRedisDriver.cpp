/*
 * Driver to use the force sensor optoforce (tested on 6D force sensor only for now)
 * it will create a connection, read the data, transfer the data from count value to real units
 * and publish it in redis with a key named "sai2::optoforceSensor::6Dsensor::force"
 */


#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include "OptoDAQ.h"
#include "OptoDAQWatcher.h"
#include "OptoDAQDescriptor.h"
#include "OptoPacket6D.h"
#include "filters/ButterworthFilter.h"

// For redis publication
#include "redis/RedisClient.h"

const std::string EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";

typedef unsigned long long mytime_t;

sai::ButterworthFilter filter;
const double cutoff_freq = 0.05;  //the cutoff frequency of the filter, in the range of (0 0.5) of sampling freq
bool use_filter = false;

unsigned long long counter = 0;

std::string serialNumber;  // To know which sensitivity report to use
std::string deviceName;  // To know which sensitivity report to use

//std::ofstream force_file;

mytime_t Now()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    mytime_t millisecs = t.tv_sec * 1000;
    millisecs += t.tv_nsec / (1000 * 1000);
    return millisecs;
}


mytime_t NowMicro()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    mytime_t microsecs = t.tv_sec * 1000 * 1000;
    microsecs += t.tv_nsec / (1000);
    return microsecs;
}


mytime_t ElapsedTime(mytime_t p_Time)
{
	return Now() - p_Time;
}


mytime_t ElapsedTimeMicro(mytime_t p_Time)
{
	return NowMicro() - p_Time;
}


void MySleep(unsigned long p_uMillisecs)
{
	usleep(p_uMillisecs * 1000);
}

/*
 * Set the config to the DAQ
 * it is a blocking function; returns true, if the sending of
 * configuration is succeeded otherwise it returns false
 */
bool SetConfig(OptoDAQ & p_optoDAQ, int p_iSpeed, int p_iFilter)
{
	OptoConfig sensorConfig(p_iSpeed, p_iFilter, 0 /*Hardware zeroing turned off*/);
	mytime_t tNow = Now();

	bool bSuccess = false;
	do {
		bSuccess = p_optoDAQ.SendConfig(sensorConfig);
		if (bSuccess) {
			p_optoDAQ.RequestSensitivityReport(); // This call is a must
			return true;
		}
		if (ElapsedTime(tNow) > 1000) {
			// 1 sec timeout
			return false;
		}
		MySleep(1);
	} while (bSuccess == false);
	return false;
}


void ShowInformation(const OptoDAQDescriptor& descriptor)
{
	deviceName = std::string(descriptor.GetTypeName());
	serialNumber = std::string (descriptor.GetSerialNumber());
	std::cout<<"Device Name: "<<deviceName<<std::endl;
	std::cout<<"Port: "<<descriptor.GetAddress()<<std::endl;
	std::cout<<"Protocol version: "<<descriptor.GetProtocolVersion()<<std::endl;
	std::cout<<"Serial Number: "<<serialNumber<<std::endl;
}



/*
 * Opens the desired port
 * it returns true if port could be opened otherwise returns false
 */
bool OpenPort(OptoDAQDescriptor& descriptor, OptoDAQ & p_optoDAQ, int p_iIndex)
{
	OptoDAQWatcher watcher;
	watcher.Start();

	MySleep(500); // We wait some ms to be sure about OptoPorts enumerated PortList
	OptoDAQDescriptor descriptors[16]; //shouldn't need to use more than 16 sensors at once
	size_t count = watcher.GetConnectedDAQs(descriptors, 16, true);

	if (p_iIndex >= count) {
		// index overflow
		return false;
	}
	descriptor = descriptors[p_iIndex];
	ShowInformation(descriptor);
	p_optoDAQ.SetOptoDAQDescriptor(descriptor);
	bool bSuccess = p_optoDAQ.Open();
	return bSuccess;
}


// /*
//  * Blocking call to read one 3D package (with one second timeout)
//  * it return a non-negative number if it succeeded (p_Package will be the read package)
//  * otherwise returns -1
//  */
// int ReadPackage3D(OptoDAQ & p_optoDAQ, OptoPackage & p_Package)
// {
// 	int iSize = -1;
// 	mytime_t tNow = Now();
// 	for (;;) {
// 		iSize = p_optoDAQ.read(p_Package, 0, false);
// 		if (iSize < 0 || iSize > 0) {
// 			break;
// 		}
// 		// No packages in the queue so we check the timeout
// 		if (ElapsedTime(tNow) >= 1000) {
// 			break;
// 		}
// 		usleep(100);
// //		MySleep(1);
// 	}
// 	return iSize;
// }


/*
 * Blocking call to read one 6D package (with one second timeout)
 * return true if success, else false
 */
bool ReadPackage6D(OptoDAQ & p_optoDAQ, OptoPacket6D & p_Package)
{
	bool success = false;
	mytime_t tNow = Now();
	for (;;) {
		success = p_optoDAQ.GetLastPacket6D(&p_Package, false /*non-blocking*/);
		if (success) {
			break;
		}
		// No packages in the queue so we check the timeout
		if (ElapsedTime(tNow) >= 1000) {
			break;
		}
		usleep(100);
//		MySleep(1);
	}
	return success;
}


// /*
//  * The function determines if the sensor is a 3D sensor
//  */
// bool Is3DSensor(OptoDAQ & p_optoDAQ)
// {
// 	opto_version optoVersion = p_optoDAQ.getVersion();
// 	if (optoVersion != _95 && optoVersion != _64) {
// 		return true;
// 	}
// 	return false;
// }


/*
 * Remaps the measurements to a right hand basis and uses the sensivity report (in docs)
 * to transform the measurements in N and Nm
 */
bool processRaw6DSensorData(const OptoPacket6D& optoPackage, Eigen::VectorXd& data)
{
	if(deviceName == "64" && serialNumber == "HEXHB148")
    {
    	// std::cout << "round sensor detected" << std::endl;
    	if (!optoPackage.IsValid()) {
    		return false;
    	}
    	auto simplePacket = optoPackage.ToSimplePacket();
		data.setZero(6);
		data << simplePacket.Fx, 
			simplePacket.Fy,
			simplePacket.Fz,
			simplePacket.Tx,
			simplePacket.Ty,
			simplePacket.Tz;
		return true;
    }
    return false;
}

// void processRaw3DSensorData(const OptoPackage& optoPackage, Eigen::VectorXd& data)
// {

//     std::cout << "WARNING : processRaw3DSensorData not implemented." << std::endl;

// }

// void Run3DSensorExample(OptoDAQ & p_optoDAQ)
// {
// 	// start redis client
// 	HiredisServerInfo info;
// 	info.hostname_ = "127.0.0.1";
// 	info.port_ = 6379;
// 	info.timeout_ = { 1, 500000 }; // 1.5 seconds
// 	auto redis_client = CDatabaseRedisClient();
// 	redis_client.serverIs(info);

//     if(use_filter)
//     {
//         filter.setDimension(3);
//         filter.setCutoffFrequency(cutoff_freq);
//     }

//     Eigen::VectorXd force_raw = Eigen::VectorXd::Zero(3);
//     Eigen::VectorXd force_filtered = Eigen::VectorXd::Zero(3);

// 	mytime_t tNow = Now();
// 	unsigned int uTotalReadPackages = 0;
// 	while(true)
// 	{
// 		mytime_t tLoopNow = NowMicro();
// 		OptoPackage optoPackage;
// 		int iReadSize = ReadPackage3D(p_optoDAQ, optoPackage);
// 		if (iReadSize < 0) {
// 			std::cout<<"Something went wrong, DAQ closed!"<<std::endl;
// 			return;
// 		}
// 		uTotalReadPackages += (unsigned int)iReadSize;

// 		// Formatting output in C style
// 		double dLoopTime = ElapsedTimeMicro(tLoopNow) / 1000.0;
// 		mytime_t TotalElapsedTime = ElapsedTime(tNow);
// 		double dTotalTime = (double)TotalElapsedTime / 1000.0; // Elapsed time in sec
// 		double dFrequency = 0.0;
// 		if (dTotalTime > 0.0) {
// 			dFrequency = (uTotalReadPackages / dTotalTime);
// 		}
// //		fprintf(stdout, "Elapsed: %.1f s Loop time: %.2f ms Samples: %u Sample rate: %.2f Hz\r\n", dTotalTime, dLoopTime, uTotalReadPackages, dFrequency);
// //		fflush(stdout);

// 		processRaw3DSensorData(optoPackage, force_raw);

// 		if(use_filter)
// 		{
// 		    force_filtered = filter.update(force_raw);
// 		}
// 		else
// 		{
// 		    force_filtered = force_raw;
// 		}

// 		//send to redis
// 		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY, force_filtered);
// 	}

// }

void Run6DSensorExample(OptoDAQ & p_optoDAQ)
{
	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = CDatabaseRedisClient();
	redis_client.serverIs(info);

    if(use_filter)
    {
        filter.setDimension(6);
        filter.setCutoffFrequency(cutoff_freq);
    }

    Eigen::VectorXd force_raw = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd force_filtered = Eigen::VectorXd::Zero(6);

	mytime_t tNow = Now();
	unsigned int uTotalReadPackages = 0;
	int ctr = 0;
//	do {
	while(true)
	{
		mytime_t tLoopNow = NowMicro();
		OptoPacket6D optoPackage;
		if (!ReadPackage6D(p_optoDAQ, optoPackage) || !p_optoDAQ.IsValid()) {
			std::cout<<"Something went wrong, DAQ closed!"<<std::endl;
			return;
		}

		// Formatting output in C style
		double dLoopTime = ElapsedTimeMicro(tLoopNow) / 1000.0;
		mytime_t TotalElapsedTime = ElapsedTime(tNow);
		double dTotalTime = (double)TotalElapsedTime / 1000.0; // Elapsed time in sec
		double dFrequency = 0.0;
		if (dTotalTime > 0.0) {
			dFrequency = (1 / dTotalTime);
		}
		//fprintf(stdout, "Elapsed: %.1f s Loop time: %.2f ms Samples: %u Sample rate: %.2f Hz\r\n", dTotalTime, dLoopTime, uTotalReadPackages, dFrequency);
		//fflush(stdout);

		if (!processRaw6DSensorData(optoPackage, force_raw)) {
			std::cout<<"Packet process failed"<<std::endl;
			return;
		}

		if(use_filter)
		{
		    force_filtered = filter.update(force_raw);
		}
		else
		{
		    force_filtered = force_raw;
		}

		// if(counter%500 == 0)
		// {
		// 	std::cout << force_filtered(2) << std::endl;
		// }


		// publish to redis
		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY, force_filtered);

		counter++;

//		force_file << force_filtered.transpose() << "\n";

	}


}



int main()
{
	OptoDAQDescriptor descriptor;
	OptoDAQ optoDAQ;
	
	// Changeable values, feel free to play with them
	int iPortIndex = 0;  // The index of the port which will be opened

	int iSpeed = 500; // Speed in Hz (between 0 and 500)
	int iFilter = 4;  // Filter cutoff frequency in Hz (0 - no filtering; 1 - 500 Hz, 2 - 150 Hz, 3 - 50 Hz, 4 - 15 Hz (default), 5 - 5 Hz, 6 - 1.5 Hz)
	///////////////////
	if (OpenPort(descriptor, optoDAQ, iPortIndex) == false) {
		std::cout<<"Could not open port"<<std::endl;
		return 0;
	}
	bool bConfig = SetConfig(optoDAQ, iSpeed, iFilter);
	if (bConfig == false) {
		std::cout<<"Could not set config"<<std::endl;
		optoDAQ.Close();
		return 0;
	}

//
//
//	std::cout << "open file\n";
//	force_file.open("forces.txt");


	Run6DSensorExample(optoDAQ);


	optoDAQ.Close();
	return 0;
}

