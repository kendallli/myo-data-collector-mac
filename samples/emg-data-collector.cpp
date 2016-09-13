// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.

// This sample illustrates how to log EMG and IMU data. EMG streaming is only supported for one Myo at a time, and this entire sample is geared to one armband

#define _USE_MATH_DEFINES
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <fstream>
#include <time.h>
#include <chrono>

#include <sys/time.h>

#include <myo/myo.hpp>

class DataCollector : public myo::DeviceListener {
public:
	DataCollector()
	{
		openFiles();
	}
	// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	void onUnpair(myo::Myo* myo, uint64_t timestamp)
	{
		//clean up
		emgSamples.fill(0);
	}

	void openFiles() {
		time_t timestamp = std::time(0);

		// Open file for EMG log
		if (emgFile.is_open()) {
			emgFile.close();
		}
		std::ostringstream emgFileString;
		emgFileString << "emg-" << timestamp << ".csv";
		emgFile.open(emgFileString.str(), std::ios::out);
		emgFile << "timestamp,emg1,emg2,emg3,emg4,emg5,emg6,emg7,emg8" << std::endl;

		// Open file for gyroscope log
		if (gyroFile.is_open()) {
			gyroFile.close();
		}
		std::ostringstream gyroFileString;
		gyroFileString << "gyro-" << timestamp << ".csv";
		gyroFile.open(gyroFileString.str(), std::ios::out);
		gyroFile << "myot,x,y,z" << std::endl;

		// Open file for accelerometer log
		if (accelerometerFile.is_open()) {
			accelerometerFile.close();
		}
		std::ostringstream accelerometerFileString;
		accelerometerFileString << "accelerometer-" << timestamp << ".csv";
		accelerometerFile.open(accelerometerFileString.str(), std::ios::out);	
		accelerometerFile << "myot,myox,myoy,myoz,myost" << std::endl;

		// Open file for orientation log
		if (orientationFile.is_open()) {
			orientationFile.close();
		}
		std::ostringstream orientationFileString;
		orientationFileString << "orientation-" << timestamp << ".csv";
		orientationFile.open(orientationFileString.str(), std::ios::out);
		orientationFile << "timestamp,x,y,z,w" << std::endl;

		// Open file for orientation (Euler angles) log
		if (orientationEulerFile.is_open()) {
			orientationEulerFile.close();
		}
		std::ostringstream orientationEulerFileString;
		orientationEulerFileString << "orientationEuler-" << timestamp << ".csv";
		orientationEulerFile.open(orientationEulerFileString.str(), std::ios::out);
		orientationEulerFile << "timestamp,roll,pitch,yaw" << std::endl;

		// Open file for orientation (Euler angles) log
		if (allDataFile.is_open()) {
			allDataFile.close();
		}
        /**
		//open file for all data log
		std::ostringstream allDataFileString;
		allDataFileString << "allData-" << timestamp << ".csv";
		allDataFile.open(allDataFileString.str(), std::ios::out);
		allDataFile << "timestamp,emg1,emg2,emg3,emg4,emg5,emg6,emg7,emg8,";
		allDataFile << "timestamp_accelerometer,accelerometer-x,accelerometer-y,accelerometer-z,";
		allDataFile << "timestamp_gyro,gyro-x,gyro-y,gyro-z,";
		allDataFile << "timestamp_orientation,orientation-x,orientation-y,orientation-z,,orientation-w,";
		allDataFile << "roll,pitch,yaw" << std::endl;
         **/
	}

	// onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
	{
		//print();
		//store in gloable variables
		g_timestamp = timestamp;
		//to file
		emgFile << timestamp;
		for (size_t i = 0; i < 8; i++) {
			emgFile << ',' << static_cast<int>(emg[i]);
			emgSamples[i] = emg[i];

		}
		emgFile << std::endl;


	}

	// onOrientationData is called whenever new orientation data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onOrientationData(myo::Myo *myo, uint64_t timestamp, const myo::Quaternion< float > &rotation) {
		orientationFile << timestamp
			<< ',' << rotation.x()
			<< ',' << rotation.y()
			<< ',' << rotation.z()
			<< ',' << rotation.w()
			<< std::endl;
		//store in gloable variables
		timestamp_orientation = timestamp;
		orientation_x = rotation.x();
		orientation_y = rotation.y();
		orientation_z = rotation.z();
		orientation_w = rotation.w();
		orientationEulerFile << timestamp;
		calculateEulerAngles(orientationEulerFile, rotation.x(), rotation.y(), rotation.z(), rotation.w());
		orientationEulerFile << std::endl;
		
	}
	void calculateEulerAngles(std::ofstream &file, float x, float y, float z, float w){
		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;

		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		roll = atan2(2.0f * (w * x + y * z),
			1.0f - 2.0f * (x * x + y * y));
		pitch = asin(max(-1.0f, min(1.0f, 2.0f * (w * y - z * x))));
		yaw = atan2(2.0f * (w * z + x * y),
			1.0f - 2.0f * (y * y + z * z));

		file << ',' << roll
			<< ',' << pitch
			<< ',' << yaw;
	}
	// onAccelerometerData is called whenever new acceleromenter data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onAccelerometerData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &accel) {

		printVector(accelerometerFile, timestamp, accel);
		//store in gloable variables
		timestamp_accelerometer = timestamp;
		accelerometer_x = accel.x();
		accelerometer_y = accel.y();
		accelerometer_z = accel.z();
	}

	// onGyroscopeData is called whenever new gyroscope data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onGyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &gyro) {
		printVector(gyroFile, timestamp, gyro);
		//store in gloable variables
		timestamp_gyro = timestamp;
		gyro_x = gyro.x();
		gyro_y = gyro.y();
		gyro_z = gyro.z();
	}
	
	//Called when a paired Myo has been connected
	void onConnect(myo::Myo *myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
		//Reneable streaming
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
		openFiles();
		timeCount = 0;
	}

	// Helper to print out accelerometer and gyroscope vectors
	void printVector(std::ofstream &file, uint64_t timestamp, const myo::Vector3< float > &vector) {
        
        
        timeval time;
        gettimeofday(&time, NULL);
        long systemTimestamp = (time.tv_sec * 1000) + (time.tv_usec / 1000);
        
        if (isFirstTime) {
            
            file << "myot,myox,myoy,myoz,myost";
            file << std::endl;
            isFirstTime = false;
            
        }
        //time_t systemTimestamp = std::time(0);
		file << timestamp;
        printVectorWithoutEndlAndTimestamp(file, vector);
        file << ",";
        file << systemTimestamp;
		file << std::endl;
	}
	void printVectorWithoutEndlAndTimestamp(std::ofstream &file, const myo::Vector3< float > &vector){
		
		file << ',' << vector.x()
			<< ',' << vector.y()
			<< ',' << vector.z();
	}
	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
	{
		poseData = pose;
		std::cout << pose.toString() << std::endl;
	}
	
	//read and write in 1 or 2 mins period

	void print()
    {
        
		allDataFile << g_timestamp;
        
        std::cout <<timestamp_accelerometer<< std::endl;
		//emg data added
		for (size_t i = 0; i < 8; i++) {
			allDataFile << ',' << static_cast<int>(emgSamples[i]);

		}
		//accelerometer data added
		allDataFile << ',' << timestamp_accelerometer
			<< ',' << accelerometer_x
			<< ',' << accelerometer_y
			<< ',' << accelerometer_z;
		//gyro data added
		allDataFile << ',' << timestamp_gyro
			<< ',' << gyro_x
			<< ',' << gyro_y
			<< ',' << gyro_z;
		//orientation data added
		allDataFile << ',' << timestamp_orientation
			<< ',' << orientation_x
			<< ',' << orientation_y
			<< ',' << orientation_z
			<< ',' << orientation_w;

		calculateEulerAngles(allDataFile, orientation_x, orientation_y, orientation_z, orientation_w);

		allDataFile << std::endl;
    }

	// The files we are logging to
	std::ofstream emgFile;
	std::ofstream gyroFile;
	std::ofstream orientationFile;
	std::ofstream orientationEulerFile;
	std::ofstream accelerometerFile;
	std::ofstream allDataFile;

	//The timestamp of when the event is received by the SDK. 
	//Timestamps are 64 bit unsigned integers that correspond
	//to a number of microseconds since some (unspecified) period
	//in time. Timestamps are monotonically non-decreasing.
	uint64_t g_timestamp;
	uint64_t timestamp_accelerometer;
	uint64_t timestamp_gyro;
	uint64_t timestamp_orientation;
	
	//data arrays
	myo::Vector3< float >	 *accelData;
	myo::Quaternion< float > *rotationData;
	myo::Vector3< float >	 *gyroData;
	myo::Pose poseData;

	// These values are set by onOrientationData() and onPose() above.

	std::array<int8_t, 8>	 emgSamples;
	float accelerometer_x,accelerometer_y,accelerometer_z;
	float gyro_x, gyro_y, gyro_z;
	float orientation_x, orientation_y, orientation_z, orientation_w;
	float roll, pitch, yaw;

	int timeCount;
    bool isFirstTime = true;

};

int main(int argc, char** argv)
{
	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {
        
		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.ruili.myo-data-capture");

		std::cout << "Attempting to find a Myo..." << std::endl;

		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		myo::Myo* myo = hub.waitForMyo(10000);

		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}

		// We've found a Myo.
		std::cout << "Connected to a Myo armband! Logging to the file system. Check your home folder or the folder this application lives in." << std::endl << std::endl;

		// Next we enable EMG streaming on the found Myo.
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);

		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector collector;
		
		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(&collector);

		// Finally we enter our main loop.
		while (1) {
			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
			// In this case, we wish to update our display 50 times a second, so we run for 1000/20 milliseconds.
			hub.run(1);
		}

		// If a standard exception occurred, we print out its message and exit.
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}
