/*
*	myoDemo.cpp
*
*	A program to acquire EMG and accelerometer data
*   	in MATLAB Engine (MAC OS X) from a C++ program.
*
* 	Author: Joao Apura
*   	e-mail: joao.apura@tecnico.ulisboa.pt
*           Instituto Superior Tecnico - ULisboa
*           2014-2015
*
*   	If some error is printed out regarding some libraries while
*   	calling ./demoMyo, please do this in the terminal:
*       	MLDIR = Matlab directory (/Applications/MATLAB_R2014b.app)
*       	export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:$MLDIR/bin/maci64/:$MLDIR/extern/include
*       	export PATH=$PATH:$MLDIR/bin
*
*/

/* Libraries */
#define _USE_MATH_DEFINES
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <string.h>
#include <time.h>
#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <fstream>
#include <chrono>
#include "engine.h"

/* Myo framwork */
#include <myo/myo.hpp>

/* Variables */
#define  BUFSIZE 256
using namespace std;

class DataCollector : public myo::DeviceListener {

    public:
        DataCollector() : emgSamples()
        {
            openFiles();
        }
	
    /*
	   Functions belonging to our DataCollector object
	*/  
    void openFiles() {
        
        // Define time variables
		time_t timestamp = std::time(0);
        struct tm * timeinfo;
        char buffer[80];
        
        // Change time string for the log's name
        timeinfo = localtime(&timestamp);
        strftime (buffer,80,"%d:%m:%Y-%H:%M:%S",timeinfo);
        
		// Open file for EMG log
		if (emgFile.is_open()) {
			emgFile.close();
		}
  
		std::ostringstream emgFileString;
		emgFileString << "DataMyo/emg-" << buffer << ".csv";
		emgFile.open(emgFileString.str(), std::ios::out);
		emgFile << "timestamp,emg1,emg2,emg3,emg4,emg5,emg6,emg7,emg8" << std::endl;

		// Open file for gyroscope log
		if (gyroFile.is_open()) {
			gyroFile.close();
		}
		std::ostringstream gyroFileString;
		gyroFileString << "DataMyo/gyro-" << buffer << ".csv";
		gyroFile.open(gyroFileString.str(), std::ios::out);
		gyroFile << "timestamp,x,y,z" << std::endl;

		// Open file for accelerometer log
		if (accelerometerFile.is_open()) {
			accelerometerFile.close();
		}
		std::ostringstream accelerometerFileString;
		accelerometerFileString << "DataMyo/accelerometer-" << buffer << ".csv";
		accelerometerFile.open(accelerometerFileString.str(), std::ios::out);
		accelerometerFile << "timestamp,x,y,z" << std::endl;

		// Open file for orientatin log
		if (orientationFile.is_open()) {
			orientationFile.close();
		}
		std::ostringstream orientationFileString;
		orientationFileString << "DataMyo/orientation-" << buffer << ".csv";
		orientationFile.open(orientationFileString.str(), std::ios::out);
		orientationFile << "timestamp,x,y,z,w" << std::endl;

	}
    
    void onUnpair(myo::Myo* myo, uint64_t timestamp){
    // onUnpair is called when Myo is disconnected from Myo Connect by the user
        
        std::cout << "We have lost our MYO";
        emgSamples.fill(0);
	}
    
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg){
    // onEmgData() is called whenever a paired Myo has provided new EMG data, 
    // and EMG streaming is enabled.

        emgFile << timestamp;
		for (int i = 0; i < emgSamples.size(); i++) {
            emgSamples[i] = emg[i];
            emgFile << ',' << static_cast<int>(emg[i]);
		}
        emgFile << std::endl;
	}
    
	void onOrientationData(myo::Myo *myo, uint64_t timestamp, const myo::Quaternion< float > &rotation) {
	// onOrientationData is called whenever new orientation data is provided
        
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;
        
        orientationFile << timestamp
			<< ',' << rotation.x()
			<< ',' << rotation.y()
			<< ',' << rotation.z()
			<< ',' << rotation.w()
			<< std::endl;
        
        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (rotation.w() * rotation.x() + rotation.y() * rotation.z()),
                           1.0f - 2.0f * (rotation.x() * rotation.x() + rotation.y() * rotation.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (rotation.w() * rotation.y() - rotation.z() * rotation.x()))));
        float yaw = atan2(2.0f * (rotation.w() * rotation.z() + rotation.x() * rotation.y()),
                          1.0f - 2.0f * (rotation.y() * rotation.y() + rotation.z() * rotation.z()));
        
        oriSamples[0] = roll;
        oriSamples[1] = pitch;
        oriSamples[2] = yaw;
	}

	void onAccelerometerData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &accel) {
    // onAccelerometerData is called whenever new acceleromenter data is provided
        
		printVector(accelerometerFile, timestamp, accel);

        for (int i = 0; i < acceSamples.size(); i++) {
            acceSamples[i] = accel[i];
        }
        stampTime = std::to_string(timestamp);
	}

	void onGyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &gyro) {
	// onGyroscopeData is called whenever new gyroscope data is provided
        
        printVector(gyroFile, timestamp, gyro);
        
        for (int i = 0; i < gyroSamples.size(); i++) {
            gyroSamples[i] = gyro[i] * (float)M_PI / 180;
        }
	}
  
	// Helper to print out accelerometer and gyroscope vectors
	void printVector(std::ofstream &file, uint64_t timestamp, const myo::Vector3< float > &vector) {
		file << timestamp
			<< ',' << vector.x()
			<< ',' << vector.y()
			<< ',' << vector.z()
			<< std::endl;
	}
    
    void onConnect(myo::Myo *myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
        //Reneable streaming
        myo->setStreamEmg(myo::Myo::streamEmgEnabled);
    }
	
//    void print(){
//		// Clear the current line
//		std::cout << '\n';
//
//		// Print out the EMG data.
//		//Store in global object? good idea?
//		for (size_t i = 0; i < emgSamples.size(); i++) {
//            std::ostringstream oss;
//			oss << static_cast<int>(emgSamples[i]);
//			std::string emgString = oss.str();
//
//			//std::cout << emgString << ';';
//			//std::cout << emgStringSamples;
//
//
//		}
//		//std::cout << emgStringSamples;
//		std::cout << std::flush;
//	}

    // Variables
	std::ofstream emgFile;            // EMG file we are logging to
	std::ofstream gyroFile;           // Gyroscope file we are logging to
	std::ofstream orientationFile;    // Orientation file we are logging to
	std::ofstream accelerometerFile;  // Accelerometer file we are logging to
    
    std::array<float, 3> acceSamples; // Array with accelerometer samples
    std::array<float, 3> gyroSamples; // Array with gyroscope samples
    std::array<float, 3> oriSamples;  // Array with orientation samples
	std::array<int8_t, 8> emgSamples; // Array with EMG samples
    std::string stampTime;            // Time stamp from Myo as string
};


int main(int argc, char** argv)
{
    /*
	* Create Engine and call engOpen with a NULL string, 
    * to start a MATLAB process on the current host. 
	*/
	Engine *ep = engOpen(NULL);
    
	/*
	* Initialize MYO and make sure MYO is connected to MyoConnector prior 
	* to opening Matlab.
	*/

	try {
		// Create a hub to connect MYO
		myo::Hub hub("com.JApura.myoDemo");
		std::cout << "Connecting to your MYO. Please, wait." << std::endl;

		// Check MYO (10 s)
		myo::Myo* myo = hub.waitForMyo(10000);

		// If we didn't find a myo, print out an error.
		if (!myo) {
			throw std::runtime_error("Please, make sure Myo is plugged in.");
		}
		std::cout << "Connected to a Myo armband! Yeey!" << std::endl << std::endl;
        
        /*
         * Data - EMG, Accelerometer, Gyroscope & Orientation
         */
        // C++ variables
        double emg[8];
        double acce[3];
        double gyro[3];
        double ori[4];
        
        // MATLAB input matrices
        mxArray *EMG = mxCreateDoubleMatrix(1, 8, mxREAL);
        mxArray *Acce = mxCreateDoubleMatrix(1, 3, mxREAL);
        mxArray *Gyro = mxCreateDoubleMatrix(1, 3, mxREAL);
        mxArray *Ori = mxCreateDoubleMatrix(1, 3, mxREAL);
        
        // Pointers to the MATLAB matrices (to copy data)
        double *pEMG = mxGetPr(EMG);
        double *pAcce = mxGetPr(Acce);
        double *pGyro = mxGetPr(Gyro);
        double *pOri = mxGetPr(Ori);
        
        // Evaluate strings with variables (for further computing)
        engEvalString(ep, "AcceData=zeros(31,3);");
        engEvalString(ep, "GyroData=zeros(31,3);");
        engEvalString(ep, "OriData=zeros(31,3);");
        
        
		// Enable EMG streaming
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
        
        // Unlock foreeeever!
        // myo->unlock(myo::Myo::unlockHold);

		// Construct deviceListener
		DataCollector collector;
		hub.addListener(&collector);
        
        // Ask for user input (acquisition time in seconds)
        string input = "";
        int myAcquisitionTime = 0;
        while (true){
            std::cout << "Please, insert the acquisition time (in seconds): ";
            getline(cin, input);
            
            // Safely cast as an number
            stringstream myStream(input);
            if (myStream >> myAcquisitionTime)
                break;
            std::cout << "Invalid number, please try again. " << std::endl;
        }

        /*
         *  Main loop
         */
        
        // Variables to compute elapsed time, according to Myo timestamps
        double tstart = 0, tend = 0;
        double elapsedTime = 0;
        
        // Notify user that the acquisition has started
        myo->notifyUserAction();
        
        // Variables to compute elapsed computational time using chono library
        double elapsedSec = 0;
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        
        int x = 0;
		while (elapsedTime < myAcquisitionTime) {

            // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
            hub.run(1);
            
            // Extract first timestamp from Myo (string casted as a number)
            if (tstart == 0){
                stringstream myStream(collector.stampTime);
                myStream >> tstart;
            }
            
            // Extracting samples from DataCollector
            std::array<float, 3> acceData = collector.acceSamples;
            std::array<float, 3> gyroData = collector.gyroSamples;
            std::array<float, 3> oriData = collector.oriSamples;
            std::array<int8_t, 8> emgData = collector.emgSamples;
            
            for (int i = 0; i < emgData.size(); i++){
                
                if (i < 3) {
                    // Accelerometer samples
                    acce[i] = acceData[i];
                    pAcce[i] = acce[i];
                
                    // Gyroscope samples
                    gyro[i] = gyroData[i];
                    pGyro[i] = gyro[i];

                    // Orientation samples
                    ori[i] = oriData[i];
                    pOri[i] = ori[i];
                }
                
                // EMG samples
                emg[i] = emgData[i];
                pEMG[i] = emg[i];
            }
            
			/*
			* Plot the result
			*/
            engPutVariable(ep, "Acce", Acce);
            engPutVariable(ep, "Gyro", Gyro);
            engPutVariable(ep, "Ori", Ori);
            engPutVariable(ep, "EMG", EMG);
            engEvalString(ep,"MyoPlot");
            
            // Extract timestamps from Myo (string casted as a number) and compute elapsed time
            stringstream myStream(collector.stampTime);
            myStream >> tend;
            elapsedTime = (tend - tstart)/1000000;
            
            // Keep track of how many runs Myo has performed
            x++;
            if (x % 30 == 0){
                std::cout << x << endl;
            }
		}
        
        // Compute elapsed computational time using chono library
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        elapsedSec = elapsed_seconds.count();
        
        // Print elapsed computational time
		std::cout << elapsedSec << "\n";
        
        // Free willy!
        mxDestroyArray(EMG);
        mxDestroyArray(Acce);
        mxDestroyArray(Gyro);
        mxDestroyArray(Ori);
        
        // Notify user that the acquisition has ended
        myo->notifyUserAction();
        
        // Remove listener from hub
        hub.removeListener(&collector);
	}
    
    // If a standard exception occurred, print out its message and exit.
	catch(const std::exception& e) {
		std::cerr << "Huston, we have a problem: " << e.what() << std::endl;
		std::cerr << "Try again, please. It Myo Connect running?";
		std::cin.ignore();
		return 1;
	}
    
	/*
	* Close MATLAB engine and exit.
	*/
	engClose(ep);
	return EXIT_SUCCESS;
}








