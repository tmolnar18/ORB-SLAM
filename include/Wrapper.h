/*
 * Wrapper.h
 *
 *  Created on: 29th Jan 2017
 *      Author: Samuel Dudley
 */
#ifndef WRAPPER_H
#define WRAPPER_H


#include<iostream>
#include<string>
#include<sstream>
#include<algorithm>
#include<fstream>
#include<vector>
#include<chrono>
#include<iomanip>
#include<opencv2/core.hpp>
#include<Converter.h>

#include<System.h>
#include<boost/python.hpp>
// numpy
//#include "numpy/ndarrayobject.h"

class Wrapper
{
	public:
		Wrapper(std::string strVocFile, std::string strConfigFile); // wrapper constructor
		void initialize(const bool bUseViewer = true, const bool reuse= false, const string & mapFilePath = "");
		void shutdown();
		//TODO: make it private?
		void configure(std::string strConfigFile);
		void track();
		int  getStatus();
		void reset();
		void getCurrentFrame();
		bool getIsInitialized();

	public:
		std::string msg;
		std::string vocabularyFilePath;
		std::string configurationFilePath;

		bool isInitialized;

		//xiAPIplusCameraOcv cam;
		cv::Mat src;
		cv::Mat im;

		cv::Mat currentFrame; //im with tracking visualization drawn on it
		cv::FileStorage fsConfiguration;

		const double tframe = 0.1;


		cv::Mat cameraMatrix;
		cv::Mat distorsionCoeff;


	private:
		bool initialized();

	private:
		ORB_SLAM2::System* SLAM;


};

#endif // WRAPPER_H
