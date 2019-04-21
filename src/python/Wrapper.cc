/*
 * Wrapper.cc
 *
 *  Created on: 29th Jan. 2017
 *      Author: Samuel Dudley
 *      Brief: Python wrapper for SLAM system
 */

#include <Wrapper.h>

using namespace std;
namespace python = boost::python;

Wrapper::Wrapper(string strVocFile, string strConfigFile)
{
	configurationFilePath = strConfigFile; // "./../Examples/Monocular/XIMEA.yaml";
	vocabularyFilePath = strVocFile; // "./../Vocabulary/ORBvoc.bin";
	configure(strConfigFile);

	isInitialized = false;
}

void Wrapper::initialize(const bool bUseViewer, const bool reuse, const string & mapFilePath)
{
		SLAM = new ORB_SLAM2::System(vocabularyFilePath, configurationFilePath, ORB_SLAM2::System::MONOCULAR, bUseViewer, reuse, mapFilePath);
		isInitialized = true;
}

void Wrapper::configure(string strConfigFile)
{
	// Check configuration file
	cv::FileStorage fsConfiguration(configurationFilePath.c_str(), cv::FileStorage::READ);
	if(!fsConfiguration.isOpened())
	{
	   cerr << "Failed to open configuration file at: " << configurationFilePath << endl;
	   exit(-1);
	}

/*
	// Get marker configuration info
	cv::FileNode marker_info = fsConfiguration["Marker.data"];
	cout << "attempting to read marker data" << endl;

	for(cv::FileNodeIterator fit = marker_info.begin(); fit != marker_info.end(); ++fit)
	{
//		string name = (string)(*fit)["name"];
//		string dict = (string)(*fit)["dictionary"];
//		int id = (int)(*fit)["id"];
//		float ssize = (float)(*fit)["size"];

		MarkerData m;
		*fit >> m;

		// map insert will not change the value if the key already exists, the [] operator will.
		// As we want to overwrite any existing values on configuration yaml reload we use
		// use the [] operator

		markerConfigurations[m.id] = m;

		cout << m.id << " : " << m.ssize << "m" << endl;
	}


	// arUco configuration
    cameraMatrix = (cv::Mat_<float>(3,3) << fsConfiguration["Camera.fx"]   , 0, fsConfiguration["Camera.cx"],
											0, fsConfiguration["Camera.fy"],    fsConfiguration["Camera.cy"],
										    0,                               0,                           1);

    distorsionCoeff  = (cv::Mat_<float>(4,1) << fsConfiguration["Camera.k1"], fsConfiguration["Camera.k2"], fsConfiguration["Camera.p1"], fsConfiguration["Camera.p2"]);

    cameraParameters.setParams(cameraMatrix, distorsionCoeff, cv::Size(fsConfiguration["Camera.width"],fsConfiguration["Camera.height"]));

    markerDetector.setThresholdParams(7, 7);
    markerDetector.setThresholdParamRange(2, 0);

    markerDetector.setDictionary(fsConfiguration["Marker.dictionary"], 0.f);
    setTrackMarkers(static_cast<int>(fsConfiguration["Marker.track"]));

*/
    fsConfiguration.release();
}


void Wrapper::shutdown()
{
	if (!initialized()) {
		return;
	}
	SLAM->Shutdown();
	exit(1);
}

bool Wrapper::getIsInitialized()
{
	return isInitialized;
}

bool Wrapper::initialized()
{
	if (!isInitialized) {
		cerr << "System is not initialized " << endl;
		return false;
	} else {
		return true;
	}
}


void Wrapper::track()
{
	if (!initialized()) {
		return;
	}

	SLAM->TrackMonocular(im, tframe);
}

/*
int Wrapper::getStatus()
{
	if (!initialized()) {
		return -1;
	}
	cout << "status: " << SLAM->GetStatus() << endl;
	return SLAM->GetStatus();
}
*/
/*
void Wrapper::getCurrentFrame()
{
	if (!initialized()) {
		return;
	}
	currentFrame = SLAM->GetFrameDrawer()->DrawFrame();
	// return currentFrame; // TODO: need cv::Mat to numpy.array converter for this
}
*/

void Wrapper::reset()
{
	if (!initialized()) {
		return;
	}
	SLAM->Reset();
}

BOOST_PYTHON_MODULE(libSLAM)
{
	using namespace python;
	class_<Wrapper>("Wrapper", init<std::string, std::string>()) // constructor for Wrapper class
		//.add_property("status", &Wrapper::getStatus) // read-only
		//.add_property("currentFrame", &Wrapper::getCurrentFrame) // read-only
		.add_property("isInitialized", &Wrapper::getIsInitialized) // read-only
		.def("shutdown", &Wrapper::shutdown)
		.def("track", &Wrapper::track)
		.def("reset", &Wrapper::reset)
		.def("initialize", &Wrapper::initialize)
	;
}
