/*****************************
Copyright 2011 Rafael Mu��oz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list
of conditions and the following disclaimer in the documentation and/or other materials
provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Mu��oz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Mu��oz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Mu��oz Salinas.
********************************/

#pragma warning (disable : 4290)

#include <iostream>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "AttitudePosition.h"
#include "MarkerWorldCoornidate.h"
#include "my_serial.h"
#include <fstream>
#include <map>

extern std::vector<MarkerWorld> CoordinateTable;

using namespace std;
using namespace cv;
using namespace aruco;

#define TRANS_WORLD 1
#define WRITE_VIDEO  0


bool loadConfig(std::string configfile_path, std::map<int, MarkerConfig> &markermap)
{
	std::ifstream configFileIn;
	configFileIn.open(configfile_path);

	if (!configFileIn.is_open())
	{
		return false;
	}

	char temp[255];
	int id = -1;
	int boardsize = -1;
	int x = -1;
	int y = -1;
	int z = -1;

	while (!configFileIn.eof())
	{
		configFileIn.getline(temp, 255);
		sscanf(temp, "%d %d %d %d %d", &id, &boardsize, &x, &y, &z);

		if (id>0)
		{
			markermap.insert(make_pair(id, MarkerConfig(id, boardsize, x, y, z)));
		}
	}

	return true;
}

cv::Point3f coordinate_camera(0, 0, 0);
Attitude atti_camera;
std::vector< aruco::Marker > Markers;
std::map<int, MarkerConfig> markermap;

int main(int argc, char **argv)
{
	try
	{
		string cameraParamFileName("PS3_640.yml");
		string imagename("SizeQR.png");
		//string imagename("3.bmp");

		string configpath("config.ini");
		bool bloadOK = loadConfig(configpath, markermap);
		if (!bloadOK)
		{
			cout << "markers config load failed!" << endl;
			return -1;
		}

		for (size_t i = 0; i < markermap.size(); i++)
		{
			cout << markermap[i] << endl;
		}

		initCoordinateTable(CoordinateTable);

		aruco::CameraParameters CamParam;
		MarkerDetector MDetector;

		cv::Size m_size(640, 480);

		// read the input image
		cv::Mat InImage;
		// try opening first as video
		VideoCapture cap(0);

		cap >> InImage;
		resize(InImage, InImage, m_size);

		//read camera parameters if specifed
		CamParam.readFromXMLFile(cameraParamFileName);

		cout << CamParam.CameraMatrix << endl;
		cout << CamParam.Distorsion << endl;
		cout << CamParam.CamSize << endl;
		CamParam.resize(m_size);
		cout << CamParam.CameraMatrix << endl;
		cout << CamParam.Distorsion << endl;
		cout << CamParam.CamSize << endl;

		cv::namedWindow("thes", 1);

		int p1 = 7;
		int p2 = 7;
		int t_p_range = 2;
		createTrackbar("p1", "thes", &p1, 101);
		createTrackbar("p2", "thes", &p2, 50);
		createTrackbar("range", "thes", &t_p_range, 31);

		ostringstream ostr_pos;
		ostringstream ostr_angle;

#if WRITE_VIDEO

		/*VideoWriter videowriter;
		videowriter.open("video.avi", CV_FOURCC('D', 'I', 'V', 'X'), 30, InImage.size());*/

		VideoWriter videowriter_process;
		videowriter_process.open("video111.avi", CV_FOURCC('D', 'I', 'V', 'X'), 60, InImage.size());

#endif

		Point3f pos_camera(0, 0, 0);
		Attitude attitude_camera;

		TickMeter tm;
		tm.reset();

#define USE_CANNY 0
#if USE_CANNY
		MDetector.setThresholdMethod(MarkerDetector::CANNY);
#endif

		while (true)
		{
			p1 = p1 / 2 * 2 + 1;
			p2 = p2 / 2 * 2 + 1;
			MDetector.setThresholdParamRange(t_p_range);
			MDetector.setThresholdParams(p1, p2);

			tm.reset();
			tm.start();

			cap >> InImage;
//			InImage = imread(imagename);
			tm.stop();
			//InImage = imread(imagename);
			resize(InImage, InImage, m_size);
			//cout << "capImg:" << tm.getTimeMilli() << endl;

#if WRITE_VIDEO
			videowriter_process << InImage;
#endif

			tm.reset();
			tm.start();

			// Ok, let's detect
			MDetector.detect(InImage, Markers, CamParam);
			//tm.stop();
			//cout << "calculateExtrinsics: " << tm.getTimeMilli() << "ms" << endl;
			// for each marker, draw info and its boundaries in the image
			static int markersize = 0;
			for (unsigned int i = 0; i < Markers.size(); i++)
			{
				Markers[i].draw(InImage, Scalar(0, 0, 255), 2);
				markersize = (markermap.find(Markers[i].id))->second.boardsize;
				if(markersize<=0)
				{
					Markers.erase(Markers.begin()+i);
				}
				else
				{
					Markers[i].calculateExtrinsics(markersize, CamParam, false);
				}

			}

			if (CamParam.isValid())
			{
				for (unsigned int i = 0; i < Markers.size(); i++)
				{
					//CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam);
					//cout << Markers[i].id << ": " << Markers[i].getCenter() << " " << Markers[i].getPerimeter()<< endl;

					//CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam);

					//circle(InImage, Point(InImage.cols / 2, InImage.rows / 2), 3, CV_RGB(0, 0, 255), -1);

					getAttitude(Markers[i], attitude_camera);

					//ostr_angle.clear();
					//ostr_angle.str("");
					//ostr_angle << "          Pit=" << (int)attitude_camera.Pit << " " << "Yaw=" << (int)attitude_camera.Yaw << " " << "Rol=" << (int)attitude_camera.Rol;

#if TRANS_WORLD
					getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_camera);

					ostr_pos.clear();
					ostr_pos.str("");
					ostr_pos << Point3i(((markermap.find(Markers[i].id))->second).coordinate + pos_camera);
					//ostr_pos << " x=" << (int)pos_camera.x << " " << "y=" << (int)pos_camera.y << " " << "z=" << (int)pos_camera.z;
					putText(InImage, ostr_pos.str(), Markers[i].getCenter() - Point2f(0, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

					ostr_pos.clear();
					ostr_pos.str("");
					ostr_pos << Point3i(pos_camera);
					//ostr_pos << " x=" << (int)pos_camera.x << " " << "y=" << (int)pos_camera.y << " " << "z=" << (int)pos_camera.z;
					putText(InImage, ostr_pos.str(), Markers[i].getCenter() - Point2f(0, 40), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

#endif

					/*ostr_pos.clear();
					ostr_pos.str("");
					ostr_pos << "          x=" << (int)pos[0] << " " << "y=" << (int)pos[1] << " " << "z=" << (int)pos[2];

					putText(InImage, ostr_pos.str(), Markers[i].getCenter(), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);*/

					/*putText(InImage, ostr_angle.str(), Markers[i].getCenter() + Point2f(0, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);*/

				}
			}

			getCameraPosWithMarkers(Markers, coordinate_camera, atti_camera, 0);

			serialSent();

			tm.stop();
			//cout << "calculateExtrinsics: " << tm.getTimeMilli() << "ms" << endl;

			ostr_pos.clear();
			ostr_pos.str("");
			ostr_pos << "x=" << (int)coordinate_camera.x << " " << "y=" << (int)coordinate_camera.y << " " << "z=" << (int)coordinate_camera.z;

			putText(InImage, ostr_pos.str(), Point(30, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

			// show input with augmented information
			cv::imshow("in", InImage);
			// show also the internal image resulting from the threshold operation
			cv::imshow("thes", MDetector.getThresholdedImage());

			tm.stop();

			//videowriter_process << InImage;

			char c_key = cv::waitKey(10);
			if (c_key == 27) // wait for key to be pressed
			{
				break;
			}
	}

#if WRITE_VIDEO
		//videowriter.release();
		videowriter_process.release();
#endif
		return 0;
}
	catch (std::exception &ex)
	{
		cout << "Exception :" << ex.what() << endl;
	}
}
