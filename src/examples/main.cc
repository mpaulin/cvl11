/*=====================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://MAVCONN.ethz.ch>

(c) 2009, 2010 MAVCONN PROJECT

This file is part of the MAVCONN project

    MAVCONN is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MAVCONN is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief LCM example
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include <cstdio>
#include <unistd.h>
#include <glib.h>
#include "mavconn.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <signal.h>

#include <interface/shared_mem/PxSHMImageClient.h>

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

//#include "../ballcatching_mattis/ballDetection.hpp"
#include "../ballcatching_mattis/config.hpp"
#include "StereoProc.h"

// Timer for benchmarking
struct timeval tv;

int sysid = 42;
int compid = 112;
bool verbose = false;
bool debug = 0;

static GString* configFile = g_string_new("conf/abc.cfg");

int imageCounter = 0;
std::string fileBaseName("frame");
std::string fileExt(".png");

bool quit = false;

float colormap_jet[128][3] =
{
		{0.0f,0.0f,0.53125f},
		{0.0f,0.0f,0.5625f},
		{0.0f,0.0f,0.59375f},
		{0.0f,0.0f,0.625f},
		{0.0f,0.0f,0.65625f},
		{0.0f,0.0f,0.6875f},
		{0.0f,0.0f,0.71875f},
		{0.0f,0.0f,0.75f},
		{0.0f,0.0f,0.78125f},
		{0.0f,0.0f,0.8125f},
		{0.0f,0.0f,0.84375f},
		{0.0f,0.0f,0.875f},
		{0.0f,0.0f,0.90625f},
		{0.0f,0.0f,0.9375f},
		{0.0f,0.0f,0.96875f},
		{0.0f,0.0f,1.0f},
		{0.0f,0.03125f,1.0f},
		{0.0f,0.0625f,1.0f},
		{0.0f,0.09375f,1.0f},
		{0.0f,0.125f,1.0f},
		{0.0f,0.15625f,1.0f},
		{0.0f,0.1875f,1.0f},
		{0.0f,0.21875f,1.0f},
		{0.0f,0.25f,1.0f},
		{0.0f,0.28125f,1.0f},
		{0.0f,0.3125f,1.0f},
		{0.0f,0.34375f,1.0f},
		{0.0f,0.375f,1.0f},
		{0.0f,0.40625f,1.0f},
		{0.0f,0.4375f,1.0f},
		{0.0f,0.46875f,1.0f},
		{0.0f,0.5f,1.0f},
		{0.0f,0.53125f,1.0f},
		{0.0f,0.5625f,1.0f},
		{0.0f,0.59375f,1.0f},
		{0.0f,0.625f,1.0f},
		{0.0f,0.65625f,1.0f},
		{0.0f,0.6875f,1.0f},
		{0.0f,0.71875f,1.0f},
		{0.0f,0.75f,1.0f},
		{0.0f,0.78125f,1.0f},
		{0.0f,0.8125f,1.0f},
		{0.0f,0.84375f,1.0f},
		{0.0f,0.875f,1.0f},
		{0.0f,0.90625f,1.0f},
		{0.0f,0.9375f,1.0f},
		{0.0f,0.96875f,1.0f},
		{0.0f,1.0f,1.0f},
		{0.03125f,1.0f,0.96875f},
		{0.0625f,1.0f,0.9375f},
		{0.09375f,1.0f,0.90625f},
		{0.125f,1.0f,0.875f},
		{0.15625f,1.0f,0.84375f},
		{0.1875f,1.0f,0.8125f},
		{0.21875f,1.0f,0.78125f},
		{0.25f,1.0f,0.75f},
		{0.28125f,1.0f,0.71875f},
		{0.3125f,1.0f,0.6875f},
		{0.34375f,1.0f,0.65625f},
		{0.375f,1.0f,0.625f},
		{0.40625f,1.0f,0.59375f},
		{0.4375f,1.0f,0.5625f},
		{0.46875f,1.0f,0.53125f},
		{0.5f,1.0f,0.5f},
		{0.53125f,1.0f,0.46875f},
		{0.5625f,1.0f,0.4375f},
		{0.59375f,1.0f,0.40625f},
		{0.625f,1.0f,0.375f},
		{0.65625f,1.0f,0.34375f},
		{0.6875f,1.0f,0.3125f},
		{0.71875f,1.0f,0.28125f},
		{0.75f,1.0f,0.25f},
		{0.78125f,1.0f,0.21875f},
		{0.8125f,1.0f,0.1875f},
		{0.84375f,1.0f,0.15625f},
		{0.875f,1.0f,0.125f},
		{0.90625f,1.0f,0.09375f},
		{0.9375f,1.0f,0.0625f},
		{0.96875f,1.0f,0.03125f},
		{1.0f,1.0f,0.0f},
		{1.0f,0.96875f,0.0f},
		{1.0f,0.9375f,0.0f},
		{1.0f,0.90625f,0.0f},
		{1.0f,0.875f,0.0f},
		{1.0f,0.84375f,0.0f},
		{1.0f,0.8125f,0.0f},
		{1.0f,0.78125f,0.0f},
		{1.0f,0.75f,0.0f},
		{1.0f,0.71875f,0.0f},
		{1.0f,0.6875f,0.0f},
		{1.0f,0.65625f,0.0f},
		{1.0f,0.625f,0.0f},
		{1.0f,0.59375f,0.0f},
		{1.0f,0.5625f,0.0f},
		{1.0f,0.53125f,0.0f},
		{1.0f,0.5f,0.0f},
		{1.0f,0.46875f,0.0f},
		{1.0f,0.4375f,0.0f},
		{1.0f,0.40625f,0.0f},
		{1.0f,0.375f,0.0f},
		{1.0f,0.34375f,0.0f},
		{1.0f,0.3125f,0.0f},
		{1.0f,0.28125f,0.0f},
		{1.0f,0.25f,0.0f},
		{1.0f,0.21875f,0.0f},
		{1.0f,0.1875f,0.0f},
		{1.0f,0.15625f,0.0f},
		{1.0f,0.125f,0.0f},
		{1.0f,0.09375f,0.0f},
		{1.0f,0.0625f,0.0f},
		{1.0f,0.03125f,0.0f},
		{1.0f,0.0f,0.0f},
		{0.96875f,0.0f,0.0f},
		{0.9375f,0.0f,0.0f},
		{0.90625f,0.0f,0.0f},
		{0.875f,0.0f,0.0f},
		{0.84375f,0.0f,0.0f},
		{0.8125f,0.0f,0.0f},
		{0.78125f,0.0f,0.0f},
		{0.75f,0.0f,0.0f},
		{0.71875f,0.0f,0.0f},
		{0.6875f,0.0f,0.0f},
		{0.65625f,0.0f,0.0f},
		{0.625f,0.0f,0.0f},
		{0.59375f,0.0f,0.0f},
		{0.5625f,0.0f,0.0f},
		{0.53125f,0.0f,0.0f},
		{0.5f,0.0f,0.0f}
};

void
colorDepthImage(cv::Mat& imgDepth, cv::Mat& imgColoredDepth)
{
	imgColoredDepth = cv::Mat::zeros(imgDepth.size(), CV_8UC3);

	for (int i = 0; i < imgColoredDepth.rows; ++i)
	{
		const float* depth = imgDepth.ptr<float>(i);
		unsigned char* pixel = imgColoredDepth.ptr<unsigned char>(i);
		for (int j = 0; j < imgColoredDepth.cols; ++j)
		{
			if (depth[j] != 0)
			{
				int idx = fminf(depth[j], 10.0f) / 10.0f * 127.0f;
				idx = 127 - idx;

				pixel[0] = colormap_jet[idx][2] * 255.0f;
				pixel[1] = colormap_jet[idx][1] * 255.0f;
				pixel[2] = colormap_jet[idx][0] * 255.0f;
			}

			pixel += 3;
		}
	}
}

void
signalHandler(int signal)
{
	if (signal == SIGINT)
	{
		fprintf(stderr, "# INFO: Quitting...\n");
		quit = true;
		exit(EXIT_SUCCESS);
	}
}

//BallDetector ballDetector;
StereoProc stereoProc("/home/mattis/ETHZ/CVLabs/cvl11/src/examples/calib_stereo_bravo_bluefox.scf");
/**
 * @brief Handle incoming MAVLink packets containing images
 *
 */
void imageHandler(const lcm_recv_buf_t* rbuf, const char* channel,
		const mavconn_mavlink_msg_container_t* container, void* user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	// Pointer to shared memory data
	PxSHMImageClient* client = static_cast<PxSHMImageClient*>(user);

	cv::Mat imgToSave;

	//printf("GOT IMG MSG\n");

	// read mono image data
	cv::Mat img_left;
	cv::Mat img_right;
	if (client->readStereoImage(msg, img_left,img_right))
	{
		//////////////////////////////////////
		// APPLY ONE OF THE OPENCV FUNCTIONS HERE, AND OUTPUT IMAGE HERE
		//////////////////////////////////////

		struct timeval tv;
		gettimeofday(&tv, NULL);
		uint64_t currTime = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
		uint64_t timestamp = client->getTimestamp(msg);
		cv::Mat img_left_und;
		cv::Mat img_right_und;
		stereoProc.undistort(img_left,img_right,img_left_und,img_right_und);
		//ballDetector.addData(img_left_und,Mat(),img_right_und,Mat(),timestamp);
		//ballDetector.render(img_left_und,img_right_und);

		float roll,pitch,yaw;
		client->getRollPitchYaw(msg,roll,pitch,yaw);
		std::cout << roll << " " << pitch << " " << yaw << std::endl;
		//swap roll and pitch to account for the
		float tmp = roll;
		roll=pitch;
		pitch = tmp;

		//Debug
		//roll=pitch = 0;


		float x,y,z;
		client->getGroundTruth(msg,x,y,z);
		//Debug
		//x = y = z = 0;
		float ca = cos(-yaw),sa = sin(-yaw),cb = cos(pitch),sb = sin(pitch),cg = cos(roll),sg = sin(roll);
		double rotMat[4][4] =
			{{ca*cb,    ca*sb*sg-sa*cg,      ca*sb*cg+sa*sg,    x},
			 {sa*cb,    sa*sb*sg+ca*cg,      sa*sb*cg-ca*sg,    y},
			 {-sb,          cb*sg,               cb*cg,         z},
			 {0,              0,                   0,           1}};

		//x->z
		//y->x
		//z->y
		double rotMat2[4][4] =
		{{0,1,0,0},
		 {0,0,1,0},
		 {1,0,0,0},
		 {0,0,0,1}
		};


		cv::Mat rot(4,4,CV_64F,rotMat);
		std::cout << rot << std::endl;
		cv::Mat rot2(4,4,CV_64F,rotMat2);

		cv::Mat P1 = stereoProc.P1*rot2*rot;
		cv::Mat P2 = stereoProc.P2*rot2*rot;


		//cout << stereoProc.P1 << endl;
		renderLine(img_left_und,P1,cv::Point3d(-10,0,0),cv::Point3d(10,0,0),cv::Scalar(255,0,0));
		renderLine(img_left_und,P1,cv::Point3d(0,-10,0),cv::Point3d(0,10,0),cv::Scalar(125,0,0));
		//cout << stereoProc.P1 << endl;
		renderPoint(img_left_und,stereoProc.P1,cv::Point3d(0,0.1,2),cv::Scalar(255,0,0));
		renderPoint(img_left_und,stereoProc.P1,cv::Point3d(0,0,2),cv::Scalar(125,0,0));
		renderPoint(img_left_und,stereoProc.P1,cv::Point3d(0.1,0,2),cv::Scalar(0,0,0));
		renderPoint(img_left_und,P1,cv::Point3d(0,-2,0),cv::Scalar(125,0,0));
		renderPoint(img_left_und,P1,cv::Point3d(0,-1.9,0),cv::Scalar(0,0,0));
		renderPoint(img_left_und,P1,cv::Point3d(0,-2,0.1),cv::Scalar(255,0,0));

		//double origin[4] = {0,2,0,1};
		//Mat orig = P1*Mat(4,1,CV_64F,origin);
		//cout << orig.at<double>(0,0)/orig.at<double>(2,0) << " " <<orig.at<double>(1,0)/orig.at<double>(2,0) << endl;

#if 1
		cv::Mat depth;
		cv::Mat undistorted;
		stereoProc.process(img_left,img_right,undistorted,depth);
		cv::namedWindow("Depth");
		cv::Mat colorDepth;
		colorDepthImage(depth,colorDepth);
		cv::imshow("Depth",colorDepth);
#endif
		uint64_t diff = currTime - timestamp;

		if (verbose)
		{
			fprintf(stderr, "# INFO: Time from capture to display: %llu ms for camera %llu\n", diff / 1000, client->getCameraID(msg));
		}

		// Display if switched on
#ifndef NO_DISPLAY
		if ((client->getCameraConfig() & PxSHM::CAMERA_FORWARD_LEFT) == PxSHM::CAMERA_FORWARD_LEFT)
		{
			cv::namedWindow("Left Image (Forward Camera)");
			cv::imshow("Left Image (Forward Camera)", img_left_und);
		}
		else
		{
			cv::namedWindow("Left Image (Downward Camera)");
			cv::imshow("Left Image (Downward Camera)", img_left_und);
		}
		if ((client->getCameraConfig() & PxSHM::CAMERA_FORWARD_RIGHT) == PxSHM::CAMERA_FORWARD_RIGHT)
		{
			cv::namedWindow("Right Image (Forward Camera)");
			cv::imshow("Right Image (Forward Camera)", img_right_und);
		}
		else
		{
			cv::namedWindow("Right Image (Downward Camera)");
			cv::imshow("Right Image (Downward Camera)", img_right_und);
		}
#endif
		img_left.copyTo(imgToSave);
	}

#ifndef NO_DISPLAY
	int c = cv::waitKey(3);
	switch (static_cast<char>(c))
	{
	case 'f':
	{
		char index[20];
		sprintf(index, "%04d", imageCounter++);
		cv::imwrite(std::string(fileBaseName+index+fileExt).c_str(), imgToSave);
	}
	break;
	default:
		break;
	}
#endif
}

static void
mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel,
		const mavconn_mavlink_msg_container_t* container, void * user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);
	mavlink_message_t response;
	lcm_t* lcm = static_cast<lcm_t*>(user);
	//printf("Received message #%d on channel \"%s\" (sys:%d|comp:%d):\n", msg->msgid, channel, msg->sysid, msg->compid);

	switch(msg->msgid)
	{
	uint32_t receiveTime;
	uint32_t sendTime;
	case MAVLINK_MSG_ID_ATTITUDE:
		gettimeofday(&tv, NULL);
		receiveTime = tv.tv_usec;
		sendTime = mavlink_msg_attitude_get_time_boot_ms(msg);
		//printf("Received attitude message, transport took %f ms\n", (receiveTime - sendTime)/1000.0f);
		break;
	case MAVLINK_MSG_ID_GPS_RAW_INT:
	{
		mavlink_gps_raw_int_t gps;
		mavlink_msg_gps_raw_int_decode(msg, &gps);
		//printf("GPS: lat: %f, lon: %f, alt: %f\n", gps.lat/(double)1E7, gps.lon/(double)1E7, gps.alt/(double)1E6);
		break;
	}
	case MAVLINK_MSG_ID_RAW_PRESSURE:
	{
		mavlink_raw_pressure_t p;
		mavlink_msg_raw_pressure_decode(msg, &p);
		//printf("PRES: %f\n", p.press_abs/(double)1000);
	}
	break;
	default:
		//printf("ERROR: could not decode message with ID: %d\n", msg->msgid);
		break;
	}
}


void* lcm_wait(void* lcm_ptr)
{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (1)
	{
		lcm_handle (lcm);
	}
	return NULL;
}

// Handling Program options
static GOptionEntry entries[] =
{
		{ "sysid", 'a', 0, G_OPTION_ARG_INT, &sysid, "ID of this system, 1-255", "42"},
		{ "compid", 'c', 0, G_OPTION_ARG_INT, &compid, "ID of this component, 1-255", "55" },
		{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", (verbose) ? "true" : "false" },
		{ "debug", 'd', 0, G_OPTION_ARG_NONE, &debug, "Debug mode, changes behaviour", (debug) ? "true" : "false" },
		{ "config", 'g', 0, G_OPTION_ARG_STRING, configFile, "Filename of paramClient config file", "config/parameters_2pt.cfg"},
		{ NULL }
};

int main(int argc, char* argv[])
{
	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- localize based on natural features");
	g_option_context_add_main_entries (context, entries, "Localization");
	if (!g_option_context_parse (context, &argc, &argv, &error))
	{
		g_print ("Option parsing failed: %s\n", error->message);
		exit(EXIT_FAILURE);
	}
	g_option_context_free(context);
	// Handling Program options
	lcm_t* lcm = lcm_create("udpm://");
	if (!lcm)
	{
		fprintf(stderr, "# ERROR: Cannot initialize LCM.\n");
		exit(EXIT_FAILURE);
	}

	mavconn_mavlink_msg_container_t_subscription_t * comm_sub =
			mavconn_mavlink_msg_container_t_subscribe (lcm, MAVLINK_MAIN, &mavlink_handler, lcm);

	// Thread
	GThread* lcm_thread;
	GError* err;

	if( !g_thread_supported() )
	{
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	if( (lcm_thread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcm, TRUE, &err)) == NULL)
	{
		printf("Thread create failed: %s!!\n", err->message );
		g_error_free ( err ) ;
	}

	PxSHMImageClient client;
	client.init(true, PxSHM::CAMERA_FORWARD_LEFT, PxSHM::CAMERA_FORWARD_RIGHT);

	// Ready to roll
	fprintf(stderr, "# INFO: Image client ready, waiting for images..\n");

	// Subscribe to MAVLink messages on the image channel
	mavconn_mavlink_msg_container_t_subscription_t* imgSub = mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_IMAGES, &imageHandler, &client);

	signal(SIGINT, signalHandler);

	while (!quit)
	{
		// Block waiting for new image messages,
		// once an image is received it will be
		// displayed
		lcm_handle(lcm);
	}

	mavconn_mavlink_msg_container_t_unsubscribe(lcm, imgSub);
	mavconn_mavlink_msg_container_t_unsubscribe (lcm, comm_sub);
	lcm_destroy (lcm);
	g_thread_join(lcm_thread);
	return 0;
}

