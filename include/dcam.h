//
// dcam.h   wrapper for librealsense   
//
// Any project that uses this wrapper should have librealsense.  
// Statically linked version should work for most projects.
// At least on windows, librealsense should be in the third_party subfolder.
// for windows compilation/linking it use pragmas so hopefully builds work the first time. 
// This wrapper helps provide the necessary include and linking directives in this file.
//
// Any of the initial functionality for working with depth data is part of this header file.
// This includes typical camera setup for our tracking usages and first past data filtering 
// and resizing.  See the implementation of RSCam below.  
//
// The wrapper returns the streaming frames using the Image<T> class which includes the 
// associated camera information along with some convenience functions for working with the data.
// See misc_image.h for details.
//
//

#pragma once
#ifndef RS_CAM_H
#define RS_CAM_H

#include <assert.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <exception>

#include "../third_party/misc_json.h"
#include "../third_party/geometric.h"
#include "../third_party/misc.h"
#include "misc_image.h"

// note, on mac if librealsense has been brew installed and the makefile is linking with an installed librealsense, then the include below is not good
#include "../third_party/librealsense/include/librealsense/rs.hpp"
#include "../third_party/librealsense/include/librealsense/rsutil.h"

#ifdef RS_USE_LIBREALSENSE_DLL
#ifdef _WIN64
#ifdef _DEBUG
#pragma comment(lib, "../third_party/librealsense/bin/x64/realsense-d.lib") 
#else  // Release win32
#pragma comment(lib, "../third_party/librealsense/bin/x64/realsense.lib") 
#endif // _DEBUG vs release for x64
#else WIN32
#ifdef _DEBUG
#pragma comment(lib, "../third_party/librealsense/bin/Win32/realsense-d.lib") 
#else  // Release win32
#pragma comment(lib, "../third_party/librealsense/bin/Win32/realsense.lib") 
#endif // _DEBUG vs release for  win32
#endif // WIN_32  // done DLL cases
#else  // Statically Linked 
#if (_MSC_VER >= 1910 )
#ifdef _WIN64
#ifdef _DEBUG
#pragma comment(lib, "../third_party/librealsense/librealsense.vc15/realsense-s/obj/Debug-x64/realsense-sd.lib") 
#else  // release x64
#pragma comment(lib, "../third_party/librealsense/librealsense.vc15/realsense-s/obj/Release-x64/realsense-s.lib")   
#endif // _DEBUG vs release for x64
#else  // we have _WIN32
#ifdef _DEBUG
#pragma comment(lib, "../third_party/librealsense/librealsense.vc15/realsense-s/obj/Debug-Win32/realsense-sd.lib")  
#else  // release 32 
#pragma comment(lib, "../third_party/librealsense/librealsense.vc15/realsense-s/obj/Release-Win32/realsense-s.lib")   // pragma easier than using settings within project properties
#endif // _DEBUG vs release for Win32 
#endif // _WIN64 vs _WIN32
#else // _MSC_VER < 1910  before vs2017
#ifdef _WIN64
#ifdef _DEBUG
#pragma comment(lib, "../third_party/librealsense/librealsense.vc14/realsense-s/obj/Debug-x64/realsense-sd.lib") 
#else  // release x64
#pragma comment(lib, "../third_party/librealsense/librealsense.vc14/realsense-s/obj/Release-x64/realsense-s.lib")   
#endif // _DEBUG vs release for x64
#else  // we have _WIN32
#ifdef _DEBUG
#pragma comment(lib, "../third_party/librealsense/librealsense.vc14/realsense-s/obj/Debug-Win32/realsense-sd.lib")  
#else  // release 32 
#pragma comment(lib, "../third_party/librealsense/librealsense.vc14/realsense-s/obj/Release-Win32/realsense-s.lib")   // pragma easier than using settings within project properties
#endif // _DEBUG vs release for Win32 
#endif // _WIN64 vs _WIN32
#endif // _MSC_VER
#endif // RS_LOCAL

template<class F> void visit_fields(rs::intrinsics &i, F f) { f("coeffs", i.coeffs);f("fx", i.fx);f("fy", i.fy);f("ppx", i.ppx);f("ppy", i.ppy); f("width", i.width);f("height", i.height); }
template<class F> void visit_fields(rs::extrinsics &e, F f) { f("translation", e.translation); f("rotation", e.rotation); }
struct intrinsic_set { rs::intrinsics depth_intrinsics, color_intrinsics, feye_intrinsics;rs::extrinsics d2f, d2c; };
template<class F> void visit_fields(intrinsic_set &s, F f) { f("depth_intrinsics", s.depth_intrinsics); f("color_intrinsics", s.color_intrinsics);f("feye_intrinsics", s.feye_intrinsics); f("d2f", s.d2f); f("d2c", s.d2c); }
intrinsic_set MakeIntrinsicSet(rs::device *&dev)
{
	return intrinsic_set{ dev->get_stream_intrinsics(rs::stream::depth) ,dev->get_stream_intrinsics(rs::stream::color) ,dev->get_stream_intrinsics(rs::stream::fisheye),
	                      dev->get_extrinsics(rs::stream::depth, rs::stream::fisheye),dev->get_extrinsics(rs::stream::depth, rs::stream::color) };
}


//
// DCam  
//
// Parent class to RSCam, the librealsense wrapper.
// Provides a generic depth camera interface based on rs::intrinsics. 
// Converts to/from the generic DCamera class used in misc_image.h.
//
class DCam 
{
public:
	rs::intrinsics zintrin;
	float          depth_scale = 0.001f;
	float          get_depth_scale(){ return depth_scale;  }
	const int2&    dim()      const { return *(reinterpret_cast<const int2*  >(&zintrin.width)); }
	int2&          dim()            { return *(reinterpret_cast<      int2*  >(&zintrin.width)); }
	const float2&  focal()    const { return *(reinterpret_cast<const float2*>(&zintrin.fx));    }
	float2&        focal()          { return *(reinterpret_cast<      float2*>(&zintrin.fx));    }
	const float2&  principal()const { return *(reinterpret_cast<const float2*>(&zintrin.ppx));   }
	float2&        principal()      { return *(reinterpret_cast<      float2*>(&zintrin.ppx));   }

	float2  fov()   const { return{ zintrin.hfov(), zintrin.vfov() }; }  // in degrees
	inline float3   deprojectz(float2 p, float d)             const { auto t = zintrin.deproject({p.x, p.y}, d); return float3(t.x,t.y,t.z); }
	inline float3   deprojectz(int2 p, unsigned short d)      const { return deprojectz(float2((float)p.x, (float)p.y), d); }
	inline float3   deprojectz(int x, int y, unsigned short d)const { return deprojectz(float2((float)x  , (float)y  ), d); }
	inline float3   deprojectz(int x, int y, float d)         const { return deprojectz(float2((float)x  , (float)y  ), d); }
	inline float2   project(const float3 &v)                  const { auto t = zintrin.project((const rs::float3&) v); return float2(t.x,t.y); }
	inline float2x2 deprojectextents()                        const { return float2x2(deprojectz(float2(0, 0), 1.0f).xy(), deprojectz(asfloat2(dim()), 1.0f).xy()); }   
	DCamera dcamera() { return { dim(), focal(), principal() ,get_depth_scale() };  }
	void intrinsicsimport(const DCamera &dcamera)
	{
		dim() = dcamera.dim(); 
		focal() = dcamera.focal(); 
		depth_scale = dcamera.depth_scale;
		principal() = dcamera.principal();
	}

};

//
// RSCam
//
// Wrapper for RealSense camera. 
// This implementation intended to support close range hand tracking research.
// Has a feature for streaming from file as if it were a device. 
// Automatic downsampling to a suitable image size.
// For DS4 it filters out depth pixels that are dark in IR to clean up contours. 
//
class RSCam : public DCam   
{
public:
	rs::context ctx;
	rs::device * dev;

	std::ifstream                    filein;
	std::vector<unsigned short>      fbuffer;
	bool                             enable_filter_depth = true;
	int                              image_width_max_allowed = 320;

	std::vector<uint16_t>            filtered_depth;
	std::vector<uint16_t>            background;
	void  addbackground(const unsigned short *dp,unsigned short fudge=3) 
	{
		background.resize(product(dim()),4096);
		for (int i = 0; i < dim().x*dim().y; i++)
			background[i] = std::min(background[i], (unsigned short) (dp[i]-fudge));
	}
	Image<unsigned char> cache_ir;
	void UpdateIR()
	{
		if (!dev)
			return;
		auto ir=(const uint8_t *)dev->get_frame_data(rs::stream::infrared);
		cache_ir = Image<unsigned char>(this->dcamera(),std::vector<unsigned char>(ir,ir+product(dim())));
		while (cache_ir.dim().x > image_width_max_allowed)
			cache_ir = DownSampleFst(cache_ir);

	}
	Image<unsigned short> FilterDS4(unsigned short *dp)  // modified in-place  very ds4 specific
	{
		Image<unsigned short> filtered_depth(dcamera(),std::vector<unsigned short>(dp, dp + product(dim())));
		dp = filtered_depth.raster.data();
		UpdateIR();
		auto dleft = cache_ir.raster.data();// (const uint8_t *)dev->get_frame_data(rs::stream::infrared);
		// depth_scale = dev->get_depth_scale() ;   <- to do the math in meters use this term   and dont assume native is mm
		auto hasneighbor = [](unsigned short *p, int stride) ->bool	{ return (std::abs(p[stride] - p[0]) < 10 || std::abs(p[-stride] - p[0]) < 10); };
		for (int i = 0; i < dim().x*dim().y; i++)
		{
			if(dp[i]<30  || dleft[i] <8)  // ignore dark pixels - these could be background, but stereo match put them in foreground
			{
				dp[i] = 4096;
			}
		}
		for (int i = 0; i < dim().x*dim().y; i++) // for (int2 p: rect_iteration(dim()))
		{
			int x = i%dim().x, y = i / dim().x;
			if (x < 2 || x >= dim().x - 2 || y < 2 || y >= dim().y - 2)
				continue;
			if ( !hasneighbor(dp + i, 1) || !hasneighbor(dp + i, dim().x) || !hasneighbor(dp + i, 2) || !hasneighbor(dp + i, dim().x * 2))  // ignore flying pixels
			{
				dp[i] = 4096;
			}
		}
		for (int i = 0; i < dim().x*dim().y; i++)
		{
			if (background.size() == dim().x*dim().y)
				if (dp[i]>background[i])
					dp[i] = 4096;
		}
		while (filtered_depth.dim().x > image_width_max_allowed)  // typically we dont want more than 320x240
			filtered_depth = DownSampleFst(filtered_depth);
		return filtered_depth;
	}
	Image<unsigned short> FilterIvy(unsigned short *dp)  // modified in-place  ivy cam specific
	{
		Image<unsigned short> filtered_depth(dcamera(), std::vector<unsigned short>(dp, dp + product(dim())));
		while (filtered_depth.dim().x > image_width_max_allowed)  // typically we dont want more than 320x240
			filtered_depth = DownSampleFst(filtered_depth);

		dp = filtered_depth.raster.data();
		auto constant_number = (unsigned short)(4.0f / dev->get_depth_scale());
		for (int i = 0; i < product(filtered_depth.dim()); i++)
		{
			if (dp[i] == 0)
			{
				dp[i] = constant_number;
			}
		}
		UpdateIR();
		return filtered_depth;
	}
	Image<unsigned short>  GetFileDepth()
	{
		Image<unsigned short> image(dcamera(), std::vector<unsigned short>(product(dim())));
		filein.read((char*)image.raster.data(), sizeof(unsigned short)*image.raster.size());
		return image;
	}
	Image<unsigned short> ToImage(unsigned short *p)
	{
		Image<unsigned short> image(dcamera() , std::vector<unsigned short>(p, p + product(dim())));
		return image;
	}
	Image<unsigned short> GetDepth()
	{ 
		if (filein.is_open()) return GetFileDepth(); 
		dev->wait_for_frames();
		auto rawdepth = (uint16_t  *)dev->get_frame_data(rs::stream::depth);
		return (enable_filter_depth)? ((dev->get_stream_mode_count(rs::stream::infrared2))?FilterDS4(rawdepth):FilterIvy(rawdepth)): ToImage(rawdepth);  // filter is r200 specific 
	}
	std::string CamName() { return (dev) ? ((dev->get_stream_mode_count(rs::stream::infrared2)) ? "dscam" : "ivycam") : "filecam"; }
	Image<unsigned char> fisheye= Image<unsigned char>({ 640,480 });
	Image<unsigned char> &GetFishEye()
	{
		if(! dev->is_stream_enabled(rs::stream::fisheye))
			return fisheye;
		auto data = (unsigned char*)dev->get_frame_data(rs::stream::fisheye);
		fisheye.raster = std::vector<unsigned char>(data, data + product(fisheye.dim()));
		return fisheye;
	}
	inline bool Init()
	{
		if (ctx.get_device_count() == 0) throw std::runtime_error("No device found");
		dev = ctx.get_device(0);
		std::cout << "Found Device:  '" << dev->get_name() << "'" << std::endl;
		std::cout << "Firmware version: " << dev->get_firmware_version() << std::endl;
		std::cout << "Serial number: " << dev->get_serial() << std::endl;
		try
		{
			dev->enable_stream(rs::stream::depth, 320, 240, rs::format::z16, 60);
			dev->enable_stream(rs::stream::infrared, 320, 240, rs::format::y8, 60);
			//dev->enable_stream(rs::stream::infrared2, 320, 240, rs::format::y8, 60);
			dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
			if (dev->supports(rs::capabilities::fish_eye))
			{
				dev->set_option(rs::option::fisheye_color_auto_exposure, 1);
				dev->enable_stream(rs::stream::fisheye, 640, 480, rs::format::raw8, 60);
			}

			dev->start();
			if (dev->supports_option(rs::option::r200_lr_auto_exposure_enabled)) {
				dev->set_option(rs::option::r200_lr_auto_exposure_enabled, 1);
				dev->set_option(rs::option::r200_auto_exposure_mean_intensity_set_point, 180.0);
				//rs_apply_depth_control_preset((rs_device*)dev, 4); // from leo: Bit less noise
				rs_apply_depth_control_preset((rs_device*)dev, 2); // from leo: denser data
			}

		}
		catch (...)
		{
			dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
			dev->enable_stream(rs::stream::infrared,640, 480, rs::format::y8, 60);
			dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 30);
			dev->start();
		}

		if (dev->supports_option(rs::option::sr300_auto_range_enable_laser)) {
			if (false && dev) {
				const rs_option arr_options[] = {
					RS_OPTION_SR300_AUTO_RANGE_ENABLE_MOTION_VERSUS_RANGE,
					RS_OPTION_SR300_AUTO_RANGE_ENABLE_LASER,
					RS_OPTION_SR300_AUTO_RANGE_MIN_MOTION_VERSUS_RANGE,
					RS_OPTION_SR300_AUTO_RANGE_MAX_MOTION_VERSUS_RANGE,
					RS_OPTION_SR300_AUTO_RANGE_START_MOTION_VERSUS_RANGE,
					RS_OPTION_SR300_AUTO_RANGE_MIN_LASER,
					RS_OPTION_SR300_AUTO_RANGE_MAX_LASER,
					RS_OPTION_SR300_AUTO_RANGE_START_LASER,
					RS_OPTION_SR300_AUTO_RANGE_UPPER_THRESHOLD,
					RS_OPTION_SR300_AUTO_RANGE_LOWER_THRESHOLD,
					RS_OPTION_F200_LASER_POWER,
					RS_OPTION_F200_ACCURACY,
					RS_OPTION_F200_FILTER_OPTION,
					RS_OPTION_F200_CONFIDENCE_THRESHOLD,
					RS_OPTION_F200_MOTION_RANGE
				};

				const double arr_values[][15] = {
					{ 1,     1, 180,  303,  180,   2,  16,  -1, 1000, 450,  1,  1,  5,  1, -1 }, /* ShortRange                */
					{ 1,     0, 303,  605,  303,  -1,  -1,  -1, 1250, 975,  1,  1,  7,  0, -1 }, /* LongRange                 */
					{ 0,     0,  -1,   -1,   -1,  -1,  -1,  -1,   -1,  -1, 16,  1,  6,  2, 22 }, /* BackgroundSegmentation    */
					{ 1,     1, 100,  179,  100,   2,  16,  -1, 1000, 450,  1,  1,  6,  3, -1 }, /* GestureRecognition        */
					{ 0,     1,  -1,   -1,   -1,   2,  16,  16, 1000, 450,  1,  1,  3,  1,  9 }, /* ObjectScanning            */
					{ 0,     0,  -1,   -1,   -1,  -1,  -1,  -1,   -1,  -1, 16,  1,  5,  1, 22 }, /* FaceAnalytics             */
					{ 2,     0,  40, 1600,  800,  -1,  -1,  -1,   -1,  -1,  1, -1, -1, -1, -1 }, /* FaceLogin                 */
					{ 1,     1, 100,  179,  179,   2,  16,  -1, 1000, 450,  1,  1,  6,  1, -1 }, /* GRCursor                  */
					{ 0,     0,  -1,   -1,   -1,  -1,  -1,  -1,   -1,  -1, 16,  1,  5,  3,  9 }, /* Default                   */
					{ 1,     1, 180,  605,  303,   2,  16,  -1, 1250, 650,  1,  1,  5,  1, -1 }, /* MidRange                  */
					{ 2,     0,  40, 1600,  800,  -1,  -1,  -1,   -1,  -1,  1, -1, -1, -1, -1 }, /* IROnly                    */
					{ 0,     1,  -1,   -1,   -1,  -1,  -1,  -1, 1250, 450,  1,  2,  5,  5, -1 }, /* Special                    */
					{ 0,     0,  -1,   -1,   -1,  -1,  -1,  -1, 1250, 450, 16,  2,  5,  3,  9 }, /* Special2                   */
					{ 0,     0,  -1,   -1,   -1,  -1,  -1,  -1, 1250, 450, 16,  2,  0,  0, -1 }  /* RAW                        */

				};

				{
					auto device = (rs_device*)dev;
					auto preset = 12;
					if (arr_values[preset][14] != -1) rs_set_device_options(device, arr_options, 15, arr_values[preset], 0);
					if (arr_values[preset][13] != -1) rs_set_device_options(device, arr_options, 14, arr_values[preset], 0);
					else rs_set_device_options(device, arr_options, 11, arr_values[preset], 0);
				}
			}
		}

		zintrin = dev->get_stream_intrinsics(rs::stream::depth);
		depth_scale = dev->get_depth_scale();
		std::cout << " dims: " << this->dim() << "  " << this->focal() <<  "  " << this->principal()  << "  " << this->depth_scale << "  " << fov() << "  // w h fx fy px py depth_scale fovx fovy (degrees)\n"; 

		return true;
	}
	inline bool Init(const char *filename_)
	{
		if (!filename_ || !*filename_)
			return Init();
		filein.open(filename_, std::ios_base::binary | std::ios_base::in);
		if (!filein.is_open())
			throw(std::runtime_error((std::string("unable to open depth camera file:  ")+filename_).c_str()));
		std::cout << "loading json file for depth camera specs " << fileprefix(filename_) <<  std::endl;
		DCamera fcam;
		from_json(fcam,json::parsefile(fileprefix(filename_) + ".json").get_object().at("dcamera"));
		//std::cout << json::tabbed(to_json(fcam)) << std::endl; 
		intrinsicsimport(fcam);
		std::cout << "depth image size " << dim() << std::endl; 
		fbuffer.resize(dim().x*dim().y);
		return true;
	}
	RSCam() :dev(nullptr), cache_ir({ 320,240 },(unsigned char)0), image_width_max_allowed(320)
	{
	}
};

#endif
