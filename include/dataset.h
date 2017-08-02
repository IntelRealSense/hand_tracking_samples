//
//  datasets stream from depth camera and associated pose ground truth 
//  this module has the support code for serializing depth/ir data and the ground truth pose information
//
//  data file format and layout, system uses parallel files, 
//  filename.json  has the camera intrisics and other header data
//  filename.rs    binary depth unsigned short width x height x number_of_frames
//  filename.ir    raw ir data single 8 bit channel
//  filename.pose  ascii array of bone transformations, position and orientation per bone per frame
//

#pragma once
#ifndef DEPTH_CAMERA_DATASET_H
#define DEPTH_CAMERA_DATASET_H

#include "../third_party/geometric.h"
#include "../third_party/misc_json.h"
#include "../third_party/misc.h"
#include "misc_image.h"

struct DatasetInfo
{
	DCamera     dcamera;
	float4      mplane;
	std::string fname;
	std::string camtype;
	bool        hasir;  // this was for interleaved ir to be depricated
	int2        rgb_dim;
	int2        feye_dim;
	float       segment_scale;  // the hand size 
};
template<class F> void visit_fields(DatasetInfo &o, F f) 
{ 
	f("dcamera", o.dcamera); f("mplane", o.mplane)   ;  f("fname", o.fname);  f("camtype", o.camtype); f("hasir", o.hasir); 
	f("rgb_dim", o.rgb_dim); f("feyedim", o.feye_dim);  
	f("segment_scale", o.segment_scale); 
}


struct Frame
{
	Image<unsigned short> depth = Image<unsigned short>({ 0,0 });
	std::vector<Pose> pose;
	std::vector<Pose> startpose;  // in case I need to revert ctrl-z in gui
	float4 mplane = float4(0,0,0,FLT_MAX);
	Image<unsigned char> ir= Image<unsigned char>({ 0,0 });
	std::string fname;
	int fid =0;
	Image<byte3> rgb = Image<byte3>({ 0,0 });
	Image<unsigned char> fisheye = Image<unsigned char>({ 0,0 });
};
template<class F> void visit_fields(Frame & o, F f) { f("pose", o.pose); f("camera", o.depth.cam); f("mplane", o.mplane); }
Frame MakeFrame(Image<unsigned short> depth, std::vector<Pose> pose) { Frame f; f.depth = std::move(depth); f.pose = f.startpose = pose; return f; }
Frame MakeFrame(Image<unsigned short> depth, std::vector<Pose> pose, float4 mplane) { auto f = MakeFrame(depth, pose);f.mplane = mplane;return f; }
Frame MakeFrame(Image<unsigned short> depth, std::vector<Pose> pose, Image<unsigned char> ir, Image<byte3> rgb , Image<unsigned char> fisheye) 
{
	auto f = MakeFrame(depth, pose);
	f.ir = ir;  f.rgb = rgb;   f.fisheye = fisheye;
	return f;
}

struct DepthDataStreamOut
{
	std::string   prefix;
	std::ofstream file_out_depth;   // depth data    
	std::ofstream file_out_poses;   // ground truth pose data initial guess
	std::ofstream file_out_ir;      // ir stream_data data    
	std::ofstream file_out_rgb;     // color     
	std::ofstream file_out_feye;    // fish-eye     
	DepthDataStreamOut() {}
	DepthDataStreamOut(std::string prefix):prefix(prefix)
	{
		file_out_depth.open(prefix + ".rs"  , std::ios::binary | std::ios::trunc);
		file_out_ir.open   (prefix + ".ir"  , std::ios::binary | std::ios::trunc);
		file_out_poses.open(prefix + ".pose", std::ios::out    | std::ios::trunc);
	}
	DepthDataStreamOut& AddRGB()     { file_out_rgb.open (prefix + ".rgb" ,  std::ios::binary |std::ios::out | std::ios::trunc); return *this;}
	DepthDataStreamOut& AddFishEye() { file_out_feye.open(prefix + ".feye",  std::ios::binary |std::ios::out | std::ios::trunc); return *this;}
	DepthDataStreamOut(DatasetInfo dsi) :DepthDataStreamOut(dsi.fname)
	{
		std::ofstream(dsi.fname + ".json") << tabbed(to_json(dsi), 2) << std::endl;
	}
	static std::string NextFreeNameSet(std::string prefix) { return fileprefix(freefilename(prefix, ".rs")); }  // for finding next free set of file names to use for streaming out
	DepthDataStreamOut(std::string nameset, const DCamera &dcam, const float4 &mplane, std::string camtype,float segment_scale) :DepthDataStreamOut(DatasetInfo{ dcam,mplane,NextFreeNameSet(nameset),camtype  ,false,{640,480},{ 640,480 },segment_scale }) {}

	void SaveFrame(const Image<unsigned short> &dimage, const Image<unsigned char> &irimage, const std::vector<Pose> &pose)
	{
		if (!file_out_depth.is_open())
			throw "hey file wasn't opened";
		file_out_depth.write((char*)dimage.raster.data() , product(dimage.dim() ) * sizeof(*dimage.raster.data()) );
		file_out_ir.write   ((char*)irimage.raster.data(), product(irimage.dim()) * sizeof(*irimage.raster.data()));
		for (const auto & p : pose)
			file_out_poses << p.position << "  " << p.orientation << "   ";
		file_out_poses << "\n";
	};
	void SaveFrame (const Frame &frame) 
	{ 
		SaveFrame(frame.depth, frame.ir, frame.pose); 
		if (file_out_rgb.is_open() && product(frame.rgb.dim()))
			file_out_rgb.write((char*)frame.rgb.raster.data(), product(frame.rgb.dim()) * 3); //product(frame.rgb.dim())*sizeof(*frame.rgb.raster.data())    );
		if (file_out_feye.is_open() && product(frame.fisheye.dim()))
			file_out_feye.write((char*)frame.fisheye.raster.data(), product(frame.fisheye.dim()) ); //product(frame.fisheye.dim())*sizeof(*frame.fisheye.raster.data()));
	}
	void SaveFrames(const std::vector<Frame> &frames) { for(auto &frame:frames) SaveFrame(frame); }

};


std::vector<Frame> load_dataset(std::string bname, unsigned int pose_array_size , std::function<void(Frame&)> post_process = [](Frame&) {})
{
	std::cout << "Loading DataSet: " << bname << "\n";
	std::vector<Frame> frames;
	std::ifstream file_in_depth(bname + ".rs", std::ios_base::binary | std::ios_base::in);
	if (!file_in_depth.is_open())
		throw(std::runtime_error("unable to open .rs file"));
	DatasetInfo dsi;
	if (!fileexists(bname + ".json"))
		throw "file not found";
	from_json(dsi, json::parsefile(bname + ".json"));
	std::cout << "json:\n" << tabbed(to_json(dsi), 2) << std::endl;
	std::cout << "dsi.dcamera: " << dsi.dcamera << "\n" << "dsi.mplane " << dsi.mplane << "\n";

	std::ifstream file_in_pose   (bname + ".pose", std::ios_base::in );
	std::ifstream file_in_ir     (bname + ".ir"  , std::ios_base::in | std::ios_base::binary);
	std::ifstream file_in_rgb    (bname + ".rgb" , std::ios_base::in | std::ios_base::binary);
	std::ifstream file_in_fisheye(bname + ".feye", std::ios_base::in | std::ios_base::binary);
	//std::ifstream 
	std::cout << "importing data";
	int k = 0;
	while (!file_in_depth.eof())
	{
		std::vector<unsigned short> dbuf(product(dsi.dcamera.dim()));
		std::vector<unsigned char>  ibuf(product(dsi.dcamera.dim()), (unsigned char)0);   // ir matches depth buffer
		if (!file_in_depth.read((char*)dbuf.data(), dbuf.size()*sizeof(unsigned short)))
			break;
		if (dsi.hasir &&!file_in_depth.read((char*)ibuf.data(), ibuf.size()*sizeof(unsigned char)))   // if interleaved depth and ir  - this should be depricated
			break;
		if (file_in_ir.is_open())
			file_in_ir.read((char*)ibuf.data(), ibuf.size()*sizeof(unsigned char));
		Image<unsigned char> feyeimage(dsi.feye_dim);
		if (product(feyeimage.dim()) && file_in_fisheye.is_open())
			file_in_fisheye.read((char*)feyeimage.raster.data(), product(feyeimage.dim())*sizeof(*feyeimage.raster.data()));
		Image<byte3> cimage(dsi.rgb_dim);
		if (product(cimage.dim()) && file_in_rgb.is_open())
			file_in_rgb.read((char*)cimage.raster.data(), product(cimage.dim())*sizeof(*cimage.raster.data()));
		std::vector<Pose> pose(pose_array_size);
		if (file_in_pose.is_open()) for (auto &p : pose)
			file_in_pose >> p;
		std::vector<unsigned short> irbuf(product(dsi.dcamera.dim()));
		auto f = MakeFrame(Image<unsigned short>(dsi.dcamera, dbuf),pose, Image<unsigned char>{ dsi.dcamera,ibuf } ,cimage,feyeimage);
		f.fname = bname;f.fid = k;  
		post_process(f);
		frames.push_back(f);
		if (k++ % 100 == 0)
		{
			std::cout << ".";
			std::cout.flush();
		}
	}
	std::cout << "done\n";
	std::cout.flush();
	return frames;
}




#endif // DEPTH_CAMERA_DATASET_H

