//
//  dataset-exporter   for exporting the ground truth data from the binary streams into individual image files per frame
// 
//  Current implementation will put everthing into subfolder 'tmp'
//  Currently needs to be run from a directory that is a subdirectory of the main solution directory
//  Also creates two ascii text files labels_full.txt and labels_seg.txt that provide image xy locations of key locations and othre values
//  Each line corresponds to one frame.  See code for what each of these values refer to.
//  This is meant to be reference code which can be easily modified  as needed to export desired data in preferred format.
// 

#include <iostream>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../third_party/stb_image_write.h"

#include "../third_party/geometric.h"  // the core geometry libs and consequently includes the standard math we use
#include "../third_party/misc.h"
#include "../third_party/json.h"
#include "../third_party/misc_json.h"
#include "../include/misc_image.h"
#include "../include/handtrack.h"  
#include "../include/dataset.h"


inline void ImageWrite(std::string prefix,int k, const Image<byte3> &image)  
{
	std::string filename = ToString() << "tmp/" << prefix << k << ".png" ;   // use png and put things in a ./tmp/ subdirectory
	stbi_write_png(filename.c_str(), image.dim().x, image.dim().y, 3, image.raster.data(), image.dim().x * 3);
}


int main(int argc, char *argv[]) try
{
	float2 drange = { 0.20f, 0.70f }; // in m   
	auto handmodel = LoadHandModel();
	if (argc < 2)
	{
		std::cout << "Usage " << argv[0] << " filename.rs [filenames] " << std::endl;
		return 0;
	}
	int k = 0;
	std::ofstream labels_full("tmp/labels_full.txt");
	std::ofstream labels_seg ("tmp/labels_seg.txt");
	for (int i = 1;i < argc;i++)
	{
		auto prefix = fileprefix(argv[i]);
		std::cout << "loading  " << prefix << std::endl;
		//std::vector<Frame> load_dataset(std::string bname, unsigned int pose_array_size, std::function<void(Frame&)> post_process = [](Frame&) {})
		auto frames = load_dataset(prefix, 17);
		std::cout << "framecount " << frames.size() << std::endl;
		for (auto &frame : frames)
		{
			labels_full << k << "  ";
			labels_seg  << k << "  ";
			auto fpimage = Transform(frame.depth, [ &](unsigned short d) {return (float)clamp(1.0f - (d*frame.depth.cam.depth_scale - drange.x) / (drange.y - drange.x), 0.0f, 1.0f); });
			Image<byte3> full_depth = ToRGB(ToGrayScale(fpimage));
			Image<byte3> full_ir    = ToRGB(frame.ir);
			ImageWrite("full_depth_", k, full_depth);
			ImageWrite("full_ir_"   , k, full_ir   );
			int2 bmin(std::numeric_limits<int>::max()), bmax(std::numeric_limits<int>::lowest());
			handmodel.SetPose(frame.startpose);
			for (unsigned int i = 1;i < handmodel.rigidbodies.size();i++)
			{
				for (auto &v : handmodel.rigidbodies[i].shapes[0].verts)
				{
					auto p = frame.depth.cam.projectz(frame.startpose[i] * v);
					bmin = min(int2(p), bmin);
					bmax = max(int2(p), bmax);
				}
			}
			bmin = max(bmin, int2(0, 0)); 
			bmax = min(bmax, frame.depth.cam.dim() - int2(1, 1));
			labels_full << bmin << "  " << bmax << "   ";
			for (auto image : { &full_depth ,&full_ir })
			{
				for (int y : {bmin.y, bmax.y}) for (int x = bmin.x;x <= bmax.x;x++)
					image->pixel(int2(x, y)) = byte3(128, 0, 0);
				for (int x : {bmin.x, bmax.x}) for (int y = bmin.y;y <= bmax.y;y++)
					image->pixel(int2(x, y)) = byte3(128, 0, 0);
			}
			int cc = 0;
			for (auto k : handmodelfeaturepoints)
			{
				auto p = int2(full_depth.cam.projectz(frame.startpose[k.bone] * k.offset));
				p = min(full_depth.cam.dim()-int2(1,1), max(int2(0, 0), p));  // just clamp in case out of bounds
				labels_full << p << " ";
				full_ir.pixel(p) = full_depth.pixel(p) = byte3(rainbow_colors[cc++] * 255.0f);
			}
			ImageWrite("debug_depth_", k, full_depth);
			ImageWrite("debug_ir_"   , k, full_ir);
			// reduce frame to 64x64 segment
			auto segment = HandSegmentVR(frame.depth, 0xF, drange);  
			auto cnn_input = Transform(segment, [drange, &segment](unsigned short d) {return (float)clamp(1.0f - (d*segment.cam.depth_scale - drange.x) / (drange.y - drange.x), 0.0f, 1.0f); });
			auto input_rgb = Transform(cnn_input, [](float d)->byte3 { return byte3(ToGrayScale(d)); });
			ImageWrite("segment_depth_",k, input_rgb);
			if (frame.ir.raster.size())
			{
				auto ir = Sample(frame.ir, segment.cam);
				ir.cam.pose = Pose();
				ImageWrite("segment_ir_", k, ToRGB(ir));
			}
			auto pose = Transform(frame.pose, [&](Pose p) {return segment.cam.pose.inverse() * p;});
			segment.cam.pose = Pose();
			cc = 0;
			for (auto p : ImageFeaturePoints(pose, handmodelfeaturepoints, segment.cam))   // cnn_labels.image_points)
			{
				labels_seg << p << "  ";   // in the 64x64 image space
				input_rgb.plotsafe(int2(p), byte3(rainbow_colors[cc++] * 255.0f));
			}
			ImageWrite("debug_segdepth_", k, input_rgb);  // with the keypoints added
			DCamera hcam = camsub(segment.cam, 4);  // used for making 16x16 heatmaps 
			auto cnn_labels = GatherHandExpectedCNN(pose, hcam);   // results in the 16x16 heatmap image space 
			for (auto v : cnn_labels.vals)  // 16 angle values
			{
				labels_seg  << v << " ";
				labels_full << v << " ";
			}
			auto hmaps = ToRGB(ImageConcat(std::vector<Image<unsigned char>>{ ImageConcat(cnn_labels.hmaps),cnn_labels.vmap }));
			ImageWrite("heatmaps_",k, hmaps);
			//std::cout << k << " " << filed << " " << filel << std::endl;
			labels_full << std::endl;
			labels_seg  << std::endl;
			k++;
		}
	}
	std::cout << "done" << std::endl;
	return 0;
}
catch (const char *c)
{
	std::cerr << c << std::endl;
}
catch (std::exception e)
{
	std::cerr << e.what() << std::endl;
}

