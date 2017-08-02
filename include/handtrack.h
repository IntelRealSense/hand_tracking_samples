//
// handtrack.h - articulated hand skeletal model tracking system
// 
// The objective of the hand tracking is to take images of depth data as input and provides hand pose output.
// This header file encapsulate the articulated hand tracking to provide runtime hand interaction as
// well as support routines for auto-labelling ground truth and other useful items for building a hand
// pose estimation system.  Note, the camera or depth stream interface is de-coupled from the code here.   
//
// Typical implementation of a application that used this hand tracking runtime system would be:
//       main(){ ...
//          HandTracker my_hand_tracker;
//          while() // main loop
//             ...
//             my_hand_tracker.update(my_depth_data_from_some_camera);
//             ...
//             my_render_function( htk.handmodel.GetPose(), my_hand_rig); 
//       }
//
// The system here consists of a geometric based system to fit a model to depth (3D pointcloud) data and
// some CNN based classification to provide high level infomation (landmarks etc) about the input depth image.
// 
// To allow for easier cross-platform compilation and testing, without requiring developers to install
// various dependencies on their systems, simple single-header-only implementations are used for each system
// including basic vector math, geometry, simulation, 3D rig model support, and cnn machine learning.   
//
// The geometric fitting (an icp-like step) uses a rigidbody simulation system described in physics.h and physmodel.h.
// When the hand model is somewhat close to the pose where it should be, the model snaps into place pretty quick.
// Because, based on the depth data alone, its not known where individual depth pixels coorespond to on the user's hand.
// Consequently, without a good prior, the model may settle into a bad local minimum, and thus return an incorrect pose. 
// Therefore, a little bit of occasional higher level classification can prevent this from happening.
//
// The cnn.h implementation shows an typical CNN architecture that, when trained, can (ideally) provide key 
// landmark information that can improve the geometric based tracking system to then fit the model correctly to
// the depth data and consequently reconstruct the hand pose of the user in the original depth image.
// Its easy to replace this CNN layer with your own preferred ML system.   Alternatively it possible to 
// load the input and label data into another package, experiment and train architectures there, and import
// the weights back into this system. 
//
// The current CNN architecture used below is simply
//    - 1  64x64 single channel (depth data) input 
//    - 8  16x16 heatmap outputs, 8 features on the model  
//    - 16 16x1  heatmap outputs, collection of palm wrist and finger angles
//    - Two conv+tanh+pool and then two fully connected layers, similar to classic digits but wider
//
// On a high-end desktop computer the CNN forward prop and many steps of the model simulation can run at interactive rates.
// However lower-end computers would be less responsive.   Fortunately, we can expect some degree of temporal coherence and
// the the CNN output isn't really needed every frame.  That is why the system here creates two hand models so that it
// can do more costly processing in a background thread while continuing to do fast incremental updates in the main thread.   
//
// It may appear there are lot of tunable parameters, this is because this code had been used for a lot of experimentation.
// After more ground truth data is collected and cnn architectures and error rates improve, the reliance on clever systems
// decreases.  At such time any elaborate complicated code here will no longer be necessary.
// 

#pragma once
#ifndef   HANDTRACK_SYSTEM_H
#define   HANDTRACK_SYSTEM_H

#include <iostream>
#include <fstream>
#include <future>   // for HandTracker runtime to put cnn eval in separate thread

#include "../third_party/misc_json.h"
#include "../third_party/geometric.h"
#include "../third_party/cnn.h"
#include "misc_image.h"
#include "physmodel.h"

// the directory where ground truth captured hand mocap goes
#define DEFAULT_HANDPOSE_FILE_PREFIX   "../datasets/hand_data_"

static const int key_angles_count = 16;  // have up to 16 1D heatmaps of length 16   fits a 16x16 image too

static const float3 rainbow_colors[] = { { 0.75f,0.5f,0.5f },{ 0.5f,0.75f,0.5f },{ 0.5f,0.5f,0.75f },{ 1,0,0 },{ 0,1,0 },{ 0,0,1 },{ 1,1,0 },{ 1,0,1 } };

struct ModelFeaturePoint { int bone; float3 offset; };  // local bone space points on the 3D model
static const std::vector<ModelFeaturePoint> handmodelfeaturepoints =
{  // this is based on 3D right hand model  
	{ 1,{ 0, 0.0f, 0.0f } },{ 1,{ -0.03f, 0, -0.03f } },{ 1,{ 0.03f, 0, -0.03f } },    //  three palm points
	{ 4,{ 0, 0, 0 } },{ 7,{ 0, 0, 0 } },{ 10,{ 0, 0, 0 } },{ 13,{ 0, 0, 0 } },{ 16,{ 0, 0, 0 } },   // finger tips order thumb,index,middle,ring,pinky
};
inline float3 Skin(const Pose *pose, const ModelFeaturePoint &p) { return pose[p.bone] * p.offset; }
inline std::vector<float3> Skin(const Pose *pose, const std::vector<ModelFeaturePoint> &pts) { return Transform(pts, [&](const ModelFeaturePoint &p) {return Skin(pose, p); }); }
inline std::vector<float3> Skin(const std::vector<Pose> &pose, const std::vector<ModelFeaturePoint> &pts) { return Skin(pose.data(), pts); }


template<class T> inline std::vector<const std::vector<T>*> AddressesRasterData(const std::vector<Image<T>> &images)
{
	return Transform(images, [&](const Image<T> &image)->const std::vector<T>* { return &image.raster; });
}

inline std::vector<float2> ImageFeaturePoints(const std::vector<Pose> &pose, const std::vector<ModelFeaturePoint> &modelfeaturepoints, const DCamera &hcam)
{
	// converts array of 3d model points to 2d image locations based on projection and pose of camera 'hcam'
	return hcam.projectz(Transform(Skin(pose, modelfeaturepoints), [&hcam](float3 v) {return hcam.pose.inverse()*v; }));
}

inline std::vector<Image<unsigned char>> RenderHeatMaps(const std::vector<Pose> &pose, const std::vector<ModelFeaturePoint> &modelfeaturepoints, const DCamera &hcam)
{
	return RenderHeatMaps(ImageFeaturePoints(pose,modelfeaturepoints,hcam), hcam);   // returns 8 16x16 8bit monochrome heatmaps 
}

inline CNN PoseInitializerCNN(std::string filename)
{
	const int debug = 0;
	if (debug)std::cout << "cnn build\n";
	CNN cnn({});
	cnn.layers.push_back(new CNN::LConv({ 64,64,1 }, { 5,5,1,16 }, { 60,60,16 }));   // 5x5 conv layer 1 channel in and 16 channels out
	cnn.layers.push_back(new CNN::LActivation<TanH>(60 * 60 * 16));
	cnn.layers.push_back(new CNN::LMaxPool({ 60,60,16 }));
	cnn.layers.push_back(new CNN::LMaxPool({ 30,30,16 }));
	cnn.layers.push_back(new CNN::LConv({ 15,15,16 }, { 4,4,16,64 }, { 12,12,64 }));  // 4x4 convolutions 16 in 64 out channels
	cnn.layers.push_back(new CNN::LActivation<TanH>(12 * 12 * 64));
	cnn.layers.push_back(new CNN::LMaxPool({ 12,12,64 }));
	cnn.layers.push_back(new CNN::LFull(6 * 6 * 64, 16 * 16 * 8));
	cnn.layers.push_back(new CNN::LActivation<TanH>(16 * 16 * 8));
	cnn.layers.push_back(new CNN::LFull(16 * 16 * 8, 16 * 16 * 8 + 16 * key_angles_count));  // output node for each heatmap pixel 
	cnn.layers.push_back(new CNN::LSoftMaxChunked(concat(std::vector<int>(8, 16 * 16), std::vector<int>(key_angles_count, 16))));
	cnn.Init();
	{
		if (debug)std::cout << "cnn loadb \n";
		std::ifstream is(filename, std::ios_base::in | std::ios_base::binary);
		if (is.is_open())
			cnn.loadb(is);  // binary 32bit float weights and bias values from trained cnn
		else
			if (debug)std::cout << "unable to init cnn from file\n";
		if (debug)std::cout << "... done.\n";
	}
	return cnn;
}


auto HandPoseToKeyAngleSet(const std::vector<Pose> &pose, const Pose &reference_frame)   
{
	// Provide an over-simplified minimal set of key angles to roughly describe a hand pose 
	// based on the provided standard quat-vec array of bone transforms. 
	std::vector<float> values;  // set of key values mapped to mostly local (relative) angular transforms 
	auto palmq = qmul(reference_frame.inverse().orientation, pose[1].orientation);                // palm orientation  
	values.push_back(atan2(qxdir(palmq).x, -qxdir(palmq).z) / (3.14159f*2.0f)+0.5f);              // wrist roll 
	values.push_back(asin(clamp(qzdir(palmq).z, -1.0f, 1.0f)) / 3.14159f + 0.5f);                 // pitch away or toward camera
	values.push_back(asin(clamp(qzdir(palmq).x, -1.0f, 1.0f)) / 3.14159f + 0.5f);                 // tilt 
	values.push_back(acos(dot(qxdir(pose[1].orientation),qzdir(pose[4].orientation)))/3.14159f) ; // thumb tip angle
	for (auto bid: { 6,9,12,15 }) // index mid ring pinky   middle bone of each of these
		values.push_back( acos(clamp(dot(qydir(pose[1].orientation),qydir(pose[bid].orientation)), -1.0f, 1.0f)) / 3.14159f);   // next 4 value entries    acos is 0..pi  val is 0..1
	auto pz = qzdir(palmq);
	values.push_back(0.5f + atan2(-pz.x,-pz.y) / (3.14159f*2.0f));  // 0.5 should be palm/arm pointing +y
	for (auto i = values.size(); i < key_angles_count; i++)
		values.push_back(0.0f);  // unused 
	return values;  // hand pose roughly described using 9 relative-angle numbers in [0..1] range  
}

//
// GatherHandExpectedCNN 
// The expected output, or label, we want the CNN to produce is based on the ground-truth or actual skeletal pose of the hand that  
// the user had when the input image was taken.   The 2nd parameter, hcam, specifies the dimensions of the 2D heatmaps and any 
// other information such as focal length, depth scale, and position/orientation of the depth camera that took the input image. 
// In addition to the vector of expected values in the output nodes, this function also provides other immediate values and images
// in the returned Set which can be useful for inspection and visualization purposes.
//
inline auto GatherHandExpectedCNN(const std::vector<Pose> &pose, const DCamera &hcam)  // the labels we train our cnn to try and output
{
	auto featurepoints = ImageFeaturePoints(pose, handmodelfeaturepoints, hcam);
	auto hmaps = RenderHeatMaps(featurepoints, hcam);        // 8 2D 16x16 heatmaps corresponding to the locations of 8 key feature points
	DCamera htcam = hcam;
	htcam.dim().y *= (int)handmodelfeaturepoints.size();
	auto vals  = HandPoseToKeyAngleSet(pose, hcam.pose); // mostly angles and such as seen by resample camera
	auto vmap  = Render1DHeatMaps(vals,16);
	auto addrs = AddressesRasterData(hmaps);
	addrs.push_back(&vmap.raster);
	auto cnn_expected = Transform(concat(addrs), GrayScaleToFloat);  // this is the array of numbers the cnn output nodes are trained to produce
	struct Set { std::vector<float> cnn_expected; std::vector<float2> image_points;  std::vector<Image<unsigned char>> hmaps; Image<unsigned char> vmap; std::vector<float> vals; };
	return Set { cnn_expected ,featurepoints, hmaps, vmap ,vals};
}


//
// CNNOutputAnalysis
// The CNN outputs an array of floating point numbers between 0 and 1.   The CNNOutputAnalysis struct decodes
// this output into the information that the CNN was trained to provide such as key landmark positions and angles.
// Additionally, intermediate values and images are provided for inspection and visualization purposes. 
//
struct CNNOutputAnalysis 
{ 
	std::vector<float4> crays;                 // 3D directions of the heatmap peaks   w member stores a certainty value
	std::vector<float2> image_points;          // peak locations on the heatmaps
	std::vector<Image<unsigned char>> hmaps;   // the 2D feature heatmaps 
	std::vector<float> confidence;             // a certainty value for each of the heatmap peaks
	Image<float> vmap;                         // the 1D heatmaps as rows in a 2D image
	std::vector<float> vals;                   // from the 1D heatmaps 
	float wristroll, pitch, tilt;              // angle landmarks 
	float4 palmq;                              // orientation of the palm bone 
	std::vector<float> finger_clenched;        // five finger states,  0 is open, 3.14 (180 degrees) is clenched.

	void calc_angles() // input is the 8 normalized values in [0..1] range, reconstructs the angles.   this mirrors the GatherKeyValues() routine that gathers the key angles from a given hand pose.
	{
		wristroll = vals[0] * 3.1415f*2.0f + 3.1415f / 2.0f;
		pitch     = (vals[1] - 0.5f) * 3.1415f;
		tilt      = (vals[2] - 0.5f) * 3.1415f;
		palmq     = qmul(normalize(float4{ 1.0f, 0, 0, 1.0f }), qmul(rotation_quat(float3{ -1,0,0 }, pitch), rotation_quat(float3{ 0,0,1 }, wristroll)));  // doesn't include the camera's pose
		for (int i = 0; i < 5; i++)
			finger_clenched.push_back(vals[3 + i] * 3.1415f);
	}
	auto ApplyAngles(PhysModel &handmodel, const Pose &camera_pose, float drive_force = 10000.0f, float coneangle = 10.0f)  // only works with the rigged handmodel 
	{
		std::vector<LimitAngular> angulars;
		Append(angulars, ConstrainAngularDrive(NULL, &handmodel.rigidbodies[1], qmul(camera_pose.orientation, palmq), drive_force));
		float thumbangle = finger_clenched[0];
		angulars.push_back(ConstrainConeAngle(&handmodel.rigidbodies[1], { cos(thumbangle),0,sin(thumbangle) }, &handmodel.rigidbodies[4], { 0,0,1 }, coneangle));  // thumb tip
		for (int finger : {1, 2, 3, 4})
		{
			float a = finger_clenched[finger];
			angulars.push_back(ConstrainConeAngle(&handmodel.rigidbodies[1], { 0,-sin(a),cos(a) }, &handmodel.rigidbodies[3 + finger * 3], { 0,0,1 }, coneangle));
			angulars.push_back(ConstrainConeAngle(&handmodel.rigidbodies[1], qrot(handmodel.joints[1 + finger * 3].jointframe, qrot(handmodel.joints[1 + finger * 3].jointframe, { 0,-sin(a / 2.0f),cos(a / 2.0f) })), &handmodel.rigidbodies[2 + finger * 3], { 0,0,1 }, coneangle));
		}
		return angulars;
	}
	CNNOutputAnalysis() {}
	CNNOutputAnalysis(const std::vector<float> &cnn_output, const DCamera &hcam)
	{
		int2 hdim = hcam.dim();  assert(hdim == int2(16, 16));
		for (unsigned int i = 0; i < handmodelfeaturepoints.size(); i++)   //  cnn_output.size() / product(hdim); i++)
		{
			auto base = cnn_output.data() + product(hdim)*i;
			Image<float> fmap = Image<float>(hcam, std::vector<float>(base, base + product(hcam.dim())));
			hmaps.push_back(Transform(fmap, [](float y) {return ToGrayScale(y); }));
			int2 mx = ImageFindMax(fmap);
			float nextbest = 0.0f;  // for 2nd brightest peak's value
			for (auto p : rect_iteration(hdim)) if (abs(p.x - mx.x)>2 || abs(p.y - mx.y) > 2)  // non neighbors nor within 2
				nextbest = std::max(nextbest, fmap.pixel(p));
			float2 p=PeakSubPixel(fmap, mx);  // landmark feature point
			image_points.push_back(p);
			confidence.push_back(PeakVolume(fmap, p));
			auto n = normalize(hcam.pose*hcam.deprojectz(p, 1.0f));
			crays.push_back({ n, base[hdim.x*mx.y + mx.x]});
		}
		const float *vptr = cnn_output.data() + product(hdim)*handmodelfeaturepoints.size();
		int2 vdim(16, key_angles_count);
		vmap = Image<float>(vdim, { vptr, vptr + product(vdim) }); 
		vals = Peaks1D(vmap);
		calc_angles();
	}
};


template <class T> auto summation(const std::vector<T> &a, T s = T(0.0f)) { for (auto &x : a) s += x; return s; }
template <class T> auto average(const std::vector<T> &a) { return a.size()? summation(a)*(1.0f/a.size()): T(0.0f); }


template<class T> Image<byte3> ImageOverlayYellowBlue(const Image<unsigned char> &image_rg, const Image<T> &image_b)
{
	std::vector<byte3> rgbdata(image_rg.raster.size());
	std::transform(image_rg.raster.begin(), image_rg.raster.end(), image_b.raster.begin(), rgbdata.begin(), [](unsigned char a, T b) {return byte3(a, a, ToGrayScale(b)); });
	return Image<byte3>(image_rg.cam, rgbdata);
}

template<class T> Image<byte3> VisualizeHMaps(const std::vector<Image<unsigned char>> &hmaps, Image<T> &dmap_d)  // mostly for debugging
{
	std::vector<Image<byte3>> vmaps;
	for (auto hmap : hmaps) // by value copy, not reference
	{
		while (product(hmap.dim()) < product(dmap_d.dim()))
			hmap = UpSample(hmap);
		vmaps.push_back(UpSample(ImageOverlayYellowBlue(hmap, dmap_d)));
	}
	return ImageConcat(vmaps);
}


// HandSegmentVR() 
// Samples a 64x64 possibly-rotated subregion of the input depth image intended to be aligned and focused on the hand.
// The implementation is a bit ad hoc.  was good enough for certain controlled limited usages.
// This oversimplified hard-coded hand segmentation assumes it is seeing the right hand (and only the right hand) 
// and the wrist is entering the scene from an edge (VR use case) with an obvious entry point.
// This segmentation also rotates within the plane to align the hand with the vertical axis and scales to keep it about the same image size.
// The entry point and the centroid of the blob that determine the angle within the plane,
// and scale amount depends on the average depth
// This was done to minimize the input possibilites and reduce the machine learning burden so hopefully a CNN with fewer parameters could be used. 
// Ideally this entire function should be replaced with a better (learned) system.
//
inline Image<unsigned short> HandSegmentVR(const Image<unsigned short> &depth, int entry_options = 0xF, float2 wrange = { 0.1f,0.65f }, float diam = 0.17f)
{
	auto dim = depth.dim();
	Image<unsigned short> depthsmall = DownSampleMin(DownSampleMin(depth));
	ushort2 wranged = ushort2(wrange / depth.cam.depth_scale);
	auto dt = DistanceTransform(Threshold(depthsmall, [wranged](unsigned short d) {return /*d >= wranged.x && */d < wranged.y; }));

	int2 entry = (entry_options == 1) ? int2(dt.dim().x / 2, dt.dim().y - 1) : (entry_options == 4) ? int2(dt.dim().x - 1, dt.dim().y / 2) : (entry_options == 8) ? int2(0, dt.dim().y / 2) : int2(0, 0);
	if (entry_options & 1) for (int2 p(0, dt.dim().y - 1); p.x < dt.dim().x; p.x++) if (dt.pixel(p) > dt.pixel(entry))  entry = p;
	if (entry_options & 2) for (int2 p(0, 0); p.x < dt.dim().x; p.x++) if (dt.pixel(p) > dt.pixel(entry))  entry = p;
	if (entry_options & 4) for (int2 p(dt.dim().x - 1, 0); p.y < dt.dim().y; p.y++) if (dt.pixel(p) > dt.pixel(entry))  entry = p;
	if (entry_options & 8) for (int2 p(0, 0); p.y < dt.dim().y; p.y++) if (dt.pixel(p) > dt.pixel(entry))  entry = p;

	float  avgdepth = 0;
	float2 com(0, 0);
	float  wtotal = 0.0f;
	int    count = 0;
	const int min_blob_radius = 2;  // note that this will be after downsampled for distance transform    had a camera that was having edge noise issues

	for (auto p : rect_iteration(dt.dim()))
	{
		if (dt.pixel(p) >= min_blob_radius) // && p.y < firsty + 128 / 4)
		{
			float w = length(float2(p - entry)) + 0.00001f;
			wtotal += w;
			com += asfloat2(p)*w;
			avgdepth += depthsmall.pixel(p)*w;   // although 32 bit float, avgdepth sum happens in unsigned short raster units at its depth_scale
			count++;
		}
	}
	if (count && wtotal>0.0f)
	{
		avgdepth *= depth.cam.depth_scale / wtotal;  // avgdepth convert to meters and divide by total weighting
		com /= wtotal;
	}
	float2 extreme = float2(entry);
	for (auto p : rect_iteration(dt.dim()))
	{
		if (dt.pixel(p) >= min_blob_radius) // && p.y < firsty + 128 / 4)
			if (dot(float2(p) - float2(entry), com - float2(entry)) > dot(float2(extreme) - float2(entry), com - float2(entry)))
				extreme = float2(p);
	}
	//float scale = 2.0f;
	float angle = 0.0f;
	// const float diam = 0.17f; //  diameter_of_interest 
	avgdepth = clamp(avgdepth, 0.20f, 1.0f);
	if (count&& wtotal>0.0f&& com != float2(entry))
	{
		angle = atan2((float)com.x - entry.x, (float)entry.y - com.y);
		//scale = 2.0f / clamp(avgdepth / 0.45f, 0.55f, 2.0f);     // magic number alert, the 45cm comes from using it previously as a target distance before i had adaptive scale
		float exrad = dot(extreme - com, normalize(com - float2(entry)));
		com += normalize(com - float2(entry)) * (exrad - diam / 2.0f / avgdepth * dt.cam.focal().x);   // shift com to be half of diam ~20cm (ie 10cm) away from extreme point 
	}

	DCamera dstcam({ 64,64 }, float2(avgdepth*64.0f / diam), { 32,32 }, depth.cam.depth_scale);

	float3 zd = normalize(dstcam.deprojectz(asfloat2(dstcam.dim() / 2), 1.0f));
	float3 yd = normalize(ProjectOntoPlane({ zd,0.0f }, dt.cam.deprojectz(float2(entry), 1.0f) - dstcam.deprojectz(asfloat2(dstcam.dim() / 2), 1.0f)));
	dstcam.pose.orientation = qmul(quat_from_to(dt.cam.deprojectz(dt.cam.principal(), 1.0f), dt.cam.deprojectz(com, 1.0f)), QuatFromAxisAngle({ 0,0,1 }, angle)); //quatfrommat({ cross(yd, zd), yd, zd });
	dstcam.principal() = asfloat2(dstcam.dim())*0.5f;

	return SampleD(depth, dstcam, (unsigned short)(4.0f / depth.cam.depth_scale));  // 4m away default bg for any samples outside of src depth image
}


PhysModel LoadHandModel()
{
	PhysModel handmodel("../assets/model_hand.json");
	for (unsigned int i = 2; i < handmodel.rigidbodies.size(); i++)  // this hand model init hack allows for more finger interpenetration.
		for (auto &v : handmodel.rigidbodies[i].shapes[0].verts)     // pulling in the vertices used for the gjk shape-to-shape collision
			v = v * float3(0.7f, 0.7f, 0.9f);                        // scale back x and y only  or perhaps a bit of z

	for (int i : {7, 10, 13, 16})  // because base of thumb was pushing out fingers too much
	{
		handmodel.rigidbodies[i].ignore.push_back(&handmodel.rigidbodies[2]);
		handmodel.rigidbodies[2].ignore.push_back(&handmodel.rigidbodies[i]);
	}
	for (auto &m : handmodel.sdmeshes)
		m.material = "rimlit";
	int dmode = 1;
	for (unsigned int i = 2; i < handmodel.sdmeshes.size(); i++)
		handmodel.sdmeshes[i].hack.xyz() = rainbow_colors[3 + (i - 2) / 3];

	return handmodel;
}


float  bone_sum_error_scale = 4.0f;

float FitError(PhysModel &handmodel, const std::vector<float3> &pts, const Image<unsigned short> &dimage)  
{
	// Sum of quick approximations of hausdorff distances.   
	// Useful, but not entirely reliable, method of determining how good is the model fitting to the point cloud.  
	auto rigidbodies = Addresses(handmodel.rigidbodies);
	std::vector<float> pointerror(handmodel.rigidbodies.size(), 0.0f);
	for (auto &v : pts)
	{
		RigidBody *rb; float4 p;
		std::tie(rb, p) = closest(rigidbodies, v);
		int boneid = IndexOf(rigidbodies, rb);
		pointerror[boneid] = std::max(pointerror[boneid], dot(p, float4(v, 1.0f)));
	}
	float point_error_sum = 0.0f;
	for (auto &e : pointerror)
		point_error_sum += e;

	float bone_error_sum = 0;
	for (auto &rb : handmodel.rigidbodies)
	{
		auto position = dimage.cam.pose.inverse()*rb.position;   // local position of bone origin in depth camera space
		auto p = int2(dimage.cam.projectz(position));  // image pixel 2d
		if (!within_range(p, { 0,0 }, dimage.dim() - int2{ 1, 1 }))
			continue;
		float bone_error = dimage.pixel(p)*dimage.cam.depth_scale - position.z;
		bone_error_sum += clamp(bone_error, 0.0f, 0.01f);
	}
	return point_error_sum + bone_error_sum * bone_sum_error_scale;   // i had a 4 multiplier in the distant past - better to use distance transform approx
}


// HandModelEnhancements
// Typical rigidbody constraints are pairwise between two bones.  This does not adequately capture all range limits of the hand.
// Here we provide a few additional constraints to prevent the solver from putting the hand model in unfeasible states.
// For example, the upper two joints angles on each finger will typically match each other.
inline void HandModelEnhancements(PhysModel &handmodel, std::vector<LimitAngular>  &angulars, bool tiepinkyringmid, float3 palmxdir, float3 armdir, int fingerhold = 0)  // very specific to the hand model, additional constraints based on realistic motion/usage
{
	if (handmodel.rigidbodies[2].ignore.size() < 10)
	{
		handmodel.rigidbodies[2].ignore.clear();   
		for (auto &rb : handmodel.rigidbodies) if (&rb != &handmodel.rigidbodies[2])
		{
			handmodel.rigidbodies[2].ignore.push_back(&rb);
			rb.ignore.push_back(&handmodel.rigidbodies[2]);    // make sure base thumb bone donsnt collide with anything
		}
	}
	for (int b = 7; b < 17; b += 3)  // b is index of fingertip bone, the joint at index b-1 will be the joint attaching that to its parent   obviously this is for hand model only
	{
		handmodel.joints[b - 1].rangemin.x = handmodel.joints[b - 1].rangemax.x = acos(clamp(dot(qzdir(handmodel.rigidbodies[b - 2].orientation), qzdir(handmodel.rigidbodies[b - 1].orientation))))*180.0f / 3.14159f / 2.0f;  // half the angle of the upper knuckle
	}
	if (tiepinkyringmid) // often these fingers move together, so it may be appropriate to constrain them so they behave as such
		for (int b : { 15, 14, 12, 11 })   // base and mid bone for both pinky and mid, sub 3 to get neighbor
			angulars.push_back(ConstrainConeAngle(&handmodel.rigidbodies[b], { 0,1,0 }, &handmodel.rigidbodies[b-3], { 0,1,0 }, 10.0f));

	if (armdir != float3(0, 0, 0))  // for example, if we know a camera is upright, the arm direction might be known to fall within a certain range
		angulars.push_back(ConstrainConeAngle(NULL, armdir, &handmodel.rigidbodies[0], { 0,0,1 }, 70.0f));  // wrist upward is local +z  uses supplied armdir which could be 0,-1,0 if a y-down-camera

	if (fingerhold&(1 << 0))
		angulars.push_back(ConstrainConeAngle(&handmodel.rigidbodies[1], { -1,0,0 }, &handmodel.rigidbodies[4], { 0,0,1 }, 10.0f));  // thumb tip bone
	for (int finger : {1, 2, 3, 4})
		if (fingerhold & (1 << finger))
			angulars.push_back(ConstrainConeAngle(&handmodel.rigidbodies[1], { 0,0,-1 }, &handmodel.rigidbodies[ 3 + finger * 3], { 0,0,1 }, 10.0f));  // finger mid bone

	struct { int bone; float2 range; } knucklelimits[] = { { 14,{ -30.0f,10.0f } } ,{ 11,{ -10.0f,10.0f } },{ 8,{ -10.0f,10.0f } },{ 5,{ -10.0f,20.0f } } };  // from default_hand.chr
	for (auto &k : knucklelimits)  // finger abduction only when fingers aren't clenched   range to 0 when not extended and default otherwise  using 45 degree threshold
	{
		bool up = dot(qydir(handmodel.rigidbodies[1].orientation), qydir(handmodel.rigidbodies[k.bone].orientation)) > cos(40.0f * 3.14f / 180.0f);
		handmodel.joints[k.bone - 1].rangemin.y = (up) ? k.range[0] :  -0.0f;  // k.range[0]/2.0f; // 
		handmodel.joints[k.bone - 1].rangemax.y = (up) ? k.range[1] :   0.0f;  // k.range[1]/2.0f; // 
	}
}

//
// UnibodyFit
// Git all parts as if it was a single rigid object maintaining the relative joint angles from the prior handmodel pose. 
// The initial pose may not even match the input data so there can be a high probability of having more bad correspondences than good ones.
// Just using a soft iterative-impulse-solver fit that gives a lot of inertia to starting state.
// The result is typically an icp-like fit when there is a good match with the point cloud, and a modest following of the point cloud otherwise.
//
float  unibody_force = 0.1f;
void  UnibodyFit(PhysModel &handmodel, const std::vector<float3> &pts, float3 camera_position)
{
	auto constraints = CloudConstraints(Addresses(handmodel.rigidbodies), takesubsample(pts), camera_position);
	WingMesh box = WingMeshCube(0.1f);
	RigidBody unibody({ Shape(box.verts,box.GenerateTris()) }, handmodel.rigidbodies[1].position);
	unibody.orientation = handmodel.rigidbodies[1].orientation;
	for (auto &c : constraints)
	{
		c.position1 = unibody.pose().inverse() * (c.rb1->pose() * c.position1);
		c.rb1 = &unibody;
		c.forcelimit *= unibody_force;
	}
	handmodel.SanityCheck();
	auto angular = std::vector<LimitAngular>{};
	PhysicsUpdate(std::vector<RigidBody*>{ &unibody }, constraints, angular, std::vector<std::vector<float3> *>{});
	auto dp = unibody.pose() * handmodel.rigidbodies[1].pose().inverse();
	for (auto &rb : handmodel.rigidbodies)
		rb.pose() = dp*rb.pose();
	handmodel.SanityCheck();
}


// PoseFromScratch
// Directly assign a skeletal pose to the model based on the CNN output and points.
// This routine does not do any icp or other form of model fitting (eg rigidbody simulation) 
// The output of this routine is currently a ways off, and requires a some point cloud fitting after initial positioning. 
// Having well trained cnn's providing good initial condiditons and the ability to more directly produce 
// a pose from that (i.e. doing a better job than what you see here) will vastly reduce the complexity of the entire system.  
//
void PoseFromScratch(PhysModel &handmodel, const std::vector<float3> &pts, CNNOutputAnalysis &cnn_output_analysis, const Pose &camera_pose)
{
	auto &crays = cnn_output_analysis.crays;
	float3 palmray = normalize((crays[0] + crays[1] + crays[2]).xyz());  // based on first 3 heatmap locations  since i dont currently have a single one for palm center, crays were rotated back into larger scene
	float3 pcom(0.0f); float wsum = 0.00000000001f;
	for (auto p : pts)
	{
		float w = 1.0f / (0.000001f + length2(cross(p, palmray)));
		pcom += p*w;wsum += w;
	}
	pcom /= wsum;
	float3 p1p = pcom; // other optoin is to keep position:  handmodel.rigidbodies[1].position;
	for (auto &rb : handmodel.rigidbodies)
		Reset(rb);  // do the reset
	Pose p1(p1p, qmul(camera_pose.orientation, cnn_output_analysis.palmq));   //  normalize(float4(0.7f, 0, 0, 0.7f))  ;
	auto dp = p1 * handmodel.rigidbodies[1].pose().inverse();
	for (auto &rb : handmodel.rigidbodies)
		rb.pose() = dp*rb.pose();
	for (int finger : {1, 2, 3, 4})  // index, middle, ring, pinky
	{
		float a = cnn_output_analysis.finger_clenched[finger];
		handmodel.rigidbodies[2 + finger * 3].orientation = qmul(handmodel.joints[1 + finger * 3].jointframe, qmul(handmodel.rigidbodies[2 + finger * 3].orientation, QuatFromAxisAngle({ 1,0,0 }, a / 2.0f)));
		handmodel.rigidbodies[3 + finger * 3].orientation = qmul(handmodel.joints[1 + finger * 3].jointframe, qmul(handmodel.rigidbodies[3 + finger * 3].orientation, QuatFromAxisAngle({ 1,0,0 }, a)));
		handmodel.rigidbodies[4 + finger * 3].orientation = qmul(handmodel.joints[1 + finger * 3].jointframe, qmul(handmodel.rigidbodies[4 + finger * 3].orientation, QuatFromAxisAngle({ 1,0,0 }, a*1.25f)));
	}
	handmodel.FixPositions();
}


//
// HandTracker  
// This is the main entry point to this module
//
struct HandTracker
{
	struct tracking_with_cnn_results   // only because if running in background thread we dont want to clobber these members in the main thread
	{
		Image<float>       cnn_input;
		std::vector<float> cnn_output;
		CNNOutputAnalysis  cnn_output_analysis;
		std::vector<Pose>   pose;
	};

	float  segment_scale       = 0.17f;
	float  full_reset_on_error = 0.6f;
	bool   angles_only         = 0;
	bool   always_take_cnn     = false;
	float  drangey             = 0.7f;    // ignore anything beyond this distance  default 70cm
	int    showdepthmesh       = 1;
	int    boundary_planes     = 1;
	float  microforce          = 1.0f;
	float  cloudforce_max_point= 15.0f;   
	float  cloudforce_max_sum  = 3000.0f; 
	int    mainthreadpasses    = 1;
	int	   subsample_fraction  = 4;
	int    subsample_voxel     = 0;
	float  subsample_size      = 0.0f;
	size_t min_point_num       = 400;
	float  accum_error_threshold = 0.0f;
	float  min_cray_prob       = 0.0f;
	int    steps               = 5;
	int    steps_keypoints     = 3;
	int    steps_keyangles     = 2;
	int    steps_palmangle     = 2;
	int    steps_cloudstart    = 1;
	int    steps_unibody       = 3;
	float  prev_frame_error    = 0.0f;
	int    initializing        = 0;

	template<class F> void visit_fields(F f)
	{
		f("segment_scale"           , segment_scale           );
		f("full_reset_on_error"     , full_reset_on_error     );
		f("angles_only"             , angles_only             );
		f("always_take_cnn"         , always_take_cnn         , int2(0, 1));
		f("drangey"                 , drangey                 , float2(0.3f, 1.25f));
		f("showdepthmesh"           , showdepthmesh           , int2(0, 1));
		f("boundary_planes"         , boundary_planes         , int2(0, 1));
		f("microforce"              , microforce              );
		f("mainthreadpasses"        , mainthreadpasses        );
		f("subsample_fraction"      , subsample_fraction      );
		f("subsample_voxel"         , subsample_voxel         );
		f("subsample_size"          , subsample_size          );
		f("min_point_num"           , min_point_num           );
		f("accum_error_threshold"   , accum_error_threshold   );
		f("cloudforce_max_point"    , cloudforce_max_point    );
		f("cloudforce_max_sum"      , cloudforce_max_sum      );
		f("steps"                   , steps                   );
		f("steps_keypoints"         , steps_keypoints         );
		f("steps_keyangles"         , steps_keyangles         );
		f("steps_palmangle"         , steps_palmangle         );
		f("steps_cloudstart"        , steps_cloudstart        );
		f("prev_frame_error"        , prev_frame_error        );
		f("physics_iterations"      , physics_iterations      ); 
		f("physics_iterations_post" , physics_iterations_post ); 
		f("physics_use_collision"   , physics_use_collision   ); 
		f("physics_weak_force"      , physics_weak_force      ); 
		f("steps_unibody"           , steps_unibody           ); 
		f("bone_sum_error_scale"    , bone_sum_error_scale    ); 
		f("min_cray_prob"           , min_cray_prob           ); 
		f("unibody_force"           , unibody_force           ); 
	}

	CNN cnn;
	Image<float>       cnn_input;          
	std::vector<float> cnn_output;
	CNNOutputAnalysis  cnn_output_analysis;
	std::future< tracking_with_cnn_results > pose_estimator;
	PhysModel handmodel;
	PhysModel othermodel;

	float scale(float s) { handmodel.scale(s);othermodel.scale(s); segment_scale *= s; return segment_scale; }
	std::vector<Mesh> vanity_bones;
	const std::vector<Mesh> &get_vanity_bones(Pose pose = Pose())
	{
		for (unsigned int i = 0;i < std::min(vanity_bones.size(), handmodel.rigidbodies.size());i++)
			vanity_bones[i].pose = pose * handmodel.rigidbodies[i].pose();
		return vanity_bones;
	}
	std::vector<Mesh> load_bone_meshes(std::string filename)
	{
		// specifically for loading some hour-glass bone-looking display geometry for visualization 
		std::vector<WingMesh> controlcages;
		from_json(controlcages, json::parsefile(filename));
		auto meshes = Transform(Transform(controlcages, [](WingMesh wm)
		{
			return WingMeshSubDiv(WingMeshSubDiv(wm));
		}), [](const WingMesh &wm)
		{
			return MeshSmoothish(wm.verts, wm.GenerateTris(), true);
		});
		for (auto &m : meshes)
			m.material = "rimlit";
		for (unsigned int i = 2;i < meshes.size();i++)
			meshes[i].hack.xyz() = rainbow_colors[3 + (i - 2) / 3];
		return meshes;
	}

	Image<byte3> get_last_segment()  // generates visual image showing cnn landmarks overlayed on segmented hand
	{
		if (!cnn_output.size())
			return Image<byte3>(int2{ 64,64 }, byte3(128));
		auto last_segment = Transform(cnn_input, [](float d)->byte3 { return byte3(ToGrayScale(d)); });
		for (unsigned int i = 0; i < handmodelfeaturepoints.size(); i++)
			last_segment.pixel_clamp(int2(cnn_output_analysis.image_points[i] * 4.0f)) = byte3(rainbow_colors[i] * 255.0f);   // scale since we going back up to segment image res from heatmap res
		return last_segment;
	}
	Image<byte3> get_cnn_difference()  // last_segment but adds lines to show cnn landmarks compared to handmodel current position 
	{
		if (!cnn_output.size())
			return Image<byte3>(int2{ 128,128 }, byte3(128));
		auto image = UpSample(get_last_segment());
		for (unsigned int i = 0; i < handmodelfeaturepoints.size(); i++)
		{
			int2 p0 = int2(image.cam.projectz(image.cam.pose.inverse()*Skin(handmodel.GetPose().data(), handmodelfeaturepoints[i]))) ;
			int2 p1 = int2(cnn_output_analysis.image_points[i] * 4.0f*2.0f);   // additional 2.0 scale since we upsampled the image
			for (int t = 0;t < 32;t++)
				image.plotsafe(p0 + (p1 - p0)*t / 31,byte3(rainbow_colors[i] * 255.0f));
		}
		return image;  // visualization rainbow lines show diff between cnn output and current handmodel pose landmarks 
	}

	void MultiStepSim( PhysModel &simmodel, CNNOutputAnalysis &cnn_output_analysis, const std::vector<float3> &vpts, const Pose &camera_pose)
	{
		// this routine does a few iterations of simulation to fit the hand model to the point cloud.
		// Apologies for the complexity of this code,  we were experimenting with applying different types of cnn outputs.
		// In addition to the usual point-plane cloud constraints using known positions in space and unknown positions on the model, 
		// we can also add landmark constraints or known points on the model to rays in 3D space based on cnn heatmap output,
		// as well as apply joint angle constraints based on cnn's 1D heatmap output for angles.
		// When to use what, and how much of each is a bit ad hoc and may be improved with further experimentation.
		// Alternatively, it might make more sense to put the research energy into creating CNNs with more directly useful 
		// outputs and reduced error rate.
		// Note, some above arguments are being passed this way because this routine might be called in background thread
		simmodel.SanityCheck();
		auto &vals = cnn_output_analysis.vals;
		auto &crays = cnn_output_analysis.crays;
		float cloudforce = std::min(cloudforce_max_point, cloudforce_max_sum / vpts.size());

		for (int s = 0; s < steps; s++)
		{
			std::vector<LimitLinear > linears;
			std::vector<LimitAngular> angulars;
			if (s <steps_keyangles  || angles_only)
			{
				Append(angulars, cnn_output_analysis.ApplyAngles(simmodel, camera_pose, s< steps_palmangle ? 10000.0f : 0.0f));
			}
			if (s<steps_keypoints && !angles_only)
			{
				std::vector<LimitLinear> cnnconstraints;
				for (unsigned int i = ((steps_keyangles) ? 3 : 0); i<crays.size() && i < handmodelfeaturepoints.size(); i++) if (i >= 3 && cnn_output_analysis.finger_clenched[i - 3]<3.14f / 2.0f && crays[i].w >= min_cray_prob)
				{
					auto q = quat_from_to({ 0,0,1 }, crays[i].xyz());
					Append(cnnconstraints, ConstrainAlongDirectionDeadzone(NULL, camera_pose.position, &simmodel.rigidbodies[handmodelfeaturepoints[i].bone], handmodelfeaturepoints[i].offset, qxdir(q), 0.01f, { -100000.0f, 100000.0f }));
					Append(cnnconstraints, ConstrainAlongDirectionDeadzone(NULL, camera_pose.position, &simmodel.rigidbodies[handmodelfeaturepoints[i].bone], handmodelfeaturepoints[i].offset, qydir(q), 0.01f, { -100000.0f, 100000.0f }));
				}
				linears = cnnconstraints;
			}
			if (s >= steps_cloudstart && vpts.size() && cloudforce > 0.0f  && !angles_only)
			{
				std::vector<LimitLinear> pointcloudconstraints = CloudConstraints(Addresses(simmodel.rigidbodies), takesubsample(vpts), camera_pose.position);
				for (auto &c : pointcloudconstraints)
					c.forcelimit = float2(-cloudforce, cloudforce)*(c.rb1 == &simmodel.rigidbodies[0] ? 0.1f : 1.0f);
				Append(linears, pointcloudconstraints);
			}
			HandModelEnhancements(simmodel, angulars, false, qrot(camera_pose.orientation, float3(-1, 0, 0)), qrot(camera_pose.orientation, float3(0, -1, 0)));  // should incorporate tilt info into arm direction 0,-1,0
			simmodel.FitPointCloud(std::vector<float3>(0), linears, angulars);
			for (auto &rb : simmodel.rigidbodies)
				rb.angular_momentum = rb.linear_momentum = float3(0.0f);
		}
		simmodel.SanityCheck();
	}


	tracking_with_cnn_results update_cnn_model_threadsafe(Image<unsigned short> dimage)
	{
		// because this may be run in a separate thread, instead of writing into certain member variables,
		// local variables are created and returned to main thread.  
		float2 drange = { 0.1f,drangey };
		auto segment = HandSegmentVR(dimage, 0xF, drange, segment_scale);  // should give as cropped/zoomed/aligned subsample of the region of interest
		DCamera hcam = camsub(segment.cam, 4);
		auto cnn_input = Transform(segment, [drange, &segment](unsigned short d) {return (float)clamp(1.0f - (d*segment.cam.depth_scale - drange.x) / (drange.y - drange.x), 0.0f, 1.0f); });
		auto cnn_output = cnn.Eval(cnn_input.raster);
		auto cnn_output_analysis = CNNOutputAnalysis(cnn_output, hcam);
		auto vpts = takesubsample(PointCloud(dimage, drange),subsample_fraction);
		float olderror = FitError(handmodel, vpts, dimage);   // might be thread issue looking at handmodel object

		if (angles_only || olderror>full_reset_on_error)
		{
			PoseFromScratch(othermodel, vpts, cnn_output_analysis, segment.cam.pose);
			for (int i = 0;i < steps_unibody;i++)
				UnibodyFit(othermodel, vpts, segment.cam.pose.position);
		}

		MultiStepSim(othermodel,  cnn_output_analysis, vpts, segment.cam.pose );
		float newerror = FitError(othermodel, vpts, dimage);
		if (newerror > olderror)
			prev_frame_error = 0.0f;
		else
			prev_frame_error += olderror - newerror;

		std::vector<Pose> pose;
		if((vpts.size() > min_point_num&&initializing) || always_take_cnn || angles_only || prev_frame_error > accum_error_threshold)
			pose = othermodel.GetPose() ;
		if (prev_frame_error > accum_error_threshold) {
			prev_frame_error = 0.0f;
		}
		initializing = std::max(initializing - 1, 0);
		tracking_with_cnn_results results = { cnn_input,cnn_output,cnn_output_analysis,pose };
		return results;
	}
	static auto runcnn(Image<unsigned short> dimage, HandTracker &htk)
	{
		return htk.update_cnn_model_threadsafe(std::move(dimage));
	}
	std::vector<Pose> update_cnn_model(Image<unsigned short> dimage)  // not separate thread
	{
		auto results = update_cnn_model_threadsafe(std::move(dimage));
		cnn_input = results.cnn_input;
		cnn_output = results.cnn_output;
		cnn_output_analysis = results.cnn_output_analysis;
		return results.pose;
	}

	void kickstart(Image<unsigned short> dimage)  // runs cnn in current thread
	{
		handmodel.SetPose(update_cnn_model(std::move(dimage)));
	}

	std::vector<Pose> update(Image<unsigned short> dimage)   // use background thread for CNN
	{
		// this is intended to be the usual main update routine for typical usage of hand tracking.
		// A background thread is used for CNN evaluation where update frequency may fall behind that of the input depth stream.
		// The main thread will continue to do incremental frame-to-frame updates of the hand pose just using a quick pointcloud fit.
		auto points = takesubsample(PointCloud(dimage, { 0.1f,drangey }),subsample_fraction,subsample_voxel,subsample_size);

		if (!pose_estimator.valid())
		{
			othermodel.SetPose(handmodel.GetPose());
			pose_estimator = std::async(std::launch::async, [&]() {return runcnn(std::move(dimage), *this); });
		}
		if (pose_estimator.valid() && pose_estimator.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready)
		{
			auto results = pose_estimator.get();
			cnn_input = results.cnn_input;
			cnn_output = results.cnn_output;
			cnn_output_analysis = results.cnn_output_analysis;
			handmodel.SetPose(results.pose);
			pose_estimator = std::future< tracking_with_cnn_results>();
		}
		for(int i=0;!angles_only && i<mainthreadpasses;i++)
		{
			std::vector<LimitLinear>  linears;
			std::vector<LimitAngular> angulars;
			HandModelEnhancements(handmodel, angulars, false, float3(0, 0, 0), float3(0, 0, 0), 0);  // just adds some extra joint range limits
			if (points.size() > min_point_num && boundary_planes)
			{
				std::vector<float3> outdirs = { float3(-1, -0.25f, 0),  float3(-1, -1, 0), float3(0, -1, 0), float3(1, -1, 0), float3(1, -0.25f, 0) };// { float3(-1, 0, 0), float3(-1, -1, 0), float3(0, -1, 0), float3(1, -1, 0), float3(1, 0, 0) };
				Append(linears, cloud_chamber(handmodel, points, outdirs, { 0,0,0 }, { 0,0,1 }, 10.0f));
			}
			handmodel.FitPointCloud(points, linears, angulars, microforce);
		}
		if (points.size() < min_point_num)
			initializing = 50;
		handmodel.GetMeshes(1);
		return handmodel.GetPoseUser();  // in rig space since that the more typical convention used for modeling/rendering skin meshes
	}
	void slowfit(const std::vector<float3> &points,int hold,const std::vector<Pose> &refpose, int steps_=6, RigidBody* selectrb=NULL, const float3 &spoint=float3(0.0f), const float3 &rbpoint = float3(0.0f), const std::vector<float4> &crays= std::vector<float4>(0))
	{
		// this is yet another "update" routine, but with the emphasis on just getting an incremental and consistent/stable updated fit of the hand model to the point cloud
		// assumes we are trying to get as best fit and can assume temporal coherence
		// typically this routine is used during ground-truth auto-labeling motion capture 
		for (int st = 0; st < steps_;st++)
		{
			bool tiepinkyringmid = 0;
			int fingerhold = 0;
			std::vector<LimitLinear>  linears;
			std::vector<LimitAngular> angulars;

			HandModelEnhancements(handmodel, angulars, tiepinkyringmid, float3(0, 0, 0), float3(0, 0, 0), fingerhold);
			if (hold && refpose.size() )
				Append(angulars, handmodel.RelativeAngularConstraints(refpose, [&](const PhysModel::Joint &j) {return (&j != &handmodel.joints[0] && hold == 2) || (&j>&handmodel.joints[3]); }));   // ignore just the wrist to palm joint - nobody can keep that rigid

			for (unsigned int i = 0; st<5 && i<crays.size() && i < handmodelfeaturepoints.size(); i++)   // not called much, just a seldom used feature available in the annotation fixing program  
			{
				auto q = quat_from_to({ 0,0,1 }, crays[i].xyz());
				Append(linears, ConstrainAlongDirectionDeadzone(NULL, float3(0,0,0) , &handmodel.rigidbodies[handmodelfeaturepoints[i].bone], handmodelfeaturepoints[i].offset, qxdir(q), 0.01f, { -100000.0f, 100000.0f }));
				Append(linears, ConstrainAlongDirectionDeadzone(NULL, float3(0,0,0) , &handmodel.rigidbodies[handmodelfeaturepoints[i].bone], handmodelfeaturepoints[i].offset, qydir(q), 0.01f, { -100000.0f, 100000.0f }));
			}

			if (selectrb)
				Append(linears, ConstrainPositionNailed(NULL, spoint, selectrb, rbpoint));   //  to support user dragging individual model bones 

			if (st < steps_-1)
			{
				auto cc = CloudConstraints(Addresses(handmodel.rigidbodies), points);
				for (auto &c : cc)
					c.forcelimit *= microforce * (1.0f*(steps_ - st) / (float)steps_) *((c.rb1 == &handmodel.rigidbodies[0]) ? 0.1f*(st < steps_-2) : 1.0f);
				Append(linears, cc);
			}
			handmodel.FitPointCloud(std::vector<float3>{}, linears, angulars);
		}
	}
	void load_config(const std::string & jsonfile)
	{
		if (!fileexists(jsonfile))
			return;
		auto js = json::parsefile(jsonfile);
		from_json(*this, js);
	}

	HandTracker() : handmodel(LoadHandModel()), othermodel(LoadHandModel()),       // two models as one might be used in 2nd thread
		cnn(PoseInitializerCNN("../assets/handposedd.cnnb")),          // may want to train cnn for specific camera and use case
		vanity_bones(load_bone_meshes("../assets/vanity_bones.json")), // currently in rb com space 
		cnn_input(int2(64, 64), 0.0f),
		cnn_output(8 * 16 * 16 + 16 * 16, 0.01f),
		cnn_output_analysis(cnn_output, DCamera(int2(16)))
	{
		physics_gravity = { 0,0,0 };      // set physics globals that make sense for hand to point cloud fitting
		physics_driftmax = 0.03f / 8.0f;    // set to 1/8th the default,  just a couple of millimeters  
	}

	~HandTracker()
	{
		pose_estimator = std::future< tracking_with_cnn_results>();
	}

};

#endif // HANDTRACK_SYSTEM_H
