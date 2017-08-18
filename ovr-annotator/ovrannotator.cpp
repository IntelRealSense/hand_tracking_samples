//
//  hand pose annotator  libovr vr version
//
// This project meant to be a recording tool for hand pose but specifically 
// for using ovr headset with realsense depthcam attached
// 

#include <cctype>  // std::tolower
#include <algorithm>  // std::max

//  to link this project and librealsense statically with the oculus (ovr) 
//  statically linked libraries they must match runtime library used
//  under project settings => c++ => code generation => runtime library  
//  change from default to use "multi threaded DLL" and "multi threaded DLL debug" 
//  see project ovr-hand-tracker and ../ovr-hand-tracker/readme.md for more information on this.
#include "../ovr-hand-tracker/dx_ovr.h"

#include "../include/dcam.h"
#include "../include/handtrack.h"  
#include "../include/dataset.h"  


int main(int argc, char *argv[]) try
{
	OVRWin ovrwin("ovr-annotator");
	RSCam dcam;
	dcam.Init();

	ovrwin.vsync = 0;
	ovrwin.EnableBlend();
	WingMesh smallcube = WingMeshCube(0.01f);
	std::vector<Mesh> environment_boxes;
	for (auto p : vol_iteration({ 7,7,7 }))
	{
		environment_boxes.push_back(MeshFlatShadeTex(smallcube.verts, smallcube.GenerateTris()));
		environment_boxes.back().pose.position = float3(p - int3(3))*0.7f;
	}

	HandTracker htk;

	int  frameid    = 0;
	int  animate    = 0;
	bool recording  = 0; 
	bool inspecting = 0;
	int  kickstart  = 50;
	int  hold       = 0;
	int  meshvis    = 1;  // 1 is vanity skinny bones   0 is either subd or collision bones
	int  keeprate   = 2;  // keep this fraction of the frames that are see 

	std::vector<Pose> refpose;
	Pose last_hmd_capture_pose;

	std::vector<Frame> frames;
	DepthDataStreamOut streamout;
	streamout.prefix = DEFAULT_HANDPOSE_FILE_PREFIX;
	auto saveframes = [&]()
	{
		if (frames.size() == 0) return;
		if (!streamout.file_out_depth.is_open())  // open output file on demand.
		{
			streamout = DepthDataStreamOut(streamout.prefix, frames[0].depth.cam, { 0,0,0,FLT_MAX }, dcam.CamName(), htk.segment_scale);
		}
		streamout.SaveFrames(frames);
		frames.clear();
	};
	int current_inspection;
	int fingerhold = 0;
	bool tiepinkyringmid=false;

	ovrwin.keyboardfunc = [&](int key, int, int)
	{
		switch (std::tolower(key))
		{
		case '0':case '1':case '2': case'3': case'4': fingerhold ^= 1 << (key - 0x30);  std::cout << "fingerhold: " << fingerhold << std::endl;   break;
		case 'y': tiepinkyringmid = !tiepinkyringmid; break;
		case '-': case '_': htk.scale(1.0f / 1.02f);  std::cout << "segment_scale " << htk.segment_scale << std::endl; break;
		case '+': case '=': htk.scale(1.02f       );  std::cout << "segment_scale " << htk.segment_scale << std::endl; break;
		case 'a': animate = 1 - animate; break;
		case 'r': if (inspecting) saveframes(); recording = !recording; break;
		case 'h': ++hold %= 3; refpose = htk.handmodel.GetPose(); break;
		case '\b': if (frames.size())frames.erase(frames.begin()+ current_inspection); break; // .pop_back();break;
		case 'k': kickstart = 50; break;
		case 'i':if (inspecting) saveframes(); inspecting = !inspecting; recording = false;hold = 0; break;
		case 'm': ++meshvis %= 2; break;
		case 's': saveframes();
		case '[': current_inspection--; break;
		case ']': current_inspection++; break;
		case '{': current_inspection=0; break;
		case '}': current_inspection=(int)frames.size()-1; break;
		case '.': break; // do nothing
		default: std::cerr << "unused key " << (char)key << std::endl; break;
		}
		inspecting = (frames.size()&&!recording)?inspecting:0;
		current_inspection = std::max(0, current_inspection);
		current_inspection = std::min((int)frames.size() - 1,current_inspection);

	};
	auto color_intrinsics = dcam.dev->get_stream_intrinsics(rs::stream::color_aligned_to_depth);
	auto dcolor = Image<byte3>({ color_intrinsics.width,color_intrinsics.height });

	ovrwin.maps["rgbcam"] = ovrwin.MakeTex(nullptr, dcolor.dim().x,dcolor.dim().y);
	ovrwin.materials["rgbcam"] = { "rgbcam","","texonly" };
	ovrwin.materials["rimlit"] = { "white","","rimlit" };

	Image<unsigned short> dimage({ 320,240 });

	ovrInputState ovr_input_state_previous;

	while (ovrwin.WindowUp())
	{

		ovrInputState ovr_input_state;
		ovr_GetInputState(ovrwin.session, ovrControllerType_Remote , &ovr_input_state);
		char bcoms[32] = "r\b..............hi[]ri-+";
		char icoms[32] = "r\b................[]\bs-+";
		char rcoms[32] =  "................][......";
		char *scoms = (inspecting) ? icoms : bcoms;
		for (int i = 0;i < sizeof(bcoms);i++)
			if (ovr_input_state.Buttons&(1 << i))
				if (ovr_input_state_previous.Buttons&(1 << i))
					{std::cout << rcoms[i] << ".." << i << std::endl;   ovrwin.keyboardfunc(rcoms[i], 0, 0); }
				else
					{std::cout << scoms[i] << " " << i << std::endl;   ovrwin.keyboardfunc(scoms[i], 0, 0); }
		ovr_input_state_previous = ovr_input_state;

		float c[] = { 0.5f, 0.6f, 1.0f, 1.0f };

		frameid++;

		dimage = dcam.GetDepth();

		auto dcamcolor = (const byte3*)dcam.dev->get_frame_data(rs::stream::color_aligned_to_depth);
		if (dcamcolor)
			dcolor.raster = std::vector<byte3>(dcamcolor, dcamcolor + product(dcolor.dim()));

		if (inspecting)
		{
			dimage              = frames[current_inspection].depth;
			dcam.cache_ir       = frames[current_inspection].ir;
			dcolor              = frames[current_inspection].rgb;
			htk.handmodel.SetPose(frames[current_inspection].pose);
		}
		else
		{
			last_hmd_capture_pose = ovrwin.last_hmd_pose ;
		}

		auto c4image = Transform(dcolor.raster, [](byte3 b) { return (unsigned int)0xB0 << 24 | b.z << 16 | b.y << 8 | b.x;});

		// color data and depth based mesh
		ovrwin.TexUpdate(ovrwin.maps["rgbcam"], c4image.data());
		auto dmesh = DepthMesh(dimage, { 0.1f,0.7f }, 0.015f, 2);
		auto dxmesh = MeshSmoothish(dmesh.first, dmesh.second);
		dxmesh.material = "rgbcam";
		dxmesh.pose = last_hmd_capture_pose*Pose(float3(0.0f, 0, 0.0f), { 1,0,0,0 });
		for (auto &v : dxmesh.verts)
			v.texcoord = dimage.cam.projectz(v.position) / float2(dimage.cam.dim());

		auto points = takesubsample(PointCloud(dimage, { 0.1f,0.6f }));
		if (inspecting)
			; // do not simulate anything
		else if (points.size() < 20)
		{
			recording = false;
			kickstart = 50;  // too few points, dont do anything, but be sure to invoke CNN when hand first enters scene
		}
		else if (kickstart)
		{
			kickstart--;
			htk.kickstart(dimage);
		}
		else
			htk.slowfit(points, hold, refpose);

		auto hmeshes = htk.handmodel.GetMeshes(false);  // tracking model point cloud fitting meshes convex hulls before shrinked
		auto fmeshes = htk.handmodel.GetMeshes(true);   // tracking model subd-based bone pieces
		std::vector<Mesh>* showmeshes[] = { &hmeshes,&fmeshes };   // various ways to draw the skeleton hand 

		for(auto &m:hmeshes)
			m.pose = last_hmd_capture_pose * Pose(float3(0.0f, 0.0f, 0.0f), { 1,0,0,0 }) *  m.pose;  // depth camera to opengl and camera placed on hmd
		for (auto &m : fmeshes)
			m.pose = last_hmd_capture_pose * Pose(float3(0.0f, 0.0f, 0.0f), { 1,0,0,0 }) *  m.pose;

		if (recording && !(frameid%keeprate))
		{
			current_inspection = (int)frames.size();
			Frame f = MakeFrame(dimage, htk.handmodel.GetPose(), dcam.cache_ir, dcolor, dcam.GetFishEye());   // fish eye and color will only written out if we set it up to export those
			frames.push_back(f);
		}
		auto allmeshes = Addresses(environment_boxes);

		Append(allmeshes, Addresses(*showmeshes[meshvis])); 
		allmeshes.push_back(&dxmesh);    // draw this last since its alpha blended

		ovrwin.clearcolor = (recording) ? float4{ 0.15f, 0.1f*hold, 0, 0}: (inspecting)? float4{0.0f, 0.0f, 0.15f, 0.0f}: float4{ 0.0f, 0.15f+0.1f*hold, 0.25f*(kickstart>0), 0.0f };
		ovrwin.RenderVR({ { 0, 0, 0 }, normalize(float4(1, 0, 0, 1)) }, allmeshes);
	}
}
catch (const char *c)
{
	MessageBox(GetActiveWindow(), c, "FAIL", 0);
}
catch (std::exception e)
{
	MessageBox(GetActiveWindow(),e.what(), "FAIL",  0);
}




