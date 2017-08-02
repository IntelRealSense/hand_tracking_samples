//
//  hand pose annotation tool  
//  
//  capture and record camera frames of a hand along with its full skeletal pose information. 
//  this version runs in a simple opengl window.
//
//  Rather than recording sequences and trying to label them after, 
//  its much less work to label if the ground truth can be capatured during the recording sessions.
//  The data being collected is useful to create ground truth datasets for deep learning.
//  See the readme.md file in this directory for further explanations about this application.
//  

#include <cctype>  // std::tolower
#include <algorithm>  // std::max
#include <future>

#include "../third_party/glwin.h"
#include "../third_party/misc_gl.h"
#include "../include/dcam.h"
#include "../include/handtrack.h"
#include "../include/dataset.h"  // to load in pose files for animation bank

template <class T> void random_permutation(std::vector<T> &a)  // in place
{
	for (unsigned int i = 0;i < a.size();i++)
		std::swap(a[i], a[((unsigned int)rand()) % a.size()]);
}


int main(int argc, char *argv[]) try
{

	GLWin glwin("Hand Pose Annotation - utility to record full skeletal pose and corresponding vid frames",800,600);
	RSCam dcam;
	dcam.Init();    // initialize realsense depth camera 

	HandTracker htk;
	htk.load_config("../config.json");

	int  frameid    = 0;
	bool recording  = 0;
	bool inspecting = 0;  // when in this mode, the program is replaying previously saved frames 
	int  kickstart  = 50; // when hand enters view, use cnn for the first handful of frames to get initial fit
	int  hold       = 0;  // flag to lock fingers,thumb   useful for getting ground truth for highly occluded views
	int  meshvis    = 1;  // 1 is vanity skinny bones   0 is either subd or collision bones
	int  keeprate   = 2;  // when recording we dont need to save every frame, just every 1/keeprate of them

	std::vector<Pose> refpose;  // provides pose info when holding the fingers and thumb in fixed pose relative to palm  

	std::vector<Frame> frames;
	DepthDataStreamOut streamout;
	int total_frames_saved = 0;
	streamout.prefix = DEFAULT_HANDPOSE_FILE_PREFIX;
	auto saveframes = [&]()
	{
		if (frames.size() == 0) return;
		if (!streamout.file_out_depth.is_open())  // open output file on demand.
		{
			streamout = DepthDataStreamOut(streamout.prefix, frames[0].depth.cam, { 0,0,0,FLT_MAX }, dcam.CamName(),htk.segment_scale);
			if (dcam.dev->supports(rs::capabilities::fish_eye))
			{
				streamout.AddFishEye().AddRGB();
				std::ofstream(streamout.prefix + ".test") << json::tabbed(to_json(MakeIntrinsicSet(dcam.dev)), 2);
			}
		}
		streamout.SaveFrames(frames);
		total_frames_saved += (int)frames.size();
		frames.clear();
	};
	int current_inspection;
	int fingerhold = 0;
	bool tiepinkyringmid = false;
	Image<unsigned short> dimage({ 320,240 });
	float viewdist = 0.6f, yaw = 180.0f;              // for viewing the depth data from different vantage points  

	glwin.keyboardfunc = [&](int key, int, int)
	{
		switch (std::tolower(key))
		{
		case '0':case '1':case '2': case'3': case'4': fingerhold ^= 1 << (key - 0x30);  std::cout << "fingerhold: " << fingerhold << std::endl;   break;
		case 'y': tiepinkyringmid = !tiepinkyringmid; break;
		case '-': case '_': if(!total_frames_saved)htk.scale(1.0f / 1.02f);  std::cout << "segment_scale " << htk.segment_scale << std::endl; break;
		case '+': case '=': if(!total_frames_saved)htk.scale(1.02f       );  std::cout << "segment_scale " << htk.segment_scale << std::endl; break;
		case 'r': recording = !recording; break;
		case 'h': ++hold %= 3; refpose = htk.handmodel.GetPose(); break;
		case '\b': if (frames.size())frames.erase(frames.begin() + current_inspection); break; // .pop_back();break;
		case 'x': yaw = 180.0f; frames.clear(); break; // delete all buffered up frames
		case 'i': inspecting = !inspecting; hold = 0; recording = false;yaw = 180.0f; break;
		case 'm': ++meshvis %= 3; break;
		case 's': saveframes(); yaw = 180.0f;  break;
		case '[': current_inspection--; break;
		case ']': current_inspection++; break;
		case '{': current_inspection = 0; break;
		case '}': current_inspection = (int)frames.size() - 1; break;
		case '.': break; // do nothing
		default: std::cerr << "unused key " << (char)key << std::endl; break;
		}
		inspecting = (frames.size() && !recording) ? inspecting : 0;
		current_inspection = std::max(0, current_inspection);
		current_inspection = std::min((int)frames.size() - 1, current_inspection);

	};
	glwin.keydownfunc = [&](int key)
	{
		if (key == VK_LEFT ) glwin.keyboardfunc('[',0,0);
		if (key == VK_RIGHT) glwin.keyboardfunc(']',0,0);
	};
	auto color_intrinsics = dcam.dev->get_stream_intrinsics(rs::stream::color_aligned_to_depth);
	auto ccolor = Image<byte3>({ color_intrinsics.width,color_intrinsics.height });


	while (glwin.WindowUp())
	{
		frameid++;
		dimage = dcam.GetDepth();

		auto ccamcolor = (const byte3*)dcam.dev->get_frame_data(rs::stream::color_aligned_to_depth);
		if (ccamcolor)
			ccolor.raster = std::vector<byte3>(ccamcolor, ccamcolor + product(ccolor.dim()));


		if (inspecting)  // in this mode we are reviewing previously captured frames
		{
			dimage        = frames[current_inspection].depth;
			dcam.cache_ir = frames[current_inspection].ir;
			ccolor        = frames[current_inspection].rgb;
			htk.handmodel.SetPose(frames[current_inspection].pose);
		}

		auto dimage_byte3 = Transform(dimage, [&](unsigned short d) {return byte3((unsigned char)clamp((int)(256 * (1.0f - (d*dimage.cam.depth_scale) / htk.drangey)), 0, 255)); });

		// color data and depth based mesh

		auto dmesh  = DepthMesh(dimage, { 0.1f,0.7f }, 0.015f, 2);   // points (just locations) and tris
		auto dxmesh = MeshSmoothish(dmesh.first, dmesh.second);      // vertices with vertex attributes
		dxmesh.material = "rgbcam";
		for (auto &v : dxmesh.verts)
			v.texcoord = dimage.cam.projectz(v.position) / float2(dimage.cam.dim());   // generate texcoords based on camera intrinsics

		auto points = takesubsample(PointCloud(dimage, { 0.1f,0.6f }));
		random_permutation(points);  // just a test, as far as i can tell this did not affect results
		float error = 0;

		if (inspecting)
			; // do not simulate anything 
		else if (points.size() < 20)   // not enough in view of camera
		{
			hold = 0;
			recording = false;
			if (!kickstart)
				inspecting = frames.size()>0;  // if buffered frames then pop into inspection mode right after hand removed from capture scene
			kickstart = 50;       // too few points, dont do anything, but be sure to invoke CNN when hand first enters scene
		}
		else if (kickstart)       // hand recently entered view volume
		{
			kickstart--;
			htk.kickstart(dimage);
		}
		else
		{
			htk.slowfit(points, hold, refpose);
			error = FitError(htk.handmodel, points, dimage);
		}

		auto hmeshes = htk.handmodel.GetMeshes(false);  // tracking model point cloud fitting meshes convex hulls before shrinked
		auto fmeshes = htk.handmodel.GetMeshes(true);   // tracking model subd-based bone pieces
		auto bmeshes = htk.get_vanity_bones();
		std::vector<Mesh>* bone_display_options[] = { &hmeshes,&fmeshes,&bmeshes };   // various ways to draw the skeleton hand 

		if (recording && !(frameid%keeprate))
		{
			current_inspection = (int)frames.size();
			Frame f = MakeFrame(dimage, htk.handmodel.GetPose(), dcam.cache_ir, ccolor, dcam.GetFishEye());   // fish eye and color will only written out if we set it up to export those
			frames.push_back(f);
		}

		auto bone_meshes = Addresses(*bone_display_options[meshvis]); // meshvis == 2 ? hmeshes : meshvis ? bmeshes : fmeshes));

		if(glwin.MouseState && glwin.mousepos.y< glwin.res.y-200 )    // only rotate view if we are away from slider widgets 
			yaw += (glwin.mousepos - glwin.mousepos_previous).x ;   
		viewdist *= powf(1.1f, (float)glwin.mousewheel);
		auto glcamera = Pose({ 0,0,0.5f }, qmul(float4(0, 0, 1, 0), QuatFromAxisAngle({ 0,1,0 }, yaw*3.14f / 180.0f)))*Pose({ 0,0,viewdist }, { 0,0,0,1 });
		float4 clearcolor = (recording) ? float4{ 0.15f, 0.1f*hold, 0, 0 } : (inspecting) ? float4{ 0.0f, 0.0f, 0.15f, 0.0f } : float4{ 0.0f, 0.15f + 0.1f*hold, 0.25f*(kickstart>0), 0.0f };
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glViewport(0, 0, glwin.res.x, glwin.res.y);
		glClearColor(clearcolor.x, clearcolor.y, clearcolor.z, 0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		{
			// render scene with camera near origin but rolled 180 on x to align CV convention with GL convention
			glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
			glPerspective(60.0f, 1.0f*glwin.res.x/glwin.res.y, 0.01f, 50.0f);
			glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
			glMultMatrixf(glcamera.inverse().matrix()); 

			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE); glEnable(GL_COLOR_MATERIAL); glColor3f(1, 1, 1);
			glEnable(GL_LIGHTING); glEnable(GL_LIGHT0);
			for (auto &m : bone_meshes)
				MeshDraw(*m);
			glPopAttrib();

			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glDisable(GL_LIGHTING);
			glDisable(GL_DEPTH_TEST);
			glEnable(GL_TEXTURE_2D);
			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			glBindTexture(GL_TEXTURE_2D, 0);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ccolor.dim().x, ccolor.dim().y, 0, GL_RGB, GL_UNSIGNED_BYTE, ccolor.raster.data());
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			dxmesh.hack = float4(1, 1, 1, 0.5f);
			MeshDraw(dxmesh);
			glPopAttrib();

			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glLineWidth(2.0f);
			glColor3fv({ 0.50f, 0.50f, 0.60f });
			if(yaw!=180.0f)
				glwirefrustumz(dimage.cam.deprojectextents(), { 0.15f,htk.drangey });
			glPopAttrib();

			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();

			drawimage({ dimage_byte3.raster, dimage_byte3.dim() }, { 0.01f,0.21f }, { 0.2f,-0.2f });  // show depth feed from camera
			drawimage({ ccolor.raster      , ccolor.dim()       }, { 0.01f,0.42f }, { 0.2f,-0.2f });  // show color feed from camera
			if (kickstart && kickstart<50)
			{
				drawimage(htk.get_last_segment() , { 0.01f,0.63f }, { 0.15f,-0.2f });  // the segmentation sent to the cnn
			}
		}
		glwin.PrintString({ 0,0 }, "press ESC to quit, right hand only, capt ");
		glwin.PrintString({ 0,1 }, "hand size  %f cm   %s", htk.segment_scale,total_frames_saved?"":"(use +/- to scale)");
		if(inspecting)
			glwin.PrintString({ 0,2 }, "Inspecting frame %d of %d.  delete frame bksp  back '['  forward ']'   save batch 's' ",current_inspection,frames.size());
		else if (kickstart==50)
			glwin.PrintString({ 0,2 }, "Nothing in view of depth camera");
		else if(kickstart)
			glwin.PrintString({ 0,2 }, "Getting an initial hand fit ... %d ",kickstart);
		else if(!recording)
			glwin.PrintString({ 0,2 }, "Fitting hand model to point cloud,  'r' to record");
		else
			glwin.PrintString({ 0,2 }, "RECORDING frames  'r' to stop");
		if(!kickstart && !inspecting)
			glwin.PrintString({ 0,3 }, "fingers %s and thumb %s  'h' to hold",hold?"locked":"free",hold>1?"locked":"free");
		if (error > 0.3f)
			glwin.PrintString({ 0,4 }, "fit err = %5.3f", error);
		if(frames.size())
			glwin.PrintString({ 0,-3 }, "frames buffered: %d", (int)frames.size());
		if (total_frames_saved)
			glwin.PrintString({ 0,-1 }, "File: %s  frames saved: %d", streamout.prefix.c_str(), total_frames_saved);
		else // allow for hand resizing
		{
			float segment_scale = htk.segment_scale;
			WidgetSlider(int2{ 200, 8 }, int2{ 400,28 }, glwin, segment_scale, float2{ 0.12f ,0.22f }, "[+/-] handsize ").Draw();
			if (segment_scale != htk.segment_scale)
				htk.scale(segment_scale / htk.segment_scale);
		}
		if(frames.size())
			WidgetSwitch(int2{ 600 + 2, 125 }, { 200 - 4,30 - 2 }, glwin, inspecting, "[i] Inspecting ", [&](){hold=0;recording=0;yaw=180.0f;}).Draw();
		
		if (yaw != 180.0f)  // add a button to reset gl view camera to be aligned with depth camera
			WidgetButton(int2(glwin.res.x - 70, 170), { 60,28 }, glwin, "ReCenter", [&yaw]() {yaw = 180.0f;}).Draw();
		if (inspecting)
		{
			WidgetSlider(int2{ 200, 35 }, int2{ 400,28 }, glwin,   current_inspection, int2{ 0 ,(int)frames.size()-1}," buffered frame ").Draw();
			WidgetButton(int2{ 600 + 2, 35 }, { 200 - 4,30 - 2 }, glwin, "[BKSP] Del Frame   ", [&]() {glwin.keyboardfunc('\b', 0, 0); }).Draw();
			WidgetButton(int2{ 600 + 2, 65 }, { 200 - 4,30 - 2 }, glwin, "[x] Del all Frames ", [&]() {glwin.keyboardfunc('x' , 0, 0); }).Draw();
			WidgetButton(int2{ 600 + 2, 95 }, { 200 - 4,30 - 2 }, glwin, "[s] Save all Frames", [&]() {glwin.keyboardfunc('s' , 0, 0); }).Draw();
		}
		else if (kickstart && kickstart < 50)
		{
			int i = 50 - kickstart;
			WidgetSlider(int2{ 200, 150 }, int2{ 400,18 }, glwin, i, int2{ 0 ,50 }, " initial fit ").Draw();
		}
		else if(!kickstart && !inspecting)
		{
			WidgetSlider(int2{ 600 + 2, 185 }, { 200 - 4,30 - 2 }, glwin, hold, { 0,2 }, "[h] Hold Pose ").Draw();
			WidgetSwitch(int2{ 600 + 2, 155 }, { 200 - 4,30 - 2 }, glwin, recording, "[r] Record ").Draw();
		}
		glPopAttrib();
		glwin.SwapBuffers();

	}
}
catch (const char *c)
{
	MessageBoxA(GetActiveWindow(), c, "FAIL", 0);
}
catch (std::exception e)
{
	MessageBoxA(GetActiveWindow(), e.what(), "FAIL", 0);
}
