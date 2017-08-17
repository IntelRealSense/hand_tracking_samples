//
//  ovr-hand-tracker - a sample hand tracking viewer specifically for Oculus SDK LibOVR
//
//  This program assumes a RealSense depth camera (eg SR300) is attached to the VR HMD facing forward.
//  Like the other samples, right hand only within sensor range.
//
//  This program will not compile out-of-the-box without some additional work.
//  You will need to get the oculus sdk and put a copy of it in the ../third_party/ subfolder
//  also, librealsense, this project and oculus library libovr.lib all need to be compiled with the same project settings
//  for c/c++ => code generation => runtime library in order to avoid linker errors.
//  See both readme.md and dx_ovr.h for more information.
// 
//  This program offers a couple of viewing modes that can be changed with the number keys 1-4.
//  The point-cloud depth-mesh is drawn offset to the side of the hand model.
//  Toggle this on/off with the 'm' key.
//  The +/- keys will uniform scale the hand model which may improve results for
//  users with a hand that isn't 17 cm long. 
//  


#include <cctype>  // std::tolower
#include <algorithm>  // std::max


#define STB_IMAGE_IMPLEMENTATION
#include "../third_party/stb_image.h"
#include "../third_party/linalg.h"
#include "../third_party/mesh.h"
#include "../third_party/wingmesh.h"
#include "../third_party/json.h"

#include "dxwin.h"

#include "../include/dcam.h"
#include "../include/misc_image.h"
#include "../include/handtrack.h"  
//  to link statically with the oculus (ovr) statically linked libraries 
// under project settings => c++ => code generation => runtime library : change from default to use "multi threaded" and "multi threaded debug" (not multi threaded dll) 
#include "dx_ovr.h"

Image<byte3> loadimage(std::string filename)
{
	int2 dims;
	int unused;
	auto data = stbi_load(filename.c_str(), &dims.x, &dims.y, &unused, 3);
	if (!data)
	{
		Image<byte3> image(int2(64, 64));
		for (auto p : rect_iteration(image.dim()))
			image.pixel(p) = ((p.x & 8) == (p.y & 8)) ? byte3(128) : byte3(255);
		return image;
	}
	return Image<byte3>(dims, (byte3*)data);
}
int main(int argc, char *argv[]) try
{
	OVRWin ovrwin("htrack test on ovr");
	RSCam dcam;
	dcam.Init();
	HandTracker htk;

	ovrwin.vsync = 0;
	WingMesh smallcube = WingMeshCube(0.01f);
	std::vector<Mesh> meshes;
	for (auto p : vol_iteration({ 7,7,7 }))
	{
		meshes.push_back(MeshFlatShadeTex(smallcube.verts, smallcube.GenerateTris()));
		meshes.back().pose.position = float3(p - int3(3))*0.7f;
	}

	ovrwin.maps["rgbcam"] = ovrwin.MakeTex(nullptr, 640, 480);
	ovrwin.materials["rgbcam"] = { "rgbcam","","texonly" };
	ovrwin.materials["rimlit"] = { "white","","rimlit" };

	auto fmeshes = htk.handmodel.sdmeshes; 
	for (auto &m : fmeshes) 
		(m = MeshTexSphereMap(m)).material="handtex";
	for (auto &m : fmeshes) for (auto &v : m.verts)
		v.texcoord.x *= 2;
	auto gmap = loadimage("../assets/handtex.bmp");
	ovrwin.maps["handtex"] = ovrwin.MakeTex({ gmap.raster.data(), gmap.dim() });
	ovrwin.materials["handtex"] = Material{"handtex","","texlit"};
	int dmode = 3;
	int frame = 0;
	ovrwin.keyboardfunc = [&](int key, int, int)
	{
		switch (std::tolower(key))
		{
		case '0':case '1':case '2': case'3':case'4':  dmode = (key - '0')%4;break;
		case '+': case '=': htk.scale(     1.02f);  std::cout << "segment_scale " << htk.segment_scale << std::endl; break;
		case '-': case '_': htk.scale(1.0f/1.02f);  std::cout << "segment_scale " << htk.segment_scale << std::endl; break;
		case 'm': htk.showdepthmesh = !htk.showdepthmesh; break;
		default: std::cerr << "unused key " << (char)key << std::endl; break;
		}
	};


	Image<byte3> dcolor({ 640,480 });

	while (ovrwin.WindowUp())
	{
		float c[] = { 0.5f, 0.6f, 1.0f, 1.0f };
		frame++;
		auto dimage = dcam.GetDepth();
		// color data and depth based mesh
		dcolor = Image<byte3>({ dcam.dim() }, (const byte3*)dcam.dev->get_frame_data(rs::stream::color_aligned_to_depth));
		auto c4image = Transform(dcolor.raster, [](byte3 b) { return (unsigned int)0xFF << 24 | b.z << 16 | b.y << 8 | b.x;});
		ovrwin.TexUpdate(ovrwin.maps["rgbcam"], c4image.data());
		auto dmesh = DepthMesh(dimage, { 0.1f,0.7f }, 0.015f, 2);
		auto dxmesh = MeshSmoothish(dmesh.first, dmesh.second);
		dxmesh.material = "rgbcam";
		dxmesh.pose = ovrwin.last_hmd_pose * Pose(float3(dmode?0.2f:0.0f,0,0.0f), { 1,0,0,0 });  // push mesh to the side if showing the model
		for (auto &v : dxmesh.verts)
			v.texcoord = dimage.cam.projectz(v.position) / float2(dimage.cam.dim());

		htk.update(std::move(dimage));

		{
			for (unsigned int i = 0;i < htk.handmodel.rigidbodies.size();i++)
				fmeshes[i].pose = ovrwin.last_hmd_pose * Pose(float3(0.0f, 0.0f, 0.0f), { 1,0,0,0 }) *  htk.handmodel.rigidbodies[i].PoseUser();


		}
		auto bmeshes = htk.get_vanity_bones(ovrwin.last_hmd_pose * Pose(float3(0.0f, 0.0f, 0.0f), { 1,0,0,0 }));

		auto allmeshes = Addresses(meshes);

		for (auto &m : htk.handmodel.sdmeshes)
			m.pose = ovrwin.last_hmd_pose * Pose(float3(-0.0f ,0.0f, 0.0f), { 1,0,0,0 }) * m.pose;


		if (dmode!=0)  // because 0 is showing the depth mesh directly in front of user
		{
			auto &hmeshes = (dmode == 1) ? bmeshes : (dmode==2)? htk.handmodel.sdmeshes : fmeshes;
			Append(allmeshes, Addresses(hmeshes));
			
		}

		if(htk.showdepthmesh || !dmode)
			allmeshes.push_back(&dxmesh);

		ovrwin.RenderVR({ { 0, 0, 0 }, normalize(float4(1, 0, 0, 1)) }, allmeshes);
	}
}
catch (const char *c)
{
	MessageBox(GetActiveWindow(), "FAIL", c, 0);
}
catch (std::exception e)
{
	MessageBox(GetActiveWindow(), "FAIL", e.what(), 0);
}




