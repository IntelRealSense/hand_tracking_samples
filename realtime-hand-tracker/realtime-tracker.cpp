//
//  minimal sample implementation of hand tracking from depth camera.
//
//  The implementation here just uses simple win32 setup and gl wrapper with some basic convenience functions.
//  In particular, just uses opengl 1.1 without any toolkits or dependencies.
//  one ~100 line cpp only,  various support functionality included inline in header files.
//
//  As described in the readme file, there is no sophistication in the segmentation so use this program  
//  with just right hand in the depth camera view volume.
//  Note that results will vary from user to user based on how well it matches the hand model being used and 
//  the hands used in the datasets used to train the CNN.   
//
//  There are some runtime settings that will affect the tracking results, some are exposed as gui widgets.
//  For slower motion when the fingers are extended, its often best to only use the cnn results when 
//  there is a measurable better fit of the geometry to the point cloud.
//  for faster motions, or when there is less geometric features to track (such as a clenched rolling fist), it is usually better
//  to trust the cnn output. 
//

#include <cctype>         // std::tolower

#include "../third_party/geometric.h"    // also includes linalg.h and uses hlsl aliases
#include "../third_party/mesh.h"         // a simple mesh class with vertex format structure (not tied to dx or opengl)
#include "../third_party/glwin.h"        // does the win32 and opengl setup,  similar to glut or glfw,  header file only implementation
#include "../third_party/misc_gl.h"      // simple mesh drawing, and to draw a scene (camera pose and array of meshes)
#include "../third_party/json.h"    
#include "../include/handtrack.h"        // HandTracker - the system for tracking the hand including eval of cnn in separate thread, physics update, model loading
#include "../include/dcam.h"             // wraps librealsense

#ifdef _DEBUG
#pragma message ("ATTENTION:  Switch To RELEASE Mode!!   Debug is really slow!!")
#endif

int main(int argc, char *argv[]) try
{
	GLWin glwin("htk - testing hand tracking system  using realsense depth camera input",1280,720);
	RSCam dcam;
	dcam.Init(argc==2?argv[1]:"");
	HandTracker htk;
	htk.always_take_cnn = false;  // when false the system will just use frame-to-frame when cnn result is not more accurate
	glwin.keyboardfunc = [&](int key, int, int)
	{
		switch (std::tolower(key))
		{
		case '+': case '=': htk.scale(1.02f       );  std::cout << "segment_scale " << htk.segment_scale << std::endl; break;  // make model hand larger
		case '-': case '_': htk.scale(1.0f / 1.02f);  std::cout << "segment_scale " << htk.segment_scale << std::endl; break;  // make model hand smaller
		case 'c': htk.always_take_cnn = !htk.always_take_cnn; break;
		case 'a': htk.angles_only     = !htk.angles_only;     break;
		default: std::cerr << "unused key " << (char)key << std::endl; break;
		}
	};
	htk.load_config( "../config.json");

	while (glwin.WindowUp())
	{
		auto dimage     = dcam.GetDepth();
		auto dimage_rgb = Transform(dimage, [&](unsigned short d) {return byte3((unsigned char)clamp((int)(256 * (1.0f - (d*dimage.cam.depth_scale ) / htk.drangey)), 0, 255)); });
		auto ccolor = Image<byte3>({ dcam.dim() });
		if(dcam.dev) ccolor = Image<byte3>({ dcam.dim() }, (const byte3*)dcam.dev->get_frame_data(rs::stream::color_aligned_to_depth));  // color data from camera

		auto dmesh  = DepthMesh(dimage, { 0.1f,0.7f }, 0.015f, 2);  // create points and triangle list from the depth data
		auto dxmesh = MeshSmoothish(dmesh.first, dmesh.second);    // from 3d points to vertices will all the usual attributes
		for (auto &v : dxmesh.verts)
			v.texcoord = dimage.cam.projectz(v.position) / float2(dimage.cam.dim());
		dxmesh.pose.position.x = 0.15f; // offset 15cm to the side   note this may reveal make the depth mesh look a bit more jagged due to z noise

		htk.update(std::move(dimage));   // update the hand tracking with the current depth camera input

		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glViewport(0, 0, glwin.res.x, glwin.res.y);
		glClearColor(0.1f+0.2f*(htk.initializing==50) , 0.1f+0.1f*(htk.initializing>0), 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		{
			auto allmeshes = Addresses(htk.handmodel.sdmeshes);
			allmeshes.push_back(&dxmesh);
			// render scene with camera near origin but rolled 180 on x to align CV convention with GL convention
			render_scene({ { 0, 0, -0.05f }, normalize(float4(1, 0, 0, 0)) }, allmeshes);  
			drawimage(dimage_rgb              , { 0.01f,0.21f }, { 0.2f ,-0.2f });  // show depth feed from camera
			drawimage(ccolor                  , { 0.01f,0.42f }, { 0.2f ,-0.2f });  // show color feed from camera
			drawimage(htk.get_cnn_difference(), { 0.01f,0.63f }, { 0.15f,-0.2f });  // show segment sent to cnn
		}
		float segment_scale = htk.segment_scale;
		WidgetButton(int2{ 3, glwin.res.y - 24 }, int2{ 120,23 }, glwin, "[ESC] quit", []() {exit(0);}).Draw();
		WidgetSwitch(int2{ 3, glwin.res.y - 48 }, int2{ 220,23 }, glwin, htk.always_take_cnn, "[c] cnn priority").Draw();
		WidgetSwitch(int2{ 3, glwin.res.y - 74 }, int2{ 220,23 }, glwin, htk.angles_only    , "[a] cnn angles  ").Draw();
		WidgetSlider(int2{ 3, glwin.res.y - 102}, int2{ 220,23 }, glwin, segment_scale, float2{0.12f ,0.20f }, "[+/-] handsize ").Draw();
		if (segment_scale != htk.segment_scale)
			htk.scale(segment_scale / htk.segment_scale);  // only call the scale routine if its been changed
		glwin.PrintString({2+ 120 / glwin.font_char_dims.x,0 }, "press ESC to quit,  place right hand only in depth camera view, cnn trained for egocentric ");
		glwin.PrintString({2+ 220 / glwin.font_char_dims.x,2 }, "apply cnn %s   ('c' to toggle)", htk.always_take_cnn ? "every time" : "only when ensures closer fit");
		glwin.PrintString({2+ 220 / glwin.font_char_dims.x,4 }, "%s   ('a' to toggle)", htk.angles_only ? "not using depth, just cnn angles" : "using depth for final fit");
		glwin.PrintString({2+ 220 / glwin.font_char_dims.x,6 }, "hand size  %f cm   (use +/- to scale)", segment_scale);
		glPopAttrib();
		glwin.SwapBuffers();
	}
}
catch (const char *c)
{
	MessageBoxA(GetActiveWindow(), "FAIL", c, 0);
}
catch (std::exception e)
{
	MessageBoxA(GetActiveWindow(), "FAIL", e.what(), 0);
}
