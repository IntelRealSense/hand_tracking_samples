//
//  openvr handtracking viewer sample
//  requires depth camera and vr headset
//  just uses opengl 1.1 and glew without other dependencies
//

#if defined(__APPLE__) || defined(__MACOSX__) 
#include <stdio.h>
int main() { printf("OS not supported\n"); return 0; }
#else

#include <cctype>         // std::tolower

#include "../third_party/gl_openvr.h"
#include "../third_party/geometric.h"    // also includes linalg.h and uses hlsl aliases
#include "../third_party/mesh.h"         // a simple mesh class with vertex format structure (not tied to dx or opengl)
#include "../third_party/glwin.h"        // does the win32 and opengl setup,  similar to glut or glfw,  header file only implementation
#include "../third_party/misc_gl.h"      // simple mesh drawing, and to draw a scene (camera pose and array of meshes)
#include "dcam.h"         // wraps librealsense
#include "handtrack.h"    // HandTracker - the system for tracking the hand including eval of cnn in separate thread, physics update, model loading


#ifdef _DEBUG
#pragma message ("ATTENTION:  Switch To RELEASE Mode!!   Debug is really slow!!")
#endif

int main(int argc, char *argv[]) try
{
	GLWin glwin("htk - testing hand tracking for vive in VR vanilla opengl", 1280, 720);

    if (glewInit() != GLEW_OK) 
    {
        std::cout << "GLEW init failure : " << glewInit() << std::endl;
        return 1;
    }

    GL_OpenVR vr;

	RSCam dcam;
	dcam.Init();
	HandTracker htk;
	htk.always_take_cnn = false;  // will just use incremental frame-to-frame when cnn result is less accurate

	glwin.keyboardfunc = [&](int key, int, int)
	{
		switch (std::tolower(key))
		{
		case '+': case '=': htk.scale(       1.02f);  std::cout << "segment_scale " << htk.segment_scale << std::endl; break;
		case '-': case '_': htk.scale(1.0f / 1.02f);  std::cout << "segment_scale " << htk.segment_scale << std::endl; break;
		case 'c': htk.always_take_cnn = !htk.always_take_cnn; break;
		default: std::cerr << "unused key " << (char)key << std::endl; break;
		}
	};

	while (glwin.WindowUp()) 
    {
        vr.wait_get_poses();

        // Update all meshes
		float c[] = { 0.5f, 0.6f, 1.0f, 1.0f };
		auto dimage = dcam.GetDepth();

		auto dcolor = Transform(dimage, [&](unsigned short d) {return byte3((unsigned char)clamp((int)(256 * (1.0f - (d*dimage.cam.depth_scale ) / htk.drangey)), 0, 255)); });
		// color data and depth based mesh
		auto ccolor = Image<byte3>({ dcam.dim() }, (const byte3*)dcam.dev->get_frame_data(rs::stream::color_aligned_to_depth));

		auto dmesh = DepthMesh(dimage, { 0.1f,0.7f }, 0.015f, 2);
		auto dxmesh = MeshSmoothish(dmesh.first, dmesh.second);
		for (auto &v : dxmesh.verts)
			v.texcoord = dimage.cam.projectz(v.position) / float2(dimage.cam.dim());
		dxmesh.pose.position.x = 0.15f; // offset 15cm to the side  this will reveal dcamera z noise making the depth mesh look a bit more jagged
		htk.update(std::move(dimage));

		std::vector<Mesh*> allmeshes;
		allmeshes.push_back(&dxmesh);
        Append(allmeshes, Addresses(htk.handmodel.sdmeshes));
        for(auto &m: allmeshes)
            m->pose = vr.get_hmd_pose() * Pose({0,0,0},{1,0,0,0}) *m->pose;  // put in front of hmd,  also compensate for cv to gl axes conventions with 180 about x

        // Render meshes onto the headset
        for (auto eye : { vr::Eye_Left, vr::Eye_Right }) 
        {
            glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
            vr.setup_for_eye(eye);// also clears the framebuffer with current clear color
            {
                glDisable(GL_BLEND);
                glEnable(GL_DEPTH_TEST);
                glEnable(GL_COLOR_MATERIAL);
                glEnable(GL_CULL_FACE);
                glColor3f(1, 1, 1);
                glEnable(GL_LIGHTING);
                glEnable(GL_LIGHT0);
                for (auto m : allmeshes)
                    MeshDraw(*m);
            }
            vr.submit_to_hmd(eye);
        }
        // Render mirror view into the window

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, glwin.res.x, glwin.res.y);

        // no need to clear color if we will draw mirror anyway
        glClear(GL_DEPTH_BUFFER_BIT);
        
        vr.draw_mirror();

        glwin.PrintString({ 0,0 }, "press ESC to quit, right hand only, cnn trained for egocentric ");
        glwin.PrintString({ 0,1 }, "hand size  %f cm   (use +/- to scale)", htk.segment_scale);
        glwin.PrintString({ 0,2 }, "apply cnn %s   ('c' to toggle)", htk.always_take_cnn ? "every time" : "only when ensures closer fit");
        glwin.SwapBuffers();
	}// end of render loop
}
catch (const char *c)
{
	MessageBox(GetActiveWindow(), "FAIL", c, 0);
}
catch (std::exception e)
{
	MessageBox(GetActiveWindow(), "FAIL", e.what(), 0);
}

#endif // excluded operating systems
