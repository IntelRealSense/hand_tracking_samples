// 
// annotation-fixer   Hand pose data set editor 
//
// This program is for viewing and editing camera data and poses captured from
// the realtime-annotator program.
// Often when collecting data there will be bad frames (with extra items in the scen),
// or the hand tracking during capture may have not been properly lined up and the
// exported poses could be incorrect for some subset of frames.
//
// Use the slider widget or arrow keys to inspect the input frames, pointcloud,
// and corresponding pose for each frame to ensure alignment.
// Incorrect frames can be fixed by turning on simulate and adjusting the model.
// During simulate the attraction of the bones to the pointcloud can be adjusted.
// There is an available feature to "hold" the hand.
// This can be used to carry relative pose information from one frame to the next
// which is useful for "fixed fingers" subsequences that only differ by overall
// wrist roll and palm orientation. Also, bad frames are easily deleted.
// Sometimes its easier to just re-record.
//
// Given model 3D pose any preferred landmark or information label set is trivially derived.
// This program also visualizes the training label heatmaps corresponding to the ground truth
// model pose as well as the current CNN's outputs.  
// 
//  
// usage (view and edit one file):
//   cd datasets/
//   ../build/annotation-fixer___.exe   my_hand_capture_file.rs
//
// if no argument is provide, then it will load an example dataset
//

#include <exception> 
#include <iostream>
#include <fstream>
#include <cctype>    // std::tolower
#include <future>
#include <sstream>

#include "../third_party/geometric.h"
#include "../third_party/glwin.h"
#include "../third_party/mesh.h"
#include "../third_party/misc.h"
#include "../third_party/misc_gl.h"
#include "../third_party/cnn.h"
#include "../third_party/json.h"
#include "../third_party/misc_json.h"
#include "../include/physmodel.h"
#include "../include/handtrack.h"  
#include "../include/dataset.h"
#include "../include/misc_image.h"

const char* helptext[] =
{
	" left mouse spins the 3D view, mousewheel to zoom in/out",
	" arrow keys or slider change current frame number",
	" [BKSP] backspace key deletes the current frame (no undo without restart).",
	" [s] key toggles 'simulation' mode to fit the hand to the point cloud.",
	" [y] during sim mode keeps the pinky ring middle fingers together.",
	" [h] key to 'hold' the current relative pose during sim,",
	"     useful to apply to other frames that only differ by palm orientation",
	" [n] key enables CNN's landmark constraints when simulating (just for fun).",
	" [z] will undo simulation's changes to the current frame",
	" [CTRL-z] will revert all frames to starting pose",
	" [CTRL-s] saves out the depth, ir, and pose datafiles",
	" [ESC] or [q] to exit the program.",
};



int main(int argc, const char **argv) try
{
	std::cout << "Hand Pose Data-Set Editor.\n";
	std::cout << "use model to point cloud fitting to capture full pose\n";


	HandTracker htk; 
	float2 drange = { 0.20f, htk.drangey }; // in m   

	std::string dfilename = argc >= 2 ? argv[1] : "../datasets/example/hand_data_example.rs";

	for (int i = 2; i < argc; i++)
		if (filesuffix(argv[i]) == ".rs" || filesuffix(argv[i]) == ".ir")
			std::cout << "only edit one .rs file at a time " <<  basepathname(argv[i]) << " not loaded\n" ;
		else if (filesuffix(argv[i]) == ".cnnb")
			htk.cnn.loadb(argv[i]);

	auto bname = basepathname(dfilename);
	std::cout << "basepathname:  '" << bname << "'\n";

	std::vector<Frame> frames = load_dataset(bname, (int)htk.handmodel.rigidbodies.size());  // only uses handmodel to know how many bones to load for each frame

	try {
		float segment_scale = htk.segment_scale;
		segment_scale = json::parsefile(bname + ".json").get_object().at("segment_scale").number<float>();
		if (segment_scale > 0.1f && segment_scale < 0.3f && segment_scale != htk.segment_scale)
			std::cout << "doing rescale to: " <<   htk.scale(segment_scale / htk.segment_scale)  << std::endl;
	}
	catch (...) {}

	// GUI
	int currentframe = 0;
	int prevframe = -1;
	GLWin glwin((std::string("hposefix - edit captured hand pose data sets.  current file: ") + bname).c_str(), 1280, 720);
	
	Pose camera({ 0, 0, -0.10f }, { 1, 0, 0, 0 });  // the viewing opengl camera for rendering the 3D scene consisting of point cloud and posed hand model
	float focuszoffset = 0.5f;  // we pivot the viewing camera about a point half a meter in front of the origin (depth camera location)

	float  nnenable   = 0.0f;   // whether use cnn generated constraints
	bool   simulating = 0;      
	float  cloudforce = 0.4f;   // make this weaker to allow for slower fitting   default is ~1.0 
	bool   tiefingers = 0;      // adds constraints to the middle, ring, and pinky fingers to keep them together  helps reduce drift for many occluded poses
	bool   spinview   = false;    
	int    needsaving = false;  // first bit means current frame modified, 2nd bit means other frames modified, 3rd bit means frames deleted,
	bool   showhelp   = false;
	bool   hold       = false;  
	float3 colorflash(0, 0, 0); // added to the clear color  just used as visual feedback for some operations
	RigidBody *selectrb = NULL; // currently selected hand bone that can be moved by the user
	float3 spoint = camera * float3(0, 0, -10);
	float3 rbpoint;
	std::vector<Pose> refpose = htk.handmodel.GetPose();  // used for 'hold' effect to maintain relative pairwise joint angles across frames when editing

	auto drawimagevp = [&](const Image<byte3> &image, int2 c, int2 d, std::string caption = "")
	{
		glViewport(c.x, c.y, d.x, d.y);
		drawimage(image, float2(0, 1.0f), float2(1.0f, -1.0f));   // note the flip on vertical 
		int k = 0;
		if (caption.length()) for (auto s : split(caption, "\n")) glwin.PrintString({ 0,k++ }, s.c_str());
	};

	glwin.preshutdown = [&]() { if (needsaving) MessageBoxA(GetActiveWindow(), "You didn't save with CTRL-s", "Modifications Aborted", 0); };
	glwin.keydownfunc = [&](int vkey)  // for keys that aren't WM_CHAR events
	{
		currentframe = clamp(currentframe+(vkey==VK_RIGHT)-(vkey==VK_LEFT)+(vkey==VK_UP)*10-(vkey==VK_DOWN)*10, 0, (int)frames.size() - 1);
		if (vkey == VK_HOME) currentframe = 0;
		if (vkey == VK_END ) currentframe = (int)frames.size() - 1;
	};
	glwin.keyboardfunc = [&](unsigned char key, int x, int y)->void
	{
		switch (std::tolower(key))
		{
		case 'q': case 27: exit(0); break;  // ESC is already handled in mswin.h MsgProc
		case 'c': camera = { { 0, 0, 0 },{ 1, 0, 0, 0 } }; break;
		case '\1': currentframe = 0; break; // ctrl-A ??
		case '\5': currentframe = (int)frames.size() - 1; break; // ctrl-e ??
		case ']': currentframe++; break;
		case '[': currentframe--; break;
		case '}': currentframe += 10; break;
		case '{': currentframe -= 10; break;
		case 's': simulating = !simulating; needsaving |= 1;  break;
		case 'h': hold = !hold; refpose = htk.handmodel.GetPose();  break;
		case 'l': htk.handmodel.SetPose(frames[currentframe].pose); break;
		case 'z': needsaving &= 6; simulating = false;  htk.handmodel.SetPose(frames[currentframe].pose = frames[currentframe].startpose); colorflash = { 0.0f, 0.0f, 1.0f }; break;
		case 'y': tiefingers = !tiefingers; break;
		case 'u': cloudforce = 0.5f* (float)!cloudforce; break;
		case '\b': frames.erase(frames.begin() + currentframe); prevframe = -1; colorflash = { 0.3f, 0.3f, 0.3f }; needsaving = 4; break;
		case '\x13': std::cout << bname << " saving depth and pose data\n";  DepthDataStreamOut(bname).SaveFrames(frames); simulating=needsaving = false; break;   // ctrl-s
		case '\x18': {auto k = frames.size();  while (--k > 0) frames.erase(frames.begin() + k--); } needsaving = 4; break; //ctrl-x
		case '\x1A': simulating = 0;  for (auto &f : frames) f.pose = f.startpose; needsaving &= 4;  break;  // ctrl-z 
		case 'n': nnenable = (key == 'N') ? nnenable - 0.1f : !nnenable; break;
		case '+': case '-': case '_': case '=': if (selectrb) htk.handmodel.SetBonePoseHierarchyW((int)(selectrb-htk.handmodel.rigidbodies.data()), qmul(selectrb->orientation, QuatFromAxisAngle({ 1,0,0 },(key=='+'||key=='=' ? -1.0f : 1.0f)*3.1415f / 24.0f))); break;
		case '/': case '?': showhelp = !showhelp; break;
		//case 'r': 	MessageBox(GetActiveWindow(), "test a", "test b", 0); exit(0);  break;
		default:
			std::cout << "unassigned key (" << (int)key << "): '" << key << "'\n";
			break;
		}
		if (argc > 2) needsaving = false;  // when multiple files loaded then this is not applicable
		if (!simulating) tiefingers = false;
		currentframe = std::min(std::max(currentframe, 0), (int)frames.size() - 1);
	};
	while (glwin.WindowUp())
	{

		auto currentframe_slider = WidgetSlider(int2{ 1,1 }, int2{ glwin.res.x - 200 - 2,20-2 }, glwin, currentframe, int2{ 0,(int)frames.size() - 1 }, "Frame ");

		auto &frame = frames[currentframe];
		if (prevframe != currentframe)
		{
			if(needsaving&1)
				needsaving |=3;   // flip the next bit so we no longer clear this flag if we just undo the current frame
			if(!simulating)
				htk.handmodel.SetPose(frame.pose);
			htk.update_cnn_model(frame.depth);
			prevframe = currentframe;
		}

		auto &cnn_input = htk.cnn_input;
		auto &cnn_output = htk.cnn_output;
		auto &cnn_output_analysis = htk.cnn_output_analysis;
		auto cnn_labels = GatherHandExpectedCNN(frame.pose, camsub(cnn_input.cam, 4)); // used for making 16x16 heatmaps 

		std::vector<float3> pts = PointCloud(frame.depth, { drange.x*0.5f,drange.y*1.25f });
		
		if (!glwin.MouseState)
		{
			spinview = false;
			selectrb = NULL;
		}

		Widget mainview(int2{ 0 + 1 ,20 + 1 }, int2{ glwin.res.x - 200 - 2,glwin.res.y - 20 - 140 - 2 - showhelp*glwin.font_char_dims.y*(2+(int)(sizeof(helptext)/sizeof(*helptext)))},glwin);
		if(mainview.inview()) // if (gui.inview(&selectrb))
		{
			auto ray = qrot(camera.orientation, normalize(mainview.mousevec()));
			float3 v1 = camera.position + ray *100.0f;
			if (!selectrb && !mainview.focus()) for (auto &rb : htk.handmodel.rigidbodies)
			{
				if (auto h = ConvexHitCheck(Planes(rb.shapes[0].verts, rb.shapes[0].tris), rb.pose(), camera.position, v1))
				{
					v1 = h.impact;
					selectrb = &rb;
					spoint = h.impact;
					rbpoint = rb.pose().inverse()*h.impact;
				}
			}
			spoint = camera.position + ray * length(spoint - camera.position)*powf(1.02f, (float)glwin.mousewheel);
		}
		else
			selectrb = nullptr;

		if(mainview.focus() && (!selectrb || !simulating)) 
		{
			auto trackball = qconj(camera.orientation);
			spinview = true;
			trackball = normalize(qmul(mainview.trackball() , trackball)); 
			camera.orientation = qconj(trackball);
			camera.position = float3(0, 0, focuszoffset) + qzdir(camera.orientation)*length(camera.position - float3(0, 0, focuszoffset));
			camera.position.z -= focuszoffset;
			camera.position *= powf(1.1f, (float)glwin.mousewheel);
			glwin.mousewheel = 0;
			camera.position.z += focuszoffset;
		}
		if (simulating && pts.size())
		{
			htk.microforce = pow(cloudforce, 2.0f);   // using a parabolic scale to adjust easier for smaller values 
			htk.slowfit(pts, hold, refpose, 3, selectrb, spoint, rbpoint,  (nnenable > 0.0f) ? cnn_output_analysis.crays : std::vector<float4>{});
			frames[currentframe].pose = htk.handmodel.GetPose();
		}

		auto clearcolor = float3(0.2f*simulating , 0.1f,  0.20f * !simulating) + colorflash; 
		colorflash = { 0,0,0 };  // used for user feedback for one time key press events

		// Drawing Section 
		glClearColor(clearcolor.x, clearcolor.y, clearcolor.z, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
		glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();

		currentframe_slider.Draw();

		mainview.DrawBegin();
		clearcolor *= 0.75f;
		glClearColor(clearcolor.x, clearcolor.y, clearcolor.z, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT );

		mainview.perspective();
		glMultMatrixf(camera.inverse().matrix());

		{
			glLineWidth(1.0f);
			glwirebox(spoint - float3(0.005f), spoint + float3(0.005f));
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			if(nnenable) for (auto &v : cnn_output_analysis.crays)
			{
				glLineWidth((simulating&& v.w > 1.0f - nnenable) ? 3.0f : 1.0f);
				glBegin(GL_LINES);
				glColor3fv(rainbow_colors[&v-&cnn_output_analysis.crays[0]]);
				for (int i : {0, 1})
					glVertex3fv(v.xyz()/dot(v.xyz(),qzdir(cnn_input.cam.pose.orientation))*drange[i]);
				glEnd();
			}
			glPopAttrib();
		}
		drawpoints(pts, { 0,  1.0f, 0.0f }, 1.0f);

		std::vector<float3> kmps;
		for (auto k : handmodelfeaturepoints)
			kmps.push_back(frame.pose[k.bone] * k.offset);
		drawpoints(kmps, { 1, 1, 0 }, 5.0f);

		{
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glLineWidth(1.0f + 3.0f*spinview);
			glColor3fv({ 0.50f, 0.50f, 0.60f });
			glwirefrustumz(frame.depth.cam.deprojectextents(), { 0.15f,std::max(drange.y,1.0f) });
			glPopAttrib();
		}

		{
			glPushMatrix();
			glColor3f(0.0f, 1.0f, 0.5f);
			glMultMatrixf(cnn_input.cam.pose.matrix());
			glwirefrustumz(cnn_input.cam.deprojectextents(), drange);
			glPopMatrix();
			glColor3f(1.0f, 1.0f, 1.0f);
		}

		// draw the hand model
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glEnable(GL_LIGHTING); glEnable(GL_LIGHT0); glColor3f(1, 1, 1);
		for (auto &m : htk.handmodel.GetMeshes())   // GetMeshes() member syncs to rigidbody poses
			MeshDraw(m);
		glPopAttrib();

		mainview.ortho();


		glwin.PrintString({ 0, -1 }, "%d / %d    %s:%d", currentframe, frames.size(), frame.fname.c_str(), frame.fid);

		mainview.DrawEnd();


		{
			auto depthmarkedimage = Transform(frame.depth, [&frame, drange](unsigned short d) {return byte3((unsigned char)clamp((int)(256 * (1.0f - (d*frame.depth.cam.depth_scale - drange.x) / (drange.y - drange.x))), 0, 255)); }); // shorttorgb_r(frame.depth.raster);
			for (auto k : frame.depth.cam.projectz(Skin(frame.pose, handmodelfeaturepoints)))
				for (auto p : rect_iteration({ 3,3 })) depthmarkedimage.pixel(clamp(int2(k) + p - int2(1, 1), int2(0, 0), frame.depth.cam.dim() - int2(1, 1))) = byte3(255, 0, 0);
			for (auto k : frame.depth.cam.projectz(Skin(htk.handmodel.GetPose(), handmodelfeaturepoints)))
				for (auto p : rect_iteration({ 3,3 })) depthmarkedimage.pixel(clamp(int2(k) + p - int2(1, 1), int2(0, 0), frame.depth.cam.dim() - int2(1, 1))) = byte3(255, 255, 0);
			drawimagevp(depthmarkedimage, { 0 + 1,glwin.res.y - 140 + 1 }, { 140*4/3 - 2,140 - 2 }, "depth data");
		}

		{
			auto imgds = Transform(cnn_input, [](float d)->byte3 { return byte3(ToGrayScale(d)); });//  .raster.data(), cnn_input.dim(), { 0.7f,0.48f }, { 0.2f, -0.2f });
			imgds = UpSample(imgds);
			for (unsigned int i = 0; i < handmodelfeaturepoints.size(); i++)
			{
				int2 p0 = linalg::clamp<int2, int2>(int2(imgds.cam.projectz(imgds.cam.pose.inverse()*Skin(htk.handmodel.GetPose().data(), handmodelfeaturepoints[i]))), { 0,0 }, imgds.dim() - int2{ 1,1 });
				int2 p1 = int2(cnn_output_analysis.image_points[i] * 4.0f*2.0f);   // scale since we upsampled the image
				for (int t = 0; t < 32; t++)
					imgds.pixel(p0 + (p1 - p0)*t / 31) = byte3(rainbow_colors[i] * 255.0f);
			}
			drawimagevp(imgds, { 140 * 4 / 3 + 1,glwin.res.y-140 + 1 }, { 140 - 2,140 - 2 }, "depth segment");
		}

		{
			auto imgir = ToRGB(Sample(frame.ir, cnn_input.cam));
			for (unsigned int i = 0; i < handmodelfeaturepoints.size(); i++)
				imgir.pixel(linalg::clamp<int2, int2>(int2(imgir.cam.projectz(imgir.cam.pose.inverse()*Skin(htk.handmodel.GetPose().data(), handmodelfeaturepoints[i]))), { 0,0 }, imgir.dim() - int2{ 1,1 })) = byte3(rainbow_colors[i] * 255.0f);
			drawimagevp(imgir, { 140 * 7 / 3 + 1,glwin.res.y - 140 + 1 }, { 140 - 2,140 - 2 },"ir segment");//  .raster.data(), cnn_input.dim(), { 0.7f,0.48f }, { 0.2f, -0.2f });
		}
		int panelx = 140 * 10 / 3, panelw = glwin.res.x - panelx - 100 * 2;
		nnenable = std::max(0.0f,nnenable-0.02f);  // let this feature drift toward 'off', its just for demo purposes
		WidgetButton({ panelx              + 6 , glwin.res.y - 30 + 7 }, { panelw/4 -12, 30 - 14}, glwin, "[z] undo frame ", [&]() { glwin.keyboardfunc('z'   ,0,0);}, needsaving&1   ).Draw();
		WidgetButton({ panelx + panelw*1/4 + 6 , glwin.res.y - 30 + 7 }, { panelw/4 -12, 30 - 14}, glwin, "[ctl-z]undo all", [&]() { glwin.keyboardfunc('\x1A',0,0);},(needsaving&2)>0).Draw();
		WidgetButton({ panelx + panelw / 2 + 6 , glwin.res.y - 30 + 7 }, { panelw/4 -12, 30 - 14}, glwin, "[bksp] delete  ", [&]() { glwin.keyboardfunc('\b'  ,0,0);}, frames.size()>1).Draw();
		WidgetButton({ panelx + panelw*3/4 + 6 , glwin.res.y - 30 + 7 }, { panelw/4 -12, 30 - 14}, glwin, "[ctl-s] save   ", [&]() { glwin.keyboardfunc('\x13',0,0);}, needsaving>0   ).Draw();
		WidgetSwitch({ panelx              + 4 , glwin.res.y - 60 + 3 }, { panelw/2 - 8, 30 - 6 }, glwin, simulating, "[s] simulating      " , [&](){ needsaving = true;}).Draw();
		WidgetSlider({ panelx + panelw*2/4 + 3 , glwin.res.y - 60 + 4 }, { panelw/4 - 6, 30 - 8 }, glwin, cloudforce, float2{ 0,2 }, "[u] fit speed ").Draw();
		WidgetSlider({ panelx + panelw*3/4 + 3 , glwin.res.y - 60 + 4 }, { panelw/4 - 6, 30 - 8 }, glwin, nnenable  , float2{ 0,1 }, "[n] use cnn ").Draw();
		WidgetSwitch({ panelx              + 1 , glwin.res.y - 90 + 3 }, { panelw/2 - 2, 30 - 6 }, glwin, hold      , "[h] hold/lock joints" , [&](){ refpose = htk.handmodel.GetPose();}).Draw();
		WidgetSwitch({ panelx + panelw / 2 + 1 , glwin.res.y - 90 + 3 }, { panelw/2 - 2, 30 - 6 }, glwin, tiefingers, "[y] tie pinkyringmid" ).Draw();
		
		auto lbl_heatmaps = VisualizeHMaps(cnn_labels.hmaps, cnn_input);
		auto cnn_heatmaps = VisualizeHMaps(cnn_output_analysis.hmaps, cnn_input); 
		auto lbl_angles = ToRGB(UpSample(UpSample(UpSample(cnn_labels.vmap))));
		auto cnn_angles = ToRGB(UpSample(UpSample(UpSample(ToGrayScale(cnn_output_analysis.vmap)))));
		drawimagevp(cnn_angles  , { glwin.res.x - 2 * 100 + 1,  0 + 1 }, { 100-2,100-2 }              , " \n \n \n \n cnn\n angles");
		drawimagevp(cnn_heatmaps, { glwin.res.x - 2 * 100 + 1,100 + 1 }, { 100-2,glwin.res.y - 100 -2}, " cnn out");
		drawimagevp(lbl_angles  , { glwin.res.x - 1 * 100 + 1,  0 + 1 }, { 100-2,100-2 }              , " \n \n \n \n actual\n angles");
		drawimagevp(lbl_heatmaps, { glwin.res.x - 1 * 100 + 1,100 + 1 }, { 100-2,glwin.res.y - 100 -2}, " labels ");
		glViewport(0, 0, glwin.res.x, glwin.res.y-140);

		int line_y = 0;
		glwin.PrintString({ 0, line_y++ }, "Mode:  %s   use [?] to turn %s help", simulating ? "Re-Fitting" : "Inspection", showhelp ? "off" : "on");
		if (showhelp       ) for (auto s : helptext) glwin.PrintString({ 0, line_y++ }, s);
		if (!showhelp      ) glwin.PrintString({ 0, line_y++ }, "fit error %f", FitError(htk.handmodel, takesubsample(pts), frame.depth));
		if (hold           ) glwin.PrintString({ 0, line_y++ }, "relative hand pose currently fixed  [h] to disable");
		if (tiefingers     ) glwin.PrintString({ 0, line_y++ }, "fingers pinky ring and middle constrined together  hit [y] to disable");
		if (needsaving     ) glwin.PrintString({ 0, line_y++ }, "*File Modified - [CTRL-S] to Save  %s", needsaving == 1 ? "  or z to undo this frame" : needsaving<4 ? " or ctrl-z to undo all frame edits" : "  frames deleted (exit program to undo)");
		if (nnenable       ) glwin.PrintString({ 0, line_y++ }, "%s [n]etwork constrains: %3.1f  %s", nnenable ? "using" : "showing", nnenable, nnenable >= 0.5f ? "  use [n] to turn off, or [N] to decrement by 0.1" : "");
		if (!cloudforce    ) glwin.PrintString({ 0 ,line_y++ }, "point cloud constraints currently disabled, use key [u] to reenable");

		glMatrixMode(GL_PROJECTION); glPopMatrix();
		glMatrixMode(GL_MODELVIEW); glPopMatrix();  //leave things in modelview mode
		glPopAttrib();
		glwin.SwapBuffers();
	}
	return 0;
}
catch (const char *c)
{
	std::cerr << c << "\n";
	MessageBoxA(GetActiveWindow(), c, "FAIL", 0); //throw(std::exception(c));
}
catch (const std::exception & e)
{
	std::cerr << e.what() << "\n";
	MessageBoxA(GetActiveWindow(), e.what(), "FAIL", 0);
}
