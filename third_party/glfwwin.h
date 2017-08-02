//
//   glfwwin.h
//
// glfwwin matches the glwin interface but uses the commonly available glfw3 lib
//

#pragma once
#ifndef GLFWWIN_H
#define GLFWWIN_H
#include "../third_party/librealsense/examples/third_party/glfw/include/GLFW/glfw3.h"

#include <functional>
#include <vector>
#include <assert.h>
#include <string>

#include "geometric.h"
#include "stb_easy_font.h"

#ifndef VERIFY
#define VERIFY (assert(0),throw(std::exception((std::string(__FILE__) + ":" + std::to_string(__LINE__)).c_str())),1)
#endif
#include <cstring>
#include <cstdarg>   // For va_list, va_start, ...
#include <cstdio>    // For vsnprintf
#ifdef _WIN32
#pragma comment(lib,"opengl32.lib")
#ifdef _DEBUG
#pragma comment(lib,"../third_party/librealsense/examples/third_party/glfw/msvc140/obj/Release-Win32/glfw3.lib")
#else
#pragma comment(lib,"../third_party/librealsense/examples/third_party/glfw/msvc140/obj/Release-Win32/glfw3.lib")
#endif
#endif

inline void glVertex3fv(const float3     &v) { glVertex3fv(&v.x); }
inline void glNormal3fv(const float3     &v) { glNormal3fv(&v.x); }
inline void glTexCoord2fv(const float2   &v) { glTexCoord2fv(&v.x); }
inline void glColor3fv(const float3      &c) { glColor3fv(&c.x); }
inline void glColor4fv(const float4      &c) { glColor4fv(&c.x); }
inline void glTranslatefv(const float3   &v) { glTranslatef(v.x, v.y, v.z); }
inline void glMultMatrixf(const float4x4 &m) { glMultMatrixf(&m.x.x); }
inline void glPerspective(float fovy, float aspect, float n, float f) { double y = n*std::tan(fovy / 2 * acos(-1) / 180), x = y*aspect;glFrustum(-x, x, -y, y, n, f); }

#define VK_END      GLFW_KEY_END
#define VK_DOWN     GLFW_KEY_DOWN
#define VK_LEFT     GLFW_KEY_LEFT
#define VK_RIGHT    GLFW_KEY_RIGHT
#define VK_HOME     GLFW_KEY_HOME
#define VK_UP       GLFW_KEY_UP

class GLFWWin 
{



	GLFWwindow* CreateOpenGLWindow(const char* title)  // make a double-buffered, rgba, opengl window
	{
		GLFWWin::CreateMSWindow(title, res);
		return hWnd;
	}

	void WinReshape(int width, int height)
	{
		glfwMakeContextCurrent(hWnd);

		res = { width,height };
		glViewport(0, 0, width, height);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glPerspective(ViewAngle, (float)width/height, 0.1f, 50.0f);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}
	void DestroyOpenGLWindow()
	{
		if(!hWnd)
			return; // already destroyed or otherwise non existent
		glfwMakeContextCurrent(hWnd);
		glfwDestroyWindow(hWnd);

		hWnd=NULL;
	}
	void PrintStringR(const float2 &p,const char *s)
	{
		if(!s) return;
		glPushAttrib(GL_LIGHTING_BIT|GL_DEPTH_BUFFER_BIT|GL_TEXTURE_BIT);
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
   			glDisable(GL_DEPTH_TEST);
			glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
			glMatrixMode(GL_PROJECTION);
			  glPushMatrix();
				glLoadIdentity();
				int2x2 viewport;  // current viewport settings  [[x y][w h]] 
				glGetIntegerv(GL_VIEWPORT, (GLint*)&viewport);
				//viewport[1] = viewport[1] * 2 / 3 ; // try to scale it up a bit here   hack
				glOrtho(0.0, viewport[1].x, viewport[1].y, 0.0, -1.0, 1.0);   // need to invert y for stb font
																			  //void print_string(float x, float y, char *text, float r, float g, float b)
				{
					static char buffer[99999]; // ~500 chars
					int num_quads;

					num_quads = stb_easy_font_print(p.x*viewport[1].x, (1.0f - p.y)*viewport[1].y - 7, (char*)s, NULL, buffer, sizeof(buffer));

					//glColor3f(r, g, b);
					glEnableClientState(GL_VERTEX_ARRAY);
					glVertexPointer(2, GL_FLOAT, 16, buffer);
					glDrawArrays(GL_QUADS, 0, num_quads * 4);
					glDisableClientState(GL_VERTEX_ARRAY);
				}

				glPopMatrix();
			glMatrixMode(GL_MODELVIEW); glPopMatrix();
		glPopAttrib();
	}
	public:
		GLFWwindow*             hWnd;
		int2                    res;
		int                     mousewheel;   // if and how much its been rolled up/down this frame
		int2                    mousepos;
		int2                    mousepos_previous;
		int2                    mousepos_lastup;
		float2                  dmouse = { 0,0 };
		float3                  MouseVector;      // 3D direction mouse points
		float3                  OldMouseVector;
		int                     MouseState;     // true iff left button down
		float 	                ViewAngle;
		bool                    downevent;
		bool                    centermouse = false;
		bool                    centered_last_frame = false;
		bool                    focus = true;

		std::function<void(int, int, int)> keyboardfunc;
		std::function<void(int)>           keydownfunc = [](int) {};  // because windows   
		std::function<void()>              preshutdown = []() {};
		std::function<void(int, int)>       reshape = [](int, int) {};

		float aspect_ratio() { return (float)res.x / (float)res.y; }
		void ComputeMouseVector()
		{
			OldMouseVector = MouseVector;
			float spread = (float)tan(ViewAngle / 2 * 3.14 / 180);
			float y = spread * ((res.y - mousepos.y) - res.y / 2.0f) / (res.y / 2.0f);
			float x = spread * (mousepos.x - res.x / 2.0f) / (res.y / 2.0f);
			MouseVector = normalize(float3(x, y, -1));
		}

		void CreateMSWindow(const char* title, int2 res, int2 win_position = { 5,25 })
		{
			hWnd = glfwCreateWindow(res.x, res.y, title, NULL, NULL);
			glfwSetWindowPos(hWnd, win_position.x, win_position.y);
			glfwSetKeyCallback(hWnd, &key_callback);
			glfwSetMouseButtonCallback(hWnd, &mouse_button_callback);
			glfwSetWindowSizeCallback(hWnd, &window_size_callback);
			glfwSetFramebufferSizeCallback(hWnd, &framebuffer_size_callback);
			glfwSetWindowFocusCallback(hWnd, &window_focus_callback);
			glfwSetScrollCallback(hWnd, &scroll_callback);
			glfwSetCursorPosCallback(hWnd, &cursor_position_callback);

			glfwSetWindowUserPointer(hWnd, this);
		}
		static void framebuffer_size_callback(GLFWwindow* window, int width, int height)
		{
			GLFWWin* win = (GLFWWin*)glfwGetWindowUserPointer(window);
			glfwMakeContextCurrent(win->hWnd);
			glViewport(0, 0, width, height);
		}
		static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
		{
			GLFWWin* win = (GLFWWin*)glfwGetWindowUserPointer(window);
			if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
				if (win->preshutdown)
					win->preshutdown();
				glfwSetWindowShouldClose(win->hWnd, (int)true);
			}
			double xpos, ypos;
			glfwGetCursorPos(window, &xpos, &ypos);
			win->mousepos.x = (int)xpos;
			win->mousepos.y = (int)ypos;
			if (key > 256) {
				if (win->keydownfunc && action == GLFW_PRESS)
					win->keydownfunc(key);
			} else {
				if (win->keyboardfunc && action == GLFW_PRESS)
					win->keyboardfunc(key, win->mousepos.x, win->mousepos.y); // to match glut's api, add the x and y.
			}
		}
		static void window_size_callback(GLFWwindow* window, int width, int height)
		{
			GLFWWin* win = (GLFWWin*)glfwGetWindowUserPointer(window);
			win->reshape(width, height);
		}
		static void window_focus_callback(GLFWwindow* window, int focused)
		{
			GLFWWin* win = (GLFWWin*)glfwGetWindowUserPointer(window);
			win->focus = focused ? true : false;
		}
		static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
		{
			GLFWWin* win = (GLFWWin*)glfwGetWindowUserPointer(window);
			win->mousewheel = (int)yoffset;
		}
		static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
		{
			GLFWWin* win = (GLFWWin*)glfwGetWindowUserPointer(window);

			auto fixnbits = [](int d) { return (d & 1 << 15) ? d - (1 << 16) : d; };
			if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
				double xpos, ypos;
				glfwGetCursorPos(window, &xpos, &ypos);
				win->mousepos.x = (int)xpos;
				win->mousepos.y = (int)ypos;
				win->mousepos_previous = win->mousepos;
				win->mousepos_lastup = win->mousepos;
				win->ComputeMouseVector();
				win->OldMouseVector = win->MouseVector; // for touch devices to avoid unwanted snappings
				win->downevent = 1;
				win->MouseState = 1;
			}
			else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
				double xpos, ypos;
				glfwGetCursorPos(window, &xpos, &ypos);
				win->mousepos.x = (int)xpos;
				win->mousepos.y = (int)ypos;
				win->ComputeMouseVector();
				win->MouseState = 0;
			}
		}
		static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
		{
			GLFWWin* win = (GLFWWin*)glfwGetWindowUserPointer(window);
			win->mousepos = { (int)xpos, (int)ypos };
			win->ComputeMouseVector();
		}
		bool WindowUp()
		{
			mousepos_previous = mousepos;
			mousewheel = 0;
			glfwPollEvents();
			if (!MouseState)
				mousepos_lastup = mousepos;
			return !glfwWindowShouldClose(hWnd);
		}
	const int2 font_char_dims = { 8,12 };  // for stb font estimated size   note 10,14 was used for the legacy redbook font 
	void PrintString(int2 cp, const char *s, ...)
	{
		va_list args;
		va_start(args, s);
		char buffer[1024];
		vsnprintf(buffer, sizeof(buffer), s, args);
		va_end(args);
		int2x2 viewport;  // current viewport settings  [[x y][w h]] 
		glGetIntegerv(GL_VIEWPORT, (GLint*)&viewport);
		int rows = std::max(1,viewport[1].y/font_char_dims.y) ;
		int cols = viewport[1].x/font_char_dims.x ;
		if(cp.y>=rows){ cp.y=rows-1;}
		if(cp.y<0) { cp.y+= rows;} // caller gives a negative y
		if(cp.y<0) { cp.y = 0;} // caller gives a too much negative y
		if(cp.x<0) { cp.x = cols+cp.x-(int)strlen(s)+1;}
		if (cp.x + (int)strlen(s)>cols) { cp.x = cols - (int)strlen(s); }
		if (cp.x<0) { cp.x = 0; }
		cp.y=rows-1-cp.y; // invert y so row 0 is at the top and -1 is at the bottom
		PrintStringR(float2((float)cp.x,(float)cp.y)/float2((float)cols, (float)rows), buffer);
	}

	void PrintStringP(int2 p, const char *s, ...)   // a bit of duplicate copy/paste code from above, blame the legacy varargs usage   todo fixme
	{
		va_list args;
		va_start(args, s);
		char buffer[1024];
		vsnprintf(buffer, sizeof(buffer), s, args);
		va_end(args);
		int2x2 viewport;  glGetIntegerv(GL_VIEWPORT, (GLint*)&viewport);
		PrintStringR(float2(p) / float2(viewport[1]), buffer);
	}

	GLFWWin(const char *title, int w = 512, int h = 512)
		:res(w,h), mousepos(0, 0), MouseState(0), mousewheel(0), ViewAngle(60.0f), reshape([this](int x, int y) {this->res = { x,y }; })//, keyboardfunc([](int, int, int){})
	{
		glfwInit();

		reshape = [this](int x, int y) {this->res = { x,y }; this->WinReshape(x, y); };
		hWnd = CreateOpenGLWindow(title);
		if (hWnd == NULL) throw("failed to create opengl window");
		glfwMakeContextCurrent(hWnd);

		glEnable(GL_DEPTH_TEST);
	}
	~GLFWWin()
	{
		DestroyOpenGLWindow();
	}

	bool SwapBuffers() {
		glfwSwapBuffers(hWnd);return true; }


};

// ensure compatability with other wrapper:
#define GLWin GLFWWin
inline bool GetActiveWindow() { return false; }
inline void MessageBox(bool, const char *err, const char *message, int)  // since a lot of windows apps will throw and use a messagebox to alert user 
{
	std::cerr << "--------------------------\n";
	std::cerr << err << std::endl << message << std::endl;
	std::cerr << "--------------------------\n";
}
inline void MessageBoxA(bool, const char *err, const char *message, int) 
{
	MessageBox(GetActiveWindow(), err, message, 0);
}
#endif // GLFWWIN_H
