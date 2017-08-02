//
//   glwin.h
//
// minimal opengl on windows framework for writing small opengl test apps
// all in one header file.  No extra downloads, installs, cmake, .DLLs, .LIBs. etc...
// avoid using 16 bit wchar.
//


#if USE_GLFW || !defined(WIN32) && !defined(_WIN32) && !defined(Win64) && !defined(_Win64)
#include "glfwwin.h"
// prevent inclusion of any of the windows specific implementation below
#define GLWIN_H
#endif


#pragma once
#ifndef GLWIN_H


#define GLWIN_H

#include <functional>
#include <vector>

#include "mswin.h"
#include "../third_party/stb_easy_font.h"

#include <cstring>
#include <cstdarg>   // For va_list, va_start, ...
#include <cstdio>    // For vsnprintf
#include <GL/gl.h>
#ifdef WIN32
#pragma comment(lib,"opengl32.lib")
#endif


inline void glVertex3fv  (const float3   &v) { glVertex3fv(&v.x);        }
inline void glNormal3fv  (const float3   &v) { glNormal3fv(&v.x);        }
inline void glTexCoord2fv(const float2   &v) { glTexCoord2fv(&v.x);      }
inline void glColor3fv   (const float3   &c) { glColor3fv(&c.x);         }
inline void glColor4fv   (const float4   &c) { glColor4fv(&c.x);         }
inline void glTranslatefv(const float3   &v) { glTranslatef(v.x,v.y,v.z);}
inline void glMultMatrixf(const float4x4 &m) { glMultMatrixf(&m.x.x);    }
inline void glPerspective(float fovy, float aspect, float n, float f) { double y = n*std::tan(fovy / 2 * acos(-1) / 180), x = y*aspect;glFrustum(-x, x, -y, y, n, f); }



class GLWin : public MSWin
{

	HWND CreateOpenGLWindow(const char* title)  // make a double-buffered, rgba, opengl window
	{
		MSWin::CreateMSWindow(title, res);
		hDC = GetDC(hWnd);

		/* there is no guarantee that the contents of the stack that become
		   the pfd are zeroed, therefore _make sure_ to clear these bits. */
		PIXELFORMATDESCRIPTOR pfd;
		memset(&pfd, 0, sizeof(pfd));
		pfd.nSize        = sizeof(pfd);
		pfd.nVersion     = 1;
		pfd.dwFlags      = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
		pfd.iPixelType   = PFD_TYPE_RGBA;
		pfd.cDepthBits   = 32;
		pfd.cColorBits   = 32;

		int pf = ChoosePixelFormat(hDC, &pfd);
		if(pf == 0) 
			throw("ChoosePixelFormat() failed:  Cannot find a suitable pixel format."); 
		if (SetPixelFormat(hDC, pf, &pfd) == FALSE) 
			throw( "SetPixelFormat() failed: Cannot set format specified.");
 
		ReleaseDC(hWnd,hDC);
		RegisterTouchWindow(hWnd, 0);
		return hWnd;
	}    
	void WinReshape(int width, int height)
	{
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
	    wglMakeCurrent(NULL, NULL);
		ReleaseDC(hWnd,hDC);
		wglDeleteContext(hRC);
		DestroyWindow(hWnd);
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
				glOrtho (0.0, viewport[1].x,viewport[1].y, 0.0,  -1.0, 1.0);   // need to invert y for stb font
				//void print_string(float x, float y, char *text, float r, float g, float b)
					{
						static char buffer[99999]; // ~500 chars
						int num_quads;

						num_quads = stb_easy_font_print(p.x*viewport[1].x, (1.0f-p.y)*viewport[1].y-7, (char*)s, NULL, buffer, sizeof(buffer));

						//glColor3f(r, g, b);
						glEnableClientState(GL_VERTEX_ARRAY);
						glVertexPointer(2, GL_FLOAT, 16, buffer);
						glDrawArrays(GL_QUADS, 0, num_quads * 4);
						glDisableClientState(GL_VERTEX_ARRAY);
					}
					//glRasterPos2f(p.x,p.y);
			  glPopMatrix();
			glMatrixMode(GL_MODELVIEW); glPopMatrix();
		glPopAttrib();
	}
public:
	const int2 font_char_dims = { 8,12 };  // for stb font estimated size   note 10,14 was used for the legacy redbook font 
	float2 CoordTexToPix(int2 cp,int len=0)  // assuming our font dims given text character "terminal" coordinates, derive opengl floating point location [0..1]
	{
		int2x2 viewport;  // current viewport settings  [[x y][w h]] 
		glGetIntegerv(GL_VIEWPORT, (GLint*)&viewport);
		int rows = std::max(1,viewport[1].y/font_char_dims.y) ;
		int cols = viewport[1].x/font_char_dims.x ;
		if(cp.y>=rows){ cp.y=rows-1;}
		if(cp.y<0) { cp.y+= rows;} // caller gives a negative y
		if(cp.y<0) { cp.y = 0;} // caller gives a too much negative y
		if(cp.x<0) { cp.x = cols+cp.x-len+1;}
		if (cp.x + len>cols) { cp.x = cols - len; }
		if (cp.x<0) { cp.x = 0; }
		cp.y=rows-1-cp.y; // invert y so row 0 is at the top and -1 is at the bottom
		return float2((float)cp.x, (float)cp.y) / float2((float)cols, (float)rows);
	}
	void PrintString(int2 cp, const char *s, ...)
	{
		va_list args;
		va_start(args, s);
		char buffer[1024];
		vsnprintf_s<1024>(buffer, sizeof(buffer), s, args);
		va_end(args);
		PrintStringR(CoordTexToPix(cp, (int)strlen(s)), buffer );
	}
	void PrintStringP(int2 p, const char *s, ...)   // a bit of duplicate copy/paste code from above, blame the legacy varargs usage   todo fixme
	{
		va_list args;
		va_start(args, s);
		char buffer[1024];
		vsnprintf_s<1024>(buffer, sizeof(buffer), s, args);
		va_end(args);
		int2x2 viewport;  glGetIntegerv(GL_VIEWPORT, (GLint*)&viewport);
		PrintStringR(float2(p)/float2(viewport[1]), buffer);
	}

	HDC     hDC;              // device context 
    HGLRC   hRC;              // opengl context 

	GLWin(const char *title, int w = 512, int h = 512) : MSWin(title, { w,h }) 
	{
		reshape = [this](int x, int y) {this->res = { x,y }; this->WinReshape(x, y); };
		hWnd = CreateOpenGLWindow(title);
		if (hWnd == NULL) throw("failed to create opengl window");

		hDC = GetDC(hWnd);
		hRC = wglCreateContext(hDC);
		wglMakeCurrent(hDC, hRC);
		ShowWindow(hWnd, SW_SHOWDEFAULT);
		UpdateWindow( hWnd );
		glEnable(GL_DEPTH_TEST);
	}
	~GLWin()
	{
		DestroyOpenGLWindow();
	}

	bool SwapBuffers() { return  (::SwapBuffers(hDC)!=0); }

	static LRESULT WINAPI MsgProcG( HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam )
	{
		if(msg==WM_NCCREATE)
			SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)(reinterpret_cast<CREATESTRUCT *>(lParam)->lpCreateParams));  // grab my pointer passed into createwindow
		auto glwin = reinterpret_cast<GLWin *>(GetWindowLongPtr(hWnd, GWLP_USERDATA));
		return (glwin)? glwin->MsgProc(hWnd, msg, wParam, lParam ) : DefWindowProc( hWnd, msg, wParam, lParam );
	}
};

#endif // GLWIN_H


