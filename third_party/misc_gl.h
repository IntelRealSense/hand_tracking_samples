//
// please include either glwin.h or glfwwin.h before including this module
//

#pragma once
#ifndef MISC_GL_H
#define MISC_GL_H

#include "geometric.h"
#include "mesh.h"
#include <assert.h>

void drawimage(const byte3* imagedata, int2 dim, float2 p, float2 s, int tid = 0)
{
	if (!imagedata)
		return;
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glBindTexture(GL_TEXTURE_2D, tid);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dim.x, dim.y, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);

	glBegin(GL_QUADS);
	glColor3f(1, 1, 1);
	float2 corners[] = { { 0.0f, 0.0f },{ 1.0f,0.0f },{ 1.0f,1.0f },{ 0.0f,1.0f } };
	for (float2 c : corners)
		glTexCoord2f(c.x, c.y), glVertex3fv(float3(p + c*s, 0.0f));
	glEnd();


	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glPopAttrib();
}
inline void drawimage(const std::vector<byte3> image, int2 dim, float2 p, float2 s, int tid = 0) { drawimage(image.data(), dim, p, s, tid); }
inline void drawimage(const std::vector<unsigned char> image, int2 dim, float2 p, float2 s, int tid = 0) { drawimage(Transform(image, [](unsigned char c) {return byte3(c);}), dim, p, s, tid); }
void drawimage(const std::pair<const std::vector<byte3>&, int2> im, float2 p, float2 s, int tid = 0) { drawimage(im.first, im.second, p, s, tid); }
void drawimage(const std::pair<const std::vector<unsigned char>&, int2> im, float2 p, float2 s, int tid = 0) { drawimage(im.first, im.second, p, s, tid); }

void progressbar(float t, bool highlight)
{
	float2 position(0.0f, 0.0f), size(1.0f, 0.05f), gap(0.01f, 0.01f);
	byte3 boost = highlight ? byte3(0, 50, 50) : byte3(0, 0, 0);
	std::vector<byte3> image(1000, byte3(boost+ byte3( 0, 103, 103 ) ));  // init to something not red
	for (int i = 0; i < (int)(t * 1000) && i < 1000; i++)
		image[i] = byte3(byte3(255, 52, 52) + boost);  // make the portion left of time t more red
	drawimage(image.data(), { 1000,1 }, position + gap, size - gap*2.0f);
}


inline void drawpoints(const std::vector<float3> &pts, float3 color = float3(1, 1, 1), float size = 1.0f)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	if (size != 1.0f) glPointSize(size);
	glBegin(GL_POINTS);
	glColor3fv(color);
	for (auto &p : pts)
		glVertex3fv(p);
	glEnd();
	glPopAttrib();
}

inline std::vector<std::pair<float3, float3>> wirefrustrum(float2 fov, float zmin, float zmax)  // generates the 12 lines of the cropped pyramid (eg view-volume frustrum) specified by field of view and range
{
	float3 cnr = { tanf(fov.y / 2.0f), tanf(fov.y / 2.0f), 1.0f };
	std::vector<std::pair<float3, float3>> lines;
	float3 generator[] = { { 1, 1, 1 },{ -1, 1, 1 },{ -1, -1, 1 },{ 1, -1, 1 } };  // ccw
	for (int i = 0; i < 4; i++)
	{
		lines.push_back(std::pair<float3, float3>((generator[i]* cnr)*zmin, (generator[i]* cnr)*zmax));
		for (float z : {zmin, zmax})
			lines.push_back(std::pair<float3, float3>((generator[i]* cnr)*z, (generator[(i + 1) % 4]* cnr)*z));
	}
	return lines;
}

inline void drawlines(const std::vector<std::pair<float3, float3>> &lines, float3 color = { 1, 1, 1 }, float linewidth = 1.0f)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glLineWidth(linewidth);
	glBegin(GL_LINES);
	glColor3fv(color);
	for (auto l : lines)
		glVertex3fv(l.first), glVertex3fv(l.second);
	glEnd();
	glLineWidth(1.0f);  // pop attrib should take care of this
	glPopAttrib();
}
void drawfrustrum(float2 fov, float zmin, float zmax, float linewidth = 1.0f) { drawlines(wirefrustrum(fov, zmin, zmax), { 1, 1, 1 }, linewidth); }

inline void glGridxy(float r, float3 c = { 0, 1, 0 }, const Pose &pose = { { 0,0,0 },{ 0,0,0,1 } })
{
	glPushMatrix();
	glMultMatrixf(pose.matrix());
	glColor3fv(c);
	glBegin(GL_LINES);
	glColor3fv({ 0.25f, 0.25f, 0.25f });
	for (float t = -4; t <= 4; t += 1.0f)
	{
		glVertex3fv(float3(t, -4.0f, 0)*r / 4.0f); glVertex3fv(float3(t, 4.0f, 0)*r / 4.0f);
		glVertex3fv(float3(-4.0f, t, 0)*r / 4.0f); glVertex3fv(float3(4.0f, t, 0)*r / 4.0f);
	}
	glEnd();
	glPopMatrix();
}

void gldraw(const std::vector<float3> &verts, const std::vector<int3> &tris)
{
	glBegin(GL_TRIANGLES);
	glColor4f(1, 1, 1, 0.25f);
	for (auto t : tris)
	{
		auto n = TriNormal(verts[t[0]], verts[t[1]], verts[t[2]]);
		glNormal3fv(n); auto vn = abs(n);
		int k = argmax(&vn.x, 3);
		for (int j = 0; j < 3; j++)
		{
			const auto &v = verts[t[j]];
			glTexCoord2f(v[(k + 1) % 3], v[(k + 2) % 3]);
			glVertex3fv(v);
		}
	}
	glEnd();
}

void gldraw(const std::vector<float3> &verts, const std::vector<int3> &tris,const std::vector<int> &color)  // palette color 
{
	float3 rainbow_colors[] = { { 0.75f,0.5f,0.5f },{ 0.5f,0.75f,0.5f },{ 0.5f,0.5f,0.75f },{ 1,0,0 },{ 0,1,0 },{ 0,0,1 },{ 1,1,0 },{ 1,0,1 } };
	glBegin(GL_TRIANGLES);
	glColor4f(1, 1, 1, 0.25f);
	for (auto t : tris)
	{
		auto n = TriNormal(verts[t[0]], verts[t[1]], verts[t[2]]);
		glNormal3fv(n); auto vn = abs(n);
		int k = argmax(&vn.x, 3);
		for (int j = 0; j < 3; j++)
		{
			const auto &v = verts[t[j]];
			glTexCoord2f(v[(k + 1) % 3], v[(k + 2) % 3]);
			glColor3fv(rainbow_colors[color[t[j]] % 8]);
			glVertex3fv(v);
		}
	}
	glEnd();
}

inline void MeshDraw(const Mesh &mesh)  
{
	glPushMatrix();
	glMultMatrixf(mesh.pose.matrix());
	auto emitvert = [](const Vertex &v) ->void {glNormal3fv(qzdir(v.orientation)); glTexCoord2fv(v.texcoord); glVertex3fv(v.position); };
	glColor4fv(mesh.hack);
	glBegin(GL_TRIANGLES);
	for (auto t : mesh.tris) for (int i = 0; i < 3; i++)
		emitvert(mesh.verts[t[i]]);
	glEnd();
	glPopMatrix();
}
inline void MeshDrawWire(const Mesh &mesh)
{
	glPushMatrix();
	glMultMatrixf(mesh.pose.matrix());
	auto emitvert = [](const Vertex &v) ->void {glNormal3fv(qzdir(v.orientation)); glTexCoord2fv(v.texcoord); glVertex3fv(v.position); };
	glBegin(GL_LINES);
	for (auto t : mesh.tris) for (int i : {0, 1, 1, 2, 2, 0})
		emitvert(mesh.verts[t[i]]);
	glEnd();
	glPopMatrix();
}
inline void glcolorbox(const float3 &r, const Pose &p)   // p = { { 0, 0, 0 }, { 0, 0, 0, 1 } })
{
	glPushMatrix();
	glMultMatrixf(p.matrix());
	glBegin(GL_QUADS);
	for (int m : {0, 1}) for (int i : {0, 1, 2})
	{
		int i1 = (i + 1 + m) % 3;
		int i2 = (i + 2 - m) % 3;
		float3 u, v, w;
		u[i1] = r[i1];
		v[i2] = r[i2];
		w[i] = (m) ? -1.0f : 1.0f;
		float3 a((float)m, (float)m, (float)m);
		a[i] = 1 - a[i];
		glColor3fv(a);
		glNormal3fv(w);
		float2 corners[] = { { -1.0f, -1.0f },{ 1.0f, -1.0f },{ 1.0f, 1.0f },{ -1.0f, 1.0f } };  // ccw order
		for (float2 t : corners)
			glTexCoord2fv(t), glVertex3fv(w*r[i] + u*t.x + v*t.y);
	}
	glEnd();
	glPopMatrix();
}

inline void glwirebox(std::function<float3(int)> p) { glBegin(GL_LINES); for (auto e : boxedges()) glVertex3fv(p(e[0])), glVertex3fv(p(e[1])); glEnd(); }
inline void glwirebox(std::vector<float3> verts) { assert(verts.size() == 8); glwirebox([&verts](int i) {return verts[i]; }); }
inline void glwirebox(const float3 &bmin, const float3 &bmax) { glwirebox([bmin, bmax](int i) {return bmin + (bmax - bmin)*float3((float)(i & 1), (float)(i>>1&1), (float)(i>>2&1)); }); }
inline void glwirefrustumz(const float2x2 &corners, const float2 &zrange) { glwirebox([&](int i)->float3 {return float3(corners[i & 1][0], corners[(i >> 1) & 1][1], 1.0f)*zrange[(i >> 2) & 1]; }); }
inline void glwirefrustumz(float2 cmin, float2 cmax, float2 zrange) { return glwirefrustumz(float2x2(cmin, cmax), zrange); }

struct VertexPC
{
	float3 position;
	float4 color; // rgba
};
std::vector<VertexPC> ColorVerts(const std::vector<float3> points, float4 c=float4(1.0f)) { return Transform(points, [c](float3 p)->VertexPC{return{p,c};});}
std::vector<VertexPC> ColorVerts(const std::vector<float3> points, float3 c) { return ColorVerts(points, float4(c, 1.0f)); } 
struct SegmentPC
{
	VertexPC endpoints[2];
	SegmentPC() {}
	SegmentPC(VertexPC a, VertexPC b) :endpoints{ a,b } {}
	SegmentPC(float3 a, float3 b, float4 c = float4(1.0f)) :endpoints{ {a,c},{b,c} } {}
};
std::vector<SegmentPC> ColorSegments(const std::vector<std::pair<float3, float3>> &lines, float4 c = float4(1.0f))
{
	return Transform(lines, [c](std::pair<float3, float3> e) {return SegmentPC(e.first, e.second, c); });
}
std::vector<SegmentPC> ColorSegments(const std::vector<float3> &points, float4 c = float4(1.0f))
{
	std::vector<SegmentPC> lines;
	for (unsigned int i = 0; i < points.size()-1; i+=2)
		lines.push_back({ points[i],points[i + 1],c });
	return lines;
}

struct render_scene
{
	render_scene()
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
		int2x2 viewport;  // current viewport settings  [[x y][w h]] 
		glGetIntegerv(GL_VIEWPORT, (GLint*)&viewport);
		glPerspective(60.0f, (float)viewport[1].x/(float)viewport[1].y, 0.01f, 50.0f);
		glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
	}
	render_scene(const Pose &camera) :render_scene() { glMultMatrixf(camera.inverse().matrix()); }
	render_scene(const Pose &camera,const std::vector<Mesh*> &meshes, const std::vector<SegmentPC> &lines = {}, const std::vector<VertexPC> &points = {}) :render_scene(camera)
	{
		glBegin(GL_LINES);
		for (auto &line : lines) for (auto v : line.endpoints)
			glColor3fv(v.color.xyz()), glVertex3fv(v.position);
		glEnd();
		glBegin(GL_POINTS);
		for (auto v : points)
			glColor3fv(v.color.xyz()), glVertex3fv(v.position);
		glEnd();
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glEnable(GL_COLOR_MATERIAL);
		glColor3f(1, 1, 1);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		for (auto m : meshes)
			MeshDraw(*m);
		glPopAttrib();
	}
	~render_scene()
	{
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		glPopAttrib();
	}
};


//
// an minimal opengl based 2d gui widget set 
//
struct Widget
{
	int2   pos;
	int2   dims;
	GLWin& glwin;  // should be able to make this const
	bool   isdrawing;
	int2   togl(int2 p) { return int2(p.x, glwin.res.y - p.y - 1) - pos; }
	int2   mouse() { return togl(glwin.mousepos); }
	int2   mouse_prev() { return togl(glwin.mousepos_previous); }
	int2   mouse_lastup() { return togl(glwin.mousepos_lastup); }
	float2 mousef() { return float2(mouse()) / float2(dims); }
	float2 mousef_prev() { return float2(mouse_prev()) / float2(dims); }
	bool   focus() { return within_range(mouse_lastup(), { 0,0 }, dims) && glwin.MouseState && glwin.focus; }
	bool   inview() { return focus() || (within_range(mouse(), { 0,0 }, dims) && glwin.focus); }
	float3 deproject(float2 p, float viewangle = 0) { p = (p - float2(0.5f, 0.5f))* float2(2.0f*dims.x / dims.y, 2.0f); float fl = tan((viewangle ? viewangle : glwin.ViewAngle) / 2.0f*3.14f / 180.0f); return normalize(float3(p* fl, -1.0f)); }
	float3 mousevec(float viewangle = 0.0f) { return deproject(mousef(), viewangle); }
	float3 mousevec_prev(float viewangle = 0.0f) { return deproject(mousef_prev(), viewangle); }
	float4 trackball() { return quat_from_to(deproject(mousef_prev()) + float3(0, 0, 2), deproject(mousef()) + float3(0, 0, 2)); }

	Widget(int2 pos, int2 dims, GLWin &glwin) :pos(pos), dims(dims), glwin(glwin), isdrawing(false) {}
	virtual void Render() {}
	void DrawBegin()
	{
		isdrawing = true;
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glViewport(pos.x, pos.y, dims.x, dims.y);
		glEnable(GL_SCISSOR_TEST);
		glScissor(pos.x, pos.y, dims.x, dims.y);
		glMatrixMode(GL_PROJECTION); glPushMatrix();
		glMatrixMode(GL_MODELVIEW); glPushMatrix();
		glLoadIdentity();
	}
	void DrawEnd()
	{
		glMatrixMode(GL_PROJECTION); glPopMatrix();
		glMatrixMode(GL_MODELVIEW); glPopMatrix();
		glViewport(0, 0, glwin.res.x, glwin.res.y);
		glPopAttrib();
		isdrawing = false;
	}
	void Draw()
	{
		DrawBegin();
		Render();
		DrawEnd();
	}
	void ortho()
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}
	void perspective(float viewangle)
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glPerspective(viewangle, (float)dims.x / dims.y, 0.01f, 10.0f);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}
	void perspective() { perspective(glwin.ViewAngle); }
	//Widget(int2x2 xywh) :Widget(xywh[0], xywh[1]) {}
};

template<class T = int>
struct WidgetSlider_ : public Widget
{
	T &value;
	linalg::vec<T, 2> range;
	std::string label;
	int is_int(int) { return 1; }
	int is_int(float) { return 0; }
	int is_int() { return is_int(value); }
	WidgetSlider_(int2 pos, int2 dims, GLWin &glwin, T &value, const linalg::vec<T, 2> range, std::string label = "") :Widget(pos, dims, glwin), value(value), range(range), label(label)
	{
		if (range.y > range.x && focus())
			value = clamp((T)(mousef().x * (range.y - range.x + is_int())) + range.x, range.x, range.y);
	}
	void drawbar(float t, float3 c0, float3 c1)
	{
		ortho();
		glBegin(GL_QUADS);
		float3 bg(0.2f);
		glColor3fv(bg); glVertex3f(0, 1.00f, -0.6f); glVertex3f(0, 0.00f, -0.6f); glVertex3f(1, 0.00f, -0.6f); glVertex3f(1, 1.00f, -0.6f);
		glColor3fv(c0); glVertex3f(0, 0.90f, -0.5f); glVertex3f(0, 0.10f, -0.5f); glVertex3f(t, 0.10f, -0.5f); glVertex3f(t, 0.90f, -0.5f);
		glColor3fv(c1); glVertex3f(t, 0.90f, -0.5f); glVertex3f(t, 0.10f, -0.5f); glVertex3f(1, 0.10f, -0.5f); glVertex3f(1, 0.90f, -0.5f);
		glColor3f(1, 1, 1);
		glEnd();
	}
	virtual void Render()
	{
		drawbar((float)(value - range.x) / (range.y - range.x), float3(1, 0, inview()*0.5f), float3(0, 1, inview()*0.5f));
		glwin.PrintStringP({ 4,dims.y / 2 - glwin.font_char_dims.y / 2 }, is_int() ? "%s%d/%d" : "%s%5.3f/%5.3f", label.c_str(), value, range.y);
	}
};

template<class T>
inline WidgetSlider_<T> WidgetSlider(int2 pos, int2 dims, GLWin &glwin, T &value, const linalg::vec<T, 2> range, std::string label = "")
{
	return WidgetSlider_<T>(pos, dims, glwin, value, range, label);      // this wrapper lets code below avoid explicit template arguments
}

struct WidgetSwitch : public Widget
{
	bool &value;
	std::string label;
	WidgetSwitch(int2 pos, int2 dims, GLWin &glwin, bool &value, std::string label = "", std::function<void()> callback = []() {return;}) :Widget(pos, dims, glwin), value(value), label(label)
	{
		if (focus() && glwin.downevent)
		{
			value = !value;
			callback();
		}
	}
	virtual void Render()
	{
		ortho();
		glBegin(GL_QUADS);
		float3 bg(0.1f + 0.2f*inview());
		float w = (float)dims.y / (float)dims.x;
		glColor3fv(bg);   glVertex3f(0, 1.00f, -0.6f); glVertex3f(0, 0.00f, -0.6f); glVertex3f(1, 0.00f, -0.6f); glVertex3f(1, 1.00f, -0.6f);
		glEnd();

		glMatrixMode(GL_PROJECTION);glLoadIdentity();
		glOrtho(0.0, (float)dims.x, 0.0, (float)dims.y, -1.0, 1.0);
		glMatrixMode(GL_MODELVIEW);
		glBegin(GL_QUADS);
		float3 c0(0, 0, 1), c1(0, 0, 0);c1[value] = 1.0f;
		int x = value * dims.y;
		glColor3fv(c0);   glVertex3f(0.0f + 2, 0.0f + dims.y - 2, -0.5f); glVertex3f(0.0f + 2, 0.0f + 2, -0.5f); glVertex3f(0.0f + dims.y * 2 - 2, 0.0f + 2, -0.5f); glVertex3f(0.0f + dims.y * 2 - 2, 0.0f + dims.y - 2, -0.5f);
		glColor3fv(c1);   glVertex3f(0.0f + x + 4, 0.0f + dims.y - 4, -0.4f); glVertex3f(0.0f + x + 4, 0.0f + 4, -0.4f); glVertex3f(0.0f + x + dims.y - 4, 0.0f + 4, -0.4f); glVertex3f(0.0f + x + dims.y - 4, 0.0f + dims.y - 4, -0.4f);
		glColor3f(1, 1, 1);
		glEnd();
		glwin.PrintStringP({ dims.y * 2 + 2,dims.y / 2 - glwin.font_char_dims.y / 2 }, "%s =%d", label.c_str(), value);
	}
};
struct WidgetButton : public Widget
{
	std::string label;
	bool active;
	WidgetButton(int2 pos, int2 dims, GLWin &glwin, std::string label = "", std::function<void()> callback = []() {return;}, bool active = true) :Widget(pos, dims, glwin), label(label), active(active)
	{
		if (focus() && glwin.downevent && active)
		{
			callback();
		}
	}
	virtual void Render()
	{
		ortho();
		glBegin(GL_QUADS);
		float3 bg = active ? float3(0.3f) + float3(0.7f*glwin.downevent*inview(), 0.7f*focus(), 0.1f + 0.4f*inview()) : float3(0.2f);
		glColor3fv(bg);   glVertex3f(0, 1.00f, -0.6f); glVertex3f(0, 0.00f, -0.6f); glVertex3f(1, 0.00f, -0.6f); glVertex3f(1, 1.00f, -0.6f);
		glEnd();
		glColor3fv(float3(active ? 1.0f : 0.4f));
		glwin.PrintStringP({ 2,dims.y / 2 - glwin.font_char_dims.y / 2 }, "%s", label.c_str());
		glColor3f(1, 1, 1);
	}
};



#endif // MISC_GL_H
