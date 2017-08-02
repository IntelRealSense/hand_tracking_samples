//
// Small collection of generic image utility routines needed for this project
//
// The image functions are designed to also work with cameras or depth cameras.
// In particular, if we resize or subsample an image we also want the resulting camera intrinsics 
// and, in some cases, the resulting camera pose relative to the source image (extrinsics).
// Additionally this project works with CNNs that output 1D and 2D heatmaps.   
// Therefore there are some support routines to generate/interpret these.
// 

#pragma once
#ifndef MISC_IMAGE_H
#define MISC_IMAGE_H

#include <iostream>
#include <fstream>
#include <string>
#include <exception>
#include <cfloat>

#include "../third_party/geometric.h"


inline bool FallsWithinRange(float t, float2 r) { return t >= r.x && t < r.y; }

// DCamera
// A generic depth camera interface.
// Class has no dependencies on external library
//
class DCamera 
{
	int2                             dim_       = { 320, 240 };                  // width , height
	float2                           focal_     = { 241.811768f, 241.811768f };  // focal length 1 pixel = 1 unit
	float2                           principal_ = { 162.830505f, 118.740089f };
public:
	float                            depth_scale = 0.001f;
	float                            get_depth_scale() { return depth_scale; }
	Pose                             pose;
	int2&                            dim()             { return dim_; }
	const int2&                      dim()       const { return dim_; }
	float2&                          focal()           { return focal_; } 
	const float2&                    focal()     const { return focal_; }
	float2&                          principal()       { return principal_; }
	const float2&                    principal() const { return principal_; }
	DCamera() {}
	DCamera(int2 dim, float2 focal, float2 principal,float depth_scale,Pose pose=Pose()) :dim_(dim), focal_(focal), principal_(principal),depth_scale(depth_scale),pose(pose) {}
	DCamera(int2 dim) :dim_(dim), focal_(asfloat2(dim)), principal_(asfloat2(dim)/2.0f) {}
	float3   deprojectz(const float2 &p, float d)       const { return float3((p.x - principal().x) / focal().x, (p.y - principal().y) / focal().y, 1.0f) * d; }
	float3   deprojectz(const int2 &p,unsigned short d) const { return deprojectz(asfloat2(p), (float)d); }
	float2   projectz(const float3 &v)                  const { return v.xy() / v.z * focal() + principal(); }
	std::vector<float2> projectz(const std::vector<float3> &pts) const { return Transform(pts, [&](const float3 &v) {return this->projectz(v); }); }  // array version of projectz
	float2x2 deprojectextents()                         const { return float2x2(deprojectz(float2(0, 0), 1.0f).xy(), deprojectz(asfloat2(dim()), 1.0f).xy()); }  // upper left and lower right xy corners at plane z==1
	float2 fov() const { return{ atan2(principal().x + 0.5f, focal().x) + atan2(dim().x - principal().x - 0.5f, focal().x), atan2(principal().y + 0.5f, focal().y) + atan2(dim().y - principal().y - 0.5f, focal().y) }; }   // using ds4 conventions with that 0.5 offset
	template<class F> friend void visit_fields(DCamera & cam, F f);
};
inline std::ostream & operator << (std::ostream & out, const DCamera& dcam) { return out << dcam.dim() << " " << dcam.focal() << "  " << dcam.principal() << " " << dcam.depth_scale; }
template<class F> void visit_fields(DCamera & cam, F f) { f("dims", cam.dim_); f("focal", cam.focal_); f("principal", cam.principal_); f("depth_scale", cam.depth_scale); }

inline DCamera camcrop(const DCamera &c, int2 offset, int2 dim) { return{ dim, c.focal(), c.principal() - float2((float)offset.x, (float)offset.y) ,c.depth_scale, c.pose}; }
inline DCamera camsub   (const DCamera &c, int s) { return{ c.dim() / s, c.focal() / (float)s, c.principal() / (float)s ,c.depth_scale , c.pose}; }
inline DCamera operator*(const DCamera &c, int s) { return{ c.dim() * s, c.focal() * (float)s, c.principal() * (float)s ,c.depth_scale , c.pose}; }
inline DCamera operator/(const DCamera &c, int s) { return{ c.dim() / s, c.focal() / (float)s, c.principal() / (float)s ,c.depth_scale , c.pose}; }

//-----------------


// some image routines defined just using std::vector<> since sometimes thats useful
//

template<class T> std::vector<T> SubImage(const T *src, int stride, int2 dim)
{
	std::vector<T> dst(dim.x*dim.y);
	for (int y = 0; y < dim.y; y++) for (int x = 0; x < dim.x; x++)
		dst[y*dim.x + x] = src[y*stride + x];
	return dst;
}
template<class T> std::vector<T> SubImage(const std::vector<T> &src, int2 srcdim, int2 dim, int2 offset) { return SubImage(src.data() + offset.x + offset.y*srcdim.x, srcdim.x, dim); }
template<class T> std::vector<T> SubImage(const std::vector<T> &src, const DCamera &sc, const DCamera &dc) { return SubImage(src, sc.dim(), dc.dim(), asint2(sc.principal() - dc.principal())); }


template<class T, class F>
std::vector<T> DownSample(F f, const std::vector<T> &src, int2 sdim)  // max,min,plus 2x2 subsampling
{
	int2 ddim = sdim / 2;
	std::vector<T> dst(ddim.x*ddim.y);
	auto s = [&src, &sdim](int2 p) { return src[p.y*sdim.x + p.x]; };
	for (int2 p(0, 0); p.y < sdim.y; p.y += 2) for (p.x = 0; p.x < sdim.x; p.x += 2)
		dst[p.y / 2 * ddim.x + p.x / 2] = f(f(s(p + int2(0, 0)), s(p + int2(1, 0))), f(s(p + int2(0, 1)), s(p + int2(1, 1))));
	return dst;
}
template<class T> std::vector<T> DownSampleAvg(const std::vector<T> &src, int2 dim) { return DownSample([](T a, T b) {return T((a+b)/2);        }, src, dim); }
template<class T> std::vector<T> DownSampleMin(const std::vector<T> &src, int2 dim) { return DownSample([](T a, T b) {return std::min<T>(a, b); }, src, dim); }
template<class T> std::vector<T> DownSampleMax(const std::vector<T> &src, int2 dim) { return DownSample([](T a, T b) {return std::max<T>(a, b); }, src, dim); }
template<class T> std::vector<T> DownSampleFst(const std::vector<T> &src, int2 dim) { return DownSample([](T a, T  ) {return a;                 }, src, dim); }

template<class T> std::vector<T> UpSample(const std::vector<T> &src, int2 sdim)
{
	std::vector<T> dst(src.size() * 4);
	for (int2 p(0, 0); p.y < sdim.y * 2; p.y++) for (p.x = 0; p.x < sdim.x * 2; p.x++)
		dst[p.y*sdim.x * 2 + p.x] = src[p.y / 2 * sdim.x + p.x / 2];
	return dst;
}

//
// Image 
// contains a raster image as a std::vector of whatever type T
// has an associated camera the specifies intrinsics for the image which includes dimensions (width and height) 
//
template <class T> struct Image
{
	DCamera cam;          // camera intrinsics including width,height in pixels of image 
	std::vector<T> raster;

	int2&      dim()               { return cam.dim(); }
	const int2 dim() const         { return cam.dim(); }
	T&         pixel(int2 p)       { return raster[p.y*dim().x + p.x]; }
	const T&   pixel(int2 p) const { return raster[p.y*dim().x + p.x]; }
	T&         pixel_clamp(int2 p)       { p=clamp(p,int2(0),dim()-int2(1)); return raster[p.y*dim().x + p.x]; }
	const T&   pixel_clamp(int2 p) const { p=clamp(p,int2(0),dim()-int2(1)); return raster[p.y*dim().x + p.x]; }
	Image() {}
	Image(int2 dim    ) :cam(dim), raster(product(dim)) {}
	Image(int2 dim,T t) :cam(dim), raster(product(dim),t) {}
	Image(DCamera cam)  :cam(cam), raster(cam.dim().x*cam.dim().y,T(0)) {}
	Image(DCamera cam, std::vector<T> data) :cam(cam), raster(std::move(data))  {}
	Image(int2    dim, std::vector<T> data) :cam(dim), raster(std::move(data))  {}
	Image(int2    dim, const T *data) :cam(dim), raster(data,data+dim.x*dim.y) {}
	operator std::pair<const std::vector<T>&, int2>() const { return { raster,dim() }; }
	void plotsafe(int2 p, T v) { if (within_range(p, { 0,0 }, dim() - int2(1))) pixel(p) = v; }
};
template<class T> Image<T> Crop(const Image<T> &src, int2 offset, int2 dim)
{
	return {DCamera(dim, src.cam.focal(), src.cam.principal() - asfloat2(offset),src.cam.depth_scale), SubImage(src.raster, src.dim(), dim, offset) };
}

template <class T> Image<T> UpSample     (const Image<T> &src) { return { src.cam * 2 , UpSample     (src.raster,src.dim()) }; }
template <class T> Image<T> DownSampleFst(const Image<T> &src) { return { src.cam / 2 , DownSampleFst(src.raster,src.dim()) }; }
template <class T> Image<T> DownSampleMin(const Image<T> &src) { return { src.cam / 2 , DownSampleMin(src.raster,src.dim()) }; }
template <class T> Image<T> DownSampleMax(const Image<T> &src) { return { src.cam / 2 , DownSampleMax(src.raster,src.dim()) }; }
template <class T> Image<T> DownSampleAvg(const Image<T> &src) { return { src.cam / 2 , DownSampleAvg(src.raster,src.dim()) }; }



template <class T> Image<T> Sample(const Image<T> &src, const DCamera &dstcam, T background = 0)  // uses point sampling
{
	Image<T> dst(dstcam);
	int2 pp;
	for (auto p : rect_iteration(dst.dim())) // for (int y = 0; y < dst.dim().y; y++) for (int x = 0; x < dst.dim().x; x++)
		dst.pixel(p) = within_range(pp = asint2(src.cam.projectz(dst.cam.pose*dst.cam.deprojectz(asfloat2(p), 1.f))), { 0,0 }, src.dim() - int2(1, 1)) ? src.pixel(pp) : background;
	return dst;
}

// Note that for depth image, the pixel values are distance from the image plane.
//  because the sampling may include a rotated camera, we here adjust the distance (sampled value) to be distance from the destination camera's image plane
template <class T> Image<T> SampleD(const Image<T> &src, const DCamera &dstcam, T background = 0)  // uses point sampling
{
	Image<T> dst(dstcam);
	float3 ppdir = dstcam.pose * dstcam.deprojectz(dstcam.principal(), 1.0f);
	int2 pp;
	for (auto p : rect_iteration(dst.dim())) // for (int y = 0; y < dst.dim().y; y++) for (int x = 0; x < dst.dim().x; x++)
		dst.pixel(p) = within_range(pp = asint2(src.cam.projectz(dst.cam.pose*dst.cam.deprojectz(asfloat2(p), 1.f))), { 0,0 }, src.dim() - int2(1, 1)) ? (T)dot(ppdir,src.cam.deprojectz(pp,src.pixel(pp))) : background;
	return dst;
}


template<typename F, typename S> auto Transform(const Image<S> &src, F f) ->  Image<decltype (f(S()))> { std::vector<decltype (f(S()))>  dst(src.raster.size()); std::transform(src.raster.begin(), src.raster.end(), dst.begin(), f); return Image<decltype (f(S()))>(src.cam, dst); }


// using this 'grayscale' for float to/from byte representations with range 0 to 1 for float and 0 to 255 for byte representations  
inline unsigned char ToGrayScale(float  x) { return (unsigned char)clamp(x*255.0f, 0.0f, 255.0f); }
inline unsigned char ToGrayScale(unsigned char x) { return x; }
inline float         GrayScaleToFloat(unsigned char c) { return c / 255.0f; }
template<class T> Image<unsigned char> ToGrayScale(const Image<T> &src) { return Transform(src, [](T x) {return ToGrayScale(x); }); }
Image<byte3>         ToRGB(const Image<unsigned char> &src) { return Transform(src, [](unsigned char p) {return byte3(p, p, p); }); }
Image<byte3>         ToRGB(const Image<float >        &src) { return ToRGB(ToGrayScale(src)); }




inline Image<unsigned char> Threshold(const Image<unsigned short> &depthdata, std::function<bool(unsigned short)> f)
{
	return Transform(depthdata, [&](unsigned short d)->unsigned char {return (f(d)) ? 255 : 0; });
}
inline Image<unsigned char> DistanceTransform(Image<unsigned char> image)  // just manhattan
{
	int2 dim = image.dim();
	auto cm = [dim](int2 p) {return clamp(p, { 0,0 }, dim - int2(1, 1)); };
	for(auto p: rect_iteration(dim))
		image.pixel(p) = (unsigned char)std::min(255, std::min(std::min(image.pixel(cm(p - int2(1,0))) + 1, image.pixel(cm(p-int2(0,1))) + 1), image.pixel(p) + 0));
	for (auto r : rect_iteration(dim))
	{
		int2 p = dim - int2(1, 1) - r ;
		image.pixel(p) = (unsigned char)std::min(255, std::min(std::min(image.pixel(cm(p + int2(1, 0))) + 1, image.pixel(cm(p + int2(0, 1))) + 1), image.pixel(p) + 0));
	}
	return image;
}
template<class T> Image<T> SetBorder(Image<T> image, T v = T(0))
{
	int2 dim = image.dim();
	for (int y = 0; y < dim.y; y++)
		image.pixel({ 0,y }) = image.pixel({ dim.x - 1, y }) = v;
	for (int x = 0; x < dim.x; x++)
		image.pixel({ x,0 }) = image.pixel({ x, dim.y - 1 }) = v;
	return image;
}


template<class T> inline std::vector<T> concat(const std::vector<const std::vector<T>*> &arrays)
{
	std::vector<T> data;
	for (auto &a : arrays)
		Append(data, *a);
	return data;
}
template<class T> inline std::vector<T> concat(std::vector<T> a, const std::vector<T> &b)
{
	Append(a, b);
	return a;
}

template<class T> inline std::vector<T> concat(const std::vector<Image<T>> &images)
{
	return concat(Transform(images, [&](const Image<T> &image)->const std::vector<T>* { return &image.raster; }));    //  todo: check about returning references 
}

template<class T> inline Image<T> ImageConcat(const std::vector<Image<T>> &images)  // all need to be of same width dim.x
{
	int2 dim(0, 0);
	for (auto &image : images)
		dim = int2(std::max(dim.x, image.dim().x), dim.y + image.dim().y);  // max of widths, sum of heights
	Image<T> dst(dim);
	T* d = dst.raster.data();
	for (auto &image : images)
	{
		for (const T* s = image.raster.data(); s < image.raster.data() + image.raster.size(); s += image.dim().x, d += dim.x)
			memcpy(d, s, sizeof(T)*image.dim().x);
	}
	return dst;
}


//
// heatmaps   
// Support routines for creating heatmap images  and  locating peaks given a heatmap images
// It is assumed that the volume, or sum of all heatmap pixels, is 1.0.   
// CNNs that use softmax layers will provide such output.
//

inline auto NormalizeHeatMap(Image<unsigned char> image)  // so sum of all pixels adds up to 255  ignoring rounding error
{
	int sum = 0;
	for (auto c : image.raster)
		sum += c;
	if (sum) for (auto &c : image.raster)
		c = c * 255 / sum;
	// just for debugging: testsum = 0; for (auto t : image.raster)testsum += t;  std::cout << "hmap rendered " << " testsum: " << testsum << std::endl;
	return image;
}

inline Image<unsigned char> RenderHeatMap(float2 peak, const DCamera &hcam)
{
	Image<unsigned char> hmap(hcam);
	int2 hp((int)peak.x, (int)peak.y);
	for (int2 p(0, std::max(0, hp.y - 2)); p.y < std::min(hcam.dim().y, hp.y + 3); p.y++)
		for (p.x = std::max(0, hp.x - 2); p.x < std::min(hcam.dim().x, hp.x + 3); p.x++)
		{
			float d2 = length2(peak - float2(p));
			hmap.pixel(p) = ToGrayScale(std::exp(-d2 / (2.0f*0.33f))); // standard deviation 0.33 pixels  (clamp(1.0f - d2*0.5f)); //  gaussian 
		}
	return NormalizeHeatMap(hmap);
}


inline std::vector<Image<unsigned char>> RenderHeatMaps(const std::vector<float2> &featurepointlocations, const DCamera &hcam)
{
	return Transform(featurepointlocations, [&hcam](float2 p) {return RenderHeatMap(p, hcam); });
}


inline Image<unsigned char> Render1DHeatMaps(const std::vector<float> &values, int width)
{
	Image<unsigned char> htensor(int2{ width,(int)values.size() }, (unsigned char)0);  // probably using using 8x8 here 
	for (int y = 0; y < (int)values.size(); y++)
	{
		float v = values[y] * (float)(width - 1);
		int sum = 0;
		for (int x = std::max(0, (int)v - 2); x < std::min(width, (int)v + 3); x++)
		{
			float d2 = pow((float)x - v, 2.0f);
			sum += htensor.pixel({ x,y }) = ToGrayScale(std::exp(-d2 / (2.0f*0.5f))); // standard deviation 0.5 pixels  (clamp(1.0f - d2*0.5f)); //  gaussian 
		}
		for (int x = std::max(0, (int)v - 2); sum && x < std::min(width, (int)v + 3); x++)   // normalize
			htensor.pixel({ x,y }) = htensor.pixel({ x,y }) * 255 / sum;
	}
	return htensor;
}


template <class T> int2 ImageFindMax(const T *image, int2 dim, std::function<bool(const T& a, const T&b)> f = [](const T& a, const T&b) {return a > b; })
{
	int2 best(0, 0);
	for (int2 p(0, 0); p.y < dim.y; p.y++) for (p.x = 0; p.x < dim.x; p.x++)
		if (f(image[dot(p, { 1, dim.x })], image[dot(best, { 1,dim.x })]))
			best = p;
	return best;
}
template <class T> int2 ImageFindMax(const Image<T> &image, std::function<bool(const T& a, const T&b)> f = [](const T& a, const T&b) {return a > b; })
{
	return ImageFindMax(image.raster.data(), image.dim(), f);
}


template<class T>
inline float2 PeakSubPixel(const T* image, int2 dim, int2 p, int r = 1)  // due to possibility of multiple peaks just sample local area to approx gaussian fit
{
	float wsum = 0.0f;
	float2 v(0, 0);
	for (int2 s(0, std::max(0, p.y - r)); s.y < std::min(dim.y, p.y + r + 1); s.y++)
		for (s.x = std::max(0, p.x - r); s.x < std::min(dim.x, p.x + r + 1); s.x++)
		{
			auto w = (float)image[s.y*dim.x + s.x];
			v += float2(s) * w;
			wsum += w;
		}
	return (wsum == 0) ? float2(p) : v / wsum;
}
template<class T> inline float2 PeakSubPixel(const Image<T> &image, int2 p, int r = 1) { return PeakSubPixel(image.raster.data(), image.dim(), p, r); }  // assumes 1 channel image!!!

template<class T> inline float PeakVolume(const Image<T> &image, float2 pf, int r = 1)
{
	int2 p(pf + float2(0.5f));
	float vol = 0.0f;
	for (int2 s(0, std::max(0, p.y - r)); s.y < std::min(image.dim().y, p.y + r + 1); s.y++)
		for (s.x = std::max(0, p.x - r); s.x < std::min(image.dim().x, p.x + r + 1); s.x++)
			vol += (float)image.pixel(s);  //  [s.y*dim.x + s.x];
	return vol;
}


template<class T>
inline float PeakSubPixel1D(const T* image, int size, int p, int r = 1)
{
	float v = 0.0f, wsum = 0.0f;
	for (int i = std::max(0, p - r); i < std::min(size, p + r + 1); i++)   // restrict to local
	{
		auto w = (float)image[i];
		v += float(i) * w;
		wsum += w;
	}
	return ((wsum == 0) ? float(p) : v / wsum) / (float)(size - 1);
}
template<class T> inline float PeakSubPixel1D(const Image<T> &image, int row, int p, int r = 1)  // assumes 1 channel image!!!
{
	return PeakSubPixel1D(image.raster.data() + row*image.dim().x, image.dim().x, p, r);
}

template<class T> inline float PeakVolume1D(const Image<T> &image, int row, float pf, int r = 1)  // not currently used
{
	int p = (int)(pf + 0.5f);
	float vol = 0.0f;
	for (int x = std::max(0, p - r); x < std::min(image.dim().x, p + r + 1); x++)
		vol += (float)image.pixel({ x, row });
	return vol;
}

inline std::vector<float> PeakWidths(const Image<float> &image, float lim = 0.10f)  // eg 80% range   not currently used
{
	std::vector<float> ranges;
	for (int row = 0; row < image.dim().y; row++)  // row is y
	{
		float prev = 0.0f, sum = 0.0f;
		int x = 0;
		while (x<image.dim().x && (sum += image.pixel({ x++,row }))  < lim)
		{
			prev = sum;
		}
		float start = x - 1.0f + (lim - prev) / (sum - prev);
		prev = sum;
		while (x<image.dim().x && (sum += image.pixel({ x++,row }))  < 1.0f - lim)
		{
			prev = sum;
		}
		float end = x - 1.0f + (1.0f - lim - prev) / (sum - prev);
		ranges.push_back((end - start) / (float)image.dim().x);
	}
	return ranges;

}

template<class T>
inline std::vector<float> Peaks1D(const Image<T> &image)
{
	std::vector<float> values;
	for (int row = 0; row < image.dim().y; row++)  // row is y
	{
		int p = (int)(std::max_element(&image.pixel({ 0,row }), &image.pixel({ 0, row }) + image.dim().x) - &image.pixel({ 0, row }));
		values.push_back(PeakSubPixel1D(image, row, p));
	}
	return values;
}

//
// Some 3D support routines.   
// Depth images be turned into point clouds or 3D meshes using the PointCloud or DepthMesh routines below. 
// The DS (stereo based) cameras work with mirrors to get some view of the back of an object.  
// Given the plane equation of a mirror in the scene the backside pointcloud contributions can be provided
// using some of the additional routines below. 
//

template<class T> std::vector<float3> PointCloud(const Image<T> &dimage, float2 filter_range)
{
	std::vector<float3> pointcloud;
	float d;
	for (int2 p : rect_iteration(dimage.dim()))  // p.y and p.x iterate over dimensions of image
		if (FallsWithinRange(d = (dimage.pixel(p)* dimage.cam.depth_scale), filter_range))
			pointcloud.push_back(dimage.cam.deprojectz(float2(p), d));
	return pointcloud;
}

template<class T> std::pair<std::vector<float3>, std::vector<int3> > DepthMesh(const Image<T> &dimage, float2 filter_range, float gaplimit = FLT_MAX, int skip = 1)
{
	std::vector<float3> verts;
	std::vector<int3> tris;
	Image<int> vmap(dimage.dim() / skip, -1);
	auto inrange = [&](int3 t) { for (int i : {0, 1, 2}) if (std::abs(verts[t[i]].z - verts[t[(i + 1) % 3]].z) > gaplimit)return false;return true;};
	float pixdepth;
	for (int2 ps : rect_iteration(dimage.dim() / skip))
	{
		int2 p, rv{ (ps.x&&vmap.pixel(ps - int2(1, 0)) != -1),(ps.y&&vmap.pixel(ps - int2(0, 1)) != -1) };
		for (int2 s : rect_iteration({ skip,skip })) if (FallsWithinRange(pixdepth = (dimage.pixel(p = ps*skip + s + rv*(int2(skip - 1) - s * 2))* dimage.cam.depth_scale), filter_range))
		{
			vmap.pixel(ps) = (int)verts.size();  // map to index of the vert we are about to add
			verts.push_back(dimage.cam.deprojectz(float2(p), pixdepth));
			break; // this breaks out of the small area loop, continues the full image
		}
	}
	for (int2 p : rect_iteration(vmap.dim() - int2(1, 1)))
	{
		int a = vmap.pixel(p), b = vmap.pixel(p + int2{ 0, 1 }), c = vmap.pixel(p + int2{ 1, 1 }), d = vmap.pixel(p + int2{ 1, 0 });  // ccw
		if (a >= 0 && c >= 0 && ((b >= 0 && inrange({ a,b,c })) || (d >= 0 && inrange({ c,d,a }))))
		{
			if (b >= 0 && inrange({ a,b,c })) tris.push_back(int3(a, b, c));
			if (d >= 0 && inrange({ c,d,a })) tris.push_back(int3(c, d, a));
		}
		else if (b >= 0 && d >= 0)
		{
			if (a >= 0 && inrange({ d,a,b })) tris.push_back(int3(d, a, b));
			if (c >= 0 && inrange({ b,c,d })) tris.push_back(int3(b, c, d));
		}
	}
	return{ verts,tris };
}


Image<unsigned short> ImageClip(Image<unsigned short> depth, float4 plane, unsigned short val)
{
	for (int2 p(0, 0);p.y < depth.dim().y;p.y++) for (p.x = 0;p.x < depth.dim().x;p.x++)
		if (dot(float4(depth.cam.deprojectz(float2(p), depth.pixel(p)*depth.cam.depth_scale), 1.0f), plane) < 0)
			depth.pixel(p) = val;
	return depth;
}

auto PlaneSplit(const std::vector<float3> &points, float4 plane, float epsilon = 0.02f)
{
	struct result { std::vector<float3> under, coplanar, over; };
	std::vector<float3> b[3]; // under, coplanar, over;
	for (auto p : points)
	{
		float pd = dot(float4(p, 1), plane);
		b[(pd > -epsilon) + (pd > epsilon)].push_back(p);
	}
	return result{ b[0], b[1], b[2] };
	//return make_tuple(move(b[0]), move(b[1]), move(b[2])); //  { under, coplanar, over };
}
std::vector<float3> Mirror(std::vector<float3> points, float4 plane)
{
	for (auto &p : points)
		p += plane.xyz()* (dot(float4(p, 1), plane)*-2.0f);
	return points;
}
auto MirrorPlaneSplit(const std::vector<float3> &points, float4 plane, float epsilon = 0.02f)
{
	auto r = PlaneSplit(points, plane, epsilon);
	r.under = Mirror(std::move(r.under), plane);
	return r;
}



#endif // MISC_IMAGE_H
