//
// PhysModel - support routines for simulating physically articulated skeletal models and point cloud fitting
//
// Used widely in robotics simulations and video games, impulse based dynamics solvers have the ability to 
// deal with a variety of different types of constriants inluding collision, friction, joints, external forces, and so on.  
// The generality and extensibility of this aproach makes it easy to explore solutions for 3D computer vision 
// challenges such as tracking and pose estimation where we are dealing with articulated connected objects.
// In particular, depth cameras can provide partial position information (known point in space but unknown on the model).
// Expressing such information, along with all the other constraints of the model, as a linear complementarity problem can 
// lead to a plausable pose for a model.
//
// The functionality provided here is built on top of the non-retained-mode rigidbody-simulation-engine 
// found in the single-header-implementation physics.h. 
// A PhysModel object is a collection of rigidbodies with some joints connecting them, as well as some 
// graphics-api-agnostic support for rendering and the basic routines for adding joint and drive constraints. 
// To make it easier to modify bone geometry, we create rigibody shapes from low poly control cages using catmull-clark subdivision.   
// The json file format and, while we wait for c++20, the template visit_fields convention are used for serialization.
// As this code is intended to support depth camera based tracking applications, there are a collection of methods 
// for finding closest bone and adding point-plane correspondences as additional constraints for the solver in order
// to fit articulated models to a point-cloud.  
// 
// Note, the physics implementation is kept simple by RigidBody objects using their center of mass as their local origin.
// This is usually different than the coordinate frames used by the initial rig/model.  So the collection of bone transforms to
// use during rendering will depend on which set of geometry is being drawn.  
//

#pragma once
#ifndef PHYSMODEL_H
#define PHYSMODEL_H

#include <algorithm>
#include <map>
#include <assert.h>
#include <cmath>

#include "../third_party/misc_json.h"
#include "../third_party/physics.h"
#include "../third_party/hull.h"
#include "../third_party/mesh.h"
#include "../third_party/wingmesh.h"
#include "../third_party/misc.h"   //  for visit fields member function template


inline std::vector<float4> Planes(const std::vector<float3> &verts, const std::vector<int3> &tris) {
	std::vector<float4> planes; 
	for (auto &t : tris)
	{
		float4 p = PolyPlane({ verts[t[0]], verts[t[1]], verts[t[2]] });
		if (p != float4(0.0f))
			planes.push_back(p);
	}
	return planes;
}

template <class T> inline unsigned IndexOf(const std::vector<T> &a, const T& e) { return (int)(std::find(a.begin(), a.end(), e) - a.begin()); }


inline std::vector<float3> spatialsubsample(const std::vector<float3> &points, int fraction = 4)
{
	std::vector<float3> a;
	for (unsigned int i = 0; i < points.size(); i+=fraction)
		a.push_back(points[i]);
	return a;
}

template <unsigned int numVoxels = 2048>
inline std::vector<float3> voxelsubsample(const std::vector<float3> &points, float voxelSize = 0.01f, int minVoxelNum = 1)
{
	struct VoxelCandidate {
		int3 pos = { 0, 0, 0 };
		float3 sum = { 0, 0, 0 };
		int cnt = 0;
	};
	std::vector<float3> output;

	//std::vector<VoxelCandidate> cand(numVoxels);
	static VoxelCandidate cand[numVoxels];
	memset(&cand, 0, numVoxels * sizeof(VoxelCandidate));
	auto voxelMask = numVoxels - 1;
	auto iVS = 1.0f / voxelSize; //inverse voxel size
	//int3 hashCoeff = { 56599, 11399, 33359 };
	//int3 hashCoeff = { 3863,2029,49139 }; //good for mm
	int3 hashCoeff = {54851, 11909, 24781}; //good for m
	auto collisions = 0;
	auto avg_step = 0;
	for (const auto & pt : points) {
		int3 intPos(static_cast<unsigned int>(std::floor(pt.x*iVS)), static_cast<unsigned int>(std::floor(pt.y*iVS)), static_cast<unsigned int>(std::floor(pt.z*iVS)));
		//auto hb = hashCoeff * intPos;
		//unsigned int hash = hb.x ^ hb.y ^ hb.z;
		unsigned int hash = dot(hashCoeff, intPos);
		unsigned int i = 0;
		for (; i < numVoxels; i++) {
			auto & bucket = cand[(hash + i) & voxelMask];
			if (bucket.cnt == 0 || bucket.pos == intPos) { 
				bucket.pos = intPos;
				bucket.sum += pt;
				bucket.cnt++;
				avg_step += i;
				break;
			}
		}
		if (i == numVoxels) { //flush on collision
			auto & bucket = cand[hash & voxelMask];
			output.emplace_back(bucket.sum / static_cast<float>(bucket.cnt));
			bucket.cnt = 1;
			bucket.pos = intPos;
			bucket.sum = pt;
			collisions++;
		}
	}
	for (const auto &pt : cand)
	{
		if (pt.cnt >= minVoxelNum)
			output.emplace_back(pt.sum / static_cast<float>(pt.cnt));
	}
	//printf("%d %d %f \t", output.size(), collisions, avg_step / static_cast<float>(output.size()));
	return output;
}

inline std::vector<float3> takesubsample(const std::vector<float3> &points, int fractionOrMinVoxel = 4, int voxel = 0, float voxelSize = 0.0f)
{
	if (!voxel)
		return spatialsubsample(points, fractionOrMinVoxel);
	else
		return voxelsubsample(points, voxelSize, fractionOrMinVoxel);
}
inline float4 mostabove(std::vector<float4> &planes, const float3 &v)  // returns plane which v is above by the largest amount
{
	assert(planes.size());
	return *std::max_element(planes.begin(), planes.end(), [&v](const float4 &a, const float4 &b) {return (dot(a, float4(v, 1)) < dot(b, float4(v, 1))); });
}
inline float4 closest(RigidBody *rb, const float3 &w)  // which plane on rigidbody rb that world-space point w is closest to
{
	return rb->pose().TransformPlane(mostabove(rb->shapes[0].planes, rb->pose().inverse()*w));  // since convex (intersection of half spaces), plane that w is most above will be closest
}

inline std::pair<RigidBody*, float4> closest(const std::vector<RigidBody*> &rigidbodies, const float3 &v)   // reasonable approx if enough facets   
{
	float4 pmin = { 0, 0, 0, std::numeric_limits<float>::max() };  
	RigidBody *rbmin = NULL;
	for (auto rb : rigidbodies)
	{
		auto n = safenormalize(v - rb->position);
		float4 p(n, -dot(rb->position,n) - rb->radius_inner);  // might still need to double check all the math using the inner and outer radius to prune rigidbodies and consequently number of planes checked
		if (dot(p, float4(v, 1)) < dot(pmin, float4(v, 1)))
		{
			pmin = p;
			rbmin = rb;
		}
	}
	for (auto &rb : rigidbodies) 
	{
		if (length(v - rb->position)-rb->radius > dot(pmin, float4(v, 1)))
			continue;
		auto p = closest(rb, v);
		if (dot(p, float4(v, 1)) < dot(pmin, float4(v, 1)))  // seek to minimize distance of point v above the plane
		{
			pmin = p; rbmin = rb;
		}
	}
	return{ rbmin, pmin };
}
static bool g_cloud_directed=1;
inline LimitLinear CloudConstraint(const std::vector<RigidBody*> &rigidbodies, float3 v, float3 origin = { 0,0,0 })
{
	int k = 0;
	RigidBody *rb; float4 p;
	std::tie(rb, p) = closest(rigidbodies, v);
	::HitInfo h;
	if (g_cloud_directed&&dot(v-origin, p.xyz()) > 0 && (h = ConvexHitCheck(rb->shapes[0].planes, rb->pose(), origin, v)))    //  in case we are closer to backside and point is behind shape
		return  ConstrainAlongDirection(NULL, v, rb, rb->pose().inverse()*h.impact, normalize(v - origin), -1.0f, 1.0f);  // dont know where on surface so use plane through centroid - no extra pushback 
																														  // note could try impulse range [0.0..1.0] so only pushing (to be tested)
	return ConstrainAlongDirection(NULL, v, rb, rb->pose().inverse()*(v - p.xyz()*dot(p, float4(v, 1))), p.xyz(), -1.0f, 1.0f);
}
inline std::vector<LimitLinear> CloudConstraints(const std::vector<RigidBody*> &rigidbodies, const std::vector<float3> &points, float3 origin = { 0,0,0 })
{
	std::vector<LimitLinear> linears;
	for (auto v : points)
		linears.push_back(CloudConstraint(rigidbodies, v, origin));
	return linears;
}

inline float4 containing_plane(const std::vector<float3> &points, float3 outdir, float3 origin = { 0,0,0 }, float3 viewdir = { 0,0,1 })  // all points lie under plane   useful for dcam pointcloud boundary collision planes
{
	float3 best = viewdir - outdir;  // should be outdir*some_big_number
	best += origin;
	float3 tangent = cross(best, outdir);
	for (auto &p : points)
		if (dot(cross(best - origin, p - origin), tangent) > 0)
			best = p;
	float3 n = normalize(cross(tangent, best));
	return{ n,-dot(n,origin) };
}


void scale(Shape &shape, float s)
{
	for (auto &v : shape.verts)
		v *= s;
	for (auto &p : shape.planes)
		p.w *= s;
}
void scale(RigidBody &rb, float s)
{
	for (auto &shape : rb.shapes)
		scale(shape, s);
	rb.bmin *= s;
	rb.bmax *= s;
	rb.com  *= s;
	rb.radius *= s;
	rb.radius_inner *= s;
	rb.tensorinv_massless /= s*s;
	rb.Iinv /= s*s;
}
void scale(Mesh &m, float s)
{
	for (auto &v : m.verts)
		v.position *= s;
}

void Reset(RigidBody &rb)
{
	rb.position = rb.position_start;
	rb.orientation = rb.orientation_start;
	rb.linear_momentum = rb.angular_momentum = float3(0.0f);
}
bool hasnan(const float3 &v) { return std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z); }
bool hasnan(const float4 &v) { return std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z) || std::isnan(v.w); }   // fixme, this should be is templatable compressable
bool hasnan(const RigidBody &rb) { return (hasnan(rb.linear_momentum) || hasnan(rb.position) || hasnan(rb.angular_momentum) || hasnan(rb.orientation)); }

#ifndef VISITFLD
#define VISITFLD(V) f(#V,V) 
#endif 
float physics_weak_force = 0.4f;

struct PhysModel
{
	struct Joint
	{
		int    rbi0, rbi1;         // indices
		float3 p0, p1;             // attachment point local  - note this is before the center of mass adjustment 
		float3 rangemin, rangemax; // in degrees
		float4 jointframe;
		template<class F> void visit_fields(F f) { VISITFLD(rbi0);VISITFLD(rbi1);VISITFLD(p0);VISITFLD(p1);VISITFLD(rangemin);VISITFLD(rangemax);VISITFLD(jointframe); }
	};

	std::vector<RigidBody> rigidbodies;
	std::vector<Joint> joints;
	std::map<std::string, unsigned int> rbindex;
	std::vector<Mesh> meshes;
	std::vector<WingMesh> controlcages;
	std::vector<WingMesh> subdivs;
	std::vector<Mesh> sdmeshes;

	void dosubdiv()
	{
		subdivs = Transform(controlcages, [](WingMesh wm)        {return WingMeshSubDiv(WingMeshSubDiv(wm));});
		sdmeshes= Transform(subdivs     , [](const WingMesh &wm) {return MeshFlatShadeTex(wm.verts, wm.GenerateTris());});
	}
	void build_ignore_lists()
	{
		for (auto &joint : joints)
		{
			rigidbodies[joint.rbi0].ignore.push_back(&rigidbodies[joint.rbi1]);
			rigidbodies[joint.rbi1].ignore.push_back(&rigidbodies[joint.rbi0]);
		}
		for (auto &ja : joints) for (auto &jb : joints) if (ja.rbi0 == jb.rbi0 && ja.rbi1 != jb.rbi1)  // ignore siblings 
		{
			rigidbodies[ja.rbi1].ignore.push_back(&rigidbodies[jb.rbi1]);
			rigidbodies[jb.rbi1].ignore.push_back(&rigidbodies[ja.rbi1]);
		}
		for (auto &ja : joints) for (auto &jb : joints) if (ja.rbi1 == jb.rbi0)  // ignore grandparents 
		{
			rigidbodies[ja.rbi0].ignore.push_back(&rigidbodies[jb.rbi1]);
			rigidbodies[jb.rbi1].ignore.push_back(&rigidbodies[ja.rbi0]);
		}
	}

	struct ModelHitInfo : public ::HitInfo
	{
		PhysModel *model = NULL;
		int rb  = -1;
		Shape *shape = NULL;
		ModelHitInfo(::HitInfo h, PhysModel *model, int rb, Shape *shape) : ::HitInfo(h), model(model), rb(rb), shape(shape){}
		ModelHitInfo(){ }
	};
	ModelHitInfo HitCheck(const float3 &v0, const float3 &v1)
	{
		ModelHitInfo hitinfo = { { false, v1, float3(0, 0, 0) }, NULL, -1, NULL };
		assert(hitinfo.hit == false);
		for (auto &rb : rigidbodies) if (auto h = ConvexHitCheck(rb.shapes[0].planes, rb.pose(), v0, hitinfo.impact))
			hitinfo = ModelHitInfo(h, this, IndexOf(Addresses(rigidbodies),&rb), &rb.shapes[0]);
		return hitinfo;
	}
	std::vector<Mesh> &GetMeshes(int show_subdiv = 0)
	{
		for (unsigned int i = 0; i < rigidbodies.size(); i++)
			sdmeshes[i].pose = Pose{ rigidbodies[i].PositionUser(),rigidbodies[i].orientation };  // the subd meshes are in rig space
		for (unsigned int i = 0; i < rigidbodies.size(); i++)
			meshes[i].pose = rigidbodies[i].pose();                  // shape meshes in center-of-mass physics reference frame
		return (show_subdiv) ? sdmeshes : meshes ;
	}

	void scale(float s)
	{
		for (auto &m : meshes)
			::scale(m, s);
		for (auto &m : sdmeshes)
			::scale(m, s);
		for (auto &rb : rigidbodies)
			::scale(rb, s);
		for (auto &rb : rigidbodies)
			rb.position = rigidbodies[0].position + (rb.position - rigidbodies[0].position) * s;
		for (auto &joint : joints)
		{
			joint.p0 *= s;
			joint.p1 *= s;
		}
	}

	std::vector<LimitAngular>GetAngularConstraints()
	{
		std::vector<LimitAngular> angulars;
		for (auto const &joint : joints)
			Append(angulars, ConstrainAngularRange(&rigidbodies[joint.rbi0], &rigidbodies[joint.rbi1], joint.jointframe , joint.rangemin, joint.rangemax));
		return angulars;
	}
	std::vector<LimitLinear>GetLinearConstraints()
	{
		std::vector<LimitLinear>  linears;
		for (auto const &joint : joints)
			Append(linears, ConstrainPositionNailed(&rigidbodies[joint.rbi0], joint.p0- rigidbodies[joint.rbi0].com, &rigidbodies[joint.rbi1], joint.p1- rigidbodies[joint.rbi1].com));
		return linears;
	}
	void GenericUpdate()  // example to show how physical simulation is done for skeletal rig
	{
		auto linear = GetLinearConstraints();
		auto angular = GetAngularConstraints();
		PhysicsUpdate(Addresses(rigidbodies), linear, angular, std::vector<std::vector<float3> *>());
		SanityCheck();
	}

	struct Label { float3 p; int rbi; };
	
	void FitPointCloud(const std::vector<float3> &points, std::vector<LimitLinear> linears = {}, std::vector<LimitAngular> angulars = {},float microforce=1.0f)  // simplistic fit
	{
		auto setmaxforce_u = [this,microforce](std::vector<LimitLinear>&& linears) ->std::vector<LimitLinear> { for (auto &c : linears) c.forcelimit = float2(-1.0f,1.0f) *( (c.rb1 == &this->rigidbodies[0] || c.rb1 == &this->rigidbodies[1] || c.rb1 == &this->rigidbodies[2]) ? physics_weak_force: 1.0f)*microforce; return linears; };
		Append(linears, setmaxforce_u( CloudConstraints(Addresses(rigidbodies), points )));
		auto setmaxforce_s = [](LimitLinear c) ->LimitLinear { c.forcelimit = float2(-10.0f, 10.0f); return c; };
		Append(linears , GetLinearConstraints ());
		Append(angulars, GetAngularConstraints());
		// angulars.push_back(ConstrainConeAngle(NULL, float3(0, -1, 0), rigidbodies[0], { 0, 0, 1 }, 70.0f));  // orient object upward

		PhysicsUpdate(Addresses(rigidbodies), linears, angulars, {});
		SanityCheck();
	}
	void SetBonePoseHierarchyW(int bid,float4 qw) // sets a single bone orientation, preserving relative pose on updating children
	{
		float4 dq = qmul(qw, qconj(rigidbodies[bid].orientation));    // so that q_new == dq * q_old   all in world
		rigidbodies[bid].orientation = qw;
		for (int c = 0;c < (int)rigidbodies.size(); c++) 
			if(ParentIdx(c)==bid)
				SetBonePoseHierarchyW(c,qmul(dq,rigidbodies[c].orientation));

		FixOrientations();
		FixPositions(); // there will be redundancy here.
	}
	std::vector<LimitAngular> DrivePose(const std::vector<Pose> &pose, float maxtorque = 1.0f)
	{
		std::vector<LimitAngular> angulars;
		for (auto &j : joints)
			Append(angulars, ConstrainAngularDrive(&rigidbodies[j.rbi0], &rigidbodies[j.rbi1], qmul(qconj(pose[j.rbi0].orientation), pose[j.rbi1].orientation), maxtorque));  // maxtorque?
		return angulars;
	}
	std::vector<LimitAngular> DriveBasePose(float maxtorque = 1.0f)
	{
		std::vector<LimitAngular> angulars;
		for (auto &j : joints)
			Append(angulars, ConstrainAngularDrive(&rigidbodies[j.rbi0], &rigidbodies[j.rbi1], (j.jointframe), maxtorque));  // maxtorque?
		return angulars;
	}
	void DriveTest()
	{
		Pose p0({ 0, 0.25f, 0.50f }, normalize(float4(0.7f, 0, 0, 0.7f)));
		// auto pose = ArrayImport<Pose>(some_data);
		auto angulars = GetAngularConstraints();
		auto linears  = GetLinearConstraints();
		Append(angulars, DrivePose(std::vector<Pose>(rigidbodies.size()), 10000.0f));
		Append(angulars, ConstrainAngularDrive(NULL, &rigidbodies[0], p0.orientation, 200000.0f));
		Append(linears , ConstrainPositionNailed(NULL, p0.position, &rigidbodies[0], { 0, 0, 0 }));
		PhysicsUpdate(Addresses(rigidbodies), linears, angulars, std::vector<std::vector<float3> *>());
		SanityCheck();
	}
	void FixOrientations()
	{
		for (auto joint : joints)
		{
			auto angulars = ConstrainAngularRange(&rigidbodies[joint.rbi0], &rigidbodies[joint.rbi1], joint.jointframe, joint.rangemin, joint.rangemax);
			for (auto a : angulars)
				if(a.targetspin*a.maxtorque>0 || a.targetspin*a.mintorque>0 )
					rigidbodies[joint.rbi1].orientation = normalize(qmul(QuatFromAxisAngle(a.axis,a.targetspin*physics_deltaT) ,rigidbodies[joint.rbi1].orientation));
		}
	}
	void FixPositions()  // ordered (top-down) animate constraints
	{
		for (auto j : joints)
			rigidbodies[j.rbi1].position += rigidbodies[j.rbi0].PoseUser()*j.p0 - rigidbodies[j.rbi1].PoseUser()*j.p1;
	}
	int ParentIdx(int i) { int p = i; for (auto const &j : joints) if (j.rbi0 == i || j.rbi1 == i) p = std::min(p, std::min(j.rbi0, j.rbi1)); return p==i?-1:p; }  // returns -1 if no parent 
	std::vector<LimitAngular> RelativeAngularConstraints(const std::vector<Pose> &refpose,const std::vector<int2> &pairs)
	{
		std::vector<LimitAngular> angulars;
		for (auto p:pairs)  //  can think of p.x as parent and p.y as child
		{
			auto dq = (refpose[p.x].inverse()* refpose[p.y]).inverse() * rigidbodies[p.x].pose().inverse()* rigidbodies[p.y].pose();
			for (int a : {0, 1, 2}) // axes
				angulars.push_back(LimitAngular(&rigidbodies[p.x], &rigidbodies[p.y], qmat(rigidbodies[p.x].orientation)[a], -dq.orientation[a] * 2.0f / physics_deltaT));
		}
		return angulars;
	}
	std::vector<LimitAngular> RelativeAngularConstraintsP(const std::vector<Pose> &refpose) { return RelativeAngularConstraints(refpose, Transform(joints, [](const Joint &j) {return int2(j.rbi0, j.rbi1);})); }
	std::vector<LimitAngular> RelativeAngularConstraints(const std::vector<Pose> &refpose, std::function<bool(const Joint& j)> filter = [](const Joint &j) {return true; })  // just the free ones
	{ 
		std::vector<LimitAngular> angulars;
		for (auto &j : joints) if(filter(j))
		{
			auto dq = (refpose[j.rbi0].inverse()* refpose[j.rbi1]).inverse() * rigidbodies[j.rbi0].pose().inverse()* rigidbodies[j.rbi1].pose();
			for (int a : {0, 1, 2}) if(j.rangemin[a]!=j.rangemax[a])
				angulars.push_back(LimitAngular(&rigidbodies[j.rbi0], &rigidbodies[j.rbi1], qmat(rigidbodies[j.rbi0].orientation)[a], -dq.orientation[a] * 2.0f / physics_deltaT));
		}
		return angulars;
	}
	std::vector<Pose> GetPose()     const { return Transform(rigidbodies, [](const RigidBody & rb) {return rb.pose(); }); }
	std::vector<Pose> GetPoseUser() const { return Transform(rigidbodies, [](const RigidBody & rb) {return Pose{ rb.PositionUser(),rb.orientation }; }); }
	PhysModel& SetPose(const std::vector<Pose> &poses) {  for (unsigned int i = 0; i < poses.size() && i < rigidbodies.size(); i++) rigidbodies[i].pose() = poses[i]; return *this; }
	void Reset() { for (auto &rb : rigidbodies)::Reset(rb); }
	void SanityCheck()
	{
		for (auto &rb : rigidbodies)
			if (hasnan(rb))
				::Reset(rb);//,std::cout << "NaN check fail\n";
	}

	PhysModel(const char * jsonfile)   
	{
		auto js = json::parsefile(jsonfile).get_object();//.get_object().at("dcamera"));
		from_json(controlcages, js.at("controlcages"));
		dosubdiv();
		from_json(joints, js.at("joints"));
		std::vector<Pose> pose;
		for (auto &sdm : subdivs)
		{
			std::vector<float3>  verts = sdm.verts;
			auto tris = calchull(verts, 48);
			float3 position = (rigidbodies.size()) ? rigidbodies[joints[rigidbodies.size() - 1].rbi0].PositionUser() + joints[rigidbodies.size() - 1].p0 - joints[rigidbodies.size() - 1].p1 : float3(0.0f);
			rigidbodies.push_back(RigidBody({ Shape(verts,tris) }, position));
		}
		build_ignore_lists();

		rbscalemass(&rigidbodies[0], 3.0f);
		rbscalemass(&rigidbodies[1], 5.0f);
		for (auto &rb : rigidbodies)
		{
			meshes.push_back(MeshFlatShadeTex(rb.shapes[0].verts, rb.shapes[0].tris)); //  1 shape each is known
			rb.damping = 0.8f;
			rb.gravscale = 0;
		}
		for (auto &rb : rigidbodies)
		{
			rb.shapes[0].planes = Planes(rb.shapes[0].verts, rb.shapes[0].tris);
			auto just_w = Transform(rb.shapes[0].planes, [](const float4 &p) {return p.w; });
			rb.radius_inner = -*std::max_element(just_w.begin(), just_w.end());
		}

	}
	~PhysModel() {}
};

template<class F> void visit_fields(PhysModel &o, F f) 
{ 
	f("controlcages", o.controlcages); 
	f("joints", o.joints);
	auto pose = o.GetPose(); f("pose", pose); o.SetPose(pose);   // shoould be getPoseUser()
}

inline std::vector<LimitLinear> cloud_chamber(PhysModel &model, const std::vector<float3> &points, const std::vector<float3> outdirs, float3 origin = { 0,0,0 }, float3 viewdir = { 0,0,1.0f },float maxforce = 100.0f)
{
	std::vector<LimitLinear> linears;
	for (auto &outdir : outdirs)
	{
		auto cplane = containing_plane(points, outdir, origin,viewdir); 
		for(auto &rb:model.rigidbodies)
			linears.push_back(ConstrainUnderPlane(&rb, cplane, maxforce));
	}
	return linears;
}

#endif   // PHYSMODEL_H

