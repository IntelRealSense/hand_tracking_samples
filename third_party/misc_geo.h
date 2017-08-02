//
// some extra geometric utility routines  
// unlike the routines in geometric.h, what's here currently are things typically more specifically to vision related applications
// 

#pragma once
#ifndef MISC_GEO_H
#define MISC_GEO_H

#include <iostream>
#include <fstream>
#include <string>
#include <exception>

#include "geometric.h"
#include "misc.h"      // for string split

// courtesty of leo keselman:
inline std::vector<float3> ObtainVoxelPointCloud(std::vector<float3> & dpts, float voxelSize, int minOccupants)
{
	std::vector<float3> points;
	// Structure for storing per-voxel data
	enum { HASH_SIZE = 1024, HASH_MASK = HASH_SIZE - 1 }; // Only works if HASH_SIZE is power of two
	struct Voxel { int3 coord; float3 point; int count; };
	Voxel voxelHash[HASH_SIZE];
	memset(voxelHash, 0, sizeof(voxelHash));

	const float inverseVoxelSize = 1.0f / voxelSize;
	static const int3 hashCoeff(7171, 3079, 4231);

	// For the pixels in our image
	for (auto &p : dpts)
	{
		// Obtain corresponding voxel
		auto fcoord = floor(p * inverseVoxelSize);
		auto vcoord = int3(static_cast<int>(fcoord.x), static_cast<int>(fcoord.y), static_cast<int>(fcoord.z));
		auto hash = dot(vcoord, hashCoeff) & HASH_MASK;
		auto & voxel = voxelHash[hash];

		// If we collide, flush existing voxel contents
		if (voxel.count && voxel.coord != vcoord)
		{
			if (voxel.count > minOccupants) points.push_back(voxel.point / (float)voxel.count);
			voxel.count = 0;
		}

		// If voxel is empty, store the point
		if (voxel.count == 0)
		{
			voxel.coord = vcoord;
			voxel.point = p;
			voxel.count = 1;
		}
		else // Otherwise just add position contribution
		{
			voxel.point += p;
			++voxel.count;
		}

	}

	// Flush remaining voxels
	for (auto it = std::begin(voxelHash); it != std::end(voxelHash); ++it)
	{
		if (it->count > minOccupants)
		{
			points.push_back(it->point / (float)it->count);
		}
	}
	return points;
}


inline std::pair<std::vector<float3>, std::vector<std::vector<int>>> OBJFormatLoadFaceMesh(const char *filename)  // keeps ngons
{
	std::vector<float3> points;
	std::vector<std::vector<int>> faces; // ngons
	std::ifstream filein(filename);
	if (!filein.is_open())
		throw "unable to open file";
	std::string line;
	while (std::getline(filein, line))
	{
		auto tokens = split(line);
		if (!tokens.size())
			continue;
		auto head = tokens.front();
		tokens.erase(tokens.begin());
		if (head == "v")
		{
			line.erase(0, 1);
			points.push_back(StringTo<float3>(line));
			continue;
		}
		if (head == "f")
		{
			std::vector<int> face;
			for (auto &token : tokens)
				face.push_back(StringTo<int>(split(token, "/")[0]) - 1);  // ugg obj files index from one instead of zero
			faces.push_back(face);
			continue;
		}
	}
	return{ points,faces };
}
inline std::vector<int3> Triangulate(const std::vector<std::vector<int>> &faces)
{
	std::vector<int3> tris;
	for (auto &face : faces)
		for (unsigned int i = 2; i < face.size(); i++)
			tris.push_back({ face[0], face[i - 1], face[i] });
	return tris;
}
inline std::pair<std::vector<float3>, std::vector<int3>> OBJFormatLoadTriMesh(const char *filename)  // just positions  triangulates ngons 
{
	std::vector<float3> points;
	std::vector<std::vector<int>> faces; // ngons
	std::tie(points, faces) = OBJFormatLoadFaceMesh(filename);
	std::vector<int3> tris = Triangulate(faces);
	return{ points,tris };
}


inline void OBJFormatSaveMesh(const std::vector<float3> & points, const std::vector<int3> & triangles, std::string filename = "", bool center = false)
{
	auto bbox = Extents(points);

	static int id = 0;
	if (filename == "")
		filename = freefilename("meshshot_", ".obj");
	std::ofstream os(filename);
	if (!os.is_open())
		throw std::exception("unable to open output file for .obj export");
	os << "# obj format pointcloud data\n\n";
	os << "g pointcloud\n\n";
	float3 centroid = (center) ? (bbox.first + bbox.second)*0.5f : float3{ 0, 0, 0 };
	for (auto &p : points)
	{
		os << "v " << (p - centroid) << "\n";   // center points on export?? 
	}
	os << "\n\n# faces\n\n";
	for (auto t : triangles)
		os << "f " << (t + int3{ 1, 1, 1 }) << "\n";  // it seems that .obj files index from 1
}




#endif // MISC_GEO_H

