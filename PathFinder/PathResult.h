// =================================================================================================
//
//	PathFinder
//	Copyright 2014 FOURNIER Antoine. All Rights Reserved.
//
//	This program is free software. You can redistribute and/or modify it
//	in accordance with the terms of the accompanying license agreement.
//
// =================================================================================================

#ifndef __PATH_RESULT_H__
#define __PATH_RESULT_H__

#include <vector>
#include "WorldPosition.h"

using namespace std;

namespace fournier
{
	/// <summary>
	/// Hold the result of a PathFinder attempt to find a path between two points.
	/// </summary>
	struct PathResult
	{
		/// <summary>Indicate if a path has been found between the two given positions.</summary>
		bool isPathFound = false;

		/// <summary>Contains the list of all the points defining the path found.</summary>
		vector<WorldPosition> waypointsList;

		/// <summary>Totla time in microseconds the PathFinder spent to find this path.</summary>
		long totalComputeTime = 0;

		/// <summary>Time in microseconds the PathFinder spent on the A* algorithm.</summary>
		long AStarComputeTime = 0;

		/// <summary>Time in microseconds the PathFinder spent on the creation of the waypoints.</summary>
		long waypointsCreationTime = 0;

		/// <summary>Number frame it took to complete the search.</summary>
		int numbreFrame = 0;

		/// <summary>Number of node checked by the PathFinder to find this path.</summary>
		int numberNodeChecked = 0;

		~PathResult()
		{
			waypointsList.clear();
		}
	};

}

#endif
