// =================================================================================================
//
//	PathFinder
//	Copyright 2014 FOURNIER Antoine. All Rights Reserved.
//
//	This program is free software. You can redistribute and/or modify it
//	in accordance with the terms of the accompanying license agreement.
//
// =================================================================================================

#ifndef __PATH_PARAM_H__
#define __PATH_PARAM_H__

#include <vector>
#include "../cube.h"
#include "WorldPosition.h"

using namespace std;

namespace fournier
{
	/// <summary>
	/// Used to give a set of parameters to the PathFinder.
	/// </summary>
	struct PathParam
	{
		/// <summary>Starting position of the path.</summary>
		WorldPosition startPosition;

		/// <summary>Ending position of the path.</summary>
		WorldPosition endPosition;

		/// <summary>
		/// Indicate what type of cube will be considered by the PathFinder as usable for the path finding algorithm.
		/// CUBE_AIR cannot be walkable.
		/// If empty, every cube exept CUBE_AIR will be used as walkable.
		/// </summary>
		vector<NYCubeType> walkableCubeTypeList;

		/// <summary>Indicate if the path can have diagonal moves.</summary>
		bool allowDiagonalMovements;

		/// <summary>Maximum positive difference on the Z axis between two cube to be able to walk from one to the other.</summary>
		int maximumJumpHeight;

		/// <summary>Maximum negative difference on the Z axis between two cube to be able to walk from one to the other.</summary>
		int maximumFallHeight;


		/// <param name="_startPosition">Starting position of the path</param>
		/// <param name="_endPosition">Ending position of the path</param>
		/// <param name="_walkableCubeType">
		/// Indicate what type of cube will be considered by the PathFinder as usable for the path finding algorithm.
		/// CUBE_AIR cannot be walkable.
		/// If empty, every cube exept CUBE_AIR will be used as walkable.
		/// </param>
		/// <param name="_allowDiagonalMovements">Indicate if the path can have diagonal moves.</param>
		/// <param name="_maximumJunmpHeight">Maximum positive difference on the Z axis between two cube to be able to walk from one to the other.</param>
		/// <param name="_maximumFallHeight">Maximum negative difference on the Z axis between two cube to be able to walk from one to the other.</param>
		PathParam(WorldPosition _startPosition, WorldPosition _endPosition, vector<NYCubeType> _walkableCubeType, bool _allowDiagonalMovements = true, unsigned int _maximumJumpHeight = 999, unsigned int _maximumFallHeight = 999)
			: startPosition(_startPosition), endPosition(_endPosition),
			walkableCubeTypeList(_walkableCubeType),
			allowDiagonalMovements(_allowDiagonalMovements),
			maximumJumpHeight(_maximumJumpHeight), maximumFallHeight(_maximumFallHeight)
		{
		}

		PathParam()
		{
		}

		PathParam(const PathParam&) = delete;

		~PathParam()
		{
			walkableCubeTypeList.clear();
		}
	};

}

#endif
