// =================================================================================================
//
//	PathFinder
//	Copyright 2014 FOURNIER Antoine. All Rights Reserved.
//
//	This program is free software. You can redistribute and/or modify it
//	in accordance with the terms of the accompanying license agreement.
//
// =================================================================================================

#ifndef __WOLRD_POSITION_H__
#define __WOLRD_POSITION_H__

namespace fournier
{
	/// <summary>Contains a position of a cube in our voxel world.</summary>
	struct WorldPosition
	{
		int x;
		int y;
		int z;

		// Ctor / Dtor
		WorldPosition();
		WorldPosition(int _x, int _y, int _z);
		~WorldPosition();

		// Move
		WorldPosition(WorldPosition &&_other);
		WorldPosition& operator = (WorldPosition &&_other);

		bool operator == (const WorldPosition &_other) const;
		bool operator != (const WorldPosition &_other) const;
	};

}

#endif
