// =================================================================================================
//
//	PathFinder
//	Copyright 2014 FOURNIER Antoine. All Rights Reserved.
//
//	This program is free software. You can redistribute and/or modify it
//	in accordance with the terms of the accompanying license agreement.
//
// =================================================================================================

#include "WorldPosition.h"


namespace fournier
{

	WorldPosition::WorldPosition()
		: x(0), y(0), z(0)
	{
	}

	WorldPosition::WorldPosition(int _x, int _y, int _z)
		: x(_x), y(_y), z(_z)
	{
	}

	WorldPosition::~WorldPosition()
	{
	}

	WorldPosition::WorldPosition(WorldPosition&& _other)
		: x(0), y(0), z(0)
	{
		x = _other.x;
		y = _other.y;
		z = _other.z;

		_other.x = 0;
		_other.y = 0;
		_other.z = 0;
	}

	WorldPosition& WorldPosition::operator = (WorldPosition&& _other)
	{
		if (this != &_other)
		{
			x = _other.x;
			y = _other.y;
			z = _other.z;

			_other.x = 0;
			_other.y = 0;
			_other.z = 0;
		}

		return *this;
	}

	bool WorldPosition::operator == (const WorldPosition &_other) const
	{
		return (x == _other.x && y == _other.y && z == _other.z);
	}

	bool WorldPosition::operator != (const WorldPosition &_other) const
	{
		return !(*this == _other);
	}

}
