// =================================================================================================
//
//	PathFinder
//	Copyright 2014 FOURNIER Antoine. All Rights Reserved.
//
//	This program is free software. You can redistribute and/or modify it
//	in accordance with the terms of the accompanying license agreement.
//
// =================================================================================================

#ifndef __PRECISE_TIMER_H__
#define __PRECISE_TIMER_H__

#include <windows.h>

namespace fournier
{

	/// <summary>
	/// Timer giving time in microseconds.
	/// <summary>
	class PreciseTimer
	{

	private:

		LARGE_INTEGER mFrequency;


	public:

		PreciseTimer()
		{
			QueryPerformanceFrequency(&mFrequency);
		}

		long getTimeMicroSeconds() const
		{
			LARGE_INTEGER time;
			QueryPerformanceCounter(&time);
			time.QuadPart *= 1000000;
			time.QuadPart /= mFrequency.QuadPart;
			return (long)time.QuadPart;
		}
	};

}

#endif
