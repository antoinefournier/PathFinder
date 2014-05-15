// =================================================================================================
//
//	PathFinder
//	Copyright 2014 FOURNIER Antoine. All Rights Reserved.
//
//	This program is free software. You can redistribute and/or modify it
//	in accordance with the terms of the accompanying license agreement.
//
// =================================================================================================

#include "PathFinder.h"

#include <array>
#include <windows.h>

#include "../chunk.h"
#include "../cube.h"
#include "engine/timer.h"
#include "WorldPosition.h"
#include "PreciseTimer.h"


inline int index(int _x, int _y)
{
	return _x + _y * MAT_SIZE_CUBES;
}

inline int manhatanDistance(int _x, int _y, int _destX, int _destY)
{
	return ((_x < _destX) ? _destX - _x : _x - _destX) + ((_y < _destY) ? _destY - _y : _y - _destY);
}

inline void addNeightbours(int _x, int _y, array<int, 8> &_posX, array<int, 8> &_posY, int &_numberNeightbours)
{
	if (_x < 0 || _x >= MAT_SIZE_CUBES || _y < 0 || _y >= MAT_SIZE_CUBES)
		return;

	_posX[_numberNeightbours] = _x;
	_posY[_numberNeightbours] = _y;
	++_numberNeightbours;
}


namespace fournier
{
	PathFinder* PathFinder::mInstance = nullptr;

	PathFinder* PathFinder::getInstance()
	{
		if (mInstance == nullptr)
			mInstance = new PathFinder();
		return mInstance;
	}

	PathFinder::PathFinder()
	{
		mIsInitialized = false;
		mWorld = nullptr;
		mTimer = new PreciseTimer();
		mTimeAllowedPerFrame = 5000;
	}

	PathFinder::~PathFinder()
	{
		reset(); // Reset is doing what we need
		delete mTimer;
	}

	void PathFinder::setAllowedComputeTimePerFrame(long _microseconds)
	{
		mTimeAllowedPerFrame = _microseconds;

		if (mTimeAllowedPerFrame < 1)
			mTimeAllowedPerFrame = 1;
	}

	long PathFinder::getAllowedComputeTimePerFrame() const
	{
		return mTimeAllowedPerFrame;
	}

	void PathFinder::initialize(NYWorld* _world)
	{
		if (mIsInitialized)
			return;

		mWorld = _world;

		mHeightList = vector<int>(MAT_SIZE_CUBES * MAT_SIZE_CUBES);
		mCubeTypeList = vector<NYCubeType>(MAT_SIZE_CUBES * MAT_SIZE_CUBES);

		for (unsigned int y = 0; y < MAT_SIZE_CUBES; ++y)
		{
			for (unsigned int x = 0; x < MAT_SIZE_CUBES; ++x)
			{
				for (unsigned int z = MAT_HEIGHT_CUBES; z >= 0; --z)
				{
					if (mWorld->getCube(x, y, z)->_Type == CUBE_AIR)
						continue;

					mHeightList[index(x, y)] = z;
					mCubeTypeList[index(x, y)] = mWorld->getCube(x, y, z)->_Type;
					mObstaclesList[index(x, y)] = false;

					break;
				}
			}
		}

		mNumberSearchDone = 0;
		mIsInitialized = true;
	}

	void PathFinder::reset()
	{
		mWorld = nullptr;

		mHeightList.clear();
		mCubeTypeList.clear();

		for (auto it = mAStarStateList.begin(); it != mAStarStateList.end(); ++it)
		{
			delete (*it)->parameters;
			delete (*it)->result;
		}
		mAStarStateList.clear();

		mIsInitialized = false;
	}

	bool isInMapRange(int _x, int _y)
	{
		if (_x < 0 || _x >= MAT_SIZE_CUBES ||
			_y < 0 || _y >= MAT_SIZE_CUBES)
			return false;
		return true;
	}

	int PathFinder::getNumberSearchRunning() const
	{
		return mAStarStateList.size();
	}

	vector<int> PathFinder::getRunningSearchIds() const
	{
		vector<int> idsList;
		for (auto it = mAStarStateList.begin(); it != mAStarStateList.end(); ++it)
			idsList.push_back((*it)->id);
		return idsList;
	}

	void PathFinder::setObstacle(const WorldPosition &_position, bool _hasObstacle)
	{
		if (!isInMapRange(_position.x, _position.y))
			return;

		// Check if the new state will change the map
		bool lastState = mObstaclesList[index(_position.x, _position.y)];
		if (lastState == _hasObstacle)
			return;

		// Change the state
		mObstaclesList[index(_position.x, _position.y)] = _hasObstacle;

		// Stop and destroy every running search
		for (auto it = mAStarStateList.begin(); it != mAStarStateList.end(); ++it)
		{
			delete (*it)->parameters;
			delete (*it)->result;
		}
		mAStarStateList.clear();
	}

	void PathFinder::update()
	{
		long maxAllowedTime = mTimeAllowedPerFrame;
		long start, end;

		/// Update all the A* search running
		for (auto it = mAStarStateList.begin(); it != mAStarStateList.end();)
		{
			// We dont have any time left
			if (maxAllowedTime <= 0)
				break;

			(*it)->result->numbreFrame += 1;

			// A* search
			if ((*it)->isAStarFinished == false)
			{
				start = mTimer->getTimeMicroSeconds();
				int numberNodeChecked = 0;

				computeSearch(*it, numberNodeChecked, maxAllowedTime);

				end = mTimer->getTimeMicroSeconds();

				maxAllowedTime -= end - start;
				(*it)->result->numberNodeChecked += numberNodeChecked;
				(*it)->result->AStarComputeTime += end - start;
			}

			// We dont have any time left
			if (maxAllowedTime <= 0)
				break;

			// Construct the final list of waypoints
			if ((*it)->isAStarFinished == true && (*it)->isPathGenerated == false)
			{
				start = mTimer->getTimeMicroSeconds();

				constructPath(*it);

				end = mTimer->getTimeMicroSeconds();
				maxAllowedTime -= end - start;
				(*it)->result->waypointsCreationTime += end - start;
			}

			// We dont have any time left
			if (maxAllowedTime <= 0)
				break;

			// Set the results and call the callback and remove the element
			if ((*it)->isAStarFinished && (*it)->isPathGenerated)
			{
				if ((*it)->result->waypointsList.size() > 0 && 
					(*it)->result->waypointsList.back().x == (*it)->parameters->endPosition.x && (*it)->result->waypointsList.back().y == (*it)->parameters->endPosition.y)
					(*it)->result->isPathFound = true;

				(*it)->result->totalComputeTime += (*it)->result->AStarComputeTime + (*it)->result->waypointsCreationTime;

				(*it)->callback((*it)->id, (*it)->parameters, (*it)->result);
				it = mAStarStateList.erase(it);
			}
			else
			{
				++it;
			}
		}
	}

	bool PathFinder::findPath(PathParam *_parameters, PathResult *_result)
	{
		// Create the state and save the parameters and results object
		AStarState* state = new AStarState();
		state->parameters = _parameters;
		state->result = _result;
		state->id = -1;
		state->callback = nullptr;
		state->isAStarFinished = false;
		state->isPathGenerated = false;

		mNumberSearchDone += 1;

		// Check if the two given positions are valids
		if (!isInMapRange(state->parameters->startPosition.x, state->parameters->startPosition.y) || !isInMapRange(state->parameters->endPosition.x, state->parameters->endPosition.y))
			return false;

		// Make sure the result's datas are initialized
		state->result->numbreFrame = 0;
		state->result->numberNodeChecked = 0;
		state->result->totalComputeTime = 0;
		state->result->AStarComputeTime = 0;
		state->result->waypointsCreationTime = 0;

		int numberNodeChecked = 0;

		// Add the first node to the open list
		state->addToOpenList(state->parameters->startPosition.x, state->parameters->startPosition.y);


		long startTimer = mTimer->getTimeMicroSeconds();

		// A* search
		computeSearch(state, numberNodeChecked);

		long middleTimer = mTimer->getTimeMicroSeconds();

		// Construct the final list of waypoints
		constructPath(state);

		long endTimer = mTimer->getTimeMicroSeconds();


		// Save the final result
		state->result->numberNodeChecked = numberNodeChecked;
		state->result->totalComputeTime = endTimer - startTimer;
		state->result->AStarComputeTime = middleTimer - startTimer;
		state->result->waypointsCreationTime = endTimer - middleTimer;
		
		state->result->numbreFrame = 0;

		if (state->result->waypointsList.back().x == state->parameters->endPosition.x && state->result->waypointsList.back().y == state->parameters->endPosition.y)
			state->result->isPathFound = true;

		// Delete the state, but keep the parameters and result
		delete state;

		return true;
	}

	int PathFinder::startSearch(PathParam *_parameters, PathResult *_result, void(*_callback)(int, PathParam*, PathResult*))
	{
		// Create the state and save the parameters and results object
		AStarState* state = new AStarState();
		state->parameters = _parameters;
		state->result = _result;
		state->id = ++mNumberSearchDone;
		state->callback = _callback;
		state->isAStarFinished = false;
		state->isPathGenerated = false;

		// Check if the two given positions are valids
		if (!isInMapRange(state->parameters->startPosition.x, state->parameters->startPosition.y) || !isInMapRange(state->parameters->endPosition.x, state->parameters->endPosition.y))
			return -1;

		// Make sure the result's datas are initialized
		state->result->numbreFrame = 0;
		state->result->numberNodeChecked = 0;
		state->result->totalComputeTime = 0;
		state->result->AStarComputeTime = 0;
		state->result->waypointsCreationTime = 0;

		// Add the first node to the open list
		state->addToOpenList(state->parameters->startPosition.x, state->parameters->startPosition.y);

		// Save the state to be computed later
		mAStarStateList.push_back(state);

		return state->id;
	}

	void PathFinder::stopSearch(int _id)
	{
		// Find the corresponding AStarState
		int index = -1;
		for (int n = 0; n < (int)mAStarStateList.size(); ++n)
		{
			if (mAStarStateList[n]->id == _id)
			{
				index = n;
				break;
			}
		}

		// Remove and delete the AStarState
		if (index != -1)
			mAStarStateList.erase(mAStarStateList.begin() + index);
	}


	bool PathFinder::computeSearch(AStarState* _state, int& _numberNodeChecked, long _maximumTimeAllowed)
	{
		// We used this to temporary store the position of each neightbours node
		array<int, 8> neightboursXList, neightboursYList;
		int numberValidNeightbours = 0;
		long startTime = mTimer->getTimeMicroSeconds();

		_state->isAStarFinished = false;

		while (!_state->isOpenListEmpty())
		{
			++_numberNodeChecked;

			// We check the time spent every after nodes
			// The compute time to getTimerMicroseconds is insignifiant compared to the A* node's test
			if (_maximumTimeAllowed > 0)
				if (mTimer->getTimeMicroSeconds() - startTime >= _maximumTimeAllowed)
					return false;


			// Find the best node
			int n = _state->getBestNodeInOpenList();
			int actualX = n % MAT_SIZE_CUBES;
			int actualY = n / MAT_SIZE_CUBES;
			//_state->removeFromOpenList(actualX, actualY);

			// Check if we are at the destination
			if (actualX == _state->parameters->endPosition.x && actualY == _state->parameters->endPosition.y)
			{
				_state->addToClosedList(actualX, actualY);
				break;
			}

			// Find the valid neightbours of the actual node
			numberValidNeightbours = 0;

			addNeightbours(actualX - 1, actualY, neightboursXList, neightboursYList, numberValidNeightbours);
			addNeightbours(actualX + 1, actualY, neightboursXList, neightboursYList, numberValidNeightbours);
			addNeightbours(actualX, actualY - 1, neightboursXList, neightboursYList, numberValidNeightbours);
			addNeightbours(actualX, actualY + 1, neightboursXList, neightboursYList, numberValidNeightbours);

			if (_state->parameters->allowDiagonalMovements)
			{
				addNeightbours(actualX - 1, actualY - 1, neightboursXList, neightboursYList, numberValidNeightbours);
				addNeightbours(actualX + 1, actualY + 1, neightboursXList, neightboursYList, numberValidNeightbours);
				addNeightbours(actualX + 1, actualY - 1, neightboursXList, neightboursYList, numberValidNeightbours);
				addNeightbours(actualX - 1, actualY + 1, neightboursXList, neightboursYList, numberValidNeightbours);
			}

			// For each neightbours, check if it can be a valid waypoint for the path
			for (int n = 0; n < numberValidNeightbours; ++n)
			{
				int newX = neightboursXList[n];
				int newY = neightboursYList[n];

				if (_state->isInClosedList(newX, newY))
					continue;

				// Check if there is an obstacle on the cube
				if (mObstaclesList[index(newX, newY)])
					continue;

				// Check if the type of the cube is walkable
				int numberWalkableCubeType = _state->parameters->walkableCubeTypeList.size();
				if (numberWalkableCubeType > 0)
				{
					NYCubeType type = mCubeTypeList[index(newX, newY)];
					int canWalk = false;
					for (auto it = _state->parameters->walkableCubeTypeList.begin(); it != _state->parameters->walkableCubeTypeList.end(); ++it)
					{
						if ((*it) == type)
						{
							canWalk = true;
							break;
						}
					}
					if (!canWalk)
						continue;
				}
				
				// The neightbour is too high
				if (mHeightList[index(newX, newY)] - mHeightList[index(actualX, actualY)] > _state->parameters->maximumFallHeight)
					continue;
				
				// The neightbour is too low
				if (mHeightList[index(actualX, actualY)] - mHeightList[index(newX, newY)] > _state->parameters->maximumJumpHeight)
					continue;
				
				bool isInOpenList = _state->isInOpenList(newX, newY);

				// Update the neightbours node's data if needed
				float newG = _state->G(actualX, actualY) + ((actualX != newX && actualY != newY) ? 1.4142f : 1.0f);
				if (!isInOpenList || newG < _state->G(newX, newY))
				{
					_state->setParent(newX, newY, actualX, actualY);
					_state->G(newX, newY, newG);
					_state->H(newX, newY, (float)manhatanDistance(newX, newY, _state->parameters->endPosition.x, _state->parameters->endPosition.y));
					_state->sortOpenList(newX, newY);
				}

				// Add it the the open list
				if (!isInOpenList)
					_state->addToOpenList(newX, newY);
			}

			// Add the actual node to the closed list
			_state->addToClosedList(actualX, actualY);
		}

		_state->isAStarFinished = true;
		return true;
	}

	bool PathFinder::constructPath(AStarState *_state, long _maximumTimeAllowed)
	{
		vector<int> posX, posY;
		int x, y, z, nextX, nextY, nextZ;

		_state->isPathGenerated = false;

		// Get the list of every point in the path
		_state->getListPoint(posX, posY);

		if (posX.size() > 0)
		{
			nextX = posX[posX.size() - 1];
			nextY = posY[posY.size() - 1];
			nextZ = mHeightList[index(nextX, nextY)];
		}

		for (int n = (int)posX.size() - 1; n >= 0; --n)
		{
			long startTime = mTimer->getTimeMicroSeconds();

			// We check the time spent every after nodes
			if (_maximumTimeAllowed > 0)
				if (mTimer->getTimeMicroSeconds() - startTime >= _maximumTimeAllowed)
					return false;

			x = nextX;
			y = nextY;
			z = nextZ;

			_state->temporaryWaypointsList.push_back(WorldPosition(x, y, z));

			// If the next point is lower or higher than the next, add another point at the top of the actual or next point
			// to have a path where all point are aligned with the voxel grid
			if (n > 0)
			{
				nextX = posX[n - 1];
				nextY = posY[n - 1];
				nextZ = mHeightList[index(nextX, nextY)];

				if (z < nextZ)
					_state->temporaryWaypointsList.push_back(WorldPosition(x, y, nextZ));
				else if (z > nextZ)
					_state->temporaryWaypointsList.push_back(WorldPosition(nextX, nextY, z));

			}
		}

		_state->result->waypointsList = _state->temporaryWaypointsList;
		_state->isPathGenerated = true;
		return true;
	}
}