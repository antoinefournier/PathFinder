// =================================================================================================
//
//	PathFinder
//	Copyright 2014 FOURNIER Antoine. All Rights Reserved.
//
//	This program is free software. You can redistribute and/or modify it
//	in accordance with the terms of the accompanying license agreement.
//
// =================================================================================================

#ifndef __PATHFINDER_H__
#define __PATHFINDER_H__

#include <vector>
#include <bitset>
#include <algorithm>
#include "../world.h"
#include "PathParam.h"
#include "PathResult.h"

class NYWorld;
class NYTimer;
enum NYCubeType;

using namespace std;

namespace fournier
{
	class PreciseTimer;
	struct WorldPosition;

	/// <summary>
	/// Singleton used to find a path between two cube of our voxel world.
	/// The path finding is done in a NYWorld using the A* algorithm.
	/// The actual implementation do not support a voxel world with caves nor changing wolrd.
	/// 
	/// Before using the PathFinder initialize(NYWorld*) must be called with the instance of the NYWorld.
	/// This will construct an internal 2D representation of the world where each cell is the topmost cube of the voxel world's column.
	/// </summary>
	class PathFinder
	{
		struct AStarState;

	public:

		/// <summary>
		/// Get the unique instance of this class.
		/// </summary>
		static PathFinder* getInstance();

		/// <summary>
		/// Stop all running algorithm, free all allocated memory and put the PathFinder in its non-initialized state.
		/// </summary>
		void reset();

		/// <summary>
		/// Initialize the Pathfinder with the actual state of the world.
		/// If the Pathfinder is already initialized, nothing will be done.
		/// </summary>
		/// <param name="_world">NYWorld used to define the state of the world.</param>
		void initialize(NYWorld* _world);

		/// <summary>
		/// Update the search actually running.
		/// It has to be called every frame if the ability to run search on multiple frames is used.
		/// </summary>
		void update();

		/// <summary>
		/// Find a path between the two given position.
		/// The Z value of the given starting and ending position will be set to the topmost Cube of their column if needed.
		/// </summary>
		/// <param name="_parameters">Parameters of the path.</param>
		/// <param name="_result">Results containing the path if found and some debug datas.</param>
		/// <returns>Return false if an the parameters are incorects (probably the given position are outside the maps bounds).</returns>
		bool findPath(PathParam *_parameters, PathResult *_result);

		/// <summary>
		/// Start a search that may be split on multiple frames.
		/// The Z value of the given starting and ending position will be set to the topmost Cube of their column if needed.
		/// Be sure to call update() every frame if you are using this method.
		/// </summary>
		/// <param name="_parameters">Parameters of the path.</param>
		/// <param name="_result">Results containing the path if found and some debug datas.</param>
		/// <param name="_callback">Callback that will be called when the search ends.</param>
		/// <returns>Return the index of the search, used to identify it when the callback function is called, or -1 if an error occured.</returns>
		int startSearch(PathParam *_parameters, PathResult *_result, void(*_callback)(int, PathParam*, PathResult*));

		/// <summary>
		/// Stop the search with the given id.
		/// </summary>
		/// <param name="_id">Id of the search to stop.</param>
		void stopSearch(int _id);

		/// <summary>
		/// Indicate if the given position is considered as walkable or not.
		/// Be aware that adding or removing obstacles will drop every running search.
		/// </summary>
		/// <param name="_position">Position where to add or remove the obstacle.</param>
		/// <param name="_hasObstacle">Indicate if the given position is walkable or not.</param>
		void setObstacle(const WorldPosition& _position, bool _hasObstacle);

		/// <summary>
		/// Return the number of search actually running.
		/// </summary>
		/// <return>Number of search running.</return>
		int getNumberSearchRunning() const;

		/// <summary>
		/// Return a list of the ids of the search actually running.
		/// </summary>
		/// <return>Vector of ids.</return>
		vector<int> getRunningSearchIds() const;

		/// <summary>
		/// Indicate how many microseconds the PathFinder can spend on the search computations.
		/// Note that this is an indicative value that may not be exactly accurate.
		/// </summary>
		/// <param name="_parameters">Number of microseconds limiting the computations' time.</param>
		void setAllowedComputeTimePerFrame(long _microseconds);

		/// <summary>
		/// Indicate how many microseconds the PathFinder can spend on the search computations.
		/// Note that this is an indicative value that may not be exactly respected.
		/// </summary>
		/// <returns>Number of microseconds limiting the computations' time.</returns>
		long getAllowedComputeTimePerFrame() const;


	private:

		/// <summary>Indicate if the PathFinder has been initialized.</summary>
		bool mIsInitialized;

		/// <summary>Number of microseconds the PathFinder is allowed to spend on the search cumputations.</summary>
		long mTimeAllowedPerFrame;

		/// <summary>Timer used to get time in microseconds.</summary>
		PreciseTimer* mTimer;

		/// <summary>World used to find the paths.</summary>
		NYWorld* mWorld;

		/// <summary>List of the topmost cube of each columns of the voxel world.</summary>
		vector<int> mHeightList;

		/// <summary>List of the type of each topmost Cube of the 2D map.</summary>
		vector<NYCubeType> mCubeTypeList;

		/// <summary>List of the obstacles present in the map.</summary>
		bitset<MAT_SIZE_CUBES * MAT_SIZE_CUBES> mObstaclesList;

		/// <summary>List of the search states actually running.</summary>
		vector<AStarState*> mAStarStateList;

		/// <summary>Number of search done since the PathFinder has been initialized.</summary>
		int mNumberSearchDone;


		/// <summary>
		/// Run the A* search.
		/// </summary>
		/// <param name="_state">Actual state of the search.</param>
		/// <param name="_numberNodeChecked">Contains the number of node checked during this run of the algorithm.</param>
		/// <param name="_maximumTimeAllowed">Number of uSeconds the algorithm can spend on this search. If not set or negative, the algothim will take as much time as needed to complete.</param>
		/// <returns>Return true if the search finished completely, false if more time is needed to complete it.</returns>
		bool computeSearch(AStarState* _state, int& _numberNodeChecked, long _maximumTimeAllowed = -1);


		/// <summary>
		/// Construct the list of waypoints from a result of the A* search.
		/// </summary>
		/// <param name="_state">Actual state of the construction.</param>
		/// <param name="_maximumTimeAllowed">Number of uSeconds the algorithm can spend on this construction. If not set or negative, the algothim will take as much time as needed to complete.</param>
		/// <returns>Return true if the construction finished completely, false if more time is needed to complete it.</returns>
		bool constructPath(AStarState* _state, long _maximumTimeAllowed = -1);


		/// <summary>
		/// Nested struct holding the data for one run of the A* algorithm.
		/// This struct help us to store all the datas relative to one A* search in order to stop and resume it easily.
		/// Instead of having a list of node with value for the g and h value, we use multiple arrays.
		/// Each node is represented by an index helping us to find its corresponding data in the arrays.
		/// </summary>
		struct AStarState
		{

		public:

			AStarState()
			{
			}

			~AStarState()
			{
				// Delete everything except the parameter and result objects that do not belong to us
				mBinaryHeapDataList.clear();
				mClosedList.clear();
				mParentsList.clear();
				mGData.clear();
				mHData.clear();
				temporaryWaypointsList.clear();
			}


			//////// A* nodes ////////

		private:

			/// A Binary heap is used to store the data of the open list.
			/// The STL priority_queue is an implementation of the binary heap but it can only return the top item.
			/// We store the index of the of the node by their F value, in reverse order.

			/// <summary>Contains the index of the node in the open list sorted in a way that help us finding the node with the smallest F value.</summary>
			vector<int> mBinaryHeapDataList;

			/// <summary>Contains the index of the node in the closed list.</summary>
			vector<int> mClosedList;

			/// <summary>Contains the index of the parent of each node.</summary>
			vector<int> mParentsList = vector<int>(MAT_SIZE_CUBES * MAT_SIZE_CUBES, -1);

			/// <summary>Indicate if a node is in the open list.</summary>
			bitset<MAT_SIZE_CUBES * MAT_SIZE_CUBES> mOpenListFlags;

			/// <summary>Indicate if a node is in the closed list.</summary>
			bitset<MAT_SIZE_CUBES * MAT_SIZE_CUBES> mClosedListFlags;

			/// <summary>Cost to get to the node.</summary>
			vector<float> mGData = vector<float>(MAT_SIZE_CUBES * MAT_SIZE_CUBES, 0);

			/// <summary>Heuristic value.</summary>
			vector<float> mHData = vector<float>(MAT_SIZE_CUBES * MAT_SIZE_CUBES, 0);


		public:

			inline void G(int _x, int _y, float _value) { mGData[_x + _y * MAT_SIZE_CUBES] = _value; }
			inline float G(int _x, int _y) const { return mGData[_x + _y * MAT_SIZE_CUBES]; }
			inline float G(int _index) const { return mGData[_index]; }

			inline void H(int _x, int _y, float _value) { mHData[_x + _y * MAT_SIZE_CUBES] = _value; }
			inline float H(int _x, int _y) const { return mHData[_x + _y * MAT_SIZE_CUBES]; }
			inline float H(int _index) const { return mHData[_index]; }

			inline float F(int _x, int _y) const { return G(_x, _y) + H(_x, _y); }
			inline float F(int _index) const { return G(_index) + H(_index); }

			inline bool isInOpenList(int _x, int _y) { return mOpenListFlags[_x + _y * MAT_SIZE_CUBES]; }
			inline bool isInClosedList(int _x, int _y) { return mClosedListFlags[_x + _y * MAT_SIZE_CUBES]; }

			inline void addToOpenList(int _x, int _y)
			{
				int index = _x + _y * MAT_SIZE_CUBES;

				// Add the item at the end of the list
				mBinaryHeapDataList.push_back(index);
				float nodeF = F(index);
				int position = (int)mBinaryHeapDataList.size();

				while (position != 1)
				{
					// Check if the new node has a bigger F value than its actual parent
					if (nodeF > F(mBinaryHeapDataList[(position / 2) - 1]))
						break;

					// Swap it with its parent
					int temp = mBinaryHeapDataList[(position / 2) - 1];
					mBinaryHeapDataList[(position / 2) - 1] = index;
					mBinaryHeapDataList[position - 1] = temp;

					position = position / 2;
				}

				mOpenListFlags[_x + _y * MAT_SIZE_CUBES] = true;
			}

			inline int getBestNodeInOpenList()
			{
				if (mBinaryHeapDataList.size() == 0)
					return -1;

				int value = mBinaryHeapDataList[0];

				// Place the last element at the first position before removing it
				mBinaryHeapDataList[0] = mBinaryHeapDataList[mBinaryHeapDataList.size() - 1];
				mBinaryHeapDataList.pop_back();

				int numberItem = mBinaryHeapDataList.size();

				// Position of the two children (index + 1)
				int v = 1;
				int u;

				while (true)
				{
					u = v;

					// Be sure to not go outside of the array
					if (2 * u + 1 <= numberItem)
					{
						// Find the child with the lowest F cost
						if (F(mBinaryHeapDataList[u - 1]) >= F(mBinaryHeapDataList[(2 * u) - 1]))
							v = 2 * u;
						if (F(mBinaryHeapDataList[v - 1]) >= F(mBinaryHeapDataList[(2 * u + 1) - 1]))
							v = 2 * u + 1;
					}
					else if (2 * u <= numberItem) // Only one child available
					{
						if (F(mBinaryHeapDataList[u - 1]) >= F(mBinaryHeapDataList[(2 * u) - 1]))
							v = 2 * u;
					}

					if (u == v)
						break;

					// One child was better, swapt its position with the node
					int temp = mBinaryHeapDataList[u - 1];
					mBinaryHeapDataList[u - 1] = mBinaryHeapDataList[v - 1];
					mBinaryHeapDataList[v - 1] = temp;
				}

				mOpenListFlags[value] = false;

				return value;
			}

			inline bool isOpenListEmpty() { return (mBinaryHeapDataList.size() == 0); }

			inline void setParent(int _x, int _y, int _parentX, int _parentY) { mParentsList[_x + _y * MAT_SIZE_CUBES] = _parentX + _parentY * MAT_SIZE_CUBES; }

			inline void sortOpenList(int _x, int _y)
			{
				int index = _x + _y * MAT_SIZE_CUBES;
				int position = -1;

				// Find the node
				int numberItem = mBinaryHeapDataList.size();
				for (int n = 0; n < numberItem; ++n)
				{
					if (mBinaryHeapDataList[n] == index)
					{
						position = n + 1;
						break;
					}
				}

				if (position == -1)
					return;

				float nodeF = F(index);

				while (position != 1)
				{
					// Check if the new node has a bigger F value than its actual parent
					if (nodeF > F(mBinaryHeapDataList[(position / 2) - 1]))
						break;

					// Swap it with its parent
					int temp = mBinaryHeapDataList[(position / 2) - 1];
					mBinaryHeapDataList[(position / 2) - 1] = index;
					mBinaryHeapDataList[position - 1] = temp;

					position = position / 2;
				}
			}

			inline void addToClosedList(int _x, int _y)
			{
				int index = _x + _y * MAT_SIZE_CUBES;

				if (mClosedListFlags[index])
					return;

				mClosedList.push_back(_x + _y * MAT_SIZE_CUBES);
				mClosedListFlags[index] = true;
			}

			inline void removeFromClosedList(int _x, int _y)
			{
				int index = _x + _y * MAT_SIZE_CUBES;

				if (!mClosedListFlags[index])
					return;

				mClosedList.erase(remove(mClosedList.begin(), mClosedList.end(), _x + _y * MAT_SIZE_CUBES), mClosedList.end());
				mClosedListFlags[index] = false;
			}

			inline void getListPoint(vector<int> &_posX, vector<int> &_posY)
			{
				int index = mClosedList.back();
				while (index != -1)
				{
					_posX.push_back(index % MAT_SIZE_CUBES);
					_posY.push_back(index / MAT_SIZE_CUBES);
					index = mParentsList[index];
				}
			}


			//////// Other datas ////////

		public:

			/// <summary>Indicate if the A* search is finished.</summary>
			bool isAStarFinished;

			/// <summary>Indicate if the construction of the WorldPosition is finished.</summary>
			bool isPathGenerated;

			/// <summary>Unique id of the search.</summary>
			int id;

			/// <summary>Callback to call when the search ends.</summary>
			void(*callback)(int, PathParam*, PathResult*);

			/// <summary>Pointer to the user created PathParam.</summary>
			PathParam *parameters;

			/// <summary>Pointer to the user created PathResult.</summary>
			PathResult *result;

			/// <summary>List of waypoint of the path.</summary>
			vector<WorldPosition> temporaryWaypointsList;

		}; // AStarState


		/// Singleton ///

		static PathFinder *mInstance;

		PathFinder();
		~PathFinder();
		PathFinder(const PathFinder &) = delete;
		PathFinder& operator=(const PathFinder &) = delete;

	};
}

#endif
