#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#include <cmath>
#include <numbers>
#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion

bool AStarPather::initialize()
{
    // handle any one-time setup requirements you have

    /*
        If you want to do any map-preprocessing, you'll need to listen
        for the map change message.  It'll look something like this:

        Callback cb = std::bind(&AStarPather::your_function_name, this);
        Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

        There are other alternatives to using std::bind, so feel free to mix it up.
        Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
        object that std::function can wrap will suffice.
    */



    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */
}

PathResult AStarPather::compute_path(PathRequest &request)
{
    /*
        This is where you handle pathing requests, each request has several fields:

        start/goal - start and goal world positions
        path - where you will build the path upon completion, path should be
            start to goal, not goal to start
        heuristic - which heuristic calculation to use
        weight - the heuristic weight to be applied
        newRequest - whether this is the first request for this path, should generally
            be true, unless single step is on

        smoothing - whether to apply smoothing to the path
        rubberBanding - whether to apply rubber banding
        singleStep - whether to perform only a single A* step
        debugColoring - whether to color the grid based on the A* state:
            closed list nodes - yellow
            open list nodes - blue

            use terrain->set_color(row, col, Colors::YourColor);
            also it can be helpful to temporarily use other colors for specific states
            when you are testing your algorithms

        method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
            will be A* generally, unless you implement extra credit features

        The return values are:
            PROCESSING - a path hasn't been found yet, should only be returned in
                single step mode until a path is found
            COMPLETE - a path to the goal was found and has been built in request.path
            IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
    */

    // WRITE YOUR CODE HERE

    if (request.newRequest)
    {
        request.newRequest = false;
        InitRequest(request);
    }

    /*if (wait)
    {
        int s = 0;
        for (int i = 0; i < 1000; ++i)
        {
            s += i;
        }
        wait = s < 69;
        return PathResult::PROCESSING;
    }
    wait = true;*/

    while (!_openList.empty())
    {
        Node curr = _openList.top();

        if (curr.IsOpen(_allNodes) && red)
        {
            terrain->set_color(IntToRow(curr._pos), IntToCol(curr._pos), Colors::Red);
            red = false;
            return PathResult::PROCESSING;
        }
        red = true;

        _openList.pop();

        if (curr.IsOpen(_allNodes))
        {
            if (curr._pos == goal)
            {
                FinishRequest(request);
                return PathResult::COMPLETE;
            }

            AddNeighboors(curr);

            if (debug)
                ColorClosed(curr._pos);

            return PathResult::PROCESSING;
        }
    }

    return PathResult::IMPOSSIBLE;
}

void AStarPather::InitRequest(const PathRequest& request)
{
    grid_width = terrain->get_map_width();
    int start = GridToInt(terrain->get_grid_position(request.start));
    goal = GridToInt(terrain->get_grid_position(request.goal));
    debug = request.settings.debugColoring;
    h = request.settings.heuristic;
    weight = request.settings.weight;

    if (_debugColor)
        ColorInit(start);

    const int size = grid_width * terrain->get_map_height();
    _allNodes.clear();
    _allNodes.resize(size);
    //std::memset(&_allNodes[0], static_cast<int>(NodeCore()), sizeof(_allNodes[0]) * size);

    //std::fill(_allNodes.cbegin(), _allNodes.cbegin() + size, unexplored);

    while (!_openList.empty())
        _openList.pop(); // The priority queue's container is a vector that has a
                         // clear method so I shouldn't have to do this in O(n). 
    _openList.emplace(start);
    //_allNodes.emplace(_allNodes.cbegin() + start);
    _allNodes[start].SetStart();
}

void AStarPather::AddNeighboors(Node& curr)
{
    int x = IntToCol(curr._pos);
    int y = IntToRow(curr._pos);

    bool cardResult[4];

    for (DirectT i = cardStart; i < cardEnd; ++i)
    {
        int col = x + _x_comp[i];
        int row = y + _y_comp[i];
        if(cardResult[i] = terrain->is_valid_grid_position(row, col) && !terrain->is_wall(row, col))
            AddCard(curr, CoordToInt(row, col), i);
    }

    for (DirectT i = diagStart; i < diagEnd; ++i)
    {
        if (cardResult[i % numEach] && cardResult[(i + 1) % numEach])
        {
            int col = x + _x_comp[i];
            int row = y + _y_comp[i];
            if(!terrain->is_wall(row, col))
                AddDiag(curr, CoordToInt(row, col), i);
        }
    }
}

void AStarPather::FinishRequest(PathRequest& request)
{
    if (request.settings.rubberBanding)
        Rubberbanding(request);

    int r = IntToRow(goal);
    int c = IntToCol(goal);
    char d;

    while(true)
    {
        request.path.push_front(terrain->get_world_position(r, c));
        d = _allNodes[CoordToInt(r,c)]._parent;
        if (d == done)
            break;
        c -= _x_comp[d];
        r -= _y_comp[d];
    }
}

void AStarPather::Rubberbanding(PathRequest& request)
{
    int back = goal;

    // incase start and goal are the same node
    int mid = _allNodes[back]._parent;
    if (mid < 0)
       return;

    int front = _allNodes[mid]._parent;

    while (front >= 0)
    {
        int r1 = std::min<int>(IntToRow(back), IntToRow(front));
        int r2 = std::max<int>(IntToRow(back), IntToRow(front));
        int c1 = std::min<int>(IntToCol(back), IntToCol(front));
        int c2 = std::max<int>(IntToCol(back), IntToCol(front));

        bool skip = true;
        for (int r = r1; r <= r2; ++r)
            for (int c = c1; c <= c2; ++c)
                if (terrain->is_wall(r, c))
                {
                    skip = false;
                    break;
                }

        if (skip)
            _allNodes[back]._parent = front;
        else
            back = mid;

        mid = front;
        front = _allNodes[mid]._parent;
    }
}

void AStarPather::ColorInit(int start)
{
    terrain->set_color(IntToRow(start), IntToCol(start), Colors::Orange);
    terrain->set_color(IntToRow(goal), IntToCol(goal), Colors::Orange);
}

void AStarPather::ColorOpen(int open)
{
    terrain->set_color(IntToRow(open), IntToCol(open), Colors::Blue);
}

void AStarPather::ColorClosed(int closed)
{
    terrain->set_color(IntToRow(closed), IntToCol(closed), Colors::Yellow);
}