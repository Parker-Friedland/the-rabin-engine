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

//typedef AStarPather::Node Node;
//typedef AStarPather::QUEUE QUEUE;
//typedef AStarPather::ITER ITER;

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

        if (_debugColor)
            ColorInit();
    }

    while (!_openList.empty())
    {
        Node curr = _openList.top();
        _openList.pop();

        if (curr.IsOpen(_allNodes))
        {
            if (curr._pos == _goal)
            {
                FinishRequest(request, curr);
                return PathResult::COMPLETE;
            }

            ProcessRequest(curr);

            if (_debugColor)
                ColorClosed(curr._pos);
        }
    }
    
    // Just sample code, safe to delete
    //GridPos start = terrain->get_grid_position(request.start);
    //GridPos goal = terrain->get_grid_position(request.goal);
    terrain->set_color(_start, Colors::Orange);
    terrain->set_color(_goal, Colors::Orange);
    request.path.push_back(request.start);
    request.path.push_back(request.goal);
    return PathResult::COMPLETE;
}

void AStarPather::InitRequest(const PathRequest& request)
{
    _debugColor = request.settings.debugColoring;
    _h = request.settings.heuristic;

    _start = terrain->get_grid_position(request.start);
    _goal = terrain->get_grid_position(request.goal);

    _allNodes.clear();
    while (!_openList.empty())
        _openList.pop(); // The priority queue's container is a vector that has a
                         // clear method so I shouldn't have to do this in O(n). 
    _openList.emplace(_start, _h, _goal);
    _allNodes.emplace(_start, _openList.top());
}

void AStarPather::ProcessRequest(Node& curr)
{
    GridPos next = curr._pos;

    next.col += 1;
    bool right = terrain->is_valid_grid_position(next) && !terrain->is_wall(next);
    if (right)
        AddAdj(curr, next);

    next.col -= 2;
    bool left = terrain->is_valid_grid_position(next) && !terrain->is_wall(next);
    if (left)
        AddAdj(curr, next);
    next.col += 1;

    next.row += 1;
    bool up = terrain->is_valid_grid_position(next) && !terrain->is_wall(next);
    if (up)
        AddAdj(curr, next);

    next.row -= 2;
    bool down = terrain->is_valid_grid_position(next) && !terrain->is_wall(next);
    if (down)
        AddAdj(curr, next);

    if (right && up)
    {
        ++next.row;
        ++next.col;

        if (!terrain->is_wall(next))
            AddDiag(curr, next);

        --next.row;
        --next.col;
    }

    if (left && up)
    {
        ++next.row;
        --next.col;

        if (!terrain->is_wall(next))
            AddDiag(curr, next);

        --next.row;
        ++next.col;
    }

    if (left && down)
    {
        --next.row;
        --next.col;

        if (!terrain->is_wall(next))
            AddDiag(curr, next);

        ++next.row;
        ++next.col;
    }

    if (right && down)
    {
        --next.row;
        ++next.col;

        if (!terrain->is_wall(next))
            AddDiag(curr, next);
    }
}

void AStarPather::FinishRequest(PathRequest& request, const Node& goal)
{
    const Node* curr = &goal;
    do
    {
        request.path.push_back(terrain->get_world_position(curr->_pos));
    } while (curr = curr->_parent);
}

void AStarPather::ColorInit()
{
    terrain->set_color(_start, Colors::Orange);
    terrain->set_color(_goal, Colors::Orange);
}

void AStarPather:: ColorOpen(const GridPos& open)
{
    terrain->set_color(open, Colors::Blue);
}

void AStarPather::ColorClosed(const GridPos& closed)
{
    terrain->set_color(closed, Colors::Yellow);
}