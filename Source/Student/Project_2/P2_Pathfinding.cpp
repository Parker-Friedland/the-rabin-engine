#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#define _USE_MATH_DEFINES

#include <cmath>
#include <numbers>
#include <algorithm>

struct Node
{
    Node(GridPos pos, Heuristic h, GridPos goal) : 
        _pos(pos), _parent(nullptr), _g(0), _f(AStarPather::Distance(h, pos, goal)) {}

    Node(GridPos pos, Node* parent, Heuristic h, GridPos goal) : 
        _pos(pos), _parent(nullptr), _g(parent->_g + 1), _f(_g + AStarPather::Distance(h, pos, goal)) {}

    bool IsOpen(MAP m)
    {
        _f == m.find(_pos)->second._f;
    }

    GridPos _pos;
    Node* _parent;
    float _g; // given cost
    float _f; // given cost + heuristic cost

    bool operator<(const Node& rhs) const
    {
        return _f < rhs._f;
    }

    bool operator==(const Node& rhs) const
    {
        return _pos == rhs._pos;
    }

    bool operator!=(const Node& rhs) const
    {
        return _pos != rhs._pos;
    }
};

typedef std::priority_queue<Node, std::vector<Node>, std::less<Node>> QUEUE;
typedef std::unordered_map<GridPos, Node> MAP;

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

    GridPos start = terrain->get_grid_position(request.start);
    GridPos goal = terrain->get_grid_position(request.goal);

    QUEUE openList = QUEUE(std::less<Node>(), std::vector<Node>());
    MAP allNodes = MAP();

    Heuristic h = request.settings.heuristic;

    openList.emplace(start, h, goal);

    while (!openList.empty())
    {
        Node curr = openList.top();
        openList.pop();

        if (curr.IsOpen(allNodes))
        {
            if (curr._pos == goal)
                return PathResult::COMPLETE;

            GridPos next = curr._pos;

            next.col += 1;
            bool right = terrain->is_valid_grid_position(next) && !terrain->is_wall(next);
            if (right)
                openList.emplace(next, curr, h, goal);

            next.col -= 2;
            bool left = terrain->is_valid_grid_position(next) && !terrain->is_wall(next);
            if (left)
                openList.emplace(next, curr, h, goal);
            next.col += 1;

            next.row += 1;
            bool up = terrain->is_valid_grid_position(next) && !terrain->is_wall(next);
            if (up)
                openList.emplace(next, curr, h, goal);

            next.row -= 2;
            bool down = terrain->is_valid_grid_position(next) && !terrain->is_wall(next);
            if (down)
                openList.emplace(next, curr, h, goal);
            next.row += 1;

            if (right && up)
            {
                ++next.row;
                ++next.col;

                if(!terrain->is_wall(next))
                    openList.emplace(next, curr, h, goal);

                --next.row;
                --next.col;
            }

            if (left && up)
            {
                ++next.row;
                --next.col;

                if (!terrain->is_wall(next))
                    openList.emplace(next, curr, h, goal);

                --next.row;
                ++next.col;
            }
        }

        
    }

    //open
    
    // Just sample code, safe to delete
    //GridPos start = terrain->get_grid_position(request.start);
    //GridPos goal = terrain->get_grid_position(request.goal);
    terrain->set_color(start, Colors::Orange);
    terrain->set_color(goal, Colors::Orange);
    request.path.push_back(request.start);
    request.path.push_back(request.goal);
    return PathResult::COMPLETE;
}

float calculateH(Heuristic hType)
{

}
