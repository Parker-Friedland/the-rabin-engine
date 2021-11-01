#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
    return false;
}

float distance_to_closest_wall(int row, int col)
{
    /*
        Check the euclidean distance from the given cell to every other wall cell,
        with cells outside the map bounds treated as walls, and return the smallest
        distance.  Make use of the is_valid_grid_position and is_wall member
        functions in the global terrain to determine if a cell is within map bounds
        and a wall, respectively.
    */

    float closest = std::numeric_limits<float>::max();

    for(int r = -1; r <= terrain->get_map_height(); ++r)
        for(int c = -1; c <= terrain->get_map_width(); ++c)
            if (!terrain->is_valid_grid_position(r, c) || terrain->is_wall(r, c))
            {
                float dist = distance(row - r, col - c);
                if (dist < closest)
                    closest = dist;
            }
    
    return closest; // REPLACE THIS
}

float distance(int r, int c)
{
    return std::sqrtf(static_cast<float>(r * r) + static_cast<float>(c * c));
}

bool is_clear_path(int row0, int col0, int row1, int col1)
{
    /*
        Two cells (row0, col0) and (row1, col1) are visible to each other if a line
        between their centerpoints doesn't intersect the four boundary lines of every
        wall cell.  You should puff out the four boundary lines by a very tiny amount
        so that a diagonal line passing by the corner will intersect it.  Make use of the
        line_intersect helper function for the intersection test and the is_wall member
        function in the global terrain to determine if a cell is a wall or not.
    */
    
    const Vec2 start = Vec2(row0, col0), stop = Vec2(row1, col1);
    Vec2 topRight, topLeft, botRight, botLeft;

    for (int r = row0; r <= row1; ++r)
    {
        topLeft.y = topRight.y = static_cast<float>(r) + 0.5f;
        botLeft.y = botRight.y = static_cast<float>(r) - 0.5f;

        for (int c = col0; c <= col0; ++c)
        {
            topLeft.x = botLeft.x = static_cast<float>(c) + 0.5f;
            topRight.x = botRight.x = static_cast<float>(c) - 0.5f;

            if (terrain->is_wall(r, c)
                && line_intersect(start, stop, topLeft, topRight)
                && line_intersect(start, stop, topRight, botRight)
                && line_intersect(start, stop, botRight, botLeft)
                && line_intersect(start, stop, botLeft, topLeft))
            {
                return false;
            }
        }
    }

    return true;
}

void analyze_openness(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the value 1 / (d * d),
        where d is the distance to the closest wall or edge.  Make use of the
        distance_to_closest_wall helper function.  Walls should not be marked.
    */

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
        {
            float d = distance_to_closest_wall(r, c);
            layer.set_value(r, c, 1 / (d * d));
        }
}

void analyze_visibility(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the number of cells that
        are visible to it, divided by 160 (a magic number that looks good).  Make sure
        to cap the value at 1.0 as well.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
        {
            float v = visibility_num(r, c) / 160.f;
            if (v > 1.0)
                v = 1.0;
            layer.set_value(r, c, v);
        }
}

int visibility_num(int row, int col)
{
    /*
        Mark every cell in the given layer with the number of cells that
        are visible to it, divided by 160 (a magic number that looks good).  Make sure
        to cap the value at 1.0 as well.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    int num = 0;

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            if(is_clear_path(row, col, r, c))
                ++num;

    return num;
}

void analyze_visible_to_cell(MapLayer<float> &layer, int row, int col)
{
    /*
        For every cell in the given layer mark it with 1.0
        if it is visible to the given cell, 0.5 if it isn't visible but is next to a visible cell,
        or 0.0 otherwise.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            layer.set_value(r, c, 0.f);

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            if (is_clear_path(row, col, r, c))
            {
                layer.set_value(r, c, 1.f);

                for (int neighbor_r = r - 1; neighbor_r <= r + 1; ++neighbor_r)
                    for (int neighbor_c = c - 1; neighbor_c <= c + 1; ++neighbor_c)
                        if (terrain->is_valid_grid_position(neighbor_r, neighbor_c) 
                            && layer.get_value(neighbor_r, neighbor_c) == 0.f)
                            layer.set_value(r, c, 0.5f);
            }
}

void analyze_agent_vision(MapLayer<float> &layer, const Agent *agent)
{
    /*
        For every cell in the given layer that is visible to the given agent,
        mark it as 1.0, otherwise don't change the cell's current value.

        You must consider the direction the agent is facing.  All of the agent data is
        in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.

        Take the dot product between the view vector and the vector from the agent to the cell,
        both normalized, and compare the cosines directly instead of taking the arccosine to
        avoid introducing floating-point inaccuracy (larger cosine means smaller angle).

        Give the agent a field of view slighter larger than 180 degrees.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    const GridPos& pos = terrain->get_grid_position(agent->get_position());
    const Vec3& forward = agent->get_forward_vector();


    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            layer.set_value(r, c, visible_to_agent(pos, forward, 185.f, r, c));
}

bool visible_to_agent(const GridPos& pos, const Vec3& forward, float fov, int r, int c)
{
    Vec3 to_cell = Vec3(r - pos.row, c - pos.col, 0.f);
    to_cell.Normalize();

    if (forward.Dot(to_cell) < std::cosf(fov * (std::_Pi / 180.f)))
        return false;

    return is_clear_path(pos.row, pos.col, r, c);
}

void propagate_solo_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        For every cell in the given layer:

            1) Get the value of each neighbor and apply decay factor
            2) Keep the highest value from step 1
            3) Linearly interpolate from the cell's current value to the value from step 2
               with the growing factor as a coefficient.  Make use of the lerp helper function.
            4) Store the value from step 3 in a temporary layer.
               A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */
    
    float temp[40][40];

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            temp[r][c] = calculate_solo_occupancy(layer, r, c, decay);

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            layer.set_value(r, c, 
                (1.f - growth) * layer.get_value(r, c) + growth * temp[r][c]);
}

void propagate_dual_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        Similar to the solo version, but the values range from -1.0 to 1.0, instead of 0.0 to 1.0

        For every cell in the given layer:

        1) Get the value of each neighbor and apply decay factor
        2) Keep the highest ABSOLUTE value from step 1
        3) Linearly interpolate from the cell's current value to the value from step 2
           with the growing factor as a coefficient.  Make use of the lerp helper function.
        4) Store the value from step 3 in a temporary layer.
           A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */

    float temp[40][40];

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            temp[r][c] = calculate_dual_occupancy(layer, r, c, decay);

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            layer.set_value(r, c,
                (1.f - growth) * layer.get_value(r, c) + growth * temp[r][c]);
}

float calculate_solo_occupancy(const MapLayer<float>& layer, int r, int c, float decay)
{
    float max = 0.f;

    for (int r_offset = r - 1; r_offset <= r + 1; ++r_offset)
        for (int c_offset = c - 1; c_offset <= c + 1; ++c_offset)
            if (terrain->is_valid_grid_position(r + r_offset, c + c_offset))
            {
                float influence = layer.get_value(r + r_offset, c + c_offset)
                    * std::expf(r_offset * c_offset == 0.f ? decay : decay * std::sqrtf(2));
                if (influence > max)
                    max = influence;
            }

    return max;
}

float calculate_dual_occupancy(const MapLayer<float>& layer, int r, int c, float decay)
{
    float max = 0.f;
    bool negative = false;

    for (int r_offset = r - 1; r_offset <= r + 1; ++r_offset)
        for (int c_offset = c - 1; c_offset <= c + 1; ++c_offset)
            if (terrain->is_valid_grid_position(r + r_offset, c + c_offset))
            {
                float influence = std::abs(layer.get_value(r + r_offset, c + c_offset))
                    * std::expf(r_offset * c_offset == 0.f ? decay : decay * std::sqrtf(2));
                if (influence > max)
                {
                    max = influence;
                    negative = layer.get_value(r + r_offset, c + c_offset) < 0.f;
                }
            }

    return negative ? -max : max;
}

void normalize_solo_occupancy(MapLayer<float> &layer)
{
    /*
        Determine the maximum value in the given layer, and then divide the value
        for every cell in the layer by that amount.  This will keep the values in the
        range of [0, 1].  Negative values should be left unmodified.
    */

    float max = 0;

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            if (layer.get_value(r, c) > max)
                max = layer.get_value(r, c);

    if (max == 0)
        return;

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            layer.set_value(r, c, layer.get_value(r, c) / max);
}

void normalize_dual_occupancy(MapLayer<float> &layer)
{
    /*
        Similar to the solo version, but you need to track greatest positive value AND 
        the least (furthest from 0) negative value.

        For every cell in the given layer, if the value is currently positive divide it by the
        greatest positive value, or if the value is negative divide it by -1.0 * the least negative value
        (so that it remains a negative number).  This will keep the values in the range of [-1, 1].
    */

    float max = 0;
    float min = 0;

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
        {
            float value = layer.get_value(r, c);
            if (value > max)
                max = value;
            else if (value < min)
                min = value;
        }

    if (max == 0.f || min == 0.f)
        return;

    min *= -1;

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
        {
            float value = layer.get_value(r, c);
            if (value >= 0.f)
                layer.set_value(r, c, value / max);
            else
                layer.set_value(r, c, value / min);
        }
}

void enemy_field_of_view(MapLayer<float> &layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent *enemy)
{
    /*
        First, clear out the old values in the map layer by setting any negative value to 0.
        Then, for every cell in the layer that is within the field of view cone, from the
        enemy agent, mark it with the occupancy value.  Take the dot product between the view
        vector and the vector from the agent to the cell, both normalized, and compare the
        cosines directly instead of taking the arccosine to avoid introducing floating-point
        inaccuracy (larger cosine means smaller angle).

        If the tile is close enough to the enemy (less than closeDistance),
        you only check if it's visible to enemy.  Make use of the is_clear_path
        helper function.  Otherwise, you must consider the direction the enemy is facing too.
        This creates a radius around the enemy that the player can be detected within, as well
        as a fov cone.
    */

    const GridPos& pos = terrain->get_grid_position(enemy->get_position());
    const Vec3& forward = enemy->get_forward_vector();

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            if (layer.get_value(r, c) < 0.f)
                layer.set_value(r, c, 0.f);

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            if (in_enemy_range(pos, forward, fovAngle, closeDistance, r, c))
                layer.set_value(r, c, occupancyValue);
}

bool in_enemy_range(const GridPos& pos, const Vec3& forward, float fov, float closeDistance, int r, int c)
{
    return distance(pos.row - r, pos.col - c) < closeDistance
        ? is_clear_path(pos.row, pos.col, r, c)
        : visible_to_agent(pos, forward, fov, r, c);
}

bool enemy_find_player(MapLayer<float> &layer, AStarAgent *enemy, Agent *player)
{
    /*
        Check if the player's current tile has a negative value, ie in the fov cone
        or within a detection radius.
    */

    const auto &playerWorldPos = player->get_position();

    const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

    // verify a valid position was returned
    if (terrain->is_valid_grid_position(playerGridPos) == true)
    {
        if (layer.get_value(playerGridPos) < 0.0f)
        {
            return true;
        }
    }

    // player isn't in the detection radius or fov cone, OR somehow off the map
    return false;
}

bool enemy_seek_player(MapLayer<float> &layer, AStarAgent *enemy)
{
    /*
        Attempt to find a cell with the highest nonzero value (normalization may
        not produce exactly 1.0 due to floating point error), and then set it as
        the new target, using enemy->path_to.

        If there are multiple cells with the same highest value, then pick the
        cell closest to the enemy.

        Return whether a target cell was found.
    */

    float max = 0.f;
    int max_r = 0;
    int max_c = 0;

    for (int r = 0; r < terrain->get_map_height(); ++r)
        for (int c = 0; c < terrain->get_map_width(); ++c)
            if (layer.get_value(r, c) > max)
            {
                max = layer.get_value(r, c);
                max_r = r;
                max_c = c;
            }

    if (max == 0)
        return false;

    enemy->path_to(terrain->get_world_position(max_r, max_c));
    return true;
}
