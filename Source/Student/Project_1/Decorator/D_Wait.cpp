#include <pch.h>
#include "D_Wait.h"

D_Wait::D_Wait(float delay) : delay(delay), timer(0.0f)
{}

void D_Wait::on_enter()
{
    timer = delay;

    BehaviorNode::on_enter();
}

void D_Wait::on_update(float dt)
{
    timer -= dt;

    if (timer < 0.0f)
    {
        BehaviorNode *child = children.front();

        child->tick(dt);

        // assume same status as child
        set_status(child->get_status());
        set_result(child->get_result());
    }
}
