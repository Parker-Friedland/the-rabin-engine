#pragma once
#include "BehaviorNode.h"

class D_Wait : public BaseNode<D_Wait>
{
public:
    D_Wait(float delay);

protected:
    float delay;
    float timer;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};