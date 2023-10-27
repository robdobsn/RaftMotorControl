/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionPipelineIF
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "MotionPipelineIF.h"
#include "MotionBlock.h"

class MotionPipelineIF
{
public:
    // Peek the block which would be got (if there is one)
    virtual MotionBlock* peekGet() = 0;

    // Remove last element from queue
    virtual bool remove() = 0;

    // Clear the pipeline
    virtual void clear() = 0;
};
