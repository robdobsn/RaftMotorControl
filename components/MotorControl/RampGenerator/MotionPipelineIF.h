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

    // Add
    virtual bool add(const MotionBlock &block) = 0;

    // Can get
    virtual bool FUNCTION_DECORATOR_IRAM_ATTR canGet() = 0;

    // Peek Nth element from the put position
    virtual MotionBlock *peekNthFromPut(unsigned int N) = 0;

    // Peek Nth element from the get position
    virtual MotionBlock *peekNthFromGet(unsigned int N) = 0;

    // Count
    virtual unsigned int count() const = 0;

    // CanAccept
    virtual bool canAccept() const = 0;

    // Remaining
    virtual unsigned int remaining() const = 0;

    // Debug
    virtual void debugShowBlocks(const AxesParams &axesParams) const
    {
    }
};
