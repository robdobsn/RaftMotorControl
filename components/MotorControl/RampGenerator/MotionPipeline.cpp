/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionPipeline.cpp
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MotionPipeline.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Peek at next item in pipeline
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MotionBlock* FUNCTION_DECORATOR_IRAM_ATTR MotionPipeline::peekGet()
{
    // Check if queue is empty
    if (!_pipelinePosn.canGet())
        return NULL;
    // get pointer to the last item (don't remove)
    return &(_pipeline[_pipelinePosn._getPos]);
}
