// RBotFirmware
// Rob Dobson 2016-2018

#pragma once

#include "MotionPipelineIF.h"
#include "MotionRingBuffer.h"
#include "MotionBlock.h"
#include <vector>

class MotionPipeline : public MotionPipelineIF
{
public:
    MotionPipeline() : _pipelinePosn(0)
    {
    }

    void setup(int pipelineSize)
    {
        _pipeline.resize(pipelineSize);
        _pipelinePosn.init(pipelineSize);
    }

    // Clear the pipeline
    virtual void clear() override final
    {
        _pipelinePosn.clear();
    }

    virtual unsigned int count() const override final
    {
        return _pipelinePosn.count();
    }

    unsigned int size() const
    {
        return _pipelinePosn.size();
    }

    virtual unsigned int remaining() const override final
    {
        return _pipelinePosn.remaining();
    }

    // Check if ready to accept data
    virtual bool canAccept() const override final
    {
        return _pipelinePosn.canPut();
    }

    // Add to pipeline
    virtual bool add(const MotionBlock &block) override final
    {
        // Check if full
        if (!_pipelinePosn.canPut())
            return false;

        // Add the item
        _pipeline[_pipelinePosn._putPos] = block;
        _pipelinePosn.hasPut();
        return true;
    }

    // Can get from queue (i.e. not empty)
    virtual bool IRAM_ATTR canGet() override final
    {
        return _pipelinePosn.canGet();
    }

    // Get from queue
    inline bool IRAM_ATTR get(MotionBlock &block)
    {
        // Check if queue is empty
        if (!_pipelinePosn.canGet())
            return false;

        // read the item and remove
        block = _pipeline[_pipelinePosn._getPos];
        _pipelinePosn.hasGot();
        return true;
    }

    // Remove last element from queue
    virtual bool IRAM_ATTR remove() override final
    {
        // Check if queue is empty
        if (!_pipelinePosn.canGet())
            return false;

        // remove item
        _pipelinePosn.hasGot();
        return true;
    }

    // Peek the block which would be got (if there is one)
    virtual MotionBlock* peekGet() override final;

    // Peek from the put position
    // 0 is the last element put in the queue
    // 1 is the one put in before that
    // returns NULL when nothing to peek
    virtual MotionBlock *peekNthFromPut(unsigned int N) override final
    {
        // Get index
        int nthPos = _pipelinePosn.getNthFromPut(N);
        if (nthPos < 0)
            return NULL;
        return &(_pipeline[nthPos]);
    }

    const MotionBlock *peekNthFromPutConst(unsigned int N) const
    {
        // Get index
        int nthPos = _pipelinePosn.getNthFromPut(N);
        if (nthPos < 0)
            return NULL;
        return &(_pipeline[nthPos]);
    }

    // Peek from the get position
    // 0 is the element next got from the queue
    // 1 is the one got after that
    // returns NULL when nothing to peek
    MotionBlock *peekNthFromGet(unsigned int N)
    {
        // Get index
        int nthPos = _pipelinePosn.getNthFromGet(N);
        if (nthPos < 0)
            return NULL;
        return &(_pipeline[nthPos]);
    }

    const MotionBlock *peekNthFromGetConst(unsigned int N) const
    {
        // Get index
        int nthPos = _pipelinePosn.getNthFromGet(N);
        if (nthPos < 0)
            return NULL;
        return &(_pipeline[nthPos]);
    }

    // Debug
    void debugShowBlocks(const AxesParams &axesParams) const override final
    {
        int elIdx = 0;
        bool headShown = false;
        for (int i = count() - 1; i >= 0; i--)
        {
            const MotionBlock *pBlock = peekNthFromPutConst(i);
            if (pBlock)
            {
                if (!headShown)
                {
                    pBlock->debugShowBlkHead();
                    headShown = true;
                }
                pBlock->debugShowBlock(elIdx++, axesParams);
            }
        }
    }

    // Debug
    void debugShowTopBlock(const AxesParams &axesParams)
    {
        int cnt = count();
        if (cnt == 0)
            return;
        MotionBlock *pBlock = peekNthFromPut(cnt-1);
        if (pBlock)
            pBlock->debugShowBlock(0, axesParams);
    }

private:
    MotionRingBufferPosn _pipelinePosn;
    std::vector<MotionBlock> _pipeline;
};
