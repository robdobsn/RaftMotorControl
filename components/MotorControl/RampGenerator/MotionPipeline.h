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
#if USE_SINGLE_SPLIT_BLOCK
        // For split-blocks, check effective capacity including sub-blocks
        return canAcceptSplitAware(1);
#else
        return _pipelinePosn.canPut();
#endif
    }

#if USE_SINGLE_SPLIT_BLOCK
    // Split-block aware capacity check
    // Counts split-blocks as their sub-block count for capacity planning
    bool canAcceptSplitAware(unsigned int additionalBlocks) const
    {
        if (!_pipelinePosn.canPut())
            return false;
            
        // Calculate effective usage including sub-blocks
        unsigned int effectiveUsed = 0;
        unsigned int totalBlocks = count();
        
        for (unsigned int i = 0; i < totalBlocks; i++)
        {
            const MotionBlock* block = peekNthFromPutConst(i);
            if (block && block->isSplitBlock())
            {
                // Count remaining sub-blocks
                unsigned int remaining = block->getTotalSubBlocks() - block->getCurrentSubBlock();
                effectiveUsed += remaining;
            }
            else
            {
                effectiveUsed += 1;
            }
        }
        
        return (effectiveUsed + additionalBlocks) <= _pipelinePosn.size();
    }
    
    // Get pointer to last added block (for configuring split metadata)
    // Returns NULL if pipeline is empty
    // WARNING: Only call immediately after add() before any ISR can remove blocks
    MotionBlock* getLastAddedBlock()
    {
        return peekNthFromPut(0); // 0 = last block added
    }
#endif

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
    virtual bool MOTOR_TICK_FN_DECORATOR canGet() override final
    {
        return _pipelinePosn.canGet();
    }

    // Get from queue
    inline bool MOTOR_TICK_FN_DECORATOR get(MotionBlock &block)
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
    // Note: For split-blocks, this is called only when all sub-blocks are complete
    virtual bool MOTOR_TICK_FN_DECORATOR remove() override final
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
