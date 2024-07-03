/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftKinematics
//
// Rob Dobson 2016-2024
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftKinematics.h"
#include "KinematicsXYZ.h"
#include <list>

/// @brief RaftKinematics factory creator function
typedef RaftKinematics* (*RaftKinematicsFactoryCreatorFn)();

/// @brief RaftKinematics factory record type definition
class RaftKinematicsFactoryTypeDef
{
public:
    RaftKinematicsFactoryTypeDef(const String& name, RaftKinematicsFactoryCreatorFn createFn)
    {
        _name = name;
        _createFn = createFn;
    }
    bool isIdenticalTo(const RaftKinematicsFactoryTypeDef& other) const
    {
        if (!_name.equalsIgnoreCase(other._name))
            return false;
        return _createFn == other._createFn;
    }
    bool nameMatch(const String& name) const
    {
        return _name.equalsIgnoreCase(name);
    }
    String _name;
    RaftKinematicsFactoryCreatorFn _createFn;
};

/// @brief RaftKinematics system
class RaftKinematicsSystem
{
public:
    // Constructor / Destructor
    RaftKinematicsSystem()
    {
        // Register all kinematics
        registerKinematics("XYZ", KinematicsXYZ::create);
    }

    // Destructor
    ~RaftKinematicsSystem()
    {
    }

    /// @brief Register a kinematics type
    /// @param kinematicsConstrName Name of the kinematics type
    /// @param createFn Function to create a kinematics of this type
    void registerKinematics(const char* kinematicsConstrName, RaftKinematicsFactoryCreatorFn createFn)
    {
        // See if already registered
        RaftKinematicsFactoryTypeDef newElem(kinematicsConstrName, createFn);
        for (const RaftKinematicsFactoryTypeDef& el : _kinematicsFactoryTypeList)
        {
            if (el.isIdenticalTo(newElem))
                return;
        }

        // Add to list
        _kinematicsFactoryTypeList.push_back(newElem);
    }

    /// @brief Create a kinematics type
    /// @param name Name of the kinematics type
    /// @return RaftKinematics* pointer to the created kinematics
    static RaftKinematics* createKinematics(const char* name)
    {
        return getInstance().create(name);
    }

private:

    // Singleton instance accessor
    static RaftKinematicsSystem& getInstance()
    {
        static RaftKinematicsSystem instance;
        return instance;
    }

    // List of kinematics types that can be created
    std::list<RaftKinematicsFactoryTypeDef> _kinematicsFactoryTypeList;

    // Factory method
    RaftKinematics* create(const char* name)
    {
        // Find the kinematics type
        for (const RaftKinematicsFactoryTypeDef& el : _kinematicsFactoryTypeList)
        {
            if (el.nameMatch(name))
                return el._createFn();
        }
        return nullptr;
    }

    // Delete copy constructor and assignment operator to prevent copies
    RaftKinematicsSystem(const RaftKinematicsSystem&) = delete;
    RaftKinematicsSystem& operator=(const RaftKinematicsSystem&) = delete;
};
