/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// GeometryManager
//
// Rob Dobson 2016-2024
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxisGeomBase.h"
#include "AxisGeomXYZ.h"
#include <map>

class GeometryManager
{
public:
    // Constructor / Destructor
    GeometryManager()
    {
        // Add default geometry
        AxisGeomBase* pAxisGeom = new AxisGeomXYZ();
        registerGeometry("XYZ", pAxisGeom);
        _axisGeomToDelete.push_back(pAxisGeom);

        // Set default geometry
        setGeometry("XYZ");
    }

    // Destructor
    ~GeometryManager()
    {
        // Delete all geometries that we added in constructor
        for (auto pAxisGeom : _axisGeomToDelete)
            delete pAxisGeom;
    }

    // Register a geometry
    void registerGeometry(const char* pGeometryName, AxisGeomBase* pAxisGeom)
    {
        _axisGeomMap[std::string(pGeometryName)] = pAxisGeom;
    }

    // Set a geometry
    bool setGeometry(const char* pGeometryName)
    {
        // Find the geometry
        std::map<std::string, AxisGeomBase*>::iterator it = _axisGeomMap.find(std::string(pGeometryName));
        if (it == _axisGeomMap.end())
            return false;

        // Set the geometry
        AxisGeomBase* pAxisGeom = it->second;
        if (pAxisGeom)
        {
            _pAxisGeom = pAxisGeom;
            return true;
        }
        return false;
    }

    // Get current geometry
    const AxisGeomBase* getGeometry() const
    {
        return _pAxisGeom;
    }

private:
    // Map of axis geometry
    std::map<std::string, AxisGeomBase*> _axisGeomMap;

    // Geometries to delete
    std::vector<AxisGeomBase*> _axisGeomToDelete;

    // Current axis geometry
    AxisGeomBase* _pAxisGeom = nullptr;
};
