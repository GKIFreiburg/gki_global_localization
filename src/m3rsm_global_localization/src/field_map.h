#ifndef FIELD_MAP_
#define FIELD_MAP_H 

#include <vector>
#include <math.h> 
#include <iostream>
// For ROS_ASSERT
#include "ros/assert.h"

// A likelyhood fieldmap represents the likelyhood to perceive an obstacle at
// the given map position. It is based on a real map, but may have a different
// (coarser) resolution. 
class LikelyHoodFieldMap {

public:

    // Default constructor
    LikelyHoodFieldMap(size_t const x = 0, size_t const y = 0, 
            int const _resolution = 1) : 
        resolution(_resolution)
        {
            max_x = ceil( (double) x / (double) resolution);
            max_y = ceil( (double) y / (double) resolution);
            grid.resize(max_x * max_y);
        }

    // Transform an x or y point to the point it would be in the original map
    int valueWithoutResolution(int const& value) const;
    // Transform an x or y point from the original map to a point in the grid
    int applyResolution(int const& value) const;

    void toPGM(std::string const& filename) const;

    inline const int getResolution() const {
        return resolution;
    }

    inline const size_t xSize() const {
        return max_x;
    } 

    inline const size_t ySize() const {
        return max_y;
    } 

    inline const double getEntry(size_t x, size_t y) const {
        size_t i = getIndex(x,y);
        ROS_ASSERT(i < grid.size());
        return grid[i];
    }

    inline void setEntry(double e, size_t x, size_t y) {
        size_t i = getIndex(x,y);
        ROS_ASSERT(i < grid.size());
        grid[x + max_x * y] = e;
    }

    inline const size_t gridSize() const {
        return grid.size();
    }

private: 
    // Map related things
    int resolution;
    size_t max_x;
    size_t max_y;

    // Grid represented as a 1d array for memory locality
    std::vector<double> grid;

    const size_t getIndex(int x, int y) const {
        return (x + y * max_x);
    }
};

#endif /* FIELD_MAP_H */
