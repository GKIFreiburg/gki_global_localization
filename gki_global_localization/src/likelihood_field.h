#ifndef LIKELIHOOD_FIELD_H
#define LIKELIHOOD_FIELD_H 

#include "field_map.h"
#include "laser.h"
#include "map.h"
#include <vector>
#include <queue>
#include <visualization_msgs/Marker.h>


// Each entry corresponds to a rotation (given the pose index), a range of
// translations and a probability of the likelihood of this entry. Translations
// are stored by the first translation in x and y direction (x0,y0) and the last
// translation in x and y direction (x1,y1)
struct Entry {
        Entry(double const& prob, size_t const& x0, size_t const& x1,
                size_t const& y0, size_t const& y1,
                int const& res, int const& _poseindex) :
            probability(prob),
            xt0(x0),xt1(x1),yt0(y0),yt1(y1),
            resolution(res),
            poseindex(_poseindex) {}

    double probability;
    // Start and end of the x,y translations
    size_t xt0;
    size_t xt1;
    size_t yt0;
    size_t yt1;
    int resolution;
    // index to get the rotated poses for this entry
    int poseindex;

    bool operator<(Entry const& other) const {
        // Note that an entry has a higher probability if its log probability
        // is smaller.
        return probability > other.probability;
    }

};

// Generates a likelihood field map out of a occupancy grid. This can be used
// to get precomputed p(z|x,m) values for other applications. For further
// information see the book Probabilistic Robotics, P. 169ff. 

class LikelyHoodField {
public:

    LikelyHoodField(map_t* map, double const& _sigma_hit, 
          double const& _z_rand, double const& _z_hit, double const& _max_range) :
        occupancyGrid(map),
        sigma_hit(_sigma_hit),
        z_rand(_z_rand),
        z_hit(_z_hit),
        max_range(_max_range) {
            LikelyHoodFieldMap fieldmap(map->size_x, map->size_y);
            fieldMaps.push_back(fieldmap);
        }

    // Computes the pose with the highest likelihood, given scan data
    tf::Pose likelyHoodFieldModel(const LaserData& data);

    // Projects the laser data, given an angle theta and the pose of the laser
    std::vector<tf::Vector3> projectData(double const& theta, 
            LaserData const& data) const;

    const Entry extractProbability();
    // Creates four new entries, each with one quarter of the translation
    void split(Entry const& entry);
    // Create an entry for the given translation range in x/y direction
    void computeEntry(size_t xt0, size_t xt1, 
        size_t yt0, size_t yt1, Entry const& old);

    // If x and y lie withing the bounds of the map
    inline bool outOfBounds(const int& x, const int& y) const;

    // Given a laser scan, compute the end point of that scan in the map
    tf::Vector3 projectScan(const tf::Vector3& pose, double const& obs_range, 
            double const& obs_bearing) const;

    // Initializes the likelyhood field map used to lookup the probabilities
    void initializeLikelyHoodFieldMap();
    // Constructs a map with half the resolution, but with a 3x3 convolution
    // kernel for each point
    void constructCoarserMap(LikelyHoodFieldMap const& source);

    // Safes all likelyhood maps as pgm files
    void toPGM(std::string const& filename);
    
private:
    // ground truth map
    map_t* occupancyGrid;
    // Laser model parameters
    double sigma_hit;
    double z_rand;
    double z_hit;
    double max_range;
    // A list of fieldmaps, each fieldmap coarser than the one before. The first
    // entry is the likelihood field with resolution 1, while each subsequenting
    // fieldmap has half the resolution. An entry in a fieldmap is the maximum
    // over a 3x3 grid of entries in the fieldmap with the higher resolution. 
    // For more information see the paper "M3RSM: Many-toMany Multi-Resolution
    // Scan Matching".
    std::vector<LikelyHoodFieldMap> fieldMaps;

    // Entries are inserted in a priority queue, because we always extract the
    // element with the highest probability
    std::priority_queue<Entry> heap;
    // We rotate the poses one time for each angle and index them 
    std::vector<std::vector<tf::Vector3> > rotatedPoses;

    // Computes the distance from this pose to the nearest obstacle
    double distanceToNearestObstacle(int const& pose_x, int const& pose_y) const;

};

#endif /* LIKELIHOOD_FIELD_H */
