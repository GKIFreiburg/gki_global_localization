#include "likelihood_field.h"
#include "map.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm> 

using std::vector;

tf::Pose LikelyHoodField::likelyHoodFieldModel(const LaserData& data) {
    auto const& coarsest = fieldMaps.back(); 
    constexpr int theta_steps = 2;
    for (double theta = 0; theta < 360; theta += theta_steps) {
        vector<tf::Vector3> projectedPoses = projectData(theta, data);
        rotatedPoses.push_back(projectedPoses);
        double prob = projectedPoses.size() * coarsest.getEntry(0,0);
        size_t xt0 = 0;
        size_t xt1 = fieldMaps[0].xSize() - 1;
        size_t yt0 = 0;
        size_t yt1 = fieldMaps[0].ySize() - 1;
        // Since the coarsest map has only 1 cell all poses have the same
        // probability
        auto max_resolution = coarsest.getResolution();
        int poseindex = rotatedPoses.size() - 1;
        Entry entry{prob, xt0, xt1, yt0, yt1, max_resolution, poseindex};
        heap.push(entry);
    }
    Entry best = extractProbability();
    //TODO input has to be tfScalary 
    tf::Vector3 position;
    position.setX(MAP_WXGX(occupancyGrid, (int) best.xt0));
    position.setY(MAP_WYGY(occupancyGrid, (int) best.yt0));
    // Since we save the poses for each theta we can compute theta by dividing
    // the poseindex by #theta_steps
    tf::Quaternion orientation = tf::createQuaternionFromYaw(
            (double) (best.poseindex * theta_steps) / 180.00 * M_PI);
    tf::Pose result(orientation, position);
    ROS_INFO("Global pose is: %.3f %.3f %.3f",
            result.getOrigin().x(),
            result.getOrigin().y(),
            tf::getYaw(orientation));
    return result;
}

const Entry LikelyHoodField::extractProbability() {
    Entry entry = heap.top();
    heap.pop();
    while (entry.resolution != 1) {
        std::cout << "Current Heap size: " << heap.size() << std::endl;
        std::cout << "Retrieved element with probabability " << entry.probability 
            << " and resolution/translation " << entry.resolution << "/ x:" 
            << entry.xt0 << "," << entry.xt1 << " y: " << entry.yt0 << "," 
            << entry.yt1 << std::endl;
        split(entry);
        entry = heap.top();
        heap.pop();
    } 
    //std::cout << "Retrieved element with probabability " << entry.probability 
    //    << " and resolution/translation " << entry.resolution << "/ x:" 
    //    << entry.xt0 << "," << entry.xt1 << " y: " << entry.yt0 << "," 
    //    << entry.yt1 << " theta/2: " << entry.poseindex << std::endl;
    return entry;
}

void LikelyHoodField::split(Entry const& entry) {
    auto diff_x = (entry.xt1 - entry.xt0) / 2;
    auto diff_y = (entry.yt1 - entry.yt0) / 2;
    // Create four new entries, with one quarter of the translation (x/2,y/2)
    // s,t is the start of x,y respectively
    // Entry for x=[s,x/2] y=[t,y/2]
    computeEntry(entry.xt0, entry.xt1 - diff_x, 
            entry.yt0, entry.yt1 - diff_y, entry);

    // Entry for x=[x/2, x] y=[t,y/2]
    computeEntry(entry.xt1 - diff_x + 1, entry.xt1, 
            entry.yt0, entry.yt1 - diff_y, entry);

    // Entry for x=[s,x/2] y=[y/2, y]
    computeEntry(entry.xt0, entry.xt1 - diff_x, 
            entry.yt1 - diff_y + 1, entry.yt1, entry);

    // Entry for x=[x/2, x] y=[y/2,y ]
    computeEntry(entry.xt1 - diff_x + 1, entry.xt1, 
            entry.yt1 - diff_y + 1, entry.yt1, entry);
}

void LikelyHoodField::computeEntry(size_t xt0, size_t xt1, 
        size_t yt0, size_t yt1, Entry const& old) {
    double prob = 0;
    int const newRes = old.resolution / 2;
    int const mapindex = log2(old.resolution) - 1;
    size_t const maxX = fieldMaps[mapindex].xSize();
    size_t const maxY = fieldMaps[mapindex].ySize();
    for (tf::Vector3 const& pose : rotatedPoses[old.poseindex]) {
        // Translate and decimate the poses
        size_t decimated_x = (pose.getX() + xt0) / newRes;
        size_t decimated_y = (pose.getY() + yt0) / newRes;
        if (decimated_x >= maxX || decimated_y >= maxY) {
            // If the translated and decimated points do not lie in the map
            // these translations are invalid and we don't have to insert a new
            // entry
            return;
        }
        prob += fieldMaps[mapindex].getEntry(decimated_x, decimated_y);
    }
    Entry finer(prob, xt0, xt1, yt0, yt1, newRes, old.poseindex);
    heap.push(finer);
}

inline bool LikelyHoodField::outOfBounds(const int& x, const int& y) const {
    return (occupancyGrid->cells[MAP_INDEX(occupancyGrid,x,y)].occ_state != -1);
}

vector<tf::Vector3> LikelyHoodField::projectData(double const& theta, 
        LaserData const& data) const {
    vector<tf::Vector3> projectedPoses;
    tf::Vector3 position;
    position.setX(MAP_WXGX(occupancyGrid, 0));
    position.setY(MAP_WYGY(occupancyGrid, 0));
    // Since we save the poses for each theta we can compute theta by dividing
    // the poseindex by #theta_steps
    tf::Quaternion orientation = tf::createQuaternionFromYaw(theta / 180 * M_PI);
    tf::Pose pose(orientation, position);

    for (tf::Vector3 point : data.ranges) {
        tf::Vector3 projectedPoint = pose * point;
        projectedPoint.setX(MAP_GXWX(occupancyGrid, projectedPoint.getX()));
        projectedPoint.setY(MAP_GYWY(occupancyGrid, projectedPoint.getY()));
        projectedPoses.push_back(projectedPoint);
    }
    return projectedPoses;
}

void LikelyHoodField::initializeLikelyHoodFieldMap() {
    double const z_hit_denom = 2.0 * sigma_hit * sigma_hit;
    double const z_hit_mult = 1.0 / sqrt(2 * M_PI * sigma_hit);
    auto const randomNoise =  z_rand / max_range;
    auto max_dist = occupancyGrid->max_occ_dist;
    std::cout << "computing likelyhood field..." << std::endl;

    for (int y = 0; y < fieldMaps[0].ySize(); ++y) {
        for (int x = 0; x < fieldMaps[0].xSize(); ++x) {
            auto pose_x = fieldMaps[0].valueWithoutResolution(x);
            auto pose_y = fieldMaps[0].valueWithoutResolution(y);
            auto distToObstacle = distanceToNearestObstacle(pose_x, pose_y);
            auto gaussNoise = z_hit * z_hit_mult * 
                exp(-(distToObstacle * distToObstacle) / z_hit_denom);
            // Likelihood fields contain log probabilities, see Wikipedia for
            // further information on log probabilities
            fieldMaps[0].setEntry(-log(gaussNoise + randomNoise), x, y);
        }
    }
    std::cout << "Done!" << std::endl;
    std::cout << "Computing coarse resolution maps...!" << std::endl;
    // Stepwise initialization of coarser maps, until we can represent the whole 
    // grid as a single point in the map
    while (fieldMaps.back().gridSize() != 1) {
        constructCoarserMap(fieldMaps.back());
    }
    std::cout << "Done!" << std::endl;
}

void LikelyHoodField::constructCoarserMap(LikelyHoodFieldMap const& source) {
    LikelyHoodFieldMap map(fieldMaps[0].xSize(), fieldMaps[0].ySize(), 
            2 * source.getResolution());
    std::cout << "Computing resolution " << 2 * source.getResolution() 
        << " with size (" << map.xSize() << "," << map.ySize() << ")"
        << std::endl;

    for (size_t y = 0; y < source.ySize(); y+=2) {
        for (size_t x = 0; x < source.xSize(); x+=2) {
            // Convolution kernel of 3: Project a 3x3 grid around the original
            // point and compute the maximum value
            vector<size_t> xCoords = {x-1, x, x+1};
            vector<size_t> yCoords = {y-1, y, y+1};
            vector<double> values;
            for (auto const& x_cord : xCoords) {
                for (auto const& y_cord : yCoords) {
                    // Note that values < 0 overflow to max size_t value
                    if (x_cord < source.xSize() && y_cord < source.ySize()) {
                        values.push_back(source.getEntry(x_cord,y_cord));
                    }
                }
            }
            // Note that the max element has the lowest log probability, thus
            // we check for the smallest element
            auto max = std::min_element(std::begin(values), std::end(values));
            map.setEntry(*max, x/2, y/2);
        }
    }
    fieldMaps.push_back(map);
}


void LikelyHoodField::toPGM(std::string const& filename) {
    int i = 1;
    for (auto const& fieldMap : fieldMaps) {
        fieldMap.toPGM(std::to_string(i) + filename);
        ++i;
    }
}

inline double LikelyHoodField::distanceToNearestObstacle(int const& x, 
        int const& y) const {
    return occupancyGrid->cells[MAP_INDEX(occupancyGrid,x,y)].occ_dist;
}
