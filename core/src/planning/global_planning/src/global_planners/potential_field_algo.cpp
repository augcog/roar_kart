#include "global_planning/planners/potential_field_algo.hpp"
#include <iostream>
#include <stack>
#include <map>
#include <algorithm>
#include <queue>

using namespace ROAR::global_planning;

PotentialFieldPlanning::PotentialFieldPlanning(uint64_t nx, uint64_t ny, uint64_t max_iter)
{
    this->nx = nx;
    this->ny = ny;
    this->max_iter = max_iter;
    this->map_ = std::make_shared<std::vector<float>>(nx * ny);
}

PotentialFieldPlanning::~PotentialFieldPlanning()
{
}

void PotentialFieldPlanning::setObstacleCoords(std::vector<std::tuple<uint64_t, uint64_t>> obstacle_map)
{
    // for every x, y, calculate its position on the map, and set the value to 1
    std::vector<uint8_t> obs_map_linear(nx * ny, 0);

    // set map

    for (auto &point : obstacle_map)
    {
        uint64_t index = this->get_index(point);
        obs_map_linear[index] = 100;
    }
    this->setObstacles(obs_map_linear);
}

int PotentialFieldPlanning::setObstacles(std::vector<uint8_t> obstacle_map)
{
    // check if obstacle_map is same size as map
    if (obstacle_map.size() != nx * ny)
    {
        throw std::invalid_argument("obstacle_map size does not match map size");
    }

    // set map
    map_ = std::make_shared<std::vector<float>>(std::vector<float>(obstacle_map.begin(), obstacle_map.end()));
    int num_obstacles = 0;
    for (uint64_t i = 0; i < map_->size(); i++)
    {
        if ((*map_)[i] > OBSTACLE_THRESHOLD)
        {
            num_obstacles++;
            (*map_)[i] = 99999999999;
        }
    }
    return num_obstacles;
}

int PotentialFieldPlanning::setObstacles(std::vector<int8_t> obstacle_map)
{
    // check if obstacle_map is same size as map
    if (obstacle_map.size() != nx * ny)
    {
        throw std::invalid_argument("obstacle_map size does not match map size");
    }

    int num_obstacles = 0;
    for (uint64_t i = 0; i < map_->size(); i++)
    {
        if (obstacle_map[i] > 0)
        {
            num_obstacles++;
            (*map_)[i] = 9999;
        }

        if ((*map_)[i] > 100) {
            // get coord
            auto coord = getCoordFromIndex(i);
            uint64_t x = std::get<0>(coord);
            uint64_t y = std::get<1>(coord);
            
        }
    }
    return num_obstacles;
}

void PotentialFieldPlanning::inflateObstacles(int radius, float weight)
{
    if (this->map_ == nullptr)
    {
        throw std::invalid_argument("map is not set");
    }

    std::vector<uint64_t> obstacle_indices;
    for (uint64_t i = 0; i < map_->size(); i++)
    {
        if ((*map_)[i] > OBSTACLE_THRESHOLD)
        {
            obstacle_indices.push_back(i);
        }
    }

    // for every obstacle, inflate
    for (int i = 0; i < obstacle_indices.size(); i++)
    {
        uint64_t index = obstacle_indices[i];
        uint64_t x = index % nx;
        uint64_t y = index / nx;

        for (int dx = -radius; dx <= radius; dx++)
        {
            for (int dy = -radius; dy <= radius; dy++)
            {
                int64_t new_x = x + dx;
                int64_t new_y = y + dy;

                if (new_x >= 0 && new_x < nx && new_y >= 0 && new_y < ny)
                {
                    uint64_t new_index = get_index(std::make_tuple(new_x, new_y));
                    float distance = sqrt(dx * dx + dy * dy);
                    float new_value = (*map_)[new_index] + (*map_)[index] * weight / (distance + 1);
                    (*map_)[new_index] = new_value;
                }
            }
        }
    }
}

bool PotentialFieldPlanning::setStart(std::tuple<uint64_t, uint64_t> start)
{
    // check if start is valid
    // check if start is within bounds
    uint64_t x = std::get<0>(start);
    uint64_t y = std::get<1>(start);

    if (x < 0 || x >= nx || y < 0 || y >= ny)
    {
        return false;
    }
    this->start = std::make_shared<std::tuple<uint64_t, uint64_t>>(start);

    return true;
}

PotentialFieldPlanning::PotentialFieldPlanningResult PotentialFieldPlanning::plan(const PotentialFieldPlanning::PotentialFieldPlanningInput::SharedPtr input)
{
    PotentialFieldPlanningResult result;
    result.status = false;

    // check if goal is within bound
    uint64_t gx = std::get<0>(input->goal);
    uint64_t gy = std::get<1>(input->goal);
    if (gx < 0 || gx >= nx || gy < 0 || gy >= ny)
    {
        return result;
    }

    // check if map exist
    if (map_ == nullptr)
    {
        return result;
    }

    // check if start exist
    if (start == nullptr)
    {
        return result;
    }

    uint64_t num_obstacles = 0;
    for (uint64_t i = 0; i < nx * ny; i++)
    {
        if ((*map_)[i] >= OBSTACLE_THRESHOLD)
        {
            num_obstacles++;
        }
    }
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ParkingPlanner"), "[plan] num_obstacles: " << num_obstacles);

    // generate costmap
    std::shared_ptr<std::vector<float>> costmap = p_generateCostMapFromGoal(input->goal);
    result.costmap = costmap;

    // greedy search
    std::shared_ptr<std::vector<std::tuple<uint64_t, uint64_t>>> path = std::make_shared<std::vector<std::tuple<uint64_t, uint64_t>>>();

    p_greedySearch(path, *start, input->goal, max_iter, nx, ny, input->goal_threshold, costmap);

    result.path = path;
    result.status = true;
    return result;
}

bool PotentialFieldPlanning::p_greedySearch(std::shared_ptr<std::vector<std::tuple<uint64_t, uint64_t>>> path,
                                            std::tuple<uint64_t, uint64_t> start, std::tuple<uint64_t, uint64_t> goal,
                                            uint64_t max_iter, uint64_t nx, uint64_t ny, uint64_t goal_threshold,
                                            const std::shared_ptr<std::vector<float>> costmap)
{
    uint64_t num_obstacles = 0;
    for (uint64_t i = 0; i < nx * ny; i++)
    {
        if ((*costmap)[i] >= OBSTACLE_THRESHOLD)
        {
            num_obstacles++;
        }
    }
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ParkingPlanner"), "[p_greedySearch] num_obstacles: " << num_obstacles);

    // check if start is within bounds
    uint64_t x = std::get<0>(start);
    uint64_t y = std::get<1>(start);
    if (x < 0 || x >= nx || y < 0 || y >= ny)
    {
        return false;
    }

    // check if goal is within bounds
    uint64_t gx = std::get<0>(goal);
    uint64_t gy = std::get<1>(goal);
    if (gx < 0 || gx >= nx || gy < 0 || gy >= ny)
    {
        return false;
    }

    // check if costmap is valid
    if (costmap == nullptr)
    {
        return false;
    }

    // check if nx*ny == length of costmap
    if (nx * ny != costmap->size())
    {
        return false;
    }

    // check if path is valid
    if (path == nullptr)
    {
        return false;
    }

    if (start == goal)
    {
        path->push_back(start);
        return true;
    }

    // clear path
    path->clear();

    // define a priority queue that uses the cost as comparator, and index as value
    struct potential_field_planning_queue_comparator
    {
        const std::vector<float> *costmap;

        potential_field_planning_queue_comparator(const std::vector<float> *costmap) : costmap(costmap) {}

        bool operator()(const uint64_t a, const uint64_t b) const
        {
            return costmap->at(a) > costmap->at(b);
        }
    };

    auto comparator = potential_field_planning_queue_comparator(costmap.get());
    std::priority_queue<uint64_t, std::vector<uint64_t>, decltype(comparator)> pq(comparator);
    pq.push(this->get_index(start));

    // visited map
    std::vector<bool> visited = std::vector<bool>(nx * ny, false);

    // parent mapping
    std::map<uint64_t, uint64_t> parent;

    uint64_t iter = 0;
    while (!pq.empty())
    {
        if (iter >= max_iter)
        {
            return false;
        }

        int64_t index = pq.top();
        if (visited[index]) {
            pq.pop();
            int x = index % nx;
            int y = index / nx;
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("ParkingPlanner"), "[p_greedySearch] index: " << index << " is visited" << " x: " << x << " y: " << y);
            continue;
        }
        pq.pop();
        visited[index] = true; // mark as visited

        uint64_t x = index % nx;
        uint64_t y = index / nx;
        std::tuple<uint64_t, uint64_t> coord = std::make_tuple(x, y);
        
        // print the coord and the cost
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ParkingPlanner"), "iter: " << iter << "/" << max_iter  <<  " coord: " << x << ", " << y << ", cost: " << costmap->at(index));

        if (this->isWithinGoal(coord, goal, goal_threshold))
        {
            // Reconstruct the path by backtracking through the parent mapping.
            while (coord != start)
            {
                path->push_back(coord);
                uint64_t parent_index = parent[index];
                coord = getCoordFromIndex(parent_index);
                index = parent_index;
            }
            path->push_back(start);
            std::reverse(path->begin(), path->end()); // Reverse the path to start from the start point.
            return true;                              // Goal found.
        }

        // get neighbors sorted by cost
        std::vector<std::tuple<uint64_t, uint64_t>> neighbors = this->getNeighbors(coord);

        // for every neighbor, if not visited and have lower cost than now, add to stack
        for (auto &neighbor : neighbors)
        {
            uint64_t neighbor_index = this->get_index(neighbor);
            if (!visited[neighbor_index])
            {
                pq.push(neighbor_index);
                parent[neighbor_index] = index;
            }
        }

        iter++;
    }

    return false;
}