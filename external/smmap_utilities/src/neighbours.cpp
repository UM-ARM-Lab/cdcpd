#include "smmap_utilities/neighbours.h"

using namespace smmap;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Rope and Cloth neighbour functions /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

LineNeighbours::LineNeighbours(const ssize_t num_nodes)
    : num_nodes_(num_nodes)
{}

std::vector<ssize_t> LineNeighbours::getNodeNeighbours(const ssize_t node) const
{
    std::vector<ssize_t> neighbours;
    neighbours.reserve(2);

    // Left
    if (node > 0)
    {
        neighbours.push_back(node - 1);
    }

    // Right
    if (node + 1 < num_nodes_)
    {
        neighbours.push_back(node + 1);
    }

    return neighbours;
}

Grid4Neighbours::Grid4Neighbours(const ssize_t num_nodes, const ssize_t stride)
    : num_nodes_(num_nodes)
    , stride_(stride)
{}

std::vector<ssize_t> Grid4Neighbours::getNodeNeighbours(const ssize_t node) const
{
    std::vector<ssize_t> neighbours;
    neighbours.reserve(4);

    const bool on_upper_edge = node < stride_;
    const bool on_left_edge = node % stride_ == 0;
    const bool on_right_edge = (node + 1) % stride_ == 0;
    const bool on_bottom_edge = node + stride_ >= num_nodes_;

    // Up
    if (!on_upper_edge)
    {
        neighbours.push_back(node - stride_);
    }

    // Left
    if (!on_left_edge)
    {
        neighbours.push_back(node - 1);
    }

    // Right
    if (!on_right_edge)
    {
        neighbours.push_back(node + 1);
    }

    // Down
    if (!on_bottom_edge)
    {
        neighbours.push_back(node + stride_);
    }

    return neighbours;
}
