#ifndef NEIGHBOURS_H
#define NEIGHBOURS_H

#include <sys/types.h>
#include <vector>

namespace smmap
{
    class LineNeighbours
    {
        public:
            LineNeighbours(const ssize_t num_nodes);
            std::vector<ssize_t> getNodeNeighbours(const ssize_t node) const;

        private:
            const ssize_t num_nodes_;
    };

    class Grid4Neighbours
    {
        public:
            Grid4Neighbours(const ssize_t num_nodes, const ssize_t stride);
            std::vector<ssize_t> getNodeNeighbours(const ssize_t node) const;

        private:
            const ssize_t num_nodes_;
            const ssize_t stride_;
    };

    // Helper structure to convert between x ind, y ind, and node ind for cloth;
    // TODO: Update this structure to be a "Grid8Neighbours" class, similar to above
    /*
    struct NodeXYInd
    {
        NodeXYInd()
        {}

        NodeXYInd(ssize_t x_num, ssize_t y_num)
            : num_x_steps_(x_num)
            , num_y_steps_(y_num)
            , num_nodes_(x_num * y_num)
        {}

        void SetNodeXYInd(ssize_t x_num, ssize_t y_num)
        {
            num_x_steps_ = x_num;
            num_y_steps_ = y_num;
            num_nodes_ = x_num * y_num;
        }

        ssize_t GetNodeInd(ssize_t x_ind, ssize_t y_ind)
        {
            assert(NodeInBound(x_ind, y_ind)||"xy_ind out of bound");
            return y_ind * num_x_steps_ + x_ind;
        }

        // first element is x ind, second is y ind
        std::pair<ssize_t, ssize_t> GetXYInd(ssize_t node_ind)
        {
            assert(NodeInBound(node_ind)||"node_ind out of bound");
            std::pair<ssize_t, ssize_t> xy_ind;
            xy_ind.second = node_ind / num_x_steps_;
            xy_ind.first = node_ind - xy_ind.second * num_x_steps_;
            return xy_ind;
        }

        // the size of vector is always 8, value is -1 if out of bound.
        // Layout :
        //   3  2  1
        //   4  X  0
        //   5  6  7
        std::vector<ssize_t> Neighbor8Ind(ssize_t node_ind)
        {
            const int num_neighbor = 8;
            std::vector<ssize_t> nearest_8_neighbor(num_neighbor, -1);

            if (NodeInBound(node_ind + 1))
            { nearest_8_neighbor.at(0) = node_ind + 1;}

            if (NodeInBound(node_ind + 1 + num_x_steps_))
            { nearest_8_neighbor.at(1) = node_ind + 1 + num_x_steps_;}

            if (NodeInBound(node_ind + num_x_steps_))
            { nearest_8_neighbor.at(2) = node_ind + num_x_steps_;}

            if (NodeInBound(node_ind - 1 + num_x_steps_))
            { nearest_8_neighbor.at(3) = node_ind - 1 + num_x_steps_;}

            if (NodeInBound(node_ind - 1))
            { nearest_8_neighbor.at(4) = node_ind - 1;}

            if (NodeInBound(node_ind - 1 - num_x_steps_))
            { nearest_8_neighbor.at(5) = node_ind - 1 - num_x_steps_;}

            if (NodeInBound(node_ind - num_x_steps_))
            { nearest_8_neighbor.at(6) = node_ind - num_x_steps_;}

            if (NodeInBound(node_ind + 1 - num_x_steps_))
            { nearest_8_neighbor.at(7) = node_ind + 1 - num_x_steps_;}

            return nearest_8_neighbor;
        }

        bool NodeInBound(ssize_t node_ind)
        {
            if ((node_ind >=0) && (node_ind < num_nodes_))
                return true;
            return false;
        }

        bool NodeInBound(ssize_t x_ind, ssize_t y_ind)
        {
            if ((x_ind >=0) && (x_ind < num_x_steps_))
            {
                if (((y_ind >=0) && (y_ind < num_y_steps_)))
                    return true;
            }
            return false;
        }

        ssize_t num_x_steps_;
        ssize_t num_y_steps_;
        ssize_t num_nodes_;
    };
    */
}

#endif // NEIGHBOURS_H
