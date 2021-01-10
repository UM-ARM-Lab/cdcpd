#ifndef ARC_GRAPH_HPP
#define ARC_GRAPH_HPP

#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/serialization.hpp>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

namespace arc_dijkstras {
enum EDGE_VALIDITY { VALID, INVALID, UNKNOWN };

/***********************************
 **    Graph Edge
 **********************************/
class GraphEdge {
 protected:
  int64_t from_index_;
  int64_t to_index_;
  double weight_;
  EDGE_VALIDITY edge_validity_;

 public:
  static uint64_t Serialize(const GraphEdge& edge, std::vector<uint8_t>& buffer) { return edge.serializeSelf(buffer); }

  static std::pair<GraphEdge, uint64_t> Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current) {
    GraphEdge temp_edge;
    const uint64_t bytes_read = temp_edge.deserializeSelf(buffer, current);
    return std::make_pair(temp_edge, bytes_read);
  }

  GraphEdge(const int64_t from_index, const int64_t to_index, const double weight)
      : from_index_(from_index), to_index_(to_index), weight_(weight), edge_validity_(EDGE_VALIDITY::UNKNOWN) {}

  GraphEdge() : from_index_(-1), to_index_(-1), weight_(0.0), edge_validity_(EDGE_VALIDITY::UNKNOWN) {}

  uint64_t serializeSelf(std::vector<uint8_t>& buffer) const {
    const uint64_t start_buffer_size = buffer.size();
    arc_utilities::SerializeFixedSizePOD<int64_t>(from_index_, buffer);
    arc_utilities::SerializeFixedSizePOD<int64_t>(to_index_, buffer);
    arc_utilities::SerializeFixedSizePOD<double>(weight_, buffer);
    arc_utilities::SerializeFixedSizePOD<EDGE_VALIDITY>(edge_validity_, buffer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  uint64_t deserializeSelf(const std::vector<uint8_t>& buffer, const uint64_t current) {
    using namespace arc_utilities;
    uint64_t current_position = current;
    const std::pair<int64_t, uint64_t> deserialized_from_index =
        DeserializeFixedSizePOD<int64_t>(buffer, current_position);
    from_index_ = deserialized_from_index.first;
    current_position += deserialized_from_index.second;
    const std::pair<int64_t, uint64_t> deserialized_to_index =
        DeserializeFixedSizePOD<int64_t>(buffer, current_position);
    to_index_ = deserialized_to_index.first;
    current_position += deserialized_to_index.second;
    const std::pair<double, uint64_t> deserialized_weight = DeserializeFixedSizePOD<double>(buffer, current_position);
    weight_ = deserialized_weight.first;
    current_position += deserialized_weight.second;
    const std::pair<EDGE_VALIDITY, uint64_t> deserialized_validity =
        DeserializeFixedSizePOD<EDGE_VALIDITY>(buffer, current_position);
    edge_validity_ = deserialized_validity.first;
    current_position += deserialized_validity.second;

    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - current;
    return bytes_read;
  }

  bool operator==(const GraphEdge& other) const {
    return (from_index_ == other.getFromIndex() && to_index_ == other.getToIndex() && weight_ == other.getWeight() &&
            edge_validity_ == other.getValidity());
  }

  std::string print() const {
    return std::string("(" + std::to_string(from_index_) + "->" + std::to_string(to_index_) +
                       ") : " + std::to_string(weight_));
  }

  int64_t getFromIndex() const { return from_index_; }

  int64_t getToIndex() const { return to_index_; }

  double getWeight() const { return weight_; }

  void setFromIndex(const int64_t new_from_index) { from_index_ = new_from_index; }

  void setToIndex(const int64_t new_to_index) { to_index_ = new_to_index; }

  void setWeight(const double new_weight) { weight_ = new_weight; }

  EDGE_VALIDITY getValidity() const { return edge_validity_; }

  bool isValid() const { return edge_validity_ == EDGE_VALIDITY::VALID; }

  bool isInvalid() const { return edge_validity_ == EDGE_VALIDITY::INVALID; }

  void setValidity(const EDGE_VALIDITY new_validity) { edge_validity_ = new_validity; }
};

inline std::ostream& operator<<(std::ostream& stream, const GraphEdge& edge) {
  stream << edge.print();
  return stream;
}

/***********************************
 **    Graph Node
 **********************************/
template <typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
class GraphNode {
 protected:
  NodeValueType value_;
  double distance_;
  std::vector<GraphEdge> in_edges_;
  std::vector<GraphEdge> out_edges_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static uint64_t Serialize(
      const GraphNode<NodeValueType, Allocator>& node, std::vector<uint8_t>& buffer,
      const std::function<uint64_t(const NodeValueType&, std::vector<uint8_t>&)>& value_serializer) {
    return node.serializeSelf(buffer, value_serializer);
  }

  static std::pair<GraphNode<NodeValueType, Allocator>, uint64_t> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t current,
      const std::function<std::pair<NodeValueType, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>&
          value_deserializer) {
    GraphNode<NodeValueType, Allocator> temp_node;
    const uint64_t bytes_read = temp_node.deserializeSelf(buffer, current, value_deserializer);
    return std::make_pair(temp_node, bytes_read);
  }

  GraphNode(const NodeValueType& value, const double distance, const std::vector<GraphEdge>& new_in_edges,
            const std::vector<GraphEdge>& new_out_edges)
      : value_(value), distance_(distance), in_edges_(new_in_edges), out_edges_(new_out_edges) {}

  explicit GraphNode(const NodeValueType& value) : value_(value), distance_(std::numeric_limits<double>::infinity()) {}

  GraphNode() : distance_(std::numeric_limits<double>::infinity()) {}

  uint64_t serializeSelf(
      std::vector<uint8_t>& buffer,
      const std::function<uint64_t(const NodeValueType&, std::vector<uint8_t>&)>& value_serializer) const {
    const uint64_t start_buffer_size = buffer.size();
    // Serialize the value
    value_serializer(value_, buffer);
    // Serialize the distance
    arc_utilities::SerializeFixedSizePOD<double>(distance_, buffer);
    // Serialize the in edges
    arc_utilities::SerializeVector<GraphEdge>(in_edges_, buffer, GraphEdge::Serialize);
    // Serialize the in edges
    arc_utilities::SerializeVector<GraphEdge>(out_edges_, buffer, GraphEdge::Serialize);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  uint64_t deserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t current,
      const std::function<std::pair<NodeValueType, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>&
          value_deserializer) {
    using namespace arc_utilities;
    uint64_t current_position = current;
    // Deserialize the value
    const std::pair<NodeValueType, uint64_t> value_deserialized = value_deserializer(buffer, current_position);
    value_ = value_deserialized.first;
    current_position += value_deserialized.second;
    // Deserialize the distace
    const std::pair<double, uint64_t> distance_deserialized = DeserializeFixedSizePOD<double>(buffer, current_position);
    distance_ = distance_deserialized.first;
    current_position += distance_deserialized.second;
    // Deserialize the in edges
    const std::pair<std::vector<GraphEdge>, uint64_t> in_edges_deserialized =
        DeserializeVector<GraphEdge>(buffer, current_position, GraphEdge::Deserialize);
    in_edges_ = in_edges_deserialized.first;
    current_position += in_edges_deserialized.second;
    // Deserialize the out edges
    const std::pair<std::vector<GraphEdge>, uint64_t> out_edges_deserialized =
        DeserializeVector<GraphEdge>(buffer, current_position, GraphEdge::Deserialize);
    out_edges_ = out_edges_deserialized.first;
    current_position += out_edges_deserialized.second;
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - current;
    return bytes_read;
  }

  std::string print() const {
    std::ostringstream strm;
    strm << "Node : " << distance_ << " In Edges : ";
    if (in_edges_.size() > 0) {
      strm << in_edges_[0].print();
      for (size_t idx = 1; idx < in_edges_.size(); idx++) {
        strm << ", " << in_edges_[idx].print();
      }
    }
    strm << " Out Edges : ";
    if (out_edges_.size() > 0) {
      strm << out_edges_[0].print();
      for (size_t idx = 1; idx < out_edges_.size(); idx++) {
        strm << ", " << out_edges_[idx].print();
      }
    }
    return strm.str();
  }

  const NodeValueType& getValue() const { return value_; }

  NodeValueType& getValue() { return value_; }

  void addInEdge(const GraphEdge& new_in_edge) { in_edges_.push_back(new_in_edge); }

  void addOutEdge(const GraphEdge& new_out_edge) { out_edges_.push_back(new_out_edge); }

  void addEdgePair(const GraphEdge& new_in_edge, const GraphEdge& new_out_edge) {
    addInEdge(new_in_edge);
    addOutEdge(new_out_edge);
  }

  double getDistance() const { return distance_; }

  void setDistance(const double distance) { distance_ = distance; }

  GraphEdge& getEdgeTo(const int64_t other_node_ind) {
    for (auto& e : out_edges_) {
      if (e.getToIndex() == other_node_ind) {
        return e;
      }
    }
    throw std::invalid_argument("Invalid node index, no edge exists");
  }

  const GraphEdge& getEdgeTo(const int64_t other_node_ind) const {
    for (const auto& e : out_edges_) {
      if (e.getToIndex() == other_node_ind) {
        return e;
      }
    }
    throw std::invalid_argument("Invalid node index, no edge exists");
  }

  const std::vector<GraphEdge>& getInEdges() const { return in_edges_; }

  std::vector<GraphEdge>& getInEdges() { return in_edges_; }

  const std::vector<GraphEdge>& getOutEdges() const { return out_edges_; }

  std::vector<GraphEdge>& getOutEdges() { return out_edges_; }

  void setInEdges(const std::vector<GraphEdge>& new_in_edges) { in_edges_ = new_in_edges; }

  void setOutEdges(const std::vector<GraphEdge>& new_out_edges) { out_edges_ = new_out_edges; }
};

/***********************************
 **    Graph
 **********************************/
template <typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
class Graph {
 protected:
  std::vector<GraphNode<NodeValueType, Allocator>> nodes_;

  size_t markConnectedComponentUndirected(const int64_t starting_node_idx, std::vector<uint32_t>& components,
                                          const uint32_t component_id) const {
    // When we push into the queue, we mark as connected, so we don't need a separate
    // "queued" tracker to
    // avoid duplication, unlike what is used in CollisionMapGrid::MarkConnectedComponent
    std::queue<int64_t> working_queue;
    working_queue.push(starting_node_idx);
    size_t num_marked = 1;

    while (working_queue.size() > 0) {
      const auto next_node_idx = working_queue.front();
      working_queue.pop();

      const auto& out_edges = nodes_[next_node_idx].getOutEdges();
      for (const auto& edge : out_edges) {
        const auto neighbour_idx = edge.getToIndex();
        if (components[neighbour_idx] == 0) {
          components[neighbour_idx] = component_id;
          ++num_marked;
          working_queue.push(neighbour_idx);
        }
      }
    }

    return num_marked;
  }

 public:
  static uint64_t Serialize(
      const Graph<NodeValueType, Allocator>& graph, std::vector<uint8_t>& buffer,
      const std::function<uint64_t(const NodeValueType&, std::vector<uint8_t>&)>& value_serializer) {
    return graph.serializeSelf(buffer, value_serializer);
  }

  static std::pair<Graph<NodeValueType, Allocator>, uint64_t> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t current,
      const std::function<std::pair<NodeValueType, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>&
          value_deserializer) {
    Graph<NodeValueType, Allocator> temp_graph;
    const uint64_t bytes_read = temp_graph.deserializeSelf(buffer, current, value_deserializer);
    return std::make_pair(temp_graph, bytes_read);
  }

  Graph(const std::vector<GraphNode<NodeValueType, Allocator>>& nodes) {
    if (CheckGraphLinkage(nodes)) {
      nodes_ = nodes;
    } else {
      throw std::invalid_argument("Invalid graph linkage");
    }
  }

  Graph(const size_t expected_size) { nodes_.reserve(expected_size); }

  Graph() {}

  uint64_t serializeSelf(
      std::vector<uint8_t>& buffer,
      const std::function<uint64_t(const NodeValueType&, std::vector<uint8_t>&)>& value_serializer) const {
    const uint64_t start_buffer_size = buffer.size();
    const auto graph_state_serializer = std::bind(GraphNode<NodeValueType, Allocator>::Serialize, std::placeholders::_1,
                                                  std::placeholders::_2, value_serializer);
    arc_utilities::SerializeVector<GraphNode<NodeValueType, Allocator>>(nodes_, buffer, graph_state_serializer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  uint64_t deserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t current,
      const std::function<std::pair<NodeValueType, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>&
          value_deserializer) {
    using namespace arc_utilities;
    const auto graph_state_deserializer = std::bind(GraphNode<NodeValueType, Allocator>::Deserialize,
                                                    std::placeholders::_1, std::placeholders::_2, value_deserializer);
    const auto deserialized_nodes =
        DeserializeVector<GraphNode<NodeValueType, Allocator>>(buffer, current, graph_state_deserializer);
    nodes_ = deserialized_nodes.first;
    return deserialized_nodes.second;
  }

  std::string print() const {
    std::ostringstream strm;
    strm << "Graph - Nodes : ";
    if (nodes_.size() > 0) {
      strm << nodes_[0].print();
      for (size_t idx = 1; idx < nodes_.size(); idx++) {
        strm << "\n" << nodes_[idx].print();
      }
    }
    return strm.str();
  }

  void shrinkToFit() { nodes_.shrink_to_fit(); }

  bool indexInRange(const int64_t index) const { return index >= 0 && index < (int64_t)(nodes_.size()); }

  bool checkGraphLinkage() const { return CheckGraphLinkage(getNodes()); }

  GraphEdge& getEdge(const int64_t node_ind_1, const int64_t node_ind_2) {
    return getNode(node_ind_1).getEdgeTo(node_ind_2);
  }

  const GraphEdge& getEdge(const int64_t node_ind_1, const int64_t node_ind_2) const {
    return getNode(node_ind_1).getEdgeTo(node_ind_2);
  }

  GraphEdge& getReverseEdge(const GraphEdge& e) { return getNode(e.getToIndex()).getEdgeTo(e.getFromIndex()); }

  static bool CheckGraphLinkage(const Graph<NodeValueType, Allocator>& graph) {
    return CheckGraphLinkage(graph.getNodes());
  }

  static bool CheckGraphLinkage(const std::vector<GraphNode<NodeValueType, Allocator>>& nodes) {
    // Go through every node and make sure the edges are valid
    for (size_t idx = 0; idx < nodes.size(); idx++) {
      const GraphNode<NodeValueType, Allocator>& current_node = nodes[idx];
      // Check the in edges first
      const std::vector<GraphEdge>& in_edges = current_node.getInEdges();
      for (size_t in_edge_idx = 0; in_edge_idx < in_edges.size(); in_edge_idx++) {
        const GraphEdge& current_edge = in_edges[in_edge_idx];
        // Check from index to make sure it's in bounds
        const int64_t from_index = current_edge.getFromIndex();
        if (from_index < 0 || from_index >= (int64_t)nodes.size()) {
          return false;
        }
        // Check to index to make sure it matches our own index
        const int64_t to_index = current_edge.getToIndex();
        if (to_index != (int64_t)idx) {
          return false;
        }
        // Check edge validity (edges to ourself are not allowed)
        if (from_index == to_index) {
          return false;
        }
        // Check to make sure that the from index node is linked to us
        const GraphNode<NodeValueType, Allocator>& from_node = nodes[(size_t)from_index];
        const std::vector<GraphEdge>& from_node_out_edges = from_node.getOutEdges();
        bool from_node_connection_valid = false;
        // Make sure at least one out edge of the from index node corresponds to the current node
        for (size_t from_node_out_edge_idx = 0; from_node_out_edge_idx < from_node_out_edges.size();
             from_node_out_edge_idx++) {
          const GraphEdge& current_from_node_out_edge = from_node_out_edges[from_node_out_edge_idx];
          if (current_from_node_out_edge.getToIndex() == (int64_t)idx) {
            from_node_connection_valid = true;
          }
        }
        if (from_node_connection_valid == false) {
          return false;
        }
      }
      // Check the out edges second
      const std::vector<GraphEdge>& out_edges = current_node.getOutEdges();
      for (size_t out_edge_idx = 0; out_edge_idx < out_edges.size(); out_edge_idx++) {
        const GraphEdge& current_edge = out_edges[out_edge_idx];
        // Check from index to make sure it matches our own index
        const int64_t from_index = current_edge.getFromIndex();
        if (from_index != (int64_t)idx) {
          return false;
        }
        // Check to index to make sure it's in bounds
        const int64_t to_index = current_edge.getToIndex();
        if (to_index < 0 || to_index >= (int64_t)nodes.size()) {
          return false;
        }
        // Check edge validity (edges to ourself are not allowed)
        if (from_index == to_index) {
          return false;
        }
        // Check to make sure that the to index node is linked to us
        const GraphNode<NodeValueType, Allocator>& to_node = nodes[(size_t)to_index];
        const std::vector<GraphEdge>& to_node_in_edges = to_node.getInEdges();
        bool to_node_connection_valid = false;
        // Make sure at least one in edge of the to index node corresponds to the current node
        for (size_t to_node_in_edge_idx = 0; to_node_in_edge_idx < to_node_in_edges.size(); to_node_in_edge_idx++) {
          const GraphEdge& current_to_node_in_edge = to_node_in_edges[to_node_in_edge_idx];
          if (current_to_node_in_edge.getFromIndex() == (int64_t)idx) {
            to_node_connection_valid = true;
          }
        }
        if (to_node_connection_valid == false) {
          return false;
        }
      }
    }
    return true;
  }

  const std::vector<GraphNode<NodeValueType, Allocator>>& getNodes() const { return nodes_; }

  std::vector<GraphNode<NodeValueType, Allocator>>& getNodes() { return nodes_; }

  const GraphNode<NodeValueType, Allocator>& getNode(const int64_t index) const { return nodes_.at((size_t)index); }

  GraphNode<NodeValueType, Allocator>& getNode(const int64_t index) { return nodes_.at((size_t)index); }

  int64_t addNode(const GraphNode<NodeValueType, Allocator>& new_node) {
    nodes_.push_back(new_node);
    return (int64_t)(nodes_.size() - 1);
  }

  int64_t addNode(const NodeValueType& new_value) { return addNode(GraphNode<NodeValueType, Allocator>(new_value)); }

  GraphEdge& addEdgeBetweenNodes(const int64_t from_index, const int64_t to_index, const double edge_weight) {
    // We retrieve the nodes first, since retrieval performs bounds checks first
    GraphNode<NodeValueType, Allocator>& from_node = getNode(from_index);
    GraphNode<NodeValueType, Allocator>& to_node = getNode(to_index);
    if (from_index == to_index) {
      throw std::invalid_argument("Invalid circular edge from==to not allowed");
    }
    const GraphEdge new_edge(from_index, to_index, edge_weight);
    from_node.addOutEdge(new_edge);
    to_node.addInEdge(new_edge);
    return from_node.getEdgeTo(to_index);
  }

  std::pair<const GraphEdge, const GraphEdge> addEdgesBetweenNodes(const int64_t first_index,
                                                                   const int64_t second_index,
                                                                   const double edge_weight) {
    GraphEdge& e1 = addEdgeBetweenNodes(first_index, second_index, edge_weight);
    GraphEdge& e2 = addEdgeBetweenNodes(second_index, first_index, edge_weight);
    return std::make_pair(e1, e2);
  }

  /**
   * @brief getConnectedComponentsUndirected
   * @return A vector of the component ids for each node, and the total number of components
   */
  std::pair<std::vector<uint32_t>, uint32_t> getConnectedComponentsUndirected() const {
    size_t total_num_marked = 0;
    auto connected_components = std::make_pair(std::vector<uint32_t>(nodes_.size(), 0), 0u);

    for (size_t node_idx = 0; node_idx < nodes_.size() && total_num_marked < nodes_.size(); ++node_idx) {
      // If we have not yet marked this node, then mark it and anything it can reach
      if (connected_components.first[node_idx] == 0) {
        connected_components.second++;
        size_t num_marked =
            markConnectedComponentUndirected(node_idx, connected_components.first, connected_components.second);
        total_num_marked += num_marked;
      }
    }
    return connected_components;
  }
};
}  // namespace arc_dijkstras

#endif
