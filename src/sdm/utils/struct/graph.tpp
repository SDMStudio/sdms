#include <sdm/utils/struct/graph.hpp>

namespace sdm
{
    template <typename TNode, typename TEdge>
    Graph<TNode, TEdge>::Graph()
    {
        // std::cout << "constrcut Graph"<< std::endl;
    }

    template <typename TNode, typename TEdge>
    Graph<TNode, TEdge>::~Graph()
    {
        // std::cout << "destruct Graph"<< std::endl;
    }

    template <typename TNode, typename TEdge>
    std::shared_ptr<GraphNode<TNode, TEdge>> Graph<TNode, TEdge>::getNode(const TNode &node_value) const
    {
        auto iterator_on_node = this->node_space_.find(node_value);
        return (iterator_on_node != this->node_space_.end()) ? iterator_on_node->second : nullptr;
    }

    template <typename TNode, typename TEdge>
    void Graph<TNode, TEdge>::addNode(const TNode &node_value)
    {
        auto iterator = this->node_space_.find(node_value);
        if (iterator == this->node_space_.end())
        {
            this->node_space_.emplace(node_value, std::make_shared<GraphNode<TNode, TEdge>>(node_value));
        }
    }

    template <typename TNode, typename TEdge>
    number Graph<TNode, TEdge>::getNumNodes() const
    {
        return this->node_space_.size();
    }

    template <typename TNode, typename TEdge>
    std::shared_ptr<GraphNode<TNode, TEdge>> Graph<TNode, TEdge>::getSuccessor(const TNode &node_value, const TEdge &edge) const
    {
        auto node = this->getNode(node_value);
        return (node) ? node->getSuccessor(edge) : nullptr;
    }

    template <typename TNode, typename TEdge>
    void Graph<TNode, TEdge>::addSuccessor(const TNode &node_value, const TEdge &edge_value, const TNode &succ_node_value)
    {
        auto node = this->getNode(node_value);
        // Check if the node exists
        if (node)
        {
            // If exists, check if no such edge exists
            if (!node->getSuccessor(edge_value))
            {
                // If no such edge exists, check if successor exists
                auto succ_node = this->getNode(succ_node_value);
                if (!succ_node)
                {
                    // Add successor node if not exists in the graph
                    this->addNode(succ_node_value);
                    succ_node = this->getNode(succ_node_value);
                }
                // Add the edge
                node->addSuccessor(edge_value, succ_node);
            }
        }
    }
    
    template <typename TNode, typename TEdge>
    bool Graph<TNode, TEdge>::contains(const TNode &node_value) const
    {
        return (this->getNode(node_value) != nullptr);
    }

    template <typename TNode, typename TEdge>
    std::shared_ptr<Graph<TNode, TEdge>> Graph<TNode, TEdge>::getptr()
    {
        return this->shared_from_this();
    }

    template <typename TNode, typename TEdge>
    template <class Archive>
    void Graph<TNode, TEdge>::serialize(Archive &archive, const unsigned int)
    {
        using boost::serialization::make_nvp;
        // archive &make_nvp("data", data_);
        // archive &boost::serialization::base_object<TNode>(*this);
        // archive &make_nvp("successors", successors);
    }

    template <typename TNode, typename TEdge>
    std::string Graph<TNode, TEdge>::str() const
    {
        std::ostringstream str_result;
        str_result << "<graph>" << std::endl;
        for (const auto &pair_value_node : this->node_space_)
        {
            sdm::tools::indentedOutput(str_result, pair_value_node.second->str().c_str(), 1);
            str_result << std::endl;
        }
        str_result << "</graph>";
        return str_result.str();
    }
}