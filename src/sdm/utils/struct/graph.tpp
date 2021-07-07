#include <sdm/utils/struct/graph.hpp>

namespace sdm
{
    template <typename TNode, typename TEdge>
    Graph<TNode, TEdge>::Graph()
    {
        this->node_space_ = std::make_shared<std::unordered_map<TNode, std::shared_ptr<Graph>>>();
    }

    template <typename TNode, typename TEdge>
    Graph<TNode, TEdge>::Graph(const TNode &data, const std::shared_ptr<std::unordered_map<TNode, std::shared_ptr<Graph>>> &node_space) : data_(data), node_space_(node_space)
    {
    }

    template <typename TNode, typename TEdge>
    Graph<TNode, TEdge>::Graph(std::shared_ptr<Graph<TNode, TEdge>> predecessor, const TNode &data) : Graph(data, predecessor->node_space)
    {
        this->predecessors.insert(predecessor);
    }

    template <typename TNode, typename TEdge>
    Graph<TNode, TEdge>::~Graph()
    {
    }

    template <typename TNode, typename TEdge>
    std::shared_ptr<Graph<TNode, TEdge>> Graph<TNode, TEdge>::getNode(const TNode &node_value) const
    {
        auto iterator_on_node = this->node_space_->find(node_value);
        return (iterator_on_node != this->node_space_->end()) ? iterator_on_node->second : nullptr;
    }

    template <typename TNode, typename TEdge>
    void Graph<TNode, TEdge>::addNode(const TNode &node_value)
    {
        auto iterator = this->node_space_->find(node_value);
        if (iterator == this->node_space_->end())
        {
            this->node_space_->emplace(node_value, std::make_shared<Graph<TNode, TEdge>>(node_value, this->node_space_));
        }
    }

    template <typename TNode, typename TEdge>
    std::shared_ptr<Graph<TNode, TEdge>> Graph<TNode, TEdge>::getSuccessor(const TEdge &edge) const
    {
        auto iterator_on_successor = this->successors.find(edge);
        return (iterator_on_successor != this->successors.end()) ? iterator_on_successor->second : nullptr;
    }

    template <typename TNode, typename TEdge>
    std::set<std::shared_ptr<Graph<TNode, TEdge>>> Graph<TNode, TEdge>::getPredecessors() const
    {
        return this->predecessors;
    }

    template <typename TNode, typename TEdge>
    void Graph<TNode, TEdge>::addSuccessor(const TEdge &edge_value, const TNode &node_value)
    {
        if (this->successors.find(edge_value) == this->successors.end())
        {
            // Get the address of the node corresponding to the value
            auto node = this->getNode(node_value);
            if (node == nullptr)
            {
                // Add node if not exists in the graph
                this->addNode(node_value);
                node = this->getNode(node_value);
            }
            this->successors.emplace(edge_value, node);
            node->addPredecessor(this->getData());
        }
    }

    template <typename TNode, typename TEdge>
    void Graph<TNode, TEdge>::addPredecessor(const TNode &node_value)
    {
        auto node = this->getNode(node_value);
        if (node != nullptr)
        {
            this->predecessors.insert(node);
        }
    }

    template <typename TNode, typename TEdge>
    TNode &&Graph<TNode, TEdge>::data() const
    {
        return this->data_;
    }

    template <typename TNode, typename TEdge>
    TNode Graph<TNode, TEdge>::getData() const
    {
        return this->data_;
    }

    template <typename TNode, typename TEdge>
    void Graph<TNode, TEdge>::setData(const TNode &data)
    {
        this->data_ = data;
    }

    template <typename TNode, typename TEdge>
    number Graph<TNode, TEdge>::getNumSuccessors() const
    {
        return this->successors.size();
    }

    template <typename TNode, typename TEdge>
    number Graph<TNode, TEdge>::getNumPredecessors() const
    {
        return this->predecessors.size();
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

        archive &make_nvp("data", data_);
        // archive &boost::serialization::base_object<TNode>(*this);
        archive &make_nvp("successors", successors);
        archive &make_nvp("predecessors", predecessors);
    }

    template <typename TNode, typename TEdge>
    std::string Graph<TNode, TEdge>::str() const
    {
        std::ostringstream str_result;
        str_result << "<graph>" << std::endl;
        for (const auto &pair_value_node : *this->node_space_)
        {
            sdm::tools::indentedOutput(str_result, pair_value_node.second->node_str().c_str(), 1);
            str_result << std::endl;
        }
        str_result << "</graph>";
        return str_result.str();
    }

    template <typename TNode, typename TEdge>
    std::string Graph<TNode, TEdge>::node_str() const
    {
        std::ostringstream str_result;
        str_result << "<node data=\"" << this->getData() << "\" address=\"" << this << "\" num_succ=" << this->getNumSuccessors() << "\" num_pred=\"" << this->getNumPredecessors() << "\">" << std::endl;
        for (const auto pair_edge_succ : this->successors)
        {
            str_result << "\t<successor edge=\"" << pair_edge_succ.first << "\" node=\"" << pair_edge_succ.second << "\"/>" << std::endl;
        }
        str_result << "</node>";
        return str_result.str();
    }
}