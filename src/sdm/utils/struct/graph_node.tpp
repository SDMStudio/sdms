#include <sdm/utils/struct/graph_node.hpp>

namespace sdm
{

    template <typename TNode, typename TEdge>
    GraphNode<TNode, TEdge>::GraphNode(const TNode &data) : data_(data)
    {
        // std::cout << "construct GraphNode"<< std::endl;
    }

    // template <typename TNode, typename TEdge>
    // GraphNode<TNode, TEdge>::GraphNode(std::shared_ptr<GraphNode<TNode, TEdge>> predecessor, const TNode &data) : GraphNode(data)
    // {
    //     this->predecessors.insert(predecessor);
    // }

    template <typename TNode, typename TEdge>
    GraphNode<TNode, TEdge>::~GraphNode()
    {
        // std::cout << "destruct GraphNode "<< this->getData()<< std::endl;
    }

    template <typename TNode, typename TEdge>
    std::shared_ptr<GraphNode<TNode, TEdge>> GraphNode<TNode, TEdge>::getSuccessor(const TEdge &edge) const
    {
        auto iterator_on_successor = this->successors.find(edge);
        return (iterator_on_successor != this->successors.end()) ? iterator_on_successor->second.lock() : nullptr;
    }

    template <typename TNode, typename TEdge>
    std::shared_ptr<GraphNode<TNode, TEdge>> GraphNode<TNode, TEdge>::getPredecessor(const TEdge &edge) const
    {
        auto iterator_on_predecessor = this->predecessors.find(edge);
        return (iterator_on_predecessor != this->predecessors.end()) ? iterator_on_predecessor->second.lock() : nullptr;
    }

    template <typename TNode, typename TEdge>
    void GraphNode<TNode, TEdge>::addSuccessor(const TEdge &edge_value, const std::shared_ptr<GraphNode> &succ_node)
    {
        if (succ_node)
            this->successors.emplace(edge_value, succ_node);
    }

    template <typename TNode, typename TEdge>
    void GraphNode<TNode, TEdge>::addPredecessor(const TEdge &edge_value, const std::shared_ptr<GraphNode> &pred_node)
    {
        if (pred_node)
            this->predecessors.emplace(edge_value, pred_node);
    }

    template <typename TNode, typename TEdge>
    TNode &&GraphNode<TNode, TEdge>::data() const
    {
        return this->data_;
    }

    template <typename TNode, typename TEdge>
    TNode GraphNode<TNode, TEdge>::getData() const
    {
        return this->data_;
    }

    template <typename TNode, typename TEdge>
    void GraphNode<TNode, TEdge>::setData(const TNode &data)
    {
        this->data_ = data;
    }

    template <typename TNode, typename TEdge>
    number GraphNode<TNode, TEdge>::getNumSuccessors() const
    {
        return this->successors.size();
    }

    template <typename TNode, typename TEdge>
    number GraphNode<TNode, TEdge>::getNumPredecessors() const
    {
        return this->predecessors.size();
    }

    template <typename TNode, typename TEdge>
    std::shared_ptr<GraphNode<TNode, TEdge>> GraphNode<TNode, TEdge>::getptr()
    {
        return this->shared_from_this();
    }

    template <typename TNode, typename TEdge>
    template <class Archive>
    void GraphNode<TNode, TEdge>::serialize(Archive &archive, const unsigned int)
    {
        using boost::serialization::make_nvp;

        archive &make_nvp("data", data_);
        // archive &boost::serialization::base_object<TNode>(*this);
        archive &make_nvp("successors", successors);
        archive &make_nvp("predecessors", predecessors);
    }

    template <typename TNode, typename TEdge>
    std::string GraphNode<TNode, TEdge>::str() const
    {
        std::ostringstream str_result;
        str_result << "<node data=\"" << this->getData()->str() << "\" address=\"" << this << "\" num_succ=" << this->getNumSuccessors() << "\" num_pred=\"" << this->getNumPredecessors() << "\">" << std::endl;
        // for (const auto pair_edge_succ : this->successors)
        // {
        //     str_result << "\t<successor edge=\"" << pair_edge_succ.first.first->str() << "\" node=\"" << pair_edge_succ.second << "\"/>" << std::endl;
        // }
        str_result << "</node>";
        return str_result.str();
    }
}