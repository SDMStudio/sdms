#include <sdm/utils/struct/graph.hpp>

namespace sdm
{
    template <typename TNode, typename TEdge>
    Graph<TNode, TEdge>::Graph()
    {
    }

    template <typename TNode, typename TEdge>
    Graph<TNode, TEdge>::Graph(const TNode &data) : data_(data)
    {
    }

    template <typename TNode, typename TEdge>
    Graph<TNode, TEdge>::Graph(std::shared_ptr<Graph<TNode, TEdge>> predecessor, const TNode &data) : Graph(data)
    {
        this->predecessors.insert(predecessor);
    }

    template <typename TNode, typename TEdge>
    Graph<TNode, TEdge>::~Graph()
    {
    }

    template <typename TNode, typename TEdge>
    const TNode &Graph<TNode, TEdge>::getData() const
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
    const std::shared_ptr<Graph<TNode, TEdge>> &Graph<TNode, TEdge>::getSuccessor(const TEdge &edge) const
    {
        return this->successors.at(edge);
    }

    template <typename TNode, typename TEdge>
    const std::set<std::shared_ptr<Graph<TNode, TEdge>>> &Graph<TNode, TEdge>::getPredecessors() const
    {
        return this->predecessors;
    }

    template <typename TNode, typename TEdge>
    void Graph<TNode, TEdge>::addSuccessor(const TEdge &edge, const TNode &node)
    {
        if (this->successors.find(edge) == this->successors.end())
        {
            this->successors.emplace(edge, std::make_shared<Graph<TNode, TEdge>>(this->getptr(), node));
        }
    }

    template <typename TNode, typename TEdge>
    std::shared_ptr<Graph<TNode, TEdge>> Graph<TNode, TEdge>::getptr()
    {
        return this->shared_from_this();
    }

    template <typename TNode, typename TEdge>
    std::string Graph<TNode, TEdge>::str()
    {
        std::ostringstream str_result;
        str_result << "<graph data=\"" << this->getData() << "\" address=\"" << this->getptr() << "\" num_succ=" << this->getNumSuccessors() << "\" num_pred=\"" << this->getNumPredecessors() << "\"/>";
        return str_result.str();
    }
}