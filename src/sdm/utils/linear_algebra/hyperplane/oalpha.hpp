#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/hyperplane/alpha_vector.hpp>

namespace sdm
{
    class oAlpha : public AlphaVector
    {
    public:
        oAlpha(double default_value);

        bool isDominated(const Hyperplane &other) const;

        double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o) const;
        void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, double value);

        size_t hash(double precision) const;
        bool isEqual(const std::shared_ptr<Hyperplane> &other, double precision) const;

    protected:
        std::unordered_map<std::shared_ptr<HistoryInterface>, MappedVector<std::shared_ptr<State>, double>> repr;
    };
}
