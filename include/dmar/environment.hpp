#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <random>
#include <unordered_map>
#include <vector>

namespace dmar {

struct Position {
    int row = 0;
    int column = 0;

    friend bool operator==(const Position& left, const Position& right) {
        return left.row == right.row && left.column == right.column;
    }
    friend bool operator!=(const Position& left, const Position& right) {
        return !(left == right);
    }
};

class GridEnvironment {
public:
    GridEnvironment(int rows, int columns);

    int rows() const noexcept;
    int columns() const noexcept;
    bool in_bounds(Position position) const noexcept;
    bool traversable(Position position) const noexcept;
    void set_obstacle(Position position, bool blocked = true);
    void add_task(Position position);
    bool has_task(Position position) const noexcept;
    bool remove_task(Position position) noexcept;
    std::size_t unresolved_tasks() const noexcept;
    std::vector<Position> legal_moves(Position position) const;
    std::optional<int> shortest_distance(Position from, Position target) const;
    std::optional<Position> shortest_step(Position from, Position target) const;
    std::vector<Position> view(Position center, int radius) const;
    Position random_legal_move(Position position, std::mt19937& random) const;

private:
    using DistanceMap = std::vector<int>;

    std::size_t index(Position position) const;
    const DistanceMap& distances_from(Position from) const;
    int rows_;
    int columns_;
    std::vector<bool> obstacles_;
    std::vector<bool> tasks_;
    mutable std::shared_ptr<std::unordered_map<std::size_t, DistanceMap>> distance_cache_;
};

}  // namespace dmar