#include "dmar/environment.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <queue>

namespace dmar {

GridEnvironment::GridEnvironment(int rows, int columns)
    : rows_(rows), columns_(columns), obstacles_(static_cast<std::size_t>(rows * columns), false),
      tasks_(static_cast<std::size_t>(rows * columns), false),
      distance_cache_(std::make_shared<std::unordered_map<std::size_t, DistanceMap>>()) {}

int GridEnvironment::rows() const noexcept { return rows_; }
int GridEnvironment::columns() const noexcept { return columns_; }

bool GridEnvironment::in_bounds(Position position) const noexcept {
    return position.row >= 0 && position.row < rows_ && position.column >= 0 && position.column < columns_;
}

bool GridEnvironment::traversable(Position position) const noexcept {
    return in_bounds(position) && !obstacles_[index(position)];
}

void GridEnvironment::set_obstacle(Position position, bool blocked) {
    if (!in_bounds(position)) return;
    obstacles_[index(position)] = blocked;
    if (blocked) tasks_[index(position)] = false;
    distance_cache_->clear();
}

void GridEnvironment::add_task(Position position) {
    if (traversable(position)) tasks_[index(position)] = true;
}

bool GridEnvironment::has_task(Position position) const noexcept {
    return in_bounds(position) && tasks_[index(position)];
}

bool GridEnvironment::remove_task(Position position) noexcept {
    if (!has_task(position)) return false;
    tasks_[index(position)] = false;
    return true;
}

std::size_t GridEnvironment::unresolved_tasks() const noexcept {
    return static_cast<std::size_t>(std::count(tasks_.begin(), tasks_.end(), true));
}

std::vector<Position> GridEnvironment::legal_moves(Position position) const {
    static constexpr std::array<Position, 4> directions{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
    std::vector<Position> result;
    for (const auto direction : directions) {
        Position next{position.row + direction.row, position.column + direction.column};
        if (traversable(next)) result.push_back(next);
    }
    return result;
}

std::optional<int> GridEnvironment::shortest_distance(Position from, Position target) const {
    if (!traversable(from) || !traversable(target)) return std::nullopt;
    const int distance = distances_from(from)[index(target)];
    return distance == -1 ? std::nullopt : std::optional<int>(distance);
}

std::optional<Position> GridEnvironment::shortest_step(Position from, Position target) const {
    if (!traversable(from) || !traversable(target)) return std::nullopt;
    if (from == target) return from;
    const auto& distances = distances_from(target);
    const int target_distance = distances[index(from)];
    if (target_distance <= 0) return std::nullopt;
    for (const Position next : legal_moves(from)) {
        if (distances[index(next)] == target_distance - 1) {
            return next;
        }
    }
    return std::nullopt;
}

std::vector<Position> GridEnvironment::view(Position center, int radius) const {
    std::vector<Position> result;
    for (int row = center.row - radius; row <= center.row + radius; ++row) {
        for (int column = center.column - radius; column <= center.column + radius; ++column) {
            Position candidate{row, column};
            const int distance = std::abs(row - center.row) + std::abs(column - center.column);
            if (in_bounds(candidate) && distance <= radius) result.push_back(candidate);
        }
    }
    return result;
}

Position GridEnvironment::random_legal_move(Position position, std::mt19937& random) const {
    auto moves = legal_moves(position);
    if (moves.empty()) return position;
    std::uniform_int_distribution<std::size_t> distribution(0, moves.size() - 1);
    return moves[distribution(random)];
}

std::size_t GridEnvironment::index(Position position) const {
    return static_cast<std::size_t>(position.row * columns_ + position.column);
}

const GridEnvironment::DistanceMap& GridEnvironment::distances_from(Position from) const {
    const std::size_t source = index(from);
    const auto cached = distance_cache_->find(source);
    if (cached != distance_cache_->end()) return cached->second;

    DistanceMap distances(static_cast<std::size_t>(rows_ * columns_), -1);
    std::queue<Position> frontier;
    frontier.push(from);
    distances[source] = 0;
    while (!frontier.empty()) {
        const Position current = frontier.front();
        frontier.pop();
        for (const Position next : legal_moves(current)) {
            const std::size_t next_index = index(next);
            if (distances[next_index] != -1) continue;
            distances[next_index] = distances[index(current)] + 1;
            frontier.push(next);
        }
    }
    const auto inserted = distance_cache_->emplace(source, std::move(distances));
    return inserted.first->second;
}

}  // namespace dmar