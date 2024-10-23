
#ifndef STATE_H
#define STATE_H

class State
{

public:
    // Constructors
    State() = default;
    State(double x_, double y_, double heading_ = 0)
        : x(x_), y(y_), heading(heading_), gridx(0), gridy(0), gheading(0) {}

    /* data */
    double x{};
    double y{};
    double heading{};

    double gridx{};
    double gridy{};
    double gheading{};

    // Define equality operator (==)
    bool operator==(const State &other) const
    {
        return std::abs(x - other.x) < 1e-9 &&
               std::abs(y - other.y) < 1e-9 &&
               std::abs(heading - other.heading) < 1e-9;
    }

    // Define less-than operator (<) for State comparison
    bool operator<(const State &other) const
    {
        // Compare x coordinates first
        if (x != other.x)
        {
            return x < other.x;
        }
        // If x is the same, compare y coordinates
        if (y != other.y)
        {
            return y < other.y;
        }
        // If both x and y are the same, compare heading
        return heading < other.heading;
    }
};

struct Circle
{
    Circle() = default;
    Circle(double x, double y, double r) : x(x), y(y), r(r) {}
    double x{};
    double y{};
    double r{};
};

struct StateHash
{
    std::size_t operator()(const State &s) const
    {
        // Combine the hashes of x, y, and heading to create a unique hash
        return ((std::hash<double>()(s.x) ^ (std::hash<double>()(s.y) << 1)) >> 1) ^ (std::hash<double>()(s.heading) << 1);
    }
};

#endif // STATE_H