#ifndef NODE_H
#define NODE_H

#include <vector>
#include <memory>
#include "State.h"

using namespace std;

namespace planner
{
    class Node
    {
    public:
        // Constructor
        Node(const State &Current_state, const vector<State> &Trajectory, double Cost_path, double steeringAngle, int direction, std::weak_ptr<Node> Parent)
            : Current_state(Current_state), Trajectory(Trajectory), Cost_path(Cost_path), steeringAngle(steeringAngle), direction(direction), Parent(Parent)
        {
        }

        // Variables
        State Current_state;
        vector<State> Trajectory;
        double Cost_path;
        double steeringAngle; // heading
        int direction;
        std::weak_ptr<Node> Parent; // Use weak_ptr to prevent cyclic references
    };

    class HolonomicNode
    {
    public:
        // Default constructor
        HolonomicNode()
            : Current_state(), cost(1000), Parent()
        {
        }

        // Parameterized constructor
        HolonomicNode(const State &Current_state, double cost, std::weak_ptr<HolonomicNode> Parent)
            : Current_state(Current_state), cost(cost), Parent(Parent)
        {
        }

        // Variables
        State Current_state;
        double cost;
        std::weak_ptr<HolonomicNode> Parent; // Use weak_ptr
    };

    class NodeGreater
    {
    public:
        bool operator()(const std::shared_ptr<HolonomicNode> &node1, const std::shared_ptr<HolonomicNode> &node2) const
        {
            return node1->cost > node2->cost;
        }
    };

} // namespace planner

#endif // NODE_H