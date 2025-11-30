# Behavior Tree Robot Task

A C++ implementation of a behavior tree for a robot task where the robot navigates to a room, opens a fridge, picks an apple, and exits the room.

## Overview

This project demonstrates the use of BehaviorTree.CPP to create a modular behavior tree for robotic task execution. The robot follows a predefined sequence of actions to complete its mission, with conditional checks and fallback behaviors.

## Task Flow

```
Start → Move to Room Door → Check Door → [If Closed: Open Door] → Enter Room → 
Move to Fridge Door → Check Door → [If Closed: Open Door] → Find Apple → 
Pick Apple → Close Fridge Door → Move to Room Door → Exit Room → End
```

## Features

- Modular behavior tree design using BehaviorTree.CPP
- Clean separation of actions and conditions
- Fallback behaviors for door handling
- Comprehensive logging of execution flow
- Easy to extend with new behaviors

## Prerequisites

- Ubuntu 18.04 or higher (tested on Ubuntu 20.04/22.04)
- CMake (version 3.5 or higher)
- C++14 compatible compiler
- BehaviorTree.CPP library

## Installation

### Install BehaviorTree.CPP

```bash
sudo apt-get update
sudo apt-get install libbehaviortree-cpp-dev
```

### Clone and Build

```bash
# Clone the repository
git clone https://github.com/your-username/behavior-tree-robot-task.git
cd behavior-tree-robot-task

# Create build directory and compile
mkdir build
cd build
cmake ..
make
```

## Project Structure

```
behavior_tree_task/
├── CMakeLists.txt          # Build configuration
├── src/
│   ├── main.cpp           # Main application and tree definition
│   ├── nodes.h            # Node class declarations
│   └── nodes.cpp          # Node implementations
└── README.md
```

## Usage

Run the behavior tree executable:

```bash
./build/bt_task
```

### Expected Output

```
=== Starting Behavior Tree Execution ===
[MoveTowardsDoorOfRoom] Moving towards the door of the room...
[IsDoorClosed] Checking if room door is closed: YES
[OpenDoor] Opening the door...
[EnterRoom] Entering the room...
[MoveTowardsDoorOfFridge] Moving towards the door of the fridge...
[IsDoorClosed] Checking if fridge door is closed: YES
[OpenDoor] Opening the door...
[FindApple] Finding the apple in the fridge...
[PickApple] Picking the apple...
[CloseDoorOfFridge] Closing the door of the fridge...
[MoveTowardsDoorOfRoom] Moving towards the door of the room...
[ExitRoom] Exiting the room...
=== Behavior Tree Execution Completed ===
```

## Behavior Tree Nodes

### Action Nodes
- `MoveTowardsDoorOfRoom` - Navigate to room entrance
- `OpenDoor` - Open closed doors
- `EnterRoom` - Enter through doorway
- `MoveTowardsDoorOfFridge` - Navigate to fridge
- `FindApple` - Locate apple in fridge
- `PickApple` - Pick up the apple
- `CloseDoorOfFridge` - Close fridge door
- `ExitRoom` - Leave the room

### Condition Nodes
- `IsDoorClosed` - Check door status (room or fridge)

## Customization

### Adding New Nodes

1. Declare the node in `src/nodes.h`:
```cpp
class NewAction : public BT::SyncActionNode
{
public:
    NewAction(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override;
};
```

2. Implement in `src/nodes.cpp`:
```cpp
BT::NodeStatus NewAction::tick()
{
    std::cout << "[NewAction] Performing action..." << std::endl;
    return BT::NodeStatus::SUCCESS;
}
```

3. Register in `RegisterNodes` function:
```cpp
factory.registerNodeType<NewAction>("NewAction");
```

### Modifying the Behavior Tree

Edit the XML tree in `src/main.cpp` to change the execution flow or add new behaviors.

## Dependencies

- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) - Behavior Tree library
- CMake - Build system
- C++14 Standard Library

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- BehaviorTree.CPP library for providing the behavior tree framework
- ROS community for inspiration in robotic task planning

## Related Projects

- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) - The underlying behavior tree library
- [ROS2 Behavior Trees](https://github.com/ros-planning/navigation2) - ROS2 navigation with behavior trees

---

For questions or support, please open an issue on GitHub.
