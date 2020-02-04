add_library(bt
        composites/MemSelector.cpp
        composites/MemSequence.cpp
        composites/ParallelSequence.cpp
        composites/MemParallelSequence.cpp
        composites/Selector.cpp
        composites/Sequence.cpp
        decorators/Failer.cpp
        decorators/Inverter.cpp
        decorators/Repeater.cpp
        decorators/Succeeder.cpp
        decorators/UntilFail.cpp
        decorators/UntilSuccess.cpp
        BehaviorTree.cpp
        Blackboard.cpp
        Blackboard.cpp
        Composite.cpp
        Decorator.cpp
        Leaf.cpp
        Node.cpp
        RoleDivider.cpp
        Role.cpp
)
target_link_libraries(bt
        PUBLIC roboteam_utils
        )
target_include_directories(bt
        INTERFACE include
        PRIVATE include/bt
        )
add_library(rtt::bt ALIAS bt)