# compile all files in bt directory
# file(GLOB BT_SOURCES *.cpp *.hpp)

set(BT_SOURCES
        composites/MemSelector.hpp
        composites/MemSelector.cpp
        composites/MemSequence.hpp
        composites/MemSequence.cpp
        composites/ParallelSequence.hpp
        composites/ParallelSequence.cpp
        composites/Selector.hpp
        composites/Selector.cpp
        composites/Sequence.hpp
        composites/Sequence.cpp
        decorators/Failer.hpp
        decorators/Failer.hpp
        decorators/Inverter.hpp
        decorators/Inverter.cpp
        decorators/Repeater.hpp
        decorators/Repeater.cpp
        decorators/Succeeder.hpp
        decorators/Succeeder.cpp
        decorators/UntilFail.hpp
        decorators/UntilFail.cpp
        decorators/UntilSuccess.hpp
        decorators/UntilSuccess.cpp

        BehaviorTree.hpp
        BehaviorTree.cpp
        Blackboard.hpp
        Blackboard.cpp
        Composite.hpp
        Composite.cpp
        Decorator.hpp
        Decorator.cpp
        Leaf.hpp
        Leaf.cpp
        Node.hpp
        Node.cpp
        )

add_library(ai_bt
        ${BT_SOURCES}
        )
add_dependencies(ai_bt
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )


