#pragma once

#include "bt/Composite.hpp"
namespace bt {

    class MemSelector : public Composite {
        FRIEND_TEST(BehaviorTreeTest, selectorComposites);

        private:
        size_t index = 0;

        public:
        /**
         * Constructor with parameter of the children for the mem selector
         * @param children nodes that must be given in order from left to right,
         * left getting ticked first
         */
        MemSelector(nvector children);

        MemSelector();
        void initialize() override;
        Status update() override;
        std::string node_name() override { return "MemSelector"; };
    };
}  // namespace bt
