//
// Created by rolf on 10-10-18.
//

#ifndef ROBOTEAM_AI_STRATEGYCOMPOSER_HPP
#define ROBOTEAM_AI_STRATEGYCOMPOSER_HPP


#include <boost/optional.hpp>
#include <map>
#include <string>

#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"

#include "bt/bt.hpp"
#include "roboteam_utils/LastRef.h"

#include "RefStateSwitch.hpp"

namespace rtt {

/**
 * \brief The value which indicates that a referee state has no special treatment in the strategy
 */
    extern const std::string UNSET;

/**
 * \class StrategyComposer
 * \brief Builds a single, static, horribly ugly Strategy tree by combining ones meant for different referee states.
 */
    class StrategyComposer {
    private:
        StrategyComposer() = delete;
        static std::shared_ptr<bt::BehaviorTree> mainStrategy;
        static void init();
        static bool initialized;
        class Forwarder final : public bt::Leaf {
        public:
            Forwarder(bt::Blackboard::Ptr bb, bt::Node::Ptr target);
            bt::Node::Status Update() override;
            void Initialize() override;
            void Terminate(bt::Node::Status status) override;
        private:
            bt::Node::Ptr target;
        };
    public:
        static std::shared_ptr<bt::BehaviorTree> getMainStrategy();

        // SET THIS IN StrategyComposer.cpp !!
        static const std::map<RefState, boost::optional<std::string>> MAPPING;
    };

}

#endif //ROBOTEAM_AI_STRATEGYCOMPOSER_HPP
