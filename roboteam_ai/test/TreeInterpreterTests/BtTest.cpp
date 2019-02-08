#include <gtest/gtest.h>
#include "roboteam_ai/src/bt/bt.hpp"

namespace {
std::vector<std::string> traces;

class Tracer : public bt::Leaf {
    public:
        std::string id;

        explicit Tracer(std::string id)
                :id{std::move(id)} { }

        void initialize() override {
            traces.push_back("Initialize: " + id);
            initialize_();
        }

        Status update() override {
            traces.push_back("Update: " + id);
            return update_();
        }

        void terminate(Status s) override {
            traces.push_back("Terminate: " + id);
            terminate_(s);
        }

        // Spoofed functions
        virtual void initialize_() { }

        virtual Status update_() { return status; }

        virtual void terminate_(Status) { }
};

class Once : public Tracer {
    public:
        explicit Once(const std::string &id)
                :Tracer("Once-" + id) { }
};

class Runner : public Tracer {
    public:
        explicit Runner(const std::string &id)
                :Tracer("Runner-" + id) { }

        Status update_() override {
            return Status::Running;
        }

        void terminate_(Status s) override {
            if (s == Status::Running || s == Status::Waiting) {
                setStatus(Status::Failure);
            }
        }
};

class Counter : public Tracer {
    public:
        int max;
        int runningCount;
        Status statusToReturn;

        Counter(Status statusToReturn, const std::string &id, int max)
                :Tracer("Counter-" + id), max{max}, runningCount{0} {
            this->statusToReturn = statusToReturn;
        }

        void initialize_() override {
            runningCount = 0;
        }

        Status update_() override {
            runningCount ++;

            if (runningCount == max) {
                return statusToReturn;
            }

            return Status::Running;
        }

        void terminate_(Status s) override {
            if (s == Status::Running || s == Status::Waiting) {
                setStatus(Status::Failure);
            }
        }
};

} // anonymous namespace

// Behavior Tree with one leaf //
TEST(BehaviorTreeTest, BehaviorTreeWithOneLeaf) {
    {
        bt::Leaf::Ptr once = std::make_shared<Once>("A");
        once->setStatus(bt::Node::Status::Success);
        bt::BehaviorTree bt(once);

        traces.clear();
        bt.tick();

        std::vector<std::string> expectedTrace = {
                "Initialize: Once-A",
                "Update: Once-A",
                "Terminate: Once-A"
        };

        EXPECT_EQ(expectedTrace, traces);
        EXPECT_EQ(bt.getStatus(), bt::Node::Status::Success);
    }
    {
        bt::Leaf::Ptr runner = std::make_shared<Runner>("A");
        bt::BehaviorTree bt(runner);

        traces.clear();
        bt.tick();
        bt.tick();
        bt.tick();
        bt.terminate(bt.getStatus());

        std::vector<std::string> expectedTrace = {
                "Initialize: Runner-A",
                "Update: Runner-A",
                "Update: Runner-A",
                "Update: Runner-A",
                "Terminate: Runner-A"
        };

        EXPECT_EQ(expectedTrace, traces);
        EXPECT_EQ(bt.getStatus(), bt::Node::Status::Failure);
    }
    {
        bt::Leaf::Ptr counter = std::make_shared<Counter>(bt::Node::Status::Success, "A", 3);
        bt::BehaviorTree bt(counter);

        traces.clear();
        bt.tick();
        bt.tick();
        bt.tick();

        std::vector<std::string> expectedTrace = {
                "Initialize: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Terminate: Counter-A"
        };

        EXPECT_EQ(expectedTrace, traces);
        EXPECT_EQ(bt.getStatus(), bt::Node::Status::Success);
    }
    {
        bt::Leaf::Ptr counter = std::make_shared<Counter>(bt::Node::Status::Success, "A", 5);
        bt::BehaviorTree bt(counter);

        traces.clear();
        bt.tick();
        bt.tick();
        bt.tick();
        bt.terminate(bt.getStatus());

        std::vector<std::string> expectedTrace = {
                "Initialize: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Terminate: Counter-A"
        };

        EXPECT_EQ(expectedTrace, traces);
        EXPECT_EQ(bt.getStatus(), bt::Node::Status::Failure);
    }
}

// Behavior Tree with counters & sequences
TEST(BehaviorTreeTest, BehaviorTreeWithSequencesAndCounters) {
    {
        bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 2);
        bt::MemSequence::Ptr memSeq = std::make_shared<bt::MemSequence>();

        // return success when no children
        EXPECT_EQ(memSeq->node_name(), "MemSequence");
        EXPECT_EQ(memSeq->update(), bt::Node::Status::Success);

        memSeq->addChild(counterA);
        memSeq->addChild(counterB);
        bt::BehaviorTree bt(memSeq);

        traces.clear();
        bt.tick();
        bt.tick();
        bt.tick();

        std::vector<std::string> expectedTrace = {
                "Initialize: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Terminate: Counter-A",
                "Initialize: Counter-B",
                "Update: Counter-B",
                "Update: Counter-B",
                "Terminate: Counter-B"
        };

        EXPECT_EQ(expectedTrace, traces);
        EXPECT_EQ(bt.getStatus(), bt::Node::Status::Success);
    }

    {
        bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 2);
        bt::Leaf::Ptr counterC = std::make_shared<Counter>(bt::Node::Status::Success, "C", 3);
        bt::Leaf::Ptr counterD = std::make_shared<Counter>(bt::Node::Status::Success, "D", 4);

        bt::ParallelSequence::Ptr parSeq = std::make_shared<bt::ParallelSequence>(3, 2);

        // return success when no children
        EXPECT_EQ(parSeq->node_name(), "ParallelSequence");

        parSeq->addChild(counterA);
        parSeq->addChild(counterB);
        parSeq->addChild(counterC);
        parSeq->addChild(counterD);

        bt::BehaviorTree bt;
        bt.SetRoot(parSeq);

        traces.clear();
        bt.tick();
        bt.tick();

        //after two ticks it should still be running
        EXPECT_TRUE(parSeq->IsRunning());

        bt.tick();
        bt.tick();

        EXPECT_TRUE(parSeq->IsSuccess());
        EXPECT_FALSE(parSeq->IsFailure());

    }

    {
        bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 1);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 1);
        bt::Leaf::Ptr counterC = std::make_shared<Counter>(bt::Node::Status::Success, "C", 2);

        bt::ParallelSequence::Ptr parSeq = std::make_shared<bt::ParallelSequence>(true, true);

        // return success when no children
        EXPECT_EQ(parSeq->node_name(), "ParallelSequence");

        parSeq->addChild(counterA);
        parSeq->addChild(counterB);
        parSeq->addChild(counterC);

        bt::BehaviorTree bt;
        bt.SetRoot(parSeq);

        bt.tick();
        EXPECT_TRUE(parSeq->IsRunning());
        bt.tick();
        EXPECT_TRUE(parSeq->IsSuccess());
    }

    {
        bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 2);
        bt::Sequence::Ptr seq = std::make_shared<bt::Sequence>();

        // return success when no children
        EXPECT_EQ(seq->node_name(), "Sequence");
        EXPECT_EQ(seq->update(), bt::Node::Status::Success);

        seq->addChild(counterA);
        seq->addChild(counterB);
        bt::BehaviorTree bt(seq);

        traces.clear();
        bt.tick();
        bt.tick();
        bt.tick();
        bt.tick();

        std::vector<std::string> expectedTrace = {
                "Initialize: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Terminate: Counter-A",
                "Initialize: Counter-B",
                "Update: Counter-B",
                "Initialize: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Terminate: Counter-A",
                "Update: Counter-B",
                "Terminate: Counter-B"
        };

        EXPECT_EQ(expectedTrace, traces);
        EXPECT_EQ(bt.getStatus(), bt::Node::Status::Success);
    }
    {
        bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 2);
        bt::Sequence::Ptr seq = std::make_shared<bt::Sequence>();
        seq->addChild(counterA);
        seq->addChild(counterB);
        bt::BehaviorTree bt(seq);

        traces.clear();
        bt.tick();
        bt.tick();
        bt.tick();
        bt.terminate(bt.getStatus());

        std::vector<std::string> expectedTrace = {
                "Initialize: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Terminate: Counter-A",
                "Initialize: Counter-B",
                "Update: Counter-B",
                "Initialize: Counter-A",
                "Update: Counter-A",
                "Terminate: Counter-A",
                "Terminate: Counter-B"
        };

        EXPECT_EQ(expectedTrace, traces);
        EXPECT_EQ(bt.getStatus(), bt::Node::Status::Failure);
    }
}

TEST(BehaviorTreeTest, composities) {
    bt::MemParallelSequence memParallelSequence;
    EXPECT_EQ(memParallelSequence.node_name(), "MemParallelSequence");
    EXPECT_EQ(memParallelSequence.getStatus(), bt::Node::Status::Waiting);
    EXPECT_EQ(memParallelSequence.update(), bt::Node::Status::Success);

    bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
    memParallelSequence.addChild(counterA);

    memParallelSequence.initialize();
    EXPECT_EQ(memParallelSequence.update(), bt::Node::Status::Running);
    EXPECT_EQ(memParallelSequence.update(), bt::Node::Status::Success);

    bt::MemParallelSequence memParallelSequence2;
    bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
    bt::Leaf::Ptr counterC = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
    memParallelSequence2.addChild(counterB);
    memParallelSequence2.addChild(counterC);
    memParallelSequence2.initialize();
    EXPECT_EQ(memParallelSequence2.update(), bt::Node::Status::Running);
    EXPECT_EQ(memParallelSequence2.update(), bt::Node::Status::Success);

    bt::MemParallelSequence memParallelSequence3;
    bt::Leaf::Ptr counterD = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
    bt::Leaf::Ptr counterE = std::make_shared<Counter>(bt::Node::Status::Failure, "A", 2);
    memParallelSequence3.addChild(counterD);
    memParallelSequence3.addChild(counterE);
    memParallelSequence3.initialize();
    EXPECT_EQ(memParallelSequence3.update(), bt::Node::Status::Running);
    EXPECT_EQ(memParallelSequence3.update(), bt::Node::Status::Failure);
}

TEST(BehaviorTreeTest, selectorComposites) {
    // selector
    bt::Selector selector;
    EXPECT_EQ(selector.node_name(), "Selector");
    // it should be Failure when initialized
    EXPECT_EQ(selector.getStatus(), bt::Node::Status::Waiting);
    // it should fail after the first update with no children
    EXPECT_EQ(selector.update(), bt::Node::Status::Failure);

    bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
    bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 1);
    selector.addChild(counterA);
    selector.addChild(counterB);

    EXPECT_EQ(selector.update(), bt::Node::Status::Running);
    EXPECT_EQ(selector.update(), bt::Node::Status::Success);

    // memSelector
    bt::MemSelector memSelector;
    EXPECT_EQ(memSelector.node_name(), "MemSelector");

    EXPECT_EQ(memSelector.getStatus(), bt::Node::Status::Waiting);
//    memSelector.index = 22; //TODO: Fix this test later sometime.
//    memSelector.initialize();
//    EXPECT_EQ(memSelector.index, (unsigned int) 0);

    // return success if no children
    EXPECT_EQ(memSelector.update(), bt::Node::Status::Success);

    // add two children to memSelector
    bt::Leaf::Ptr counterC = std::make_shared<Counter>(bt::Node::Status::Success, "C", 2);
    bt::Leaf::Ptr counterD = std::make_shared<Counter>(bt::Node::Status::Success, "D", 1);
    memSelector.addChild(counterC);
    memSelector.addChild(counterD);

    EXPECT_EQ(memSelector.update(), bt::Node::Status::Running);
    EXPECT_EQ(memSelector.update(), bt::Node::Status::Success);
}

TEST(BehaviorTreeTest, decorators) {
    bt::Leaf::Ptr child = std::make_shared<Counter>(bt::Node::Status::Success, "A", 1);

    bt::Succeeder succeeder;
    EXPECT_EQ(succeeder.node_name(), "Succeeder");
    child = std::make_shared<Counter>(bt::Node::Status::Success, "D", 1);
    succeeder.addChild(child);
    EXPECT_EQ(succeeder.update(), bt::Node::Status::Success);

    bt::Failer failer;
    EXPECT_EQ(failer.node_name(), "Failer");
    child = std::make_shared<Counter>(bt::Node::Status::Success, "D", 1);
    failer.addChild(child);
    EXPECT_EQ(failer.update(), bt::Node::Status::Failure);

    bt::Inverter inverter;
    EXPECT_EQ(inverter.node_name(), "Inverter");
    child = std::make_shared<Counter>(bt::Node::Status::Success, "D", 1);
    inverter.addChild(child);
    EXPECT_EQ(inverter.update(), bt::Node::Status::Failure);

    bt::Inverter inverter2;
    child = std::make_shared<Counter>(bt::Node::Status::Failure, "D", 1);
    inverter2.addChild(child);
    EXPECT_EQ(inverter2.update(), bt::Node::Status::Success);

    bt::Repeater repeater;
    EXPECT_EQ(repeater.node_name(), "Repeater");
    child = std::make_shared<Counter>(bt::Node::Status::Success, "D", 1);
    child->setStatus(bt::Node::Status::Failure);
    repeater.addChild(child);
    repeater.initialize();
    EXPECT_EQ(repeater.update(), bt::Node::Status::Running);

    bt::UntilFail untilFailer;
    EXPECT_EQ(untilFailer.node_name(), "UntilFail");

    child = std::make_shared<Counter>(bt::Node::Status::Success, "D", 1);
    untilFailer.addChild(child);
    EXPECT_EQ(untilFailer.update(), bt::Node::Status::Running);

    bt::UntilFail untilFailer2;
    child = std::make_shared<Counter>(bt::Node::Status::Failure, "D", 1);
    untilFailer2.addChild(child);
    EXPECT_EQ(untilFailer2.update(), bt::Node::Status::Success);

    bt::UntilSuccess untilSucceeder;
    EXPECT_EQ(untilSucceeder.node_name(), "UntilSuccess");

    child = std::make_shared<Counter>(bt::Node::Status::Success, "D", 2);
    child->setStatus(bt::Node::Status::Failure);
    untilSucceeder.addChild(child);
    EXPECT_EQ(untilSucceeder.update(), bt::Node::Status::Running);
    EXPECT_EQ(untilSucceeder.update(), bt::Node::Status::Success);
}

TEST(BehaviorTreeTest, StatusToString) {
    EXPECT_EQ(bt::statusToString(bt::Node::Status::Failure), "Failure");
    EXPECT_EQ(bt::statusToString(bt::Node::Status::Waiting), "Waiting");
    EXPECT_EQ(bt::statusToString(bt::Node::Status::Success), "Success");
    EXPECT_EQ(bt::statusToString(bt::Node::Status::Running), "Running");
}

//TODO: fix that this goes out of the namespace. Currently is hard to do because the FRIEND_TEST is within the namespace in BehaviorTree.hpp
namespace bt {
TEST(BehaviorTreeTest, it_sets_blackboards) {
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->setDouble("A1", 12);

    bt::BehaviorTree tree(bb);

    EXPECT_TRUE(tree.globalBB->hasDouble("A1"));
    EXPECT_EQ(tree.globalBB->getDouble("A1"), 12);

    bt::Blackboard::Ptr bb2 = std::make_shared<bt::Blackboard>();
    bb2->setDouble("A1", 55);

    tree.SetGlobalBlackboard(bb2);

    EXPECT_TRUE(tree.globalBB->hasDouble("A1"));
    EXPECT_EQ(tree.globalBB->getDouble("A1"), 55);
}
}

TEST(BehaviorTreeTest, it_terminates_nodes) {
    bt::Succeeder succeeder;
    succeeder.addChild(std::make_shared<Counter>(bt::Node::Status::Failure, "D", 2));
    succeeder.terminate(bt::Node::Status::Success);
    EXPECT_EQ(succeeder.getStatus(), bt::Node::Status::Waiting);

    bt::Succeeder succeeder2;
    succeeder2.addChild(std::make_shared<Counter>(bt::Node::Status::Failure, "D", 2));
    succeeder2.update();

    succeeder2.terminate(bt::Node::Status::Running);
    EXPECT_EQ(succeeder2.getStatus(), bt::Node::Status::Failure);

    bt::Succeeder succeeder3;
    succeeder3.addChild(std::make_shared<Counter>(bt::Node::Status::Failure, "D", 2));
    succeeder3.update();

    succeeder3.terminate(bt::Node::Status::Failure);
    EXPECT_EQ(succeeder3.getStatus(), bt::Node::Status::Waiting);
}