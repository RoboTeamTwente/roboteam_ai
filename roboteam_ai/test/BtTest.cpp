#include <gtest/gtest.h>
#include "../src/bt/bt.hpp"

namespace {
std::vector<std::string> traces;

class Tracer : public bt::Leaf {
    public:
        std::string id;

        explicit Tracer(std::string id)
                :id{std::move(id)} { }

        void Initialize() override {
            traces.push_back("Initialize: " + id);
            Initialize_();
        }

        Status Update() override {
            traces.push_back("Update: " + id);
            return Update_();
        }

        void Terminate(Status s) override {
            traces.push_back("Terminate: " + id);
            Terminate_(s);
        }

        // Spoofed functions
        virtual void Initialize_() { }

        virtual Status Update_() { return status; }

        virtual void Terminate_(Status) { }
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

        Status Update_() override {
            return Status::Running;
        }

        void Terminate_(Status s) override {
            if (s == Status::Running || s == Status::Invalid) {
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

        void Initialize_() override {
            runningCount = 0;
        }

        Status Update_() override {
            runningCount ++;

            if (runningCount == max) {
                return statusToReturn;
            }

            return Status::Running;
        }

        void Terminate_(Status s) override {
            if (s == Status::Running || s == Status::Invalid) {
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
        bt.Tick();

        std::vector<std::string> expectedTrace = {
                "Initialize: Once-A",
                "Update: Once-A",
                "Terminate: Once-A"
        };

        ASSERT_EQ(expectedTrace, traces);
        ASSERT_EQ(bt.getStatus(), bt::Node::Status::Success);
    }
    {
        bt::Leaf::Ptr runner = std::make_shared<Runner>("A");
        bt::BehaviorTree bt(runner);

        traces.clear();
        bt.Tick();
        bt.Tick();
        bt.Tick();
        bt.Terminate(bt.getStatus());

        std::vector<std::string> expectedTrace = {
                "Initialize: Runner-A",
                "Update: Runner-A",
                "Update: Runner-A",
                "Update: Runner-A",
                "Terminate: Runner-A"
        };

        ASSERT_EQ(expectedTrace, traces);
        ASSERT_EQ(bt.getStatus(), bt::Node::Status::Failure);
    }
    {
        bt::Leaf::Ptr counter = std::make_shared<Counter>(bt::Node::Status::Success, "A", 3);
        bt::BehaviorTree bt(counter);

        traces.clear();
        bt.Tick();
        bt.Tick();
        bt.Tick();

        std::vector<std::string> expectedTrace = {
                "Initialize: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Terminate: Counter-A"
        };

        ASSERT_EQ(expectedTrace, traces);
        ASSERT_EQ(bt.getStatus(), bt::Node::Status::Success);
    }
    {
        bt::Leaf::Ptr counter = std::make_shared<Counter>(bt::Node::Status::Success, "A", 5);
        bt::BehaviorTree bt(counter);

        traces.clear();
        bt.Tick();
        bt.Tick();
        bt.Tick();
        bt.Terminate(bt.getStatus());

        std::vector<std::string> expectedTrace = {
                "Initialize: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Update: Counter-A",
                "Terminate: Counter-A"
        };

        ASSERT_EQ(expectedTrace, traces);
        ASSERT_EQ(bt.getStatus(), bt::Node::Status::Failure);
    }
}

// Behavior Tree with counters & sequences
TEST(BehaviorTreeTest, BehaviorTreeWithSequencesAndCounters) {
    {
        bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 2);
        bt::MemSequence::Ptr memSeq = std::make_shared<bt::MemSequence>();

        // return success when no children
        ASSERT_EQ(memSeq->node_name(), "MemSequence");
        ASSERT_EQ(memSeq->Update(), bt::Node::Status::Success);

        memSeq->AddChild(counterA);
        memSeq->AddChild(counterB);
        bt::BehaviorTree bt(memSeq);

        traces.clear();
        bt.Tick();
        bt.Tick();
        bt.Tick();

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

        ASSERT_EQ(expectedTrace, traces);
        ASSERT_EQ(bt.getStatus(), bt::Node::Status::Success);
    }

    {
        bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 2);
        bt::Leaf::Ptr counterC = std::make_shared<Counter>(bt::Node::Status::Success, "C", 3);
        bt::Leaf::Ptr counterD = std::make_shared<Counter>(bt::Node::Status::Success, "D", 4);

        bt::ParallelSequence::Ptr parSeq = std::make_shared<bt::ParallelSequence>(3, 2);

        // return success when no children
        ASSERT_EQ(parSeq->node_name(), "ParallelSequence");

        parSeq->AddChild(counterA);
        parSeq->AddChild(counterB);
        parSeq->AddChild(counterC);
        parSeq->AddChild(counterD);

        bt::BehaviorTree bt;
        bt.SetRoot(parSeq);

        traces.clear();
        bt.Tick();
        bt.Tick();

        //after two ticks it should still be running
        ASSERT_TRUE(parSeq->IsRunning());

        bt.Tick();
        bt.Tick();

        ASSERT_TRUE(parSeq->IsSuccess());
        ASSERT_FALSE(parSeq->IsFailure());

    }

    {
        bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 1);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 1);
        bt::Leaf::Ptr counterC = std::make_shared<Counter>(bt::Node::Status::Success, "C", 2);

        bt::ParallelSequence::Ptr parSeq = std::make_shared<bt::ParallelSequence>(true, true);

        // return success when no children
        ASSERT_EQ(parSeq->node_name(), "ParallelSequence");

        parSeq->AddChild(counterA);
        parSeq->AddChild(counterB);
        parSeq->AddChild(counterC);

        bt::BehaviorTree bt;
        bt.SetRoot(parSeq);

        bt.Tick();
        ASSERT_TRUE(parSeq->IsRunning());
        bt.Tick();
        ASSERT_TRUE(parSeq->IsSuccess());
    }

    {
        bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 2);
        bt::Sequence::Ptr seq = std::make_shared<bt::Sequence>();

        // return success when no children
        ASSERT_EQ(seq->node_name(), "Sequence");
        ASSERT_EQ(seq->Update(), bt::Node::Status::Success);

        seq->AddChild(counterA);
        seq->AddChild(counterB);
        bt::BehaviorTree bt(seq);

        traces.clear();
        bt.Tick();
        bt.Tick();
        bt.Tick();
        bt.Tick();

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

        ASSERT_EQ(expectedTrace, traces);
        ASSERT_EQ(bt.getStatus(), bt::Node::Status::Success);
    }
    {
        bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 2);
        bt::Sequence::Ptr seq = std::make_shared<bt::Sequence>();
        seq->AddChild(counterA);
        seq->AddChild(counterB);
        bt::BehaviorTree bt(seq);

        traces.clear();
        bt.Tick();
        bt.Tick();
        bt.Tick();
        bt.Terminate(bt.getStatus());

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

        ASSERT_EQ(expectedTrace, traces);
        ASSERT_EQ(bt.getStatus(), bt::Node::Status::Failure);
    }
}

TEST(BehaviorTreeTest, selectorComposites) {
    // selector
    bt::Selector selector;
    ASSERT_EQ(selector.node_name(), "Selector");
    // it should be invalid when initialized
    ASSERT_EQ(selector.getStatus(), bt::Node::Status::Invalid);
    // it should fail after the first update with no children
    ASSERT_EQ(selector.Update(), bt::Node::Status::Failure);

    bt::Leaf::Ptr counterA = std::make_shared<Counter>(bt::Node::Status::Success, "A", 2);
    bt::Leaf::Ptr counterB = std::make_shared<Counter>(bt::Node::Status::Success, "B", 1);
    selector.AddChild(counterA);
    selector.AddChild(counterB);

    ASSERT_EQ(selector.Update(), bt::Node::Status::Running);
    ASSERT_EQ(selector.Update(), bt::Node::Status::Success);

    // memSelector
    bt::MemSelector memSelector;
    ASSERT_EQ(memSelector.node_name(), "MemSelector");

    ASSERT_EQ(memSelector.getStatus(), bt::Node::Status::Invalid);
    memSelector.index = 22;
    memSelector.Initialize();
    ASSERT_EQ(memSelector.index, (unsigned int) 0);

    // return success if no children
    ASSERT_EQ(memSelector.Update(), bt::Node::Status::Success);

    // add two children to memSelector
    bt::Leaf::Ptr counterC = std::make_shared<Counter>(bt::Node::Status::Success, "C", 2);
    bt::Leaf::Ptr counterD = std::make_shared<Counter>(bt::Node::Status::Success, "D", 1);
    memSelector.AddChild(counterC);
    memSelector.AddChild(counterD);

    ASSERT_EQ(memSelector.Update(), bt::Node::Status::Running);
    ASSERT_EQ(memSelector.Update(), bt::Node::Status::Success);
}

TEST(BehaviorTreeTest, decorators) {
    bt::Leaf::Ptr child = std::make_shared<Counter>(bt::Node::Status::Success, "A", 1);

    bt::Succeeder succeeder;
    ASSERT_EQ(succeeder.node_name(), "Succeeder");
    child = std::make_unique<Counter>(bt::Node::Status::Success, "D", 1);
    succeeder.AddChild(child);
    ASSERT_EQ(succeeder.Update(), bt::Node::Status::Success);

    bt::Failer failer;
    ASSERT_EQ(failer.node_name(), "Failer");
    child = std::make_unique<Counter>(bt::Node::Status::Success, "D", 1);
    failer.AddChild(child);
    ASSERT_EQ(failer.Update(), bt::Node::Status::Failure);

    bt::Inverter inverter;
    ASSERT_EQ(inverter.node_name(), "Inverter");
    child = std::make_unique<Counter>(bt::Node::Status::Success, "D", 1);
    inverter.AddChild(child);
    ASSERT_EQ(inverter.Update(), bt::Node::Status::Failure);

    bt::Inverter inverter2;
    child = std::make_unique<Counter>(bt::Node::Status::Failure, "D", 1);
    inverter2.AddChild(child);
    ASSERT_EQ(inverter2.Update(), bt::Node::Status::Success);

    bt::Repeater repeater;
    ASSERT_EQ(repeater.node_name(), "Repeater");
    child = std::make_unique<Counter>(bt::Node::Status::Success, "D", 1);
    child->setStatus(bt::Node::Status::Failure);
    repeater.AddChild(child);
    repeater.Initialize();
    ASSERT_EQ(repeater.Update(), bt::Node::Status::Running);

    bt::UntilFail untilFailer;
    ASSERT_EQ(untilFailer.node_name(), "UntilFail");

    child = std::make_unique<Counter>(bt::Node::Status::Success, "D", 1);
    untilFailer.AddChild(child);
    ASSERT_EQ(untilFailer.Update(), bt::Node::Status::Running);

    bt::UntilFail untilFailer2;
    child = std::make_unique<Counter>(bt::Node::Status::Failure, "D", 1);
    untilFailer2.AddChild(child);
    ASSERT_EQ(untilFailer2.Update(), bt::Node::Status::Success);

    bt::UntilSuccess untilSucceeder;
    ASSERT_EQ(untilSucceeder.node_name(), "UntilSuccess");

    child = std::make_unique<Counter>(bt::Node::Status::Success, "D", 2);
    child->setStatus(bt::Node::Status::Failure);
    untilSucceeder.AddChild(child);
    ASSERT_EQ(untilSucceeder.Update(), bt::Node::Status::Running);
    ASSERT_EQ(untilSucceeder.Update(), bt::Node::Status::Success);
}

TEST(BehaviorTreeTest, StatusToString) {
    ASSERT_EQ(bt::statusToString(bt::Node::Status::Failure), "Failure");
    ASSERT_EQ(bt::statusToString(bt::Node::Status::Invalid), "Invalid");
    ASSERT_EQ(bt::statusToString(bt::Node::Status::Success), "Success");
    ASSERT_EQ(bt::statusToString(bt::Node::Status::Running), "Running");
}

//TODO: fix that this goes out of the namespace. Currently is hard to do because the FRIEND_TEST is within the namespace in BehaviorTree.hpp
namespace bt {
TEST(BehaviorTreeTest, it_sets_blackboards) {
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->setDouble("A1", 12);

    bt::BehaviorTree tree(bb);

    ASSERT_TRUE(tree.globalBB->hasDouble("A1"));
    ASSERT_EQ(tree.globalBB->getDouble("A1"), 12);

    bt::Blackboard::Ptr bb2 = std::make_shared<bt::Blackboard>();
    bb2->setDouble("A1", 55);

    tree.SetGlobalBlackboard(bb2);

    ASSERT_TRUE(tree.globalBB->hasDouble("A1"));
    ASSERT_EQ(tree.globalBB->getDouble("A1"), 55);
}
}



TEST(BehaviorTreeTest, it_terminates_nodes) {
    bt::Succeeder succeeder;
    succeeder.AddChild(std::make_unique<Counter>(bt::Node::Status::Failure, "D", 2));
    succeeder.Terminate(bt::Node::Status::Success);
    ASSERT_EQ(succeeder.getStatus(), bt::Node::Status::Invalid);

    bt::Succeeder succeeder2;
    succeeder2.AddChild(std::make_unique<Counter>(bt::Node::Status::Failure, "D", 2));
    succeeder2.Update();

    succeeder2.Terminate(bt::Node::Status::Running);
    ASSERT_EQ(succeeder2.getStatus(), bt::Node::Status::Failure);

    bt::Succeeder succeeder3;
    succeeder3.AddChild(std::make_unique<Counter>(bt::Node::Status::Failure, "D", 2));
    succeeder3.Update();

    succeeder3.Terminate(bt::Node::Status::Failure);
    ASSERT_EQ(succeeder3.getStatus(), bt::Node::Status::Invalid);
}