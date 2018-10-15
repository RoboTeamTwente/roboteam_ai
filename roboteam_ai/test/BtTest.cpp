#include <utility>

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
    explicit Once(const std::string& id)
            :Tracer("Once-" + id) { }
};

class Runner : public Tracer {
public:
    explicit Runner(const std::string& id)
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

    Counter(const std::string& id, int max)
            :Tracer("Counter-" + id), max{max}, runningCount{0} { }

    void Initialize_() override {
        runningCount = 0;
    }

    Status Update_() override {
        runningCount ++;

        if (runningCount == max) {
            return Status::Success;
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
        bt::Leaf::Ptr counter = std::make_shared<Counter>("A", 3);
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
        bt::Leaf::Ptr counter = std::make_shared<Counter>("A", 5);
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
        bt::Leaf::Ptr counterA = std::make_shared<Counter>("A", 2);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>("B", 2);
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
        bt::Leaf::Ptr counterA = std::make_shared<Counter>("A", 2);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>("B", 2);
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
        bt::Leaf::Ptr counterA = std::make_shared<Counter>("A", 2);
        bt::Leaf::Ptr counterB = std::make_shared<Counter>("B", 2);
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

  bt::Leaf::Ptr counterA = std::make_shared<Counter>("A", 2);
  bt::Leaf::Ptr counterB = std::make_shared<Counter>("B", 1);
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
  ASSERT_EQ(memSelector.index, 0);

  // return success if no children
  ASSERT_EQ(memSelector.Update(), bt::Node::Status::Success);

  // add two children to memSelector
  bt::Leaf::Ptr counterC = std::make_shared<Counter>("C", 2);
  bt::Leaf::Ptr counterD = std::make_shared<Counter>("D", 1);
  memSelector.AddChild(counterC);
  memSelector.AddChild(counterD);

  ASSERT_EQ(memSelector.Update(), bt::Node::Status::Running);
  ASSERT_EQ(memSelector.Update(), bt::Node::Status::Success);
}


TEST(BehaviorTreeTest, decorators) {
  bt::Leaf::Ptr child = std::make_shared<Counter>("A", 1);

  bt::Succeeder succeeder;
  ASSERT_EQ(succeeder.node_name(), "Succeeder");
  child = std::make_unique<Counter>("D", 1);
  succeeder.AddChild(child);
  ASSERT_EQ(succeeder.Update(), bt::Node::Status::Success);

  bt::Inverter inverter;
  ASSERT_EQ(inverter.node_name(), "Inverter");
  child = std::make_unique<Counter>("D", 1);
  inverter.AddChild(child);
  ASSERT_EQ(inverter.Update(), bt::Node::Status::Failure);

  child = std::make_shared<Once>("D");
  child->setStatus(bt::Node::Status::Failure);
  ASSERT_EQ(inverter.Tick(), bt::Node::Status::Failure);

  bt::Repeater repeater;
  ASSERT_EQ(repeater.node_name(), "Repeater");
  repeater.Initialize();

  bt::UntilFail untilFailer;
  ASSERT_EQ(untilFailer.node_name(), "UntilFail");

  bt::UntilSuccess untilSucceeder;
  ASSERT_EQ(untilSucceeder.node_name(), "UntilSuccess");
}

TEST(BehaviorTreeTest, StatusToString) {
  ASSERT_EQ(bt::statusToString(bt::Node::Status::Failure), "Failure");
  ASSERT_EQ(bt::statusToString(bt::Node::Status::Invalid), "Invalid");
  ASSERT_EQ(bt::statusToString(bt::Node::Status::Success), "Success");
  ASSERT_EQ(bt::statusToString(bt::Node::Status::Running), "Running");
}
