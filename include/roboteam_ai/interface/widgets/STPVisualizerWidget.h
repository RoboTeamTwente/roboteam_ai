//
// Created by mrlukasbos on 1-2-19.
//

#ifndef ROBOTEAM_AI_TREEVISUALIZER_H
#define ROBOTEAM_AI_TREEVISUALIZER_H

#include <sstream>

#include <QtWidgets/QTextEdit>
#include <include/roboteam_ai/stp/StpInfo.h>

namespace rtt::ai::stp {
    class Play;
    class Role;
    class Tactic;
    class Skill;
}

namespace rtt::ai::interface {

    class MainWindow;

    class STPVisualizerWidget : public QTextEdit {
    Q_OBJECT

        FRIEND_TEST(TreeVisualizerTest, it_properly_displays_trees);

        FRIEND_TEST(TreeVisualizerTest, it_sets_proper_color_for_status);

    private:
        std::string_view getColorForStatus(stp::Status status);

        std::mutex contentLock;
        std::stringstream updateContent;
        MainWindow *parent = nullptr;

        void displayPlay(stp::Play* play);
        void displayRole(stp::Role* role, stp::Status state, bool last, bool updatingForKeeper = false);
        void displayTactic(stp::Tactic* tactic, bool last);
        void displaySkill(stp::Skill* skill, bool last);

        void outputStatus(stp::Status status);

    public:
        explicit STPVisualizerWidget(MainWindow *parent);
        void updateContents(stp::Play* currentPlay);
        void updateKeeperContents(stp::Role *pRole, stp::Status state);

    public slots:
        void outputStpData();

    };

}  // namespace rtt::ai::interface
#endif  // ROBOTEAM_AI_TREEVISUALIZER_H
