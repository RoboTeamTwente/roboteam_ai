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
        static QColor getColorForStatus(stp::Status status);
        static constexpr const char* getNameForStatus(stp::Status status);

        std::stringstream updateContent;
        MainWindow *parent = nullptr;

        void displayPlay(stp::Play* play);
        void displayRole(stp::Role* role, stp::Status state);
        void displayTactic(stp::Tactic* tactic);
        void displaySkill(stp::Skill* skill);

    public:
        explicit STPVisualizerWidget(MainWindow *parent);
        void updateContents(stp::Play* currentPlay);

    public slots:
        void outputStpData();
    };

}  // namespace rtt::ai::interface
#endif  // ROBOTEAM_AI_TREEVISUALIZER_H
