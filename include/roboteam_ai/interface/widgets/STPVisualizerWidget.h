//
// Created by mrlukasbos on 1-2-19.
//

#ifndef ROBOTEAM_AI_TREEVISUALIZER_H
#define ROBOTEAM_AI_TREEVISUALIZER_H

#include <QtWidgets/QTextEdit>
#include <sstream>

#include "mutex"
#include "utilities/StpInfoEnums.h"
#include "world/Field.h"

namespace rtt::ai::stp {
class Play;
class Role;
class Tactic;
class Skill;
}  // namespace rtt::ai::stp

namespace rtt::ai::interface {

class MainWindow;

/**
 * Class that visualizes the new STP system.
 */
class STPVisualizerWidget : public QTextEdit {
    Q_OBJECT

    FRIEND_TEST(TreeVisualizerTest, it_properly_displays_trees);

    FRIEND_TEST(TreeVisualizerTest, it_sets_proper_color_for_status);

   private:
    /**
     * Gets the color for a status code
     * @param status Status value
     * @return a view to a (static constexpr) char*
     *
     * See impl.
     */
    std::string_view getColorForStatus(stp::Status status);

    /**
     * Mutex that's locked whenever content is edited or read
     */
    std::mutex contentLock;

    /**
     * Content that's used for updating the html edit
     */
    std::stringstream updateContent;

    /**
     * Main window pointer, used for signaling the other (keeper tree) widget
     */
    MainWindow* parent = nullptr;

    /**
     * Displays play, calls displayRole, tactic and skill to display
     * them respectively
     * @param play Play to display, null is checked
     */
    void displayPlay(stp::Play* play);

    /**
     * Displays role
     * @param role Role to display
     * @param state State of this Role
     * @param last true if it's the last role, for future, might want to append more data
     * @param updatingForKeeper true if you're updating for keeper,
     * to prevent recursive calls
     */
    void displayRole(stp::Role* role, stp::Status state, bool last, bool updatingForKeeper = false);

    /**
     * Displays tactic
     * @param tactic tactic to display
     * @param last same as for displayRole
     */
    void displayTactic(stp::Tactic* tactic, bool last);

    /**
     * Displays a skill
     * @param skill Skill to display
     * @param last same as displayRole
     */
    void displaySkill(stp::Skill* skill, bool last);

    /**
     * Outputs a status in the correct format.
     * @param status status to output
     *
     * stream << getColor(status) << status << white;
     */
    void outputStatus(stp::Status status);

   public slots:
    /**
     * Calls this->setHtml(updateContent);
     */
    void outputStpData();

   public:
    /**
     * Constructor, sets `this->parent` and makes it readonly
     * @param parent Parent to set this->parent to
     */
    explicit STPVisualizerWidget(MainWindow* parent);

    /**
     * Clears stream, displayPlay() and outputStpData();
     * @param currentPlay
     */
    void updateContents(stp::Play* currentPlay);

    /**
     * Updates `this` to reflect the keeper content
     * @param pRole Role pointer that the keerp has
     * @param state State of this Role.
     */
    void updateKeeperContents(stp::Role* pRole, stp::Status state);
};
}  // namespace rtt::ai::interface
#endif  // ROBOTEAM_AI_TREEVISUALIZER_H
