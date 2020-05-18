//
// Created by john on 4/30/20.
//

#ifndef RTT_INVARIANTSWIDGET_HPP
#define RTT_INVARIANTSWIDGET_HPP

#include <array>
#include <stp/invariants/BaseInvariant.h>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QTextEdit>

namespace rtt::ai::interface {
    class InvariantsWidget : public QTextEdit {
    Q_OBJECT
    private:
        /**
         * Map that holds all the invariants, ordered map because I want them to be alphabetical
         * Invariants are costless so this just takes up some memory for nothing ngl.
         * If we get some global instance of invariatns somewhere we could use it so that this is just
         * an std::reference_wrapper to it.
         */
        std::map<std::string, std::unique_ptr<stp::invariant::BaseInvariant>> invariants;
        std::stringstream data;
        std::mutex dataLock;
    	
        ~InvariantsWidget() override = default;

    public:
        /**
         * Constructor that sets readonly to true
         * @param parent Parent to pass to QTextEdit's constructor
         *
         * Sets this->invariants.
         */
        explicit InvariantsWidget(QWidget* parent = nullptr);

    public slots:
        /**
         * Slot that updates invariants gets them from this->invariants;
         * The behavior is undefined if this is called using a non-signal-slot style.
         */
        void updateInvariants();
    };

}

#endif //RTT_INVARIANTSWIDGET_HPP
