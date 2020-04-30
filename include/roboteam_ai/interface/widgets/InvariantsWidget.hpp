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
    class InvariantsWidget : public QVBoxLayout {
    Q_OBJECT
    private:
        std::unordered_map<std::string, std::unique_ptr<stp::invariant::BaseInvariant>> invariants;
        QTextEdit* textEdit;

        virtual ~InvariantsWidget() {};

    public:
        explicit InvariantsWidget(QWidget* parent = nullptr);
        void updateInvariants();
    };

}

#endif //RTT_INVARIANTSWIDGET_HPP
