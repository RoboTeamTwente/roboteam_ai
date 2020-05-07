//
// Created by john on 4/30/20.
//

#ifndef RTT_PLAYSWIDGET_HPP
#define RTT_PLAYSWIDGET_HPP

#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QTextEdit>

namespace rtt::ai::interface {

    class PlaysWidget : public QTextEdit {
    Q_OBJECT
    private:
        std::stringstream data;
        std::mutex dataMtx;
    public:
        explicit PlaysWidget(QWidget* parent = nullptr);
        void updatePlays();

        ~PlaysWidget() override = default;
    };
}

#endif //RTT_PLAYSWIDGET_HPP
