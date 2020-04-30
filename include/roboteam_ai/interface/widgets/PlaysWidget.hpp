//
// Created by john on 4/30/20.
//

#ifndef RTT_PLAYSWIDGET_HPP
#define RTT_PLAYSWIDGET_HPP

#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QTextEdit>

namespace rtt::ai::interface {

    class PlaysWidget : public QVBoxLayout {
    Q_OBJECT
    public:
        explicit PlaysWidget(QWidget* parent = nullptr);
        void updatePlays();

    private:
        QTextEdit* textEdit = new QTextEdit();
    };
}

#endif //RTT_PLAYSWIDGET_HPP
