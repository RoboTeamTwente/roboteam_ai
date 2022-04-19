//
// Created by john on 4/30/20.
//

#ifndef RTT_PLAYSWIDGET_HPP
#define RTT_PLAYSWIDGET_HPP

#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>

namespace rtt::ai::interface {

class PlaysWidget : public QTextEdit {
    Q_OBJECT
   public:
    /**
     * Constructor that sets readonly(true)
     * @param parent parent of the widget
     */
    explicit PlaysWidget(QWidget* parent = nullptr);
    ~PlaysWidget() override = default;

   public slots:
    /**
     * Slot that updates plays, gets them from STPManager.
     * The behavior is undefined if this is called using a non-signal-slot style.
     */
    void updatePlays();
};
}  // namespace rtt::ai::interface

#endif  // RTT_PLAYSWIDGET_HPP
