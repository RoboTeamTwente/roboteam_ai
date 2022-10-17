//
// Created by mrlukasbos on 15-5-19.
//

#ifndef ROBOTEAM_AI_RULESETWIDGET_H
#define ROBOTEAM_AI_RULESETWIDGET_H

#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

namespace rtt::ai::interface {

class RuleSetWidget : public QWidget {
    Q_OBJECT

   private:
    QVBoxLayout *vLayout;
    void updateLabels();

   public:
    explicit RuleSetWidget(QWidget *parent = nullptr);

   public slots:
    void updateContents();
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_RULESETWIDGET_H
