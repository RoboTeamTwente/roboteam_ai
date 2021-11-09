//
// Created by mrlukasbos on 7-5-19.
//

#ifndef ROBOTEAM_AI_PIDSWIDGET_H
#define ROBOTEAM_AI_PIDSWIDGET_H

#include <QLayout>

#include "widget.h"

namespace rtt::ai::interface {

class PidsWidget : public QWidget {
    Q_OBJECT
   public:
    explicit PidsWidget(QWidget *parent = nullptr);
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_PIDSWIDGET_H
