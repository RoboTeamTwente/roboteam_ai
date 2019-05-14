//
// Created by mrlukasbos on 7-5-19.
//

#ifndef ROBOTEAM_AI_PIDSWIDGET_H
#define ROBOTEAM_AI_PIDSWIDGET_H

#include "QLayout"
#include "widget.h"

namespace rtt {
namespace ai {
namespace interface {

class PidsWidget : public QWidget {
Q_OBJECT
public:
    explicit PidsWidget(QWidget * parent = nullptr);
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_PIDSWIDGET_H
