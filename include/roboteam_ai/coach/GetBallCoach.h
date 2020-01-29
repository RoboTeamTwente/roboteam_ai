//
// Created by rolf on 17-4-19.
//

#ifndef ROBOTEAM_AI_GETBALLCOACH_H
#define ROBOTEAM_AI_GETBALLCOACH_H

namespace rtt::ai::coach {

class GetBallCoach {
 private:
  bool gettingBall = false;
  int idGettingBall = -1;
  bool shouldWeGetBall(const Field &field);
  int bestBallGetterID();

 public:
  void update(const Field &field);
  bool weAreGettingBall();
  int getBallGetterID();
};

extern GetBallCoach getBallCoachObj;
extern GetBallCoach *getBallCoach;

}  // namespace rtt::ai::coach

#endif  // ROBOTEAM_AI_GETBALLCOACH_H
