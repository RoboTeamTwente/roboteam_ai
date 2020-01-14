//
// Created by john on 1/13/20.
//

#ifndef RTT_BALL_VIEW_HPP
#define RTT_BALL_VIEW_HPP


#include <Vector2.h>

namespace rtt::world {

    namespace ball {
        class Ball;
    }

    namespace view {

        /**
         * Ball view class, provides the Ball interface with uility functions that don't belong on a POD type
         */
        class BallView {
            const ball::Ball* const _ptr;
        public:
            /**
             * Constructs a BallView
             * _ptr is asserted
             * @param _ptr Pointer that this BallView should provide a view of
             */
            explicit BallView(ball::Ball const* const _ptr);

            /**
             * Dereference operator that allows std::optional style dereferencing
             * Undefined behavior will occur if the contained pointer is nullptr
             * @return Returns a reference to the ball you're viewing
             */
            const ball::Ball& operator*();

            /**
             * Gets the internally viewed pointer
             * @return Returns _ptr
             */
            [[nodiscard]] const ball::Ball* get();

            /**
             * Member dereference operator that allows std::optional style member access
             * Undefined behavior will occur if _ptr is nullptr
             * @return Returns get()
             */
            const ball::Ball* operator->();

            /**
             * Get the expected position where the ball will end (lay still) after following his path.
             * @return A vector which represents this position.
             */
            [[nodiscard]] Vector2 getExpectedBallEndPosition() const;

        };
    }
}


#endif //RTT_BALL_VIEW_HPP
