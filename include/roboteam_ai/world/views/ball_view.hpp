//
// Created by john on 1/13/20.
//

#ifndef RTT_BALL_VIEW_HPP
#define RTT_BALL_VIEW_HPP


#include <Vector2.h>
#include "../ball.hpp"

namespace rtt::world {

    namespace view {

        /**
         * Ball view class, provides the Ball interface with uility functions that don't belong on a POD type
         */
        class BallView {
            const ball::Ball* const _ptr;
        public:
            /**
             * Explicitly delete move constructors, copying this POD type is trivial
             */
            BallView(BallView&&) = delete;
            BallView& operator=(BallView&&) = delete;

            /**
             * Copy constructor, internal pointer is copied over
             * @param old Old BallView to copy from
             */
            BallView(BallView const& old) noexcept;

            /**
             * Copy assignment operatorm, does nothing important.
             * @param old Old ballview to copy
             * @return Returns a reference to `this`
             */
            BallView& operator=(BallView const& old) noexcept;

            /**
             * Explicitly defaulted destructor, no special destruction nessecary as
             * this struct does not own the robot
             */
            ~BallView() = default;


            /**
             * Constructs a BallView
             * _ptr is asserted
             * @param _ptr Pointer that this BallView should provide a view of
             */
            explicit BallView(ball::Ball const* const _ptr) noexcept;

            /**
             * Dereference operator that allows std::optional style dereferencing
             * Undefined behavior will occur if the contained pointer is nullptr
             * @return Returns a reference to the ball you're viewing
             */
            const ball::Ball& operator*() const noexcept;

            /**
             * Gets the internally viewed pointer
             * @return Returns _ptr
             */
            [[nodiscard]] const ball::Ball* get() const noexcept;

            /**
             * Member dereference operator that allows std::optional style member access
             * Undefined behavior will occur if _ptr is nullptr
             * @return Returns get()
             */
            const ball::Ball* operator->() const noexcept;

            /**
             * Get the expected position where the ball will end (lay still) after following his path.
             * @return A vector which represents this position.
             */
            [[nodiscard]] Vector2 getExpectedBallEndPosition() const noexcept;

        };
    }
}


#endif //RTT_BALL_VIEW_HPP
