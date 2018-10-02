#include <utility>

#include "roboteam_world/world/filtered_world.h"
#include <vector>
#include "roboteam_utils/Vector2.h"

namespace rtt {

    /** NOTES:
     *
     *  Baris:
     *      I removed a lot of old hacks and unused functions that were commented out. If things break
     *      It might be useful to get back on git and find the hacks old teams used.
     *
     */

    FilteredWorld::FilteredWorld(Predictor predictor) : fresh{false} {
        reset();
        this->predictor = std::move(predictor);
    }

    /// Reset the world, clear all the buffers and states
    void FilteredWorld::reset() {

        // Clear the input buffers
        robots_blue_buffer.clear();
        robots_yellow_buffer.clear();

        // These are used for the final world state
        robots_blue_world.clear();
        robots_yellow_world.clear();
        ball_world = rtt::Ball();

        // Initialize the input buffers.
        robots_blue_buffer = RobotMultiCamBuffer();
        robots_yellow_buffer = RobotMultiCamBuffer();

        // The cameras
        world_cams = std::map<int, bool>();

    }

    /// Create a message that has the world in it
    roboteam_msgs::World FilteredWorld::as_message() const {

        roboteam_msgs::World returnMsg;

        for (auto& robot : robots_blue_world) {
            returnMsg.them.push_back(robot.second.as_message());
        }

        for (auto& robot : robots_yellow_world) {
            returnMsg.us.push_back(robot.second.as_message());
        }

        returnMsg.ball = ball_world.as_message();
        return returnMsg;
    }

    /// To be called when a detection_frame message is received.
    void FilteredWorld::detection_callback(const roboteam_msgs::DetectionFrame msg) {

        buffer_detection_frame(msg);

        if (is_calculation_needed()) {
            // Reset the camera update flags.
            for (auto& cam : world_cams) {
                cam.second = false;
            }
            // double time_grsim = msg.t_capture;
            //double time_now = ros::Time::now().toSec();
            double time_now=msg.t_capture;
            merge_frames(time_now);
            timeLastUpdated=time_now;
            fresh = true;
        }
    }
    /// Consume a message if it is fresh
    boost::optional<roboteam_msgs::World> FilteredWorld::consumeMsg() {
        if (isFresh()) {
            setFresh(false);
            return as_message();
        }
        return boost::none;
    }

    /// Adds a received detection frame to the buffers.
    /// might break if there is a half field play not sure => rework only playing with cam 0 and 3 or something
    /// (Baris September)
    void FilteredWorld::buffer_detection_frame(const roboteam_msgs::DetectionFrame msg) {

        auto cam_id = msg.camera_id;

        // Set this cameras updated flag.
        // If this camera hasn't sent frames before, it is now added to the list of cameras.
        world_cams[cam_id] = true;
        timeFrameCaptured[cam_id]=msg.t_capture;

        // ==== Robots ====
        // Add the robot data
        for (const roboteam_msgs::DetectionRobot robot : msg.them) {
            int bot_id = robot.robot_id;

            robots_blue_buffer[bot_id][cam_id] = roboteam_msgs::DetectionRobot(robot);
        }
        for (const roboteam_msgs::DetectionRobot robot : msg.us) {
            int bot_id = robot.robot_id;

            robots_yellow_buffer[bot_id][cam_id] = roboteam_msgs::DetectionRobot(robot);
        }

        // ==== Ball ====
        // Find the ball with the smallest distance from the previous position +velocity * time between the two frames

        if (! msg.balls.empty()) {
            Position previousBallPos = ball_world.get_position();
            Position previousBallVel = ball_world.get_velocity();
            roboteam_msgs::DetectionBall closestBall = msg.balls[0];
            Position predictedPosition= previousBallPos+previousBallVel*(msg.t_capture-timeLastUpdated);
            double closestDist2 = Vector2(closestBall.pos).dist2(Vector2(predictedPosition.x,predictedPosition.y));

            for (auto const & ball : msg.balls) {

                double dist2 = Vector2(ball.pos).dist2(Vector2(predictedPosition.x,predictedPosition.y));
                if (dist2 < closestDist2) {
                    closestBall = ball;
                    closestDist2 = dist2;
                }
            }

            ball_buffer[cam_id] = closestBall; // msg.balls[0];
        }
        else {
            ball_buffer.erase(cam_id);
        }
    }


    /// Returns true when every camera's frame has updated.
    /// When there are no cameras, this function will always return false.
    bool FilteredWorld::is_calculation_needed() const {
        if (world_cams.empty()) {
            // No cameras? No use doing a frame merge calculation.
            return false;
        }
        for (auto& cam : world_cams) {

            if (!cam.second) {
                // there is only one camera
                return false;
            }
        }
        return true;
    }

    /**
     * Merges the frames multiple cameras gives into a world state.
     *
     * Merges the robots using FilteredWorld::merge_robots
     * Picks the best option for the ball.
     * Clears the buffers for the frames afterwards.
     *
     */
    void FilteredWorld::merge_frames(double timestamp) {
        std::string s;
        get_PARAM_OUR_COLOR(s);
        bool isBlueOurTeam = s == "blue";
        merge_robots(robots_blue_buffer, robots_blue_world, old_blue, timestamp, isBlueOurTeam);
        merge_robots(robots_yellow_buffer, robots_yellow_world, old_yellow, timestamp, !isBlueOurTeam);

        // Take the ball from the camera where extrapolation with respect to the world is closest to
        // the extrapolation of the last world state's velocity and position
        if (!ball_buffer.empty()) {
            ball_world.set_visible(true);

            // extrapolation of the last world's state velocity and position
            Position previousBallPos = ball_world.get_position();
            Position previousBallVel = ball_world.get_velocity();
            Position predictedPosition = previousBallPos+ previousBallVel*(timestamp-timeLastUpdated);

            // First extrapolation
            roboteam_msgs::DetectionBall closestBall=ball_buffer.begin()->second;
            int best_camera=ball_buffer.begin()->first;
            // Initial extrapolation. Does the same as below
            Position Extrapolation=previousBallPos+(Position(closestBall.pos)-previousBallPos)*(1/(timeFrameCaptured[best_camera]-timeLastUpdated))*(timestamp-timeLastUpdated);
            double closestDist2= Vector2(Extrapolation.x,Extrapolation.y).dist2(Vector2(predictedPosition.x,predictedPosition.y));

            for (auto const & detectedBall : ball_buffer){
                // Extrapolate from detectionframe's capture time to current time.
                Position detectedBallPos=Position(detectedBall.second.pos);
                Extrapolation= previousBallPos + (detectedBallPos-previousBallPos)*(1/(timeFrameCaptured[detectedBall.first]-timeLastUpdated))*(timestamp-timeLastUpdated);
                double dist2 = Vector2(Extrapolation.x,Extrapolation.y).dist2(Vector2(predictedPosition.x,predictedPosition.y));
                // Pick the Extrapolation which works best
                if (dist2 < closestDist2) {
                    //best_camera=detectedBall.first;
                    closestBall=detectedBall.second;
                    closestBall.pos=Vector2(Extrapolation.x,Extrapolation.y);
                    closestDist2 = dist2;
                }
            }
            // Move the ball to the one that has best extrapolation
            //ball_world.move_to(ball_buffer[best_camera].pos.x,ball_buffer[best_camera].pos.y,ball_buffer[best_camera].z);
            ball_world.move_to(closestBall.pos.x,closestBall.pos.y,closestBall.z);
        }
        else {
            ball_world.set_visible(false);
        }

        // Update the predictor and get a speed vector for the ball
        // Pushes the Reference of ball to predictor. This is why we can update velocity later.
        predictor.update(ball_world, timestamp);
        boost::optional<Position> ballVel = predictor.computeBallVelocity();

        if (ballVel) {
            Position vel = *ballVel;
            ball_world.set_velocity(static_cast<float>(vel.x), static_cast<float>(vel.y));
        }

        // Clear the buffers.
        robots_blue_buffer.clear();
        robots_yellow_buffer.clear();
    }

    /// Merges the robots from different frames
    void FilteredWorld::merge_robots(RobotMultiCamBuffer& robots_buffer, std::map<int,
            rtt::Robot>& robots_output, std::map<int, rtt::Robot>& old_buffer, double timestamp, bool our_team) {
        //For every robot buffer
        for (auto& robot_buffer : robots_buffer) {
            auto bot_id = (uint) robot_buffer.first;

            Robot robot;
            robot.set_id(bot_id);


            // Places the robot to the extrapolation of the last frame we saw it in. (assumes good camera calibration)
            //TODO: A position is 'bad' if the internal velocity of the robot is exceedingly large. This is hard to implement for different scenario's
            //TODO: Rotation is now fixed to last. Could still be extrapolated.
            //TODO: Catch case if time measurement is off.
            Vector2 previousPosition;
            //If the robot was on last world, get the previous position. If not, initialize it to position(0,0)
            if (robots_output.find(bot_id)!=robots_output.end()) {
                previousPosition = Vector2(robots_output[bot_id].get_position().x,
                                                   robots_output[bot_id].get_position().y);
            }
            else {
                previousPosition=Vector2(0,0);
            }

            float w =0;
            Vector2 Extrapolation;
            //float previousw=robot.get_position().rot;
            //Vector2 previousVelocity = Vector2(robot.get_velocity().x,robot.get_velocity().y);
            //Vector2 predictedPosition= previousPosition+previousVelocity*(timestamp-timeLastUpdated);

            double last_frame=timeLastUpdated;
            for (auto& buf : robot_buffer.second){
                if(timeFrameCaptured[buf.first]>last_frame){
                    last_frame=timeFrameCaptured[buf.first];
                    Vector2 bufPosition = buf.second.pos;
                    Vector2 bufVelocity = (bufPosition-previousPosition)/(timeFrameCaptured[buf.first]-timeLastUpdated);
                    Extrapolation= previousPosition+bufVelocity*(timestamp-timeLastUpdated);
                    w=buf.second.orientation;
                }
            }
            // Assign the robot position and rotation to the extrapolation calculated.
            robot.move_to((float)Extrapolation.x, (float)Extrapolation.y);
            robot.rotate_to(w);

            // Send an update and discard old data for buffers used for calculations
            predictor.update(robot, our_team, timestamp);
            // compute velocity of the robot and update if received
            boost::optional<Position> robotVel = predictor.computeRobotVelocity(bot_id, our_team);
            if (robotVel) {
                Position vel = *robotVel;

                robot.set_vel(static_cast<float>  (vel.x),
                              static_cast<float>  (vel.y),
                              static_cast<float>  (vel.rot) );
            }

            // Update the last detection time used in calculations.
            robot.update_last_detection_time(timestamp);
            // add the updated robot to robot_output and old_buffer.
            robots_output[bot_id] = robot;
            old_buffer[bot_id] = robot;
        }

        // Remove old robots.
        auto botIter = robots_output.begin();

        while (botIter != robots_output.end()) {
            // Remove robots that are not detected for 0.5 seconds.
            if (botIter->second.is_detection_old(timestamp, 20)) {
                ROS_INFO("Removing bot: %i. Too old.", botIter->second.get_id());
                botIter = robots_output.erase(botIter);
            } else if (botIter->second.is_detection_from_future(timestamp)) {
                ROS_INFO("Removing bot: %i. It's from the future?", botIter->second.get_id());
                botIter = robots_output.erase(botIter);
            } else {
                ++botIter;
            }
        }
    }
    bool FilteredWorld::isFresh() {
        return fresh;
    }

    void FilteredWorld::setFresh(bool newFresh) {
        fresh = newFresh;
    }
}
