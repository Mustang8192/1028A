

#include "1028A/chassis/chassis.hpp"
#include "1028A/util.hpp"
#include "pros/misc.hpp"
#include <cmath>
#include <string>
#include <vector>

std::vector<std::string> readElement(const std::string &input,
                                     std::string delimiter) {
  std::string token;
  std::string s = input;
  std::vector<std::string> output;
  size_t pos = 0;

  // main loop
  while ((pos = s.find(delimiter)) !=
         std::string::npos) { // while there are still delimiters in the string
    token = s.substr(0, pos); // processed substring
    output.push_back(token);
    s.erase(0, pos + delimiter.length()); // remove the read substring
  }

  output.push_back(s); // add the last element to the returned string

  return output;
}

std::vector<_1028A::Pose> getData(const asset &path) {
  std::vector<_1028A::Pose> robotPath;
  std::string line;
  std::vector<std::string> pointInput;
  _1028A::Pose pathPoint(0, 0, 0);

  // format data from the asset
  std::string data(reinterpret_cast<char *>(path.buf), path.size);
  std::vector<std::string> dataLines = readElement(data, "\n");

  // read the points until 'endData' is read
  for (std::string line : dataLines) {
    if (line == "endData" || line == "endData\r")
      break;
    pointInput = readElement(line, ", ");          // parse line
    pathPoint.x = std::stof(pointInput.at(0));     // x position
    pathPoint.y = std::stof(pointInput.at(1));     // y position
    pathPoint.theta = std::stof(pointInput.at(2)); // velocity
    robotPath.push_back(pathPoint);                // save data
  }

  return robotPath;
}

int findClosest(_1028A::Pose pose, std::vector<_1028A::Pose> path) {
  int closestPoint;
  float closestDist = 1000000;
  float dist;

  // loop through all path points
  for (int i = 0; i < path.size(); i++) {
    dist = pose.distance(path.at(i));
    if (dist < closestDist) { // new closest point
      closestDist = dist;
      closestPoint = i;
    }
  }

  return closestPoint;
}

float circleIntersect(_1028A::Pose p1, _1028A::Pose p2, _1028A::Pose pose,
                      float lookaheadDist) {
  // calculations
  // uses the quadratic formula to calculate intersection points
  _1028A::Pose d = p2 - p1;
  _1028A::Pose f = p1 - pose;
  float a = d * d;
  float b = 2 * (f * d);
  float c = (f * f) - lookaheadDist * lookaheadDist;
  float discriminant = b * b - 4 * a * c;

  // if a possible intersection was found
  if (discriminant >= 0) {
    discriminant = sqrt(discriminant);
    float t1 = (-b - discriminant) / (2 * a);
    float t2 = (-b + discriminant) / (2 * a);

    // prioritize further down the path
    if (t2 >= 0 && t2 <= 1)
      return t2;
    else if (t1 >= 0 && t1 <= 1)
      return t1;
  }

  // no intersection found
  return -1;
}

_1028A::Pose lookaheadPoint(_1028A::Pose lastLookahead, _1028A::Pose pose,
                            std::vector<_1028A::Pose> path,
                            float lookaheadDist) {
  // find the furthest lookahead point on the path

  // optimizations applied:
  // - made the starting index the one after lastLookahead's index,
  // as anything before would be discarded
  // - searched the path in reverse, as the first hit would be
  // the garunteed farthest lookahead point
  for (int i = path.size() - 1; i > lastLookahead.theta; i--) {
    // since we are searching in reverse, instead of getting
    // the current pose and the next one, we should get the
    // current pose and the *last* one
    _1028A::Pose lastPathPose = path.at(i - 1);
    _1028A::Pose currentPathPose = path.at(i);

    float t =
        circleIntersect(lastPathPose, currentPathPose, pose, lookaheadDist);

    if (t != -1) {
      _1028A::Pose lookahead = lastPathPose.lerp(currentPathPose, t);
      lookahead.theta = i;
      return lookahead;
    }
  }

  // robot deviated from path, use last lookahead point
  return lastLookahead;
}

float findLookaheadCurvature(_1028A::Pose pose, float heading,
                             _1028A::Pose lookahead) {
  // calculate whether the robot is on the left or right side of the circle
  float side = _1028A::sgn(std::sin(heading) * (lookahead.x - pose.x) -
                           std::cos(heading) * (lookahead.y - pose.y));
  // calculate center point and radius
  float a = -std::tan(heading);
  float c = std::tan(heading) * pose.x - pose.y;
  float x =
      std::fabs(a * lookahead.x + lookahead.y + c) / std::sqrt((a * a) + 1);
  float d = std::hypot(lookahead.x - pose.x, lookahead.y - pose.y);

  // return curvature
  return side * ((2 * x) / (d * d));
}

void _1028A::Chassis::follow(const asset &path, int timeout, float lookahead,
                             bool async, bool forwards, float maxSpeed,
                             bool log) {
  // try to take the mutex
  // if its unsuccessful after 10ms, return
  if (!mutex.take(10))
    return;
  // if the function is async, run it in a new task
  if (async) {
    pros::Task task([&]() {
      follow(path, timeout, lookahead, false, forwards, maxSpeed, log);
    });
    mutex.give();
    pros::delay(10); // delay to give the task time to start
    return;
  }

  std::vector<_1028A::Pose> pathPoints =
      getData(path); // get list of path points
  Pose pose = this->getPose(true);
  Pose lastPose = pose;
  Pose lookaheadPose(0, 0, 0);
  Pose lastLookahead = pathPoints.at(0);
  lastLookahead.theta = 0;
  float curvature;
  float targetVel;
  float prevLeftVel = 0;
  float prevRightVel = 0;
  int closestPoint;
  float leftInput = 0;
  float rightInput = 0;
  int compState = pros::competition::get_status();
  distTravelled = 0;

  // loop until the robot is within the end tolerance
  for (int i = 0;
       i < timeout / 10 && pros::competition::get_status() == compState; i++) {
    // get the current position of the robot
    pose = this->getPose(true);
    if (!forwards)
      pose.theta -= M_PI;

    // update completion vars
    distTravelled += pose.distance(lastPose);
    lastPose = pose;

    // find the closest point on the path to the robot
    closestPoint = findClosest(pose, pathPoints);
    // if the robot is at the end of the path, then stop
    if (pathPoints.at(closestPoint).theta == 0)
      break;

    // find the lookahead point
    lookaheadPose = lookaheadPoint(lastLookahead, pose, pathPoints, lookahead);
    lastLookahead = lookaheadPose; // update last lookahead position

    // get the curvature of the arc between the robot and the lookahead point
    float curvatureHeading = M_PI / 2 - pose.theta;
    curvature = findLookaheadCurvature(pose, curvatureHeading, lookaheadPose);

    // get the target velocity of the robot
    targetVel = pathPoints.at(closestPoint).theta;

    // calculate target left and right velocities
    float targetLeftVel =
        targetVel * (2 + curvature * drivetrain.trackWidth) / 2;
    float targetRightVel =
        targetVel * (2 - curvature * drivetrain.trackWidth) / 2;

    // ratio the speeds to respect the max speed
    float ratio =
        std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) /
        maxSpeed;
    if (ratio > 1) {
      targetLeftVel /= ratio;
      targetRightVel /= ratio;
    }

    // update previous velocities
    prevLeftVel = targetLeftVel;
    prevRightVel = targetRightVel;

    // move the drivetrain
    if (forwards) {
      drivetrain.leftMotors->move(targetLeftVel);
      drivetrain.rightMotors->move(targetRightVel);
    } else {
      drivetrain.leftMotors->move(-targetRightVel);
      drivetrain.rightMotors->move(-targetLeftVel);
    }

    pros::delay(10);
  }

  // stop the robot
  drivetrain.leftMotors->move(0);
  drivetrain.rightMotors->move(0);
  // set distTravelled to -1 to indicate that the function has finished
  distTravelled = -1;
  // give the mutex back
  mutex.give();
}
