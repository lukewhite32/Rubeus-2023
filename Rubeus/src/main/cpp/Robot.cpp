/* By Tyler Clarke
  This is an experiment with c++20 features and new paradigms to make FRC robot code cleaner.
  The idea: structuring an FRC robot like a real C++ program instead of like Java gone even wronger. Craszy.
*/
#include <FRL/bases/AwesomeRobotBase.hpp>
#include <FRL/motor/SparkMotor.hpp>
#include <FRL/swerve/SwerveModule.hpp>
#include <constants.h>


SwerveModule frontLeftSwerve (
  new SparkMotor(FRONT_LEFT_SPEED),
  new SparkMotor(FRONT_LEFT_DIREC),
  FRONT_LEFT_CANCODER,
  FRONT_LEFT_OFFSET
);

SwerveModule frontRightSwerve (
  new SparkMotor(FRONT_RIGHT_SPEED),
  new SparkMotor(FRONT_RIGHT_DIREC),
  FRONT_RIGHT_CANCODER,
  FRONT_RIGHT_OFFSET
);

SwerveModule mainSwerve (
  new SparkMotor(BACK_LEFT_SPEED),
  new SparkMotor(BACK_LEFT_DIREC),
  BACK_LEFT_CANCODER,
  BACK_LEFT_OFFSET
);

SwerveModule backRightSwerve (
  new SparkMotor(BACK_RIGHT_SPEED),
  new SparkMotor(BACK_RIGHT_DIREC),
  BACK_RIGHT_CANCODER,
  BACK_RIGHT_OFFSET
);


class TeleopMode : public RobotMode {
public:
  void Loop(){
    mainSwerve.SetDirection(0); // 0 the entire drive
  }
};


class AutonomousMode : public RobotMode {

};


class TestMode : public RobotMode {

};


class DisabledMode : public RobotMode {

};


#ifndef RUNNING_FRC_TESTS // I'm afraid to remove this.
int main() {
  mainSwerve.Link(&backRightSwerve); // Weird, right? This can in fact be used here.
  backRightSwerve.Link(&frontRightSwerve);
  frontRightSwerve.Link(&frontLeftSwerve);
  // As it turns out, int main actually still exists and even works here in FRC. I'm tempted to boil it down further and get rid of that stupid StartRobot function (replace it with something custom inside AwesomeRobot).
  return frc::StartRobot<AwesomeRobot<TeleopMode, AutonomousMode, TestMode, DisabledMode>>(); // Look, the standard library does these nested templates more than I do.
}
#endif
