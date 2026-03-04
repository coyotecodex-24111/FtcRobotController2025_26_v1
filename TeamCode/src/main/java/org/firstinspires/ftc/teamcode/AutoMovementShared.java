package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoMovementShared {
    private Robot robot;
    private final Telemetry telemetry;

    double spinSpeed = 0.2;
    //for every 16 inches we tell TINA to go, TINA goes 15
    //change this factor to 1 for HAMMY
    double TINA_FACTOR_FORWARD = 16/15;

    final double autoLaunchLong = 1740;
    final double autoLaunchShort = 1680;
    final double autoLaunchWall = 2010;
    final long feedwheelSleepDuration = 1000;

    final int ballsToLaunch = 5;

    public AutoMovementShared(Robot hammy, Telemetry telemetry) {
        this.robot = hammy;
        this.telemetry = telemetry;
    }
  public void autoForGoal(boolean Red, boolean Wall, boolean Long) {
      double strafeDistance = 20;
      int rotationDegrees = -45;

      if(Red){
          strafeDistance = -strafeDistance;
          rotationDegrees = -rotationDegrees;
      }

      //TOTAL: 5500ms for the flywheel to get up to speed

      if(Wall){
          if(Long) {
              robot.setTargetVelocity(autoLaunchLong);
              moveRobot(74, 0, 0, 0.5, 3000);
              moveRobot(0, 0, rotationDegrees, 0.5, 1000);
              launchNBalls(ballsToLaunch);
              //moveRobot(13,0,0,0.5,1000);
          }
          else {
              robot.setTargetVelocity(autoLaunchWall);
              rotateBegin(false,0.2);
              sleep(300);
              robot.stopAllDriveMotors();
              launchNBalls(ballsToLaunch);
              sleep(1000);
          }
      }
      else {
          //Starting from GOAL
          //Ball will be going backwards from the goal
          if(Long) {
              robot.setTargetVelocity(autoLaunchLong);
              moveRobot(-61, 0, 0, 0.5, 3000);
              sleep(2000);
              launchNBalls(ballsToLaunch);
          }
          else {
              robot.setTargetVelocity(autoLaunchShort);
              moveRobot(-40, 0, 0, 0.5, 3000);
              sleep(2000);
              launchNBalls(ballsToLaunch);
          }
      }


      //Robot will be moving to the left
      //Note: positive value = strafe to the left and negative value will strafe to the right
      moveRobot(0, strafeDistance, 0, 0.5, 4000);
  }
    public void autoNoLaunch(boolean Red, boolean Wall) {
        double strafeDistance = 5;
        int rotationDegrees = -25;

        if(Red){
            strafeDistance = -strafeDistance;
            rotationDegrees = -rotationDegrees;
        }
        if(Wall) {
            //start on the edge of the small launch zone, wheel in the launch zone
            sleep(25000);
            moveRobot(2,0,0,0.5,1000);
            moveRobot(0,strafeDistance,0,0.5,2000);
        }
        else {
            moveRobot(0, 0, -45, 0.5, 4000);
            moveRobot(-40, 0, 0, 0.5, 4000);
        }

        //moveRobot(0,0,rotationDegrees,0.5,2000);
    }

    public void moveRobot(double forward, double strafeLeft, double rotateCW, double speed, int sleep) {
        //From 10/59 to 100/59, our factor was off by 10 so 10/59 times 10 = 100/59
        // Fudge factor is off, the distance isn't as accurate as desired, 72/70.5 * 0.92 = 0.94
        final double FORWARD_RATIO = (100 / 59.0) * (0.92);
        //From 100/50.875 to 120/50.875, our factor is off by 1.2 so 100/50.875 times 1.2 = 120/50.875
        final double SIDE_RATIO = (100 / 50.875) * (1.1);
        final double COUNTS_PER_INCH = (312) / (3.78 * 3.1415);
        final int rotate = (int) (0.44 * rotateCW);
        final double leftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (forward * FORWARD_RATIO - strafeLeft * SIDE_RATIO - rotate) * COUNTS_PER_INCH;

        robot.leftFrontDrive.setTargetPosition((int) leftFrontTarget);
        robot.rightFrontDrive.setTargetPosition((int) (robot.rightFrontDrive.getCurrentPosition() + (forward * FORWARD_RATIO + strafeLeft * SIDE_RATIO + rotate) * COUNTS_PER_INCH));
        robot.leftBackDrive.setTargetPosition((int)   (robot.leftBackDrive.getCurrentPosition()   + (forward * FORWARD_RATIO + strafeLeft * SIDE_RATIO - rotate) * COUNTS_PER_INCH));
        robot.rightBackDrive.setTargetPosition((int)  (robot.rightBackDrive.getCurrentPosition()  + (forward * FORWARD_RATIO - strafeLeft * SIDE_RATIO + rotate) * COUNTS_PER_INCH));

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);
        robot.rightBackDrive.setPower(speed);
        robot.leftBackDrive.setPower(speed);

        telemetry.addData("forward: %4.2f ", forward  );
        telemetry.addData("strafe: %4.2f " , strafeLeft );
        telemetry.addData( "rotate: %4.2f " , rotate );
        telemetry.addData( " speed: %4.2f " , speed );
        telemetry.addData( " sleep: %4.2f" , sleep );
        telemetry.update();

        sleep(sleep);
    }
    public void launchNBalls(int N){
        //Robot will launch balls x3
        robot.launchBall(robot.FIRST_LAUNCH_DURATION);
        for(int i = 2; i <= N; i++) {
            sleep(feedwheelSleepDuration);
            robot.launchBall(robot.DEFAULT_FEED_DURATION);
        }
        robot.setFlywheelPower(0);
    }
    public void rotateBegin(boolean rotateCW, double speed) {
        robot.driveUsingEncoder();

        if(!rotateCW) {
            speed = -speed;
        }
            robot.leftFrontDrive.setPower(speed);
            robot.rightFrontDrive.setPower(-speed);
            robot.leftBackDrive.setPower(speed);
            robot.rightBackDrive.setPower(-speed);
    }
}
