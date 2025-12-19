package org.firstinspires.ftc.teamcode;

// Declare imports
//import static org.firstinspires.ftc.teamcode.Direction.LEFT;
//import static org.firstinspires.ftc.teamcode.Direction.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Linear Opmode is for sequential code like in Autonomous operation
// (doing things in a linear fashion, one after the other.

@Autonomous(name="AutoModeBasicV1", group="Auto Basic")

public class AutoModeBasicV1 extends LinearOpMode {
    private Robot robot;

    public final double VERSION = 1;
    final ElapsedTime runtime = new ElapsedTime();

    double spinSpeed = 0.2;
    //for every 16 inches we tell TINA to go, TINA goes 15
    //change this factor to 1 for HAMMY
    double TINA_FACTOR_FORWARD = 16/15;

    final double autoLaunchPower = 0.68;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        robot.driveUsingEncoder();
;

        waitForStart();

        if (opModeIsActive()) {
            //Ball will be going backwards from the goal
            robot.setFlywheelPower(autoLaunchPower);
            sleep(5000);
            moveRobot(-15,0,0,0.5,4000);
            //Robot will launch balls x3
            robot.launchBall(robot.FIRST_LAUNCH_DURATION);
            sleep(4000);
            robot.launchBall(robot.DEFAULT_FEED_DURATION);
            sleep(4000);
            robot.launchBall(robot.DEFAULT_FEED_DURATION);
            sleep(4000);
            //Robot will be moving to the left
            //Note: positive value = strafe to the left and negative value will strafe to the right
            moveRobot(0, 15, 0, 0.5, 4000);

        }
    }


        public void moveRobot(double forward, double strafeLeft, int rotate, double speed, int sleep) {
            //From 10/59 to 100/59, our factor was off by 10 so 10/59 times 10 = 100/59
            final double FORWARD_RATIO = (100 / 59.0);
            //From 100/50.875 to 120/50.875, our factor is off by 1.2 so 100/50.875 times 1.2 = 120/50.875
            final double SIDE_RATIO = (100 / 50.875) * (1.1);
            final double COUNTS_PER_INCH = (312) / (3.78 * 3.1415);
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
}