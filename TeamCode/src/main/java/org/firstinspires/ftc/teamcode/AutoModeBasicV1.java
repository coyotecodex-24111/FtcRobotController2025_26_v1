package org.firstinspires.ftc.teamcode;

// Declare imports
import static org.firstinspires.ftc.teamcode.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.Direction.RIGHT;

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

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        robot.driveUsingEncoder();
;

        waitForStart();

        if (opModeIsActive()) {
            spin(LEFT, 10000);
            sleep(1000);
            spin(RIGHT, 5000);
        }
    }

    private void spin(Direction direction, long milliSeconds) {
        int speedPolarityLeft = direction == LEFT ? -1 : 1;
        int speedPolarityRight = direction == LEFT ? 1 : -1;

        double max;

        double leftFrontPower = spinSpeed * speedPolarityLeft;
        double rightFrontPower = spinSpeed * speedPolarityRight;
        double leftBackPower = spinSpeed * speedPolarityLeft;
        double rightBackPower = spinSpeed * speedPolarityRight;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status (Version: " + VERSION + ")", "Run Time: " + runtime);
        telemetry.addData("About to spin: " + direction + " for " + milliSeconds + "ms", "Run Time: " + runtime);
        telemetry.addData("About to spin: Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("About to spin: Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();

        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightBackDrive.setPower(rightBackPower);




            sleep(milliSeconds);
        robot.stopAllDriveMotors();
    }
        public void moveRobot(int forward, int strafe, int rotate, double speed, int sleep) {
            final double FORWARD_RATIO = (100 / 59.0);
            final double SIDE_RATIO = (100 / 50.875);
            final double COUNTS_PER_INCH = (312) / (3.78 * 3.1415);

            robot.leftFrontDrive.setTargetPosition((int) (robot.leftFrontDrive.getCurrentPosition() + (forward * FORWARD_RATIO - strafe * SIDE_RATIO - rotate) * COUNTS_PER_INCH));
            robot.rightFrontDrive.setTargetPosition((int) (robot.rightFrontDrive.getCurrentPosition() + (forward * FORWARD_RATIO + strafe * SIDE_RATIO + rotate) * COUNTS_PER_INCH));
            robot.leftBackDrive.setTargetPosition((int) (robot.leftBackDrive.getCurrentPosition() + (forward * FORWARD_RATIO + strafe * SIDE_RATIO - rotate) * COUNTS_PER_INCH));
            robot.rightBackDrive.setTargetPosition((int) (robot.rightBackDrive.getCurrentPosition() + (forward *FORWARD_RATIO - strafe * SIDE_RATIO + rotate) * COUNTS_PER_INCH));

            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftFrontDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);
            robot.rightBackDrive.setPower(speed);
            robot.leftBackDrive.setPower(speed);

            sleep(sleep);
        }
}