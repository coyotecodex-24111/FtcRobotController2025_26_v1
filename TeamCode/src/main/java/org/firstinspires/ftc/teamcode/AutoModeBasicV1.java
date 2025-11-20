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
            moveRobot(4,0,0,0.5,2000);
        }
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