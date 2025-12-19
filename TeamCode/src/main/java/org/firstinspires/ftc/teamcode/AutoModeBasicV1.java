package org.firstinspires.ftc.teamcode;

// Declare imports
//import static org.firstinspires.ftc.teamcode.Direction.LEFT;
//import static org.firstinspires.ftc.teamcode.Direction.RIGHT;

import static org.firstinspires.ftc.teamcode.AutoMovementShared.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Linear Opmode is for sequential code like in Autonomous operation
// (doing things in a linear fashion, one after the other.

@Autonomous(name="AutoModeBasicV1", group="Auto Basic")

public class AutoModeBasicV1 extends LinearOpMode {
    private Robot robot;

    private AutoMovementShared autoMove;

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
        autoMove = new AutoMovementShared(robot, telemetry);
        robot.init();
        robot.driveUsingEncoder();

        waitForStart();

        if (opModeIsActive()) {
            autoMove.autoForGoal();

        }
    }

}