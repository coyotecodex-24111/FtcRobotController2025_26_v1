package org.firstinspires.ftc.teamcode;

// Declare imports
//import static org.firstinspires.ftc.teamcode.Direction.LEFT;
//import static org.firstinspires.ftc.teamcode.Direction.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

// Linear Opmode is for sequential code like in Autonomous operation
// (doing things in a linear fashion, one after the other.

@Autonomous(name="AutoRedWall", group="Auto Basic")

public class AutoRedWall extends LinearOpMode {
    private Robot robot;
    private AutoMovementShared autoMove;
    public final double VERSION = 1;
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        autoMove = new AutoMovementShared(robot, telemetry);
        robot.init();
        robot.driveUsingEncoder();

        waitForStart();

        if (opModeIsActive()) {
            autoMove.autoForGoal(true, true,true);
        }
    }
}