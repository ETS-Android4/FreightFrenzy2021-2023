package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Red Hub Warehouse", group = "Competition")
@Disabled

public class AutoRedHub extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    private State state = State.RUN1;

    public AutoRedHub() {

    }   // end of TestAuto constructor

    public void runOpMode() {
        telemetry.addData("Robot State = ", "READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        /*
         * Initialize the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode);

        /*
         * Calibrate / initialize the game sensor
         */

        robot.servoIntake.setPosition(robot.INTAKECUPUP);
        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            switch (state) {
                case TEST:

                    break;

                case PLACE_SE:

                    break;

                case RUN1:
                    // strafe away from the wall
//                    sleep(5000);

                    // strafe into scoring position
                    drive.driveTime(0.5, 90, 1);

                    // drive forward to score in the alliance hub
                    drive.driveTime(0.5, 180, 0.9);

//                    sleep(500);

                    // put arm up to score the shipping element
                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONHIGH - 80);
                    robot.motorArm.setPower(0.4);

                    sleep(2000);

                    // return arm to ready position
                    robot.motorArm.setTargetPosition(-10);
                    robot.motorArm.setPower(0.4);

                    sleep(1000);

                    // drive towards the wall
                    drive.driveTime(0.5, 0, .7);

                    // rotate towards the warehouse
                    drive.driveTurn(-90, 0.3);

                    // strafe into the wall
                    drive.driveTime(0.5, 90, 1);

                    // drive into the warehouse
                    drive.driveTime(0.5, 0, 2.2);

                    // strafe out of the way for another bot to park in the warehouse
                    drive.driveTime(0.5, -90, 1.2);

                    state = State.HALT;

                    break;

                case PARK:

                    state = State.HALT;

                    break;

                case HALT:

                    // Stop all motors
                    drive.motorsHalt();

                    // End the program
                    requestOpModeStop();

                    break;
            }   // end of the switch state
        }   // end of while opModeIsActive()
        // End the program
        requestOpModeStop();

    }// end of runOpMode constructor

    enum State {
        TEST, PLACE_SE, RUN1, PARK, HALT;
    }   // end of enum State

}   // end of class AutoBlueStorage