package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Blue Hub", group = "Competition")

public class AutoBlueHub extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    private State state = State.RUN1;

    public AutoBlueHub() {

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

        if (opModeIsActive()) {
            switch (state) {
                case TEST:

                    break;

                case PLACE_SE:

                    break;

                case RUN1:
                    // strafe away from the wall
                    drive.driveTime(.5, 180, 1);

                    sleep(500);

                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONHIGH);
                    robot.motorArm.setPower(0.4);

                    sleep(3000);

                    robot.motorArm.setTargetPosition(0);
                    robot.motorArm.setPower(0.4);

                    sleep(3000);

                    drive.driveTime(.5, 0, .5);

                    drive.driveTurn(-90, 3);

                    drive.driveTime(.5, 0, 3);

                    // drive towards the turntable
//                    drive.driveTime(0.5, 0, 1.5);

//                    drive.driveTime(.1, 90, .25);

                    // park in storage
//                    drive.driveTime(.5, -90, 1.15);

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
        }   // end of if opModeIsActive()

    }// end of runOpMode constructor

    enum State {
        TEST, PLACE_SE, RUN1, PARK, HALT;
    }   // end of enum State

}   // end of class AutoBlueStorage