package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Red Hub Duck", group = "Competition")

public class AutoRedHubDuck extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    private State state = State.RUN1;

    public AutoRedHubDuck() {

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
                    // strafe into scoring position
                    drive.driveTime(0.5, -90, 1);

                    // drive to hub to score
                    drive.driveTime(0.5, 180, 0.9);

                    // score in the hub
                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONHIGH - 80);
                    robot.motorArm.setPower(0.4);

                    sleep(2000);

                    // reset the arm to normal position
                    robot.motorArm.setTargetPosition(0);
                    robot.motorArm.setPower(0.4);

//                    sleep(3000);

                    // drive towards the wall
                    drive.driveTime(0.5, 0, 0.7);

                    // rotate towards the carousel
                    drive.driveTurn(-90, 0.3);

                    // strafe into the wall
                    drive.driveTime(0.5, 90, 1);

                    // strafe away from the wall
                    drive.driveTime(0.5, -90, .9);

//                    sleep(500);

                    // drive towards the turntable
                    drive.driveTime(0.5, 180, 2.4);

                    // strafe into the carousel
                    drive.driveTime(0.2, 90, 0.3);

                    // turn duck motor on
                    robot.motorDuck.setPower(-robot.duckSpeed);

                    // wait for the duck to drop off
                    sleep(robot.autoSleepTime);
                    drive.motorsHalt();
                    robot.motorDuck.setPower(0);

                    // park in storage
                    drive.driveTime(0.5, -90, 1.25);

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