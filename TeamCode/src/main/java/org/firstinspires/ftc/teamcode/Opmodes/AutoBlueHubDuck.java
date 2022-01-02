package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Blue Hub Duck", group = "Competition")
@Disabled

public class AutoBlueHubDuck extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    private State state = State.RUN1;

    public AutoBlueHubDuck() {

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
                    sleep(7000);

                    // strafe towards the alliance hub
                    drive.driveTime(0.5, 90, 1);

                    // drive towards the alliance hub
                    drive.driveTime(0.5, 180, .9);

                    // place the scoring element in the hub
                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONHIGH - 80);
                    robot.motorArm.setPower(0.4);

                    sleep(2000);

                    // return to the place
                    robot.motorArm.setTargetPosition(-10);
                    robot.motorArm.setPower(0.4);

                    sleep(1500);

                    // drive towards the outside wall
                    drive.driveTime(0.5, 0, .9);

                    // rotate towards the duck
                    drive.driveTurn(-90, 0.3);

                    // Strafe into the wall
                    drive.driveTime(0.5, 90, 1.1);

                    // Strafe away from the wall
                    drive.driveTime(0.5, -90, 0.8);

                    // drive towards the turntable
                    drive.driveTime(0.5, 0, 2.1);

                    // Strafe towards the turntable
                    drive.driveTime(0.3, 90, .25);

                    // turn duck motor on
                    robot.motorDuck.setPower(robot.duckSpeed);
                    sleep(robot.autoSleepTime);
                    robot.motorDuck.setPower(0);

                    // park in storage
                    drive.driveTime(0.5, -90, 1.3);

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

        // End the program
        requestOpModeStop();

    }// end of runOpMode constructor

    enum State {
        TEST, PLACE_SE, RUN1, PARK, HALT;
    }   // end of enum State

}   // end of class AutoBlueHubDuck