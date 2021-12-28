package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Blue Storage", group = "Competition")

public class AutoBlueStorage extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    private State state = State.RUN1;

    public AutoBlueStorage() {

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
                    drive.driveTime(.5, -90, 0.5);

                    sleep(500);

                    // drive towards the turntable
                    drive.driveTime(0.5, 0, 1.5);

                    drive.driveTime(0.2, 90, .25);

                    // turn duck motor on
                    robot.motorDuck.setPower(robot.duckSpeed);
                    sleep(robot.autoSleepTime);
                    robot.motorDuck.setPower(0);

                    // park in storage
                    drive.driveTime(.5, -90, 1.15);

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

}   // end of class AutoBlueStorage