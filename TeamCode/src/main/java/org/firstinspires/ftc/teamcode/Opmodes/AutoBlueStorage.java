package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMechanum;

import java.util.List;

@Autonomous(name = "Test Blue Storage", group = "Programming Class")
//@Disabled

public class AutoBlueStorage extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    private State state = State.PARK;

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
        DriveMechanum drive = new DriveMechanum(robot, opMode);

        /*
         * Calibrate / initialize the game sensor
         */

        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            switch (state) {
                case TEST:

                    break;

                case RING_DETECT:
                    // straffe away from the wall
                    drive.driveTime(.5, -90, 0.5);

                    // drive towards the turntable
                    drive.driveTime(0.5, 0, 1.5);

                    

                    state = State.HALT;

//                    robot.servoWobbleGrab.setPosition(robot.SERVO_WOBBLE_GRAB_OPEN);
                    sleep(500);

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
        TEST, RING_DETECT, PATH_DECISION, WOBBLE1A, WOBBLE1B, WOBBLE1C, RESET_START, WOBBLE2A, WOBBLE2B, WOBBLE2C, PARK, HALT;
    }   // end of enum State

}