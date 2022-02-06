package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Test Autonomous", group = "Development")
//@Disabled


public class TestAuto extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;

    public TestAuto(){

    }   // end of TestAuto constructor

    public void runOpMode(){
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

    if(opModeIsActive()){
        drive.driveTime(0.20, 0, 10);
        drive.motorsHalt();
    }   // end of if opModeIsActive()

    }// end of runOpMode constructor
}

