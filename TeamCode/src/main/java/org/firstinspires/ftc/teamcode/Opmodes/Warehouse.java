package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;

@Autonomous(name = "Warehouse", group = "Competition")
//@Disabled

public class Warehouse extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();

    public void runOpMode(){
        telemetry.addData("Robot State = ", "READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();

        if(opModeIsActive()){
            robot.motorLF.setPower(1);
            robot.motorRF.setPower(1);
            robot.motorLR.setPower(1);
            robot.motorRR.setPower(1);
            sleep(1000);
            robot.motorLF.setPower(0);
            robot.motorRF.setPower(0);
            robot.motorLR.setPower(0);
            robot.motorRR.setPower(0);
        }   // end of if opModeIsActive()

    }// end of runOpMode constructor
}

