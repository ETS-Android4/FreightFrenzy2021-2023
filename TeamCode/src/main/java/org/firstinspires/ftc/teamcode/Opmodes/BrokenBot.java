package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;

@TeleOp(name="Broken Bot Testing", group="Dev")
//@Disabled

public class BrokenBot extends LinearOpMode{


    /* Declare OpMode members. */
        HWProfile robot           = new HWProfile();

        @Override
        public void runOpMode() {


            /* Initialize the hardware variables.
             * The init() method of the hardware class does all the work here
             */
            robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Say", "Hello Driver");    //
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                if (gamepad2.dpad_down) {
                    robot.motorLF.setPower(1);
                    telemetry.addData("Motor = ", "MotorLF");
                } else robot.motorLF.setPower(0);

                if (gamepad2.dpad_up) {
                    robot.motorLR.setPower(1);
                    telemetry.addData("Motor = ", "MotorLR");
                }else robot.motorLR.setPower(0);

                if (gamepad2.dpad_right) {
                    robot.motorRF.setPower(1);
                    telemetry.addData("Motor = ", "MotorRF");
                }else robot.motorRF.setPower(0);

                if(gamepad2.dpad_left) {
                    robot.motorRR.setPower(1);
                    telemetry.addData("Motor = ", "MotorRR");
                }else robot.motorRR.setPower(0);



                // Send telemetry message to signify robot running;
                telemetry.addData("motorRF",  robot.motorRF.getCurrentPosition());
                telemetry.addData("motorLF", robot.motorLF.getCurrentPosition());
                telemetry.addData("motorLR", robot.motorLR.getCurrentPosition());
                telemetry.addData("motorRR", robot.motorRR.getCurrentPosition());
                telemetry.update();

            } // end of while loop
        } // end of runOpMode
    } // end of LinearOpMode


