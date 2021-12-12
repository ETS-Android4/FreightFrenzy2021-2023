package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;

@TeleOp(name="Broken Bot Testing", group="Dev")
//@Disabled

public class BrokenBot extends LinearOpMode{

    public double lfValue = 0, lrValue = 0, rfValue = 0, rrValue = 0;

    /* Declare OpMode members. */
        HWProfile robot           = new HWProfile();
        public double servoPosition = 0;
        public double robotAngle, rightX, rightY, v1, v2, v3, v4, theta, theta2, r;
        public int targetPosition=0;
        public double cupPosition=0;

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
            robot.servoIntake.setPosition(0);

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Arm angle",robot.motorArm.getCurrentPosition());
                telemetry.update();
                robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
                rightX = gamepad1.right_stick_x;
                rightY = -gamepad1.right_stick_y;
                r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

                v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
                v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
                v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
                v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);

                robot.motorLF.setPower(com.qualcomm.robotcore.util.Range.clip((v1 + lfValue), -1, 1));
                robot.motorRF.setPower(com.qualcomm.robotcore.util.Range.clip((v2 + rfValue), -1, 1));
                robot.motorLR.setPower(com.qualcomm.robotcore.util.Range.clip((v3 + lrValue), -1, 1));
                robot.motorRR.setPower(com.qualcomm.robotcore.util.Range.clip((v4 + rrValue), -1, 1));

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


                if(gamepad2.left_trigger >0){
                    servoPosition = servoPosition + 0.005;
                    if(servoPosition >1) servoPosition = 1;
                }  else if(gamepad2.right_trigger >0 ) {
                    servoPosition = servoPosition - 0.005;
                    if(servoPosition < -1) servoPosition = -1;
                }   // end if

                if(gamepad1.dpad_down) {
                    targetPosition = robot.ARMPOSITIONDOWN;
                    cupPosition= robot.INTAKECUPDOWN;
                } else if(gamepad1.dpad_right){
                    targetPosition = robot.ARMCUPMID;
                    cupPosition = robot.INTAKECUPMID;
                }else if(gamepad1.dpad_left ) {
                    targetPosition = robot.ARMCUPHIGH;
                    cupPosition = -0.5;
                }else if(gamepad1.dpad_up){
                    targetPosition = robot.ARMCUPSHARED;
                    cupPosition = robot.INTAKECUPHIGH;
                }


                robot.servoIntake.setPosition(cupPosition);
                sleep(50);
                robot.motorArm.setTargetPosition(targetPosition);
                robot.motorArm.setPower(-0.5);


                robot.servoIntake.setPosition(servoPosition);

                if (gamepad2.a){
                    robot.motorIntake.setPower(1);
                } else {
                    robot.motorIntake.setPower(0);
                }
                if(Math.abs(gamepad2.right_stick_y) > 0.1){
                    robot.motorArm.setPower(gamepad2.right_stick_y * 0.5);
                }   else {
                    robot.motorArm.setPower(0);
                }   // end if
                telemetry.addData("ArmEncoder", robot.motorArm.getCurrentPosition());
                telemetry.addData("servoIntake Position = ", robot.servoIntake.getPosition());
                telemetry.addData("motorRF", robot.motorRF.getCurrentPosition());
                telemetry.addData("motorLF", robot.motorLF.getCurrentPosition());
                telemetry.addData("motorRR", robot.motorRR.getCurrentPosition());
                telemetry.addData("motorLR", robot.motorLR.getCurrentPosition());
                telemetry.update();
                // Send telemetry message to signify robot running;

            } // end of while loop
        } // end of runOpMode
    } // end of LinearOpMode