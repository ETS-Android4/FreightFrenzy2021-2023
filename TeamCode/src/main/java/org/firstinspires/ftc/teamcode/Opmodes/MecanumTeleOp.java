package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;

@TeleOp(name = "Teleop Mode", group = "Competition")

public class MecanumTeleOp extends LinearOpMode {
        private final static HWProfile robot = new HWProfile();

        @Override
        public void runOpMode(){
            double v1, v2, v3, v4, robotAngle;
            double theta;
            double theta2 = 180;
            double r;
            double rightX, rightY;
            boolean fieldCentric = false;
            double servoPosition = 0;

            ElapsedTime currentTime= new ElapsedTime();
            double buttonPress = currentTime.time();

            robot.init(hardwareMap);

            telemetry.addData("Ready to Run: ","GOOD LUCK");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {

                /*
                 * Mecanum Drive Control section
                 */
                if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                    theta = robot.imu.getAngularOrientation().firstAngle + 90;
                } else {
                    theta = 0;      // do not adjust for the angular position of the robot
                }

                robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
                rightX = gamepad1.right_stick_x;
                rightY = -gamepad1.right_stick_y;
                r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

                v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
                v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
                v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
                v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);

                robot.motorLF.setPower(com.qualcomm.robotcore.util.Range.clip((v1), -1, 1));
                robot.motorRF.setPower(com.qualcomm.robotcore.util.Range.clip((v2), -1, 1));
                robot.motorLR.setPower(com.qualcomm.robotcore.util.Range.clip((v3), -1, 1));
                robot.motorRR.setPower(com.qualcomm.robotcore.util.Range.clip((v4), -1, 1));

                // Control which direction is forward and which is backward from the driver POV
                if (gamepad1.x && (currentTime.time() - buttonPress) > 0.3) {
                    if (theta2 == 180) {
                        theta2 = 0;
                    } else {
                        theta2 = 180;
                    }
                    buttonPress = currentTime.time();
                }   // end if (gamepad1.x && ...)

                if (gamepad1.left_bumper) {
                    robot.motorDuck.setPower(robot.duckSpeed);
                } else if (gamepad1.right_bumper) {
                    robot.motorDuck.setPower(-robot.duckSpeed);
                } else {
                    robot.motorDuck.setPower(0);
                }   // end of if(gamepad1.left_bumper)

               /* if(gamepad2.left_trigger >0){
                    servoPosition = servoPosition + 0.005;
                    if(servoPosition >1) servoPosition = .1;
                }  else if(gamepad2.right_trigger >0 ) {
                    servoPosition = servoPosition - 0.005;
                    if(servoPosition < -1) servoPosition = -.1;
                }   // end if
                */
                if(gamepad2.left_trigger>0){
                    servoPosition= 0.5;
                }else{
                    servoPosition= 0;
                }

                robot.servoIntake.setPosition(servoPosition);

                if (gamepad2.a){
                    robot.motorIntake.setPower(0.75);
                } else {
                    robot.motorIntake.setPower(0);
                }
                if(Math.abs(gamepad2.right_stick_y) > 0.1){
                    robot.motorArm.setPower(gamepad2.right_stick_y * 0.5);
                }   else {
                    robot.motorArm.setPower(0);
                }   // end if

                // make sure that button press is limited to once every 0.3 seconds

                telemetry.addData("V1 = ", v1);
                telemetry.addData("V2 = ", v2);
                telemetry.addData("V3 = ", v3);
                telemetry.addData("V4 = ", v4);
                telemetry.addData("dpad_up = ", gamepad1.dpad_up);
                telemetry.addData("dpad_down = ", gamepad1.dpad_down);
                telemetry.addData("dpad_left = ", gamepad1.dpad_left);
                telemetry.addData("dpad_right = ", gamepad1.dpad_right);
                telemetry.addData("Left Stick X = ", gamepad1.left_stick_x);
                telemetry.addData("Left Stick Y = ", gamepad1.left_stick_y);
                telemetry.addData("Right Stick X = ", gamepad1.right_stick_x);
                telemetry.addData("Right Stick Y = ", gamepad1.right_stick_y);
                telemetry.addData("Theta = ", theta);
                telemetry.addData("Theta2 = ", theta);
                telemetry.addData("IMU Value: ", theta);
                telemetry.update();

            }   // end of while(opModeIsActive)
        }   // end of runOpMode()
    }       // end of MSTeleop class
