package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMechanum;

@TeleOp(name = "Teleop Mode", group = "Mouse Spit")

public class MecanumTeleOp extends LinearOpMode {
        private final static HWProfile robot = new HWProfile();
        private LinearOpMode opMode = this;

        @Override
        public void runOpMode(){
            double v1, v2, v3, v4, robotAngle, powerLevel=1;
            double modePower = 1;
            double theta;
            double theta2 = 180;
            double r;
            double rightX, rightY;
            boolean fieldCentric = false;
            double armPosition = 0.9;

            double buttonPress=0;
            ElapsedTime currentTime= new ElapsedTime();

            robot.init(hardwareMap);

            telemetry.addData("Ready to Run: ","GOOD LUCK");
            telemetry.update();

            /*
             * Initialize the drive class
             */
            DriveMechanum drive = new DriveMechanum(robot, opMode);

            waitForStart();

            /* Prepare the robot for driver control operation
             * - Set the transfer to the down position
             * - Move the ring kick servo to the ready position
             *  - turn on the intake motor
             *  - set the shooter speed to the desired speed
             */

            buttonPress = currentTime.time();
            while (opModeIsActive()) {

                /*
                 * Mecanum Drive Control section
                 */

/*
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive

                theta = robot.imu.getAngularOrientation().firstAngle;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }   // end of if(fieldCentric)
            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            v1 = (r * Math.cos(robotAngle + Math.toRadians(theta + theta2)) - rightX + rightY) * powerLevel;
            v2 = (r * Math.sin(robotAngle + Math.toRadians(theta + theta2)) + rightX + rightY) * powerLevel;
            v3 = (r * Math.sin(robotAngle + Math.toRadians(theta + theta2)) - rightX + rightY) * powerLevel;
            v4 = (r * Math.cos(robotAngle + Math.toRadians(theta + theta2)) + rightX + rightY) * powerLevel;

            robot.motorRF.setPower(Range.clip((v3 * modePower), -1, 1));
            robot.motorLF.setPower(Range.clip((v4 * modePower), -1, 1));
            robot.motorRR.setPower(Range.clip((v1 * modePower), -1, 1));
            robot.motorLR.setPower(Range.clip((v2 * modePower), -1, 1));

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
                    robot.motorDuck.setPower(1);
                } else if (gamepad1.right_bumper) {
                    robot.motorDuck.setPower(-1);
                } else {
                    robot.motorDuck.setPower(0);
            }


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
