/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test: TeleOp Drive & Arm", group = "Motors")
//@Disabled
public class Shawn_TeleopDriveAndArm extends OpMode {

    /* Declare OpMode members. */

    public static final double ARM_INCREMENT      = 0.01;
    public static final double CLAW_INCREMENT     = 0.03;
    public static final int LOOP_WAIT             = 5;
    public static final double rightStrafeControl = 0.875;
    public static final double leftStrafeControl  = 0.885;

    HardwareShawn Shawn = new HardwareShawn();   // Use Shawn's hardware

    boolean drive = true;

    double initServoPos;

    final int ticksPerRotation = 1440;

    double armPosition = 1;
    double clawPosition;

    int numLoops = 0;

    boolean controlType = true;

    @Override
    public void init() {

        Shawn.init(hardwareMap, false);

        Shawn.armServo.setPosition(armPosition);
        initServoPos = Shawn.armServo.getPosition();

        Shawn.rightClaw.setPosition(clawPosition);
        Shawn.leftClaw.setPosition(1 - clawPosition);

        telemetry.addData("Initial servo armPosition", "%4.2f", initServoPos);
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {

        int control = 1;

        double lf = 0;
        double lr = 0;
        double rf = 0;
        double rr = 0;

        if (gamepad1.right_bumper) {
            control = 2;
        }
        if (gamepad1.left_bumper) {
            control = 4;
        }

            if (controlType) { // y controls
                // Left stick's y direction is used to control left wheels'
                // forward and reverse movement
                double lsy = gamepad1.left_stick_y;

                // Left stick's x direction is used for strafing
                double lsx = gamepad1.left_stick_x;

                if (Math.abs(lsy) > Math.abs(lsx)) {
                    lsx = 0;
                } else {
                    lsy = 0;
                }

                lf = lsy + lsx;
            lr = lsy - lsx;

            // Right stick's y direction is used to control right wheels'
            // forward and reverse movement
            rf = gamepad1.right_stick_y - lsx;
            rr = gamepad1.right_stick_y + lsx;

            // Adjusting front wheel powers, depending on which direction
            // the robot is strafing, to make it strafe "straight"
            if (control == 1) {
                if (lsx >= 0) {
                    lf *= rightStrafeControl;
                    rf *= rightStrafeControl;
                } else {
                    lf *= leftStrafeControl;
                    rf *= leftStrafeControl;
                }
            }

            lf = Range.clip(lf, -1, 1);
            lr = Range.clip(lr, -1, 1);
            rf = Range.clip(rf, -1, 1);
            rr = Range.clip(rr, -1, 1);

        } else { // x control

            if (-gamepad1.left_stick_y != 0) {
                lf = gamepad1.left_stick_y;
                lr = gamepad1.left_stick_y;
                rf = gamepad1.left_stick_y;
                rr = gamepad1.left_stick_y;
            }

            if (gamepad1.right_stick_x != 0) {
                lf = -gamepad1.right_stick_x;
                lr = -gamepad1.right_stick_x;
                rf = gamepad1.right_stick_x;
                rr = gamepad1.right_stick_x;

            }

            if (gamepad1.left_trigger != 0) {

                if (control == 1) {
                    lf = -1 * leftStrafeControl;
                    rf = 1 * leftStrafeControl;
                } else {
                    lf = -1;
                    rf = 1;
                }
                lr = 1;
                rr = -1;

//                if (control == 1) {
//                    if (gamepad1.right_stick_x >= 0) {
//                        lf = gamepad1.right_stick_x  * rightStrafeControl;
//                        rf = -gamepad1.right_stick_x * rightStrafeControl;
//                    }
//                    else{
//                        lf = gamepad1.right_stick_x  * leftStrafeControl;
//                        rf = -gamepad1.right_stick_x * leftStrafeControl;
//                    }
//                }
//                else {
//                    lf = gamepad1.right_stick_x;
//                    rf = -gamepad1.right_stick_x;
//                }
//                lr = -1;
//                rr = 1;

            }
            if (gamepad1.right_trigger != 0) {
                if (control == 1) {
                    lf = 1 * rightStrafeControl;
                    rf = -1 * rightStrafeControl;
                } else {
                    lf = 1;
                    rf = -1;
                }
                lr = -1;
                rr = 1;
            }

        }

        // for debugging
        if (gamepad2.x) {
            Shawn.leftRear.setPower(1);
        }
        if (gamepad2.y) {
            Shawn.leftFront.setPower(1);
        }
        if (gamepad2.a) {
            Shawn.rightRear.setPower(1);
        }
        if (gamepad2.b) {
            Shawn.rightFront.setPower(1);
        }

        Shawn.leftRear.setPower(lf / control);
        Shawn.leftFront.setPower(lr / control);
        Shawn.rightRear.setPower(rf / control);
        Shawn.rightFront.setPower(rr/ control);

        if (gamepad1.x) {
            controlType = false;
        }
        if (gamepad1.y) {
            controlType = true;
        }

        int clawControl = 1;

        if (gamepad2.right_bumper) {
            clawControl = 2;
        }
        if (gamepad2.left_bumper) {
            clawControl = 4;
        }

        if (numLoops == LOOP_WAIT) {
            if (-gamepad2.left_stick_y < 0) {
                if (armPosition < 1) {
                    armPosition += ARM_INCREMENT/clawControl;
                    if (armPosition > 1) {
                        armPosition = 1;
                    }
                }
            } else if (-gamepad2.left_stick_y > 0) {
                if (armPosition > 0) {
                    armPosition -= ARM_INCREMENT/clawControl;
                    if (armPosition < 0) {
                        armPosition = 0;
                    }
                }
            }
        }

        Shawn.armServo.setPosition(armPosition);

        telemetry.addData("gamepad2.left_stick_y: ", "%4.2f", -gamepad2.left_stick_y);
        telemetry.addData("servo armPosition: ", "%4.2f", Shawn.armServo.getPosition());
        telemetry.addData("armPosition: ", "%4.2f", armPosition);

        if (numLoops == LOOP_WAIT) {
            if (gamepad2.right_stick_x > 0) {
                if (clawPosition < 1) {
                    clawPosition += CLAW_INCREMENT/clawControl;
                    if (clawPosition > 1) {
                        clawPosition = 1;
                    }
                }
            } else if (gamepad2.right_stick_x < 0) {
                if (clawPosition > 0) {
                    clawPosition -= CLAW_INCREMENT/clawControl;
                    if (clawPosition < 0) {
                        clawPosition = 0;
                    }
                }
            }
            numLoops = 0;
        }

        Shawn.rightClaw.setPosition(clawPosition - 0.05);
        Shawn.leftClaw.setPosition(1 - clawPosition);

        if (numLoops < 10) {
            numLoops += 1;
        }



    }

}
