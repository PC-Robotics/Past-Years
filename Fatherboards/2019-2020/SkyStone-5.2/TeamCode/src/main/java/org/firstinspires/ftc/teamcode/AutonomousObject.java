/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Base64;

import static java.lang.Thread.sleep;


/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class AutonomousObject {

    private float stickSensitivity = 0.25f; //> than this gets registered as input
    private final double encoderTicksPerInch = 42;
    private final int blockInches = 24;
    private final float motorPower = .5f;
    // Gabe told me to do this. Help.
    public DcMotor leftMotor;
    public DcMotor leftMotor2;
    public DcMotor rightMotor;
    public DcMotor rightMotor2;

    public DcMotor liftPivotMotor;

    public CRServo intakeExtensionServo;
    public CRServo intakeClawServo;

    HardwareMap hwMap;

    public void init(HardwareMap ahwMap)  {

        hwMap = ahwMap;
        //Connects motors to hub & phone- use name in quotes for config
        leftMotor = hwMap.get(DcMotor.class, "left_Motor");
        leftMotor2 = hwMap.get(DcMotor.class, "left_Motor2");

        rightMotor = hwMap.get(DcMotor.class, "right_Motor");
        rightMotor2 = hwMap.get(DcMotor.class, "right_Motor2");

        liftPivotMotor = hwMap.get(DcMotor.class, "liftPivotMotor");

        intakeExtensionServo = hwMap.get(CRServo.class, "intakeExtensionServo");
        intakeClawServo = hwMap.get(CRServo.class, "intakeClawServo");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftPivotMotor.setDirection(DcMotor.Direction.REVERSE);
        liftPivotMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
    }
    /*
    public void encoderDrive(int inches,
                             double leftInches, double rightInches,
                             double timeoutS) {

        int ticks = inches * (int) encoderTicksPerInch;

        int leftTarget1 = leftMotor.getCurrentPosition() + ticks;
        int leftTarget2 = leftMotor2.getCurrentPosition() + ticks;
        int rightTarget1 = rightMotor.getCurrentPosition() + ticks;
        int rightTarget2 = rightMotor2.getCurrentPosition() + ticks;

        leftMotor.setTargetPosition(leftTarget1);
        leftMotor2.setTargetPosition(leftTarget2);
        rightMotor.setTargetPosition(rightTarget1);
        rightMotor2.setTargetPosition(rightTarget2);


        // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void strafeRight(int inches) {
        int ticks = inches * (int) encoderTicksPerInch;

        int leftTarget1 = leftMotor.getCurrentPosition() + ticks;
        int leftTarget2 = leftMotor2.getCurrentPosition() + ticks;
        int rightTarget1 = rightMotor.getCurrentPosition() + ticks;
        int rightTarget2 = rightMotor2.getCurrentPosition() + ticks;

        leftMotor.setTargetPosition(leftTarget1);
        leftMotor2.setTargetPosition(leftTarget2);
        rightMotor.setTargetPosition(rightTarget1);
        rightMotor2.setTargetPosition(rightTarget2);

        while(leftMotor.isBusy() || leftMotor2.isBusy() || rightMotor.isBusy() || rightMotor2.isBusy()) {
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        strafeRight();

        boolean achieved = leftMotor.getCurrentPosition() == leftTarget1 && leftMotor2.getCurrentPosition() == leftTarget2
                && rightMotor.getCurrentPosition() == rightTarget1 && rightMotor2.getCurrentPosition() == rightTarget2;
        //if(!(leftMotor.isBusy() && leftMotor2.isBusy() && rightMotor.isBusy() && rightMotor2.isBusy())) {
            brake();
        //    leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //}

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void strafeLeft(int inches) {
        int ticks = inches * (int) encoderTicksPerInch;

        int leftTarget1 = leftMotor.getCurrentPosition() + inches;
        int leftTarget2 = leftMotor2.getCurrentPosition() + inches;
        int rightTarget1 = rightMotor.getCurrentPosition() + inches;
        int rightTarget2 = rightMotor2.getCurrentPosition() + inches;

        leftMotor.setTargetPosition(leftTarget1);
        leftMotor2.setTargetPosition(leftTarget2);
        rightMotor.setTargetPosition(rightTarget1);
        rightMotor2.setTargetPosition(rightTarget2);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        strafeLeft();

        boolean achieved = leftMotor.getCurrentPosition() == leftTarget1 && leftMotor2.getCurrentPosition() == leftTarget2
                && rightMotor.getCurrentPosition() == rightTarget1 && rightMotor2.getCurrentPosition() == rightTarget2;
        if(achieved) {
            brake();
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void forward(int inches) {
        int ticks = inches * (int) encoderTicksPerInch;

        int leftTarget1 = leftMotor.getCurrentPosition() + ticks;
        int leftTarget2 = leftMotor2.getCurrentPosition() + ticks;
        int rightTarget1 = rightMotor.getCurrentPosition() + ticks;
        int rightTarget2 = rightMotor2.getCurrentPosition() + ticks;

        leftMotor.setTargetPosition(leftTarget1);
        leftMotor2.setTargetPosition(leftTarget2);
        rightMotor.setTargetPosition(rightTarget1);
        rightMotor2.setTargetPosition(rightTarget2);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        forward();

        boolean achieved = leftMotor.getCurrentPosition() == leftTarget1 && leftMotor2.getCurrentPosition() == leftTarget2
                && rightMotor.getCurrentPosition() == rightTarget1 && rightMotor2.getCurrentPosition() == rightTarget2;
        if(achieved) {
            brake();
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void backward(int inches) {
        int ticks = inches * (int) encoderTicksPerInch;

        int leftTarget1 = leftMotor.getCurrentPosition() + ticks;
        int leftTarget2 = leftMotor2.getCurrentPosition() + ticks;
        int rightTarget1 = rightMotor.getCurrentPosition() + ticks;
        int rightTarget2 = rightMotor2.getCurrentPosition() + ticks;

        leftMotor.setTargetPosition(leftTarget1);
        leftMotor2.setTargetPosition(leftTarget2);
        rightMotor.setTargetPosition(rightTarget1);
        rightMotor2.setTargetPosition(rightTarget2);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backward();

        boolean achieved = leftMotor.getCurrentPosition() == leftTarget1 && leftMotor2.getCurrentPosition() == leftTarget2
                && rightMotor.getCurrentPosition() == rightTarget1 && rightMotor2.getCurrentPosition() == rightTarget2;
        if(achieved) {
            brake();
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    */

    public void strafeRight() {
        leftMotor.setPower(-motorPower);
        rightMotor.setPower(motorPower);
        leftMotor2.setPower(motorPower);
        rightMotor2.setPower(-motorPower);
    }
    

    public void strafeLeft() {
        leftMotor.setPower(motorPower);
        rightMotor.setPower(-motorPower);
        leftMotor2.setPower(-motorPower);
        rightMotor2.setPower(motorPower);
    }

    public void forward() {
        leftMotor.setPower(-motorPower);
        rightMotor.setPower(-motorPower);
        leftMotor2.setPower(-motorPower);
        rightMotor2.setPower(-motorPower);
    }

    public void backward() {
        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);
        leftMotor2.setPower(motorPower);
        rightMotor2.setPower(motorPower);
    }

    public void brake() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor2.setPower(0);
    }

    public void liftUp() {
        liftPivotMotor.setPower(-.35);
    }

    public void liftDown() {
        liftPivotMotor.setPower(.01);
    }

    public void clawUp() {
        intakeClawServo.setPower(-0.6);
    }

    public void clawDown() {
        intakeClawServo.setPower(-0.1);
    }

    public void extendForward() {
        intakeExtensionServo.setPower(1);
    }

    public void extendBack() {
        intakeExtensionServo.setPower(.7);
    }

    public void extendStop()
    {
        intakeExtensionServo.setPower(0);
    }

}
