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
@Autonomous(name = "TempAutoRight")

public class TempAutoRight extends LinearOpMode {
    private float stickSensitivity = 0.25f; //> than this gets registered as input

    // Gabe told me to do this. Help.
    public DcMotor leftMotor;
    public DcMotor leftMotor2;
    public DcMotor rightMotor;
    public DcMotor rightMotor2;

    public float motorPower = 1f;

    public DcMotor liftPivotMotor;
    public float liftPivotPower = 0.3f;

    public float servoSensitivity = 0.1f;
    public float servoPower = 1;
    public CRServo intakeExtensionServo;
    public CRServo intakeMainServo;

    @Override
    public void runOpMode() {
        //Connects motors to hub & phone- use name in quotes for config
        leftMotor = hardwareMap.get(DcMotor.class, "left_Motor");
        leftMotor2 = hardwareMap.get(DcMotor.class, "left_Motor2");

        rightMotor = hardwareMap.get(DcMotor.class, "right_Motor");
        rightMotor2 = hardwareMap.get(DcMotor.class, "right_Motor2");

        liftPivotMotor = hardwareMap.get(DcMotor.class, "liftPivotMotor");

        intakeExtensionServo = hardwareMap.get(CRServo.class, "intakeExtensionServo");
        intakeMainServo = hardwareMap.get(CRServo.class, "intakeMainServo");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        liftPivotMotor.setDirection(DcMotor.Direction.FORWARD);
        liftPivotMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart(); //press play button, actives opMode

        if (opModeIsActive()) {

            forward();
            liftUp();
            intakeOut();
            sleep(2000);

            brake();
            intakeIn();
            liftDown();
            sleep(4000);

            backward();
            intakeHold();
            sleep(4000);

            brake();
            sleep(1000);

            strafeRight();
            sleep(4000);

            brake();
            sleep(1000);

            intakeRelease();
            sleep(500);

            intakeBrake();
            sleep(2000);

        }
    }

    public void strafeRight()
    {
        leftMotor.setPower(0.5);
        rightMotor.setPower(-0.5);
        leftMotor2.setPower(-0.5);
        rightMotor2.setPower(0.5);
    }

    public void strafeLeft()
    {
        leftMotor.setPower(-0.5);
        rightMotor.setPower(0.5);
        leftMotor2.setPower(0.5);
        rightMotor2.setPower(-0.5);
    }

    public void brake()
    {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor2.setPower(0);
    }

    public void forward()
    {
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        leftMotor2.setPower(0.5);
        rightMotor2.setPower(0.5);
    }

    public void backward()
    {
        leftMotor.setPower(-0.5);
        rightMotor.setPower(-0.5);
        leftMotor2.setPower(-0.5);
        rightMotor2.setPower(-0.5);
    }

    public void liftUp()
    {
        liftPivotMotor.setPower(.7);
    }

    public void liftDown()
    {
        liftPivotMotor.setPower(.1);
    }

    public void intakeOut()
    {
        intakeMainServo.setPower(0.7);
        intakeExtensionServo.setPower(0.7);
    }

    public void intakeIn()
    {
        intakeMainServo.setPower(-0.7);
        intakeExtensionServo.setPower(-0.7);
    }

    public void intakeHold()
    {
        intakeMainServo.setPower(-.1);
        intakeExtensionServo.setPower(.1);
    }

    public void intakeRelease()
    {
        intakeMainServo.setPower(.1);
        intakeExtensionServo.setPower(-.1);
    }

    public void intakeBrake()
    {
        intakeExtensionServo.setPower(0);
        intakeMainServo.setPower(0);

    }

}
