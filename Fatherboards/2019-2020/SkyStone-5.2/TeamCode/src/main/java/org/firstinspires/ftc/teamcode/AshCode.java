package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class AshCode extends LinearOpMode{

    private float stickSensitivity = 0.13f; //> than this gets registered as input

    // Gabe told me to do this. Help.
    public DcMotor leftMotor;
    public DcMotor leftMotor2;
    public DcMotor rightMotor;
    public DcMotor rightMotor2;

    public float motorPower = 1f;

    public DcMotor liftPivotMotor;
    public float liftPivotServoPos = 0; //What the motor above's current pos is; Sets to this at start
    public float liftPivotSensitivity = 0.5f;
    private boolean liftLocked; //If true, the lift stays in place
    private float liftLockPower = 0.5f; //What the motors's power needs to be to stay in place holding our lift's weight

    public float servoSensitivity = 0.005f;
    public Servo intakeServo;

    @Override
    public void runOpMode()
    {
        //connects motors to hub & phone- use name in quotes for config
        leftMotor = hardwareMap.get(DcMotor.class, "left_Motor");
        leftMotor2 = hardwareMap.get(DcMotor.class, "left_Motor2");

        rightMotor = hardwareMap.get(DcMotor.class, "right_Motor");
        rightMotor2 = hardwareMap.get(DcMotor.class, "right_Motor2");

        /*drawerMotor = hardwareMap.get(DcMotor.class, "drawer_Motor");
        liftPivotMotor= hardwareMap.get(DcMotor.class, "pivot_Motor");
        intakeServo1 = hardwareMap.get(CRServo.class, "leftVexMotor");
        intakeServo2 = hardwareMap.get(CRServo.class, "rightVexMotor");
        intakePivotServo = hardwareMap.get(Servo.class, "intakePivotServo");*/

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);

        /*drawerMotor.setDirection(DcMotor.Direction.REVERSE);
        drawerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftPivotMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));*/


        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //intakePivotServo.setPosition(intakePivotServoPos);

        //drawerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //drawerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart(); //press play button, actives opMode

        while (opModeIsActive())
        {
            drive();
            Intake();
            //pivotLift();
            //toggleIntake();
            //toggleDrawerSlides();
            telemetry.update();
            //telemetry.addData("drawerMotor", drawerMotor.getCurrentPosition());
        }//opModeIsActive

    }//runOpMode

    public void drive()
    {
        /*
        deadzone. If result of setPower() is small. Telemetry was giving values for setPower so idk xy cords.
        Note: we forgot to include the rightstick in the if statement so
        the robot wasn't accounting for it at all (lol)
         */
        if((Math.abs(gamepad1.left_stick_x) > 0.2 || (Math.abs(gamepad1.left_stick_y) > 0.2 )) || (Math.abs(gamepad1.right_stick_x) > 0.2 || (Math.abs(gamepad1.right_stick_y) > 0.2 )))
        {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX * motorPower;
            final double v2 = r * Math.sin(robotAngle) - rightX * motorPower;
            final double v3 = r * Math.sin(robotAngle) + rightX * motorPower;
            final double v4 = r * Math.cos(robotAngle) - rightX * motorPower;

            leftMotor.setPower(v1);
            rightMotor.setPower(v2);
            leftMotor2.setPower(v3);
            rightMotor2.setPower(v4);
        }
        else
        {
            leftMotor.setPower(0);
            leftMotor2.setPower(0);

            rightMotor.setPower(0);
            rightMotor2.setPower(0);
        }

        if(gamepad1.left_bumper)
            motorPower = 1f;
        else if(gamepad1.right_bumper)
            motorPower = 0.5f;

        telemetry.addData("Left Motor: ", leftMotor.getPower());
        telemetry.addData("Left Motor 2: ", leftMotor2.getPower());

        telemetry.addData("Right Motor: ", rightMotor.getPower());
        telemetry.addData("Right Motor 2: ", rightMotor2.getPower());

        telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);
    }

    public void Intake()
    {
        //Servo
        if(gamepad2.a)
            intakeServo.setPosition(-1);
        if(gamepad2.y)
            intakeServo.setPosition(1);

        telemetry.addData("Servo Pos / Port" ,intakeServo.getPosition() + " | " + intakeServo.getPortNumber());
    }

    public void pivotLift() {
        if(Math.abs(gamepad2.right_stick_y) > stickSensitivity)
            liftPivotMotor.setPower((gamepad2.right_stick_y * liftPivotSensitivity));
        /*if(Math.abs(gamepad2.left_stick_y) > stickSensitivity)
        {
            liftPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftPivotMotor.setPower(-gamepad2.left_stick_y * liftPivotSensitivity);
        }
        if (gamepad2.right_bumper) {
            liftPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftPivotMotor.setTargetPosition(1650);
            liftPivotMotor.setPower(0.35f);
        }
        if (gamepad2.left_bumper) {
            liftPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftPivotMotor.setTargetPosition(-1650);
            liftPivotMotor.setPower(0.35f);
        }*/
    }
}
