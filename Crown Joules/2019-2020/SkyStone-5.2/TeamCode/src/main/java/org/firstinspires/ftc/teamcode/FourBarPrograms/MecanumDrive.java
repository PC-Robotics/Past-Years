package org.firstinspires.ftc.teamcode.FourBarPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TankDrive")
public class MecanumDrive extends LinearOpMode {

    MecanumHardware2 robot = new MecanumHardware2();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double clawPosition = 0.5;                  // Servo mid position
        final double CLAW_SPEED = 0.01;                 // sets rate to move servo
        boolean grabberVal = true;
        double motorTester = 0.0;
        boolean directionToggle = true;

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

            robot.leftFront.setPower(gamepad1.left_trigger);
            robot.leftBack.setPower(gamepad1.left_trigger);
            robot.rightFront.setPower(gamepad1.right_trigger);
            robot.rightBack.setPower(gamepad1.right_trigger);


            if(gamepad1.a && directionToggle){
                robot.leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
                robot.rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
                robot.leftBack.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
                robot.rightBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
                directionToggle = false;
            }
            else if(gamepad1.a && !directionToggle){
                robot.leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
                robot.rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
                robot.leftBack.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
                robot.rightBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
                directionToggle = true;
            }

            telemetry.addData("Toggle: ", directionToggle);


/*
            boolean xToggle = true;
            boolean yToggle = true;

            if(gamepad1.x && xToggle){
                robot.input.setPower(1);
                xToggle = false;
            }
            else if(gamepad1.x && !xToggle){
                robot.input.setPower(0);
                xToggle = true;
            }

            if(gamepad1.y && yToggle){
                robot.output.setPower(1);
                yToggle = false;
            }
            else if(gamepad1.y && !yToggle){
                robot.output.setPower(1);
                yToggle = true;
            }
*/
            //double slowModeVal = gamepad1.left_trigger;
            /*
            double liftVal = 0.75;

            //The left joystick is used to translate the robot, while the right joystick controls the rotation of the robot.
                /*double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x; //Mess with 0.5 value
                double v1 = r * Math.cos(robotAngle) + rightX;
                double v2 = r * -Math.sin(robotAngle) - rightX;
                double v3 = r * Math.sin(robotAngle) + rightX;
                double v4 = r * -Math.cos(robotAngle) - rightX;
                //left_stick_x (top) is neg, was positive before
                if (gamepad1.right_bumper) {
                    v1 = v1 * 0.2;
                    v2 = v2 * 0.2;
                    v3 = v3 * 0.2;
                    v4 = v4 * 0.2;
                }
            double motorCoeff = 1.2;
            double magnitude = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double fld = (magnitude * Math.cos(robotAngle) + rightX) * motorCoeff; //cos +
            double frd = (magnitude * Math.sin(robotAngle) - rightX) * motorCoeff; //sin -
            double bld = (magnitude * Math.sin(robotAngle) + rightX) * motorCoeff; //sin
            double brd = (magnitude * Math.cos(robotAngle) - rightX) * motorCoeff; //cos
            if (gamepad1.right_bumper) {

                fld = fld * 0.4;
                frd = frd * 0.4;
                bld = bld * 0.4;
                brd = brd * 0.4;

            }


            robot.leftFront.setPower(fld);
            robot.rightFront.setPower(frd);
            robot.leftBack.setPower(bld);
            robot.rightBack.setPower(brd);

            telemetry.addData("leftFront", "%.2f", fld);
            telemetry.addData("rightFront", "%.2f", frd);
            telemetry.addData("leftBack", "%.2f", bld);
            telemetry.addData("rightBack", "%.2f", brd);
            //telemetry.addData("grabber", "%.2f", clawPosition);
            //telemetry.addData("grabberServo", robot.grabber.getPosition());
            */

         /* WORK WITH THIS  if (gamepad1.x)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.y)
                clawPosition -= CLAW_SPEED;
                clawPosition = Range.clip(clawPosition, robot.MIN_SERVO, robot.MAX_SERVO);
            robot.grabber.setPosition(clawPosition);
                */

         /*
            if (gamepad2.x && grabberVal) {

                clawPosition += 0.045;
                grabberVal = false;


            } else if (gamepad2.y && grabberVal) {

                clawPosition -= 0.045;
                grabberVal = false;
            }

            if (!gamepad2.x && !gamepad2.y) {
                grabberVal = true;
            }


            clawPosition = Range.clip(clawPosition, 0, 1);
            robot.grabber.setPosition(clawPosition);
            //sleep(500);

            robot.lift.setPower(gamepad2.left_stick_y);

          /*  if (gamepad2.b) {
                robot.lift.setPower(-liftVal);
                motorTester = 1.0;
            }
            else if (gamepad2.a) {
                robot.lift.setPower(liftVal);
                motorTester = 0.0;
            }
            else {
                robot.lift.setPower(0.0);
            } */

            //telemetry.addData("slowmode", "%.2f", slowModeVal);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            //sleep(50);
        }
    }
}