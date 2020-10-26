package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OneStoneBlueAlliance")
public class OneStoneBlueAlliance extends LinearOpMode {

    AutonomousObject robot = new AutonomousObject();

    @Override()
    public void runOpMode()
    {
        waitForStart();
        if(opModeIsActive()) {
            robot.init(hardwareMap);

            robot.extendForward();
            robot.clawUp();
            sleep(500);

            robot.forward();
            sleep(800); //1300

            robot.brake();
            robot.extendStop();
            robot.clawDown();
            sleep(1000); //1000

            robot.backward();
            robot.clawDown();
            sleep(700); //1000

            robot.brake();
            robot.liftUp();
            sleep(1000);

            robot.strafeLeft();
            sleep(3000); //4000

            robot.brake();
            sleep(1000);

            robot.clawUp();
            sleep(500);

            robot.backward();
            robot.clawUp();
            sleep(250);

            robot.brake();
            robot.liftDown();
            robot.clawDown();
            sleep(700);

            robot.strafeRight();
            sleep(1500); //1600

            robot.brake();
            sleep(25000);


            /*robot.init(hardwareMap);

            robot.extendForward();
            robot.clawUp();
            robot.forward();
            sleep(1600);

            robot.brake();
            robot.extendStop();
            robot.clawDown();
            sleep(1000);

            robot.backward();
            robot.clawDown();
            sleep(600);

            robot.brake();
            robot.liftUp();
            sleep(1000);

            robot.strafeLeft();
            sleep(5800); //3700

            robot.brake();
            sleep(1000);

            robot.clawUp();
            robot.extendBack();
            //robot.backward();
            sleep(250);

            robot.clawUp();
            robot.brake();
            robot.liftDown();
            robot.extendBack();
            sleep(700);

            robot.clawUp();
            robot.strafeRight();
            robot.extendStop();
            sleep(400); //1500

            robot.clawDown();
            sleep(500);

            robot.strafeRight();
            sleep(1600); //1500

            robot.brake();
            sleep(25000);*/
        }
    }
}
