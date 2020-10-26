package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FourBarPrograms.MecanumHardware2;

public class TankDrive extends LinearOpMode {

    MecanumHardware2 robot = new MecanumHardware2();
    
    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        telemetry.addData("Say:", "Driver");
    }
}
