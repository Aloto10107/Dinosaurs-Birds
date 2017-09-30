package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Devices.DriveBase;



/**
 * Created by aloto10107 on 9/6/17.
 */

@TeleOp(name = "theTeleOpMode", group = "Driver Controlled")
public abstract class HoloTeleOp extends OpMode {

    public DriveBase drive;


    @Override
    public void init() {
        drive = new DriveBase(hardwareMap);
    }

    @Override
    public void loop() {
        double rightY = gamepad1.left_stick_y;
        double rightX = gamepad1.left_stick_x;
        double leftX = gamepad1.right_stick_x;

        drive.setMotor_bl(rightY - rightX - leftX);
        drive.setMotor_fl(rightY + rightX - leftX);
        drive.setMotor_br(rightY + rightX + leftX);
        drive.setMotor_fr(rightY - rightX + leftX);


    }
}


