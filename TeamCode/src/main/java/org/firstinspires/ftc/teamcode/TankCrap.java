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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Devices.DriveBase;



/**
 * Created by aloto10107 on 9/6/17.
 */

@TeleOp(name = "HoloTeleOp", group = "Driver Controlled")
public class TankCrap extends OpMode {

    public DriveBase drive;
    private boolean preLeftBumper = true;
    private boolean preB = true;

    @Override
    public void init() {

        drive = new DriveBase(hardwareMap);
        drive.upanddown.setPosition(1);
    }

    @Override
    public void loop() {
        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;


        leftY = Math.pow(leftY, 3);
        leftX = Math.pow(leftX, 3);
        //rightX = Math.pow(rightX, 3);

        drive.rightMotors[0].setPower(rightY);
        drive.rightMotors[1].setPower(rightY);
        drive.leftMotors[0].setPower(leftY);
        drive.leftMotors[1].setPower(leftY);

        drive.setLift(0.9 * (gamepad2.left_stick_y));
        drive.setSidearm(0.9 * (gamepad2.right_stick_x));


        if (gamepad2.left_trigger == 1) {
            drive.BodGot();
        }
        if (gamepad2.left_trigger != 1) {
            drive.NoBodGot();
        }
        if (gamepad2.x) {
            drive.ReverseBodGot();
        }
        if (gamepad2.y) {
            drive.skillup();
        }
        if (!gamepad2.y) {
            drive.skilldown();
        }
        /*if (gamepad2.left_bumper && !preLeftBumper){
            preLeftBumper = true;
        }
        else if (!gamepad2.left_bumper){
            preLeftBumper = false;
        }
        if (gamepad2.b) {
            drive.jaws.setPosition(.08);
        }
        if(gamepad2.left_bumper && preLeftBumper){
            drive.jaws.setPosition(0.125);
            preLeftBumper = false;
        }*/

        if (gamepad2.left_bumper && !preLeftBumper) {

            if (drive.jaws.getPosition() != .125){
                drive.jaws.setPosition(.125);
            }
            else if (drive.jaws.getPosition() == .125) {
                drive.jaws.setPosition(0.02);
            }
        }
        if (gamepad2.b && !preB){
            if(drive.jaws.getPosition() != 0.08){
                drive.jaws.setPosition(0.08);
            } else if (drive.jaws.getPosition() == 0.08) {
                drive.jaws.setPosition(0.02);
            }
        }
        preB = gamepad2.b;
        preLeftBumper = gamepad2.left_bumper;
        if (gamepad2.right_bumper){
            try {
                drive.pickyuppything();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

//there are 26 lightning bolts on the robot
        if (gamepad2.a) {
            try {
                drive.standyuppything();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        if (gamepad2.right_trigger == 1){
            drive.dump.setPosition(.85);
        }
        if (gamepad2.right_trigger != 1){
            drive.dump.setPosition(0);
        }
        telemetry.update();

    }

}