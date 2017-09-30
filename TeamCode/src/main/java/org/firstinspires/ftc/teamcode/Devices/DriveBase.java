package org.firstinspires.ftc.teamcode.Devices;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveBase {
    private DcMotor[] leftMotors;
    private DcMotor[] rightMotors;
    private DcMotor lift;
    private Servo[] pinchies;

    public DriveBase(HardwareMap hardwareMap) {
        leftMotors = new DcMotor[2];
        leftMotors[0] = hardwareMap.dcMotor.get("motor_fl");
        leftMotors[1] = hardwareMap.dcMotor.get("motor_bl");

        leftMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotors = new DcMotor[2];
        rightMotors[0] = hardwareMap.dcMotor.get("motor_fr");
        rightMotors[1] = hardwareMap.dcMotor.get("motor_br");

        rightMotors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotors[1].setDirection(DcMotorSimple.Direction.FORWARD);

        pinchies = new Servo[2];
        pinchies[0] = hardwareMap.servo.get("pinch_r");
        pinchies[1] = hardwareMap.servo.get("pinch_l");

        lift = hardwareMap.dcMotor.get("PinchArm");
    }
    // Sets power of the two left motors
    public synchronized void setLeft(double power)
    {
        double convertedPower = (power);
        // for each motor in leftMotors
        for (DcMotor motor : leftMotors)
        {
            // Set the motor power to power
            motor.setPower(convertedPower);
        }
    }

    // Sets power of the two right motors
    public synchronized void setRight(double power)
    {
        double convertedPower = (power);
        // for each motor in RightMotors
        for (DcMotor motor : rightMotors)
        {
            // Set the motor power to power
            motor.setPower(convertedPower);
        }
    }
    public synchronized void setBoth (double leftPower, double rightPower)
    {
        setLeft(leftPower);
        setRight(rightPower);
    }
    public synchronized void setMotor_fl(double power)
    {
        double convertedPower = (power);

        leftMotors[0].setPower(convertedPower);
    }
    public synchronized void setMotor_bl(double power)
    {
        double convertedPower = (power);

        leftMotors[1].setPower(convertedPower);
    }
    public synchronized void setMotor_fr(double power)
    {
        double convertedPower = (power);

        rightMotors[0].setPower(convertedPower);
    }
    public synchronized void setMotor_br(double power)
    {
        double convertedPower = (power);

        rightMotors[1].setPower(convertedPower);
    }
    public void pinch()
    {
        pinchies[0].setPosition(1);
        pinchies[1].setPosition(0);
    }
    public void notPinch()
    {
        pinchies[0].setPosition(0);
        pinchies[1].setPosition(1);
    }
}
