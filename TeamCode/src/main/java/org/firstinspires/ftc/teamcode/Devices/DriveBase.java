package org.firstinspires.ftc.teamcode.Devices;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveBase {
    private DcMotor[] leftMotors;
    private DcMotor[] rightMotors;
    private DcMotor lift;
    private DcMotor sidearm;
    private Servo[] pinchies;
    private Servo relicgrab;
    BNO055IMU imu;
    public Orientation angles;

    public DriveBase(HardwareMap hardwareMap) {
        leftMotors = new DcMotor[2];
        leftMotors[0] = hardwareMap.dcMotor.get("motor_fl");
        leftMotors[1] = hardwareMap.dcMotor.get("motor_bl");

        leftMotors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotors[1].setDirection(DcMotorSimple.Direction.FORWARD);

        rightMotors = new DcMotor[2];
        rightMotors[0] = hardwareMap.dcMotor.get("motor_fr");
        rightMotors[1] = hardwareMap.dcMotor.get("motor_br");

        rightMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        pinchies = new Servo[2];
        pinchies[0] = hardwareMap.servo.get("pinch_r");
        pinchies[1] = hardwareMap.servo.get("pinch_l");

        relicgrab = hardwareMap.servo.get("RelicGrab");

        lift = hardwareMap.dcMotor.get("PinchArm");
        sidearm = hardwareMap.dcMotor.get("SideArm");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
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
    public synchronized void setLift(double power)
    {
        double convertedPower = (power);

        lift.setPower(convertedPower);
    }
    public synchronized void setSidearm(double power)
    {
        double convertedPower = (power);

        sidearm.setPower(convertedPower);
    }
    public void pinch()
    {
        pinchies[0].setPosition(0);
        pinchies[1].setPosition(1);
    }
    public void notPinch()
    {
        pinchies[0].setPosition(.6);
        pinchies[1].setPosition(.4);
    }
    public void grab()
    {
        relicgrab.setPosition(0);
    }
    public void Notgrab()
    {
        relicgrab.setPosition(1);
    }
    public void imuINIT(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }
    public float getHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void gyroTurn(float degrees){

        float error;
        error = degrees - getHeading();

    }

}
