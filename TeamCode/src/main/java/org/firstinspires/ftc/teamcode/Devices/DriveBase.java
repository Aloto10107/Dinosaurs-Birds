package org.firstinspires.ftc.teamcode.Devices;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class DriveBase extends LinearOpMode{
    private DcMotor[] leftMotors;
    private DcMotor[] rightMotors;
    private DcMotor lift;
    private DcMotor sidearm;
    private Servo[] pinchies;
    private Servo skill_crane;
    private Servo jaws;
    public Servo upanddown;
    BNO055IMU imu;
    private Orientation angles;
    private ColorSensor color;
    public DistanceSensor distance;
    public float Gerror;
    public float Derror;

    VuforiaLocalizer vuforia;

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

        pinchies = new Servo[5];
        pinchies[0] = hardwareMap.servo.get("top_right");
        pinchies[1] = hardwareMap.servo.get("top_left");
        pinchies[2] = hardwareMap.servo.get("bottom_right");
        pinchies[3] = hardwareMap.servo.get("bottom_left");
        pinchies[4] = hardwareMap.servo.get("center");

        lift = hardwareMap.dcMotor.get("PinchArm");
        sidearm = hardwareMap.dcMotor.get("SideArm");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        skill_crane = hardwareMap.servo.get("skill_crane");
        jaws = hardwareMap.servo.get("jaws");
        upanddown = hardwareMap.servo.get("upanddown");
        color = hardwareMap.get(ColorSensor.class, "color");
        distance = hardwareMap.get(DistanceSensor.class, "color");
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
    public synchronized void turn(double power, long time){
        setMotor_bl(-power);
        setMotor_fl(-power);
        setMotor_br(power);
        setMotor_fr(power);
        sleep(time);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_br(0);
        setMotor_fr(0);
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
        pinchies[0].setPosition(1);
        pinchies[1].setPosition(0);
        pinchies[2].setPosition(0);
        pinchies[3].setPosition(1);
    }
    public void notPinch()
    {
        pinchies[0].setPosition(.6);
        pinchies[1].setPosition(.4);
        pinchies[2].setPosition(.6);
        pinchies[3].setPosition(.4);
    }
    public void flip()
    {
        if (pinchies[4].getPosition() == 1){
            pinchies[4].setPosition(0);
        }
        else if (pinchies[4].getPosition() == 0){
            pinchies[4].setPosition(1);
        }
    }
    public void openJaws(){
        jaws.setPosition(0);
    }
    public void closeJaws(){
        jaws.setPosition(1);
    }
    public void skillup(){
/*        double newPos = skill_crane.getPosition() + 0.2;
        skill_crane.setPosition(newPos);*/
        skill_crane.setPosition(0);
    }
    public void skilldown(){
/*        double newPos = skill_crane.getPosition() - 0.2;
        skill_crane.setPosition(newPos);*/
        skill_crane.setPosition(1);
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

        Gerror = getHeading() - degrees;
        while (Math.abs(Gerror) >= 4) {
            float Kp = (float) 0.01;
            Gerror = getHeading() - degrees;
            setMotor_bl(-Gerror * Kp);
            setMotor_fl(-Gerror * Kp);
            setMotor_br(Gerror * Kp);
            setMotor_fr(Gerror * Kp);
            if (Math.abs(Gerror) <= 4) {
                setMotor_bl(0);
                setMotor_fl(0);
                setMotor_br(0);
                setMotor_fr(0);
                break;
            }
        }
    }
    public void toDistance(float position){

        Derror = (float) (distance.getDistance(DistanceUnit.MM) - position);
        while (Math.abs(Derror) >= 4) {
            float Kp = (float) 0.01;
            Derror = (float) (distance.getDistance(DistanceUnit.MM) - position);
            setMotor_bl(-Derror * Kp);
            setMotor_fl(-Derror * Kp);
            setMotor_br(Derror * Kp);
            setMotor_fr(Derror * Kp);
            if (Math.abs(Derror) <= 4) {
                setMotor_bl(0);
                setMotor_fl(0);
                setMotor_br(0);
                setMotor_fr(0);
                break;
            }
        }
    }

    public int[] getColor() {

        int red = color.red();
        int green = color.green();
        int blue = color.blue();
        int[] colors = new int[] {red, green, blue};
        return colors;
    }



    @Override
    public void runOpMode() throws InterruptedException {

    }
}


