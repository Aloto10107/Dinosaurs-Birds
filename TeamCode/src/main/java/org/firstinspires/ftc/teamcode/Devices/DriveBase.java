package org.firstinspires.ftc.teamcode.Devices;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.jvm.ByteCodes.error;
import static com.sun.tools.javac.util.Constants.format;
import static java.lang.Thread.sleep;

import java.lang.Math;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class DriveBase {
    public DcMotor[] leftMotors;
    public DcMotor[] rightMotors;
    private DcMotor lift;
    private DcMotor sidearm;
    private Servo[] pinchies;
    private DcMotor[] BodGot;
    public Servo skill_crane;
    public Servo jaws;
    public Servo upanddown;
    public Servo flag;
    public BNO055IMU imu;
    private Orientation angles;
    private Acceleration acceleration;
    private Position position = null;
    private Velocity velocity = null;
    private ColorSensor color;
    public DistanceSensor distance;
    public float Gerror;
    public float deltaError;
    public float Derror;
    public float currentTime;
    public float currentError;
    public float preError;
    public double SpinPos = 0;
    public float tears = 0;
    float deltaTime = 0;
    float preTime = 0;
    float PDout = 0;
    VuforiaLocalizer vuforia;

    public DriveBase(HardwareMap hardwareMap) {
        leftMotors = new DcMotor[2];
        leftMotors[0] = hardwareMap.dcMotor.get("motor_fl");
        leftMotors[1] = hardwareMap.dcMotor.get("motor_bl");

        leftMotors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        //leftMotors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftMotors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotors = new DcMotor[2];
        rightMotors[0] = hardwareMap.dcMotor.get("motor_fr");
        rightMotors[1] = hardwareMap.dcMotor.get("motor_br");

        rightMotors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        //rightMotors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinchies = new Servo[4];
        pinchies[0] = hardwareMap.servo.get("top_right");
        pinchies[1] = hardwareMap.servo.get("top_left");
        pinchies[2] = hardwareMap.servo.get("bottom_right");
        pinchies[3] = hardwareMap.servo.get("bottom_left");

        BodGot = new DcMotor[2];
        BodGot[0] = hardwareMap.dcMotor.get("InLeft");
        BodGot[1] = hardwareMap.dcMotor.get("InRight");

        BodGot[0].setDirection(DcMotorSimple.Direction.FORWARD);
        BodGot[1].setDirection(DcMotorSimple.Direction.REVERSE);

        lift = hardwareMap.dcMotor.get("PinchArm");
        sidearm = hardwareMap.dcMotor.get("SideArm");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        skill_crane = hardwareMap.servo.get("skill_crane");
        jaws = hardwareMap.servo.get("jaws");
        upanddown = hardwareMap.servo.get("upanddown");
        flag = hardwareMap.servo.get("flag");
        color = hardwareMap.colorSensor.get("color");
        distance = hardwareMap.get(DistanceSensor.class, "range");
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
    public synchronized void turn(double power, long time) throws InterruptedException {
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
    public void standyuppything() throws InterruptedException {

        skill_crane.setPosition(0.7);
        sleep(100);
        jaws.setPosition(0.05);
        sleep(100);
        sidearm.setPower(-.1);
        sleep(100);
        sidearm.setPower(0);
    }
    public void pickyuppything() throws InterruptedException{
        skill_crane.setPosition(0.7);
        sleep(100);
        jaws.setPosition(0.07);
        sleep(100);
        skill_crane.setPosition(0);
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
    public void redpinch()
    {
        pinchies[0].setPosition(1);
        pinchies[1].setPosition(0);
    }

    public void bluepinch()
    {
        pinchies[2].setPosition(0);
        pinchies[3].setPosition(1);
    }
    public void rednotPinch()
    {
        pinchies[0].setPosition(.6);
        pinchies[1].setPosition(.4);

    }
    public void bluenotPinch()
    {
        pinchies[2].setPosition(.4);
        pinchies[3].setPosition(.6);
    }
    /*public void flip()
    {
        if (SpinPos == .89){
            pinchies[4].setPosition(.2);
            SpinPos = 0;
        }
        else if (SpinPos == 0){
            pinchies[4].setPosition(.8);
            SpinPos = .89;
        }
    }*/
    public void BodGot(){
        BodGot[0].setPower(-1);
        BodGot[1].setPower(1);
    }
    public void ReverseBodGot(){
        BodGot[0].setPower(1);
        BodGot[1].setPower(-1);
    }
    public void NoBodGot(){
        BodGot[0].setPower(0);
        BodGot[1].setPower(0);
    }
    public void openJaws(){
        jaws.setPosition(0);
    }
    public void closeJaws(){
        jaws.setPosition(1);
    }
    public void jawsDown() throws InterruptedException {
        double newPos = jaws.getPosition() - 0.001;
        jaws.setPosition(newPos);
        Thread.sleep(100);
    }
    public void skillup(){
        //double newPos = skill_crane.getPosition() - 0.2;
        //skill_crane.setPosition(newPos);
        skill_crane.setPosition(.7);
    }
    public void skilldown(){
        //double newPos = skill_crane.getPosition() + 0.2;
        //skill_crane.setPosition(newPos);
        skill_crane.setPosition(0);

    }

    public void imuINIT(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = null;
        imu.initialize(parameters);
    }
    public float getHeading() {
        //PLZ dont touch *touch*
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
        }//there are 26 lightning bolts on the robot
    public double[] getAcceleration(){

        acceleration = imu.getAcceleration();
        double[] accel = new double[] {acceleration.xAccel,acceleration.yAccel,acceleration.zAccel};

        return accel;
    }
    public void drive(float angle, long time, double power){
        double[] MP;
        MP = new double[4];
        //gyroTurn(angle);
        ElapsedTime runTime = new ElapsedTime();
        while (runTime.time(TimeUnit.MILLISECONDS) < time) {
            float theta = angle;
            MP[0] = (Math.sin(theta) + Math.cos(theta)) * power;
            MP[1] = (Math.sin(theta) - Math.cos(theta)) * power;
            MP[2] = (Math.sin(theta) - Math.cos(theta)) * power;
            MP[3] = (Math.sin(theta) + Math.cos(theta)) * power;
            setMotor_bl(PDoutput(angle) + MP[0]);
            setMotor_fl(PDoutput(angle) + MP[1]);
            setMotor_br(PDoutput(angle) + MP[2]);
            setMotor_fr(PDoutput(angle) + MP[3]);
        }
    }
    public void gyroBadTurn(float degrees){

        float Kp = (float) 0.008;
        while (Gerror <= 5){
            Gerror = degrees - getHeading();
            PDout = Kp * Gerror;
            setMotor_bl(-PDout);
            setMotor_fl(-PDout);
            setMotor_br(PDout);
            setMotor_fr(PDout);
            if (Gerror <= 5) {
                setMotor_bl(0);
                setMotor_br(0);
                setMotor_fr(0);
                setMotor_fl(0);
                break;
            }
        }
    }
    public void FrontandBack (double power, long time, float angle) {
        ElapsedTime WaitTime = new ElapsedTime();
        while (WaitTime.time(TimeUnit.MILLISECONDS) < time){
            setMotor_bl(power - PDoutput(angle));
            setMotor_fl(power - PDoutput(angle));
            setMotor_fr(power + PDoutput(angle));
            setMotor_br(power + PDoutput(angle));
        }
    }
    public void SidetoSide (double power, long time, float angle) {
        ElapsedTime WaitTime = new ElapsedTime();
        while (WaitTime.time(TimeUnit.MILLISECONDS) < time){
            setMotor_bl(-power - PDoutput(angle));
            setMotor_fl(power - PDoutput(angle));
            setMotor_fr(-power + PDoutput(angle));
            setMotor_br(power + PDoutput(angle));
            //going right is positive
        }
    }
    public void gyroTurn(float degrees){

        float Kp = (float) 0.008;
        float Kd = (float) 0.0001;
        while (true){
            deltaTime = System.nanoTime() - preTime;
            Gerror = degrees - getHeading();
            PDout = (Kp * Gerror) + (Kd * (Gerror/deltaTime));
            setMotor_bl(-PDout);
            setMotor_fl(-PDout);
            setMotor_br(PDout);
            setMotor_fr(PDout);
            preTime = currentTime;
            if (Math.abs(Gerror) <= 5) {
                    setMotor_bl(0);
                    setMotor_br(0);
                    setMotor_fr(0);
                    setMotor_fl(0);
                    break;
            }
        }
    }
    public float PDoutput(float degrees){
        float Kp = (float) 0.008;
        float Kd = (float) 0.0001;
        deltaTime = System.nanoTime() - preTime;
        Gerror = degrees - getHeading();
        PDout = (Kp * Gerror) + (Kd * (Gerror/deltaTime));
        preTime = currentTime;
        return PDout;
    }

    public void toDistance(float position){

            Derror = (float) (distance.getDistance(DistanceUnit.MM) - position);
            while (Math.abs(Derror) >= 4) {
                float Kp = (float) 0.0001;
                Derror = (float) (distance.getDistance(DistanceUnit.MM) - position);
                setMotor_bl(Derror * Kp);
                setMotor_fl(Derror * Kp);
                setMotor_br(-Derror * Kp);
                setMotor_fr(-Derror * Kp);
                if (Math.abs(Derror) <= 4) {
                    setMotor_bl(0);
                    setMotor_fl(0);
                    setMotor_br(0);
                    setMotor_fr(0);
                    break;
                }
            }
//there are 26 lightning bolts on the robot
    }
    public double getDistance(){
        return distance.getDistance(DistanceUnit.MM);
    }
    public int[] getColor() {

        int red = color.red();
        int green = color.green();
        int blue = color.blue();
        int[] colors = new int[] {red, green, blue};
        return colors;
    }
    public void cry()
    {
        tears++;
    }

}


