package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutoOp extends LinearOpMode {
    private DcMotor leftfront = null;
    private DcMotor leftback = null;
    private DcMotor rightfront = null;
    private DcMotor rightback = null;
    protected DcMotor arm = null;
    protected DcMotor modelX = null;
    protected Servo finger1 = null;
    protected Servo finger2 = null;
    protected Servo claw = null;
    
    protected BNO055IMU imu = null;
    private double zero_heading = 0.0;
    private Orientation angles = null;
    
    private ElapsedTime runtime = new ElapsedTime();
    private Telemetry telemetry_ = null;
    
    private double speed = 0.55*1.4;
    private double quit_speed = 2.0;
    private double strafe_spd = 1.2;
    private double turn90 = 0.73;//73, 75
    private double armSpd = 0.5;
    private double armPower = 0.25;
    private double armStatic = 0.3;
    protected double wheelPower = 0.8;
    private double margin = 1;
    private double ticksPerTile = 1120/4/3.141592653589793238462643383279502884*24*(4.0/3.0); // 1120 ticks/rev * 1/4pi rev/in * 24 in/tile * gear_ratio
    private double turningCorrection = 0.001;
    
    /** Initialization **/
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, boolean has_arm, boolean has_claw){
        leftfront = hardwareMap.get(DcMotor.class, "leftDrive_0");
        leftback = hardwareMap.get(DcMotor.class, "leftDrive_1");
        rightfront = hardwareMap.get(DcMotor.class, "rightDrive_0");
        rightback = hardwareMap.get(DcMotor.class, "rightDrive_1");
        modelX = hardwareMap.get(DcMotor.class, "ModelX");
        finger1 = hardwareMap.get(Servo.class, "finger1");
        finger2 = hardwareMap.get(Servo.class, "finger2");
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.FORWARD);
        modelX.setDirection(DcMotor.Direction.FORWARD);
        telemetry_ = telemetry;
        if(has_arm){
            arm = hardwareMap.get(DcMotor.class, "arm");
            arm.setDirection(DcMotor.Direction.REVERSE);
        }
        if(has_claw){
            claw = hardwareMap.get(Servo.class, "claw");
        }
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zero_heading = angles.firstAngle;
    }
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, boolean has_arm){
        initialize(hardwareMap, telemetry, has_arm, has_arm);
    }
    
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        initialize(hardwareMap, telemetry, true, true);
    }
    
    /** Gyrometer Access **/
    public double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading + 360 + margin) % 360.0;
    }
    public double getNegAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading - 720 - margin) % 360.0;
    }
    public double getSmAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading + 360) % 360.0 - 180.0;
    }
    @Deprecated
    public void zeroHeading(){
        //double off = getAngle();
        //zero_heading += off;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zero_heading = angles.firstAngle;
    }
    
    /** Low-Level Conviniences **/
    public void leftWheels(double spd){
        leftfront.setPower(spd);
        leftback.setPower(spd);
    }
    public void rightWheels(double spd){
        rightfront.setPower(spd*0.95);
        rightback.setPower(spd*0.85);
    }
    
    /** Speed-Based Movement **/
    public void driveSpd(double spd){
        leftWheels(spd);
        rightWheels(spd);
    }
    public void turnRightSpd(double spd){
        leftWheels(spd);
        rightWheels(-spd);
    }
    public void turnLeftSpd(double spd){
        leftWheels(-spd);
        rightWheels(spd);
    }
    public void strafeLeftSpd(double spd, double corr){
        leftfront.setPower(-spd+corr);
        leftback.setPower(spd+corr);
        rightfront.setPower(spd-corr);
        rightback.setPower(-spd-corr);
    }
    public void strafeLeftSpd(double spd){
        strafeLeftSpd(spd, 0.0);
    }
    public void strafeRightSpd(double spd, double corr){
        leftfront.setPower(spd+corr);
        leftback.setPower(-spd+corr);
        rightfront.setPower(-spd-corr);
        rightback.setPower(spd-corr);
    }
    public void strafeRightSpd(double spd){
        strafeRightSpd(spd, 0.0);
    }
    
    /** Encoder-Based Movement **/
    public void move(double y_tiles, double x_tiles, String name){
        resetWheels();
        leftfront.setTargetPosition((int) ((y_tiles+x_tiles)*ticksPerTile));
        leftback.setTargetPosition((int) ((y_tiles-x_tiles)*ticksPerTile));
        rightfront.setTargetPosition((int) ((y_tiles-x_tiles)*ticksPerTile));
        rightback.setTargetPosition((int) ((y_tiles+x_tiles)*ticksPerTile));
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setPower(wheelPower);
        leftback.setPower(wheelPower);
        rightfront.setPower(wheelPower);
        rightback.setPower(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (leftfront.isBusy() || rightfront.isBusy() || leftback.isBusy() || rightback.isBusy()) && runtime.seconds() < Math.sqrt(y_tiles*y_tiles+x_tiles*x_tiles)*quit_speed){
            telemetry_.addData("Path", "%s: %s lf, %s rf, %s lb, %s rb", name, leftfront.getCurrentPosition(), rightfront.getCurrentPosition(), leftback.getCurrentPosition(), rightback.getCurrentPosition());
            telemetry_.update();
        }
        leftfront.setPower(0.0);
        leftback.setPower(0.0);
        rightfront.setPower(0.0);
        rightback.setPower(0.0);
        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void move(double y_tiles, double x_tiles){
        move(y_tiles, x_tiles, "Moving");
    }
    public void strafeLeft_enc(double tiles){
        move(0.0, -tiles, "Strafing Left");
    }
    public void strafeRight_enc(double tiles){
        move(0.0, tiles, "Strafing Right");
    }
    public void forward_enc(double tiles){
        move(tiles, 0.0, "Forward");
    }
    public void backward_enc(double tiles){
        move(-tiles, 0.0, "Backward");
    }
    
    /** Encoder Utilities **/
    public void resetWheels(){
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double[] getVector(){
        double[] offset = new double[2];
        double lf = leftfront.getCurrentPosition() / ticksPerTile;
        double lb = leftback.getCurrentPosition() / ticksPerTile;
        double rf = rightfront.getCurrentPosition() / ticksPerTile;
        double rb = rightback.getCurrentPosition() / ticksPerTile;
        offset[0] = (lf + lb + rf + rb) / 4;
        offset[1] = (lf + lb + rf + rb) / 4;
        return offset;
    }
    
    /** Gyroscope-Based Rotation **/
    public double left_gyro(double quarters){
        if(quarters >= 2){
            quarters -= left_gyro(quarters-2);
        }
        double a = 0.0;
        leftWheels(-wheelPower);
        rightWheels(wheelPower);
        runtime.reset();
        while(opModeIsActive() && ((a=getAngle()) < 90*quarters+margin)){
            telemetry_.addData("Path", "Turning Left: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        leftWheels(wheelPower*0.2);
        rightWheels(-wheelPower*0.2);
        while(opModeIsActive() && ((a=getAngle()) > 90*quarters+margin)){
            telemetry_.addData("Path", "Turning Left: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        stopWheels();
        runtime.reset();
        zero_heading = (zero_heading + 90.0) % 360.0;
        return quarters;
    }
    public void right_gyro(double quarters){
        if(quarters >= 4){
            right(quarters-2);
            quarters -= 2;
        }
        double a = 0.0;
        leftWheels(wheelPower);
        rightWheels(-wheelPower);
        runtime.reset();
        while(opModeIsActive() && ((a=getNegAngle()) > -90*quarters-margin)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        leftWheels(-wheelPower*0.2);
        rightWheels(wheelPower*0.2);
        while(opModeIsActive() && ((a=getNegAngle()) < -90*quarters-margin)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        stopWheels();
        runtime.reset();
        zero_heading = (zero_heading - 90.0) % 360.0;
    }
    
    /** Time-Based Movement (Backwards-Compatibility) **/
    @Deprecated
    public void strafeLeft_time(double tiles){
        zeroHeading();
        strafeLeftSpd(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * strafe_spd / wheelPower)){
            telemetry_.addData("Path", "Strafing Left: %2.5f S Elapsed", runtime.seconds());
            strafeLeftSpd(wheelPower, getSmAngle()*turningCorrection);
        }
        stopWheels();
    }
    @Deprecated
    public void strafeRight_time(double tiles){
        zeroHeading();
        strafeRightSpd(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * strafe_spd / wheelPower)){
            telemetry_.addData("Path", "Strafing Right: %2.5f S Elapsed", runtime.seconds());
            strafeRightSpd(wheelPower, getSmAngle()*turningCorrection);
        }
        stopWheels();
    }
    @Deprecated
    public void forward_time(double tiles){
        leftWheels(wheelPower);
        rightWheels(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * speed / wheelPower)){
            telemetry_.addData("Path", "Forward: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    @Deprecated
    public void backward_time(double tiles){
        leftWheels(-wheelPower);
        rightWheels(-wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * speed / wheelPower)){
            telemetry_.addData("Path", "Backward: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    @Deprecated
    public void left_time(double quarters){
        leftWheels(-wheelPower);
        rightWheels(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < quarters * turn90 / wheelPower)){
            telemetry_.addData("Path", "Turning Left: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    public void right_time(double quarters){
        leftWheels(wheelPower);
        rightWheels(-wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < quarters * turn90 / wheelPower)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), getAngle());
            telemetry_.update();
        }
        stopWheels();
    }
    
    /** Movement Interface **/
    public void strafeLeft(double tiles){
        strafeLeft_enc(tiles);
    }
    public void strafeRight(double tiles){
        strafeRight_enc(tiles);
    }
    public void forward(double tiles){
        forward_enc(tiles);
    }
    public void backward(double tiles){
        backward_enc(tiles);
    }
    public void left(double quarters){
        left_gyro(quarters);
    }
    public void right(double quarters){
        right_gyro(quarters);
    }
    
    /** Mechanisms **/
    public void armUp(double pct){
        arm.setPower(armStatic+armPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < pct * armSpd)){
            telemetry_.addData("Path", "Raising arm: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        arm.setPower(armStatic);
    }
    public void armUp(){
        armUp(1.0);
    }
    public void armDown(double pct){
        arm.setPower(armStatic-armPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < pct * armSpd)){
            telemetry_.addData("Path", "Lowering arm: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        arm.setPower(armStatic);
    }
    public void armDown(){
        armDown(1.0);
    }
    public void fingerUp(){
        finger1.setPosition(0.0);
        finger2.setPosition(1.0);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.4){
            telemetry_.addData("Path", "Opening Fingers: %2.3f S elapsed", runtime.seconds());
            telemetry_.update();
        }
    }
    public void fingerDown(){
        finger1.setPosition(0.85);
        finger2.setPosition(0.1);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.4){
            telemetry_.addData("Path", "Closing Fingers: %2.3f S elapsed", runtime.seconds());
            telemetry_.update();
        }
    }
    public void openClaw(){
        claw.setPosition(1.0);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5){
            telemetry_.addData("Path", "Opening Claw: %2.3f S elapsed", runtime.seconds());
            telemetry_.update();
        }
    }
    public void closeClaw(){
        claw.setPosition(0.3);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5){
            telemetry_.addData("Path", "Closing Claw: %2.3f S elapsed", runtime.seconds());
            telemetry_.update();
        }
    }
    public void modelXUp(){
        modelX.setPower(0.5);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 0.1)){
            telemetry_.addData("Path", "Raising ModelX: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        arm.setPower(0.0);
    }
    public void modelXDown(){
        modelX.setPower(-0.5);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 0.1)){
            telemetry_.addData("Path", "Lowering ModelX: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        arm.setPower(-0.1);
    }
    
    /** Miscelaneous Utilities **/
    public void pause(double seconds){
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < seconds)){
            telemetry_.addData("Path", "Wating: %2.5f / %2.5f S", runtime.seconds(), seconds);
            telemetry_.update();
        }
    }
    @Deprecated
    public void stopMotion(){
        stopWheels();
    }
    public void stopWheels(){
        leftWheels(0.0);
        rightWheels(0.0);
    }
}
