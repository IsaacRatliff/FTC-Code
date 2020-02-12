package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous

public abstract class VuforiaOp extends AutoOp {
    private VuforiaLocalizer localizer;
    private VuforiaLocalizer.Parameters params;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;
    
    protected OpenGLMatrix lastKnownLocation;
    protected OpenGLMatrix phoneLocation;
    public double red_off;  // x; positive when skystone is right of the phone
    public double blue_off;  // y; negative if skystone in viewig field
    public double green_off;  // z; skystones should start on the ground.
    public float[] raw_offset;  // green, red, blue; zxy
    
    protected static final String VUFORIA_KEY = "AeTCFCP/////AAABmfHb70GlwET4rTC9SGWFDMVfC6cZN+OykKIBJrcpYIVTGzbWZ11w8AoTq6mrdM68JbbKzA3C/+v46Jo8+pcraqzQ5QPOv23oHfxgDMKAlbxYGixALMMcSsE8Anv2NeBhMd+zLoRTYAnU8YQ7S35RYmeFRufFLUHF+Tkps1tZoWOxd6yyzuw10nwVVlZ1F4JhHVEHXoFsvUDXVxHENaFOxcy244tY+AM/qM4U9jD1RXLMPrp+ZLWmG/Bt8ja1avZHE0EWK5DKaKD0uWrT9A9WwT8w+aNjNxJKOdL9mq/fhZwDRPS6Q7flBwIQvVbq6bTXl4erMprFkPXo4yaZJvJo8ag+V5mEVe/cQ2N0A7qNtb3f";
    
    public void initVision(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");
        localizer = ClassFactory.createVuforiaLocalizer(params);
        visionTargets = localizer.loadTrackablesFromAsset("Skystone");
        target = visionTargets.get(0);
        target.setName("skystone");
        //target.setLocation(createMatrix(0, 0, 0, 0, 0, 0));
        //phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        //listener.setPhoneInformation(phoneLocation, params.cameraDirection);
        //lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
        green_off = 0;
        red_off = 0;
        blue_off = 0;
        raw_offset = null;
    }
    
    public void activateVision(){
        visionTargets.activate();
    }
    
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z)
            .multiplied(Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }
    
    public void calcOffset(){
        OpenGLMatrix pose = listener.getPose();
        if(pose == null){
            raw_offset = null;
            return;
        }
        VectorF trans = pose.getTranslation();
        float[] off = {trans.get(0), trans.get(1), trans.get(2)};
        // smooth the readings
        green_off = 0.9*green_off + 0.1*off[0];
        red_off = 0.9*red_off + 0.1*off[1];
        blue_off = 0.9*blue_off + 0.1*off[2];
        raw_offset = off;
    }
    
    public void toSkystone(double threshold, double nullspd, double factor){
        while(opModeIsActive() && (raw_offset == null || red_off < -threshold || red_off > threshold)){
            telemetry.addData("Path", "seeking skystone");
            calcOffset();
            if(raw_offset == null){
                strafeRightSpd(nullspd);
                telemetry.addData("Skystone", "missing");
            }
            else{
                strafeLeftSpd(red_off/factor);
                telemetry.addData("Skystone", "%.0f", red_off);
            }
            telemetry.update();
        }
    }
}
