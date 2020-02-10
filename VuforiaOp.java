package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
    
    protected static final String VUFORIA_KEY = "";
    
    public void initVision(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        localizer = ClassFactory.createVuforiaLocalizer(params);
        visionTargets = localizer.loadTrackablesFromAsset("Skystone");
        target = visionTargets.get(0);
        target.setName("skystone");
        target.setLocation(createMatrix(0, 0, 0, 0, 0, 0));
        phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, params.cameraDirection);
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
        visionTargets.activate();
    }
    
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z)
            .multiplied(Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }
    
    public float[] getOffset(){
        OpenGLMatrix latestLocation = null;
        float[] r = {0, 0};
        return r;
    }
}
