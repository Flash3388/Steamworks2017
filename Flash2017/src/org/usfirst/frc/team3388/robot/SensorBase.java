package org.usfirst.frc.team3388.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.flash3388.flashlib.cams.Camera;
import edu.flash3388.flashlib.cams.CvMultiCamera;
import edu.flash3388.flashlib.robot.devices.AMT10Encoder;
import edu.flash3388.flashlib.robot.devices.DoubleDataSource;
import edu.flash3388.flashlib.robot.devices.Gyro;
import edu.flash3388.flashlib.robot.devices.RangeFinder;
import edu.flash3388.flashlib.robot.flashboard.DashboardCamViewSelector;
import edu.wpi.first.wpilibj.Ultrasonic;

public class SensorBase {
	
	public static enum SonicMode{
		Avg, Right, Left
	}
	public static final int SONIC_RIGHT = 0, SONIC_LEFT = 1, CAM_COMPRESSION_QUALITY = 10,
			 CAM_RESOLUTION_WIDTH = 260, CAM_RESOLUTION_HEIGHT = 195;
	
	public static final double ULTRA_OFFSET_ROBOT = 21.0,
							   BOILER_OPENNING_OFFSET = 0.0;//40.7;
	
	private AMT10Encoder encoder;
	private Ultrasonic[] sonics;
	private Gyro gyro, gyroProp;
	private AHRS navx;
	private Camera camera;
	private DashboardCamViewSelector camSelector;
	private SonicMode sonicMode = SonicMode.Avg;
	private RangeFinder finderR, finderL;
	
	private DoubleDataSource.VarDataSource gyroAngle, encoderRate, targetDistance;
	private DoubleDataSource.VarDataSource[] sonicRanges;
	
	public SensorBase(){
		if(Robot.USE_FLASHBOARD){
			CvMultiCamera camera = new CvMultiCamera(0, CAM_RESOLUTION_WIDTH, CAM_RESOLUTION_HEIGHT,
													CAM_COMPRESSION_QUALITY);
			camera.setFPS(20);
			camSelector = new DashboardCamViewSelector();
			camera.setSelector(camSelector);
			this.camera = camera;
		}
		
		gyroAngle = new DoubleDataSource.VarDataSource();
		encoderRate = new DoubleDataSource.VarDataSource();
		targetDistance = new DoubleDataSource.VarDataSource();
		sonics = new Ultrasonic[Robot.s_sonics.length];
		sonicRanges = new DoubleDataSource.VarDataSource[Robot.s_sonics.length];
		for (int i = 0; i < Robot.s_sonics.length; i++) {
			sonicRanges[i] = new DoubleDataSource.VarDataSource();
			sonics[i] = new Ultrasonic(Robot.s_sonics[i][0], Robot.s_sonics[i][1]);
		}
		if(sonics.length > 0)
			sonics[0].setAutomaticMode(true);
		
		encoder = new AMT10Encoder(Robot.s_encoder);
		encoder.setAutomaticUpdate(false);
		
		navx = new AHRS(Robot.s_navx_port_serial);
		gyro = new Gyro(){
			@Override
			public double getAngle() {
				return navx.getAngle();
			}
		};
		gyroProp = new Gyro(){
			@Override
			public double getAngle() {
				return getGyroAngle();
			}
		};
		
		finderR = new RangeFinder(){
			@Override
			public double getRangeCM() {
				return getSonicRange(SONIC_RIGHT);
			}
			@Override
			public void ping() {}
		};
		finderL = new RangeFinder(){
			@Override
			public double getRangeCM() {
				return getSonicRange(SONIC_LEFT);
			}
			@Override
			public void ping() {}
		};
	}
	
	public Camera getCamera(){
		return camera;
	}
	public void switchCamera(){
		int select = camSelector.getCameraIndex() + 1;
		if(select >= Robot.CAM_COUNT)
			select = 0;
		camSelector.select(select);
	}
	
	public double getEncoderRateRaw(){
		return encoder.getRate();
	}
	public double getRangeRaw(int sonic){
		return sonics[sonic].getRangeMM() / 10.0;
	}
	public double getGyroAngleRaw(){
		return gyro.getAngle();
	}
	public Gyro getGyroProp(){
		return gyroProp;
	}
	public AHRS getNavx(){
		return navx;
	}
	public RangeFinder getRangeFinderLeft(){
		return finderL;
	}
	public RangeFinder getRangeFinderRight(){
		return finderR;
	}
	public DoubleDataSource getEncoderRateProperty(){
		return encoderRate;
	}
	public double getEncoderRate(){
		return encoderRate.get();
	}
	public DoubleDataSource getGyroAngleProperty(){
		return gyroAngle;
	}
	public double getGyroAngle(){
		return gyroAngle.get();
	}
	public DoubleDataSource getSonicRangeProperty(int sonic){
		return sonicRanges[sonic];
	}
	public double getSonicRange(int sonic){
		return sonicRanges[sonic].get();
	}
	public int getUltrasonicCount(){
		return Robot.s_sonics.length;
	}
	public double getTargetDistance(){
		return targetDistance.get();
	}
	public DoubleDataSource getTargetDistanceProperty(){
		return targetDistance;
	}
	public void setSonicMode(SonicMode mode){
		sonicMode = mode;
	}
	public void setSonicModeRight(){
		setSonicMode(SonicMode.Right);
	}
	public void setSonicModeLeft(){
		setSonicMode(SonicMode.Left);
	}
	public void setSonicModeAvg(){
		setSonicMode(SonicMode.Avg);
	}
	public SonicMode getSonicMode(){
		return sonicMode;
	}
	public void update(){
		for (int i = 0; i < Robot.s_sonics.length; i++)
			sonicRanges[i].set(getRangeRaw(i)+ULTRA_OFFSET_ROBOT);
		gyroAngle.set(getGyroAngleRaw());
		encoderRate.set(getEncoderRateRaw());
		
		if(sonicRanges.length < 1){
			targetDistance.set(-1);
			return;
		}
		
		targetDistance.set(getRangeTarget()+BOILER_OPENNING_OFFSET);
	}
	
	private double getRangeTarget(){
		switch(sonicMode){
		case Left:
			return getSonicRange(SONIC_LEFT);
		case Right:
			return getSonicRange(SONIC_RIGHT);
		default:
			double r = 0;
			for (int i = 0; i < sonicRanges.length; i++) 
				r += getSonicRange(i);
			r /= sonicRanges.length;
			return r;
		}
	}
}
