package org.usfirst.frc.team3388.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.flash3388.flashlib.math.PolynomialInterpolation;
import edu.flash3388.flashlib.robot.Action;
import edu.flash3388.flashlib.robot.ActionGroup;
import edu.flash3388.flashlib.robot.FlashRoboUtil;
import edu.flash3388.flashlib.robot.InstantAction;
import edu.flash3388.flashlib.robot.TimedAction;
import edu.flash3388.flashlib.robot.actions.AngleRotateAction;
import edu.flash3388.flashlib.robot.actions.RangeApproach;
import edu.flash3388.flashlib.robot.actions.VisionRotate;
import edu.flash3388.flashlib.robot.devices.BooleanDataSource;
import edu.flash3388.flashlib.flashboard.BooleanProperty;
import edu.flash3388.flashlib.flashboard.DashboardChooser;
import edu.flash3388.flashlib.flashboard.DashboardInput;
import edu.flash3388.flashlib.flashboard.DoubleProperty;
import edu.flash3388.flashlib.flashboard.Flashboard;
import edu.flash3388.flashlib.flashboard.Tester;
import edu.flash3388.flashlib.robot.hid.XboxController;
import edu.flash3388.flashlib.robot.rio.FlashRio;
import edu.flash3388.flashlib.robot.rio.RioControllers;
import edu.flash3388.flashlib.robot.rio.RioControllers.ControllerType;
import edu.flash3388.flashlib.robot.systems.MecanumDrive;
import edu.flash3388.flashlib.robot.systems.SingleMotorSystem;
import edu.flash3388.flashlib.robot.System;
import edu.flash3388.flashlib.robot.SystemAction;
import edu.flash3388.flashlib.util.FlashUtil;
import edu.flash3388.flashlib.util.Log;
import edu.flash3388.flashlib.vision.RemoteVision;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends FlashRio{
	
	public static final double VISION_SPEED = 0.35, MAX_RPM = 5300, LERP_MARGIN = 20,
			COLLECTOR_BURST_TIMEOUT = 1000, COLLECTOR_AUTO_FIRE_SPEED = 0.5, 
			FEEDER_AUTO_FIRE_SPEED = 0.95, FEEDER_SPEED = 0.95,
			CLIMBER_SPEED = 1, CLIMBER_SPEED_CATCH = 0.5,
			SHOOTER_VOLTAGE_CHANGE_VALUE = 0.1, SHOOTER_VBUS_CHANGE_VALUE = 0.005,
			SHOOTER_MAX_DISTANCE = 250, SHOOTER_MIN_DISTANCE = 80,
			SHOOTER_RPM_RANGE_STABLE = 100, SHOOTER_FEEDER_ACTIVATE_WAIT = 450,
			AUTO_FIRE_TIME = 9.5, AUTO_RIGHT_TIME = 2.5, AUTO_RIGHT_SPEED = 1,
			AUTO_BACK_TIME = 2.1;
	
	public static final int CAM_COUNT = 2;
	public static final int m_fr = 2, m_fl = 1, m_rr = 4, m_rl = 3, m_shooter = 6, m_collector = 7, m_collector2 = 8, m_climber = 5, 
			m_feeder = 9,
			s_encoder = 9;
	public static final String e_lerp_file = "/home/lvuser/interpolation.ini";
	public static final SerialPort.Port s_navx_port_serial = SerialPort.Port.kUSB;
	public static final I2C.Port s_navx_port_i2c = I2C.Port.kOnboard;
	public static final int[][] s_sonics = {
			{0, 1}, {4, 3}
	};
	public static final boolean USE_FLASHBOARD = true,
						        IN_COMPETITION = true,
						        IN_PRESENTATION = false,
				        		ASSISTANT_CONTROL = true;
	
	static SensorBase sensorBase;
	static Log robotLog;
	
	boolean done = false, lockSaved = false, autoShootInitialized = false, rpmStabilized = false;
	double lockedSpeed = 0, speedDifferance = 0, lastSpeed, visionSpeed = VISION_SPEED, feederdSpeed = FEEDER_SPEED;
	double feederSpeed, shooterSpeed, climberSpeed, collectorSpeed, climbSpeed = CLIMBER_SPEED;
	long lastAutoFeed, autoShooterStartTime;
	
	PolynomialInterpolation lerp;
	
	CANTalon frontL, frontR, rearL, rearR, shooterController;
	VictorSP feeder, collectorVic;
	MecanumDrive driveTrain;
	SingleMotorSystem shooter, collector, climber;
	
	XboxController xbox, xboxAssist;
	
	InstantAction switchBrake, switchStabilizer, switchSensitivity, switchCam, switchManualShooter, 
				switchRounded, switchManualDistance, speedClimberAction;
	Action rotateAndFire, auto, climbAction, collectAction, fireAction, timedFireAction, moveRight;
	VisionRotate rotateV;
	AngleRotateAction rotate;
	ActionGroup autoFire, autoFireV, autoFireC, lineCrosser, hopperAutoBlue, hopperAutoRed, autoFireS, autoFireB,
				hopperCollectRed, hopperCollectBlue;
	
	BooleanDataSource.VarDataSource sensitivity, shooterLock, useFeeder, manualShooter, manualDistance, assistantControl;
	DoubleProperty dEncoderValue, dGyroAngle, dTarget;
	BooleanProperty bStabilize, bSensitivity, bBrakemode, bShooterLock, bManualShooter, bAssistantControl;
	DashboardChooser<Action> cAutonomous;
	DashboardInput iManualDistance;
	SendableChooser<Action> chooser;
	
	RemoteVision visionO;
	
	Alliance currentAlliance;
	
	//------------------------------------------------
	//---------------Initialization-------------------
	//------------------------------------------------
	static{
		//Flashboard.setProtocolTcp();
	}
	
	@Override
	protected void initRobot() {
		robotLog = new Log("Robot");
		robotLog.log("FLASHBOARD: "+USE_FLASHBOARD+" | COMPETITION: "+IN_COMPETITION+" | PRESENTATION: "+IN_PRESENTATION, "Robot");
		
		if(IN_COMPETITION || IN_PRESENTATION){
			robotLog.disable();
			FlashUtil.getLog().disable();
			getPowerLog().disable();
		}
		
		initSystems();
		initVision();
		initActions();
		initDashboard();
		initHid();
		
		lerp = new PolynomialInterpolation(LERP_MARGIN);
		lerp.readFromFile(e_lerp_file);
		lerp.updateValues();
		FlashRoboUtil.setVoltageSource(()->m_ds.getBatteryVoltage());
		robotLog.save();
	}
	
	private void initVision(){
		if(USE_FLASHBOARD){
			visionO = Flashboard.getVision();
			visionO.enableRemoteParameters(true);
			visionO.start();
		}
	}
	private void initSystems(){
		frontL = new CANTalon(m_fl);
		frontR = new CANTalon(m_fr);
		rearL = new CANTalon(m_rl);
		rearR = new CANTalon(m_rr);
		driveTrain = new MecanumDrive(new RioControllers(frontR), new RioControllers(rearR), 
				new RioControllers(frontL), new RioControllers(rearL));
		driveTrain.enableVoltageScaling(true);
		
		shooterController = (CANTalon) RioControllers.controllerFromType(ControllerType.CANTalon, m_shooter);
		RioControllers conRio = new RioControllers(shooterController);
		shooter = new SingleMotorSystem(conRio);
		shooter.setName("Shooter");
		shooter.setInverted(true);
		collectorVic = new VictorSP(m_collector2);
		SpeedController conCol = RioControllers.controllerFromType(ControllerType.CANTalon, m_collector);
		collector = new SingleMotorSystem(
				new RioControllers(conCol, collectorVic));
		collector.setName("Collector");
		collector.setInverted(true);
		climber = new SingleMotorSystem(
				new RioControllers(RioControllers.controllerFromType(ControllerType.CANTalon, m_climber)));
		climber.setName("Climber");
		climber.setDefaultSpeed(CLIMBER_SPEED);
		feeder = new VictorSP(m_feeder);
		feeder.setInverted(true);
		
		if(USE_FLASHBOARD && !IN_COMPETITION && !IN_PRESENTATION){
			Tester.init();
			Tester.getInstance().registerMotors(frontL, frontR, rearL, rearR);
		}
		
		sensorBase = new SensorBase();
		driveTrain.getStabilzer().setGyro(sensorBase.getGyroProp());
		driveTrain.getStabilzer().setOffsetMargin(5);
		driveTrain.getStabilzer().setBaseSpeed(0.5);
	}
	private void initActions(){
		sensitivity = new BooleanDataSource.VarDataSource();
		useFeeder = new BooleanDataSource.VarDataSource(true);
		shooterLock = new BooleanDataSource.VarDataSource();
		manualShooter = new BooleanDataSource.VarDataSource();
		manualDistance = new BooleanDataSource.VarDataSource();
		assistantControl = new BooleanDataSource.VarDataSource(ASSISTANT_CONTROL);
		
		rotateV = new VisionRotate(driveTrain, visionO, VISION_SPEED);
		rotateV.setPixelMargin(8);
		rotateV.setMinSpeed(0.35);
		rotateV.setCenterTimeout(200);
		rotateV.setLossTimeout(1500);
		rotateV.enableVoltageScaling(true);
		
		fireAction = new SystemAction(shooter, new Action(){
			@Override
			protected void initialize() {
				setFireMode(false);
				useFeeder.set(true);
			}
			@Override
			protected void execute() {
				automaticShooter();
				shooterController.set(shooterSpeed);
			}
			@Override
			protected void end() {
				stopShooter();
				setFireMode(true);
			}
		});
		timedFireAction = new TimedAction(fireAction, AUTO_FIRE_TIME);
		
		rotateAndFire = new ActionGroup(rotateV, fireAction);
		
		climbAction = climber.FORWARD_ACTION;
		collectAction = collector.FORWARD_ACTION;
		
		switchBrake = new InstantAction(){
			@Override
			protected void execute() {
				driveTrain.enableBrakeMode(!driveTrain.inBrakeMode());
				robotLog.log("Brakemode: "+driveTrain.inBrakeMode());
			}
		};
		switchStabilizer = new InstantAction(){
			@Override
			protected void execute() {
				driveTrain.enableStabilizing(!driveTrain.isStabilizing());
				robotLog.log("Stabilizing: "+driveTrain.isStabilizing(), "Drive Train");
			}
		};
		switchRounded = new InstantAction(){
			@Override
			protected void execute() {
				driveTrain.setAngleRounding(driveTrain.getAngleRounding() == 0? 45 : 0);
				robotLog.log("Rounded: "+(driveTrain.getAngleRounding() != 0), "Drive Train");
			}
		};
		switchSensitivity = new InstantAction(){
			@Override
			protected void execute() {
				sensitivity.switchValue();
				robotLog.log("Sensitivity: "+sensitivity.get(), "Drive Train");
			}
		};
		switchCam = new InstantAction(){
			@Override
			protected void execute() {
				sensorBase.switchCamera();
			}
		};
		switchManualDistance = new InstantAction(){
			@Override
			protected void execute() {
				manualDistance.switchValue();
				robotLog.log("Manual Distance: "+manualDistance.get(), "Firing System");
			}
		};
		
		moveRight = new SystemAction(driveTrain, new Action(){
			@Override
			protected void initialize() {
			}
			@Override
			protected void execute() {
				if(currentAlliance == Alliance.Blue)
					driveTrain.left(AUTO_RIGHT_SPEED);//right ?? reversed   
				else driveTrain.right(AUTO_RIGHT_SPEED);
			}
			@Override
			protected void end() {
				driveTrain.stop();
				driveTrain.enableBrakeMode(false);
			}
		});
		Action moveForward = new SystemAction(driveTrain, new Action(){
			@Override
			protected void execute() {
				driveTrain.forward(0.5);
			}
			@Override
			protected void end() {
				driveTrain.stop();
			}
		});
		Action moveBack = new TimedAction(new SystemAction(driveTrain, new Action(){
			@Override
			protected void execute() {
				driveTrain.backward(0.5);
			}
			@Override
			protected void end() {
				driveTrain.stop();
			}
		}), AUTO_BACK_TIME);
		
		RangeApproach app =  new RangeApproach(driveTrain, sensorBase.getRangeFinderRight(), 0.4);
		app.setDistanceThreshold(86);
		app.setMaxSpeed(0.5);
		app.setMinSpeed(0.3);
		app.setDistanceMargin(20);
		app.enableVoltageScaling(true);
		app.setPastTimeout(3);
		RangeApproach app2 =  new RangeApproach(driveTrain, sensorBase.getRangeFinderRight(), 0.4);
		app2.setDistanceThreshold(86);
		app2.setMaxSpeed(0.5);
		app2.setMinSpeed(0.3);
		app2.setDistanceMargin(20);
		app2.enableVoltageScaling(true);
		app2.setPastTimeout(3);
		
		Action collectHopper = new SystemAction(collector, new Action(){
			@Override
			protected void execute() {
				collector.forward(0.3);
			}
			@Override
			protected void end() {
				collector.stop();
			}
		});
		
		HolonomicRotateMove moveHolo = new HolonomicRotateMove(driveTrain, sensorBase.getGyroProp(), 0.6, false);
		moveHolo.setDestAngle(270);
		moveHolo.setMinSpeed(0.45);
		moveHolo.setMaxSpeed(0.6);
		moveHolo.setMaxMisses(1);
		moveHolo.setSpeedY(-0.36);
		HolonomicRotateMove moveHolo2 = new HolonomicRotateMove(driveTrain, sensorBase.getGyroProp(), 0.6, false);
		moveHolo2.setDestAngle(115);
		moveHolo2.setMinSpeed(0.45);
		moveHolo2.setMaxSpeed(0.6);
		moveHolo2.setMaxMisses(1);
		moveHolo2.setSpeedY(0.2);
		HolonomicRotateMove moveHolor = new HolonomicRotateMove(driveTrain, sensorBase.getGyroProp(), 0.6, false);
		moveHolor.setDestAngle(90);
		moveHolor.setMinSpeed(0.45);
		moveHolor.setMaxSpeed(0.6);
		moveHolor.setMaxMisses(1);
		moveHolor.setSpeedY(-0.36);
		HolonomicRotateMove moveHolor2 = new HolonomicRotateMove(driveTrain, sensorBase.getGyroProp(), 0.6, false);
		moveHolor2.setDestAngle(245);
		moveHolor2.setMinSpeed(0.45);
		moveHolor2.setMaxSpeed(0.6);
		moveHolor2.setMaxMisses(1);
		moveHolor2.setSpeedY(0.2);
		
		Action prepShooter = new SystemAction(shooter, new Action(){
			@Override
			protected void execute() {
				shooter.forward(0.7);
			}
			@Override
			protected void end() {
			}
		});
		
		lineCrosser = new ActionGroup()
				.addSequential(new TimedAction(moveForward, 1.0));
		lineCrosser.setName("Line Crosser");
		autoFireC = new ActionGroup()
				.addSequential(switchManualDistance)
				.addSequential(timedFireAction)
				.addSequential(new TimedAction(moveRight, AUTO_RIGHT_TIME));
		autoFireC.setName("Fire and Cross");
		autoFireS = new ActionGroup()
				.addSequential(switchManualDistance)
				.addSequential(timedFireAction)
				.addSequential(switchStabilizer)
				.addSequential(new TimedAction(moveRight, AUTO_RIGHT_TIME))
				.addSequential(switchStabilizer);
		autoFireS.setName("Fire and Cross - Stable");
		autoFireB = new ActionGroup()
				.addSequential(switchManualDistance)
				.addSequential(new TimedAction(fireAction, AUTO_FIRE_TIME - AUTO_BACK_TIME))
				.addSequential(moveBack)
				.addSequential(new TimedAction(moveRight, AUTO_RIGHT_TIME + 2.5));
		autoFireB.setName("Fire and Cross - Other side");
		hopperAutoBlue = new ActionGroup()
				.addParallel(new TimedAction(collectHopper, 1.0))
				.addSequential(app)
				.addSequential(moveHolo)
				.addSequential(new TimedAction(collectHopper, 2.0))
				.addSequential(moveHolo2)
				.addParallel(prepShooter)
				.addWaitAction(0.3)
				.addParallel(rotateV)
				.addSequential(fireAction);
		hopperAutoBlue.setName("Hopper - Blue");
		hopperAutoRed = new ActionGroup()
				.addParallel(new TimedAction(collectHopper, 1.0))
				.addSequential(app)
				.addSequential(moveHolor)
				.addSequential(new TimedAction(collectHopper, 2.0))
				.addSequential(moveHolor2)
				.addParallel(prepShooter)
				.addWaitAction(0.3)
				.addParallel(rotateV)
				.addSequential(fireAction);
		hopperAutoRed.setName("Hopper - Red");
		hopperCollectRed = new ActionGroup()
				.addParallel(new TimedAction(collectHopper, 1.0))
				.addSequential(app)
				.addSequential(moveHolor)
				.addSequential(new TimedAction(collectHopper, 5.0));
		hopperCollectRed.setName("Hopper Collect - Red");
		hopperCollectBlue = new ActionGroup()
				.addParallel(new TimedAction(collectHopper, 1.0))
				.addSequential(app)
				.addSequential(moveHolo)
				.addSequential(new TimedAction(collectHopper, 5.0));
		hopperCollectBlue.setName("Hopper Collect - Blue");
	}
	private void initHid(){		
		InstantAction stop = new InstantAction(){
			@Override
			protected void execute() {
				stopAll();
			}
		};
		InstantAction manShooter = new InstantAction(){
			@Override
			protected void execute() {
				setFireMode(!manualShooter.get());
			}
		};
		InstantAction incSpeed = new InstantAction(){
			@Override
			protected void execute() {
				if(manualShooter.get()){
					if(lockedSpeed + SHOOTER_VBUS_CHANGE_VALUE <= 1)
						lockedSpeed += SHOOTER_VBUS_CHANGE_VALUE;
					else lockedSpeed = 1;
					robotLog.log("LockedSpeed = "+lockedSpeed, "Firing System");
				}else{
					if(shooterSpeed + SHOOTER_VOLTAGE_CHANGE_VALUE <= shooterController.getBusVoltage())
						shooterSpeed += SHOOTER_VOLTAGE_CHANGE_VALUE;
					else shooterSpeed = 0;
					robotLog.log("ShooterSpeed = "+shooterSpeed, "Firing System");
				}
			}
		};
		InstantAction decSpeed = new InstantAction(){
			@Override
			protected void execute() {
				if(manualShooter.get()){
					if(lockedSpeed - SHOOTER_VBUS_CHANGE_VALUE >= 0)
						lockedSpeed -= SHOOTER_VBUS_CHANGE_VALUE;
					else lockedSpeed = 0;
					robotLog.log("LockedSpeed = "+lockedSpeed, "Firing System");
				}else{
					if(shooterSpeed - SHOOTER_VOLTAGE_CHANGE_VALUE >= 0)
						shooterSpeed -= SHOOTER_VOLTAGE_CHANGE_VALUE;
					else shooterSpeed = 0;
					robotLog.log("ShooterSpeed = "+shooterSpeed, "Firing System");
				}
			}
		};
		InstantAction lockSpeed = new InstantAction(){
			@Override
			protected void execute() {
				if(manualShooter.get()){
					shooterLock.set(!shooterLock.get());
					robotLog.log("Shooter Lock: "+shooterLock.get(), "Firing System");
				}else
					autoShootInitialized = false;
			}
		};
		InstantAction switchAssist = new InstantAction(){
			@Override
			protected void execute() {
				if(ASSISTANT_CONTROL){
					assistantControl.switchValue();
					robotLog.log("AssistantControl: "+assistantControl.get(), "Robot");
				}
			}
		};
		
		xbox = new XboxController(2);
		xbox.B.whenPressed(switchAssist);
		xbox.Y.whenPressed(manShooter);
		xbox.A.whenPressed(stop);
		xbox.Start.whenPressed(climbAction);
		xbox.DPad.Up.whenPressed(switchCam);
		//xbox.DPad.Left.whenPressed(switchSensitivity);
		xbox.DPad.Right.whenPressed(switchStabilizer);
		xbox.RB.whenPressed(switchSensitivity);
		xbox.LB.whenPressed(rotateV);
		xbox.X.whenPressed(switchManualDistance);
		xbox.LeftStickButton.whenPressed(lockSpeed);
		xbox.RightStickButton.whenPressed(switchBrake);
		
		if(ASSISTANT_CONTROL){
			xboxAssist = new XboxController(1);
			xboxAssist.LB.whenPressed(lockSpeed);
			xboxAssist.DPad.Down.whenPressed(decSpeed);
			xboxAssist.DPad.Up.whenPressed(incSpeed);
			//xboxAssist.Back.whenPressed(feeder);
			xboxAssist.A.whenPressed(stop);
			xboxAssist.Start.whenPressed(climbAction);
		}
	}
	private void initFlashboard(){
		cAutonomous = new DashboardChooser<Action>("Autonomous Chooser")
				.addDefault("Empty", Action.EMPTY)
				.addOption("LineCross", lineCrosser)
				.addOption("FireC", autoFireC)
				.addOption("FireS", autoFireS)
				.addOption("FireB", autoFireB)
				.addOption("HopperB", hopperAutoBlue)
				.addOption("HopperR", hopperAutoRed)
				.addOption("HopperC-R", hopperCollectRed)
				.addOption("HopperC-B", hopperCollectBlue);
		
		iManualDistance = new DashboardInput("Manual d");
		
		bAssistantControl = new BooleanProperty("Assistant Control", assistantControl);
		bShooterLock = new BooleanProperty("Shooter Lock", shooterLock);
		bSensitivity = new BooleanProperty("Sensitivity", sensitivity);
		bStabilize = new BooleanProperty("Stabilizing", ()->driveTrain.isStabilizing());
		bBrakemode = new BooleanProperty("Brake Mode",()->driveTrain.inBrakeMode());
		bManualShooter = new BooleanProperty("Manual S", manualShooter);
		
		dEncoderValue = new DoubleProperty("Encoder", sensorBase.getEncoderRateProperty());
		dGyroAngle = new DoubleProperty("Gyro", sensorBase.getGyroAngleProperty());
		dTarget = new DoubleProperty("Target", sensorBase.getTargetDistanceProperty());
		
		Flashboard.attach(bAssistantControl, bSensitivity, bStabilize, bBrakemode, bManualShooter, 
						  dEncoderValue, dTarget, dGyroAngle, 
						  cAutonomous,
						  iManualDistance
						  );
		for (int i = 0; i < s_sonics.length; i++) {
			DoubleProperty sonic = new DoubleProperty("Sonic "+(i+1), sensorBase.getSonicRangeProperty(i));
			Flashboard.attach(sonic);
		}
		
		Flashboard.getCameraView().add(sensorBase.getCamera());
		if(Tester.getInstance() != null){
			Flashboard.attach(Tester.getInstance());
		}
		
		Flashboard.start();
	}
	public void initSmartDashboard(){
		chooser = new SendableChooser<Action>();
		chooser.addDefault("nothing", Action.EMPTY);
		chooser.addObject("Cross", lineCrosser);
		chooser.addObject("Fire&Cross", autoFireC);
		chooser.addObject("Fire&Cross b", autoFireB);
		chooser.addObject("Fire&Cross s", autoFireS);
		chooser.addObject("Hopper R", hopperAutoRed);
		chooser.addObject("Hopper B", hopperAutoBlue);
		chooser.addObject("HopperC R", hopperCollectRed);
		chooser.addObject("HopperC B", hopperCollectBlue);
		SmartDashboard.putData("Autotnomous", chooser);
		SmartDashboard.putNumber("ManDis", 0);
		updateSmartDashboard();
	}
	public void initDashboard(){
		if(USE_FLASHBOARD)
			initFlashboard();
		else initSmartDashboard();
	}

	//------------------------------------------------
	//---------------Mode Runs------------------------
	//------------------------------------------------

	
	@Override
	protected void teleopInit() {
		robotLog.save();
		if(auto != null)
			auto.cancel();
		Action a = driveTrain.getCurrentAction();
		if(a != null && a.isRunning())
			a.cancel();
		setDriveSensitivity(0.2);
		
		updateFromFlashboard();
		reset();
		manualShooter.set(true);
		sensorBase.setSonicModeAvg();
		robotLog.save();
	}
	@Override
	protected void teleopPeriodic() {
		FlashRoboUtil.updateHID();
		updateFromFlashboard();
		sensorBase.update();
		updateSmartDashboard();
		
		if(!driveTrain.hasCurrentAction()){
			if(!sensitivity.get()){
				if(driveTrain.getSensitivityLimit() == 0){
					setDriveSensitivity(0.2);
				}
				driveTrain.mecanumDrive_polar(xbox.LeftStick, xbox.RightStick);
			}
			else {
				if(driveTrain.getSensitivityLimit() > 0){
					setDriveSensitivity(0);
				}
				double mag = xbox.LeftStick.getMagnitude() * 0.5;
				double angle = xbox.LeftStick.getAngle();
				double rot = xbox.RightStick.getX();
				driveTrain.mecanumDrive_polar(mag, angle, rot);
			}                                                                                                     
		}
		if(!shooter.hasCurrentAction()){
			if(manualShooter.get())
				manualShooter();
			else
				automaticShooter();
		}
		
		updateSystems();
	}
	
	@Override
	protected void autonomousInit() {
		if(IN_PRESENTATION){
			robotLog.save();
			if(currentAlliance == null)
				currentAlliance = DriverStation.getInstance().getAlliance();
			updateFromFlashboard();
			reset();
			
			setDriveSensitivity(0);
			Action a = getSelectedAutonomous();
			if(a != null){
				a.start();
				robotLog.log("Starting autonomous: "+a.getName(), "Autonomous");
			}else robotLog.log("Autonomous not choosen", "Autonomous");
			done = false;
			auto = a;
			robotLog.save();
		}
	}
	@Override
	protected void autonomousPeriodic() {
		if(IN_PRESENTATION){
			updateFromFlashboard();
			sensorBase.update();
			
			if(auto != null && !auto.isRunning() && !done){
				robotLog.log("Autonomous Done", "Autonomous");
				stopAll();
				done = true;
			}
			
			updateSystems();
		}
	}
	@Override
	protected void disabledInit() {
		robotLog.save();
		reset();
	}
	@Override
	protected void disabledPeriodic() {
		sensorBase.update();
		//updateFromFlashboard();
	}
	
	//------------------------------------------------
	//------------------Systems-----------------------
	//------------------------------------------------
	
	public void reset(){
		stopAll();
	}
	public Action getSelectedAutonomous(){
		return USE_FLASHBOARD? cAutonomous.getSelected() : chooser.getSelected();
	}
	public void updateSmartDashboard(){
		if(USE_FLASHBOARD) return;
		SmartDashboard.putNumber("Sonic L", sensorBase.getSonicRange(SensorBase.SONIC_LEFT));
		SmartDashboard.putNumber("Sonic R", sensorBase.getSonicRange(SensorBase.SONIC_RIGHT));
	}
	public void updateFromFlashboard(){
		if(!USE_FLASHBOARD) return;
	}
	public void updateSystems(){
		feeder.set(feederSpeed);
		if(!shooter.hasCurrentAction())
			shooterController.set(shooterSpeed);
		if(!climber.hasCurrentAction())
			climber.forward(climberSpeed);
		if(!collector.hasCurrentAction())
			collector.forward(collectorSpeed);
	}
	public void resetSystemSpeeds(){
		stopClimber();
		stopCollector();
		stopFeeder();
		stopShooter();
	}
	public void stopAll(){
		robotLog.log("Stopping all", "Robot");
		if(auto != null){
			auto.cancel();
			robotLog.log("Canceling autonomous action: "+auto.getName());
			auto = null;
		}
		setFireMode(true);
		manualDistance.set(false);
		cancelCurrentAction(driveTrain);
		cancelCurrentAction(shooter);
		cancelCurrentAction(collector);
		cancelCurrentAction(climber);
		resetSystemSpeeds();
		
		driveTrain.stop();
		shooter.stop();
		collector.stop();
		climber.stop();
		feeder.set(0);
	}
	public void cancelAllActions(){
	}
	public void cancelCurrentAction(System s){
		Action a = s.getCurrentAction();
		if(a != null && a.isRunning()){
			a.cancel();
			robotLog.log("Canceling action: "+a.getClass().getName(), s.getName());
		}
	}
	public void setDriveSensitivity(double sen){
		driveTrain.setSensitivityLimit(sen);
		robotLog.log("Sensitivity Limit: "+sen, "Drive Train");
	}
	
	//------------------------------------------------
	//------------------Shooter-----------------------
	//------------------------------------------------
	
	private double getForCollector(){
		return assistantControl.get()? xboxAssist.Triggers.Left.getValue() : xbox.Triggers.Left.getValue();
	}
	private double getForShooter(){
		return assistantControl.get()? xboxAssist.Triggers.Right.getValue() : xbox.Triggers.Right.getValue();
	}
	
	public void setShooter(double set){
		shooterSpeed = set;
	}
	public void stopShooter(){
		setShooter(0);
	}
	public void setShooterVoltageMode(){
		if(!shooterController.getControlMode().equals(TalonControlMode.Voltage))
			shooterController.changeControlMode(TalonControlMode.Voltage);
	}
	public void setShooterVbusMode(){
		if(!shooterController.getControlMode().equals(TalonControlMode.PercentVbus))
			shooterController.changeControlMode(TalonControlMode.PercentVbus);
	}
	public void setFireMode(boolean manual){
		autoShootInitialized = false;
		manualShooter.set(manual);
		shooterLock.set(false);
		resetSystemSpeeds();
		robotLog.log("Manual Shooter: "+manualShooter.get(), "Robot");
	}
	
	public void manualShooter(){
		setShooterVbusMode();
		if(shooterLock.get()){
			if(!lockSaved){
				lockSaved = true;
				lockedSpeed = lockedSpeed == 0? getForShooter() : lockedSpeed;
				robotLog.log("LockedSpeed= "+lockedSpeed, "Robot");
			}
			setShooter(lockedSpeed);
			if(lockedSpeed > 0.1 && useFeeder.get())
				setFeeder(FEEDER_SPEED);
			else if(feeder.get() != 0)
				stopFeeder();
		}else{
			if(lockSaved){
				lockSaved = false;
				lockedSpeed = 0;
			}
			double mv = getForShooter();
			setShooter(mv);
			if(mv > 0.1 && useFeeder.get())
				setFeeder(feederdSpeed);
			else if(feeder.get() != 0)
				stopFeeder();
		}
		double collectorVal = getForCollector();
		if(collectorVal != getCollectorSpeed())
			setCollector(collectorVal);
	}
	public void automaticShooter(){
		setShooterVoltageMode();
		long millis =  FlashUtil.millis();
		if(!autoShootInitialized){
			autoShootInitialized = true;
			rpmStabilized = false;
			lastSpeed = sensorBase.getEncoderRate();
			autoShooterStartTime = millis;
			double voltage = interpolateVoltageForShooter();
			if(voltage < 0){
				robotLog.log("TARGET IS AT AN INVALID RANGE!!", "Robot");
				setFireMode(true);
				return;
			}
			setShooter(voltage);
			lastAutoFeed = millis;
		}
		if(!rpmStabilized){
			if(Math.abs(sensorBase.getEncoderRate() - lastSpeed) <= SHOOTER_RPM_RANGE_STABLE && 
					millis - autoShooterStartTime >= SHOOTER_FEEDER_ACTIVATE_WAIT){
				rpmStabilized = true;
				robotLog.log("RPM Stabilized");
			}
			else lastSpeed = sensorBase.getEncoderRate();
		}
		if(rpmStabilized && getFeederSpeed() == 0 && useFeeder.get())
			setFeeder(feederdSpeed);
		else if(getFeederSpeed() != 0 && !useFeeder.get())
			stopFeeder();
		if(rpmStabilized && millis - lastAutoFeed >= COLLECTOR_BURST_TIMEOUT){
			setCollector((getCollectorSpeed() != 0)? 0 : COLLECTOR_AUTO_FIRE_SPEED);
			lastAutoFeed = millis;
		}
	}
	public double interpolateVoltageForShooter(){
		double distance = manualDistance.get()? getManualDistance() :
			sensorBase.getTargetDistance();
		if(distance > SHOOTER_MAX_DISTANCE || distance < SHOOTER_MIN_DISTANCE){
			robotLog.log("Target Distance: "+distance+" Manual: "+manualDistance.get(), "Robot");
			return -1;
		}
		double voltage = lerp.interpolate(distance);
		double filteredVoltage = filterInterpolationValues(voltage);
		robotLog.log("Shooting: \nDistance: "+distance+" Voltage: "+voltage+" Filtered: "+filteredVoltage+
				" Manual: "+manualDistance.get(), "Robot");
		return filteredVoltage;
	}
	public double getManualDistance(){
		return (USE_FLASHBOARD? iManualDistance.doubleValue(140) : 
			SmartDashboard.getNumber("ManDis", 120)) + SensorBase.BOILER_OPENNING_OFFSET;//
	}
	public double filterInterpolationValues(double d){
		return d;//Mathf.limit(d, 0, shooterController.getBusVoltage());
	}
	
	//------------------------------------------------
	//------------------Climber-----------------------
	//------------------------------------------------
	
	public void setClimber(double set){
		climberSpeed = set;
	}
	public void stopClimber(){
		setClimber(0);
	}
	public double getClimberSpeed(){
		return climberSpeed;
	}
	
	//------------------------------------------------
	//------------------Collector-----------------------
	//------------------------------------------------
	
	public void setCollector(double set){
		collectorSpeed = set;
	}
	public void stopCollector(){
		setCollector(0);
	}
	public double getCollectorSpeed(){
		return collectorSpeed;
	}
	
	//------------------------------------------------
	//------------------Feeder-----------------------
	//------------------------------------------------
	
	public void setFeeder(double setSpeed){
		feederSpeed = setSpeed;
	}
	public void stopFeeder(){
		setFeeder(0);
	}
	public double getFeederSpeed(){
		return feederSpeed;
	}
	
	//------------------------------------------------
	//------------------Getters-----------------------
	//------------------------------------------------
	
	public XboxController getController(){
		return xbox;
	}
}
