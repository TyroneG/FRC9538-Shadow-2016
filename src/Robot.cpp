
#include "WPILib.h"
#define CAMERA "cam2"


class Robot: public IterativeRobot  // FRC9538 Shadow 2016
{
	RobotDrive myRobot,Robot8; // robot 10 wheel drive system
	Victor Intake, Flywheel1, Flywheel2;  //motor to control catapult
	Joystick stick, manip; // only joystick
	LiveWindow *lw;
	AnalogInput AnIn2, AnIn3;
	DigitalInput DiIn9, DiIn8, DiIn7, DiIn6, DiIn5, DiIn0;
	float OutputX, OutputY;
	Relay AutoOn, LedOn;
	Solenoid *driveSolenoid  = new Solenoid(0);
	Solenoid *Arm1Solenoid  = new Solenoid(1);
	Solenoid *Arm2Solenoid  = new Solenoid(2);
	Solenoid *FlyArm  = new Solenoid(5);
	Solenoid *ClampSolenoid  = new Solenoid(3);
	Solenoid *FlipoSolenoid  = new Solenoid(4);
	Timer FwdTime, LoopTime, AutoDelayTime,UnclampDelay,FlyWheelDelay;
	bool CatDown, CatActive, ArmExtend, FlyWheelShoot, FlyWheelReady, FlyEngaged;
	bool AutoLow, AutoHi,AutoLowLong, AutoCatDown, AutoFireLatch, AutoReLatch, AutoDelayChk, AutoDelayCmp ;
	bool CompleteOuter, CompleteCourtYard, CYstep1, CYstep2, CYstep3, COstep1, COstep2, COstep3;
	float AutoMode;
	 AnalogGyro gyro;
	double angleSetpoint = 0.0;
	//Preferences *prefs; test

	// Camera setup
	IMAQdxSession session;
	Image *frame;
	IMAQdxError imaqError;

	//gyro calibration constant, may need to be adjusted
	//gyro value of 360 is set to correspond to one full revolution
	const double voltsPerDegreePerSecond = .0128;
	const double pGain = .06; //propotional turning constant

	// Chooser setup
	SendableChooser *chooser, *Def_pos, *DebugDisplay, *CamMenu, *autoDelay;
	const std::string autoNameDefault = "0) Off";
	const std::string autoOuterD1 = "D1) Terrain: Low 2sec";
	const std::string autoOuterD2=  "D2) Rock Wall: High 1.5sec";
	const std::string autoOuterB1=  "B1) Moat: Low 4sec";
	const std::string autoOuterA1=  "A1) Portcullis";
	const std::string autoOuterA2=  "A2) Cheval de Frise";
	const std::string autoOuterB2=  "B2) Ramparts Low 2.5sec";
	const std::string autoOuterB3=  "B3) Ramparts Low 2.0sec";
	const std::string autoOuterE0=  "E) Low Bar Low 7.0sec";
	const std::string autoDefDefault = "0) Not Shooting";
	const std::string autoDef1 = "1) Low Bar";
	const std::string autoDef2 = "2) Position 2";
	const std::string autoDef3 = "3) Position 3";
	const std::string autoDef4 = "4) Position 4";
	const std::string autoDef5 = "5) Position 5";
	const std::string DebugDefault = "0) Debug Off";
	const std::string DebugSet1 = "1) Debug On";
	const std::string autoDelayDefault = "0) Delay Off";
	const std::string autoDelay1 = "1) Delay 5 secs";
	const std::string autoDelay2 = "2) Delay 10 secs";
	const std::string CamDefault = "0) Camera Off";
	const std::string CamOn = "0) Camera On";
	std::string autoSelected;
	std::string DefSelected;
	std::string DebugSelected;
	std::string CamSelected;
	std::string autoDelaySelected;






public:
	Robot() :

		myRobot(4, 5, 0, 1),// these must be initialized in the same order (PWM 0, PWM1, PWM2, PWM3)
		Robot8(3,2),
		Intake(7),Flywheel1(9),Flywheel2(8),		// PWM 9 for Fly Wheel motor1
		stick(0),		// as they are declared above. 0 = 1st controller for drive
		manip(1),		// 1 = 2nd controller for manipulator control
		//	Button A = Button 1   LX Axis = Axis 0
		//  Button B = Button 2   LY Axis = Axis 1
		//	Button X = Button 3   L Trigger = Axis 2
		//	Button Y = Button 4   R Trigger = Axis 3
		//	L Bumper = Button 5   RX Axis = Axis 4
		//	R Bumper = Button 6   RY Axis = Axis 5
		// 	Back Button = Button 7
		//	Start Button = Button 8
		// 	L Analog Click = Button 9
		// 	R Analog CLick = Button 10
		lw(NULL),
		AnIn2(2), AnIn3(3), 	// set In Analog channels 2, 3,
		DiIn9(9), DiIn8(8), DiIn7(7), DiIn6(6), DiIn5(5), DiIn0(0),	// set Digital Channels
		// Ch9 Catapult down1, Ch8 Catapult down2, Ch7 Catapult Latched
		// Ch6 Intake Forward,  Ch 5 Intake Rearwards
		OutputX(0), OutputY(0),
		AutoOn(0), LedOn(3),			// Set Relay channel 0 for Auto Status
										// Relay channel 3 for LED on
		CatDown(false), CatActive(true),ArmExtend(false),FlyWheelShoot(false),FlyWheelReady(false),FlyEngaged(false),
		AutoLow(false),AutoHi(false),AutoLowLong(false),AutoCatDown(false),AutoFireLatch(false),
		AutoDelayChk(false),AutoDelayCmp(false),
		CompleteOuter(false), CompleteCourtYard(false),CYstep1(false),CYstep2(false),CYstep3(false),
		COstep1(false),COstep2(false),COstep3(false),
		AutoMode(0),
		gyro(0),
		chooser(), Def_pos(),DebugDisplay(), CamMenu(), autoDelay()






	{
		myRobot.SetExpiration(0.15);



	}

private:
	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		driveSolenoid->Set(false);      //turn off all solenoids
		Arm1Solenoid->Set(false);
		Arm2Solenoid->Set(false);
		ClampSolenoid->Set(false);
		FlipoSolenoid->Set(false);
		FlyArm->Set(false);
		AutoOn.Set(Relay::Value::kOff);
		SmartDashboard::PutNumber("Auto Relay", (double) 0);
		OutputY = 0;
		OutputX = 0;
		gyro.Calibrate();

	    // create an image
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		//the camera name (ex "cam0") can be found through the roborio web interface
		imaqError = IMAQdxOpenCamera(CAMERA, IMAQdxCameraControlModeController, &session);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError) + "\n");
		}
		imaqError = IMAQdxConfigureGrab(session);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long)imaqError) + "\n");
		}


		SmartDashboard::PutString("DB/String 0", "New Name = Auto Low");
		SmartDashboard::PutString("DB/String 1", "DB/Button1=Auto Hi 1.5");
		SmartDashboard::PutString("DB/String 2", "DB/Button2 = Auto Low 4");
		//	prefs = Preferences::GetInstance();
	//	AutoMode = prefs->GetFloat("AutoMode");
	//	AutoLow = prefs->GetBoolean("AutoLow");

		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoOuterA1, (void*)&autoOuterA1);
		chooser->AddObject(autoOuterA2, (void*)&autoOuterA2);
		chooser->AddObject(autoOuterB1, (void*)&autoOuterB1);
		chooser->AddObject(autoOuterB2, (void*)&autoOuterB2);
		chooser->AddObject(autoOuterB3, (void*)&autoOuterB3);
		chooser->AddObject(autoOuterD1, (void*)&autoOuterD1);
		chooser->AddObject(autoOuterD2, (void*)&autoOuterD2);
		chooser->AddObject(autoOuterE0, (void*)&autoOuterE0);
		SmartDashboard::PutData("Auto Outer6", chooser);

		Def_pos = new SendableChooser();
		Def_pos->AddDefault(autoDefDefault,(void*)&autoDefDefault);
		Def_pos->AddObject(autoDef1,(void*)&autoDef1);
		Def_pos->AddObject(autoDef2,(void*)&autoDef2);
		Def_pos->AddObject(autoDef3,(void*)&autoDef3);
		Def_pos->AddObject(autoDef4,(void*)&autoDef4);
		Def_pos->AddObject(autoDef5,(void*)&autoDef5);
		SmartDashboard::PutData("Auto Def Position3", Def_pos);

		DebugDisplay = new SendableChooser();
		DebugDisplay->AddDefault(DebugDefault,(void*)&DebugDefault);
		DebugDisplay->AddObject(DebugSet1,(void*)&DebugSet1);
		SmartDashboard::PutData("Debug Menu", DebugDisplay);

		CamMenu = new SendableChooser();
		CamMenu->AddDefault(CamDefault,(void*)&CamDefault);
		CamMenu->AddObject(CamOn,(void*)&CamOn);
		SmartDashboard::PutData("Camera Menu", CamMenu);

		autoDelay = new SendableChooser();
		autoDelay->AddDefault(autoDelayDefault,(void*)&autoDelayDefault);
		autoDelay->AddObject(autoDelay1,(void*)&autoDelay1);
		autoDelay->AddObject(autoDelay2,(void*)&autoDelay2);
		SmartDashboard::PutData("AutoDelay", autoDelay);

	}

	bool DriveFwd(std::string L, std::string M, float a, float spd = 1.0, bool Hgear = false)
	{

		bool P;
		double turningValue1;
		turningValue1 = (angleSetpoint - gyro.GetAngle()) * -pGain;
		double Gvalue=gyro.GetAngle();
		SmartDashboard::PutNumber("Turn Value", turningValue1);
		SmartDashboard::PutNumber("Gyro Value", Gvalue);



		if (L==M and FwdTime.Get()< a){
			driveSolenoid->Set(Hgear);
			myRobot.ArcadeDrive(spd,turningValue1,true);	//drive straight
			Robot8.ArcadeDrive(spd,turningValue1,true);   // for a seconds
			P = true;
		} else
			P = false;

		return P;

	}

	bool TurnRight(std::string L, std::string M, double Tvalue=0, bool Hgear = false)
	{
		bool P2;
		double turningValue2, LowAngle, HighAngle, Gvalue;
		LowAngle = Tvalue - 4.0, HighAngle = Tvalue +4.0;
		Gvalue = gyro.GetAngle();
		turningValue2 = (Tvalue - gyro.GetAngle()) * -pGain;
		SmartDashboard::PutNumber("Turn Value", turningValue2);
		SmartDashboard::PutNumber("Gyro Value", Gvalue);

		if (L==M and !(Gvalue > LowAngle and Gvalue < HighAngle)){
			driveSolenoid->Set(Hgear);
			myRobot.ArcadeDrive(0,(turningValue2/2),true);	//drive straight
			Robot8.ArcadeDrive(0,(turningValue2/2),true);   // for a seconds
			P2 = true;
		}	else
			P2 = false;

		return P2;

	}


	void AutonomousInit()
	{
		CompleteOuter=false;
		CompleteCourtYard=false;
		AutoFireLatch=false;
		AutoReLatch=false;
		AutoDelayChk = false;
		AutoDelayCmp = false;
		COstep1=false;
		COstep2=false;
		COstep3=false;
		CYstep1=false; CYstep2=false; CYstep3=false;
		//gyro.SetSensitivity(voltsPerDegreePerSecond); //calibrates gyro values to equal degrees
		AutoOn.Set(Relay::Value::kOff);
		SmartDashboard::PutNumber("Auto Relay", (double) 0);
		gyro.Reset();
		myRobot.SetSafetyEnabled(false); Robot8.SetSafetyEnabled(false);


		FwdTime.Stop();
		FwdTime.Reset();
		FwdTime.Start();

		AutoDelayTime.Stop();
		AutoDelayTime.Reset();
		AutoDelayTime.Start();


		autoSelected = *((std::string*)chooser->GetSelected());
		std::cout << "Auto selected: " << autoSelected << std::endl;
		autoDelaySelected = *((std::string*)autoDelay->GetSelected());

		DefSelected = *((std::string*)Def_pos->GetSelected());
		//std::cout << "Defense position: " << DefSelected << std::endl;
		if (DefSelected == autoDef1)
			std::cout << "Moving from Low Bar" << std::endl;
		else if (DefSelected == autoDef2)
			std::cout << "Moving from Position 2" << std::endl;
		else if (DefSelected == autoDef3)
			std::cout << "Moving from Position 3" << std::endl;
		else if (DefSelected == autoDef4)
			std::cout << "Moving from Position 4" << std::endl;
		else if (DefSelected == autoDef5)
			std::cout << "Moving from Position 5" << std::endl;
		else
			std::cout << "Not Shooting" << std::endl;



	}

	void AutonomousPeriodic()
	{
		// AUTO code controlled  by drivers station



		bool ChkPt2 = false;
		bool ChkPt1 = false;
		bool ChkPt0 = false;

		//
		//
		// Code to delay Auto
		// Options (no delay, 5 sec, 10 sec)
		//
		//
		// Check Auto delay
		//		no delay: set AutoDelay flags to complete
		//		5sec or 10 sec: start timer
		if (!AutoDelayChk and (autoDelaySelected == autoDelayDefault)){
			AutoDelayChk = true;
			AutoDelayCmp = true;
		}
		else if (!AutoDelayChk){
			AutoDelayChk = true;
			AutoDelayTime.Stop();
			AutoDelayTime.Reset();
			AutoDelayTime.Start();
		}
		// Check AutoDelay: set AutoDelay flags to complete after 5 secs
		if (!AutoDelayCmp and (autoDelaySelected == autoDelay1) and (AutoDelayTime.Get() > 5 )){
			AutoDelayCmp = true;
			FwdTime.Stop();
			FwdTime.Reset();
			FwdTime.Start();
		}
		// Check AutoDelay: set AutoDelay flags to complete after 10 secs
		else if (!AutoDelayCmp and (autoDelaySelected == autoDelay2) and (AutoDelayTime.Get() > 10 )){
			AutoDelayCmp = true;
			FwdTime.Stop();
			FwdTime.Reset();
			FwdTime.Start();
		}

		if (!COstep1){
			ChkPt0 = DriveFwd(autoSelected, autoOuterA2, 3.25,0.5, false);
			}
		if (!ChkPt0 and !COstep1){
			COstep1 = true;
			FwdTime.Stop();
			FwdTime.Reset();
			FwdTime.Start();
			}

		if (COstep1 and (autoSelected == autoOuterA2) and !COstep2){
			COstep2 = true;
			Arm1Solenoid->Set(true);	// extend Arm
			Arm2Solenoid->Set(true);
			myRobot.ArcadeDrive(0,0,true); 	// stop robot
			Robot8.ArcadeDrive(0,0,true);
		}
		else if (COstep1 and (autoSelected != autoOuterA2) )
			COstep2 = true;

		if (COstep2 and !COstep3){
			ChkPt2 = DriveFwd(autoSelected, autoOuterA2, 1.0,0.0, false);
			}
		if (!ChkPt0 and !ChkPt2 and !COstep3){
			COstep3 = true;
			FwdTime.Stop();
			FwdTime.Reset();
			FwdTime.Start();
			}



		if(!CompleteOuter and  COstep2 and (autoSelected== autoOuterE0) ){
			Arm1Solenoid->Set(true);	// extend Arm
			Arm2Solenoid->Set(true);
			}


		if (!CompleteOuter and COstep2 and COstep3 and AutoDelayCmp){
		ChkPt1 = ChkPt1 or DriveFwd(autoSelected, autoOuterD1, 2.0); //drive straight Low gear 2sec 15ft
		ChkPt1 = ChkPt1 or DriveFwd(autoSelected, autoOuterD2, 1.5, 1.0, true); //drive straight High gear 1.5sec
		ChkPt1 = ChkPt1 or DriveFwd(autoSelected, autoOuterB1, 4.0); //drive straight Low gear 4sec
		ChkPt1 = ChkPt1 or DriveFwd(autoSelected, autoOuterB2, 2.5); //drive straight Low gear 2.5sec
		ChkPt1 = ChkPt1 or DriveFwd(autoSelected, autoOuterB3, 2.0); //drive straight Low gear 2.0sec
		ChkPt1 = ChkPt1 or DriveFwd(autoSelected, autoOuterA2, 1.5); //drive straight Low gear 1.5sec
		ChkPt1 = ChkPt1 or DriveFwd(autoSelected, autoOuterE0, 9.0,-0.5); //drive straight Low gear 3.5sec
		}
		if (!ChkPt0 and !ChkPt1 and !ChkPt2 and !CompleteOuter and AutoDelayCmp){
			driveSolenoid->Set(false);	    // Low gear
			myRobot.ArcadeDrive(0,0,true); 	// stop robot
			Robot8.ArcadeDrive(0,0,true);
			FwdTime.Stop();
			FwdTime.Reset();
			FwdTime.Start();
			Arm1Solenoid->Set(false);	// retract Arm
			Arm2Solenoid->Set(false);
			CompleteOuter=true;
		}



		if (!CompleteCourtYard and (DefSelected == autoDef1) and CompleteOuter){		//Position 1 code (low bar)
			ChkPt1 = false;
			if(!CYstep1){
				//SmartDashboard::PutNumber("FWD Time", FwdTime.Get());
			ChkPt1 = TurnRight(DefSelected, autoDef1, 35);	// Turn to 45 degrees
			}
			if (!CYstep1 and !ChkPt1){
				myRobot.ArcadeDrive(0,0,true); 	// stop robot
				Robot8.ArcadeDrive(0,0,true);
				FwdTime.Stop();
				FwdTime.Reset();
				FwdTime.Start();
				CYstep1=true;
				}

			ChkPt1 = false;
			if (CYstep1 and !CYstep2){
			ChkPt1 = DriveFwd(DefSelected, autoDef1, 0.0,-0.75,false);		//Drive forward 0.75 for 0.5 sec
			}
			if (!ChkPt1 and CYstep1){
				myRobot.ArcadeDrive(0,0,true); 	// stop robot
				Robot8.ArcadeDrive(0,0,true);
				FwdTime.Stop();
				FwdTime.Reset();
				FwdTime.Start();
				CYstep2=true;
				AutoOn.Set(Relay::Value::kForward); //Set Relay Forward on Auto has started
				LedOn.Set(Relay::Value::kForward); //Turn LED on for Auto shoot
				SmartDashboard::PutNumber("Auto Relay", (double) 2);
				CompleteCourtYard = true;
				}


		}
		 else if (!CompleteCourtYard and DefSelected == autoDef2 and CompleteOuter){  // position 2 code
			//std::cout << "Moving from Position 2" << std::endl;
				ChkPt1 = false;
				if(!CYstep1){
					//SmartDashboard::PutNumber("FWD Time", FwdTime.Get());
				ChkPt1 = TurnRight(DefSelected, autoDef1, -90);		//Turn toward center
				}
				if (!CYstep1 and !ChkPt1){
					myRobot.ArcadeDrive(0,0,true); 	// stop robot
					Robot8.ArcadeDrive(0,0,true);
					FwdTime.Stop();
					FwdTime.Reset();
					FwdTime.Start();
					CYstep1=true;
					}

				ChkPt1 = false;
				if (CYstep1 and !CYstep2){
				ChkPt1 = DriveFwd(DefSelected, autoDef1,0.53,-1.0,false);	// drive to position 3
				}
				if (!ChkPt1 and CYstep1){
					myRobot.ArcadeDrive(0,0,true); 	// stop robot
					Robot8.ArcadeDrive(0,0,true);
					FwdTime.Stop();
					FwdTime.Reset();
					FwdTime.Start();
					CYstep2=true;
					}
				// add code to turn -90  to face castle
				ChkPt1 = false;
				if (CYstep1 and CYstep2 and !CYstep3){
				ChkPt1 = TurnRight(DefSelected, autoDef1,-90);	// turn -90  to face castle
				}
				if (!ChkPt1 and CYstep1 and CYstep2){
					myRobot.ArcadeDrive(0,0,true); 	// stop robot
					Robot8.ArcadeDrive(0,0,true);
					FwdTime.Stop();
					FwdTime.Reset();
					FwdTime.Start();
					CYstep3=true;
					}

		 }
		 else if (!CompleteCourtYard and DefSelected == autoDef3 and CompleteOuter){ // position 3 code
			//std::cout << "Moving from Position 3" << std::endl;
				ChkPt1 = false;
				if(!CYstep1){
					//SmartDashboard::PutNumber("FWD Time", FwdTime.Get());
				ChkPt1 = TurnRight(DefSelected, autoDef1, 180);		//Turn toward Castle
				}
				if (!CYstep1 and !ChkPt1){
					myRobot.ArcadeDrive(0,0,true); 	// stop robot
					Robot8.ArcadeDrive(0,0,true);
					FwdTime.Stop();
					FwdTime.Reset();
					FwdTime.Start();
					CYstep1=true;
					}

				ChkPt1 = false;
				if (CYstep1 and !CYstep2){
				ChkPt1 = DriveFwd(DefSelected, autoDef1,0.1,-0.5,false);	// drive toward Castle
				}
				if (!ChkPt1 and CYstep1){
					myRobot.ArcadeDrive(0,0,true); 	// stop robot
					Robot8.ArcadeDrive(0,0,true);
					FwdTime.Stop();
					FwdTime.Reset();
					FwdTime.Start();
					CYstep2=true;
					}

		 }
		 else if (!CompleteCourtYard and DefSelected == autoDef4 and CompleteOuter){  // position 4 code
			//std::cout << "Moving from Position 4" << std::endl;
				ChkPt1 = false;
				if(!CYstep1){
					//SmartDashboard::PutNumber("FWD Time", FwdTime.Get());
				ChkPt1 = TurnRight(DefSelected, autoDef1, 90);		//Turn toward center				//AutoSolenoid->Set(true);	// verify stage2 temp use only remove for final
				}
				if (!CYstep1 and !ChkPt1){
					myRobot.ArcadeDrive(0,0,true); 	// stop robot
					Robot8.ArcadeDrive(0,0,true);
					FwdTime.Stop();
					FwdTime.Reset();
					FwdTime.Start();
					CYstep1=true;
					}

				ChkPt1 = false;
				if (CYstep1 and !CYstep2){
				ChkPt1 = DriveFwd(DefSelected, autoDef1,0.53,-1.0,false);	// drive to position 3				//Arm2Solenoid->Set(true);	// verify stage2 temp use only remove for final
				}
				if (!ChkPt1 and CYstep1){
					myRobot.ArcadeDrive(0,0,true); 	// stop robot
					Robot8.ArcadeDrive(0,0,true);
					FwdTime.Stop();
					FwdTime.Reset();
					FwdTime.Start();
					CYstep2=true;
					}
				// add code to turn -90  to face castle
				ChkPt1 = false;
				if (CYstep1 and CYstep2 and !CYstep3){
				ChkPt1 = TurnRight(DefSelected, autoDef1, 90);	// turn  90  to face castle
				}
				if (!ChkPt1 and CYstep1 and CYstep2){
					myRobot.ArcadeDrive(0,0,true); 	// stop robot
					Robot8.ArcadeDrive(0,0,true);
					FwdTime.Stop();
					FwdTime.Reset();
					FwdTime.Start();
					CYstep3=true;
					}


		 }
		 else if (!CompleteCourtYard and DefSelected == autoDef5 and CompleteOuter){ // position 5 code
			//std::cout << "Moving from Position 5" << std::endl;
				ChkPt1 = false;
				if(!CYstep1){
					//SmartDashboard::PutNumber("FWD Time", FwdTime.Get());
				ChkPt1 = DriveFwd(DefSelected, autoDef1, 1.0, 1.0,false);		//Drive forward 1 sec 7.5 ft
				}
				if (!CYstep1 and !ChkPt1){
					myRobot.ArcadeDrive(0,0,true); 	// stop robot
					Robot8.ArcadeDrive(0,0,true);
					FwdTime.Stop();
					FwdTime.Reset();
					FwdTime.Start();
					CYstep1=true;
					}

				ChkPt1 = false;
				if (CYstep1 and !CYstep2){
				ChkPt1 = TurnRight(DefSelected, autoDef1, 135);	// Turn to 45 degrees
				}
				if (!ChkPt1 and CYstep1){
					myRobot.ArcadeDrive(0,0,true); 	// stop robot
					Robot8.ArcadeDrive(0,0,true);
					FwdTime.Stop();
					FwdTime.Reset();
					FwdTime.Start();
					CYstep2=true;
					}

		 }
		 else
			// Stage 2 Complete
			 SmartDashboard::PutNumber("Auto Relay", (double) 0);



		// Auto code controlled by auto electronics
		// Relay CH0= Auto ON, DIO CH0= AutoFire
		// Analog IN CH2= Left Motor, Analog IN CH2 = Right Motor

		DebugSelected = *((std::string*)DebugDisplay->GetSelected());
		if (DebugSelected == DebugSet1 and !AutoFireLatch){		// Display Debug if On
			CompleteCourtYard = true;
			AutoOn.Set(Relay::Value::kForward); //Set Relay Forward on Auto has started
			SmartDashboard::PutNumber("Auto Relay", (double) 2);
		}

		if (CompleteCourtYard) {


			Arm1Solenoid->Set(true);	//extend Arm
			Arm2Solenoid->Set(true);
			float LhMtr, RhMtr;
		if(!AutoFireLatch){
			LhMtr = AnIn3.GetVoltage(); // get Analog value from channel 3
			RhMtr = AnIn2.GetVoltage(); // get Analog value from channel 2
			}
		else {
			LhMtr = 2.5;
			RhMtr = 2.5;
			}

		SmartDashboard::PutNumber("Analog Channel2", (double) LhMtr);
		SmartDashboard::PutNumber("Analog Channel3", (double) RhMtr);
		float LhMtrO = (LhMtr - 2.5) / 2.5;	  // Center and scale LH motor
		float RhMtrO = (RhMtr - 2.5) / 2.5;	  // Center and scale RH motor
		SmartDashboard::PutNumber("Left Motor", (double) LhMtrO);
		SmartDashboard::PutNumber("Right Motor", (double) RhMtrO);

		    myRobot.TankDrive(LhMtrO, RhMtrO, false); 	// Drive robot
			Robot8.TankDrive(LhMtrO, RhMtrO, false);

		bool AutoFire = DiIn0.Get();      // get Digital value from channel 0
		bool AutoDwn1  = DiIn8.Get();      // get Digital value from channel 8
		bool AutoDwn2  = DiIn9.Get();      // get Digital value from channel 8
		SmartDashboard::PutBoolean("Auto Launch", AutoFire);

		AutoCatDown = !AutoDwn1 or !AutoDwn2; // Latch CatDown if one switch is false
		SmartDashboard::PutBoolean("Auto Down", AutoCatDown);

		float AutoLatchSpeed;
		if (!AutoCatDown and FwdTime.Get() > 1.0 ){	//Not Latched drive forward
			 AutoLatchSpeed = 1.0;
			 AutoReLatch=true;
			}
		else if (AutoCatDown and !AutoFire and !AutoFireLatch){
			AutoLatchSpeed = -1.0;
			AutoFireLatch=true;
			 AutoOn.Set(Relay::Value::kOff);  //Set Relay off Auto is over
			 LedOn.Set(Relay::Value::kOff); //Turn LED off Auto is over

			 SmartDashboard::PutNumber("Auto Relay", (double) 0);
				FwdTime.Stop();
				FwdTime.Reset();
				FwdTime.Start(); }
		else if (AutoCatDown and AutoFireLatch and !AutoReLatch){
			AutoLatchSpeed = -1.0;
			FwdTime.Stop();
			FwdTime.Reset();
			FwdTime.Start(); }

		else{
			AutoLatchSpeed = 0.0;
			}
		Flywheel1.Set(AutoLatchSpeed);
		Flywheel2.Set(AutoLatchSpeed);

		// END of Auto code controlled by auto electronics


		}

	}


	void TeleopInit()
	{
		FwdTime.Stop();
		FwdTime.Reset();
		LoopTime.Stop();
		LoopTime.Reset();
		FlyWheelDelay.Stop();
		FlyWheelDelay.Reset();
		FlyWheelShoot=false;
		FlyWheelReady=false;
		FlyEngaged=false;
		Flywheel1.Set(0.0);
		Flywheel2.Set(0.0);


		AutoOn.Set(Relay::Value::kOff);  //Set Relay off Auto is over
		SmartDashboard::PutNumber("Auto Relay", (double) 0);
		myRobot.SetSafetyEnabled(true); Robot8.SetSafetyEnabled(true);
		Arm1Solenoid->Set(false);  Arm2Solenoid->Set(false); FlyArm->Set(false);
		FlipoSolenoid->Set(false);	// Set Flipo for Short Shot



	    // acquire images
		IMAQdxStartAcquisition(session);

	}

	void TeleopPeriodic()
	{

		LoopTime.Stop();
		LoopTime.Reset();
		LoopTime.Start();

		DebugSelected = *((std::string*)DebugDisplay->GetSelected());
		CamSelected = *((std::string*)CamMenu->GetSelected());

        if (CamSelected == CamOn){
        	// grab an image, draw the circle, and provide it for the camera server which will
        	// in turn send it to the dashboard.
        	IMAQdxGrab(session, frame, true, NULL);
        	if(imaqError != IMAQdxErrorSuccess) {
        		DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError) + "\n");
        	} else {
        		imaqDrawShapeOnImage(frame, frame, { 170, 225, 180, 235 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 0.0f);
        		CameraServer::GetInstance()->SetImage(frame);
        	}
        }
        //
        //
        // Driver controls
        //
        //
		bool LHbutton = stick.GetRawButton(5); // get Left Bumper
		bool RHbutton = stick.GetRawButton(6); // get Right Bumper

		if (RHbutton)
			driveSolenoid->Set(true);			// High gear press RH bumper
		if (LHbutton)
			driveSolenoid->Set(false);			// Low gear press LH bumper
		float LeftY = stick.GetRawAxis(1);
		float RightX = stick.GetRawAxis(4);
		// myRobot.ArcadeDrive(stick,1,stick,4,true); // drive with arcade style (use right stick)
		if(LeftY < 0.11 and LeftY > -0.11) // Set Y dead band 0.01 higher than rotation correction
			LeftY = 0;
		if(RightX < 0.11 and RightX > -0.11) // Set X dead band 0.01 higher than rotation correction
			RightX = 0;
		if(LeftY == 0 and RightX > 0.7) // Set X max turn to 70%
			RightX = 0.7;
		if(LeftY == 0 and RightX < -0.7) // Set X max turn to -70%
			RightX = -0.7;
		if(LeftY <= 0.1) // change rotation when driving forward
		{
			RightX = RightX * -1;
		}
		if(LeftY >= 0.1) // change rotation when driving backward
		{
			RightX = RightX * -1;
		}

		float LdTrig = stick.GetRawAxis(2);			// Read Left Drive Trigger
		if (LdTrig >0.1){
			RightX = RightX * (0.5);	// Reduce turn speed
			LeftY = LeftY * 0.5;}		// Reduce drive speed

		OutputY = (0.8 * OutputY) + (0.2 * LeftY); //slow down direction changes from 1 cycle to 5
		OutputX = (0.8 * OutputX) + (0.2 * RightX);
		myRobot.ArcadeDrive(OutputY,OutputX,true);
		Robot8.ArcadeDrive(OutputY,OutputX,true);


		// Read Right Trigger and turn on LED light if depressed

		float RdTrig = stick.GetRawAxis(3);			// Read Right Drive Trigger
		if (RdTrig >0.1)
			LedOn.Set(Relay::Value::kForward); //Turn LED on if Right Trigger
		else
			LedOn.Set(Relay::Value::kOff);  // Turn LED off


		//  Read motor current from PDP and display on drivers station

		PowerDistributionPanel *pdp = new PowerDistributionPanel();  // create pdp variable
		double current = pdp->GetTotalCurrent();					// Get total current


		// vibration if current to high


		float LHThr = 0.0;		// Define value for rumble
		if (current > 125.0)		// Rumble if greater than 125 amps motor current
			LHThr = 0.5;
		Joystick::RumbleType Vibrate;				// define Vibrate variable
		Vibrate = Joystick::kLeftRumble;		// set Vibrate to Left
		stick.SetRumble(Vibrate ,LHThr);  		 // Set Left Rumble to RH Trigger
		Vibrate = Joystick::kRightRumble;		// set vibrate to Right
		stick.SetRumble(Vibrate ,LHThr);	  // Set Right Rumble to RH Trigger
		//
		//
		//  Manipulator Controls
		//
		//

		//
		// Code to read Manipulator Control buttons
		//
		bool Ybutton = manip.GetRawButton(4); // get Y button state
		bool Abutton = manip.GetRawButton(1); // get A button state
		bool Xbutton = manip.GetRawButton(3); // get 2nd control X Button
		bool Bbutton = manip.GetRawButton(2); // get 2nd control B Button
		bool Startbutton = manip.GetRawButton(8); // get 2nd control Start Button
		float LmTrig = manip.GetRawAxis(2);
		float RmTrig = manip.GetRawAxis(3);
		bool RMbumper = manip.GetRawButton(6); // get Right Bumper
		bool LMbumper = manip.GetRawButton(5); // get Left Bumper
		int MPov = manip.GetPOV();


		bool FlyWheelPrep = Bbutton or FlyWheelShoot;
		//
		// code for  Shooting Ball
		//
		if (Xbutton) {					// X button turn Flywheel off
			Flywheel1.Set(0.0);
			Flywheel2.Set(0.0);
			FlyWheelReady=false;}
		else if (Ybutton){
			ClampSolenoid->Set(true);	// Y button clamp ball
		}
		else if (Startbutton) ClampSolenoid->Set(false); // Start button unclamp ball
		else if (FlyWheelPrep and !FlyWheelReady){		// B button Flywheel on 100%, extend Arm and intake off
			Flywheel1.Set(1.0);
			Flywheel2.Set(1.0);
			Arm1Solenoid->Set(true);
			Arm2Solenoid->Set(true);
			Intake.Set(0.0);
			FlyWheelReady=true;
			}
		else if (Abutton and !FlyWheelShoot){	 // A button and clamped, then unclamp
			ClampSolenoid->Set(false);
			UnclampDelay.Stop();
			UnclampDelay.Reset();
			UnclampDelay.Start();
			FlyWheelShoot=true;}

		// A button and unclamped and Unclamp >0,25 sec, then engage Flywheel to shoot
		else if (FlyWheelShoot and !FlyEngaged and UnclampDelay.Get() > 0.250 ){
			FlyArm->Set(true);
			FlyWheelDelay.Stop();
			FlyWheelDelay.Reset();
			FlyWheelDelay.Start();
			FlyEngaged=true;}

		//1sec after Fly Wheel shoots, retract and turn off Fly Wheel
		else if (FlyEngaged and FlyWheelDelay.Get()> 1.0){
			FlyArm->Set(false);
			Flywheel1.Set(0.0);
			Flywheel2.Set(0.0);
			FlyWheelDelay.Stop();
			FlyWheelShoot=false;
			FlyWheelReady=false;
			FlyEngaged=false;}

		//
		//
		// code for intake
		//
		//
		//	Arm control
		//
		if (RmTrig > 0.2 and !FlyWheelShoot){					// Ball floor to intake (intake down)
			Arm1Solenoid->Set(true);	//extend Arm
			Arm2Solenoid->Set(true);
			}
		else{
			Arm1Solenoid->Set(false);
			Arm2Solenoid->Set(false);
			}

		//
		//	Roller/FlyWheel control
		//
		if (RMbumper and LmTrig > 0.2 and FlyWheelReady ){		// prep for intake to cradle
			Flywheel1.Set(0.0);									// Turn off flywheel motors
			Flywheel2.Set(0.0);
			FlyWheelReady=false;
			Intake.Set(1.0);}
		else if (RMbumper and LmTrig > 0.2  and !FlyWheelShoot and !FlyWheelReady ){		// intake to cradle
			ClampSolenoid->Set(false);	// unclamp
			Intake.Set(0.3);
			Flywheel1.Set(-0.30);
			Flywheel2.Set(-0.30);
			FlyArm->Set(true);
			}
		else if (RMbumper and LMbumper){			// cradle to intake
			ClampSolenoid->Set(false);				// unclamp
			Intake.Set(-0.3);
			Flywheel1.Set(0.30);
			Flywheel2.Set(0.30);
			FlyArm->Set(true);
			}
		else if (LmTrig > 0.2 ){	//roller on to intake from floor
			Intake.Set(1.0);}
		else if (LMbumper){			// roller on to push to floor
			Intake.Set(-1.0);		// intake to floor
			}
		else if (!Abutton and !FlyWheelPrep) {
			Flywheel1.Set(0.0);
			Flywheel2.Set(0.0);
			FlyArm->Set(false);
			Intake.Set(0.0);
			}
		else {
			Intake.Set(0.0);
			}




		if (MPov == 0)
			FlipoSolenoid->Set(true);	// Set Flipo for Long Shot
		else if (MPov == 180)
			FlipoSolenoid->Set(false);	// Set Flipo for Short Shot

		LoopTime.Stop();
		double LastLoop = LoopTime.Get();

		if (DebugSelected == DebugSet1){		// Display Debug if On
		SmartDashboard::PutNumber("Loop Time", LastLoop);
		SmartDashboard::PutNumber("OutputX", (double) OutputX);		// Display X output to motor
		SmartDashboard::PutNumber("OutputY", (double) OutputY);		// Display Y output to motor
		SmartDashboard::PutNumber("Total Current", current);		// Display total current
		double current0 = pdp->GetCurrent(0);						// Get PDP channel 0 current
		SmartDashboard::PutNumber("Left Current PDP 0", current0);	// Display channel 0 current
		double current1 = pdp->GetCurrent(1);
		SmartDashboard::PutNumber("Left Current PDP 1", current1);
		double current2 = pdp->GetCurrent(2);
		SmartDashboard::PutNumber("Left Current PDP 2", current2);
		double current3 = pdp->GetCurrent(14);
		SmartDashboard::PutNumber("Right Current PDP 14", current3);
		double current4 = pdp->GetCurrent(13);
		SmartDashboard::PutNumber("Right Current PDP 13", current4);
		double current5 = pdp->GetCurrent(15);
		SmartDashboard::PutNumber("Right Current PDP 15", current5);
		double current6 = pdp->GetCurrent(8);
		SmartDashboard::PutNumber("Catapult Current PDP 8", current6);
		SmartDashboard::PutNumber("RH Throttle", LHThr); // Display RH trigger value
		SmartDashboard::PutNumber("Forward Time", FwdTime.Get());


		}



	}
	void DisabledInit()
	{
		IMAQdxStopAcquisition(session);
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
