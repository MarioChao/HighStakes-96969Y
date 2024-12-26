// Brain Screen Painter

#include "Graphics/BrainScreen.h"

#include "Autonomous/auton.h"
#include "Autonomous/autonFunctions.h"

#include "AutonUtilities/odometry.h"

#include "GraphUtilities/trajectoryPlanner.h"
#include "GraphUtilities/curveSampler.h"

#include "Graphics/GraphicMain.h"
#include "Graphics/GUIs/ButtonsGui.h"
#include "Graphics/GUIs/ShapesGui.h"
#include "Graphics/GUIs/SlidersGui.h"
#include "Graphics/GUIs/DocksGui.h"

#include "Simulation/robotSimulator.h"

#include "Utilities/angleUtility.h"
#include "Utilities/generalUtility.h"

#include "Videos/video-main.h"
#include "main.h"

// File-local Functions & Variables
namespace {
	using namespace gfxmain;
	using namespace auton;

	// The field and the robot
	void _drawVexField(int x, int y, int width, int height);
	void drawCoordinate(int x, int y, int width, int height);
	// Flywheel speed graph
	void drawFlywheel(int x, int y, int width, int height);

	// GUI
	// Buttons
	void createButtons();
	// Sliders
	void createSliders();
	// Docks
	void createDocks();
	// Dock GUIs
	void setDockGUIs();
	// Dock states
	void initDocks();
	// QR Codes
	void initQRCodes();

	// Getters
	double getRobotX_tiles();
	double getRobotY_tiles();
	double getRobotPolarAngle_degrees();

	// Draw info
	void drawInfo();
	void drawQRCodes();
	void drawQRCode(double x, double y, double width, vector<pair<int, int>> &QRCode, color bgCol, color qrCol);
	void drawTemperature();
	void drawMotorPower();
	void drawInertial();

	// Variables
	// Flywheel
	double fw_drawX = 20;
	double fw_prevRealY = -1, fw_newRealY;
	double fw_prevAimY = -1, fw_newAimY;

	// Guis
	vector<ButtonGui *> mainDockButtons;
	vector<ButtonGui *> autonDockButtons, allianceButtons;
	vector<ButtonGui *> autonSubdock1Buttons, autonSubdock2Buttons, autonSubdock3Buttons, autonSubdock4Buttons;
	SliderGui *slider;
	DockGui *mainDock, *mainDock_dockDock;
	DockGui *simulationDock;
	DockGui *autonDock, *autonDock_dockDock;
	DockGui *autonSubdock1, *autonSubdock2, *autonSubdock3, *autonSubdock4;
	DockGui *qrCodeDock, *motTempDock;

	// Others
	vector<pair<int, int>> vexTeamQRCord, secondQRCord;

	// Colors
	color ownColor, oppColor;

	// Robot visual config
	bool showSimulator = mainUseSimulator;
}

// Global Functions
// Draw the brain screen continuously (thread)
void brainScreenThread() {
	// Init
	ownColor = color::purple;
	oppColor = color::purple;
	createButtons();
	createSliders();
	createDocks();
	setDockGUIs();
	initDocks();
	initQRCodes();
	mainDock->setEnabled(true);

	// Screen size is 480 px by 240 px
	while (true) {
		if (video::getCurrentVideoId() == 0) {
			// Draw the main dock
			mainDock->check();
		} else {
			// Disable the main dock until no video is playing
			mainDock->setEnabled(false);
			waitUntil(video::getCurrentVideoId() == 0);
			task::sleep(30);
			mainDock->setEnabled(true);
		}

		task::sleep(20);
	}
}

namespace {
	void _drawVexField(int x, int y, int width, int height) {
		// Tiles
		Brain.Screen.setPenWidth(2);
		Brain.Screen.setPenColor(color(128, 128, 128));
		double lengthX = width / 6.0;
		double lengthY = height / 6.0;
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				Brain.Screen.drawRectangle(x + i * lengthX, y + j * lengthY, lengthX, lengthY, color::black);
			}
		}

		// Colored elements
		// Barrier
		Brain.Screen.setPenColor(color(30, 170, 170));
		Brain.Screen.drawLine(x + lengthX, y + 2 * lengthY, x + lengthX, y + 4 * lengthY);
		Brain.Screen.drawLine(x + lengthX, y + 3 * lengthY, x + 5 * lengthX, y + 3 * lengthY);
		Brain.Screen.drawLine(x + 5 * lengthX, y + 2 * lengthY, x + 5 * lengthX, y + 4 * lengthY);

		// Opponent
		Brain.Screen.setPenColor(oppColor.rgb());
		// Diagonal
		Brain.Screen.drawLine(x + lengthX, y, x, y + lengthY);
		Brain.Screen.drawLine(x + 5 * lengthX, y, x + 6 * lengthX, y + lengthY);
		// Middle
		Brain.Screen.drawLine(x, y + 3 * lengthY, x + lengthX, y + 3 * lengthY);
		// Goal
		Brain.Screen.drawLine(x + 2 * lengthX, y + 5 * lengthY, x + 2 * lengthX, y + 6 * lengthY);
		Brain.Screen.drawLine(x + 2 * lengthX, y + 5 * lengthY, x + 4 * lengthX, y + 5 * lengthY);
		Brain.Screen.drawLine(x + 4 * lengthX, y + 5 * lengthY, x + 4 * lengthX, y + 6 * lengthY);

		// Alliance
		Brain.Screen.setPenColor(ownColor.rgb());
		// Diagonal
		Brain.Screen.drawLine(x, y + 5 * lengthY, x + lengthX, y + 6 * lengthY);
		Brain.Screen.drawLine(x + 6 * lengthX, y + 5 * lengthY, x + 5 * lengthX, y + 6 * lengthY);
		// Middle
		Brain.Screen.drawLine(x + 5 * lengthX, y + 3 * lengthY, x + 6 * lengthX, y + 3 * lengthY);
		// Goal
		Brain.Screen.drawLine(x + 2 * lengthX, y, x + 2 * lengthX, y + lengthY);
		Brain.Screen.drawLine(x + 2 * lengthX, y + lengthY, x + 4 * lengthX, y + lengthY);
		Brain.Screen.drawLine(x + 4 * lengthX, y, x + 4 * lengthX, y + lengthY);
	}

	/// @brief Draw the field and robot on a grid system
	/// @param x The left edge of the grid.
	/// @param y The top edge of the grid.
	/// @param width The horizontal length of the grid.
	/// @param height The vertical length of the grid.
	void drawCoordinate(int x, int y, int width, int height) {
		_drawVexField(x, y, width, height);

		/* Robot */
		// Robot coordinate
		double scaleX = width / 6.0; // tile to width
		double scaleY = height / 6.0;
		double botX = x + getRobotX_tiles() * scaleX;
		double botY = y + height - getRobotY_tiles() * scaleY;
		botX = genutil::clamp(botX, x + 2, x + width - 2);
		botY = genutil::clamp(botY, y + 2, y + height - 2);

		/* Heading path */
		// Get heading angle
		double botAngle_degrees = getRobotPolarAngle_degrees();
		double botAngle_radians = genutil::toRadians(botAngle_degrees);

		// Calculate x and y components
		double deltaX, deltaY;
		deltaX = genutil::signum(cos(botAngle_radians)) * width;
		deltaX = genutil::clamp(deltaX, x - botX, x + width - botX);
		deltaY = genutil::signum(sin(botAngle_radians)) * height;
		deltaY = genutil::clamp(deltaY, botY - (y + height), botY - y);

		// Scale down to the smaller one
		if (!(deltaX == 0 || deltaY == 0)) {
			// Scale y
			double derivedY = deltaX * tan(botAngle_radians);
			if (fabs(derivedY) < fabs(deltaY)) deltaY = derivedY;

			// Scale x
			double derivedX = deltaY / tan(botAngle_radians);
			if (fabs(derivedX) < fabs(deltaX)) deltaX = derivedX;
		}

		// Calculate end coordinate
		double endX = botX + deltaX;
		double endY = botY - deltaY;

		/* Draw */
		// Draw path
		Brain.Screen.setPenWidth(1);
		Brain.Screen.setPenColor(color::yellow);
		Brain.Screen.drawLine(botX, botY, endX, endY);

		// Draw robot's center
		Brain.Screen.setPenColor(color::green);
		Brain.Screen.drawCircle(botX, botY, 2, color::green);
	}

	/// @brief Draw the flywheel speed graph
	/// @param x The left edge of the graph.
	/// @param y The right edge of the graph.
	/// @param width The horizontal length of the graph.
	/// @param height The vertical length of the graph.
	void drawFlywheel(int x, int y, int width, int height) {
		// Border
		Brain.Screen.setPenWidth(2);
		Brain.Screen.setPenColor(color::white);
		Brain.Screen.setFillColor(color::transparent);
		Brain.Screen.drawRectangle(x - 2, y - 2, width + 4, height + 4);

		// Clear column
		Brain.Screen.setPenWidth(1);
		Brain.Screen.setPenColor(color::black);
		Brain.Screen.drawLine(fw_drawX, y, fw_drawX, y + height);

		// Variables
		double scale = height / 1200.0;
		double actualSpeed = genutil::clamp(0, -600, 600);
		double aimSpeed = genutil::clamp(0, -600, 600);
		fw_drawX = genutil::clamp(fw_drawX, x, x + width);

		// Real velocity
		Brain.Screen.setPenWidth(1);
		Brain.Screen.setPenColor(color::yellow);
		fw_newRealY = y + height / 2.0 - actualSpeed * scale;
		// if (fw_prevRealY >= 0 && fw_drawX > 0) {
		// 	Brain.Screen.drawLine(fw_drawX - 1, fw_prevRealY, fw_drawX, fw_newRealY);
		// } else Brain.Screen.drawPixel(fw_drawX, fw_newRealY);

		// Target velocity
		Brain.Screen.setPenColor(color::green);
		fw_newAimY = y + height / 2.0 - aimSpeed * scale;
		// if (fw_prevAimY >= 0 && fw_drawX > x) {
		// 	Brain.Screen.drawLine(fw_drawX - 1, fw_prevAimY, fw_drawX, fw_newAimY);
		// } else Brain.Screen.drawPixel(fw_drawX, fw_newAimY);

		// Normal graph
		Brain.Screen.setPenColor(color::red);
		Brain.Screen.drawPixel(fw_drawX, y + height / 2.0);
		Brain.Screen.setPenColor(color::orange);
		double gph_x, gph_y;
		double trajectoryValue;
		if (mainUseSimulator && !autonfunctions::_pathFollowStarted) {
			trajectoryValue = testTrajectoryPlan.getMotionAtTime(trajectoryTestTimer.value())[1];
		} else {
			double traj_distance = autonfunctions::_trajectoryPlan.getMotionAtTime(autonfunctions::_splinePathTimer.value())[0];
			double traj_velocity = autonfunctions::_trajectoryPlan.getMotionAtTime(autonfunctions::_splinePathTimer.value())[1];
			// double traj_tvalue = autonfunctions::_curveSampler.distanceToParam(traj_distance);
			// double traj_angularVelocity = traj_velocity * autonfunctions::_splinePath.getCurvatureAt(traj_tvalue);
			trajectoryValue = traj_velocity;
		}
		gph_x = fw_drawX;
		gph_y = y + height / 2.0 - (trajectoryValue / 7 * height);
		// printf("Vel: %.3f\n", trajectoryValue);
		Brain.Screen.drawPixel(gph_x, gph_y);

		// Update
		fw_drawX++;
		if (fw_drawX > x + width) fw_drawX = x;
		fw_prevRealY = fw_newRealY;
		fw_prevAimY = fw_newAimY;
	}

	// Graphic User Interfaces

	/// @brief Create the interactable buttons on the screen.
	void createButtons() {
		// (double x, double y, double width, double height, double radius, double outlineWeight, color outline, color fill, string msg, color textFill, void (*func)());
		// -------------------------
		// --- Main Dock Buttons ---
		// -------------------------
		static void (*mainDockDisable)(double) = [](double except) {
			for (int i = 0; i < (int) mainDockButtons.size(); i++) {
				if (i != except) {
					mainDockButtons[i]->disable();
				}
			}
		};
		static void (*autonDockDisable)(double) = [](double except) {
			for (int i = 0; i < (int) autonDockButtons.size(); i++) {
				if (i != except) {
					autonDockButtons[i]->disable();
				}
			}
		};

		// Main Dock buttons
		mainDockButtons = {};
		mainDockButtons.push_back(new ButtonGui(new Rectangle(40, 10, 80, 20, color(0, 100, 0), color(50, 50, 50), 2), "Auton", white, [] {
			mainDockDisable(0);
			mainDockButtons[0]->enable();
			mainDock_dockDock->setEnabled(false);
			autonDock->setEnabled(true);
		}));
		mainDockButtons.push_back(new ButtonGui(new Rectangle(120, 10, 80, 20, color(0, 100, 0), color(50, 50, 50), 2), "Simulator", white, [] {
			mainDockDisable(1);
			mainDockButtons[1]->enable();
			mainDock_dockDock->setEnabled(false);
			simulationDock->setEnabled(true);
		}));
		mainDockButtons.push_back(new ButtonGui(new Rectangle(200, 10, 80, 20, color(200, 0, 0), color(50, 50, 50), 2), "Extra", white, [] {
			mainDockDisable(2);
			mainDockButtons[2]->enable();
			mainDock_dockDock->setEnabled(false);
			qrCodeDock->setEnabled(true);
		}));
		mainDockButtons.push_back(new ButtonGui(new Rectangle(280, 10, 80, 20, color(100, 100, 0), color(50, 50, 50), 2), "Tempera", white, [] {
			mainDockDisable(3);
			mainDockButtons[3]->enable();
			mainDock_dockDock->setEnabled(false);
			motTempDock->setEnabled(true);
		}));

		// Auton Dock buttons
		autonDockButtons = {};
		autonDockButtons.push_back(new ButtonGui(new Rectangle(420, 60, 100, 40, color(0, 100, 200), color(50, 50, 50), 2), "Normal", white, [] {
			autonDockDisable(0);
			autonDockButtons[0]->enable();
			autonDock_dockDock->setEnabled(false);
			autonSubdock1->setEnabled(true);
		}));
		autonDockButtons.push_back(new ButtonGui(new Rectangle(420, 110, 100, 40, color(200, 0, 200), color(50, 50, 50), 2), "Safe", white, [] {
			autonDockDisable(1);
			autonDockButtons[1]->enable();
			autonDock_dockDock->setEnabled(false);
			autonSubdock2->setEnabled(true);
		}));
		autonDockButtons.push_back(new ButtonGui(new Rectangle(420, 160, 100, 40, color(200, 100, 0), color(50, 50, 50), 2), "Irregular", white, [] {
			autonDockDisable(2);
			autonDockButtons[2]->enable();
			autonDock_dockDock->setEnabled(false);
			autonSubdock3->setEnabled(true);
		}));
		autonDockButtons.push_back(new ButtonGui(new Rectangle(420, 210, 100, 40, color(100, 200, 0), color(50, 50, 50), 2), "Skills", white, [] {
			autonDockDisable(3);
			autonDockButtons[3]->enable();
			autonDock_dockDock->setEnabled(false);
			autonSubdock4->setEnabled(true);
		}));

		// -----------------------------------------
		// --- Alliance & Mode Selection Buttons ---
		// -----------------------------------------
		static void (*allianceDisable)(double) = [](double except) {
			for (int i = 0; i < (int) allianceButtons.size(); i++) {
				if (allianceButtons[i] == nullptr) continue;
				if (i == except) continue;
				allianceButtons[i]->disable();
			}
		};

		// Auton selector rectangle
		double rectCenterX = 150;
		double rectCenterY = 80;
		double selectorWidth = 120;
		double selectorHeight = 80;
		double offsetX = selectorWidth + 20;
		double offsetY = selectorHeight + 20;

		// Placeholder
		// ButtonGui *tbdButton = new ButtonGui;

		/* Normal Autons */
		// Red Up
		ButtonGui *redUp = new ButtonGui(rectCenterX, rectCenterY, selectorWidth, selectorHeight, 20, color(255, 0, 0), white, 2, "Red Up", white, [] {
			allianceDisable(0);
			allianceButtons[0]->enable();
			setAutonRunType(1, autonomousType::RedUp);
		});
		// Red Down
		ButtonGui *redDown = new ButtonGui(rectCenterX, rectCenterY + offsetY, selectorWidth, selectorHeight, 20, color(255, 0, 0), white, 2, "Red Down", white, [] {
			allianceDisable(1);
			allianceButtons[1]->enable();
			setAutonRunType(1, autonomousType::RedDown);
		});
		// Blue Up
		ButtonGui *blueUp = new ButtonGui(rectCenterX + offsetX, rectCenterY, selectorWidth, selectorHeight, 20, color(0, 0, 255), white, 2, "Blue Up", white, [] {
			allianceDisable(2);
			allianceButtons[2]->enable();
			setAutonRunType(1, autonomousType::BlueUp);
		});
		// Blue Down
		ButtonGui *blueDown = new ButtonGui(rectCenterX + offsetX, rectCenterY + offsetY, selectorWidth,selectorHeight, 20, color(0, 0, 255), white, 2, "Blue Down", white, [] {
			allianceDisable(3);
			allianceButtons[3]->enable();
			setAutonRunType(1, autonomousType::BlueDown);
		});

		/* Safe Autons */
		// Red Up
		ButtonGui *redUpSafe = new ButtonGui(rectCenterX, rectCenterY, selectorWidth, selectorHeight, 20, color(255, 0, 0), white, 2, "Red Up Safe", white, [] {
			allianceDisable(4);
			allianceButtons[4]->enable();
			setAutonRunType(1, autonomousType::RedUpSafe);
		});
		// Red Down
		ButtonGui *redDownSafe = new ButtonGui(rectCenterX, rectCenterY + offsetY, selectorWidth, selectorHeight, 20, color(255, 0, 0), white, 2, "Red Down Safe", white, [] {
			allianceDisable(5);
			allianceButtons[5]->enable();
			setAutonRunType(1, autonomousType::RedDownSafe);
		});
		// Blue Up
		ButtonGui *blueUpSafe = new ButtonGui(rectCenterX + offsetX, rectCenterY, selectorWidth, selectorHeight, 20, color(0, 0, 255), white, 2, "Blue Up Safe", white, [] {
			allianceDisable(6);
			allianceButtons[6]->enable();
			setAutonRunType(1, autonomousType::BlueUpSafe);
		});
		// Blue Down
		ButtonGui *blueDownSafe = new ButtonGui(rectCenterX + offsetX, rectCenterY + offsetY, selectorWidth, selectorHeight, 20, color(0, 0, 255), white, 2, "Blue Down Safe", white, [] {
			allianceDisable(7);
			allianceButtons[7]->enable();
			setAutonRunType(1, autonomousType::BlueDownSafe);
		});

		/* Irregular Autons */
		// Red Solo AWP
		ButtonGui *redSoloAWP = new ButtonGui(rectCenterX, rectCenterY, selectorWidth, selectorHeight, 20, color(255, 0, 0), white, 2, "Red Solo AWP", white, [] {
			allianceDisable(8);
			allianceButtons[8]->enable();
			setAutonRunType(1, autonomousType::RedSoloAWP);
		});
		// Blue Solo AWP
		ButtonGui *blueSoloAWP = new ButtonGui(rectCenterX + offsetX, rectCenterY, selectorWidth, selectorHeight, 20, color(0, 0, 255), white, 2, "Blue Solo AWP", white, [] {
			allianceDisable(9);
			allianceButtons[9]->enable();
			setAutonRunType(2, autonomousType::BlueSoloAWP);
		});

		/* Skills Autons */
		// Auton Skills 59
		ButtonGui *skillsAuton59 = new ButtonGui(rectCenterX, rectCenterY, selectorWidth, selectorHeight, 20, color(0, 200, 0), white, 1, "Skills 59", white, [] {
			ownColor = color::purple;
			oppColor = color::purple;
			allianceDisable(12);
			allianceButtons[12]->enable();
			setAutonRunType(0, autonomousType::AutonSkills59);
		});
		// Auton Skills no wall stake
		ButtonGui *skillsAutonNoWS = new ButtonGui(rectCenterX + offsetX, rectCenterY, selectorWidth, selectorHeight, 20, color(0, 200, 0), white, 1, "No Wall Stake", white, [] {
			ownColor = color::purple;
			oppColor = color::purple;
			allianceDisable(13);
			allianceButtons[13]->enable();
			setAutonRunType(0, autonomousType::AutonSkillsNoWallStake);
		});
		// Driver run Auton Skills
		ButtonGui *skillsDriverRunAuton = new ButtonGui(rectCenterX, rectCenterY + offsetY, selectorWidth, selectorHeight, 20, color(200, 0, 200), white, 1, "Drive Run Auton", white, [] {
			ownColor = color::purple;
			oppColor = color::purple;
			allianceDisable(14);
			allianceButtons[14]->enable();
			setAutonRunType(0, autonomousType::DrivingRunAutonSkills);
		});
		// Skills Driver
		ButtonGui *skillsDriver = new ButtonGui(rectCenterX + offsetX, rectCenterY + offsetY, selectorWidth, selectorHeight, 20, color(200, 0, 200), white, 1, "Drive Skills", white, [] {
			ownColor = color::purple;
			oppColor = color::purple;
			allianceDisable(15);
			allianceButtons[15]->enable();
			setAutonRunType(0, autonomousType::DrivingSkills);
		});

		// Buttons groups
		allianceButtons = {
			redUp, redDown, blueUp, blueDown,
			redUpSafe, redDownSafe, blueUpSafe, blueDownSafe,
			redSoloAWP, blueSoloAWP, nullptr, nullptr,
			skillsAuton59, skillsAutonNoWS, skillsDriverRunAuton, skillsDriver,
		};
		autonSubdock1Buttons = { redUp, redDown, blueUp, blueDown };
		autonSubdock2Buttons = { redUpSafe, redDownSafe, blueUpSafe, blueDownSafe };
		autonSubdock3Buttons = { redSoloAWP, blueSoloAWP };
		autonSubdock4Buttons = { skillsAuton59, skillsAutonNoWS, skillsDriverRunAuton, skillsDriver };
	}

	/// @brief Create the interactable sliders on the screen.
	void createSliders() {
		slider = new SliderGui(-100, 100, {}, 25, 45, 475, 45);
		slider->addSliderButton(0, new Rectangle(0, 0, 10, 10, blue, white, 2));
	}

	/// @brief Create the docks / pages on the screen.
	void createDocks() {
		// Main dock
		mainDock = new DockGui(0, 0, 480, 240, {}, {});
		mainDock_dockDock = new DockGui(0, 0, 0, 0, {}, {});
		mainDock->addEnabledFunction([] {
			mainDockButtons[0]->activateButtonFunction();
			// autonDock->setEnabled(true);
			// qrCodeDock->setEnabled(false);
			// motTempDock->setEnabled(false);
		});

		// Auton Dock
		autonDock = new DockGui(0, 20, 480, 220, {}, {});
		autonDock_dockDock = new DockGui(0, 0, 0, 0, {}, {});
		autonDock->addEnabledFunction([] {
			autonDockButtons[0]->activateButtonFunction();
		});
		autonDock->addFunction([] {
			drawInertial();
		});

		// Auton Sub-docks
		autonSubdock1 = new DockGui(90, 40, 260, 180, {}, {});
		autonSubdock2 = new DockGui(90, 40, 260, 180, {}, {});
		autonSubdock3 = new DockGui(90, 40, 260, 180, {}, {});
		autonSubdock4 = new DockGui(90, 40, 260, 180, {}, {});

		// Simulation Dock
		simulationDock = new DockGui(0, 20, 480, 220, {}, {});
		simulationDock->addFunction([] {
			drawCoordinate(20, 40, 200, 180);
			// drawCoordinate(20, 40, 100, 100);
			drawFlywheel(240, 40, 200, 180);
			// drawFlywheel(20, 160, 100, 60);
			// drawFlywheel(20, 40, 300, 160);
		});

		// QR-Code Dock
		qrCodeDock = new DockGui(0, 20, 480, 220, {}, {});
		qrCodeDock->addFunction([] {
			drawInfo();
		});
		qrCodeDock->addEnabledFunction([] {
			drawQRCodes();
		});

		// Temperature Dock
		motTempDock = new DockGui(0, 20, 480, 220, {}, {});
		motTempDock->addFunction([] {
			drawTemperature();
			drawMotorPower();
		});
	}

	/// @brief Set the GUI variables corresponding to each dock.
	void setDockGUIs() {
		// Main Dock
		for (GuiClass *gui : mainDockButtons) {
			mainDock->addGui(gui);
		}
		mainDock->addGuis({autonDock, simulationDock, qrCodeDock, motTempDock});
		mainDock_dockDock->addGuis({autonDock, simulationDock, qrCodeDock, motTempDock});

		// Auton Dock
		for (GuiClass *gui : autonDockButtons) {
			autonDock->addGui(gui);
		}
		autonDock->addGuis({autonSubdock1, autonSubdock2, autonSubdock3, autonSubdock4});
		autonDock_dockDock->addGuis({autonSubdock1, autonSubdock2, autonSubdock3, autonSubdock4});

		// Auton Subdocks
		for (GuiClass *gui : autonSubdock1Buttons) autonSubdock1->addGui(gui);
		for (GuiClass *gui : autonSubdock2Buttons) autonSubdock2->addGui(gui);
		for (GuiClass *gui : autonSubdock3Buttons) autonSubdock3->addGui(gui);
		for (GuiClass *gui : autonSubdock4Buttons) autonSubdock4->addGui(gui);

		// QR-Code Dock
		qrCodeDock->addGuis({slider});
	}

	/// @brief Initialize the docks by disabling some of them.
	void initDocks() {
		autonSubdock2->setEnabled(false);
		autonSubdock3->setEnabled(false);
		qrCodeDock->setEnabled(false);
		motTempDock->setEnabled(false);
	}

	/// @brief Set the value of the qr codes
	void initQRCodes() {
		vexTeamQRCord = {
			{2, 2}, {2, 3}, {2, 4}, {2, 5}, {2, 6}, {2, 7}, {2, 8}, {2, 11}, {2, 13}, {2, 15}, {2, 17}, {2, 20}, {2, 21}, {2, 22}, {2, 24},
			{2, 25}, {2, 26}, {2, 27}, {2, 28}, {2, 29}, {2, 30}, {3, 2}, {3, 8}, {3, 12}, {3, 13}, {3, 14}, {3, 15}, {3, 16}, {3, 17}, {3, 18},
			{3, 19}, {3, 20}, {3, 24}, {3, 30}, {4, 2}, {4, 4}, {4, 5}, {4, 6}, {4, 8}, {4, 10}, {4, 11}, {4, 15}, {4, 21}, {4, 22}, {4, 24},
			{4, 26}, {4, 27}, {4, 28}, {4, 30}, {5, 2}, {5, 4}, {5, 5}, {5, 6}, {5, 8}, {5, 10}, {5, 13}, {5, 16}, {5, 18}, {5, 19}, {5, 21},
			{5, 24}, {5, 26}, {5, 27}, {5, 28}, {5, 30}, {6, 2}, {6, 4}, {6, 5}, {6, 6}, {6, 8}, {6, 10}, {6, 13}, {6, 18}, {6, 19}, {6, 20},
			{6, 21}, {6, 24}, {6, 26}, {6, 27}, {6, 28}, {6, 30}, {7, 2}, {7, 8}, {7, 10}, {7, 12}, {7, 13}, {7, 14}, {7, 17}, {7, 18}, {7, 19},
			{7, 22}, {7, 24}, {7, 30}, {8, 2}, {8, 3}, {8, 4}, {8, 5}, {8, 6}, {8, 7}, {8, 8}, {8, 10}, {8, 12}, {8, 14}, {8, 16}, {8, 18},
			{8, 20}, {8, 22}, {8, 24}, {8, 25}, {8, 26}, {8, 27}, {8, 28}, {8, 29}, {8, 30}, {9, 10}, {9, 14}, {9, 16}, {9, 17}, {9, 22}, {10, 2},
			{10, 4}, {10, 5}, {10, 6}, {10, 7}, {10, 8}, {10, 11}, {10, 13}, {10, 18}, {10, 19}, {10, 21}, {10, 24}, {10, 25}, {10, 26}, {10, 27}, {10, 28},
			{11, 2}, {11, 3}, {11, 5}, {11, 6}, {11, 11}, {11, 12}, {11, 13}, {11, 15}, {11, 16}, {11, 19}, {11, 20}, {11, 21}, {11, 22}, {11, 23}, {11, 24},
			{11, 25}, {11, 26}, {11, 30}, {12, 2}, {12, 4}, {12, 5}, {12, 6}, {12, 8}, {12, 10}, {12, 11}, {12, 13}, {12, 15}, {12, 16}, {12, 17}, {12, 22},
			{12, 23}, {12, 24}, {13, 3}, {13, 4}, {13, 5}, {13, 11}, {13, 13}, {13, 15}, {13, 18}, {13, 25}, {13, 27}, {13, 29}, {14, 2}, {14, 8}, {14, 16},
			{14, 17}, {14, 18}, {14, 19}, {14, 20}, {14, 27}, {14, 28}, {15, 2}, {15, 3}, {15, 4}, {15, 5}, {15, 9}, {15, 14}, {15, 19}, {15, 20}, {15, 21},
			{15, 22}, {15, 23}, {15, 24}, {15, 26}, {15, 30}, {16, 2}, {16, 4}, {16, 5}, {16, 6}, {16, 8}, {16, 11}, {16, 12}, {16, 14}, {16, 17}, {16, 18},
			{16, 19}, {16, 24}, {16, 25}, {16, 27}, {16, 28}, {17, 2}, {17, 3}, {17, 5}, {17, 7}, {17, 10}, {17, 12}, {17, 16}, {17, 17}, {17, 21}, {17, 23},
			{17, 24}, {17, 25}, {17, 26}, {17, 29}, {18, 6}, {18, 7}, {18, 8}, {18, 11}, {18, 13}, {18, 14}, {18, 18}, {18, 19}, {18, 22}, {18, 25}, {18, 27},
			{18, 28}, {19, 2}, {19, 3}, {19, 12}, {19, 13}, {19, 15}, {19, 16}, {19, 21}, {19, 22}, {19, 24}, {19, 25}, {19, 26}, {19, 28}, {19, 30}, {20, 2},
			{20, 4}, {20, 6}, {20, 7}, {20, 8}, {20, 9}, {20, 11}, {20, 12}, {20, 14}, {20, 15}, {20, 16}, {20, 17}, {20, 22}, {20, 23}, {20, 25}, {20, 28},
			{21, 2}, {21, 4}, {21, 5}, {21, 10}, {21, 14}, {21, 15}, {21, 18}, {21, 20}, {21, 21}, {21, 22}, {21, 24}, {21, 25}, {21, 26}, {21, 29}, {22, 2},
			{22, 6}, {22, 8}, {22, 9}, {22, 10}, {22, 16}, {22, 17}, {22, 18}, {22, 19}, {22, 22}, {22, 23}, {22, 24}, {22, 25}, {22, 26}, {22, 28}, {22, 29},
			{22, 30}, {23, 10}, {23, 12}, {23, 13}, {23, 14}, {23, 20}, {23, 22}, {23, 26}, {23, 27}, {23, 28}, {23, 29}, {23, 30}, {24, 2}, {24, 3}, {24, 4},
			{24, 5}, {24, 6}, {24, 7}, {24, 8}, {24, 11}, {24, 14}, {24, 17}, {24, 18}, {24, 21}, {24, 22}, {24, 24}, {24, 26}, {24, 27}, {24, 28}, {25, 2},
			{25, 8}, {25, 10}, {25, 14}, {25, 16}, {25, 17}, {25, 21}, {25, 22}, {25, 26}, {25, 29}, {26, 2}, {26, 4}, {26, 5}, {26, 6}, {26, 8}, {26, 10},
			{26, 11}, {26, 13}, {26, 18}, {26, 19}, {26, 22}, {26, 23}, {26, 24}, {26, 25}, {26, 26}, {26, 28}, {27, 2}, {27, 4}, {27, 5}, {27, 6}, {27, 8},
			{27, 10}, {27, 12}, {27, 13}, {27, 14}, {27, 15}, {27, 16}, {27, 20}, {27, 21}, {27, 25}, {27, 27}, {27, 28}, {27, 29}, {27, 30}, {28, 2}, {28, 4},
			{28, 5}, {28, 6}, {28, 8}, {28, 10}, {28, 13}, {28, 15}, {28, 17}, {28, 18}, {28, 20}, {28, 24}, {28, 25}, {28, 26}, {28, 27}, {28, 28}, {28, 29},
			{29, 2}, {29, 8}, {29, 11}, {29, 12}, {29, 13}, {29, 14}, {29, 16}, {29, 17}, {29, 18}, {29, 22}, {29, 25}, {29, 27}, {29, 29}, {30, 2}, {30, 3},
			{30, 4}, {30, 5}, {30, 6}, {30, 7}, {30, 8}, {30, 10}, {30, 12}, {30, 16}, {30, 19}, {30, 21}, {30, 23}, {30, 25}, {30, 26}, {30, 27}, {30, 28}
		};
		secondQRCord = {
			{2, 2}, {2, 3}, {2, 4}, {2, 5}, {2, 6}, {2, 7}, {2, 8}, {2, 13}, {2, 14}, {2, 18}, {2, 20}, {2, 21}, {2, 22}, {2, 24}, {2, 25},
			{2, 26}, {2, 27}, {2, 28}, {2, 29}, {2, 30}, {3, 2}, {3, 8}, {3, 11}, {3, 13}, {3, 17}, {3, 18}, {3, 19}, {3, 20}, {3, 24}, {3, 30},
			{4, 2}, {4, 4}, {4, 5}, {4, 6}, {4, 8}, {4, 10}, {4, 11}, {4, 14}, {4, 16}, {4, 17}, {4, 18}, {4, 21}, {4, 22}, {4, 24}, {4, 26},
			{4, 27}, {4, 28}, {4, 30}, {5, 2}, {5, 4}, {5, 5}, {5, 6}, {5, 8}, {5, 10}, {5, 12}, {5, 13}, {5, 15}, {5, 16}, {5, 17}, {5, 19},
			{5, 21}, {5, 24}, {5, 26}, {5, 27}, {5, 28}, {5, 30}, {6, 2}, {6, 4}, {6, 5}, {6, 6}, {6, 8}, {6, 10}, {6, 11}, {6, 15}, {6, 18},
			{6, 19}, {6, 20}, {6, 21}, {6, 24}, {6, 26}, {6, 27}, {6, 28}, {6, 30}, {7, 2}, {7, 8}, {7, 10}, {7, 12}, {7, 14}, {7, 16}, {7, 17},
			{7, 19}, {7, 22}, {7, 24}, {7, 30}, {8, 2}, {8, 3}, {8, 4}, {8, 5}, {8, 6}, {8, 7}, {8, 8}, {8, 10}, {8, 12}, {8, 14}, {8, 16},
			{8, 18}, {8, 20}, {8, 22}, {8, 24}, {8, 25}, {8, 26}, {8, 27}, {8, 28}, {8, 29}, {8, 30}, {9, 10}, {9, 14}, {9, 16}, {9, 22}, {10, 2},
			{10, 4}, {10, 5}, {10, 6}, {10, 7}, {10, 8}, {10, 11}, {10, 15}, {10, 17}, {10, 18}, {10, 19}, {10, 21}, {10, 24}, {10, 25}, {10, 26}, {10, 27},
			{10, 28}, {11, 2}, {11, 4}, {11, 5}, {11, 6}, {11, 7}, {11, 9}, {11, 10}, {11, 12}, {11, 15}, {11, 18}, {11, 19}, {11, 20}, {11, 21}, {11, 22},
			{11, 23}, {11, 24}, {11, 25}, {11, 26}, {11, 30}, {12, 4}, {12, 6}, {12, 8}, {12, 10}, {12, 11}, {12, 13}, {12, 14}, {12, 15}, {12, 16}, {12, 17},
			{12, 18}, {12, 22}, {12, 23}, {12, 24}, {13, 2}, {13, 11}, {13, 16}, {13, 25}, {13, 27}, {13, 29}, {14, 7}, {14, 8}, {14, 9}, {14, 12}, {14, 14},
			{14, 17}, {14, 19}, {14, 20}, {14, 27}, {14, 28}, {15, 2}, {15, 3}, {15, 11}, {15, 12}, {15, 14}, {15, 15}, {15, 18}, {15, 19}, {15, 20}, {15, 21},
			{15, 22}, {15, 23}, {15, 24}, {15, 26}, {15, 30}, {16, 3}, {16, 4}, {16, 6}, {16, 8}, {16, 11}, {16, 14}, {16, 15}, {16, 16}, {16, 17}, {16, 18},
			{16, 19}, {16, 24}, {16, 25}, {16, 27}, {16, 28}, {17, 7}, {17, 11}, {17, 13}, {17, 14}, {17, 15}, {17, 17}, {17, 18}, {17, 21}, {17, 23}, {17, 24},
			{17, 25}, {17, 26}, {17, 29}, {18, 3}, {18, 5}, {18, 8}, {18, 16}, {18, 17}, {18, 19}, {18, 22}, {18, 25}, {18, 27}, {18, 28}, {19, 2}, {19, 5},
			{19, 7}, {19, 12}, {19, 14}, {19, 18}, {19, 21}, {19, 22}, {19, 24}, {19, 25}, {19, 26}, {19, 28}, {19, 30}, {20, 2}, {20, 4}, {20, 6}, {20, 7},
			{20, 8}, {20, 9}, {20, 10}, {20, 17}, {20, 18}, {20, 20}, {20, 22}, {20, 23}, {20, 25}, {20, 28}, {21, 2}, {21, 4}, {21, 6}, {21, 7}, {21, 10},
			{21, 11}, {21, 13}, {21, 16}, {21, 20}, {21, 21}, {21, 22}, {21, 24}, {21, 25}, {21, 26}, {21, 29}, {22, 2}, {22, 7}, {22, 8}, {22, 9}, {22, 10},
			{22, 12}, {22, 14}, {22, 17}, {22, 19}, {22, 22}, {22, 23}, {22, 24}, {22, 25}, {22, 26}, {22, 28}, {22, 29}, {22, 30}, {23, 10}, {23, 13}, {23, 15},
			{23, 16}, {23, 18}, {23, 20}, {23, 22}, {23, 26}, {23, 27}, {23, 28}, {23, 29}, {23, 30}, {24, 2}, {24, 3}, {24, 4}, {24, 5}, {24, 6}, {24, 7},
			{24, 8}, {24, 13}, {24, 14}, {24, 15}, {24, 16}, {24, 17}, {24, 18}, {24, 20}, {24, 21}, {24, 22}, {24, 24}, {24, 26}, {24, 27}, {24, 28}, {25, 2},
			{25, 8}, {25, 10}, {25, 12}, {25, 14}, {25, 15}, {25, 17}, {25, 21}, {25, 22}, {25, 26}, {25, 30}, {26, 2}, {26, 4}, {26, 5}, {26, 6}, {26, 8},
			{26, 10}, {26, 11}, {26, 13}, {26, 14}, {26, 16}, {26, 19}, {26, 22}, {26, 23}, {26, 24}, {26, 25}, {26, 26}, {26, 28}, {27, 2}, {27, 4}, {27, 5},
			{27, 6}, {27, 8}, {27, 10}, {27, 11}, {27, 12}, {27, 14}, {27, 15}, {27, 18}, {27, 21}, {27, 25}, {27, 27}, {27, 28}, {27, 29}, {27, 30}, {28, 2},
			{28, 4}, {28, 5}, {28, 6}, {28, 8}, {28, 10}, {28, 11}, {28, 13}, {28, 17}, {28, 18}, {28, 19}, {28, 20}, {28, 24}, {28, 25}, {28, 26}, {28, 27},
			{28, 28}, {28, 29}, {29, 2}, {29, 8}, {29, 12}, {29, 14}, {29, 15}, {29, 16}, {29, 18}, {29, 22}, {29, 25}, {29, 27}, {29, 29}, {30, 2}, {30, 3},
			{30, 4}, {30, 5}, {30, 6}, {30, 7}, {30, 8}, {30, 10}, {30, 12}, {30, 18}, {30, 21}, {30, 23}, {30, 25}, {30, 26}, {30, 27}, {30, 28}
		};
	}

	// Getters
	double getRobotX_tiles() {
		if (showSimulator) {
			return robotSimulator.position.x;
		}
		return mainOdometry.getX();
	}
	double getRobotY_tiles() {
		if (showSimulator) {
			return robotSimulator.position.y;
		}
		return mainOdometry.getY();
	}
	double getRobotPolarAngle_degrees() {
		if (showSimulator) {
			return genutil::toDegrees(robotSimulator.angularPosition);
		}
		return angle::swapFieldPolar_degrees(mainOdometry.getLookFieldAngle_degrees());
	}

	// Draws info
	void drawInfo() {
		Brain.Screen.setPenColor(color::green);
		Brain.Screen.setFillColor(color::transparent);
		Brain.Screen.printAt(10, 35, 1, "X: %08.3f, Y: %08.3f", getRobotX_tiles(), getRobotY_tiles());

		Brain.Screen.setPenColor(color::green);
		Brain.Screen.setFillColor(color::transparent);
		Brain.Screen.printAt(240, 35, 1, "BotAng Polar: %07.3f", getRobotPolarAngle_degrees());

		// Brain.Screen.setPenColor(color(255, 190, 0));
		// Brain.Screen.setFillColor(color::transparent);
		// Brain.Screen.printAt(390, 35, 1, "MoveDB: %d", moveDB);
	}
	void drawQRCodes() {
		// 96969Y Instagram QR Code
		drawQRCode(53, 80, 160, vexTeamQRCord, color(200, 0, 0), white);

		// Another QR Code
		drawQRCode(266, 80, 160, secondQRCord, color(200, 0, 200), white);
	}
	void drawQRCode(double x, double y, double width, vector<pair<int, int> > &QRCode, color bgCol, color qrCol) {
		// Background
		Brain.Screen.setPenWidth(0);
		Brain.Screen.setFillColor(bgCol);
		Brain.Screen.drawRectangle(x, y, width, width);

		// QR Code
		Brain.Screen.setFillColor(qrCol);
		for (pair<int, int> cord : QRCode) {
			int i = cord.first * (width / 33.0) + (width / 66.0);
			int j = cord.second * (width / 33.0) + (width / 66.0);
			Brain.Screen.drawCircle(x + i, y + j, width / 66.0);
		}
	}
	void drawTemperature() {
		Brain.Screen.setPenColor(color::white);
		Brain.Screen.setFillColor(color::transparent);
		Brain.Screen.setFont(fontType::mono20);

		// Chassis temperatures
		double leftAC = LeftMotorA.temperature(celsius);
		double leftBC = LeftMotorB.temperature(celsius);
		double leftCC = LeftMotorC.temperature(celsius);
		// double leftDC = LeftMotorD.temperature(celsius);
		double rightAC = RightMotorA.temperature(celsius);
		double rightBC = RightMotorB.temperature(celsius);
		double rightCC = RightMotorC.temperature(celsius);
		// double rightDC = RightMotorD.temperature(celsius);
		Brain.Screen.printAt(10, 40, 1, "Left1: %07.3f°C, Rght1: %07.3f°C", leftAC, rightAC);
		Brain.Screen.printAt(10, 65, 1, "Left2: %07.3f°C, Rght2: %07.3f°C", leftBC, rightBC);
		Brain.Screen.printAt(10, 90, 1, "Left3: %07.3f°C, Rght3: %07.3f°C", leftCC, rightCC);
		// Brain.Screen.printAt(10, 115, 1, "Left4: %07.3f°C, Rght4: %07.3f°C", leftDC, rightDC);

		// Other temperatures
		double intake1 = IntakeMotor1.temperature(celsius);
		double intake2 = IntakeMotor2.temperature(celsius);
		double arm1 = ArmMotor.temperature(celsius);
		Brain.Screen.printAt(10, 115, 1, "Intk1: %07.3f°C, Intk2: %07.3f°C", intake1, intake2);
		Brain.Screen.printAt(10, 140, 1, "Arm_1: %07.3f°C", arm1);
	}
	void drawMotorPower() {
		// Get power info
		double leftA_watt = LeftMotorA.power(watt);
		double leftB_watt = LeftMotorB.power(watt);
		double leftC_watt = LeftMotorC.power(watt);
		double rightA_watt = RightMotorA.power(watt);
		double rightB_watt = RightMotorB.power(watt);
		double rightC_watt = RightMotorC.power(watt);

		double leftAvg_watt = (leftA_watt + leftB_watt + leftC_watt) / 3.0;
		double rightAvg_watt = (rightA_watt + rightB_watt + rightC_watt) / 3.0;

		double intake1_watt = IntakeMotor1.power(watt);
		double intake2_watt = IntakeMotor2.power(watt);

		// Draw info
		Brain.Screen.setPenColor(color::white);
		Brain.Screen.setFillColor(color::transparent);
		Brain.Screen.setFont(fontType::mono20);
		Brain.Screen.printAt(10, 190, 1, "Lavg: %07.3f W, Ravg: %07.3f W", leftAvg_watt, rightAvg_watt);
		Brain.Screen.printAt(10, 215, 1, "Intk1: %07.3f W, Intk2: %07.3f W", intake1_watt, intake2_watt);
	}
	void drawInertial() {
		Brain.Screen.setPenColor(color::green);
		Brain.Screen.setFillColor(color::transparent);
		Brain.Screen.printAt(10, 35, 1, "%07.3f", getRobotPolarAngle_degrees());
	}
}
