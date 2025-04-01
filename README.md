# VEX Highstakes

The repository for team 96969Y's newest robot program in 2024-25 VEX Robotics - High Stakes.

## Configuration

Modify robot configurations in [robot-config.cpp](./src/robot-config.cpp).

Modify robot specifications in [global-vars.cpp](./src/global-vars.cpp) (modify `botInfo`).

## Chassis Configuration

Modify chassis sensors, odometry, and auton settings in [global-vars.cpp](./src/global-vars.cpp).

Notes:

- Some dummy sensors create an error if you read its value immediately after starting the program<br>
	Examples:
	- Optical shaft encoder
	- Inertial sensor

## About

Our main control program is organized into the categories [`Autonomous`](./include/Autonomous/), [`Controller`](./include/Controller/), [`Mechanics`](./include/Mechanics/), and [`MatchSequence`](./include/MatchSequence/).

Our utility functions are organized into [`Aespa-Lib`](./include/Aespa-Lib/), [`Sensors`](./include/Sensors/), and [`Utilities`](./include/Utilities/).

Our cosmetic program is organized into the categories [`Graphics`](./include/Graphics/), [`Simulation`](./include/Simulation/), [`Videos`](./include/Videos/), and [`LedLight`](./include/LedLight/).

Our team id is [96969Y](https://www.robotevents.com/teams/VRC/96969Y), and our website is [here](https://mariochao.github.io/vex-pas1/).

# Extra

## Resources

Here's some tools that we used during our robot's development.
- [VEX Path Planner (Scratch tool)](https://scratch.mit.edu/projects/921506148/) for path planning and autonomous
- [Image Array (Web tool)](https://mariochao.github.io/image-array/) for converting images to arrays for drawing
