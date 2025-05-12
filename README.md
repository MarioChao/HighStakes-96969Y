# VEX High Stakes

The repository for PAS1's newest robot program in 2024-25 VEX Robotics - High Stakes.

Our team id is [96969Y](https://www.robotevents.com/teams/VRC/96969Y). Check out [our website](https://mariochao.github.io/vex-pas1/)!


## Configuration

Modify robot configurations in [robot-config.cpp](./src/robot-config.cpp).

Modify robot specifications, chassis sensors, odometry, and auton settings in [chassis-config.cpp](./src/chassis-config.cpp).

For more details, check [configuration tutorial](./configuration.md).

> **Dev Note:**
>
> Some dummy sensors will cause error (crash) if you read its value immediately after starting the program.<br>
> Examples: Optical shaft encoder, Inertial sensor


## About

Our main control program is organized into the categories [`Autonomous`](./include/Autonomous/), [`Controller`](./include/Controller/), [`Mechanics`](./include/Mechanics/), and [`MatchSequence`](./include/MatchSequence/).

The utility functions are organized into [`Aespa-Lib`](./include/Aespa-Lib/), [`Pas1-Lib`](./include/Pas1-Lib/), [`Sensors`](./include/Sensors/), [`Utilities`](./include/Utilities/), and [`Gfx-Lib`](./include/Gfx-Lib/).

[Cosmetics](./include/Cosmetics/) related things are organized into the categories [`Graphics`](./include/Cosmetics/Graphics/), [`LedLight`](./include/Cosmetics/LedLight/), [`Simulation`](./include/Cosmetics/Simulation/), and [`Videos`](./include/Cosmetics/Videos/).


# Extra


## Resources

Here's some tools that we used during our robot's development.
- [VEX Path Planner (Scratch tool)](https://scratch.mit.edu/projects/921506148/) for path planning and autonomous
- [Image Array (Web tool)](https://mariochao.github.io/image-array/) for converting images to arrays for drawing
- [CyberChef](https://gchq.github.io/CyberChef/) for mirroring autonomous routes
