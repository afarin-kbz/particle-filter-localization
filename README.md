# Particle Filter Localization

This repository contains MATLAB code that implements a particle filter for robot localization. The code simulates a robot's motion, updates particle states based on control inputs and measurements, and visualizes the process through an animation.

## Features

- Simulates robot motion with control inputs and disturbances.
- Implements a particle filter for state estimation.
- Visualizes the robot's path, particle distribution, and estimated state.
- Generates an animation of the localization process.

## Getting Started

### Prerequisites

- MATLAB R2021a or later

### Installation

1. Clone the repository:
   ```sh
   git clone https://github.com/yourusername/particle-filter-localization.git
   cd particle-filter-localization
2. Open MATLAB and navigate to the cloned directory.

## Code Overview
- **Initialization**: Sets up simulation parameters, initial state, control inputs, disturbance model, measurement noise, and particle filter parameters.
- Main Simulation Loop:
  - Updates the robot's state based on control inputs and disturbances.
  - Takes measurements and updates the particles' states.
  - Computes the importance weights of the particles and resamples them.
  - Estimates the current state from the particle set.
- Visualization: Plots the feature map, true state, particles, and error ellipses, and captures frames for the animation.


## Key Functions
- closestfeature(map, x): Finds the closest feature from the map to the current state.
- error_ellipse(S, mu, p): Plots an error ellipse based on the covariance matrix S and mean mu.