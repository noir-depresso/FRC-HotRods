# MAXSwerve C++ Template v2026.0

See [the online changelog](https://github.com/REVrobotics/MAXSwerve-Cpp-Template/blob/main/CHANGELOG.md) for information about updates to the template that may have been released since you created your project.

## Description

A template project for an FRC swerve drivetrain that uses REV MAXSwerve Modules.

Note that this template is designed for a drivetrain composed of four MAXSwerve Modules, each configured with two SPARKS MAX, a NEO as the driving motor, a NEO 550 as the turning motor, and a REV Through Bore Encoder as the absolute turning encoder. If you are using SPARK Flex for either the drive motor or turning motor, you will need to update the classes accordingly.

To get started, make sure you have calibrated the zero offsets for the absolute encoders in Hardware Client 2 using the `Absolute Encoder` utility under the associated turning SPARK devices.

## Prerequisites

* SPARK MAX Firmware v26.1.0
* REVLib v2026.0.0

## Configuration

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Configs.h` and `Constants.h` files.

## Build and test in restricted environments

This repository uses the Gradle wrapper (`gradlew`) and 2026 WPILib tooling.

1. Ensure wrapper is executable:
   - `chmod +x gradlew`
2. Run tests/build with wrapper:
   - `./gradlew test`
   - `./gradlew build`

If your environment blocks downloads from `https://services.gradle.org`, use one of these supported paths:

- allowlist `services.gradle.org` in your proxy/firewall,
- configure an internal mirror for Gradle distributions, or
- pre-seed Gradle `9.2.1` in CI/dev images so wrapper resolution is offline-capable.

Minimum pre-merge verification contract:

- build compiles (`./gradlew build`), and
- unit tests pass (`./gradlew test`).

