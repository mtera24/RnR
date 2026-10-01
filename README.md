# RnR

RnR (Record and Replay) is a system for recording and replaying motions using a haptic device.

This repository contains the existing C/C++ implementation of RnR for the 3D Systems Touch device.

## Current development environment

The following environment has been confirmed to build and start the existing RnR application.

- Windows 11
- Visual Studio Community 2022
- Desktop development with C++
- MSVC v142 (Visual Studio 2019 toolset)
- Windows SDK
- OpenHaptics SDK 3.5.0
- Touch Device Driver 2023.10.11
- 3D Systems Touch

OpenHaptics environment variable:

```text
OH_SDK_BASE=C:\OpenHaptics\Developer\3.5.0
```
## Visual Studio project
Current project files:
- Guide_n_ReadMotion.sln
- Guide_n_ReadMotion.vcxproj
- Guide_n_ReadMotion.cpp
Current build configuration:
- Configuration: Debug
- Platform: x64
- Platform Toolset: v142
## Confirmed operation

The following operations have been confirmed on the development PC:

1. The Visual Studio project builds successfully.
2. Guide_n_ReadMotion.exe is generated.
3. Touch is detected by Touch Smart Setup.
4. Stylus movement is detected correctly.
5. RnR detects the device as Touch.0
6. Calibration completes.
7. The RnR command menu is displayed.

Record and Replay operations have not yet been fully verified in this reconstructed environment.
## Notes

The current Visual Studio project and source file names still use the historical name Guide_n_ReadMotion / Guide_n_Record_v2_2.
They will be renamed to RnR in a separate change after the working baseline has been preserved.
The initial working baseline is preserved in Git before making structural changes.
