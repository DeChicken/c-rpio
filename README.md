# Summary
The c-rpio library aims to provide functions for utilizing the functionality of the 40-pin GPIO header on the Raspberry Pi in the C programming language. As of release 2.0.0, this library is only intended for the Raspberry Pi 4 Model B. *Do not use this library on older Raspberry Pi models.*

## Notes
- This library was written such that any function that uses the same name as an Arduino function operates in a similar manner.
- Programs using these functions to change register values must be run as the root user (sudo), otherwise the desired actions will not occur.

## Releases
- 1.0.0 - Support for Raspberry Pi 4 Model B *ONLY*. Functionality for pinMode, digitalRead, and digitalWrite.
- 2.0.0 - Added analogWrite and two example programs
- 2.0.1 - Fixed bug that did not allow 100% duty cycle in analogWrite

Shoutout to @stianeikeland and those who worked on https://github.com/stianeikeland/go-rpio for the naming inspiration!