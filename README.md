Fake Unity Server
--
This project provides an emulator for the Unity VR environment that TRAADRE uses, for debugging operator/experimental control UI
All dynamics are implemented poorly with a 100Hz Euler integration on a unicycle model
Don't run out of fuel. You will stop.

This node subscribes to /joy and lets you drive around via linear/angular velocity control sourced from axes[0] and axes[1]

