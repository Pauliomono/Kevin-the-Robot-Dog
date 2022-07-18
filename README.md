# Kevin-the-Robot-Dog
 
 

Kevin the Robot Dog is a quadruped I created for a final design project at the University of Notre Dame.

Check him out on Youtube:

[![IMAGE ALT TEXT HERE](Screenshot.png)](https://www.youtube.com/watch?v=h35KHhktcEo&t)

For more information check out the final report written on Kevin.

- See Final_Report/Automation_and_Controls_Final_Project_Report.pdf

Recent Changes:

- complete overhaul of code: migrated to PlatformIO in vscode, reorganised functions into separate files
- initial addition of remote control using bluetooth serial (can change modes, walk forward/back & steer)
- improved interpolation for foot positions: switched from linear interpolation to cubic spline interpolation for improved velocity and acceleration control

Active Development:

- adding more remote functionality: telemetry reports to remote, remote realtime PID tuning
- tweaking walking gait/motion to get smoother motion
- adding trim mode (currently drifts instead of staying in one place)

Future plans include:

- Bluetooth remote control
- reworking the entire wiring system (including custom PCB) and battery monitoring circuitry
- overhaul balance control with a more complex model (using analyital dynamics to generate EOMs) of the system & additional tuning
- possible force control? either though current sensors on servos or pressure sensors on feet
- googley eyes to make him cuter
- expanded motion capabilities (jumping, rough surfaces & inclines, possibly stairs)
