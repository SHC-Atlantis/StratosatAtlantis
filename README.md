# UAH Space Hardware Club - 2 Month

# Stratosat Team Atlantis

Team Members: Ulrich, Tristan, Josh, Gabe, Ashlyn, Cait, Maxx, Nathan, Logan, Adelle
Software Members: Cait, Maxx, Nathan, Logan

This challenge was an introduction to the design and execution of a high-altitude balloon flight, as well as an introduction to control systems, cold-gas propulsion, and active stabilization. Teams were tasked with creating a balloon payload for collecting data during a balloon flight, with the unique challenge of recording stable video. Teams developed active payload stabilization systems utilizing compressed gas and a series of thrusters to control payload rotation.

Learn more about the two month program [here](https://space.uah.edu/two-month)!
## The Process
We went through a PDR (preliminary design review), MRR (mission readiness review), FRR (flight readiness review), and PFR (post flight review) over the course of a little over 2 months. At each review we received notes for improvement from our mentors who are older members of Space Hardware Club.

## State Machine
![](https://github.com/SHC-Atlantis/StratosatAtlantis/blob/d1721f956c90735a1c8ef99f3671287a2d5a4692/Images/finalStateMachine.png?raw=true "State Machine")

## Software Flowchart
![](https://github.com/SHC-Atlantis/StratosatAtlantis/blob/d1721f956c90735a1c8ef99f3671287a2d5a4692/Images/finalFlowChart.png?raw=true "Flowchart")


## Hardware
This code was written to work on a custom PCB created by the electrical team. It has a surface mounted ICP-20100, as well as a teensy 4.0, BME280, BNO055, and OpenLog. We also have pins for 2 solenoids and 2 LEDs.
## Flight Day
On flight day we changed our stabilization altitude to 18,000 meters to better reflect the flight predictions. Shortly after we turned on the payload the LED lights began to blink so we were confident the code had successfully initialized. At that point we made sure the camera was also turned on and all of our responsibilities for flight day were done. Upon recovery the following day the payload was dead as expected. 

## The Results
When processing the data stored as a CSV on our payload SD card we found over a million lines of data, and about half of those were from during flight. We were able to create graphs that showed successful stabilization and orientation to 45 degrees. Our state machine worked through stabilization, but did not make it past. The most important feature was making it to stabilization, so we aren't counting that as a failure. We had overall mission success, with notes for minor improvements.
  
![](https://github.com/SHC-Atlantis/StratosatAtlantis/blob/main/Images/GPSandAlt.png?raw=true)
![](https://github.com/SHC-Atlantis/StratosatAtlantis/blob/main/Images/rotAccVsTime.png?raw=true)
![](https://github.com/SHC-Atlantis/StratosatAtlantis/blob/main/Images/YawAltVsTime.png?raw=true)
