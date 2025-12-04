# SimPhy
Multi-physics simulation software.
Developed By Maxence Barré, 2025

## Introduction and installation
This software offers an easy way to simulate complex interactions between physical bodies by integrating the 2nd Newton's law.
You will also be able to plot in real time the trajectories and other physical values.

### What to do with SimPhy?
Below is a list (i expect to be exhaustive) of the stuff you can do with SimPhy:
- simulate as many body as you want
- consider interactions such as spring force, gravitationnal force, torque
- modelize body represented by a single-point and by a local basis
- plot in real time the results of the simulation

### How to install?
After downloading the project from Github (for example), you have two solutions:
- you can add by hand all the modules (using pip)
- or you can use ```poetry install``` to install directly all the right dependencies.

### Quick Launch
To check that everything works well, quite a few scenarii are already implemented to present how the simulator behaves:
- choose one file from the folder ```scenarii_examples```. Note that the bigger the number is, the more "complex" the simulation is (either includes more bodies or more complex interactions)
- in the file ```core_calculation/main_simulation.py```, change the value of json_file and write the path to the json simulation you choose
- then run the simulation, some print should appear and the program will wait after printing "Press Enter to start the simulation..."
- at this point, you have two solutions:
  - either you hit run, the simulation runs by itself and does not plot anything
  - or you launch the GUI (what will be done afterwards)
- run the ```simulation_GUI/main_gui.py``` file, some prints will appear and a GUI window will pop. On it, you will at least have a 3D plot with the position of the body in space, and additionnal GUIs depending on the scenarii (eg, the energy evolution, the euler angle ...)
- in the cmd terminal of main_simulation, hit enter and the simulation will be launched.
- Note that the plotting are done in real time in the other 3D (and 2D) window


## Description of the project
This section will be useful for those who want to develop their own version of this program or that would want to create their own scenarii.
### Folder Description
The two main folders are :
- core_calculation : the folder in which you will find every single file needed to carry out the simulation.
- simulation_GUI : the folder that will carry out real-time plottings of result.

Then, others folder might be found:
- scenarii_examples : a folder full of json file gathering the json files of examples of scenarii.
- hands_on : personal tries done before implementation (might be deleted one day)
- benchmark : a folder in which you can find report of benchmark that allows you to test the exeuction speed on different computers.


### How to create your own simulations
For now, every single simulation that can be done can be written down in a json file. This helps how to implement and modify the simulations parameters.
Several sections can be found in a json file, each of them defining a specific part of the simulation:
- parameters : the general parameters of the simulation (step dt, duration, and speed of the simulation). Note : the parameter speed_simulation is used as a way to change the real-time plotting. If set to "max" or "inf" then the simulation will go as fast as possible. If set to a real lambda then, the simulation will go lambda times faster than reality : so lambda =2 then simulation will be as twice as fast ; but lambda= 0.5 will give a simulation going twice slower than reality.
- objects : it will be a dictionnary of every single objects that will play a role in the simulation. See next sub-section to have more info on this.
- forces : TODO some stuff need to be modified
- plotting : thanks to this you can ask to plot specific. Choose in ["3D", "angular_velocity", "energy", "euler_angle"]


### How to define a Body?
TODO

## Roadmap
Below is a list of the next step that will be (probably) implemented shortly:
- [ ] solid body with local basis implementation (using quaternions)
- [ ] torque calculation of "global applied" forces on a body
- [ ] assemblies of body in a bigger body (and "compilation" of the resulted body, eg : the change of inertia matrix)
- [ ] implementation of testers-programs to check the good implementation and/or good (enough) convergence (by comparing to known theorical results)
- [ ] adding different actuators (such as motors or thrust motors) to change caracteristics of a body
- [ ] adding possibilities to PID an actuator (see previous point)
- [ ] add aerodynamic considerations (coefficients)