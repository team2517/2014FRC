** Team 2517: The Green Wrenches 2014 code**
--------------------------------------------

**git_bash-template.bat**

Because we share the dev laptop, and many of the students are un-familiar with git (or anything source control related) we have a helper .bat file that will "log them in" as their git user. Mostly just a different $USER and therefore a new $HOME that they can have the different `.gitconfig` with different user/emails.

**simulation code**

In the subfolder "simulations" we have python code that we used to test or do proof-of-concept on various maths or algorithms related to the season.

Most simulations are written in python 2.7 and pygame 1.9.1 (other versions may work, but are untested)

For windows users see installing/using python portable [here](http://portablepython.com/wiki/PortablePython2.7.5.1/), this version of python includes basically all default packages a python user might want. launch the simulations via `"$PATH_TO_PORTABLE\Python-Portable.exe" "$PATH_TO_SIMULATION.py"` (quotes important!)
