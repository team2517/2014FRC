
@echo off
set USER=${TEMPLATE_USERNAME}
set LOGNAME=%USER%
set USERNAME=%USER%

set HOME=/home/%USER%
rem ember to create "F:\Git\home\%USER%\" with the skeleton files (git-global settings ect ect)

F:\Git\bin\bash.exe --login -i

