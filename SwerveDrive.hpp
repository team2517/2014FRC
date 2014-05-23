#ifndef SWERVEDRIVE_HPP
#define SWERVEDRIVE_HPP

#include "SwerveModule.hpp"
class SwerveDrive 
{
	SwerveModule *ModuleFL;
	SwerveModule *ModuleFR;
	SwerveModule *ModuleBR;
	SwerveModule *ModuleBL;
	
public:
	
	SwerveDrive(modulePlug, modulePlug, modulePlug, modulePlug);
	void moveDrive(float, float, float);
};

#endif
