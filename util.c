float deadBand(float axisValue)
{
	if(axisValue < -.05 || axisValue > .05)
	{
		return axisValue;
	}
	else
	{
		return 0.0;
	}
}
