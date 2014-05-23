#define OFFSETFILE				"offsets.dat"

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

float* readOffsets()
{
    FILE* curFile = fopen(OFFSETFILE, "r");
    float *rin = malloc(sizeof(float)*4);
    fread(rin, sizeof(float), 4 ,curFile );
    fclose(rin);
    return rin;
}

void writeOffsets(float * offsets)
{
	FILE* curFile = fopen(OFFSETFILE, "r");
	fwrite(offsets, sizeof(float), 4 ,curFile;
	fclose(offsets);
}
