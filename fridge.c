#include "stdio.h"
#include "struct.h"

// Index 0 = location 0 etc. index does not correlate to anything in the outFridge array
item_t inFridge[4];	//array of items currently in the fridge
item_t outFridge[4];	//array of items that have been removed from the fridge and are awaiting re-entry


void take_picture()
{
	system("raspistill -o test.jpeg");
	system("python takepic.py");
	return;
}


// takes a picture using picam, and from the output of the python code, populates the name and confidence
// variables of the proper location 
void vision(int loc)
{
	take_picture();

	FILE * file;
	file = fopen("returncsv.csv", "r");
	if (file)
	{
		// get first line of GVAPI output (best result)
		fscanf(file, "%s %s", inFridge[loc].name, inFridge[loc].confidence);
		inFridge[loc].location = loc;	

		printf("%s\n", inFridge[loc].name);
		printf("%s\n", inFridge[loc].confidence);
	}

	else
	{
		printf("ERROR OPENING");
	}	

	return;
}




