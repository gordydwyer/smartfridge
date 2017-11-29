#include "stdio.h"
#include "struct.h"
#include "assert.h"

// Index 0 = location 0 etc. index does not correlate to anything in the outFridge array
item_t inFridge[4];	//array of items currently in the fridge
item_t outFridge[4];	//array of items that have been removed from the fridge and are awaiting re-entry

int new_items[4] = [0,0,0,0];

// searches outFridge array for an item, if it is found, returns the index of the item in outfridge
int search_outFridge(item_t item)
{
	for(int i = 0; i < 4; ++i)
	{
		if(item.name == outFridge[i].name) return i;		
	}
	
	return -1; // not found
}

void take_picture()
{
	system("raspistill -o test.jpeg");
	system("python parsepic.py");
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
		fclose(file);
		//printf("%s\n", inFridge[loc].name);
		//printf("%s\n", inFridge[loc].confidence);
	}

	else
	{
		printf("ERROR OPENING");
	}	

	return;
}

int main()
{	
	char buf[11];
	FILE * uart;
	uart = fopen("/dev/serial0", "r+");
	assert(uart);

	// main loop
	while(1)
	{
		// Data is found
		if(fgets(buf, 1, uart) != NULL)
		{
			if(buf[0] == 'X') // if its an X, means take inventory
			{
				inventory();
				continue;
			}

			delay(200);
			fgets(buf + 1, 10, uart);

				

			// parse out bytes and assign
		}

		else
		{
			delay(1000);
		}
	}	
	
	fclose(uart);
	return 0;
}






















