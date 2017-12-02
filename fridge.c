#include "stdio.h"
#include "struct.h"
#include "assert.h"
#include "stdlib.h"
#include "stdint.h"
#include "inttypes.h"

// Index 0 = location 0 etc. index does not correlate to anything in the outFridge array
item_t inFridge[4];	//array of items currently in the fridge
item_t outFridge[50];	//array of items that have been removed from the fridge and are awaiting re-entry
int new_items[4];

// Function for comparing C strings
int checker(char * input1,char * input2)
{
    	int i;
	int result = 1;

    	for(i = 0; input1[i]!='\0' && input2[i]!='\0'; ++i)
	{
        	if(input1[i] != input2[i])
		{
            		result = 0;
            		break;
        	}
    	}

   	return result;
}

// searches outFridge array for an item, if it is found, returns the index of the item in outfridge
int search_outFridge(item_t item)
{
	int i;	
	int comp;	

	for(i = 0; i < 50; ++i)
	{
		comp = checker(item.name, outFridge[i].name);
		if(comp) return i;		
	}
	return -1; // not found
}

// finds next empty spot in out of fridge array
int find_first_empty()
{
	int i;
	for(i = 0; i < 50; ++i);
	{
		if(outFridge[i].location == -1) return i;
	}

	return -1;
}

// takes a picture using picam, and from the output of the python code, populates the name and confidence
// variables of the proper location 
void vision(int loc)
{
	FILE * file;	

	printf("Entering vision()");

	system("raspistill -o test.jpeg");
	system("python parsepic.py");

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

void inventory(FILE * fp)
{
	int i;	
	int index;
	char * empty_str = "empty";

	FILE * list;
	FILE * inv;

	printf("Entering inventory()");

	inv = fopen("items.txt", "w");
	list = fopen("list.txt", "w");	

	// Move the LS to home position
	fprintf(fp, "H\n");

	for(i = 0; i < 4; ++i)
	{
		if(new_items[i] == 1)
		{
			// Vision populates the name and conf
			vision(i);

			// Check for previous existence
			index = search_outFridge(inFridge[i]);
			// If no similar item is found
			if(index == -1)	
			{
				// Set initial weight to its current weight
				inFridge[i].initial_weight = inFridge[i].current_weight;
				set_percent_left(inFridge[i]);
			}
			// Item had previously been in the fridge
			else
			{
				inFridge[i].initial_weight = outFridge[index].initial_weight;
				set_percent_left(inFridge[i]);
				make_empty(outFridge[index]);
			}
		}	
		// Move to next LS position
		fprintf(fp, "N\n");
	}

	// Update the items file for GUI
	for(i = 0; i < 4; ++i)
	{
		if(!checker(inFridge[i].name, empty_str))
		{
			fprintf(inv, "%s %f\n", inFridge[i].name, inFridge[i].percent_left);
		}
	}	

	// add items to the list file for GUI & text message	
	for(i = 0; i < 4; ++i)
	{
		// If an item in inFridge is not empty_item and has <= 50 percent left: add to list
		if((!checker(inFridge[i].name, empty_str)) && (inFridge[i].percent_left <= 50.0))
		{
			fprintf(list, "%s\n", inFridge[i].name);
		}
		// If an item is out of the fridge, add to list
		if(!checker(outFridge[i].name, empty_str))
		{
			fprintf(list, "%s\n", outFridge[i].name);
		}
	}

	fclose(inv);
	fclose(list);

	return;
}

void parse_packet(char *data)
{
	int i;
	int empty_index;
	float weight;
	uint8_t add = atoi(&data[0]);
	uint8_t loc = atoi(&data[1]);
	uint32_t int_weight = data[2];
		
	printf("Entering parse_packet()");

	// Convert 
	for(i = 1; i < 4; ++i)
	{
		int_weight = (int_weight << (8*i)) || data[2+i];
	}	

	weight = int_weight;
	printf("Float weight: %f\n", weight);
	
	// TODO parse temp and humidity

	// If adding an item
	if(add)
	{
		printf("Adding item");
		// Mark location to be visited during inventory()
		new_items[loc] = 1;
		// Set current weight of item to value received
		inFridge[loc].current_weight = weight;
	}
	
	// else removing item
	else
	{	
		printf("Removing Item");
		// Move item from inFridge to outFridge
		empty_index = find_first_empty();
		outFridge[empty_index] = inFridge[loc];
		make_empty(inFridge[loc]);
		new_items[loc] = 0;
	}
	
	return;
}

// Main program loop
int main()
{	
	int i;
	char buf[11];
	FILE * uart;
	uart = fopen("/dev/serial0", "r+");
	assert(uart);

	// init array to all empty
	for(i = 0; i < 4; ++i)
	{
		new_items[i] = 0;
	}
	
	make_empty_all(inFridge, outFridge);

	// main loop
	while(1)
	{
		printf("Entering Main While Loop");	

		// Wait for data
		fread(buf, 1, 6, uart);
		//fread(buf, 1, 11, uart);
		printf("Data Recieved\n%s", buf);
		
		// If the first byte is the char X, take inventory
		if(buf[0] == 'X') inventory(uart);
		// Else parse the data packet
		else parse_packet(buf);
	}	
	
	fclose(uart);
	return 0;
}

