// This file will be the main loop that monitors serial comms
// for updates to presence of objects, as well as updates to their weights
// and control the inventory process (motor and camera)
#include "stdio.h"
#include "struct.h"

char temp[100];

int main()
{
	item test;
	//char * name_ptr = test.name[0];
	//char * conf_ptr = test.confidence[0];

	FILE * file;
	file = fopen("returncsv.csv", "r");
	if (file)
	{
		// get first line of GVAPI output (best result)
		fscanf(file, "%s", temp);
		int i = 0;
		while (temp[i] != ',')
		{
			test.name[i] = temp[i];
			i++;
		}	
	
		test.name[i] = '\0';
		i++;	
		int end = i + 4;
		int j = 0;
		for (i = i; i < end; i++)
		{
			test.confidence[j] = temp[i];
			j++;
		}

		test.confidence[4] = '\0';

		fclose(file);

		printf("%s\n", test.name);
		printf("%s\n", test.confidence);
	}

	else
	{
		printf("ERROR OPENING");
	}	

	return 0;
}
