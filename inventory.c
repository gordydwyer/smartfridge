#include <stdio.h>

int main()
{	
    	char str[999];
	FILE * file;
	file = fopen( "items.txt" , "r");
	if (file) 
	{
    		while (fscanf(file, "%s", str)!=EOF)
        	printf("%s\n",str);
    		fclose(file);
	}
	else
	{
		printf("Error opening file\n");
	}
    
    return 0;
}
