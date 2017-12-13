/* 	struct represents an item on the lazy susan
	name = name of item up to 24 characters long
	confidence = confidence score of google vision API
	intial_weight = weight of the item upon intital entry to fridge
	current_weight = weight of the item upon its most current entry to fridge
	perent_left = percent of the item left based upon intial and current weight
	location = which location on the lazy susan that the item is currently located at
*/	

typedef struct{
	char 	name[24];
	char	confidence[5];
	float 	initial_weight;
	float	current_weight;
	float 	percent_left;
	int 	location;
} item_t;


void make_empty(item_t item)
{
	int i;

	item.name[0] = 'e';
	item.name[1] = 'm';
	item.name[2] = 'p';
	item.name[3] = 't';
	item.name[4] = 'y';
	
	for(i = 5; i < 24; ++i)
	{
		item.name[i] = '\0';	
	}

	item.confidence[0] = '-';
	item.confidence[1] = '1';
	item.confidence[2] = '\0';
	item.confidence[3] = '\0';
	item.confidence[4] = '\0';

	item.initial_weight = -1;
	item.current_weight = -1;
	item.percent_left = -1;
	item.location = -1;

	return;
}

void make_empty_all(item_t *arr1, item_t *arr2)
{
	int i;

	for(i = 0; i < 4; ++i)
	{
		make_empty(arr1[i]);	
	}

	for(i = 0; i < 50; ++i)
	{
		make_empty(arr2[i]);
	}

	return;
}

/*
char* get_name(item_t item)
{
	return item.name;
}

char* get_confidence(item_t item)
{
	return item.confidence;
}
*/

float get_intital_weight(item_t item)
{
	return item.initial_weight;
}

float get_current_weight(item_t item)
{
	return item.current_weight;
}

float get_percent_left(item_t item)
{
	return item.percent_left;
}

int get_location(item_t item)
{
	return item.location;
}

void set_name(item_t item, char * name)
{
	strcpy(item.name, name);
	//item.name = name;
	return;
}

void set_confidence(item_t item, char * conf)
{
	strcpy(item.confidence, conf);
	//item.confidence = conf;
	return;
}

void set_initial_weight(item_t item, float weight)
{
	item.initial_weight = weight;
	return;	
}

void set_current_weight(item_t item, float weight)
{
	item.current_weight = weight;
	return;
}

void set_percent_left(item_t item)
{
	item.percent_left = 100*(item.current_weight / item.initial_weight);
	return;
}

void set_location(item_t item, int loc)
{
	item.location = loc;
	return;
}


