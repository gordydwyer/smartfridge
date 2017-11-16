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
} item;

//function to set the percent_left memeber variable
/*void set_percent(struct item)
{
	item.percent_left = 100*(item.current_weight / item.intial_weight);
	return 0;
}*/
