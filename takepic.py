import io
import os
import csv

# Imports the Google Cloud client library
from google.cloud import vision
from google.cloud.vision import types

# Instantiates a client
client = vision.ImageAnnotatorClient()

# The name of the image file to annotate
file_name = os.path.join(
    	os.path.dirname(__file__),
    	'/home/gdwyer/smartfridge/test.jpeg')

# Loads the image into memory
with io.open(file_name, 'rb') as image_file:
    	content = image_file.read()

image = types.Image(content=content)

# Performs label detection on the image file
#response = client.label_detection(image=image)
#labels = response.label_annotations

#print('Labels:')
#for label in labels:
#    	print(label.description)

#performs web detection
response = client.web_detection(image=image)
notes = response.web_detection

#if notes.web_entities:
#	print ('\n{} Web entities found: '.format(len(notes.web_entities)))
#
#        for entity in notes.web_entities:
#        	print('Score      : {}'.format(entity.score))
#           	print('Description: {}'.format(entity.description))

with io.open('returncsv.csv', 'wb') as csvfile:
	writer = csv.writer(csvfile, delimiter='\t', quoting=csv.QUOTE_NONE)

	if notes.web_entities:
		count = 0
        	for entity in notes.web_entities:
			if count <= 3:
        			writer.writerow([format(entity.description)] + [format(entity.score)])
				count = count + 1
			else:
				break
