#!/usr/bin/env python

## This file can be run to manually relearn the machine learning model
## It takes almost half a minute to run, so please be patient.
## When it is done, it automatically saves the result to 'knn_model.sav'

import process_images

classifier = process_images.Classifier()
# Trains the classifier using locally stored images
# Pass in false to use prelearned model or true to relearn model
classifier.set_train(True)
