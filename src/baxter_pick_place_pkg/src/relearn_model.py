#!/usr/bin/env python

import process_images

classifier = process_images.Classifier()
# Trains the classifier using locally stored images
# Pass in false to use prelearned model or true to relearn model
classifier.set_train(True)
