#!/usr/bin/env python3
import numpy as np
import json
from tensorflow import keras 
from tensorflow.keras.models import Sequential
from my_classes import DataGenerator
worlds=['world1','world2','world3','world4','world5','world6','world7','world8','world9','world10']
files=open('output.txt','r')
js=files.read()
item_dict = json.loads(js)
# Parameters
params = {'batch_size': 10,
          'n_classes': 6,
          'n_channels': 3,
          'shuffle': True}

# Datasets
partition =worlds # IDs
labels = item_dict# Labels

# Generators
training_generator = DataGenerator(partition,labels, **params)
#validation_generator = DataGenerator(partition['validation'], labels, **params)

# Design model

