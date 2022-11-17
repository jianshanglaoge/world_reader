#!/usr/bin/env python3

import numpy as np

import json
from tensorflow import keras 

items_list=['tomato_soup_can','tuna_fish_can','pitcher_base','sugar_box',
'potted_meat_can','mustard_bottle','master_chef_can',
'gelatin_box','cracker_box','chips_can']
cameras_list=['1080pcamera','1080pcamera_0','1080pcamera_1']
worlds=['world1','world2','world3','world4','world5','world6','world7','world8','world9','world10']


class DataGenerator(keras.utils.Sequence):
    'Generates data for Keras'
    def __init__(self, list_IDs, labels, batch_size=32, dim=(32,32,32), n_channels=3,
                 n_classes=10, shuffle=True):
        'Initialization'
        self.dim = dim
        self.batch_size = batch_size
        self.labels = labels
        self.list_IDs = list_IDs
        self.n_channels = n_channels
        self.n_classes = n_classes
        self.shuffle = shuffle
        self.on_epoch_end()
           

    def __len__(self):
        'Denotes the number of batches per epoch'
        return int(np.floor(len(self.list_IDs) / self.batch_size))

    def __getitem__(self, index):
        'Generate one batch of data'
        # Generate indexes of the batch
        indexes = self.indexes[index*self.batch_size:(index+1)*self.batch_size]

        # Find list of IDs
        list_IDs_temp = [self.list_IDs[k] for k in indexes]

        # Generate data
        X, y = self.__data_generation(list_IDs_temp)

        return X, y

    def on_epoch_end(self):
        'Updates indexes after each epoch'
        self.indexes = np.arange(len(self.list_IDs))
        if self.shuffle == True:
            np.random.shuffle(self.indexes)

    def __data_generation(self, list_IDs_temp):
        'Generates data containing batch_size samples' # X : (n_samples, *dim, n_channels)
        # Initialization
        X = np.empty((self.batch_size,self.camera_num, *self.dim, self.n_channels))
        y = np.empty((self.batch_size,self.num_items,4,3), dtype=int)
        print(xxx)
        # Generate data
        for sample_num, sample in enumerate(list_IDs_temp):
            # Store sample
            for camera_num,camera in enumerate(cameras_list):
                X[sample_num,camera_num,:] = np.load('data/'+sample+'_'+camera+ '.npy')

            # Store class
            for item_num,item in enumerate(items_list):
                y[sample_num,item_num, 0,:] = labels.get(sample).get(item).get('item_r')
                y[sample_num.item_num,1:3,:]= labels.get(sample).get(item).get('item_t')

        return X, y



