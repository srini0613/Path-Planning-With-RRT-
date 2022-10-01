# -*- coding: utf-8 -*-
"""
Created on Fri Apr 15 16:41:29 2022

@author: seenu
"""

import numpy as np

data = np.loadtxt("angle.txt", usecols=0, dtype='float')
a=len(data) 
print(a)
print(data)