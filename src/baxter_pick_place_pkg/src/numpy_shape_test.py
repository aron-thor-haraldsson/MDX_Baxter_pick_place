#!/usr/bin/env python
import numpy as np

var = [(140, 42), 'CRO', '93.3333333333']

print var
print np.shape(var)


var = [[(69, 116), 'CIR', '100.0'], [(140, 42), 'CRO', '93.3333333333']]

print var
print np.shape(var)

var = [[(69, 116), 'CIR', '100.0'], [(140, 42), 'CRO', '93.3333333333'], [(140, 42), 'CRO', '93.3333333333']]

print var
print np.shape(var)

var = []

print var
print np.shape(var)

var = False

print var
print np.shape(var)
