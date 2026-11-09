"""AX-12 raw positions to CAD joint radians; never invent a missing zero/sign."""
import math


def convert_home(raw, raw_at_cad_zero, direction):
    if any(len(values)!=4 for values in (raw,raw_at_cad_zero,direction)):
        raise ValueError('Exactly four joint values are required')
    if any(type(value) is not int or not 0<=value<=1023 for value in raw):
        raise ValueError('HOME raw values must be integer counts in 0..1023')
    if any(type(value) not in (int,float) or not math.isfinite(value) or not 0<=value<=1023
           for value in raw_at_cad_zero):
        raise ValueError('Each joint requires a measured raw_at_cad_zero')
    if any(type(value) is not int or value not in (-1,1) for value in direction):
        raise ValueError('Each joint requires direction +1 or -1')
    factor=math.radians(300)/1023
    return [sign*(count-zero)*factor for count,zero,sign in zip(raw,raw_at_cad_zero,direction)]
