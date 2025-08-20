"""
Autonomy Manager Package

This package provides algorithms and utilities for autonomous environmental sampling.
"""

# Import main algorithm classes
from .adaptiveROS import adaptiveROS
from .gridROS import gridROS
from .boundaryConversion import Conversion
from .boundaryCheck import boundaryCheck

__version__ = '0.0.0'
__author__ = 'robomechanics'

# Make main classes available at package level
__all__ = [
    'adaptiveROS',
    'gridROS', 
    'Conversion',
    'boundaryCheck',
]
