"""Polariser interface for PLI acquisition.

Defines the abstract polariser interface for rotating
optical polarisers during PLI image acquisition.
"""

from abc import ABC, abstractmethod


class PolariserInterface(ABC):
    """Abstract base class for polariser rotation control."""

    @property
    @abstractmethod
    def steps_whole_turn(self) -> float:
        '''Number of steps for a full 360-degree rotation.'''

    @property
    @abstractmethod
    def current_step(self) -> int:
        '''Current step position.'''

    @abstractmethod
    def rotate_to(self, step: int) -> bool:
        '''Rotate to an absolute step position. Returns True if movement was sent.'''

    @abstractmethod
    def rotate_home(self) -> bool:
        '''Rotate to the home position (step 0), taking the shortest path. Returns True if movement was sent.'''

    @abstractmethod
    def reset(self) -> None:
        '''Reset the step counter to 0 (without moving).'''
