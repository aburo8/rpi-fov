"""
Custom LifoQueue
"""
from queue import LifoQueue

class MaxLifoQueue():
    """
    A LIFO Queue (Stack) which allows you to specify a maximum number of elements
    """
    def __init__(self, maxSize = 50):
        self.queue = LifoQueue()
        self.maxSize = maxSize
        
    def put(self, item):
        if (self.queue.qsize() < self.maxSize):
            self.queue.put(item)
        else:
            print("Resetting Queue")
            self.queue = LifoQueue()
            self.queue.put(item)
    
    def get(self):
        return self.queue.get()
    
    def empty(self):
        return self.queue.empty()