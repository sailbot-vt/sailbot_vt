from collections import deque

class WeightedMovingAverage:
    def __init__(self, num_elements):
        self.data_queue = deque(maxlen=num_elements)
        
    def add_data(self, new_data):
        self.data_queue.append(new_data)
    
    
    def _sum_integers(self, integer):
        """Sums up all positive integers less than or equal to the number that was passed in"""
        # nothing but this formula: https://www.youtube.com/watch?app=desktop&v=bWZwF1H9YbU
        return (integer * (integer + 1))/ 2

    def get_smoothed_data(self):
        weighted_list = []

        for index, data in enumerate(self.data_queue):
            weighted_list.append(data * (index+1))

        length = len(self.data_queue) 
        return sum(weighted_list)/ self._sum_integers(length)
