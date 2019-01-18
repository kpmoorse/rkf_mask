from time import time


class Timepoints():
    def __init__(self):
        self.tp_list = []

    def add_timepoint(self):
        self.tp_list.append(time())

    def display(self):
        print('###############')
        for i in range(len(self.tp_list)-1):
            print("(%02i): %.03fms" % (i, (self.tp_list[i+1] - self.tp_list[i])*1000))
        self.tp_list = []
