
class Sample:
    def __init__(self, id, cps, doseRate, position, timestamp):
        self.id = id
        self.cps = cps
        self.doseRate = doseRate
        self.position = position
        self.timestamp = timestamp

    def __str__(self):
        return "Sample(id={}, cps={}, doseRate={}, position={}, timestamp={})".format(self.id, self.cps, self.doseRate, self.position, self.timestamp)

    def __repr__(self):
        return self.__str__()