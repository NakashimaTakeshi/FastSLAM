class Landmark():
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y

    def __str__(self):
        return "Landmark: id: " + str(self.id) + " x: " + str(self.x) + " y: " + str(self.y)

    def __repr__(self):
        return self.__str__()