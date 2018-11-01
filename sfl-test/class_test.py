class Calculator:

    def __init__(self, name, price, hight=18, width=29, weight=5):
        self.name = name
        self.price = price
        self.h = hight
        self.wi = width
        self.we = weight

    def add(self, x, y):
        print(self.name)
        result = x + y
        print(result)

    def minus(self, x, y):
        result = x - y
        print(result)

    def times(self, x, y):
        result = x * y
        print(result)

    def divide(self, x, y):
        result = x / y
        print(result)


if __name__ == '__main__':
    c = Calculator("bad Calculator", 12, )
