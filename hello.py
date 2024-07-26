class Person():
    def __init__(self, name, age):
        self.name = name
        self.age = age
    def display(self):
        print(self.name, self.age)

class Student(Person):
    def __init__(self, name, age):
        self.Sname = name
        self.Sage = age
        super().__init__('Manh',age)
    def displayS(self):
        print(self.Sname, self.Sage)

obj = Student('Hieu', 23)
obj.display()
obj.displayS()
#hello everybody
