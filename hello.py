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
<<<<<<< HEAD
#hello everybody âsdf
=======
#hello everybody sdsfds
>>>>>>> 2ff265dec506d04f1d36560aa82ccc05e4a5e93f
