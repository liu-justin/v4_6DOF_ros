from collections import deque

class dequettl(deque):
    def __init__(self, lista, ttl):
        super().__init__(lista)
        self.ttl = ttl

main = deque([])
main.appendleft(dequettl([1],5))
main.appendleft(dequettl([12],5))

while True:
    main.appendleft(dequettl([31],5))

print(main)