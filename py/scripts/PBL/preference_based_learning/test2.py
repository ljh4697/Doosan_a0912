import numpy as np



#while True:

t = 0
algorithms = ['DPB', 'greedy']
while t < 10:
    shuffled_order = np.random.choice(2)
    print(shuffled_order)
    if shuffled_order==0:
        print(t)
        print(algorithms[0])
        print(algorithms[1])
        
    else:
        print(t)
        print(algorithms[1])
        print(algorithms[0])
        
    t += 1
        
        
        
        
        
        
    