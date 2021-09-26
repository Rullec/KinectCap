import numpy as np
import matplotlib.pyplot as plt

img = []
with open("tmp.txt", 'r') as f:
    for i in f.readlines()[1:]:
        i = i.strip()
        img.append([float(num) for num in i.split(" ")])

img = np.array(img)
plt.imshow(img)
plt.show()