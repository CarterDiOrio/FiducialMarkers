import matplotlib.pyplot as plt
import numpy as np
from itertools import zip_longest

def grouper(iterable, n, fillvalue=None):
    args = [iter(iterable)] * n
    return zip_longest(*args, fillvalue=fillvalue)


# each line is formatted as x, y, x_residual, y_residual

class Residuals:
    def __init__(self, x, y, x_residual, y_residual):
        self.x = x
        self.y = y
        self.x_residual = x_residual
        self.y_residual = y_residual

def plot_residuals(residuals_file):
    residuals = []
    with open(residuals_file, 'r') as f:
        lines = f.readlines()
        lines = [line.strip().split(' ') for line in lines]

        for line in lines:
            x = float(line[0])
            y = float(line[1])
            x_residual = float(line[2])
            y_residual = float(line[3])

            residual = Residuals(x, y, x_residual, y_residual)
            residuals.append(residual)
        
    
    combined_xy_residuals = []
    for residual in residuals:
        combined_xy_residuals.append(residual.x_residual)
        combined_xy_residuals.append(residual.y_residual)

    #make bucket size 0.1 for residuals from -2 to 2
    plt.hist(combined_xy_residuals, bins=80, range=(-4, 4))
    
    plt.xlabel('Residual in pixels (x and y are counted seperately)')
    plt.ylabel('Frequency')
    plt.title('Histogram of residuals')
    plt.show()

    plt.clf()
    # plot residuals color scheme by magnitude
    plt.scatter([residual.x for residual in residuals], 
                [residual.y for residual in residuals],
                c=[np.sqrt(residual.x_residual**2 + residual.y_residual**2) for residual in residuals])
    # put a color bar
    plt.colorbar()
    plt.xlabel('x pixels')
    plt.ylabel('y pixels')
    plt.title('Residuals by magnitude')
    plt.xlim(0, 1920)
    plt.ylim(0, 1080)
    plt.gca().set_aspect('equal', adjustable='box')

    plt.gca().invert_yaxis()
    plt.show()


    # for chunk in grouper(residuals, 100):
    #   plt.clf()
    #   plt.scatter([residual.x for residual in chunk], 
    #               [residual.y for residual in chunk],
    #               c=[np.arctan2(residual.y_residual, residual.x_residual) * 180 / np.pi for residual in chunk])
    #   plt.colorbar()
    #   plt.xlabel('x pixels')
    #   plt.ylabel('y pixels')
    #   plt.title('Residuals by angle')

    #   plt.show()

    for chunk in grouper(residuals, 100):
      plt.clf()
      # plot residuals color scheme by magnitude
      plt.scatter([residual.x for residual in chunk], 
                  [residual.y for residual in chunk],
                  c=[np.sqrt(residual.x_residual**2 + residual.y_residual**2) for residual in chunk])
      
      # normalize residual magnitudes
      max_residual = max([np.sqrt(residual.x_residual**2 + residual.y_residual**2) for residual in chunk])
      min_residual = min([np.sqrt(residual.x_residual**2 + residual.y_residual**2) for residual in chunk])
      normalized_magnitudes = [(np.sqrt(residual.x_residual**2 + residual.y_residual**2) - min_residual) / (max_residual - min_residual) for residual in chunk]

      # make an arrow in the direction of each residual
      for idx, residual in enumerate(chunk):

        # get vector
        vector = np.array([residual.x_residual, residual.y_residual])

        # normalize vector
        vector = vector / np.linalg.norm(vector)

        # scale vector by normalized magnitude
        vector *= normalized_magnitudes[idx] * 100
        
        plt.arrow(residual.x, residual.y, vector[0], vector[1], head_width=0.5, head_length=0.5, fc='k', ec='k')



      # put a color bar
      plt.colorbar()
      plt.xlabel('x pixels')
      plt.ylabel('y pixels')
      plt.title('Residuals by magnitude')

      # 0,0 is at the top left

      # window is 1920x1080
      plt.xlim(0, 1920)
      plt.ylim(0, 1080)

      plt.gca().invert_yaxis()


      # force square aspect
      plt.gca().set_aspect('equal', adjustable='box')
      plt.show()



plot_residuals('residuals.txt')