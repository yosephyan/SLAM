import numpy as np
from load_data import get_encoder, get_lidar
import matplotlib.pyplot as plt
import seaborn as sns
from MapUtils.MapUtils import getMapCellsFromRay
#from MapUtilsCython.MapUtils_fclad import getMapCellsFromRay

# Initializes the parameters
width = 164
offset = 50
discretization = 5
particles = 70
xytnoise = 0.001
hit = 0.9
miss = 0.3
cap = 20

# Function to perform SLAM
def SLAM(filenum, filepath):
    # Loads the data from the files
    fl, fr, rl, rr, ts = get_encoder(filepath + "Encoders" + str(filenum))
    lidar = get_lidar(filepath + "Hokuyo" + str(filenum))

    # Matches the timestamps
    ts_array = np.array(ts)
    lidar_timestamps = np.array([l['t'] for l in lidar])
    time_differences = np.abs(ts_array.reshape(-1, 1) - lidar_timestamps)
    closest_indices = np.argmin(time_differences, axis=1)
    lidar_data = [lidar[index] for index in closest_indices]

    # Averages the encoder data
    r = (fr + rr) / 2
    l = (fl + rl) / 2

    # Initializes variables
    grid = np.zeros((100 * discretization, 100 * discretization))
    globalxy = np.zeros((2, particles))
    noise = 0

    # Loops through the data
    for index, (er, el, scans) in enumerate(zip(r, l, lidar_data[:])):
        # Retrieves the angles and scans
        angles = scans['angle']
        scan = scans['scan'].reshape(-1, 1)

        # Initializes the best particle
        maxcount = -np.inf

        # Used equations from lecture
        theta = (er - el) / width
        deltaxy = np.array([[(er + el) / 2 * np.cos(theta)], [(er + el) / 2 * np.sin(theta)]])
        deltaxy = deltaxy * (127/500) * (np.pi / 360)
        # Adds noise to the particles
        noise += np.random.normal(0, xytnoise, particles)
        matrix = np.array([[np.cos(noise), np.sin(noise)], [np.sin(noise), np.cos(noise)]]).reshape(2, 2, particles)
        noise += theta / 2
        # Calculates the change in location
        xy = (matrix * deltaxy).sum(axis = 1)
        globalxy += xy
        x = globalxy[0, :]
        y = globalxy[1, :]

        # Maps lidar data to grid
        xends = ((x + np.cos(noise + angles) * scan) + offset) * discretization
        yends = ((y + np.sin(noise + angles) * scan) + offset) * discretization
        # Maps robot to grid
        xrobot = (x + offset) * discretization
        yrobot = (y + offset) * discretization

        # Pre-calculates all wall positions
        xindex = xends.astype(int)
        yindex = yends.astype(int)

        # Loop to identify the best particle based on the number of matched walls
        for particle in range(particles):
            # Constructs a 2D array of walls
            wall = np.stack((xindex[:, particle], yindex[:, particle]), axis=-1)
            # Sums the walls
            currentwall = np.sum(grid[wall[:, 0], wall[:, 1]])
            
            # Updates the best particle
            if maxcount < currentwall:
                maxcount = currentwall
                best = particle

        # Retrieves wall and gap data
        bestwall = np.stack((xindex[:, best], yindex[:, best]), axis=-1)
        nonwall = getMapCellsFromRay(xrobot[best], yrobot[best], xends[:, best], yends[:, best], 0).T.astype(int)

        # Updates the grid
        if maxcount > 0 or index < 10:
            grid[bestwall[:, 0], bestwall[:, 1]] += hit
            grid[nonwall[:, 0], nonwall[:, 1]] -= miss

        # Clips the grid
        grid = np.clip(grid, -cap, cap)

    # Finds rows and columns where there is at least one non-zero value
    nonzero_rows = np.any(grid != 0, axis=1)
    nonzero_cols = np.any(grid != 0, axis=0)

    # Finds the indices of the first and last non-zero rows and columns
    xmin, xmax = np.where(nonzero_rows)[0][[0, -1]]
    ymin, ymax = np.where(nonzero_cols)[0][[0, -1]]

    # Adjusts xmax and ymax
    xmax += 1
    ymax += 1

    # Displays the map
    plt.figure()
    sns.heatmap(grid[xmin:xmax, ymin:ymax])
    plt.show()

# Initializes the folder and file number
folder = "./data/"
filenum = 23

# Calls the SLAM function
SLAM(filenum, folder)