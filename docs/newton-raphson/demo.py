import numpy as np
import matplotlib.pyplot as plt



# ------------------[ SETTINGS ]------------------
ANCHOR_NUMBER           = 4     # number of anchors
ANCHOR_ERROR_MAGNITUDE  = 0.1   # evenly distributed from +-ANCHOR_ERROR_MAGNITUDE



# ------------------[ GENERATE RANDOM SCENARIO ]------------------
# >> anchors    : list of position of anchors and the reading
# >> anchor_num : number of anchors
# >> tag        : real position of tag
anchor_err_mag  = ANCHOR_ERROR_MAGNITUDE
anchor_num      = ANCHOR_NUMBER
anchor_grid_x   = 2
anchor_grid_y   = 2
# anchor_grid_x   = np.random.randint(1, anchor_num + 1)
# anchor_grid_y   = int(np.ceil(anchor_num/anchor_grid_x))
anchors         = []

tag = [float(anchor_grid_x)*np.random.rand(), float(anchor_grid_y)*np.random.rand()]

x_pos = 0
y_pos = 0
for i in range(anchor_num):
    anchor_temp = []
    anchor_temp.append(float(x_pos) + 0.2*np.random.rand())
    anchor_temp.append(float(y_pos) + 0.2*np.random.rand())
    anchor_temp.append(np.sqrt((anchor_temp[0] - tag[0])**2.0 + (anchor_temp[1] - tag[1])**2.0) + anchor_err_mag*(2.0*np.random.rand() - 1.0))
    
    x_pos += 1
    if x_pos == anchor_grid_x:
        x_pos = 0
        y_pos += 1

    anchors.append(anchor_temp)



# ------------------[ SOLVE THE PROBLEM ]------------------
# [1] Initial position is the mean of anchors' position
est_x   = 0.
est_y   = 0.
for anchor in anchors:
    est_x += anchor[0]
    est_y += anchor[1]
est_x   /= anchor_num
est_y   /= anchor_num
est_pos = np.array([
    [est_x],
    [est_y]
])



# [2] Calculate the nabla and jacobian
def nablaF(est_pos:np.ndarray) -> np.ndarray:
    xp = est_pos[0].item()
    yp = est_pos[1].item()

    sumterm_x = 0.
    sumterm_y = 0.
    for anchor in anchors:
        dist = np.sqrt((xp - anchor[0])**2.0 + (yp - anchor[1])**2.0)
        sumterm_x += anchor[0] + anchor[2]*(xp - anchor[0])/dist
        sumterm_y += anchor[1] + anchor[2]*(yp - anchor[1])/dist

    return np.array([
        [sumterm_x - float(anchor_num)*xp],
        [sumterm_y - float(anchor_num)*yp]
    ])


def jacobianF(est_pos:np.ndarray) -> np.ndarray:
    xp = est_pos[0].item()
    yp = est_pos[1].item()

    sumterm_x   = 0.
    sumterm_y   = 0.
    sumterm_xy  = 0.
    for anchor in anchors:
        dist    = np.sqrt((xp - anchor[0])**2.0 + (yp - anchor[1])**2.0)
        dist32  = dist*((xp - anchor[0])**2.0 + (yp - anchor[1])**2.0)
        sumterm_x   += anchor[2]/dist - anchor[2]*((xp - anchor[0])**2.0)/dist32
        sumterm_y   += anchor[2]/dist - anchor[2]*((yp - anchor[1])**2.0)/dist32
        sumterm_xy  += (anchor[2]*(xp - anchor[0])*(yp - anchor[1]))/dist32
    
    return np.array([
        [sumterm_x - float(anchor_num), -sumterm_xy],
        [-sumterm_xy, sumterm_y - float(anchor_num)]
    ])



# [3] Update position
def updatePos(est_pos:np.ndarray) -> np.ndarray:
    return est_pos - np.linalg.inv(jacobianF(est_pos))@nablaF(est_pos)


for i in range(20):
    est_pos = updatePos(est_pos)



# ------------------[ OUTPUT THE RESULT ]------------------
print('---------------[ SCENARIO ]---------------')
print(f'>> anchor num : {anchor_num}')
print(f'>> anchor grid: {anchor_grid_x}x{anchor_grid_y}')
print(f'>> actual pos : ({tag[0]}, {tag[1]})')
print(f'>> est. pos   : ({est_pos[0].item()}, {est_pos[1].item()})')