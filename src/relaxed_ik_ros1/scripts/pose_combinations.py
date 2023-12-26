import numpy as np

def subsets_with_sum(lst, target, with_replacement=False):
    x = 0 if with_replacement else 1
    def _a(idx, l, r, t):
        if t == sum(l): r.append(l)
        elif t < sum(l): return
        for u in range(idx, len(lst)):
            _a(u + x, l + [lst[u]], r, t)
        return r
    return _a(0, [], [], target)


# sum of sequences from input list = target - start

def sum_6d_arrays_with_one_none_zero(start, final, input_list):
    mps = []
    goal = (0, 0, 0, 0, 0, 0)
    goal = list(goal)
    for i in range(0, len(start)):
        goal[i] = final[i] - start[i]
    goal = tuple(goal)

    for i in range(len(goal)):
        curr_list = [input_list[j][i] for j in range(len(input_list))]
        subsets = subsets_with_sum(curr_list, goal[i])[0]
        for s in subsets:
            if s == 1:
                t = tuple(1 if l == i else 0 for l in range(len(goal)))
                mps.append(t)

    return mps

def sum_6d_arrays_with_N_zeros(start, final, input_list):
    # sum_6d_arrays_with_one_none_zero
    pass

def decompose_array(arr):
    """
    Decomposes an array of values into singular decomposed vectors
    keeps values in respective index
    """
    ls = []
    ln = len(arr)

    for i in range(ln):
        if isinstance(arr[i], float):
            new_ls = [0] * ln
            new_ls[i] = 1
            ls.append(new_ls)

    return ls

# if __name__== "__main__":
    
#     start = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
#     # input_list = [(1, 0, 0, 0, 0, 0),
#     #             (0, 1, 0, 0, 0, 0),
#     #             (0, 0, 1, 0, 0, 0),
#     #             (0, 0, 0, 0, 1, 0)]
#     input_list = decompose_array(list(start))
#     final = (2, 1, 2, 1, 2, 1)
#     mps = sum_6d_arrays_with_one_none_zero(start=start, final=final, input_list=input_list)
#     print(mps)

# motion_primitives = {
#     'UP_Z':   (0, 0, 10, 0, 0, 0), 
#     'DOWN_Z': (0, 0, -10, 0, 0, 0),
#     'FORWARD_X': (10, 0, 0, 0, 0, 0),
#     'BACKWARD_X': (-10, 0, 0, 0, 0, 0),
#     'FORWARD_Y': (0, 10, 0, 0, 0, 0),
#     'FORWARD_Y': (0, -10, 0, 0, 0, 0),
#     'UP_ALPHA':   (0, 0, 0, 0, 0, 10), 
#     'DOWN_ALPHA': (0, 0, 0, 0, 0, -10),
#     'FORWARD_BETA': (0, 0, 0, 0, 10, 0),
#     'BACKWARD_BETA': (0, 0, 0, 0, -10, 0),
#     'FORWARD_GAMMA': (0, 0, 0, 10, 0, 0),
#     'FORWARD_GAMMA': (0, 0, 0, -10, 0, 0)  
# }

# start_ee_position = (3.424549298862583, -58.49294620503255, 37.437693122514504, -178.8479662, -1.1794894, 46.7802049)
# end_goal_ee_position = (50.06332591417975, 25.10425547246271, 27.792718303142827, -173.8439543, -9.7114613, -45.186522)

# def mod_5_to_original(mod_5_representation):
#     # Choose a value for k (it can be any integer)
#     k = 0
#     original_number = mod_5_representation + k * 5
#     return original_number

# def compute_grasp_list(start_position, end_position, movement_list):
#     augmented_list = np.array(list(movement_list.values()) * 10)
    
#     start_array = np.array(start_ee_position) % 5
#     end_array = np.array(end_goal_ee_position) % 5
#     print(start_array)
#     print(end_array)
#     print("start back to reg", mod_5_to_original(start_array[0]))
#     print('end back to reg', mod_5_to_original(end_array[0]))
#     # difference_vector = start_array - end_array
#     # adjusted_vector = np.round(difference_vector / 5) * 5
#     # new_end_array = start_array - adjusted_vector
#     # new_end_array = tuple(new_end_array)

#     # print("start pos: ", start_position)
#     # print("new_end array: ", new_end_array)

#     # grasp_list = sum_6d_arrays_with_one_none_zero(start=np.floor(start_position), final=np.floor(new_end_array), input_list=augmented_list)
#     return # grasp_list

# print(compute_grasp_list(start_ee_position, end_goal_ee_position, motion_primitives))
from scipy.spatial.transform import Rotation as R
# start = [0, 0, 0, 0, 0, 0]
# end = [-14, 43, 58, -5, 12, 0]

# def combine_small_mps(mps: list, step_size: int):
#     for i in range(len(mps)):
#         smallest_val = min(mps[i])
#         smallest_idx = mps.index(smallest_val)
#         if smallest_val < step_size:
#             mps[i][smallest_idx - 1] += smallest_val
#             print('array we want to lose \n', mps[i])
#             mps.pop(i)

#     return


def to_quaternion(xyz_euler):
    r = R.from_euler('xyz', xyz_euler[3:], degrees=True)
    return r.as_quat()


def generate_arrays(step_size, index, start, end, diff):
    num_prims = int(abs((end - start)) // step_size)
    sign = 1 if diff >= 0 else -1

    prims = np.zeros((num_prims, 6))
    prims[:, index] = step_size * sign
    prims[-1][index] += (abs(diff) % step_size) * sign
    return prims


def find_mps(start_pos, end_pos, step_size):
    '''
    Takes in the start and end positions of the end effector, takes in step size of mps
    returns all mps of step size or less to get from start position to end position
    '''
    arrs = np.zeros((0, 6))
    for i in range(len(start_pos)):
        diff = end_pos[i] - start_pos[i]
        if diff == 0:
            continue

        # generate the needed number of lists with the step size
        if abs(diff) >= step_size:
            arrs = np.concatenate((arrs, generate_arrays(step_size, i, start_pos[i], end_pos[i], diff)))
        # generates the remainder list incase not exactly equal to step size 0->12 will need a mp of 2 and 10
        else:
            # arrs[-1][i] += diff
            new_prim = np.zeros((1, 6))
            new_prim[0][i] = diff
            arrs = np.concatenate((arrs, new_prim))
            # var = arrs[-2][i] + diff
            # print(f'the last array and the value + diff {var} \n', arrs[-1])

    return arrs
 
def compute_grasp_list(start_position, end_position, step_constant):
    '''
    Computes the mps needed to get from the start to end position given, returns all values in quaternion
    '''
    mps = find_mps(start_pos=np.floor(start_position), end_pos=np.floor(end_position), step_size=step_constant)
    print("mps: ", mps)
    locations = np.zeros((mps.shape[0] + 1, mps.shape[1]))
    locations[0] = start_position
    locations[1] = start_position + mps[0] 
    for i in range(2, locations.shape[0]):
        locations[i] = locations[i-1] + mps[i-1]

    quaternions = np.zeros((mps.shape[0] + 1, mps.shape[1] + 1))
    for i in range(len(locations)):
        quats = to_quaternion(locations[i])
        quaternions[i] = np.array([locations[i][0]/100, locations[i][1]/100 ,locations[i][2]/100, quats[0], quats[1], quats[2], quats[3]])

    return quaternions


s = [3.424549298862583, -58.49294620503255, 37.437693122514504, -178.8479662, -1.1794894, 46.7802049]
e = [50.06332591417975, 25.10425547246271, 27.792718303142827, -173.8439543, -9.7114613, -45.186522]
STEP_CONST = 20
c = compute_grasp_list(start_position=s, end_position=e, step_constant=STEP_CONST)
