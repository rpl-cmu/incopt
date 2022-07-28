import os
BASE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))

import gtsam
import numpy as np
import math
import hydra
import incopt 
import matplotlib.pyplot as plt
import time

import warnings
warnings.filterwarnings("ignore")

from maze.solve_maze import solveMaze

plt.rc('axes', labelsize=14) 
plt.rc('legend', fontsize=12) 

object_methods = [method_name for method_name in dir(gtsam.gtsam.Values)
                 if callable(getattr(gtsam.gtsam.Values, method_name))]

odometryUp = np.array([-0.0, 0.5])
odometryLeft =  np.array([-0.5, -0.0])
odometryRight =  np.array([0.5, -0.0])
odometryDown = np.array([-0.0, -0.5])

moveUp = np.array([0.0, 0.5])
moveLeft =  np.array([-0.5, 0.])
moveRight =  np.array([0.5, 0.])
moveDown =  np.array([0, -0.5])

addBoundUp =  np.array([0.25, 0.25])
addBoundDown =  np.array([-0.25, -0.25])

initEstimate =  np.array([0, 0])

poseUnaryNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array([10, 10]))



DEFAULT_PARAMS = {
    "relinearizeSkip": 1, 
    "wildfireThreshold": 1e-3,
    "relinearizeThreshold": 0.1,
    "numInnerIter": 1,
    "maxConstrainedDeltaStep": 0.1
}


def load_constrained_params(enable_constraints=True, _params = DEFAULT_PARAMS):
    print("LOADED CONSTRAINED PARAMS")
    gaussNewtonParams = gtsam.ISAM2GaussNewtonParams()
    gaussNewtonParams.setWildfireThreshold(_params["wildfireThreshold"])

    params_isam2 = gtsam.ISAM2Params()
    params_isam2.setOptimizationParams(gaussNewtonParams)
    params_isam2.setRelinearizeThreshold(_params["relinearizeThreshold"])
    params_isam2.setRelinearizeSkip(_params["relinearizeSkip"])
    params_isam2.setFactorization("QR")
    params_isam2.setEnablePartialRelinearizationCheck(_params["enablePartialRelinearizationCheck"])
    params_isam2.setCacheLinearizedFactors(False)
    params_isam2.setEnableDetailedResults(True)
    params_isam2.setForceConstrainedDeltaUpdate(False)

    isam = gtsam.ISAM2(params_isam2)
    isam.setConstraintDeltaMaxStep(_params["maxConstrainedDeltaStep"])
    isam.setNumInnerIter(_params["numInnerIter"])
    isam.setEnableConstraints(enable_constraints)
    isam.setDebugPrint(False)
    isam.setRhoGrowth(5)
    isam.setRhoReduction(1)
    isam.setRhoMaxRhoVal(1e2)
    isam.setRhominRhoVal(2)
    isam.setRhoMinChange(0.5)
    return isam, params_isam2

def load_unconstrained_params(enable_constraints=False, _params = DEFAULT_PARAMS):
    print("LOADED UNCONSTRAINED PARAMS")
    gaussNewtonParams = gtsam.ISAM2GaussNewtonParams()
    gaussNewtonParams.setWildfireThreshold(_params["wildfireThreshold"])

    params_isam2 = gtsam.ISAM2Params()
    params_isam2.setOptimizationParams(gaussNewtonParams)
    params_isam2.setRelinearizeThreshold(_params["relinearizeThreshold"])
    params_isam2.setRelinearizeSkip(_params["relinearizeSkip"])
    params_isam2.setFactorization("QR")
    params_isam2.setEnablePartialRelinearizationCheck(_params["enablePartialRelinearizationCheck"])
    params_isam2.setCacheLinearizedFactors(False)
    params_isam2.setEnableDetailedResults(True)
    params_isam2.setForceConstrainedDeltaUpdate(False)

    isam = gtsam.ISAM2(params_isam2)
    isam.setConstraintDeltaMaxStep(_params["maxConstrainedDeltaStep"])
    isam.setNumInnerIter(_params["numInnerIter"])
    isam.setEnableConstraints(enable_constraints)
    isam.setDebugPrint(False)
    return isam, params_isam2

def retNonLinearError(cfg, isam, currentEstimate, enable_constraints):
    if enable_constraints:
        constrainedNonLinearError = isam.sumErrorConstrained()
        if math.isnan(constrainedNonLinearError):
            constrainedNonLinearError = 0
    else:
        all_factors = isam.getFactorsUnsafe()
        constrainedNonLinearError = 0
        for k in range(all_factors.size()):
            factor = all_factors.at(k)
            n = len(factor.keys())
            e = 0
            if(n == 1):
                e = factor.error(currentEstimate)
            constrainedNonLinearError += e
    return constrainedNonLinearError

def addToMap(factor1=None, factor2=None, var_count=0, action=None):
    unaryFactors = []
    if factor1:
        unaryFactors.append(factor1)
    if factor2:
        unaryFactors.append(factor2)
    index = var_count
    allFactors[index] = [unaryFactors, action]

def create_factor_unary(x, constraint, constraint_type, enable_constraints, noiseModel):
    if enable_constraints:
        factor = incopt.QuadraticUnaryFactorVector2ConstrainedHinge(x, constraint, constraint_type, noiseModel)
        factor.setIsConstraintFactor(enable_constraints)
        factor.setConstraintFactorType(constraint_type)
    else:
        if constraint_type == 2:
            take = 0
        elif constraint_type == 3:
            take = 1
        factor = incopt.QuadraticUnaryFactorVector2Hinge(x, constraint, take, noiseModel)
        factor.setIsConstraintFactor(0)
        factor.setConstraintFactorType(0)
    return factor

def create_factor_binary(xP, xN, constraint, constraint_type, enable_constraints):

    factor = incopt.BinaryFactorPointRobot(xP, xN, constraint, odometryNoise)
    factor.setIsConstraintFactor(False)
    factor.setConstraintFactorType(0)
    return factor 

def addGaussianNoise(pose, noisevec, addNoise = True):
    if (addNoise):
        poseNoisy = pose + noisevec
    else:
        poseNoisy = pose
    return poseNoisy


def add(command, var_count, enable_constraints, prevEstimate, samplerPoseNoise, samplerOdomNoise, poseN):
    xN = gtsam.symbol('x', var_count + 1) 
    xP = gtsam.symbol('x', var_count) 
    if command == "UP":
        boundDown = poseN + addBoundDown
        boundUp = poseN + addBoundUp
        initEstimate = odometryUp + prevEstimate
	
    elif command == "DOWN":
        boundDown = poseN + addBoundDown
        boundUp = poseN + addBoundUp
        initEstimate = odometryDown + prevEstimate
    
    elif command == "RIGHT":
        boundDown = poseN + addBoundDown
        boundUp = poseN + addBoundUp
        initEstimate = odometryRight + prevEstimate

    elif command == "LEFT":
        boundDown = poseN + addBoundDown
        boundUp = poseN + addBoundUp
        initEstimate = odometryLeft + prevEstimate

    factor1 = create_factor_unary(xN, boundDown, 3, enable_constraints, poseUnaryNoise)
    factor2 = create_factor_unary(xN, boundUp, 2, enable_constraints, poseUnaryNoise)

    var_count+=1 
    addToMap(factor1, factor2, var_count, command)
    prevEstimate = initEstimate
    return var_count, prevEstimate


def RMSE(GT, Val, label):
    N = GT.size()
    totalError = np.array([0., 0.])
    translation_error = 0
    for i in range(GT.size()):
        x = Val.atVector(gtsam.symbol('x', i+1))
        xGT = GT.atVector(gtsam.symbol('x', i+1))
        totalError += np.square(xGT-x)
        translation_error += np.linalg.norm(xGT - x)
    return np.sqrt(totalError/N), translation_error/N



def plotEstimateAlt(PATHS, maze, cfg, rm=None, plot_maze=True): 

    rmse_per_tick = [0, 0, 0]
    num_algorithms = len(PATHS)

    # Loop over the total number of time steps
    for i in range(0, len(PATHS[0])):
        estimates_gt = PATHS[0][i]
        paths_xy = []
        # loop over each algorithm at each time step
        for a in range(num_algorithms):
            xy = []
            # generate the estimate of algorithm a at timestep i
            for j in range(1, estimates_gt.size()+1):
                p = PATHS[a][i].atVector(gtsam.symbol('x', j))
                xy.append(p)
            paths_xy.append(xy)                  
        if(plot_maze):
            maze.write_svg("{}/{}/trajectories/maze{}.svg".format(BASE_PATH, cfg.figdir_outputs, i), paths_xy)
        rmse_isam2, _ = RMSE(PATHS[0][i], PATHS[1][i], "ISAM2")
        rmse_ics, _ = RMSE(PATHS[0][i], PATHS[2][i], "ICS")
        rmse_incopt, _ = RMSE(PATHS[0][i], PATHS[3][i], "INCOPT(1)")
        rmse_per_tick[0] += rmse_isam2
        rmse_per_tick[1] += rmse_ics
        rmse_per_tick[2] += rmse_incopt


    print("Average RMSD | ISAM2 = {}".format(rmse_per_tick[0]/len(PATHS[0])))
    print("Average RMSD | ICS = {}".format(rmse_per_tick[1]/len(PATHS[0])))
    print("Average RMSD | INCOPT = {}".format(rmse_per_tick[2]/len(PATHS[0]))) 


def generate_maze(cfg):
    satisfied = False
    while not satisfied:
        maze, path = solveMaze(nx=15, ny=15)
        pathGT = []                
        pathGT.insert(0, [0.5, 0.5])
        path.insert(0, [1, 1])
        for i, e in enumerate(path):
            if(i == 0 ): continue
            try:
                prev = pathGT[-1]
                if(path[i][0] > path[i-1][0]):
                    pathGT.append([prev[0] + 0.5, prev[1]])
                elif(path[i][0] < path[i-1][0]): 
                    pathGT.append([prev[0] - 0.5, prev[1]])
                elif(path[i][1] > path[i-1][1]):
                    pathGT.append([prev[0], prev[1] + 0.5])
                elif(path[i][1] < path[i-1][1]): 
                    pathGT.append([prev[0], prev[1] - 0.5])
            except:
                pass
        
        
        pathsGTAll = []
        for i in range(0, len(pathGT)):
            gtValues = gtsam.Values()
            for j in range(0, i+1):
                est = np.array([pathGT[j][0] , pathGT[j][1]])
                gtValues.insert(gtsam.symbol('x', j+1), est)           
            pathsGTAll.append(gtValues)

        PATHS = [pathsGTAll]
        
        if len(PATHS[0]) >= 150 and len(PATHS[0]) <= 200:
            satisfied = True 
    return PATHS, pathsGTAll, pathGT, maze

def run(cfg, _params=DEFAULT_PARAMS, maze=None, seed=1, pathGT=None):
    samplerOdomNoise = gtsam.Sampler(odometryNoise, seed=seed)
    samplerPoseNoise = gtsam.Sampler(poseInitNoise, seed=seed)
    graph = gtsam.NonlinearFactorGraph()
    initialEstimate = gtsam.Values()
    initialEstimateAll = gtsam.Values()
    currentEstimate = gtsam.Values()
    estimatesAll = []
    enable_constraints = _params["enable_constraints"]
    if enable_constraints:
        isam, _ = load_constrained_params(_params=_params)
        isam.setEnableConstraints(True)
    else:
        isam, _ = load_unconstrained_params(_params=_params)
        isam.setEnableConstraints(False)

    prior = np.array([pathGT[0][0], pathGT[0][1]])
    var_count = 1
    x1 = gtsam.symbol('x', var_count) 
    priorFactor = incopt.UnaryVector2Factor(x1, prior, priorNoise)
    graph.add(priorFactor)
    prevEstimate = prior

    boundDown = prior + addBoundDown

    boundUp = addBoundUp + prior
    factor1 = create_factor_unary(x1, boundDown, 3, enable_constraints, poseUnaryNoise)
    factor2 = create_factor_unary(x1, boundUp, 2, enable_constraints, poseUnaryNoise)
    addToMap(factor1= factor1, factor2=factor2, var_count=1)
    
    for i in range(1, len(pathGT)):
        if(pathGT[i][0] > pathGT[i-1][0]):
            command = "RIGHT"
        elif(pathGT[i][0] < pathGT[i-1][0]): 
            command = "LEFT"
        elif(pathGT[i][1] > pathGT[i-1][1]):
            command = "UP"
        elif(pathGT[i][1] < pathGT[i-1][1]): 
            command = "DOWN"
        PoseN = np.array([pathGT[i][0], pathGT[i][1]])
        var_count, prevEstimate = add(command, var_count, enable_constraints, prevEstimate, samplerPoseNoise, samplerOdomNoise, PoseN)

    
    nonLinearErrors = []
    num_vars = []
    relinearized_vars = []
    index = 0
    time_all = []
    averaged_non_linear_error = []
    estimates_all_per_step = []
    for i, e in allFactors.items():
        per_iter_non_linear_error = 0 
        num_updates_calls = 0
        unaryFactors = e[0]
        command = e[1]

        if i > 1:
            posePrev = currentEstimate.atVector(gtsam.symbol('x', i-1))
            if command == "UP":
                odomNoisy = addGaussianNoise(odometryUp, samplerOdomNoise.sample(), True)
            elif command == "DOWN":
                odomNoisy = addGaussianNoise(odometryDown, samplerOdomNoise.sample(), True)
            elif command == "RIGHT":
                odomNoisy = addGaussianNoise(odometryRight, samplerOdomNoise.sample(), True)
            elif command == "LEFT":
                odomNoisy = addGaussianNoise(odometryLeft, samplerOdomNoise.sample(), True)
            initEst = odomNoisy + posePrev
   
            xN = gtsam.symbol('x', i) 
            xP = gtsam.symbol('x', i-1)
            binaryFactor = create_factor_binary(xP, xN, odomNoisy, 0, enable_constraints)
            graph.add(binaryFactor)
        else:
            initEst = prior

        for f in unaryFactors:
            graph.add(f)
        
        initialEstimate.insert(gtsam.symbol('x', i), initEst)
        initialEstimateAll.insert(gtsam.symbol('x', i), initEst)
        
        start1 = time.time() 
        res = isam.update(graph, initialEstimate)
        num_updates_calls+=1
        end1 = time.time()

         # Gather STATS
        time_all.append(end1 - start1)
        num_vars.append(i)
        relinearized_vars.append(res.getVariablesRelinearized())
        if not cfg.gen_timing_results:
            if enable_constraints:
                currentEstimate = isam.calculateEstimateConstrained()
            else:
                currentEstimate = isam.calculateEstimate()
            constrainedError = retNonLinearError(cfg, isam, currentEstimate, enable_constraints)
            estimates_all_per_step.append(currentEstimate)
        else:
            constrainedError = 0
        per_iter_non_linear_error += constrainedError
        nonLinearErrors.append(constrainedError)

        num_updates = 0
        while(num_updates < _params["numOuterIter"]):
            start2 = time.time()
            res = isam.update()
            end2 = time.time()

             # Gather STATS
            time_all.append(end2 - start2)
            num_updates_calls+=1
            num_updates+=1
            num_vars.append(i)
            relinearized_vars.append(res.getVariablesRelinearized())
            if not cfg.gen_timing_results:
                if enable_constraints:
                    currentEstimate = isam.calculateEstimateConstrained()
                else:
                    currentEstimate = isam.calculateEstimate()
                constrainedError = retNonLinearError(cfg, isam, currentEstimate, enable_constraints)
                estimates_all_per_step.append(currentEstimate)
            else:
                constrainedError = 0
            per_iter_non_linear_error += constrainedError
            nonLinearErrors.append(constrainedError)

            index+=1

        
        graph.resize(0)
        initialEstimate.clear()
        if cfg.gen_timing_results:
           if enable_constraints:
               currentEstimate = isam.calculateEstimateConstrained()
           else:
               currentEstimate = isam.calculateEstimate()
        
        estimatesAll.append(currentEstimate)
        estimates_all_per_step.append(currentEstimate)
        averaged_non_linear_error.append(per_iter_non_linear_error/_params["numOuterIter"])
    

    currentEstimate = isam.calculateEstimate()
    return nonLinearErrors, currentEstimate, estimatesAll, time_all, relinearized_vars, averaged_non_linear_error, estimates_all_per_step


@hydra.main(config_path=f"{BASE_PATH}/python/config", config_name="navigation_2D")
def main(cfg):

    figure_path = "{}/{}/trajectories".format(BASE_PATH, cfg.figdir_outputs)
    if not os.path.exists(figure_path):
        os.makedirs(figure_path)

    PATHS, pathsGTAll, pathGT, maze = generate_maze(cfg)

    global odometryNoise, poseInitNoise, priorNoise, allFactors
    allFactors = {}    
    priorNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array(cfg.noise_models.priorNoise))
    odometryNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array(cfg.noise_models.odometryNoise))
    poseInitNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array(cfg.noise_models.poseInitNoise))
    EXP_PARAMS_LS = [
        {
            "relinearizeSkip": 5, 
            "wildfireThreshold": 0.001,
            "relinearizeThreshold": 0.1,
            "numInnerIter": 1,
            "numOuterIter": 2,
            "maxConstrainedDeltaStep": 0.1,
            "enable_constraints": False,
            "enablePartialRelinearizationCheck": True,
            "label": "ISAM2",
            "plot_color": "tab:red"
        },
        {
            "relinearizeSkip": 100, # Change to 5 when we want to compare timing against INcopt to achieve similar performance 
            "wildfireThreshold": 1e-15,
            "relinearizeThreshold": 1e-15,
            "numInnerIter": 100,
            "numOuterIter": 2,
            "maxConstrainedDeltaStep": 0.1,
            "enable_constraints": True,
            "enablePartialRelinearizationCheck": False,
            "label": "ICS",
            "plot_color": "tab:blue"
         },
        {
            "relinearizeSkip": 5, 
            "wildfireThreshold": 0.001,
            "relinearizeThreshold": 0.1,
            "numInnerIter": 100,
            "numOuterIter": 2,
            "maxConstrainedDeltaStep": 0.1,
            "enable_constraints": True,
            "enablePartialRelinearizationCheck": True,
            "label": "INCOPT",
            "plot_color": "tab:green"
        },
    ]
    

            
    pathsGT_all_per_step = []
    for j in range(len(pathsGTAll)):
        for k in range(int(EXP_PARAMS_LS[0]["numOuterIter"])):
            pathsGT_all_per_step.append(pathsGTAll[j])

    PATHS_ALL = [pathsGT_all_per_step]

    common_seed = int(time.time())
    all_errorConstrainedFactors = []
    time_all_all = []
    num_relinearized_vars = []
    all_averaged_non_linear_error = []
    for i, r in enumerate(EXP_PARAMS_LS):
        errorConstrainedFactors, estimate, estimatesAll, time_all, relinearized_vars, averaged_non_linear_error, estimates_all_per_step = run(cfg, _params=EXP_PARAMS_LS[i],  maze=maze, seed= common_seed, pathGT=pathGT)
        all_errorConstrainedFactors.append(errorConstrainedFactors)
        num_relinearized_vars.append(relinearized_vars)
        PATHS.append(estimatesAll)
        PATHS_ALL.append(estimates_all_per_step)
        time_all_all.append(time_all)
        all_averaged_non_linear_error.append(averaged_non_linear_error)

    for i in range(len(EXP_PARAMS_LS)):
        print("Total time {} = {}".format(EXP_PARAMS_LS[i]["label"], np.array(time_all_all[i]).sum()))
        print("Total num relinearization {} = {}".format(EXP_PARAMS_LS[i]["label"], np.array(num_relinearized_vars[i]).sum()))
        print("Constraint violation {} = {}".format(EXP_PARAMS_LS[i]["label"], np.array(all_errorConstrainedFactors[i])[-1]))

    plotEstimateAlt(PATHS, maze, cfg)


if __name__=='__main__':
    main()