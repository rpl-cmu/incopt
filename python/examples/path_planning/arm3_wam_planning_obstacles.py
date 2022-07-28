import os
BASE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import warnings
warnings.filterwarnings("ignore")

import gtsam
import incopt
import hydra
from incoptpy.thirdparty.gpmp2.datasets.generate3Ddataset import generate3Ddataset
from incoptpy.thirdparty.gpmp2.robots.generateArm import generateArm
from incoptpy.thirdparty.gpmp2.utils.plot_utils import *
from incoptpy.thirdparty.gpmp2.utils.signedDistanceField3D import signedDistanceField3D

DEFAULT_PARAMS = {
    "relinearizeSkip": 1, 
    "wildfireThreshold": 0.001,
    "relinearizeThreshold": 0.001,
    "numInnerIter": 1,
    "maxConstrainedDeltaStep": 0.01
}

global dataset, origin, origin_point3, cell_size, field
dataset = generate3Ddataset("WAMShelf")
origin = np.asarray([dataset.origin_x, dataset.origin_y, dataset.origin_z])
origin_point3 = gtsam.Point3(origin)
cell_size = dataset.cell_size

# sdf
print("calculating signed distance field ...")
field = signedDistanceField3D(dataset.map, dataset.cell_size)
print("calculating signed distance field done")

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
    isam.setRhoReduction(0.2)
    isam.setRhoMaxRhoVal(1e6)
    isam.setRhominRhoVal(2)
    isam.setRhoMinChange(0.5)
    isam.setCapByDim(np.array([0.001, 0.001*2, 0.001*5, 0.001*5, 0.001*5, 0.001*10, 0.001*10]))
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

def get_smoothness_estimate(currentEstimate, total_plot_step, DOF):
    diff = 0
    for i in range(1, total_plot_step):
        conf_c = currentEstimate.atVector(gtsam.symbol("x", i))    
        conf_p = currentEstimate.atVector(gtsam.symbol("x", i-1))
        # get wrap angle difference in the -pi to pi range
        tmp = 0
        for d in range(DOF):
            x = conf_c[d]
            y = conf_p[d]
            tmp += np.abs(np.arctan2(np.sin(x-y), np.cos(x-y)))  
        diff += tmp/DOF
    return diff 


def run(cfg, _params=DEFAULT_PARAMS):    
    # dataset
    print("===================================== {} ======================================".format(_params["label"]))
    figdir_outputs = f"{BASE_PATH}/{cfg.figdir_outputs}/"

    # arm: WAM arm
    arm = generateArm("WAMArm")
    start_conf = np.asarray([-0.8, -1.70, 1.64, 1.29, 1.1, -0.106, 2.2])
    end_conf = np.asarray([-0.0, 0.94, 0, 1.6, 0, -0.919, 1.55])

    start_vel = np.zeros(7)
    end_vel = np.zeros(7)

    # plot problem setting
    figure0 = plt.figure(0)
    axis0 = Axes3D(figure0)
    axis0.set_title("Problem Settings")
    set3DPlotRange(figure0, axis0, dataset)
    plotRobotModel(figure0, axis0, arm, start_conf)
    plotRobotModel(figure0, axis0, arm, end_conf)
    plotMap3D(figure0, axis0, dataset.corner_idx, origin, cell_size)

    total_time_sec = 2 
    total_time_step = 10 
    total_check_step = 100 
    delta_t = total_time_sec / total_time_step
    check_inter = int(total_check_step / total_time_step) - 1

    # GP
    Qc = np.identity(7)
    Qc_model = gtsam.noiseModel.Diagonal.Covariance(Qc)

    # algo settings
    cost_sigma = _params["cost_sigma"]
    epsilon_dist = 0.15 
    enable_constraints = _params["enable_constraints"]


    fix_sigma1 =  0.00001 
    pose_fix_model1 = gtsam.noiseModel.Isotropic.Sigma(7, fix_sigma1)
    vel_fix_model1 = gtsam.noiseModel.Isotropic.Sigma(7, fix_sigma1)

    fix_sigma2 =  0.00001 
    pose_fix_model2 = gtsam.noiseModel.Isotropic.Sigma(7, fix_sigma2)
    vel_fix_model2 = gtsam.noiseModel.Isotropic.Sigma(7, fix_sigma2)

    # init sdf
    sdf = incopt.SignedDistanceField(
        origin_point3, cell_size, field.shape[0], field.shape[1], field.shape[2]
    )
    for z in range(field.shape[2]):
        sdf.initFieldData(
            z, field[:, :, z]
        )  # TODO: check this line with its matlab counterpart

    #% plot settings
    plot_inter_traj = True
    plot_inter = 1
    if plot_inter_traj:
        total_plot_step = total_time_step * (plot_inter + 1)
    else:
        total_plot_step = total_time_step
    pause_time = total_time_sec / total_plot_step

    ## initial traj
    init_values = incopt.initArmTrajStraightLine(start_conf, end_conf, total_time_step)

    # plot initial traj
    if plot_inter_traj:
        plot_values = incopt.interpolateArmTraj(init_values, Qc_model, delta_t, plot_inter)
    else:
        plot_values = init_values

    DOF = len(plot_values.atVector(gtsam.symbol("x", 0)))

    ## init optimization
    graph = gtsam.NonlinearFactorGraph()
    graph_obs = gtsam.NonlinearFactorGraph()

    for i in range(total_time_step + 1):
        key_pos = gtsam.symbol("x", i)
        key_vel = gtsam.symbol("v", i)

        # priors
        if i == 0:
            graph.push_back(gtsam.PriorFactorVector(key_pos, start_conf, pose_fix_model1))
            graph.push_back(gtsam.PriorFactorVector(key_vel, start_vel, vel_fix_model1))
        elif i == total_time_step:
            graph.push_back(gtsam.PriorFactorVector(key_pos, end_conf, pose_fix_model2))
            graph.push_back(gtsam.PriorFactorVector(key_vel, end_vel, vel_fix_model2))
 
        # GP priors and cost factor
        if i > 0 :
            
            key_pos1 = gtsam.symbol("x", i - 1)
            key_pos2 = gtsam.symbol("x", i)
            key_vel1 = gtsam.symbol("v", i - 1)
            key_vel2 = gtsam.symbol("v", i)
            graph.push_back(
                incopt.GaussianProcessPriorLinear(
                    key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model
                )
            )
            # cost factor
            factor = incopt.ObstacleSDFFactorArm(key_pos, arm, sdf, cost_sigma, epsilon_dist)
            factor.setIsConstraintFactor(enable_constraints)
            factor.setConstraintFactorType(2) # 0: uncon, 1: equality, 2: inequality_leq, h(x)<=0, 3: inequality_geq, h(x)>=0
            graph.add(factor)
            graph_obs.add(factor)

            # GP cost factor
            if check_inter > 0:
                for j in range(1, check_inter + 1):
                    tau = j * (total_time_sec / total_check_step)

                    factor = incopt.ObstacleSDFFactorGPArm(
                        key_pos1,
                        key_vel1,
                        key_pos2,
                        key_vel2,
                        arm,
                        sdf,
                        cost_sigma,
                        epsilon_dist,
                        Qc_model,
                        delta_t,
                        tau,
                    )
                    factor.setIsConstraintFactor(enable_constraints)
                    factor.setConstraintFactorType(2) # 0: uncon, 1: equality, 2: inequality_leq, h(x)<=0, 3: inequality_geq, h(x)>=0
                    graph.add(factor)
                    graph_obs.push_back(factor)

    ## optimize!
    errorConstrainedFactors= []
    smoothness = []
    
    print("Initial collision cost: {}".format(graph_obs.unwhitenedError(init_values)))
    sm = get_smoothness_estimate(init_values, total_time_step, DOF)
    print("Initial unsmoothness estimate: {}".format(sm))


    if(not enable_constraints):
        use_LM = _params["use_LM"]
        use_trustregion_opt = _params["use_trustregion_opt"]
        use_isam2 = _params["use_isam2"]
        if use_LM:
            print("Optimize using LM...")
            parameters = gtsam.LevenbergMarquardtParams()  
            parameters.setlambdaInitial(1000.0)
            optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values, parameters)
        elif use_trustregion_opt:
            print("Optimize using DOGLEG...")
            parameters = gtsam.DoglegParams()
            optimizer = gtsam.DoglegOptimizer(graph, init_values, parameters)
        elif use_isam2:
            print("Optimize using ISAM2...")
            optimizer, _ = load_unconstrained_params(_params=_params)
        else:
            print("Optimize using Gauss Newton...")
            parameters = gtsam.GaussNewtonParams()
            optimizer = gtsam.GaussNewtonOptimizer(graph, init_values, parameters)

        if use_isam2:
            res = optimizer.update(graph, init_values)
            result = optimizer.calculateEstimate()
            errorObs = graph_obs.unwhitenedError(result)
            errorConstrainedFactors.append(errorObs)
            n_steps = _params["numOuterIter"]
            for i in range(0, n_steps):
                res = optimizer.update()
                result = optimizer.calculateEstimate()
                errorObs = graph_obs.unwhitenedError(result)
                diff_end = np.linalg.norm(result.atVector(gtsam.symbol("x", total_time_step))- end_conf)
                errorConstrainedFactors.append(errorObs)
                if i%10==0:
                    print("iteration {} | obstacle cost = {} | diff_end = {}".format(i, errorObs, diff_end))
                if errorObs < 1e-2 and diff_end < 1e-2:
                    break
        else:
            res = optimizer.optimizeSafely()
            result = optimizer.values()
            errorObs = graph_obs.unwhitenedError(result)
            errorConstrainedFactors.append(errorObs)

    else:
        print("Optimize using INCOPT...")
        optimizer, _ = load_constrained_params(_params=_params)
        res = optimizer.update(graph, init_values)
        if enable_constraints:
            result = optimizer.calculateEstimateConstrained()
        else:
            result = optimizer.calculateEstimate()
        errorObs = graph_obs.unwhitenedError(result)
        errorConstrainedFactors.append(errorObs)

        n_steps = _params["numOuterIter"]
        
        for i in range(0, n_steps):
            res = optimizer.update()
            if enable_constraints:
                result = optimizer.calculateEstimateConstrained()
            else:
                result = optimizer.calculateEstimate()
            errorObs = graph_obs.unwhitenedError(result)
            diff_end = np.linalg.norm(result.atVector(gtsam.symbol("x", total_time_step))- end_conf)
            if i%10==0:
                print("iteration {} | obstacle cost = {} | diff_end = {}".format(i, errorObs, diff_end))
            if errorObs < 1e-2 and diff_end < 1e-2:
                break
            errorConstrainedFactors.append(errorObs)
            diff_end = np.linalg.norm(result.atVector(gtsam.symbol("x", total_time_step))- end_conf)
            


    print("Final collision cost: {}".format(graph_obs.unwhitenedError(result)))
    sm = get_smoothness_estimate(result, total_time_step, DOF)
    smoothness.append(sm)
    print("Final unsmoothness estimate {}".format(sm))
    print("Final non-linear error {}".format(errorConstrainedFactors[-1]))
    diff_end = np.linalg.norm(result.atVector(gtsam.symbol("x", total_time_step))- end_conf)

    if plot_inter_traj:
        plot_values = incopt.interpolateArmTraj(result, Qc_model, delta_t, plot_inter)
    else:
        plot_values = result

    savePath = '{}/solution/{}'.format(figdir_outputs, _params["label"])
    if not os.path.exists(savePath):
        os.makedirs(savePath)

    for i in range(total_plot_step+1):
        figure3 = plt.figure(3, figsize=(12,10))
        axis3 = Axes3D(figure3)
        axis3.set_title("Result Values")
        axis3.view_init(elev=5., azim=-88)
        plotMap3D(figure3, axis3, dataset.corner_idx, origin, cell_size)
        set3DPlotRange(figure3, axis3, dataset)
        conf = plot_values.atVector(gtsam.symbol("x", i))
        plotRobotModel(figure3, axis3, arm, conf)
        figure3.savefig('{}/{}.png'.format(savePath, i))

    return errorConstrainedFactors, smoothness

@hydra.main(config_path=f"{BASE_PATH}/python/config", config_name="arm3_planning_obstacles")
def main(cfg):

    sigma_cost_space = cfg.sigma_cost_space

    algo = cfg.optimizationAlgorithm
    EXP_PARAMS_LS_UNCONSTRAINED = []
    for cost_sigma in sigma_cost_space: 
        if algo != "ISAM2":
            common =  {
                "enable_constraints": False,
                "enablePartialRelinearizationCheck": False,
                "cost_sigma": cost_sigma,
                "label": "{}_cost_sigma_{}".format(algo, cost_sigma),
                "use_isam2": False
            }        
            if algo == "LM":
                common["use_LM"] = True 
            else:
                common["use_LM"] = False
            if algo == "DOGLEG":
                common["use_trustregion_opt"] = True 
            else:
                common["use_trustregion_opt"] = False                
            EXP_PARAMS_LS_UNCONSTRAINED.append(common)
        else:
            EXP_PARAMS_LS_UNCONSTRAINED.append({
                "relinearizeSkip": 1, 
                "wildfireThreshold": 1e-15,
                "relinearizeThreshold": 1e-15,
                "numInnerIter": 100,
                "numOuterIter": 500,
                "maxConstrainedDeltaStep": 0.01,
                "enable_constraints": False,
                "enablePartialRelinearizationCheck": False,
                "cost_sigma": cost_sigma,
                "label": "ISAM2_cost_sigma_{}".format(cost_sigma),
                "use_LM": False,
                "use_trustregion_opt": False,
                "use_isam2": True
            })
        

    EXP_PARAMS_LS_CONSTRAINED = []
    
    for cost_sigma in sigma_cost_space: 
      EXP_PARAMS_LS_CONSTRAINED.append(
        {
            "relinearizeSkip": 1, 
            "wildfireThreshold": 1e-15,
            "relinearizeThreshold": 1e-15,
            "numInnerIter": 100,
            "numOuterIter": 500, 
            "maxConstrainedDeltaStep": 0.01,
            "enable_constraints": True,
            "enablePartialRelinearizationCheck": False,
            "cost_sigma": cost_sigma,
            "label": "INCOPT_cost_sigma_{}".format(cost_sigma)
        },
      )  

    if algo == "INCOPT":
        EXP_PARAMS_LS = EXP_PARAMS_LS_CONSTRAINED
    else: 
        EXP_PARAMS_LS = EXP_PARAMS_LS_UNCONSTRAINED

    for i, _ in enumerate(EXP_PARAMS_LS):
        run(cfg, _params=EXP_PARAMS_LS[i])




    


if __name__=='__main__':
    main()
