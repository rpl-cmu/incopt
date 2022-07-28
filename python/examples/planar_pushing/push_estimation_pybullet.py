import os
BASE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))

# use local cython installs
import sys
#sys.path.append(f"{BASE_PATH}/gtsam/install/cython")
# ffmpeg -y -framerate 30 -i %d.png -r 5 -codec:v libx264 -vf "tpad=stop_mode=clone:stop_duration=10" -pix_fmt yuv420p video40x4.mp4


import gtsam
import numpy as np
import math
import hydra
import time

import incopt 
import matplotlib.pyplot as plt
import json 

import warnings
warnings.filterwarnings("ignore")

plt.rc('axes', labelsize=18) 
plt.rc('legend', fontsize=16) 

from incoptpy.utils import vis_utils


plt.figure(figsize=(16, 12))

BASE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))

DEFAULT_PARAMS = {
    "relinearizeSkip": 10, 
    "wildfireThreshold": 0.001,
    "relinearizeThreshold": 0.001,
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


def plot_all_traj(cfg, figdir_outputs, estimated_traj_all_all, gt_traj_all_all, end_eff_pose_all_all, labels_all, colors_all):
    plt.cla()

    savePath = '{}/trajectories'.format(figdir_outputs)
    if not os.path.exists(savePath):
        os.makedirs(savePath)
    print("=============== PLOTTING TRAJECTORY ===================")
    # for all timesteps
    for t in range(len(estimated_traj_all_all[0])):

        # for each timestep, loop over the current estimate by each algorithm
        for k in range(len(estimated_traj_all_all)):

            obj_pose_vals_graph = estimated_traj_all_all[k][t]
            obj_pose_vals_gt = gt_traj_all_all[k][t]
            eff_pose_vals = end_eff_pose_all_all[k][t]
            n_steps = obj_pose_vals_graph.size()
            obj_poses_graph = np.zeros((n_steps, 3))
            obj_poses_gt = np.zeros((n_steps, 3))
            eff_poses = np.zeros((n_steps, 3))
            n_steps = obj_pose_vals_graph.size()
            obj_poses_graph = np.zeros((n_steps, 3))
            obj_poses_gt = np.zeros((n_steps, 3))
            eff_poses = np.zeros((n_steps, 3))
                
            for i in range(0, n_steps):
                key = gtsam.symbol('x', i)
                pose2 = obj_pose_vals_graph.atPose2(key)
                obj_poses_graph[i, :] = [pose2.x(), pose2.y(), pose2.theta()]
                pose2 = obj_pose_vals_gt.atPose2(key)
                obj_poses_gt[i, :] = [pose2.x(), pose2.y(), pose2.theta()]
                pose2 = eff_pose_vals.atPose2(key)
                eff_poses[i, :] = [pose2.x(), pose2.y(), pose2.theta()]

            vis_utils.draw_endeff(eff_poses, color='dimgray')
            
            vis_utils.draw_object(obj_poses_graph, shape='rect', color=colors_all[k], label='optimized ({})'.format(labels_all[k]))
            if k == 2:
                vis_utils.draw_object(obj_poses_gt, shape='rect', color='dimgray', label='groundtruth')
                vis_utils.draw_endeff(eff_poses, color='dimgray')

        plt.legend(loc='upper left')
        plt.savefig('{}/{}.png'.format(savePath,t ))
        plt.clf()
        plt.gca().axis('equal')
        plt.xlim((-0.5, 2.5))
        plt.ylim((-2.5, 1))   

def addGaussianNoise(pose, noisevec, poseNoisy, addNoise = True):
    if (addNoise):
        poseNoisy = pose.retract(noisevec)
    else:
        poseNoisy = pose
    return poseNoisy


def addDriftingGaussianNoise(posePrevNoisy, posePrevGT, poseCurrGT, noisevec, addNoise = True):
    poseBetween = posePrevGT.between(poseCurrGT)
    if (addNoise):
        poseBetweenNoisy = poseBetween.retract(noisevec)
    else:
        poseBetweenNoisy = poseBetween

    poseCurrNoisy = posePrevNoisy.compose(poseBetweenNoisy)

    return poseCurrNoisy


def retNonLinearError(enable_constraints, isam2, currentEstimate):
    if enable_constraints:
        constrainedNonLinearError = isam2.sumErrorConstrained()
        if math.isnan(constrainedNonLinearError):
            constrainedNonLinearError = 0
        return constrainedNonLinearError
    else:
        all_factors = isam2.getFactorsUnsafe()
        errors = 0
        for k in range(all_factors.size()):
            factor = all_factors.at(k)
            n = len(factor.keys())
            e = 0
            if(n == 1):
                e = factor.error(currentEstimate)
            errors += e
        return errors


def run_contact_estimation(cfg, _params=DEFAULT_PARAMS, common_seed=0):
    
    srcdir_dataset = f"{BASE_PATH}/{cfg.srcdir_dataset}/"
    dstdir_outputs = f"{BASE_PATH}/{cfg.dstdir_outputs}/"
    

    dataset_name = cfg.dataset_name
    
    enable_constraints = _params["enable_constraints"]
    
    pose0 = gtsam.Pose2(0.0, 0.0, 0.0)
    ifs =  f"{srcdir_dataset}/{dataset_name}.json"

    f = open(ifs)
    data = json.load(f)
    eePoses2D = data["ee_poses_2d"]
    objPoses2D = data["obj_poses_2d"]
    contactFlag = data["contact_flag"]
    contactNormalDirs2D = data["contact_normal_dirs2d"]
    objPolyShape = np.array(data["obj_poly_shape"]).T
    paramsDataset = data["params"]
    eeRadius = paramsDataset["ee_radius"]
    
    graph = gtsam.NonlinearFactorGraph()
    initValues = gtsam.Values()

    if(enable_constraints):
        isam2, _ = load_constrained_params(_params=_params)
    else:
        isam2, _ = load_unconstrained_params(_params=_params)

    priorNoiseModel = gtsam.noiseModel.Diagonal.Sigmas(np.array(cfg.noise_models.first_pose_prior))
    odometryNoiseModel = gtsam.noiseModel.Diagonal.Sigmas(np.array(cfg.noise_models.odom_motion))
    contactNoiseModel = gtsam.noiseModel.Diagonal.Sigmas(np.array(cfg.noise_models.contact_proj))    
    eeNoiseModel = gtsam.noiseModel.Diagonal.Sigmas(np.array(cfg.noise_models.eff_pose_prior))
    samplerOdomNoise = gtsam.Sampler(odometryNoiseModel, seed=common_seed)
    samplerEENoise = gtsam.Sampler(eeNoiseModel, seed=common_seed)
    
    contactStdVal = np.array(cfg.noise_models.contact_proj)[0]
    poseIdx = 0
    skipPose = 10
    numSteps = min(cfg.max_steps, len(objPoses2D) / skipPose)
    skipImpulse = 20
    counterImpulse = 0

    objPosePrev = pose0
    objPosePrevGT = pose0
    currentEstimate = gtsam.Values()

    poseValsGT = gtsam.Values()
    poseValsOdom = gtsam.Values()
    eePoseValsNoisy = gtsam.Values()

    contactConstraintUtils = incopt.contactConstraintUtils()

    errorConstrainedFactors = [] 
    relinearized_vars = []
    time_all = []

    gt_traj_all = []
    estimated_traj_all = []
    end_eff_pose_all = []
    contact_error_all = []
    rmsd_all = []
    for i in range(0, int(math.ceil(numSteps))):
        if i == 0:
            key0 = gtsam.symbol('x', 0)
            initValues.insert(key0, pose0)
            graph.addPriorPose2(key0, pose0, priorNoiseModel)
            isam2.update(graph, initValues)
            currentEstimate = isam2.calculateEstimate()

            poseValsOdom.insert(key0, pose0)
            poseValsGT.insert(key0, pose0)
            eePose = gtsam.Pose2(eePoses2D[0][0], eePoses2D[0][1], eePoses2D[0][2])
            eePoseValsNoisy.insert(key0, eePose)
            index_write = str(i).zfill(3)
            if enable_constraints:
                ssName = "{}/std_{}/{}ConIter{}.json".format(dstdir_outputs, contactStdVal, dataset_name, index_write)
            else:
                ssName = "{}/std_{}/{}UnconIter{}.json".format(dstdir_outputs, contactStdVal, dataset_name, index_write)
            contactConstraintUtils.writeOutputsJson(isam2=isam2, 
                                                    poseValsGraph=currentEstimate, 
                                                    poseValsGT=poseValsGT, 
                                                    poseValsOdom=poseValsOdom, 
                                                    eePoseValsNoisy=eePoseValsNoisy,
                                                    filename=ssName)
            graph.resize(0)
            initValues.clear()
            continue

        poseIdx = poseIdx + skipPose

        if ((contactFlag[poseIdx][0]) and (counterImpulse < skipImpulse)):
            counterImpulse = counterImpulse + 1

        key1 = gtsam.symbol('x', i - 1)
        key2 = gtsam.symbol('x', i)
        
        eePose = gtsam.Pose2(eePoses2D[poseIdx][0], eePoses2D[poseIdx][1], eePoses2D[poseIdx][2])
        objPoseCurrGT = gtsam.Pose2(objPoses2D[poseIdx][0], objPoses2D[poseIdx][1], objPoses2D[poseIdx][2])

        # add noise to endeff estimates
        eePoseNoisy = gtsam.Pose2()
        eePoseNoisy = addGaussianNoise(eePose, samplerEENoise.sample(), eePoseNoisy, True)

        noisevecOdom = samplerOdomNoise.sample()        
        if (contactFlag[poseIdx][0] and (counterImpulse >= skipImpulse)):
            objPoseCurrPred = addDriftingGaussianNoise(objPosePrev, objPosePrevGT, objPoseCurrGT, noisevecOdom, addNoise = True)
        else:
            objPoseCurrPred = objPoseCurrGT


        odometryMeas = objPosePrev.between(objPoseCurrPred)
        poseValsGT.insert(key2, objPoseCurrGT)
        eePoseValsNoisy.insert(key2, eePoseNoisy)

        # factor: add odometry measurements
        if enable_constraints:
            initValues.insert(key2, isam2.calculateEstimateConstrained().atPose2(key1))
        else:
            initValues.insert(key2, isam2.calculateEstimate().atPose2(key1))
        graph.add(gtsam.BetweenFactorPose2(key1, key2, odometryMeas, odometryNoiseModel))

        # factor: add contact measurements
        if (contactFlag[poseIdx][0] and (counterImpulse >= skipImpulse)):
            eeCenter__world = np.array([eePoseNoisy.x(), eePoseNoisy.y()])
            contactPoint__world = [eeCenter__world[0] + eeRadius * contactNormalDirs2D[poseIdx][0], 
                                   eeCenter__world[1] + eeRadius * contactNormalDirs2D[poseIdx][1]]
            contactMeas = incopt.ContactMeasurement(contactPoint__world, eeCenter__world, objPolyShape, eeRadius)
            factor = incopt.ContactPush2dUnaryFactor(key2, contactMeas, contactNoiseModel)
            if(enable_constraints):
                factor.setIsConstraintFactor(True)
                factor.setConstraintFactorType(1)
            else:
                factor.setIsConstraintFactor(False)
                factor.setConstraintFactorType(0)                
            graph.add(factor)
        
        start1 = time.time() 
        res = isam2.update(graph, initValues)
        end1 = time.time() 
        time_all.append(end1-start1)
        if enable_constraints:
            currentEstimate = isam2.calculateEstimateConstrained()
        else:
            currentEstimate = isam2.calculateEstimate()
        constrainedError = retNonLinearError(_params["enable_constraints"], isam2, currentEstimate)
        errorConstrainedFactors.append(constrainedError)
        relinearized_vars.append(res.getVariablesRelinearized())

        iii = 0 
        
        while(iii < _params["numOuterIter"]):
            start2 = time.time() 
            res = isam2.update()
            end2 = time.time() 
            time_all.append(end2-start2)
            iii+=1
            if enable_constraints:
                currentEstimate = isam2.calculateEstimateConstrained()
            else:
                currentEstimate = isam2.calculateEstimate()
            constrainedError = retNonLinearError(_params["enable_constraints"], isam2, currentEstimate)
            errorConstrainedFactors.append(constrainedError)
            relinearized_vars.append(res.getVariablesRelinearized())
        N = currentEstimate.size()
        totalError = np.array([0., 0., 0., 0.])
        for i in range(currentEstimate.size()):
            x = currentEstimate.atPose2(gtsam.symbol('x', i))
            xGT = poseValsGT.atPose2(gtsam.symbol('x', i))
            err = xGT.between(x)
            errLogMap = gtsam.Pose2.Logmap(err)
            translation_err = np.sqrt(errLogMap[0]**2 + errLogMap[1]**2)
            err = np.array([errLogMap[0], errLogMap[1], errLogMap[2], translation_err])
            totalError += np.square(err)

        rmsd_val = np.sqrt(totalError/N)
        rmsd_all.append(rmsd_val)
        constrainedError = retNonLinearError(_params["enable_constraints"], isam2, currentEstimate)
        contact_error_all.append(constrainedError)

        index_write = str(i).zfill(3)
        if enable_constraints:
            ssName = "{}/std_{}/{}ConIter{}.json".format(dstdir_outputs, contactStdVal, dataset_name, index_write)
        else:
            ssName = "{}/std_{}/{}UnconIter{}.json".format(dstdir_outputs, contactStdVal, dataset_name, index_write)

        contactConstraintUtils.writeOutputsJson(isam2=isam2, 
                                                poseValsGraph=currentEstimate, 
                                                poseValsGT=poseValsGT, 
                                                poseValsOdom=poseValsOdom, 
                                                eePoseValsNoisy=eePoseValsNoisy,
                                                filename=ssName)

        if (cfg.save_graph):
            ssName = "{}/std_{}/{}ConIter{}.dot".format(dstdir_outputs, contactStdVal, dataset_name, index_write)
            contactConstraintUtils.saveGraph(isam2, ssName)

        # reset variables for next iteration
        objPosePrevGT = objPoseCurrGT
        objPosePrev = currentEstimate.atPose2(key2)

        gt_traj_all.append(poseValsGT)
        end_eff_pose_all.append(eePoseValsNoisy)
        if enable_constraints:
            currentEstimate = isam2.calculateEstimateConstrained()
        else:
            currentEstimate = isam2.calculateEstimate()
        estimated_traj_all.append(currentEstimate)

        graph.resize(0)
        initValues.clear()

    if enable_constraints:
        currentEstimate = isam2.calculateEstimateConstrained()
    else:
        currentEstimate = isam2.calculateEstimate()
    N = currentEstimate.size()
    totalError = np.array([0., 0., 0.])
    for i in range(currentEstimate.size()):
        x = currentEstimate.atPose2(gtsam.symbol('x', i))
        xGT = poseValsGT.atPose2(gtsam.symbol('x', i))
        err = xGT.between(x)
        totalError += np.square(gtsam.Pose2.Logmap(err))

    return errorConstrainedFactors, relinearized_vars, time_all, gt_traj_all, end_eff_pose_all, estimated_traj_all, rmsd_all

@hydra.main(config_path=f"{BASE_PATH}/python/config", config_name="push_estimation_pybullet")
def main(cfg):
    EXP_PARAMS_LS = [
        {
            "relinearizeSkip": 1, 
            "wildfireThreshold": 0.001,
            "relinearizeThreshold":0.01,
            "numInnerIter": 100,
            "numOuterIter": 1,
            "maxConstrainedDeltaStep": 0, #NOT APPLICABLE,
            "label": "ISAM2",
            "enable_constraints": False,
            "enablePartialRelinearizationCheck": True,
            "plot_color": "tab:red"
        },        
        {
            "relinearizeSkip": 1, 
            "wildfireThreshold": 0.001, # decrease for better accuracy 
            "relinearizeThreshold":0.001, # decrease for better accuracy 
            "numInnerIter": 1, # increase for better accuracy 
            "numOuterIter": 1, # increase for better accuracy 
            "maxConstrainedDeltaStep": 10,
            "label": "InCOpt",
            "enablePartialRelinearizationCheck":  True, #for better accuracy set to False
            "enable_constraints": True,
            "plot_color": "tab:green"
        }
    ]

    common_seed = int(time.time())
    labels_all = []
    colors_all = []
    for i, r in enumerate(EXP_PARAMS_LS):
        labels_all.append(EXP_PARAMS_LS[i]["label"])
        colors_all.append(EXP_PARAMS_LS[i]["plot_color"])

    common_seed = int(time.time())
    all_errorConstrainedFactors = []
    time_all_all = []
    gt_traj_all_all = []
    end_eff_pose_all_all = []
    estimated_traj_all_all = []

    for i, r in enumerate(EXP_PARAMS_LS):
        print("============ Using {} =================".format(r["label"]))
        errorConstrainedFactors, relinearized_vars, time_all, gt_traj_all, end_eff_pose_all, estimated_traj_all, rmsd_all = run_contact_estimation(cfg, _params=EXP_PARAMS_LS[i], common_seed=common_seed)
        all_errorConstrainedFactors.append(errorConstrainedFactors)
        time_all_all.append(time_all)
        gt_traj_all_all.append(gt_traj_all)
        end_eff_pose_all_all.append(end_eff_pose_all)
        estimated_traj_all_all.append(estimated_traj_all)
        print("RMSD in x = {}".format(rmsd_all[-1][0]))
        print("RMSD in y = {}".format(rmsd_all[-1][1]))
        print("RMSD in theta = {}".format(rmsd_all[-1][2]))
        print("CONTACT ERROR {}".format(errorConstrainedFactors[-1]))
        print("RUNTIME {}".format(np.array(time_all).sum()))
        print("TOTAL NUM RELINEARIZATIONS {}".format(np.array(relinearized_vars).sum()))

    if cfg.plot_trajectory: 
        figdir_outputs = f"{BASE_PATH}/{cfg.figdir_outputs}/" 
        plot_all_traj(cfg, figdir_outputs, estimated_traj_all_all, gt_traj_all_all, end_eff_pose_all_all, labels_all, colors_all)
        plt.clf()   
    

if __name__=='__main__':
    main()
