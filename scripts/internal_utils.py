import os
import json
import glob
import re
from tqdm import tqdm

import numpy as np
from evo import EvoException
import evo.core.lie_algebra as lie
import evo.core.transformations as tr
from evo.core import result
from evo.core.trajectory import PosePath3D, PoseTrajectory3D
from evo.tools import user
from evo.common_ape_rpe import *
from evo.core.metrics import PoseRelation
from evo.tools import file_interface
from evo.tools import plot
import evo.common_ape_rpe as ape_rpe
import matplotlib.pyplot as plt


def get_inuse_landmark_number(json_input):
    landmarks = json_input["landmarks"]
    counter = 0
    for lm in landmarks:
        is_initialized = lm["value"]["is_initialized"]["atomic_data"]
        is_added = lm["value"]["is_added"]["atomic_data"]
        is_outlier = lm["value"]["is_outlier"]["atomic_data"]
        if (is_initialized and (not is_outlier)):
            counter = counter + 1
    return counter


def get_landmark_number_by_age(json_input, min_age):
    landmarks = json_input["feature_point_age"]
    counter = 0
    for lm in landmarks:
        age = lm["value"]
        if age >= min_age:
            counter = counter + 1
    return counter


def get_pose(json_input):
    pose = json_input["body_pose"]
    pos = np.array([float(pose["px"]), float(pose["py"]), float(pose["pz"])])
    quat = np.array([float(pose["qw"]), float(pose["qx"]), float(pose["qy"]), float(pose["qz"])])
    return pos, quat, float(json_input["timestamp"])


def ape(traj_ref, traj_est, pose_relation, align=False, correct_scale=False,
        align_origin=False, ref_name="reference", est_name="estimate"):
    from evo.core import metrics
    from evo.core import trajectory

    # Align the trajectories.
    only_scale = correct_scale and not align
    if align or correct_scale:
        traj_est = trajectory.align_trajectory(traj_est, traj_ref,
                                               correct_scale, only_scale)
    elif align_origin:
        traj_est = trajectory.align_trajectory_origin(traj_est, traj_ref)

    # Calculate APE.
    data = (traj_ref, traj_est)
    ape_metric = metrics.APE(pose_relation)
    ape_metric.process_data(data)

    title = str(ape_metric)
    if align and not correct_scale:
        title += "\n(with SE(3) Umeyama alignment)"
    elif align and correct_scale:
        title += "\n(with Sim(3) Umeyama alignment)"
    elif only_scale:
        title += "\n(scale corrected)"
    elif align_origin:
        title += "\n(with origin alignment)"
    else:
        title += "\n(not aligned)"

    ape_result = ape_metric.get_result(ref_name, est_name)
    ape_result.info["title"] = title

    ape_result.add_trajectory(ref_name, traj_ref)
    ape_result.add_trajectory(est_name, traj_est)
    if isinstance(traj_est, trajectory.PoseTrajectory3D):
        seconds_from_start = [
            t - traj_est.timestamps[0] for t in traj_est.timestamps
        ]
        ape_result.add_np_array("seconds_from_start", seconds_from_start)
        ape_result.add_np_array("timestamps", traj_est.timestamps)

    return ape_result


if __name__ == "__main__":
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-14-14-25-56/'
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-14-14-17-08/'
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-14-16-01-05/'
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-14-17-13-11/'
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-14-22-48-04'
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-14-22-54-34'
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-15-21-56-03'
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-15-22-00-57'
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-15-22-02-14'
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-15-22-10-09'
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-15-22-10-09'
    # path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-17-00-07-01'
    path_to_frame_dir = '/e/subspace/docker_work/dataset/result/logs/2020-06-20-11-56-46'

    path_to_reference_trajectory = "/e/subspace/docker_work/dataset/V1_02_medium/mav0/state_groundtruth_estimate0/data.csv"

    frame_list = glob.glob(path_to_frame_dir + "/frames/frame_*.json")
    frame_path_list = [""] * len(frame_list)
    for pframe in frame_list:
        fname = os.path.basename(pframe)
        fidx = int(re.findall(r'\d+', fname)[0])
        frame_path_list[fidx] = pframe

    frame_internals = []
    for idx in tqdm(range(0, len(frame_list))):
        j_string = open(frame_path_list[idx], 'r')
        j = json.load(j_string)
        frame_internals.append(j["value0"])

    inuse_lm_number = []
    aged_1_lm_number = []
    aged_2_lm_number = []
    aged_3_lm_number = []
    aged_4_lm_number = []

    traj_position = np.full((len(frame_internals), 3), 0.0)
    traj_orientation = np.full((len(frame_internals), 4), 0.0)
    traj_timestamp = np.full((len(frame_internals), 1), 0.0)

    feature_pass_rate_tracking = []
    feature_pass_rate_verification = []
    feature_pass_rate_vision_frontend = []

    count = 0
    for fi in frame_internals:
        lm_number = get_inuse_landmark_number(fi)
        inuse_lm_number.append(lm_number)
        aged_1_lm_number.append(get_landmark_number_by_age(fi, 1))
        aged_2_lm_number.append(get_landmark_number_by_age(fi, 2))
        aged_3_lm_number.append(get_landmark_number_by_age(fi, 3))
        aged_4_lm_number.append(get_landmark_number_by_age(fi, 4))
        pos, ori, tstamp = get_pose(fi)
        traj_position[count, :] = pos
        traj_orientation[count, :] = ori
        traj_timestamp[count, :] = tstamp

        if fi["features_pre_frame_number"] != 0:
            feature_pass_rate_tracking.append(
                float(fi["features_after_tracking_number"]) / (float(fi["features_pre_frame_number"])))
        else:
            feature_pass_rate_tracking.append(0)
        if fi["is_keyframe"]:
            if fi["features_after_tracking_number"] != 0:
                feature_pass_rate_verification.append(
                    float(fi["features_after_verification_number"]) / float(fi["features_after_tracking_number"]))
                feature_pass_rate_vision_frontend.append(
                    float(fi["features_after_verification_number"]) / float(fi["features_pre_frame_number"]))
            else:
                feature_pass_rate_verification.append(0)
                feature_pass_rate_vision_frontend.append(0)
        else:
            feature_pass_rate_verification.append(feature_pass_rate_verification[-1])
            feature_pass_rate_vision_frontend.append(feature_pass_rate_vision_frontend[-1])
        count = count + 1

    print(count)
    traj_estimated = PoseTrajectory3D(traj_position, traj_orientation, traj_orientation)
    traj_reference_raw = file_interface.read_euroc_csv_trajectory(path_to_reference_trajectory)
    # Sync pose
    count = 0
    traj_position_ref = np.full((len(frame_internals), 3), 0.0)
    traj_orientation_ref = np.full((len(frame_internals), 4), 0.0)
    traj_timestamp_ref = np.full((len(frame_internals), 1), 0.0)
    for idx in range(0, len(traj_timestamp)):
        est_t = traj_timestamp[idx]
        nn_idx_gt = (np.abs(traj_reference_raw.timestamps - est_t)).argmin()
        traj_position_ref[idx] = traj_reference_raw.positions_xyz[nn_idx_gt]
        traj_orientation_ref[idx] = traj_reference_raw.orientations_quat_wxyz[nn_idx_gt]
        traj_timestamp_ref[idx] = traj_reference_raw.timestamps[nn_idx_gt]
    traj_reference_sync = PoseTrajectory3D(traj_position_ref, traj_orientation_ref, traj_timestamp_ref)

    # Compute APE
    # pose_relation = get_pose_relation("trans_part")
    pose_relation = PoseRelation.full_transformation
    result = ape(
        traj_ref=traj_reference_sync,
        traj_est=traj_estimated,
        pose_relation=pose_relation,
        align=True,
        correct_scale=True,
        align_origin=True,
    )

    plt.figure(1)
    plt.plot(inuse_lm_number, "-")
    plt.plot(aged_1_lm_number, "-")
    plt.plot(aged_2_lm_number, "-")
    plt.plot(aged_3_lm_number, "-")
    plt.plot(aged_4_lm_number, "-")
    plt.legend(["inuse", "age_1", "age_2", "age_3", "age_4"])
    plt.savefig(path_to_frame_dir + "/feature_number.png")

    plot_mode = plot.PlotMode["xyz"]
    fig_res = plt.figure(2)
    ax_res = plot.prepare_axis(fig_res, plot_mode)
    plot.traj(ax_res, plot_mode, traj_reference_sync)
    plot.traj_colormap(ax_res, result.trajectories['estimate'], result.np_arrays["error_array"],
                       plot_mode, min_map=result.stats["min"],
                       max_map=result.stats["max"],
                       title="Error mapped onto trajectory")
    plt.savefig(path_to_frame_dir + "/trajectory.png")

    fig_error = plt.figure(3)
    plt.plot(result.np_arrays["error_array"])
    plt.axhline(result.stats["rmse"], color='b')
    plt.axhline(result.stats["median"], color="r")
    plt.axhline(result.stats["mean"], color="g")
    plt.savefig(path_to_frame_dir + "/trajectory_error.png")

    fig_pass_rate = plt.figure(4)
    plt.plot(feature_pass_rate_verification)
    plt.plot(feature_pass_rate_tracking)
    plt.plot(feature_pass_rate_vision_frontend)
    plt.legend(["Verification", "Tracking", "VisionFrontend"])
    plt.ylabel("Feature pass rate")
    plt.xlabel("Frame number")
    plt.savefig(path_to_frame_dir + "/feature_pass_rate.png")

    plt.show()
