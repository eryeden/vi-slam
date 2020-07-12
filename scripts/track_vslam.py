import os
from mlflow import log_metric, log_param, log_artifact
import mlflow
import subprocess
import tempfile
import json
import glob
import re
from tqdm import tqdm
import argparse

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

def get_path_to_reference_trajectory(path_to_result_directory):
    dataset_config = open(path_to_result_directory + "/EurocKimeraDataProvider.json", 'r')
    json_dataset_config = json.load(dataset_config)
    return json_dataset_config["value0"]["euroc_dataset_root"] + "/mav0/state_groundtruth_estimate0/data.csv"


def get_path_to_dataset(path_to_result_directory):
    dataset_config = open(path_to_result_directory + "/EurocKimeraDataProvider.json", 'r')
    json_dataset_config = json.load(dataset_config)
    return json_dataset_config["value0"]["euroc_dataset_root"]


def get_all_frame_internals_json(path_to_result_directory):
    frame_list = glob.glob(path_to_result_directory + "/frames/frame_*.json")
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
    return frame_internals


def get_lm_database(path_to_result_directory):
    lm_database_json = open(path_to_result_directory + "/Mapdatabase.json", 'r')
    return json.load(lm_database_json)["value0"]


def get_lm_database_parallax_angle(lm_database):
    parallax_angles = []
    for lm in lm_database:
        # print(lm)
        is_initialized = bool(lm["value"]["is_initialized"]["atomic_data"])
        is_added = bool(lm["value"]["is_added"]["atomic_data"])
        if is_initialized:
            parallax_angles.append(float(lm["value"]["triangulate_parallax_angle"]["atomic_data"]))
    return parallax_angles


def get_lm_database_base_line_length(lm_database):
    base_line_lens = []
    for lm in lm_database:
        # print(lm)
        is_initialized = bool(lm["value"]["is_initialized"]["atomic_data"])
        is_added = bool(lm["value"]["is_added"]["atomic_data"])
        if is_initialized:
            base_line_lens.append(float(lm["value"]["triangulate_baseline_length"]["atomic_data"]))
    return base_line_lens


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


def extract_metrics(frame_internal_json):
    count = 0
    existing_ln_number = []  # Frame中に含まれるLMの数
    inuse_lm_number = []  # 最適化に利用さているFeatureの数

    keyframe_inuse_lm_number = []  # KeyFrame中で利用されているLMの数
    keyframe_takeover_lm_number = []  # 一つ前のKeyFrameから今回のKeyFrameに引き継がれたLMの数
    keyframe_triangulated_lm_number = []  # 今回のKeyFrameでTriangulateできたLMの数

    feature_pass_rate_tracking = []  # FrontendでのTracking処理での通過率
    feature_pass_rate_verification = []  # Verificationでの通過率
    feature_pass_rate_vision_frontend = []  # Vision frontendでの通過率

    # 0: triangulateしたLMは0, 1: triangulateしたLMとtakeoverしたLMは同じだけいる
    # 1以上のとき: triangulateしたLMのほうが多い。
    # １以上のとき十分に現在位置を決定できる要素となる３Frame以上で観測されたLMの比率が小さくなる。なので想定としてはこのときにiSAM2最適化でFramePoseジャンプが発生しているのではないか？
    feature_takeover_triangulate_rate = []

    # trajectory
    traj_position = np.full((len(frame_internal_json), 3), 0.0)
    traj_orientation = np.full((len(frame_internal_json), 4), 0.0)
    traj_timestamp = np.full((len(frame_internal_json), 1), 0.0)

    for fi in tqdm(frame_internal_json):
        lm_number = get_inuse_landmark_number(fi)
        inuse_lm_number.append(lm_number)
        existing_ln_number.append(get_landmark_number_by_age(fi, 1))
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
                    float(fi["features_after_verification_number"]) / float(fi["features_after_detection_number"]))
                feature_pass_rate_vision_frontend.append(
                    float(fi["features_after_verification_number"]) / float(fi["features_pre_frame_number"]))
            else:
                feature_pass_rate_verification.append(0)
                feature_pass_rate_vision_frontend.append(0)
            keyframe_inuse_lm_number.append(lm_number)
            keyframe_takeover_lm_number.append(int(fi["take_over_landmark_number"]))
            keyframe_triangulated_lm_number.append(int(fi["triangulated_landmark_number"]))

            if (int(fi["take_over_landmark_number"]) == 0):
                feature_takeover_triangulate_rate.append(1)
            else:
                feature_takeover_triangulate_rate.append(
                    float(fi["triangulated_landmark_number"]) / float(fi["take_over_landmark_number"]))

        else:
            feature_pass_rate_verification.append(feature_pass_rate_verification[-1])
            feature_pass_rate_vision_frontend.append(feature_pass_rate_vision_frontend[-1])
        count = count + 1

    output_dict = {}
    output_dict["existing_ln_number"] = existing_ln_number
    output_dict["inuse_lm_number"] = inuse_lm_number

    output_dict["keyframe_inuse_lm_number"] = keyframe_inuse_lm_number
    output_dict["keyframe_takeover_lm_number"] = keyframe_takeover_lm_number
    output_dict["keyframe_triangulated_lm_number"] = keyframe_triangulated_lm_number

    output_dict["feature_pass_rate_tracking"] = feature_pass_rate_tracking
    output_dict["feature_pass_rate_verification"] = feature_pass_rate_verification
    output_dict["feature_pass_rate_vision_frontend"] = feature_pass_rate_vision_frontend

    output_dict["feature_takeover_triangulate_rate"] = feature_takeover_triangulate_rate

    # trajectory
    output_dict["traj_position"] = traj_position
    output_dict["traj_orientation"] = traj_orientation
    output_dict["traj_timestamp"] = traj_timestamp

    return output_dict


def launch_vslam(bin_file, euroc_config, output_dir):
    cmd = "{} -e {} -r {} -f".format(bin_file, euroc_config, output_dir)
    print("Execute: {}".format(cmd))
    res = subprocess.call(cmd, shell=True)


if __name__ == "__main__":
    # mlflowのセットアップ
    # path_to_tracking_directory = "../results/tracking/mlruns"
    mlflow.set_tracking_uri("http://localhost:5000")
    # mlflow.set_tracking_uri(path_to_tracking_directory)
    mlflow.set_experiment('vslam_experiment')

    # path_to_euroc_config = "/home/ery/Devel/vi-slam/params/EurocKimeraDataProvider.json"
    # path_to_euroc_config = "/home/ery/Devel/vi-slam/params/euroc_V1_01.json"
    # path_to_euroc_config = "/home/ery/Devel/vi-slam/params/euroc_V1_02.json"
    # path_to_bin = "/home/ery/Devel/vi-slam/build/example/vslam/example_cd_arg"

    path_to_euroc_config = "/home/ery/Devel/vi-slam/params/euroc_V1_02.json"
    path_to_bin = "/home/ery/Devel/vi-slam/build/example/vslam/example_cd_arg"

    # tmp directoryを作成して、その中に結果を出力するようにする
    with tempfile.TemporaryDirectory() as path_to_tmp_directory:
        # vslamの実行
        launch_vslam(path_to_bin, path_to_euroc_config, path_to_tmp_directory)

        # Evaluate
        frame_internals = get_all_frame_internals_json(path_to_tmp_directory)
        metrics = extract_metrics(frame_internals)

        # Evaluate trajectory
        # import evo.common_ape_rpe as common
        # from evo.core import sync
        # from evo.tools import file_interface, log
        # from evo.core import metrics
        # from evo.core import trajectory
        # from evo.core.metrics import PoseRelation
        # from evo.main_ape import ape
        # traj_estimated = PoseTrajectory3D(result["traj_position"], result["traj_orientation"], result["traj_timestamp"])
        # traj_reference_raw = file_interface.read_euroc_csv_trajectory(get_path_to_reference_trajectory(path_to_tmp_directory))
        # traj_ref, traj_est = sync.associate_trajectories(traj_reference_raw, traj_estimated, 0.01, 0.0,first_name="reference", snd_name="estimate")
        # pose_relation = PoseRelation.translation_part
        # result = ape(
        #     traj_ref=traj_ref,
        #     traj_est=traj_est,
        #     pose_relation=pose_relation,
        #     align=True,
        #     correct_scale=True,
        #     align_origin=True,
        #     ref_name="reference",
        #     est_name="estimate",
        # )

        # Log input params
        path_to_dataset_root = get_path_to_dataset(path_to_tmp_directory)
        log_param("Dataset", os.path.basename(path_to_dataset_root))

        # Visualize
        plt.figure(1)
        plt.plot(metrics["inuse_lm_number"], "-")
        plt.plot(metrics["existing_ln_number"], "-")
        plt.legend(["inuse", "existing"])
        plt.title("LM number")
        plt.xlabel("Frame no.")
        plt.ylabel("LM number")
        plt.savefig(path_to_tmp_directory + "/feature_number.png", dpi=300)
        log_artifact(path_to_tmp_directory + "/feature_number.png")

        fig_pass_rate = plt.figure(4)
        plt.plot(metrics["feature_pass_rate_verification"])
        plt.plot(metrics["feature_pass_rate_tracking"])
        plt.plot(metrics["feature_pass_rate_vision_frontend"])
        plt.legend(["Verification", "Tracking", "VisionFrontend"])
        plt.ylabel("Feature pass rate")
        plt.xlabel("Frame number")
        plt.savefig(path_to_tmp_directory + "/feature_pass_rate.png", dpi=300)
        log_artifact(path_to_tmp_directory + "/feature_pass_rate.png")

        fig_keyframe_lm_number = plt.figure(5)
        plt.plot(metrics["keyframe_inuse_lm_number"])
        plt.plot(metrics["keyframe_takeover_lm_number"])
        plt.plot(metrics["keyframe_triangulated_lm_number"])
        plt.legend(["Inuse", "Takeover", "Triangulated"])
        plt.ylabel("Landmark number")
        plt.xlabel("KeyFrame")
        plt.title("Key Frame Landmark number")
        plt.savefig(path_to_tmp_directory + "/keyframe_lm_number.png", dpi=300)
        log_artifact(path_to_tmp_directory + "/keyframe_lm_number.png")

        fig_triangulate_takeover = plt.figure(6)
        plt.plot(metrics["feature_takeover_triangulate_rate"])
        plt.legend(["Rate"])
        plt.ylabel("Rate(tri/takeover)")
        plt.xlabel("KeyFrame")
        plt.title("Triangulate and Takeover rate")
        plt.savefig(path_to_tmp_directory + "/triangulate_takeover_rate.png", dpi=300)
        log_artifact(path_to_tmp_directory + "/triangulate_takeover_rate.png")

        # LM stat
        lm_database = get_lm_database(path_to_tmp_directory)
        parallax = get_lm_database_parallax_angle(lm_database)
        baselength = get_lm_database_base_line_length(lm_database)
        fig_hist_triangulate_parallax = plt.figure(7)
        plt.hist(parallax, bins=100)
        plt.axvline(max(parallax))
        plt.axvline(min(parallax))
        plt.xlabel("Triangulate parallax [rad]")
        plt.ylabel("Frequency")
        plt.savefig(path_to_tmp_directory + "/hist_triangulate_parallax.png", dpi=300)
        log_artifact(path_to_tmp_directory + "/hist_triangulate_parallax.png")
        fig_hist_baseline = plt.figure(8)
        plt.hist(baselength, bins=50)
        plt.xlabel("Triangulate baseline length")
        plt.ylabel("Frequency")
        plt.savefig(path_to_tmp_directory + "/hist_triangulate_baseline_length.png", dpi=300)
        log_artifact(path_to_tmp_directory + "/hist_triangulate_baseline_length.png")

        log_artifact(path_to_tmp_directory + "/EurocKimeraDataProvider.json")
        log_artifact(path_to_tmp_directory + "/FeatureDetectorANMS.json")
        log_artifact(path_to_tmp_directory + "/FeatureTrackerLSSDLucasKanade.json")
        log_artifact(path_to_tmp_directory + "/FeatureVerification5PointRANSAC.json")
        log_artifact(path_to_tmp_directory + "/iSAM2Backend.json")
        log_artifact(path_to_tmp_directory + "/trajectory_body_frame.tum")
        log_artifact(path_to_tmp_directory + "/Mapdatabase.json")
        mlflow.end_run()

        plt.show()
