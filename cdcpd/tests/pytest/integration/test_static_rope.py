import os
import sys
from pathlib import Path

# import rosbag
# import ros_numpy
import numpy as np

import pycdcpd

ROOT = Path(os.path.dirname(__file__))
TEST_DIR = ROOT / ".."
sys.path.append(TEST_DIR.as_posix())
from utils import cdcpd_bagfiles

CDCPD_DIR = TEST_DIR / ".." / ".." / ".."
DEMO_BAG = CDCPD_DIR / "demos" / "rosbags" / "demo_1_static_rope.bag"

MAX_ROPE_LENGTH = 0.46
NUM_POINTS = 15
ROPE_START_POSITION = np.array((-MAX_ROPE_LENGTH / 2, 0, 1.0), dtype=np.float32).reshape(3, 1)
ROPE_END_POSITION = np.array((MAX_ROPE_LENGTH / 2, 0, 1.0), dtype=np.float32).reshape(3, 1)

def test_resim_point_equivalency():
    """Run CDCPD on rosbag input so as to simulate a full run of CDCPD and test that the
    functionality has not changed significantly since recording of the bag file
    """
    # Read in the ros bagfile that we'll be resimulating and checking CDCPD performance against.
    bagdata = cdcpd_bagfiles.BagData(DEMO_BAG.as_posix(), raw_clouds_topic="/cdcpd/original",
                                     gurobi_output_topic="/cdcpd/output")
    input_clouds = bagdata.raw_clouds.data
    configs_expected = bagdata.gurobi_output.data

    # Setup the initial tracking of the rope.
    print("Constructing tracking map")
    rope_configuration = pycdcpd.RopeConfiguration(NUM_POINTS, MAX_ROPE_LENGTH, ROPE_START_POSITION,
                                                   ROPE_END_POSITION)
    rope_configuration.initializeTracking()
    tracking_map = pycdcpd.TrackingMap()
    tracking_map.add_def_obj_configuration(rope_configuration)
    print("Done with constructing tracking map")

    objective_value_threshold = 1.0
    use_recovery = False
    alpha = 0.5
    beta = 1.0
    lambda_arg = 1.0
    k = 100.0
    zeta = 10.0
    obstacle_cost_weight = 0.001
    # min_distance_threshold = 0.01
    fixed_points_weight = 10.0

    cdcpd_instance = pycdcpd.PyCDCPD(tracking_map, objective_value_threshold, use_recovery, alpha,
        beta, lambda_arg, k, zeta, obstacle_cost_weight, fixed_points_weight)
    print("Done with constructing CDCPD runner", flush=True)

    llbb = cdcpd_instance.get_last_lower_bounding_box()
    lubb = cdcpd_instance.get_last_upper_bounding_box()
    segmenter = pycdcpd.SegmenterHSV(llbb, lubb)

    use_initial_tracking = True
    edges = tracking_map.form_edges_matrix(use_initial_tracking)
    num_edges = edges.shape[1]
    num_clouds = len(input_clouds)
    for i, cloud in enumerate(input_clouds):
        print(f"Step {i}/{num_clouds - 1}", end='\r', flush=True)

        llbb = cdcpd_instance.get_last_lower_bounding_box()
        lubb = cdcpd_instance.get_last_upper_bounding_box()
        segmenter.set_last_lower_bounding_box(llbb)
        segmenter.set_last_upper_bounding_box(lubb)
        segmenter.set_input_cloud_from_matrices(cloud['xyz'], cloud['rgb'])
        segmenter.segment()
        cloud_segmented = segmenter.get_segmented_cloud_matrix()
        # print("Input clouds shape:", cloud.shape)
        # print("Input clouds dtype:", cloud.dtype)
        # print("Input clouds 'rgb' sample:", cloud['rgb'])

        # Segment the input cloud since we get the full raw XYZRGB point cloud.


        # Downsample the input cloud.
        cloud_downsampled = pycdcpd.downsampleMatrixCloud(cloud_segmented)

        # And clip the cloud based on the bounding box from the last iteration.
        llbb = cdcpd_instance.get_last_lower_bounding_box()
        lubb = cdcpd_instance.get_last_upper_bounding_box()
        cloud_clipped = pycdcpd.boxFilterMatrixCloud(cloud_downsampled, llbb, lubb)

        # Create the inputs for this iteration of the CDCPD C++ run
        inputs = pycdcpd.CDCPDIterationInputs()
        inputs.pred_choice = 0
        inputs.tracking_map = tracking_map
        inputs.X = cloud_clipped
        inputs.Y_emit_prior = np.ones((tracking_map.get_total_num_points(), 1), dtype=np.float64)

        out = cdcpd_instance.run(inputs)

        # Update the tracking map with the new vertices.
        gurobi_out = out.get_gurobi_output()
        # gurobi_out = out.get_cpd_output()
        tracking_map.update_def_obj_vertices_from_mat(gurobi_out)

        # Check this iterations cloud against the true cloud we got.
        assert (np.allclose(configs_expected[i]['xyz'], gurobi_out)), f"Failed at frame #{i}!"
