import csv

from agx_arm_mit_controller.fit_gravity_calibration import _fit_joint_calibration, _fit_scale_and_bias, _load_csv_rows


def test_fit_scale_and_bias_matches_linear_relation():
    scale, bias = _fit_scale_and_bias([1.0, 2.0, 3.0], [3.0, 5.0, 7.0])

    assert abs(scale - 2.0) < 1e-9
    assert abs(bias - 1.0) < 1e-9


def test_fit_joint_calibration_rejects_static_logs():
    scale, bias, fitted, reason = _fit_joint_calibration(
        [0.3, 0.3005, 0.301],
        [3.0, 3.01, 3.02],
        [4.0, 4.1, 4.2],
        min_joint_span=0.05,
        min_model_span=0.1,
        max_abs_scale=10.0,
        max_abs_bias=16.0,
    )

    assert (scale, bias, fitted) == (1.0, 0.0, False)
    assert "joint span" in reason


def test_fit_joint_calibration_rejects_unsafe_fit():
    scale, bias, fitted, reason = _fit_joint_calibration(
        [0.0, 0.2, 0.4],
        [0.0, 0.1, 0.2],
        [0.0, 5.0, 10.0],
        min_joint_span=0.05,
        min_model_span=0.05,
        max_abs_scale=10.0,
        max_abs_bias=16.0,
    )

    assert (scale, bias, fitted) == (1.0, 0.0, False)
    assert "scale" in reason


def test_load_csv_rows_reads_multiple_files(tmp_path):
    header = ["time", "q1", "tau_measured_1", "tau_g_urdf_1"]
    path_a = tmp_path / "a.csv"
    path_b = tmp_path / "b.csv"

    for path, rows in (
        (path_a, [["0.0", "0.1", "1.0", "0.9"]]),
        (path_b, [["1.0", "0.2", "1.1", "1.0"]]),
    ):
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            writer.writerows(rows)

    rows, resolved_paths = _load_csv_rows([str(path_a), str(path_b)])

    assert len(rows) == 2
    assert [row["time"] for row in rows] == ["0.0", "1.0"]
    assert resolved_paths == [path_a.resolve(), path_b.resolve()]