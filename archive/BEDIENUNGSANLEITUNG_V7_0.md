# Bedienungsanleitung Robotrol v7.0

## 1. Zweck und Scope
Robotrol v7.0 ist eine Steuerzentrale fuer 6-DoF-Roboterarme mit FluidNC/GRBL. Das System kombiniert:
- serielle Maschinensteuerung
- Bewegungsqueue und Programmausfuehrung
- Gamepad-Jogging
- TCP/FK/IK-Werkzeuge
- Kamera-, Schachbrett- und Pick&Place-Funktionen
- optionalen FluidNC-OTA/Config-Workflow

## 2. Start und Grundbetrieb
1. Abhaengigkeiten installieren:
   `pip install numpy opencv-python pillow pyserial pygame pyyaml`
2. Anwendung starten:
   `python Robotrol_FluidNC_v7_0.py`
3. Port waehlen, verbinden, Status beobachten.
4. Profil laden (Moveo, EB15_red, EB300), dann Limits/Homing pruefen.

## 3. Sicherheitsreihenfolge vor Bewegung
1. Not-Aus-Funktion pruefen (`emergency_stop`).
2. Endstop-/Manuallimits laden und plausibilisieren.
3. Homing nur bei korrekt referenzierter Maschine.
4. Erst mit langsamer Geschwindigkeit testen.
5. Pick&Place zuerst im Simulations-/Shadow-Betrieb verifizieren.

## 4. Hauptfunktionen in der UI
- Verbindung/Status: Connect/Disconnect, Live-Status, Parser fuer Maschinenantworten.
- Queue/Programm: G-Code sammeln, editieren, starten, pausieren, fortsetzen, abbrechen.
- Direktsteuerung: Achsfahrten, Nullsetzen, Home, TCP-bezogene Befehle.
- Gamepad: Mapping, Geschwindigkeitsprofile, Trigger fuer Bewegungen.
- TCP Pose: FK-basierte Poseanzeige (XYZ + Roll/Pitch/Yaw).
- TCP Kinematik: IK-Loesung und Sequenzvorschau fuer TCP-Zielbewegungen.
- Vision/Kalibrierung: Kamerabild, Board-Pose, Base-Cam-/Marker-Transformationen.
- Pick&Place: Erkennung, Kalibrierung, Zyklen, Self-Learning (TNT).
- OTA/FluidNC: $$-Werte, Config-Upload, Firmware-Upload, Neustart.

## 5. Konfigurationsdateien
- `configs/robot.json`: Robotik-/Geometrieparameter.
- `configs/camera.json`: Kamera- und Transformationsparameter.
- `configs/calibration.json`: Kalibrier-Messdaten.
- `configs/transforms.json`: Koordinatentransformationen.
- `configs/project_flags.json`: Sprach-/Projektflags.
- `Moveo.json`, `EB15_red.json`, `EB300.json`: Profile.

## 6. Typische Workflows
### 6.1 Verbindung und Testfahrt
1. COM-Port aktualisieren (`refresh_ports`) und verbinden (`connect`).
2. Profil anwenden (`apply_profile`).
3. Endstop-/Manuallimits laden.
4. Kleine Jog-Fahrten und Statuspolling pruefen.

### 6.2 Queue-basierter Ablauf
1. Befehle in Queue aufnehmen (`enqueue`, `axis_to_queue`, `add_current_pose_to_queue`).
2. Queue kontrollieren (insert/edit/delete).
3. Ausfuehrung starten (`start_run`), ggf. pausieren/fortsetzen.
4. Bei Fehlern `stop_abort` oder `emergency_stop`.

### 6.3 Pick&Place + Vision
1. Kamera anbinden und Bild pruefen.
2. Board-/Marker-Kalibrierung ausfuehren.
3. Detektion und Bounds-Checks testen.
4. Einzelzyklus starten, dann Mehrfachzyklus.
5. Self-Learning erst in `shadow`, dann ggf. `active`.

## 7. Fehlerbehebung (Kurz)
- Keine Verbindung: Port/Backend/Timeout pruefen.
- Falsche Pose: DH-Parameter und Post-Transform kontrollieren.
- Instabile Detektion: Kamera-Parameter/Belichtung/Filter nachstellen.
- Ueberschrittene Limits: Manuelle/Endstop-Limits und Profile vergleichen.

## 8. Vollstaendiger Funktionskatalog (v7.0)
Die folgende Liste wurde aus dem Quellcode extrahiert (Datei: `tools/function_index_v7_0.json`).


### board_pose_v1.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `BoardPose`: `__init__`, `start_camera`, `stop_camera`, `_update_ui`, `_camera_matrix`, `_estimate_homography`, `_rotation_to_euler`, `_draw_virtual_board`, `_update_indicators`, `get_field_position`, `get_frame`, `set_tnt_cfg`, `pause_feed`, `resume_feed`, `_resume_safe`, `stop`
- `BoardDetectorCalibration`: `__init__`, `_build_ui`, `_log`, `capture_point`, `compute_transform`, `save_transform`

### camera_capturev_v1_1.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `CameraCapture`: `__init__`, `_list_cameras`, `_on_camera_change`, `start`, `stop`, `_update_loop`, `_tk_update`, `get_latest_frame`, `get_frame`

### chess/__init__.py
Top-Level-Funktionen: keine
Klassen: keine

### chess/board_detector.py
Top-Level-Funktionen:
- `_as_gray`
- `_estimate_outer_corners`
- `detect_board`
Klassen: keine

### chess/config.py
Top-Level-Funktionen:
- `load_config`
Klassen: keine

### chess/piece_classifier.py
Top-Level-Funktionen:
- `_load_template`
- `classify_piece`
Klassen: keine

### chess/piece_detector.py
Top-Level-Funktionen:
- `_to_gray`
- `detect_pieces`
Klassen: keine

### chess/state.py
Top-Level-Funktionen:
- `board_to_fen`
- `build_state`
Klassen: keine

### chess/types.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `BoardDetection`: keine Methoden
- `PieceDetection`: keine Methoden
- `ChessState`: keine Methoden

### chess/vision_pipeline.py
Top-Level-Funktionen:
- `detect_chess_state`
- `load_config`
Klassen: keine

### chess_vision_ui.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `ChessVisionTab`: `__init__`, `_build_ui`, `_reload_config`, `_get_image`, `_detect_state`, `_render_board`, `_set_status`

### config_profiles.py
Top-Level-Funktionen:
- `_read_json`
- `_write_json`
- `_candidate_base_dirs`
- `_resolve_write_base_dir`
- `_find_existing_profile_path`
- `_find_base_dir_with_legacy_files`
- `profile_path`
- `build_profile_from_legacy`
- `load_profile`
- `save_profile`
Klassen: keine

### control/__init__.py
Top-Level-Funktionen: keine
Klassen: keine

### control/app.py
Top-Level-Funktionen:
- `build_pipeline`
- `run`
Klassen: keine

### control/config.py
Top-Level-Funktionen:
- `load_config`
- `load_all_configs`
Klassen: keine

### control/errors.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `PickPlaceError`: keine Methoden
- `PerceptionError`: keine Methoden
- `PlanningError`: keine Methoden
- `ControlError`: keine Methoden

### control/executor.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `TrajectoryExecutor`: `__init__`, `execute_pick`, `execute_place`

### control/fsm.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `State`: keine Methoden
- `PickPlaceStateMachine`: `__init__`, `run_cycle`, `_call`, `_safe_stop`

### control/gripper.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `GripperInterface`: `open`, `close`, `is_object_grasped`
- `MockGripper`: `__init__`, `open`, `close`, `is_object_grasped`

### control/pipeline.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `PipelineContext`: `detect`, `plan_grasp`, `execute_pick`, `plan_place`, `execute_place`, `safe_stop`
- `PickPlacePipeline`: `__init__`, `run_cycle`

### control/robot.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `RobotInterface`: `move_cartesian`, `move_joint`, `stop`
- `MockRobot`: `__init__`, `move_cartesian`, `move_joint`, `stop`

### control/transforms.py
Top-Level-Funktionen:
- `identity`
- `matmul`
- `make_transform`
- `extract_translation`
- `extract_rotation`
- `transform_point`
- `invert_transform`
- `normalize_vector`
- `dot`
- `cross`
- `rotation_matrix_from_quaternion`
- `quaternion_from_rotation_matrix`
Klassen: keine

### fluidnc_updater_v2.py
Top-Level-Funktionen:
- `parse_dollars`
Klassen und Methoden:
- `FluidNCClient`: `__init__`, `set_host`, `command`, `is_alive`, `get_dollars`, `list_localfs`, `get_active_config`, `set_active_config`, `reboot`, `upload_firmware`, `upload_config_yaml`
- `FluidNCUpdaterFrame`: `__init__`, `_auto_detect_ip`, `_start_auto_detect`, `_build_ui`, `_set_connected`, `_with_thread`, `_on_connect`, `_ui_info`, `_build_tab_status`, `_build_tab_configs`, `_build_tab_ota`, `_set_connected`, `_with_thread`, `_on_connect`, `_ui_info`, `_refresh_dollars`, `_fill_flags`, `_fill_axes`, `_export_dollars`, `_refresh_files`, `_get_active`, `_set_active_from_sel`, `_reboot`, `_pick_fw`, `_upload_fw`, `_pick_yaml`, `_upload_yaml`

### gamepad_block_v3.py
Top-Level-Funktionen:
- `DEFAULT_AXIS`
- `map_speed_linear`
- `_rpy_to_R`
- `attach_gamepad_tab`
Klassen und Methoden:
- `GamepadConfigUI`: `__init__`, `_build_ui`, `_build_table`, `_build_button_mapping`, `_load_config`, `save_config`, `reload_into_ui`, `reset_defaults`, `get_params`, `get_speed_factor`, `get_button_actions`

### ik_rotosim.py
Top-Level-Funktionen:
- `_clamp`
- `_wrap_pm180`
- `_rot_z`
- `_fmt`
Klassen und Methoden:
- `IKLimits`: keine Methoden
- `RotoSimIK`: `__init__`, `_fk_points_and_dir`, `_tcp_pose5`, `_limit_q`, `_numeric_jacobian`, `_pick_seed`, `solve_from_pose_locked`

### learning/__init__.py
Top-Level-Funktionen: keine
Klassen: keine

### learning/tnt_self_learning.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `TntSelfLearningManager`: `__init__`, `_merge_defaults`, `_normalize_policy`, `_context_key`, `_get_context_bucket`, `_load_json`, `_save_json`, `save`, `save_settings`, `set_enabled`, `set_mode`, `sync_baseline_from_configs`, `reset_policy_from_configs`, `_clamp`, `_propose_candidate`, `_apply_params_to_configs`, `before_cycle`, `_reward`, `_maybe_rollback`, `after_cycle`, `_append_log`, `get_status`

### perception/__init__.py
Top-Level-Funktionen: keine
Klassen: keine

### perception/camera.py
Top-Level-Funktionen:
- `build_camera`
Klassen und Methoden:
- `Frame`: keine Methoden
- `CameraInterface`: `get_frame`
- `MockCamera`: `get_frame`
- `ImageFileCamera`: `__init__`, `_load_cv2`, `get_frame`
- `OpenCVCamera`: `__init__`, `get_frame`

### perception/marker_detector.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `MarkerObservation`: keine Methoden
- `ArucoMarkerDetector`: `__init__`, `_load_cv2`, `_dictionary`, `detect`

### perception/pose_estimator.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `PoseEstimator`: `__init__`, `set_mock_pose`, `detect_object_pose`, `_mock_object_pose`, `_detect_from_camera`, `_detect_tnt_pose`

### perception/quality.py
Top-Level-Funktionen:
- `reprojection_error`
- `confidence_from_reprojection`
Klassen: keine

### perception/tnt_detector.py
Top-Level-Funktionen:
- `_save_debug_image`
- `_get_color_order`
- `_to_bgr`
- `_build_mask`
- `detect_tnt_contour`
- `detect_tnt_pose`
Klassen: keine

### perception/types.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `ObjectPose`: keine Methoden

### pickplace_ui.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `_CameraCaptureAdapter`: `__init__`, `get_frame`
- `PickPlaceTab`: `__init__`, `_build_ui`, `_build_run_tab`, `_build_sim_tab`, `_build_cfg_tab`, `_build_calib_tab`, `_build_base_cam_tab`, `_build_camera_tab`, `_get_tcp_position_mm`, `_capture_board_point`, `_clear_board_points`, `_capture_square_point`, `_clear_square_points`, `_compute_base_T_board_from_points`, `_fit_base_T_board_from_squares`, `_parse_board_targets`, `_board_square_center`, `_board_to_base`, `_test_board_moves`, `_reset_camera_calib`, `_capture_camera_sample`, `_compute_camera_intrinsics`, `_save_camera_intrinsics`, `_build_marker_tab`, `_build_detect_tab`, `_build_help_tab`, `_make_matrix_grid`, `_set_matrix_vars`, `_read_matrix_vars`, `_identity_matrix`, `_ensure_configs`, `_write_config`, `_load_calibration_into_ui`, `_save_board_config`, `_compute_base_T_cam`, `_rot_error_deg`, `_camera_model_from_cfg`, `_detect_board_points`, `_validate_base_T_cam_once`, `_simulate_base_cam_noise`, `_save_base_T_cam`, `_compute_marker_T_obj`, `_save_marker_T_obj`, `_detect_once`, `_load_tnt_into_ui`, `_build_tnt_config`, `_apply_tnt_filters`, `_save_tnt_filters`, `_rpy_from_R`, `_wrap_angle_deg`, `_rpy_to_R`, `_current_rpy`, `_get_speed_slider_feed`, `_wait_for_idle`, `_pickup_test`, `_resolve_grasp_normal`, `_check_board_bounds`, `_check_workspace_bounds`, `_check_calibration_sanity`, `_ui`, `_log`, `_set_status`, `_refresh_stack_index`, `_reset_stack_index`, `_init_pipeline`, `_ensure_pipeline`, `_attach_camera_capture`, `_update_detection_overlay`, `_ensure_learner`, `_refresh_learning_from_manager`, `_on_learning_toggle`, `_on_learning_mode_change`, `_show_learning_status`, `_reset_learning_policy`, `_learning_before_cycle`, `_make_cycle_context`, `_learning_after_cycle`, `_start_worker`, `_run_one_cycle`, `_run_n_cycles`, `_run_simulation`, `_stop`, `_render_config`, `_get_calibration_image`

### planning/__init__.py
Top-Level-Funktionen: keine
Klassen: keine

### planning/grasp_planner.py
Top-Level-Funktionen:
- `_axis_from_config`
- `_build_orientation_from_normal`
Klassen und Methoden:
- `GraspPlanner`: `__init__`, `plan`

### planning/place_planner.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `PlacePlanner`: `__init__`, `plan`

### planning/types.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `GraspPlan`: keine Methoden
- `PlacePlan`: keine Methoden

### robosim_visualizer_v90.py
Top-Level-Funktionen:
- `_fmt`
- `dir_to_yaw_tilt`
- `rot_x`
- `rot_y`
- `rot_z`
- `fk_points`
- `fk_points_dh_rows`
- `tcp_pose_from_points`
- `tcp_pose_5`
Klassen und Methoden:
- `Geometry`: keine Methoden
- `Pose`: keine Methoden
- `UDPListener`: `__init__`, `run`
- `VisualizerApp`: `__init__`, `_apply_robot_profile`, `_current_fk`, `_current_tcp_pose`, `_build_ui`, `on_udp`, `loop`, `fix_position`, `_workspace_probe`, `move_tool_axis`, `retreat_to_fix`, `add_gripper`, `add_delay`, `clear_gcode`, `append_gcode`, `export_gcode`, `log`, `redraw`, `_draw_2d`, `_draw_path_2d`, `_draw_path_3d`, `_draw_fixed_frame_3d`, `_draw_fixed_frame_2d`

### Robotrol_FluidNC_v7_0.py
Top-Level-Funktionen:
- `_no_popup`
- `_load_project_flags`
- `_load_camera_config`
- `_profile_has_endstops`
- `map_speed`
- `toggle_theme`
- `_on_profile_select`
- `refresh_ports`
- `connect`
- `disconnect`
- `_log_profile_msg`
- `apply_profile`
- `_gp_label_click`
- `on_close`
Klassen und Methoden:
- `SerialClient`: `__init__`, `set_backend`, `is_fluidnc`, `is_grbl`, `is_custom`, `supports_endstops`, `supports_ota`, `supports_axis_homing`, `supports_global_homing`, `supports_softlimits`, `status_query_line`, `parse_status_line`, `connect`, `disconnect`, `list_ports`, `_mirror_udp`, `send_line`, `send_ctrl_x`, `_rx_loop`
- `ExecuteApp`: `__init__`, `_build_ui`, `_toggle_right_vision`, `_tick_right_vision`, `get_current_tcp_mm`, `set_gamepad_config_ui`, `get_profile_section`, `set_profile_section`, `save_profile`, `apply_profile`, `get_gamepad_config`, `save_gamepad_config`, `save_cam_to_base`, `save_dh_model_to_profile`, `_apply_profile_runtime_flags`, `_send_vis_robot_profile`, `_default_endstop_limits`, `_default_manual_limits`, `_normalize_manual_limits`, `_normalize_endstop_limits`, `_load_endstop_limits`, `_load_manual_limits`, `_write_endstop_limits`, `_write_manual_limits`, `_set_endstop_vars_from_limits`, `_get_endstop_limits_from_vars`, `_apply_axis_limits_to_ui`, `_apply_endstop_limits_from_vars`, `_load_endstop_limits_into_vars`, `_save_endstop_limits_from_vars`, `_manual_axis_limits`, `_effective_axis_limits`, `log`, `send_now`, `insert_delay`, `_parse_cli`, `_init_cli_history`, `_add_to_history`, `_on_cli_return`, `_on_cli_up`, `_on_cli_down`, `cli_add_to_program`, `cli_send_now`, `_handle_tcp_gcode`, `enqueue`, `_append_vis_path_fixed`, `_append_vis_path_kin`, `_send_vis_path`, `_send_vis_fixed_frame`, `clear_program`, `load_program`, `save_program`, `_init_queue_edit`, `_queue_delete`, `_queue_insert`, `_queue_edit_item`, `_queue_on_click`, `_queue_on_drag`, `_queue_on_drop`, `clamp_with_limits`, `axis_to_queue`, `axis_to_cli`, `axis_direct_send`, `do_zero`, `manual_zero_current_pose`, `goto_home`, `enqueue_home`, `enqueue_axis`, `set_all_to_queue`, `add_current_pose_to_queue`, `set_all_now`, `start_run`, `pause_run`, `resume_run`, `stop_abort`, `emergency_stop`, `worker`, `_update_status_block`, `on_serial_line`, `request_and_parse_settings`, `update_position_display`, `move_fixed_tcp_gamepad`, `set_gamepad_mode_cycle`, `fix_tcp_from_gamepad`, `rotate_fixed_tcp_roll`, `on_gamepad_trigger`, `_tick_status_poll`

### Robotrol_FluidNC_v7_0_launcher.py
Top-Level-Funktionen: keine
Klassen: keine

### simulation/__init__.py
Top-Level-Funktionen: keine
Klassen: keine

### simulation/mock_world.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `MockWorld`: `__init__`, `pose_for_cycle`

### simulation/simulation_loop.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `SimulationRunner`: `__init__`, `run`

### tcp_pose_module_v3.py
Top-Level-Funktionen:
- `_load_dh_model`
- `_ordered_axes_from_model`
- `_build_dh_rows_mm_deg`
- `_geom_from_model`
- `_fallback_dh_rows_from_geom`
- `get_dh_model`
- `get_dh_rows_mm_deg`
- `get_dh_axes`
- `get_post_transform`
- `_clean_joint_map`
- `get_joint_angle_post_maps`
- `apply_joint_angle_post_transform`
- `derive_visualizer_geometry_mm`
- `reload_dh_model`
- `set_dh_model_from_dict`
- `_pose_diff`
- `fk6_forward_mm`
Klassen und Methoden:
- `TcpPosePanel`: `__init__`, `start`, `stop`, `on_pose_changed`, `get_current_tcp_mm`, `set_geom_dh`, `_attach_listener`, `destroy`, `_on_serial_line`, `_tick_tcp_update`

### tcp_world_kinematics_frame.py
Top-Level-Funktionen: keine
Klassen und Methoden:
- `IK6`: `__init__`, `_wrap180`, `solve_xyz`
- `TcpKinematicsFrame`: `__init__`, `_build_ui`, `_refresh_profile_post_transform`, `_dh_axes`, `_get_dh_defaults`, `refresh_dh_table`, `set_mask_from_current_pose`, `_norm`, `_get_tool_axis_world`, `_read_dh_table`, `_dh_transform`, `_mat_mul`, `_transpose`, `_mat_mul_generic`, `_mat_add_diag`, `_rpy_to_R`, `_mat3_mul`, `_mat3_transpose`, `_so3_log`, `_task_from_joints`, `_pose_error`, `_invert_matrix`, `_mat_vec_mul`, `_get_current_joint_list`, `_save_dh_to_json`, `_compute_jacobian`, `_damped_pinv`, `_wrap180_deg`, `move_tcp_pose`, `preview_tcp_gcode`, `_limits`, `_limits_for_preview`, `_check_limits`, `_emit`, `_run`, `_collect_preview_lines`, `_send_preview_to_cli`, `_execute_direct`, `_send_to_queue`, `_send_preview_to_queue`
- `TcpWorldKinematicsTabs`: `__init__`, `preview`, `preview_sequence`, `gamepad_preview_sequence`, `seq_preview`, `execute_sequence`, `queue_sequence`, `set_tcp_as_reference`, `use_current_tcp_as_ref`, `solve_and_execute_sequence`, `pose_to_mask_and_generate`, `solve_and_run_tcp_sequence`, `move_tcp_pose`, `refresh_dh_table`, `preview_tcp_gcode`

### tools/check_english_text.py
Top-Level-Funktionen:
- `_iter_files`
- `_load_flags`
- `main`
Klassen: keine

### tools/plane_g2g3_check.py
Top-Level-Funktionen:
- `_parse_words`
- `_validate_file`
- `main`
Klassen: keine

### tools/smoke_checks.py
Top-Level-Funktionen:
- `_run_language_check`
- `_run_mock_simulation`
- `_run_learner_logic_check`
- `main`
Klassen: keine
