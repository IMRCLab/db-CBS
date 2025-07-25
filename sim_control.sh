
export PYTHONPATH=~/coltrans-planning/deps/crazyflie-firmware:$PYTHONPATH
export PYTHONPATH=dynoplan/dynobench:$PYTHONPATH

# ./dynoplan/main_optimization --init_file ../stats_db/window_3robots/000/init_guess_cable_hover.yaml --env_file ../dynoplan/dynobench/envs/quad3d_payload/empty_0.yaml --models_base_path ../dynoplan/dynobench/models/ --solver_id 1 --results_file ../stats_db/window_3robots/000/output.trajopt

python3 ../dynoplan/dynobench/example/test_quad3dpayload_n.py  -cff -w --inp ../stats_db/window_3robots/000/output.trajopt.yaml --out ../stats_db/window_3robots/000/trajectory_opt.yaml --model_path ../dynoplan/dynobench/models/point_3.yaml 

python3 ../scripts/visualize_payload.py --env ../stats_db/window_3robots/000/env.yaml --robot point --result ../stats_db/window_3robots/000/trajectory_opt.yaml --output ../stats_db/window_3robots/000/trajectory_opt.html --ref ../stats_db/window_3robots/000/output.trajopt.yaml