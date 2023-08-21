

import unittest



import subprocess



class TestStandAlone(unittest.TestCase):






    def test_opti1(self):
        build_cmd = [
            "make", "multirobot_optimization"]

        run_cmd_new = [
            "./multirobot_optimization",
            "--env",
            "../example/straight.yaml",
            "--init",
            "../example/guess_indiv_straight.yaml",
            "--out",
            "buu.yaml",
            "--s",
            "1",
            ">",
            "/tmp/db_log.txt"]



        visualize_cmd = [
            "python3",
            "../scripts/visualize.py",
            "../example/straight.yaml",
            "--result",
            "/tmp/check5.yaml",
            # "--video",
            # "straight_solution_optimized.mp4"
        ]



        for i in [build_cmd, run_cmd_new, visualize_cmd]:

            print("running cmd")
            print(' '.join(i))

            out = subprocess.run(i)
            assert out.returncode == 0


    def test_opti2(self):
        run_cmd_old = [
            "./multirobot_optimization",
            "--env",
            "../example/straight.yaml",
            "--init",
            "../example/guess_indiv_straight.yaml",
            "--out",
            "buu.yaml",
            "--s",
            "0",
            ">",
            "/tmp/db_log.txt"]

        build_cmd = [
            "make", "multirobot_optimization"]

        visualize_cmd = [
            "python3",
            "../scripts/visualize.py",
            "../example/straight.yaml",
            "--result",
            "/tmp/check5.yaml",
            # "--video",
            # "straight_solution_optimized.mp4"
        ]


        print("old optimization")
        for i in [build_cmd, run_cmd_old, visualize_cmd]:
            print("running cmd")
            print(' '.join(i))

            out = subprocess.run(i)
            assert out.returncode == 0

    def test_cbs(self):
        build_cmd = [ "make" , "db_cbs" ]
        cmd_db_cbs = ["./db_cbs", "-i", "../example/classic.yaml", "-o", "classic_debug.yaml", "--jnt", "classic_debug_joint.yaml", "--opt", "classic_debug_opt.yaml" ]
        cmd_vis = [ "python3", "../scripts/visualize.py", "../example/classic.yaml", "--result", "classic_debug_opt.yaml" ]
        for i in [build_cmd, cmd_db_cbs, cmd_vis]:
            print("running cmd")
            print(' '.join(i))

            out = subprocess.run(i)
            assert out.returncode == 0

    def test_check1(self):
        build_cmd = [ "make" , "main_check_multirobot" ]
        cmd_check = ["./main_check_multirobot", "--result_file", "../more_testing/classic_solution.yaml",  "--env_file",  "../example/classic.yaml" , "--models_base_path" , "dynoplan/dynobench/models/" ] 
        for i in [build_cmd, cmd_check ]:
            print("running cmd")
            print(' '.join(i))

            out = subprocess.run(i)
            assert out.returncode == 0

    def test_check2(self):
        build_cmd = [ "make" , "main_check_multirobot" ]
        cmd_check = ["./main_check_multirobot", "--result_file", "../more_testing/result_dbcbs.yaml",
                     "--env_file",  "../example/swap2_hetero.yaml" ,"--models_base_path" , "dynoplan/dynobench/models/" ] 
        cmd = build_cmd
        print("running cmd")
        print(' '.join(build_cmd))

        out = subprocess.run(cmd)
        assert out.returncode == 0

        cmd = cmd_check
        print("running cmd")
        print(' '.join(build_cmd))

        out = subprocess.run(cmd)
        assert out.returncode == 1




if __name__ == "__main__":
    unittest.main()



